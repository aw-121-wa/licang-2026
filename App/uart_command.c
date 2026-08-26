#include "uart_command.h"
#include "jy61p.h"
#include "ball_sequence.h"
#include "motion_control.h"
#include "servo_action.h"
#include "warehouse_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
    uint8_t too_long;
    char text[UART_CMD_BUFFER_SIZE];
} UartCommandLine;

static UART_HandleTypeDef *command_uart = 0;
static uint8_t command_rx_byte = 0U;
static char command_rx_line[UART_CMD_BUFFER_SIZE];
static uint16_t command_rx_length = 0U;
static uint8_t command_rx_overflow = 0U;
static QueueHandle_t command_line_queue = 0;

QueueHandle_t ChassisCommandQueue = 0;
volatile uint32_t UartCommand_RxByteCount = 0U;
volatile uint32_t UartCommand_LineCount = 0U;
volatile uint32_t UartCommand_ParseErrorCount = 0U;
volatile uint8_t ChassisCommand_Busy = 1U;
volatile uint8_t ChassisTask_Ready = 0U;
volatile MotionControlStatus ChassisCommand_LastStatus = MOTION_STATUS_IDLE;

static void UartCommand_Send(const char *text)
{
    if ((command_uart != 0) && (text != 0))
    {
        (void)HAL_UART_Transmit(command_uart,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                100U);
    }
}

static uint8_t UartCommand_IsUnsignedDecimal(const char *text)
{
    uint8_t has_digit = 0U;

    if ((text == 0) || (*text == '\0'))
    {
        return 0U;
    }
    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9'))
        {
            return 0U;
        }
        has_digit = 1U;
        text++;
    }
    return has_digit;
}

static uint8_t UartCommand_ParseDistance(const char *text,
                                         uint32_t *distance_mm)
{
    unsigned long value;
    char *end;

    if ((distance_mm == 0) ||
        (UartCommand_IsUnsignedDecimal(text) == 0U))
    {
        return 0U;
    }
    end = 0;
    value = strtoul(text, &end, 10);
    if ((*end != '\0') || (value == 0UL) || (value > 10000UL))
    {
        return 0U;
    }
    *distance_mm = (uint32_t)value;
    return 1U;
}

static uint8_t UartCommand_ParseAngle(const char *text, float *angle_deg)
{
    float value;
    char *end;

    if ((text == 0) || (angle_deg == 0))
    {
        return 0U;
    }
    end = 0;
    value = strtof(text, &end);
    if ((end == text) || (*end != '\0') ||
        !(value > 0.0f) || !(value <= 90.0f))
    {
        return 0U;
    }
    *angle_deg = value;
    return 1U;
}

static uint8_t UartCommand_ParseRotate(const char *direction,
                                       const char *text,
                                       float *signed_angle_deg)
{
    float angle_deg;
    char *end;

    if ((direction == 0) || (text == 0) || (signed_angle_deg == 0))
    {
        return 0U;
    }
    angle_deg = strtof(text, &end);
    if ((end == text) || (*end != '\0') ||
        !(angle_deg > 0.0f) || !(angle_deg <= 360.0f))
    {
        return 0U;
    }
    if (strcmp(direction, "CCW") == 0)
    {
        *signed_angle_deg = angle_deg;
    }
    else if (strcmp(direction, "CW") == 0)
    {
        *signed_angle_deg = -angle_deg;
    }
    else
    {
        return 0U;
    }
    return 1U;
}

static uint8_t UartCommand_IsDiagonalType(ChassisCommandType type)
{
    return ((type == CHASSIS_CMD_LEFT_FRONT) ||
            (type == CHASSIS_CMD_RIGHT_FRONT) ||
            (type == CHASSIS_CMD_LEFT_REAR) ||
            (type == CHASSIS_CMD_RIGHT_REAR)) ? 1U : 0U;
}

static uint8_t UartCommand_ParseType(const char *text,
                                     ChassisCommandType *command_type)
{
    if ((text == 0) || (command_type == 0))
    {
        return 0U;
    }
    if (strcmp(text, "F") == 0)
    {
        *command_type = CHASSIS_CMD_FORWARD;
    }
    else if (strcmp(text, "B") == 0)
    {
        *command_type = CHASSIS_CMD_BACKWARD;
    }
    else if (strcmp(text, "L") == 0)
    {
        *command_type = CHASSIS_CMD_LEFT;
    }
    else if (strcmp(text, "R") == 0)
    {
        *command_type = CHASSIS_CMD_RIGHT;
    }
    else if (strcmp(text, "LF") == 0)
    {
        *command_type = CHASSIS_CMD_LEFT_FRONT;
    }
    else if (strcmp(text, "RF") == 0)
    {
        *command_type = CHASSIS_CMD_RIGHT_FRONT;
    }
    else if (strcmp(text, "LR") == 0)
    {
        *command_type = CHASSIS_CMD_LEFT_REAR;
    }
    else if (strcmp(text, "RR") == 0)
    {
        *command_type = CHASSIS_CMD_RIGHT_REAR;
    }
    else
    {
        return 0U;
    }
    return 1U;
}

static uint8_t UartCommand_IsChassisAvailable(void)
{
    if ((ChassisTask_Ready == 0U) ||
        (ChassisCommand_Busy != 0U) ||
        (ChassisCommandQueue == 0))
    {
        return 0U;
    }
    if (uxQueueMessagesWaiting(ChassisCommandQueue) != 0U)
    {
        return 0U;
    }
    return 1U;
}

static uint8_t UartCommand_SubmitMotion(const ChassisCommand *command)
{
    if ((command == 0) || (UartCommand_IsChassisAvailable() == 0U))
    {
        return 0U;
    }
    if (((command->type == CHASSIS_CMD_GRAB) ||
         (command->type == CHASSIS_CMD_BALL)) &&
        (WarehouseControl_IsReadyForAction() == 0U))
    {
        return 0U;
    }
    if ((ServoAction_SequenceState != SERVO_SEQUENCE_WAITING_MOTION) &&
        (ServoAction_SequenceState != SERVO_SEQUENCE_DONE))
    {
        return 0U;
    }
    MotionControl_ClearStopRequest();
    ChassisCommand_Busy = 1U;
    if (xQueueSend(ChassisCommandQueue, command, 0U) != pdPASS)
    {
        ChassisCommand_Busy = 0U;
        return 0U;
    }
    return 1U;
}

static void UartCommand_SendStatus(void)
{
    char response[320];
    const char *state;

    if (ChassisCommand_Busy != 0U)
    {
        state = (MotionControl_State == MOTION_STATUS_ROTATING) ?
                "ROTATING" : "RUNNING";
    }
    else if (ChassisCommand_LastStatus >= MOTION_ERROR_IMU_STARTUP)
    {
        state = "ERROR";
    }
    else
    {
        state = "IDLE";
    }
    (void)snprintf(response, sizeof(response),
                   "STATE=%s\r\nIMU=%s\r\nYAW=%.2f\r\n"
                   "HEAD_ERR=%.2f\r\nHEAD_CORR=%.2f\r\n"
                   "DIST=%.0f\r\nTARGET=%.0f\r\nLAST=%u\r\n"
                   "BALL_STATE=%s\r\nBALL_ROUND=%u\r\n"
                   "WAREHOUSE_BALL=%u\r\nSTOP=%u\r\n",
                   state,
                   (Jy61P_IsOnline(500U) != 0U) ? "ONLINE" : "OFFLINE",
                   (double)Jy61P_GetContinuousYaw(),
                   (double)MotionControl_HeadingErrorDeg,
                   (double)MotionControl_HeadingCorrectionRpm,
                   (double)MotionControl_TraveledMm,
                   (double)MotionControl_TargetDistanceMm,
                   (unsigned)ChassisCommand_LastStatus,
                   BallSequence_StateName(BallSequence_State),
                   BallSequence_Round,
                   Warehouse_BallCount,
                   MotionControl_StopRequested);
    UartCommand_Send(response);
}

static void UartCommand_SendHelp(void)
{
    UartCommand_Send(
        "F <mm>\r\nB <mm>\r\nL <mm>\r\nR <mm>\r\n"
        "LF <mm> <deg>\r\nRF <mm> <deg>\r\n"
        "LR <mm> <deg>\r\nRR <mm> <deg>\r\n"
        "ROT CCW <deg>\r\nROT CW <deg>\r\n"
        "GRAB\r\nBALL\r\nRZ\r\n"
        "STOP\r\nSTATUS\r\nHELP\r\n"
        "ARM: G0=start, G1=return, G2=clamp; GRAB=G2->turn->G1, BALL=max 6\r\n");
}

static void UartCommand_ProcessLine(UartCommandLine *line)
{
    char *command;
    char *argument;
    char *distance_text;
    char *angle_text;
    ChassisCommand chassis_command;

    if (line->too_long != 0U)
    {
        UartCommand_ParseErrorCount++;
        UartCommand_Send("ERR LINE_TOO_LONG\r\n");
        return;
    }

    command = strtok(line->text, " \t");
    if (command == 0)
    {
        return;
    }
    UartCommand_LineCount++;

    if (strcmp(command, "STOP") == 0)
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        MotionControl_RequestStop();
        if ((ChassisCommandQueue != 0) &&
            (uxQueueMessagesWaiting(ChassisCommandQueue) != 0U))
        {
            (void)xQueueReset(ChassisCommandQueue);
            ChassisCommand_Busy = 0U;
        }
        UartCommand_Send("OK STOP\r\n");
        return;
    }
    if (strcmp(command, "STATUS") == 0)
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        UartCommand_SendStatus();
        return;
    }
    if (strcmp(command, "HELP") == 0)
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        UartCommand_SendHelp();
        return;
    }
    if (strcmp(command, "GRAB") == 0)
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        if ((ServoAction_SequenceState != SERVO_SEQUENCE_WAITING_MOTION) &&
            (ServoAction_SequenceState != SERVO_SEQUENCE_DONE))
        {
            UartCommand_Send("ERR GRAB_NOT_READY\r\n");
            return;
        }
        chassis_command.type = CHASSIS_CMD_GRAB;
        chassis_command.distance_mm = 0U;
        chassis_command.angle_deg = 0.0f;
        if (UartCommand_SubmitMotion(&chassis_command) == 0U)
        {
            UartCommand_Send("ERR BUSY\r\n");
        }
        else
        {
            UartCommand_Send("OK GRAB\r\n");
        }
        return;
    }
    if ((strcmp(command, "BALL") == 0) ||
        (strcmp(command, "RZ") == 0))
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        chassis_command.type = (strcmp(command, "BALL") == 0) ?
                               CHASSIS_CMD_BALL : CHASSIS_CMD_RZ;
        chassis_command.distance_mm = 0U;
        chassis_command.angle_deg = 0.0f;
        if (UartCommand_SubmitMotion(&chassis_command) == 0U)
        {
            UartCommand_Send("ERR BUSY\r\n");
        }
        else
        {
            UartCommand_Send((chassis_command.type == CHASSIS_CMD_BALL) ?
                             "OK BALL\r\n" : "OK RZ\r\n");
        }
        return;
    }
    if (strcmp(command, "ROT") == 0)
    {
        chassis_command.type = CHASSIS_CMD_ROTATE;
        argument = strtok(0, " \t");
        angle_text = strtok(0, " \t");
        if ((UartCommand_ParseRotate(argument,
                                     angle_text,
                                     &chassis_command.angle_deg) == 0U) ||
            (strtok(0, " \t") != 0))
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR ANGLE\r\n");
            return;
        }
        chassis_command.distance_mm = 0U;
        if (UartCommand_SubmitMotion(&chassis_command) == 0U)
        {
            UartCommand_Send("ERR BUSY\r\n");
        }
        else
        {
            UartCommand_Send("OK ROT\r\n");
        }
        return;
    }

    angle_text = strtok(0, " \t");
    if ((UartCommand_ParseType(command, &chassis_command.type) == 0U) ||
        (angle_text == 0) ||
        (UartCommand_ParseDistance(angle_text,
                                    &chassis_command.distance_mm) == 0U))
    {
        UartCommand_ParseErrorCount++;
        UartCommand_Send("ERR FORMAT\r\n");
        return;
    }
    chassis_command.angle_deg = 0.0f;
    distance_text = strtok(0, " \t");
    if (UartCommand_IsDiagonalType(chassis_command.type) != 0U)
    {
        if ((distance_text == 0) ||
            (UartCommand_ParseAngle(distance_text,
                                    &chassis_command.angle_deg) == 0U))
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR ANGLE\r\n");
            return;
        }
    }
    else if (distance_text != 0)
    {
        UartCommand_ParseErrorCount++;
        UartCommand_Send("ERR FORMAT\r\n");
        return;
    }
    if (strtok(0, " \t") != 0)
    {
        UartCommand_ParseErrorCount++;
        UartCommand_Send("ERR FORMAT\r\n");
        return;
    }
    if (UartCommand_SubmitMotion(&chassis_command) == 0U)
    {
        UartCommand_Send("ERR BUSY\r\n");
    }
    else
    {
        UartCommand_Send("OK\r\n");
    }
}

void UartCommand_CreateQueues(void)
{
    command_line_queue = xQueueCreate(
        UART_CMD_LINE_QUEUE_LENGTH, sizeof(UartCommandLine));
    ChassisCommandQueue = xQueueCreate(
        CHASSIS_COMMAND_QUEUE_LENGTH, sizeof(ChassisCommand));
}

void UartCommand_Init(UART_HandleTypeDef *huart)
{
    command_uart = huart;
    command_rx_length = 0U;
    command_rx_overflow = 0U;
    command_rx_byte = 0U;
    if (command_uart != 0)
    {
        (void)HAL_UART_Receive_IT(command_uart, &command_rx_byte, 1U);
    }
}

void UartCommand_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((command_uart == 0) || (huart != command_uart))
    {
        return;
    }
    UartCommand_RxByteCount++;
    if ((command_rx_byte == '\r') || (command_rx_byte == '\n'))
    {
        if ((command_rx_length != 0U) || (command_rx_overflow != 0U))
        {
            UartCommandLine line;
            (void)memset(&line, 0, sizeof(line));
            line.too_long = command_rx_overflow;
            (void)memcpy(line.text, command_rx_line, command_rx_length);
            if ((command_line_queue == 0) ||
                (xQueueSendFromISR(command_line_queue,
                                   &line,
                                   &higher_priority_task_woken) != pdPASS))
            {
                UartCommand_ParseErrorCount++;
            }
        }
        command_rx_length = 0U;
        command_rx_overflow = 0U;
        command_rx_line[0] = '\0';
    }
    else if (command_rx_overflow == 0U)
    {
        if (command_rx_length < (UART_CMD_BUFFER_SIZE - 1U))
        {
            command_rx_line[command_rx_length++] = (char)command_rx_byte;
            command_rx_line[command_rx_length] = '\0';
        }
        else
        {
            command_rx_length = 0U;
            command_rx_line[0] = '\0';
            command_rx_overflow = 1U;
        }
    }
    (void)HAL_UART_Receive_IT(command_uart, &command_rx_byte, 1U);
    if (higher_priority_task_woken != pdFALSE)
    {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void UartCommand_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((command_uart != 0) && (huart == command_uart))
    {
        command_rx_length = 0U;
        command_rx_overflow = 0U;
        __HAL_UART_CLEAR_OREFLAG(command_uart);
        command_uart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(command_uart, &command_rx_byte, 1U);
    }
}

void UartCommand_Task(void *argument)
{
    UartCommandLine line;
    (void)argument;

    for (;;)
    {
        if ((command_line_queue != 0) &&
            (xQueueReceive(command_line_queue, &line, portMAX_DELAY) == pdPASS))
        {
            UartCommand_ProcessLine(&line);
        }
    }
}
