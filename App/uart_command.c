#include "uart_command.h"
#include "jy61p.h"
#include "ball_sequence.h"
#include "maixcam_link.h"
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
volatile ChassisCommandMode ChassisCommand_Mode = CHASSIS_MODE_IDLE;
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

static uint8_t UartCommand_IsDiagonalPathType(CompetitionPathMotionType type)
{
    return ((type == PATH_MOVE_LEFT_FRONT) ||
            (type == PATH_MOVE_RIGHT_FRONT) ||
            (type == PATH_MOVE_LEFT_REAR) ||
            (type == PATH_MOVE_RIGHT_REAR)) ? 1U : 0U;
}

static uint8_t UartCommand_ParseType(const char *text,
                                     ChassisCommandType *command_type,
                                     CompetitionPathMotionType *path_type)
{
    if ((text == 0) || (command_type == 0) || (path_type == 0))
    {
        return 0U;
    }
    if (strcmp(text, "F") == 0)
    {
        *command_type = CHASSIS_CMD_FORWARD;
        *path_type = PATH_MOVE_FORWARD;
    }
    else if (strcmp(text, "B") == 0)
    {
        *command_type = CHASSIS_CMD_BACKWARD;
        *path_type = PATH_MOVE_BACKWARD;
    }
    else if (strcmp(text, "L") == 0)
    {
        *command_type = CHASSIS_CMD_LEFT;
        *path_type = PATH_MOVE_LEFT;
    }
    else if (strcmp(text, "R") == 0)
    {
        *command_type = CHASSIS_CMD_RIGHT;
        *path_type = PATH_MOVE_RIGHT;
    }
    else if (strcmp(text, "LF") == 0)
    {
        *command_type = CHASSIS_CMD_LEFT_FRONT;
        *path_type = PATH_MOVE_LEFT_FRONT;
    }
    else if (strcmp(text, "RF") == 0)
    {
        *command_type = CHASSIS_CMD_RIGHT_FRONT;
        *path_type = PATH_MOVE_RIGHT_FRONT;
    }
    else if (strcmp(text, "LR") == 0)
    {
        *command_type = CHASSIS_CMD_LEFT_REAR;
        *path_type = PATH_MOVE_LEFT_REAR;
    }
    else if (strcmp(text, "RR") == 0)
    {
        *command_type = CHASSIS_CMD_RIGHT_REAR;
        *path_type = PATH_MOVE_RIGHT_REAR;
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

static const char *UartCommand_PathTypeName(CompetitionPathMotionType type)
{
    switch (type)
    {
    case PATH_MOVE_FORWARD:      return "F";
    case PATH_MOVE_BACKWARD:     return "B";
    case PATH_MOVE_LEFT:         return "L";
    case PATH_MOVE_RIGHT:        return "R";
    case PATH_MOVE_LEFT_FRONT:   return "LF";
    case PATH_MOVE_RIGHT_FRONT:  return "RF";
    case PATH_MOVE_LEFT_REAR:    return "LR";
    case PATH_MOVE_RIGHT_REAR:   return "RR";
    case PATH_MOVE_ROTATE:       return "ROT";
    default:                     return "?";
    }
}

static void UartCommand_SendPathShow(void)
{
    char response[128];
    CompetitionPathSegment segment;
    uint8_t index;
    uint8_t count = CompetitionPath_GetUserCount();

    (void)snprintf(response, sizeof(response), "PATH COUNT=%u\r\n", count);
    UartCommand_Send(response);
    for (index = 0U; index < count; index++)
    {
        if (CompetitionPath_GetUserSegment(index, &segment) == 0U)
        {
            break;
        }
        if (segment.type == PATH_MOVE_ROTATE)
        {
            (void)snprintf(response, sizeof(response),
                           "[%u] ROT %s ANGLE=%.1f\r\n",
                           index,
                           (segment.angle_deg > 0.0f) ? "CCW" : "CW",
                           (double)((segment.angle_deg > 0.0f) ?
                                    segment.angle_deg : -segment.angle_deg));
        }
        else if ((segment.type == PATH_MOVE_LEFT_FRONT) ||
            (segment.type == PATH_MOVE_RIGHT_FRONT) ||
            (segment.type == PATH_MOVE_LEFT_REAR) ||
            (segment.type == PATH_MOVE_RIGHT_REAR))
        {
            (void)snprintf(response, sizeof(response),
                           "[%u] %s DIST=%lu ANGLE=%.1f\r\n",
                           index,
                           UartCommand_PathTypeName(segment.type),
                           (unsigned long)segment.distance_mm,
                           (double)segment.angle_deg);
        }
        else
        {
            (void)snprintf(response, sizeof(response),
                           "[%u] %s DIST=%lu\r\n",
                           index,
                           UartCommand_PathTypeName(segment.type),
                           (unsigned long)segment.distance_mm);
        }
        UartCommand_Send(response);
    }
    UartCommand_Send("END\r\n");
}

static void UartCommand_SendStatus(void)
{
    char response[256];
    const char *state;
    const char *mode;

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
    mode = (ChassisCommand_Mode == CHASSIS_MODE_PATH) ? "PATH" :
           ((ChassisCommand_Mode == CHASSIS_MODE_MANUAL) ? "MANUAL" : "IDLE");
    (void)snprintf(response, sizeof(response),
                   "STATE=%s\r\nMODE=%s\r\nIMU=%s\r\nYAW=%.2f\r\n"
                   "PATH_COUNT=%u\r\nPATH_STEP=%u\r\nSTOP_REQ=%u\r\n"
                   "ROT_TARGET=%.1f\r\nROT_CURRENT=%.1f\r\nROT_ERROR=%.1f\r\n",
                   state,
                   mode,
                   (Jy61P_IsOnline(500U) != 0U) ? "ONLINE" : "OFFLINE",
                   (double)Jy61P_GetContinuousYaw(),
                   CompetitionPath_GetUserCount(),
                   CompetitionPath_UserCurrentStep,
                   MotionControl_StopRequested,
                   (double)MotionControl_RotateTargetDeg,
                   (double)MotionControl_RotateCurrentDeg,
                   (double)MotionControl_RotateErrorDeg);
    UartCommand_Send(response);
    (void)snprintf(response, sizeof(response),
                   "ARM=%s\r\nARM_MOTION_COUNT=%u\r\nARM_LAST_GROUP=%u\r\n",
                   ServoAction_SequenceStateName(ServoAction_SequenceState),
                   ServoAction_MotionCompletedCount,
                   ServoAction_LastCompletedGroup);
    UartCommand_Send(response);
    (void)snprintf(response, sizeof(response),
                   "BALL_STATE=%s\r\nBALL_STATUS=%s\r\nBALL_ROUND=%u\r\n"
                   "MAIX_TX=%lu\r\nMAIX_RX=%lu\r\nMAIX_INVALID=%lu\r\n"
                   "MAIX_TIMEOUT=%lu\r\nMAIX_UART_ERR=%lu\r\n",
                   BallSequence_StateName(BallSequence_State),
                   BallSequence_StatusName(BallSequence_LastStatus),
                   BallSequence_Round,
                   (unsigned long)MaixCamLink_TxRequestCount,
                   (unsigned long)MaixCamLink_RxReplyCount,
                   (unsigned long)MaixCamLink_InvalidFrameCount,
                   (unsigned long)MaixCamLink_TimeoutCount,
                   (unsigned long)MaixCamLink_UartErrorCount);
    UartCommand_Send(response);
    (void)snprintf(response, sizeof(response),
                   "WAREHOUSE_STATE=%s\r\nWAREHOUSE_STATUS=%s\r\n"
                   "WAREHOUSE_BALL_COUNT=%u\r\nWAREHOUSE_G2_DONE=%lu\r\n"
                   "WAREHOUSE_TURN_COUNT=%lu\r\nWAREHOUSE_LAST_HAL=%d\r\n"
                   "WAREHOUSE_LAST_PULSES=%lu\r\nTURNTABLE=%s\r\n"
                   "TURNTABLE_EXPECTED_MS=%lu\r\n",
                   WarehouseControl_StateName((WarehouseState)Warehouse_State),
                   WarehouseControl_StatusName(Warehouse_LastStatus),
                   Warehouse_BallCount,
                   (unsigned long)Warehouse_ActionGroup2DoneCount,
                   (unsigned long)Warehouse_TurntableMoveCount,
                   (int)Warehouse_LastTurntableStatus,
                   (unsigned long)Warehouse_LastTurntablePulses,
                   Turntable_StateName(Turntable_State),
                   (unsigned long)Turntable_LastExpectedMoveMs);
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
        "STOP\r\nPATH CLEAR\r\nPATH ADD ...\r\nPATH SHOW\r\n"
        "PATH ADD ROT CCW <deg>\r\nPATH ADD ROT CW <deg>\r\n"
        "PATH RUN\r\nPATH LOAD DEFAULT\r\nSTATUS\r\nHELP\r\n"
        "ARM: G0=start, G1=return, G2=clamp; GRAB=G2->turn->G1, BALL=max 6\r\n");
}

static void UartCommand_ProcessLine(UartCommandLine *line)
{
    char *command;
    char *argument;
    char *distance_text;
    char *angle_text;
    ChassisCommand chassis_command;
    CompetitionPathMotionType path_type;
    CompetitionPathEditResult edit_result;

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
            (ChassisCommand_Mode == CHASSIS_MODE_IDLE))
        {
            /* Remove a command accepted but not yet started by ChassisTask. */
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
    if (strcmp(command, "BALL") == 0)
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        chassis_command.type = CHASSIS_CMD_BALL;
        chassis_command.distance_mm = 0U;
        chassis_command.angle_deg = 0.0f;
        if (UartCommand_SubmitMotion(&chassis_command) == 0U)
        {
            UartCommand_Send("ERR BUSY\r\n");
        }
        else
        {
            UartCommand_Send("OK BALL\r\n");
        }
        return;
    }
    if (strcmp(command, "RZ") == 0)
    {
        if (strtok(0, " \t") != 0)
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR FORMAT\r\n");
            return;
        }
        chassis_command.type = CHASSIS_CMD_RZ;
        chassis_command.distance_mm = 0U;
        chassis_command.angle_deg = 0.0f;
        if (UartCommand_SubmitMotion(&chassis_command) == 0U)
        {
            UartCommand_Send("ERR BUSY\r\n");
        }
        else
        {
            UartCommand_Send("OK RZ\r\n");
        }
        return;
    }
    if (strcmp(command, "PATH") == 0)
    {
        argument = strtok(0, " \t");
        if ((argument != 0) && (strcmp(argument, "SHOW") == 0))
        {
            if (strtok(0, " \t") != 0)
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR FORMAT\r\n");
                return;
            }
            UartCommand_SendPathShow();
            return;
        }
        if ((argument != 0) && (strcmp(argument, "CLEAR") == 0))
        {
            if (strtok(0, " \t") != 0)
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR FORMAT\r\n");
                return;
            }
            if (UartCommand_IsChassisAvailable() == 0U)
            {
                UartCommand_Send("ERR BUSY\r\n");
            }
            else
            {
                CompetitionPath_ClearUser();
                UartCommand_Send("OK PATH CLEAR\r\n");
            }
            return;
        }
        if ((argument != 0) && (strcmp(argument, "LOAD") == 0))
        {
            argument = strtok(0, " \t");
            if ((argument == 0) || (strcmp(argument, "DEFAULT") != 0) ||
                (strtok(0, " \t") != 0))
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR FORMAT\r\n");
            }
            else if (UartCommand_IsChassisAvailable() == 0U)
            {
                UartCommand_Send("ERR BUSY\r\n");
            }
            else
            {
                CompetitionPath_LoadDefault();
                UartCommand_Send("OK PATH LOAD DEFAULT\r\n");
            }
            return;
        }
        if ((argument != 0) && (strcmp(argument, "RUN") == 0))
        {
            if (strtok(0, " \t") != 0)
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR FORMAT\r\n");
                return;
            }
            if (CompetitionPath_GetUserCount() == 0U)
            {
                UartCommand_Send("ERR PATH_EMPTY\r\n");
            }
            else
            {
                chassis_command.type = CHASSIS_CMD_RUN_PATH;
                chassis_command.distance_mm = 0U;
                chassis_command.angle_deg = 0.0f;
                if (UartCommand_SubmitMotion(&chassis_command) == 0U)
                {
                    UartCommand_Send("ERR BUSY\r\n");
                }
                else
                {
                    UartCommand_Send("OK PATH RUN\r\n");
                }
            }
            return;
        }
        if ((argument != 0) && (strcmp(argument, "ADD") == 0))
        {
            argument = strtok(0, " \t");
            distance_text = strtok(0, " \t");
            angle_text = strtok(0, " \t");
            if ((argument == 0) || (distance_text == 0))
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR FORMAT\r\n");
                return;
            }
            if (strcmp(argument, "ROT") == 0)
            {
                if ((angle_text == 0) ||
                    (UartCommand_ParseRotate(distance_text, angle_text,
                                             &chassis_command.angle_deg) == 0U) ||
                    (strtok(0, " \t") != 0))
                {
                    UartCommand_ParseErrorCount++;
                    UartCommand_Send("ERR ANGLE\r\n");
                    return;
                }
                if (UartCommand_IsChassisAvailable() == 0U)
                {
                    UartCommand_Send("ERR BUSY\r\n");
                    return;
                }
                edit_result = CompetitionPath_AddUserSegment(
                    PATH_MOVE_ROTATE, 0U, chassis_command.angle_deg);
                if (edit_result == COMPETITION_PATH_EDIT_FULL)
                {
                    UartCommand_Send("ERR PATH_FULL\r\n");
                }
                else if (edit_result != COMPETITION_PATH_EDIT_OK)
                {
                    UartCommand_Send("ERR ANGLE\r\n");
                }
                else
                {
                    (void)snprintf(line->text, sizeof(line->text),
                                   "OK PATH ADD %u\r\n",
                                   (unsigned)(CompetitionPath_GetUserCount() - 1U));
                    UartCommand_Send(line->text);
                }
                return;
            }
            if (UartCommand_ParseType(argument, &chassis_command.type,
                                      &path_type) == 0U)
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR UNKNOWN_CMD\r\n");
                return;
            }
            if (UartCommand_ParseDistance(distance_text,
                                          &chassis_command.distance_mm) == 0U)
            {
                UartCommand_ParseErrorCount++;
                UartCommand_Send("ERR DISTANCE\r\n");
                return;
            }
            chassis_command.angle_deg = 0.0f;
            if (UartCommand_IsDiagonalPathType(path_type) != 0U)
            {
                if ((angle_text == 0) ||
                    (UartCommand_ParseAngle(angle_text,
                                            &chassis_command.angle_deg) == 0U))
                {
                    UartCommand_ParseErrorCount++;
                    UartCommand_Send("ERR ANGLE\r\n");
                    return;
                }
            }
            else if (angle_text != 0)
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
            if (UartCommand_IsChassisAvailable() == 0U)
            {
                UartCommand_Send("ERR BUSY\r\n");
                return;
            }
            edit_result = CompetitionPath_AddUserSegment(
                path_type,
                chassis_command.distance_mm,
                chassis_command.angle_deg);
            if (edit_result == COMPETITION_PATH_EDIT_FULL)
            {
                UartCommand_Send("ERR PATH_FULL\r\n");
            }
            else if (edit_result != COMPETITION_PATH_EDIT_OK)
            {
                UartCommand_Send("ERR FORMAT\r\n");
            }
            else
            {
                (void)snprintf(line->text, sizeof(line->text),
                               "OK PATH ADD %u\r\n",
                               (unsigned)(CompetitionPath_GetUserCount() - 1U));
                UartCommand_Send(line->text);
            }
            return;
        }
        UartCommand_ParseErrorCount++;
        UartCommand_Send("ERR FORMAT\r\n");
        return;
    }

    if (strcmp(command, "ROT") == 0)
    {
        argument = strtok(0, " \t");
        angle_text = strtok(0, " \t");
        if ((UartCommand_ParseRotate(argument, angle_text,
                                     &chassis_command.angle_deg) == 0U) ||
            (strtok(0, " \t") != 0))
        {
            UartCommand_ParseErrorCount++;
            UartCommand_Send("ERR ANGLE\r\n");
            return;
        }
        chassis_command.type = CHASSIS_CMD_ROTATE;
        chassis_command.distance_mm = 0U;
        if (UartCommand_SubmitMotion(&chassis_command) == 0U)
        {
            UartCommand_Send("ERR BUSY\r\n");
        }
        else
        {
            UartCommand_Send("OK\r\n");
        }
        return;
    }

    angle_text = strtok(0, " \t");
    if ((UartCommand_ParseType(command, &chassis_command.type, &path_type) == 0U) ||
        (angle_text == 0) ||
        (UartCommand_ParseDistance(angle_text, &chassis_command.distance_mm) == 0U))
    {
        UartCommand_ParseErrorCount++;
        UartCommand_Send("ERR FORMAT\r\n");
        return;
    }
    chassis_command.angle_deg = 0.0f;
    distance_text = strtok(0, " \t");
    if (UartCommand_IsDiagonalPathType(path_type) != 0U)
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
