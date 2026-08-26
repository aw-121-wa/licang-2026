#ifndef UART_COMMAND_H
#define UART_COMMAND_H

#include "main.h"
#include "motion_control.h"
#include "FreeRTOS.h"
#include "queue.h"

#define UART_CMD_BUFFER_SIZE 96U
#define UART_CMD_LINE_QUEUE_LENGTH 4U
#define CHASSIS_COMMAND_QUEUE_LENGTH 4U

typedef enum
{
    CHASSIS_CMD_NONE = 0,
    CHASSIS_CMD_FORWARD,
    CHASSIS_CMD_BACKWARD,
    CHASSIS_CMD_LEFT,
    CHASSIS_CMD_RIGHT,
    CHASSIS_CMD_LEFT_FRONT,
    CHASSIS_CMD_RIGHT_FRONT,
    CHASSIS_CMD_LEFT_REAR,
    CHASSIS_CMD_RIGHT_REAR,
    CHASSIS_CMD_ROTATE,
    CHASSIS_CMD_GRAB,
    CHASSIS_CMD_BALL,
    CHASSIS_CMD_RZ
} ChassisCommandType;

typedef struct
{
    ChassisCommandType type;
    uint32_t distance_mm;
    float angle_deg;
} ChassisCommand;

extern QueueHandle_t ChassisCommandQueue;
extern volatile uint32_t UartCommand_RxByteCount;
extern volatile uint32_t UartCommand_LineCount;
extern volatile uint32_t UartCommand_ParseErrorCount;
extern volatile uint8_t ChassisCommand_Busy;
extern volatile uint8_t ChassisTask_Ready;
extern volatile MotionControlStatus ChassisCommand_LastStatus;

void UartCommand_CreateQueues(void);
void UartCommand_Init(UART_HandleTypeDef *huart);
void UartCommand_Task(void *argument);
void UartCommand_UartRxCpltCallback(UART_HandleTypeDef *huart);
void UartCommand_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* UART_COMMAND_H */
