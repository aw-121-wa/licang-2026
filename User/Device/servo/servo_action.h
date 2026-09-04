#ifndef SERVO_ACTION_H
#define SERVO_ACTION_H

#include "main.h"
#include "robot_config.h"

/* Hiwonder/Lobot 舵控板串口参数。PE7=UART7_RX，PE8=UART7_TX。 */
typedef enum
{
    SERVO_ACTION_OK = 0,
    SERVO_ACTION_ERROR_ARGUMENT,
    SERVO_ACTION_ERROR_UART,
    SERVO_ACTION_ERROR_TIMEOUT
} ServoActionStatus;

typedef enum
{
    SERVO_SEQUENCE_STARTING = 0,
    SERVO_SEQUENCE_WAITING_MOTION,
    SERVO_SEQUENCE_GRAB_RUNNING,
    SERVO_SEQUENCE_RETURN_RUNNING,
    SERVO_SEQUENCE_DONE,
    SERVO_SEQUENCE_ERROR
} ServoActionSequenceState;

extern volatile ServoActionStatus ServoAction_LastStatus;
extern volatile ServoActionSequenceState ServoAction_SequenceState;

void ServoAction_Init(UART_HandleTypeDef *huart);
/* Send an action-group request without waiting for a completion frame. */
ServoActionStatus ServoAction_StartGroupNoWait(uint8_t group,
                                               uint16_t repeat_count);
ServoActionStatus ServoAction_RunGroup(uint8_t group,
                                       uint16_t repeat_count,
                                       uint32_t timeout_ms);
void ServoAction_UartRxCpltCallback(UART_HandleTypeDef *huart);
void ServoAction_UartErrorCallback(UART_HandleTypeDef *huart);
const char *ServoAction_SequenceStateName(ServoActionSequenceState state);

#endif /* SERVO_ACTION_H */
