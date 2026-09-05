#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "robot_config.h"

typedef struct
{
    int32_t front_left;
    int32_t front_right;
    int32_t rear_left;
    int32_t rear_right;
} MotorWheelPulses;

typedef struct
{
    int16_t front_left;
    int16_t front_right;
    int16_t rear_left;
    int16_t rear_right;
} MotorWheelSpeedsRpmX10;

void MotorControl_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef MotorControl_EnableAll(void);
HAL_StatusTypeDef MotorControl_StopAll(void);
uint32_t MotorControl_DistanceMmToPulses(uint32_t distance_mm);
HAL_StatusTypeDef MotorControl_MoveWheels(const MotorWheelPulses *pulses);
HAL_StatusTypeDef MotorControl_SetWheelSpeeds(
    const MotorWheelSpeedsRpmX10 *speeds);

extern volatile uint32_t MotorControl_TxCount;
extern volatile HAL_StatusTypeDef MotorControl_LastUartStatus;
/* Host transmit-complete time of the last successful F6 synchronization. */
extern volatile uint32_t MotorControl_LastSpeedSyncTick;

#endif
