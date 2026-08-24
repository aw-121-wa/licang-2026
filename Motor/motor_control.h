#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

#define MOTOR_WHEEL_DIAMETER_MM          75U
#define MOTOR_PULSES_PER_REV            3200U
#define MOTOR_DRIVE_GEAR_RATIO_NUM      1U
#define MOTOR_DRIVE_GEAR_RATIO_DEN      1U
#define MOTOR_MOVE_SPEED_RPM            100U
#define MOTOR_MOVE_ACCELERATION         0U
#define MOTOR_SEGMENT_SETTLE_MARGIN_MS  100U
#define MOTOR_SPEED_ACCELERATION         0U
#define MOTOR_SPEED_LIMIT_RPM          150U
#define MOTOR_SPEED_COMMAND_SCALE       10U

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

#endif
