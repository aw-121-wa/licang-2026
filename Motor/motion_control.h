#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "main.h"

typedef enum
{
    MOTION_STATUS_IDLE = 0,
    MOTION_STATUS_FORWARD = 1,
    MOTION_STATUS_PAUSE = 2,
    MOTION_STATUS_LEFT = 3,
    MOTION_STATUS_FINISHED = 4,
    MOTION_ERROR_IMU_STARTUP = 0xD0,
    MOTION_ERROR_IMU_LOST = 0xE1,
    MOTION_ERROR_MOTOR_UART = 0xE2,
    MOTION_ERROR_INVALID_ARGUMENT = 0xE3
} MotionControlStatus;

void MotionControl_Init(UART_HandleTypeDef *motor_uart,
                        UART_HandleTypeDef *imu_uart);
MotionControlStatus MotionControl_MoveMm(int32_t forward_mm, int32_t left_mm);
MotionControlStatus MotionControl_RunDefaultSequence(void);

extern volatile MotionControlStatus MotionControl_State;
extern volatile float MotionControl_HeadingErrorDeg;
extern volatile float MotionControl_HeadingCorrectionRpm;
extern volatile uint8_t MotionControl_ImuHeadingHoldActive;
extern volatile uint32_t MotionControl_PeriodOverrunCount;

#endif
