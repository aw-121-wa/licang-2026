#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "main.h"

#define MOTION_CRUISE_RPM          150.0f
#define MOTION_DIAGONAL_CRUISE_RPM 135.0f

typedef enum
{
    MOTION_STATUS_IDLE = 0,
    MOTION_STATUS_FORWARD = 1,
    MOTION_STATUS_PAUSE = 2,
    MOTION_STATUS_LEFT = 3,
    MOTION_STATUS_FINISHED = 4,
    MOTION_STATUS_POLAR_MOVE = 5,
    MOTION_STATUS_DIAGONAL = 6,
    MOTION_STATUS_ROTATING = 7,
    MOTION_ERROR_IMU_STARTUP = 0xD0,
    MOTION_ERROR_IMU_LOST = 0xE1,
    MOTION_ERROR_MOTOR_UART = 0xE2,
    MOTION_ERROR_INVALID_ARGUMENT = 0xE3,
    MOTION_ERROR_ROTATE_TIMEOUT = 0xE4,
    MOTION_ERROR_MAIX_UART = 0xE5,
    MOTION_ERROR_MAIX_TIMEOUT = 0xE6,
    MOTION_ERROR_GRAY_ALIGN = 0xE7,
    MOTION_ERROR_RZ_TIMEOUT = 0xE8
} MotionControlStatus;

/* Select the one-shot movement used by MotionControl_RunDefaultSequence(). */
#define DIAGONAL_TEST_ENABLE       1U
#define DIAGONAL_TEST_DISTANCE_MM  500U
#define DIAGONAL_TEST_ANGLE_DEG    45.0f
#define DIAGONAL_TEST_LEFT_FRONT   0U
#define DIAGONAL_TEST_RIGHT_FRONT  1U
#define DIAGONAL_TEST_LEFT_REAR    2U
#define DIAGONAL_TEST_RIGHT_REAR   3U
#define DIAGONAL_TEST_DIRECTION    DIAGONAL_TEST_LEFT_FRONT

void MotionControl_Init(UART_HandleTypeDef *motor_uart,
                        UART_HandleTypeDef *imu_uart);
void MotionControl_RequestStop(void);
void MotionControl_ClearStopRequest(void);
uint8_t MotionControl_WasStopped(void);
MotionControlStatus MotionControl_PrepareForMove(void);
MotionControlStatus MotionControl_MoveMm(int32_t forward_mm, int32_t left_mm);
MotionControlStatus MotionControl_MovePolarMm(uint32_t distance_mm,
                                               float angle_deg);
MotionControlStatus MotionControl_MovePolarSegmentMm(
    uint32_t distance_mm,
    float angle_deg,
    float start_rpm,
    float cruise_rpm,
    float end_rpm);
MotionControlStatus MotionControl_MovePolarBlendSegmentMm(
    uint32_t distance_mm,
    float start_angle_deg,
    float end_angle_deg,
    uint32_t blend_time_ms,
    float start_rpm,
    float cruise_rpm,
    float end_rpm);
MotionControlStatus MotionControl_MoveLeftFrontMm(uint32_t distance_mm,
                                                   float angle_deg);
MotionControlStatus MotionControl_MoveRightFrontMm(uint32_t distance_mm,
                                                    float angle_deg);
MotionControlStatus MotionControl_MoveLeftRearMm(uint32_t distance_mm,
                                                  float angle_deg);
MotionControlStatus MotionControl_MoveRightRearMm(uint32_t distance_mm,
                                                   float angle_deg);
/* Positive angle = counter-clockwise (left); negative = clockwise (right). */
MotionControlStatus MotionControl_RotateDeg(float angle_deg);
MotionControlStatus MotionControl_RunDefaultSequence(void);

extern volatile MotionControlStatus MotionControl_State;
extern volatile uint8_t MotionControl_StopRequested;
extern volatile uint8_t MotionControl_StoppedByRequest;
extern volatile float MotionControl_HeadingErrorDeg;
extern volatile float MotionControl_HeadingCorrectionRpm;
extern volatile uint8_t MotionControl_ImuHeadingHoldActive;
extern volatile uint32_t MotionControl_PeriodOverrunCount;
extern volatile float MotionControl_TargetAngleDeg;
extern volatile float MotionControl_ForwardUnit;
extern volatile float MotionControl_LeftUnit;
extern volatile float MotionControl_BaseRpm;
extern volatile float MotionControl_EffectiveBaseRpm;
extern volatile float MotionControl_LastFrontLeftRpm;
extern volatile float MotionControl_LastFrontRightRpm;
extern volatile float MotionControl_LastRearLeftRpm;
extern volatile float MotionControl_LastRearRightRpm;
extern volatile float MotionControl_WheelScale;
extern volatile float MotionControl_TraveledMm;
extern volatile float MotionControl_TargetDistanceMm;
extern volatile float MotionControl_RotateTargetDeg;
extern volatile float MotionControl_RotateCurrentDeg;
extern volatile float MotionControl_RotateErrorDeg;
extern volatile float MotionControl_RotateCommandRpm;
extern volatile uint8_t MotionControl_RotateSettleCount;
extern volatile uint32_t MotionControl_RotateElapsedMs;

#endif
