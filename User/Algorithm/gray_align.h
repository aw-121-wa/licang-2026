#ifndef GRAY_ALIGN_H
#define GRAY_ALIGN_H

#include "main.h"
#include "robot_config.h"

/* Gray sensors are read from left to right as MID2, IN2, IN1, MID1. */
typedef enum
{
    GRAY_ALIGN_OK = 0,
    GRAY_ALIGN_CANCELED,
    GRAY_ALIGN_ERROR_TIMEOUT,
    GRAY_ALIGN_ERROR_MOTOR_UART,
    GRAY_ALIGN_ERROR_IMU
} GrayAlignStatus;

GrayAlignStatus GrayAlign_Run(void);
GrayAlignStatus GrayAlign_RunUnlimited(void);

#endif /* GRAY_ALIGN_H */
