#ifndef GRAY_ALIGN_H
#define GRAY_ALIGN_H

#include "main.h"

/* Gray sensors are read from left to right as MID2, IN2, IN1, MID1. */
#define GRAY_ALIGN_STABLE_MS            50U
#define GRAY_ALIGN_TIMEOUT_MS           5000U
#define GRAY_ALIGN_PERIOD_MS            20U
#define GRAY_ALIGN_APPROACH_RPM         25.0f
#define GRAY_ALIGN_RETREAT_RPM          25.0f

typedef enum
{
    GRAY_ALIGN_OK = 0,
    GRAY_ALIGN_CANCELED,
    GRAY_ALIGN_ERROR_TIMEOUT,
    GRAY_ALIGN_ERROR_MOTOR_UART,
    GRAY_ALIGN_ERROR_IMU
} GrayAlignStatus;

GrayAlignStatus GrayAlign_Run(void);

#endif /* GRAY_ALIGN_H */
