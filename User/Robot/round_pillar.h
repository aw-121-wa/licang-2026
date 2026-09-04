#ifndef ROUND_PILLAR_H
#define ROUND_PILLAR_H

#include "main.h"
#include "robot_config.h"

/* RZ uses one digital infrared sensor on PD10.  Change only this level after
 * the first hardware check if the installed sensor is active high. */
#define RZ_IR_DETECTED_LEVEL       GPIO_PIN_RESET

typedef enum
{
    ROUND_PILLAR_OK = 0,
    ROUND_PILLAR_CANCELED,
    ROUND_PILLAR_ERROR_IMU,
    ROUND_PILLAR_ERROR_MOTOR,
    ROUND_PILLAR_ERROR_APPROACH_TIMEOUT,
    ROUND_PILLAR_ERROR_SERVO,
    ROUND_PILLAR_ERROR_TURNTABLE,
    ROUND_PILLAR_ERROR_MAIX_UART,
    ROUND_PILLAR_ERROR_MAIX_TIMEOUT,
    ROUND_PILLAR_ERROR_ORBIT_TIMEOUT
} RoundPillarStatus;

RoundPillarStatus RoundPillar_Run(void);

#endif /* ROUND_PILLAR_H */
