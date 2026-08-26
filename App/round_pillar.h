#ifndef ROUND_PILLAR_H
#define ROUND_PILLAR_H

#include "main.h"

/* RZ uses one digital infrared sensor on PD10.  Change only this level after
 * the first hardware check if the installed sensor is active high. */
#define RZ_IR_DETECTED_LEVEL       GPIO_PIN_RESET

#define RZ_PERIOD_MS               20U

/* Approach */
#define RZ_APPROACH_RPM            25.0f
#define RZ_FINE_APPROACH_RPM       20.0f
#define RZ_AFTER_IR_DISTANCE_MM    170U
#define RZ_IR_STABLE_MS            30U
#define RZ_APPROACH_TIMEOUT_MS     5000U
#define RZ_STOP_SETTLE_MS          80U
#define RZ_CAMERA_RAISE_WAIT_MS    1000U
#define RZ_GRAB_COUNT              4U

/* Preserve the established two-stage pillar orbit before arm operation. */
#define RZ_ORBIT_FORWARD_RPM       63.0f
#define RZ_ORBIT_OMEGA_RPM         49.0f
#define RZ_CW_TARGET_DEG           (-360.0f)
#define RZ_CCW_REVERSE_DEG         90.0f
#define RZ_ORBIT_TIMEOUT_MS        15000U

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
