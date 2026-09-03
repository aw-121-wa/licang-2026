#ifndef STAIR_SEQUENCE_H
#define STAIR_SEQUENCE_H

#include "main.h"

#define STAIR_GROUP_5                     5U
#define STAIR_GROUP_6                     6U
#define STAIR_GROUP_7                     7U
#define STAIR_GROUP_8                     8U
#define STAIR_GROUP_9                     9U
#define STAIR_GROUP_10                   10U
#define STAIR_GROUP_11                   11U
#define STAIR_GROUP_12                   12U

#define STAIR_CAMERA_POSE_WAIT_MS       1000U
#define STAIR_SERVO_TIMEOUT_MS          20000U
#define STAIR_VISION_TIMEOUT_MS          3000U
#define STAIR_VISION_POLL_MS                1U
#define STAIR_FORWARD_RPM                40.0f

typedef enum
{
    STAIR_SEQUENCE_OK = 0,
    STAIR_SEQUENCE_CANCELED_BY_STOP,
    STAIR_SEQUENCE_ERROR_GRAY_ALIGN,
    STAIR_SEQUENCE_ERROR_IMU,
    STAIR_SEQUENCE_ERROR_MOTOR,
    STAIR_SEQUENCE_ERROR_SERVO,
    STAIR_SEQUENCE_ERROR_TURNTABLE,
    STAIR_SEQUENCE_ERROR_MAIX_UART
} StairSequenceStatus;

typedef enum
{
    STAIR_STATE_IDLE = 0,
    STAIR_STATE_ALIGNING,
    STAIR_STATE_PART1_G5,
    STAIR_STATE_PART1_CHECK1,
    STAIR_STATE_PART1_MOVE90,
    STAIR_STATE_PART1_G6,
    STAIR_STATE_PART1_TURN,
    STAIR_STATE_PART1_CHECK2,
    STAIR_STATE_PART1_G7,
    STAIR_STATE_PART1_MOVE117,
    STAIR_STATE_PART2_G8,
    STAIR_STATE_PART2_CHECK,
    STAIR_STATE_PART2_MOVE90,
    STAIR_STATE_PART2_G9,
    STAIR_STATE_PART2_TURN,
    STAIR_STATE_PART2_G10,
    STAIR_STATE_PART2_MOVE117,
    STAIR_STATE_PART3_G11,
    STAIR_STATE_PART3_CHECK1,
    STAIR_STATE_PART3_MOVE90,
    STAIR_STATE_PART3_CHECK2,
    STAIR_STATE_PART3_G12,
    STAIR_STATE_PART3_TURN,
    STAIR_STATE_DONE,
    STAIR_STATE_CANCELED,
    STAIR_STATE_ERROR
} StairSequenceState;

extern volatile StairSequenceState StairSequence_State;
extern volatile StairSequenceStatus StairSequence_LastStatus;

StairSequenceStatus StairSequence_Run(void);
const char *StairSequence_StateName(StairSequenceState state);
const char *StairSequence_StatusName(StairSequenceStatus status);

#endif /* STAIR_SEQUENCE_H */
