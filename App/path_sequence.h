#ifndef PATH_SEQUENCE_H
#define PATH_SEQUENCE_H

#include "main.h"
#include "ball_sequence.h"
#include "motion_control.h"
#include "round_pillar.h"
#include "stair_sequence.h"

typedef enum
{
    PATH_STEP_MOVE = 0,
    PATH_STEP_ROTATE,
    PATH_STEP_BALL,
    PATH_STEP_RZ,
    PATH_STEP_SERVO_GROUP,
    PATH_STEP_STAIR
} PathStepType;

typedef struct
{
    PathStepType type;
    uint32_t distance_mm;
    float angle_deg;
    float cruise_rpm;
    uint8_t servo_group;
} PathStep;

typedef enum
{
    PATH_SEQUENCE_IDLE = 0,

    PATH_SEQUENCE_LF20_1800,
    PATH_SEQUENCE_F2300,
    PATH_SEQUENCE_ROTATE1_178,
    PATH_SEQUENCE_BALL,
    PATH_SEQUENCE_ROTATE2_178,
    PATH_SEQUENCE_BACK1820,
    PATH_SEQUENCE_RZ,
    PATH_SEQUENCE_GROUP0,
    PATH_SEQUENCE_F330,
    PATH_SEQUENCE_BACK310,
    PATH_SEQUENCE_STAIR,
    PATH_SEQUENCE_LEFT_2000,

    PATH_SEQUENCE_DONE,
    PATH_SEQUENCE_CANCELED,
    PATH_SEQUENCE_ERROR
} PathSequenceState;

typedef enum
{
    PATH_SEQUENCE_OK = 0,
    /* Prefixed to avoid colliding with the state enum in C's global enum namespace. */
    PATH_SEQUENCE_STATUS_CANCELED,

    PATH_SEQUENCE_ERROR_MOTION,
    PATH_SEQUENCE_ERROR_ROTATE,
    PATH_SEQUENCE_ERROR_BALL,
    PATH_SEQUENCE_ERROR_RZ,
    PATH_SEQUENCE_ERROR_SERVO,
    PATH_SEQUENCE_ERROR_STAIR
} PathSequenceStatus;

extern volatile PathSequenceState PathSequence_State;
extern volatile uint8_t PathSequence_CurrentStep;
extern volatile PathSequenceStatus PathSequence_LastStatus;
extern volatile BallSequenceStatus PathSequence_LastBallStatus;
extern volatile RoundPillarStatus PathSequence_LastRzStatus;
extern volatile StairSequenceStatus PathSequence_LastStairStatus;
extern volatile MotionControlStatus PathSequence_LastMotionStatus;

PathSequenceStatus PathSequence_Run(void);
const char *PathSequence_StateName(PathSequenceState state);
const char *PathSequence_StatusName(PathSequenceStatus status);

#endif /* PATH_SEQUENCE_H */
