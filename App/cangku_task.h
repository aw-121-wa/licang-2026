#ifndef CANGKU_TASK_H
#define CANGKU_TASK_H

#include "main.h"

typedef enum
{
    CANGKU_STATE_IDLE = 0,
    CANGKU_STATE_ROTATE,
    CANGKU_STATE_FIND_LINE,
    CANGKU_STATE_BACKWARD_1,
    CANGKU_STATE_BACKWARD_2,
    CANGKU_STATE_BACKWARD_3,
    CANGKU_STATE_ACTION_15,
    CANGKU_STATE_FORWARD_1,
    CANGKU_STATE_FORWARD_2,
    CANGKU_STATE_DONE,
    CANGKU_STATE_CANCELED,
    CANGKU_STATE_ERROR
} CangkuSequenceState;

typedef enum
{
    CANGKU_STATUS_OK = 0,
    CANGKU_STATUS_CANCELED,
    CANGKU_STATUS_ERROR_IMU,
    CANGKU_STATUS_ERROR_ROTATE,
    CANGKU_STATUS_ERROR_GRAY_ALIGN,
    CANGKU_STATUS_ERROR_MOTION,
    CANGKU_STATUS_ERROR_SERVO,
    CANGKU_STATUS_ERROR_TURNTABLE
} CangkuSequenceStatus;

extern volatile CangkuSequenceState CangkuSequence_State;
extern volatile CangkuSequenceStatus CangkuSequence_LastStatus;

void CangkuSequence_Init(void);
CangkuSequenceStatus CangkuSequence_Run(void);
const char *CangkuSequence_StateName(CangkuSequenceState state);
const char *CangkuSequence_StatusName(CangkuSequenceStatus status);

#endif /* CANGKU_TASK_H */
