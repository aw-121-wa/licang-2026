#ifndef BALL_SEQUENCE_H
#define BALL_SEQUENCE_H

#include "main.h"
#include "robot_config.h"

typedef enum
{
    BALL_SEQUENCE_IDLE = 0,
    BALL_SEQUENCE_ALIGNING,
    BALL_SEQUENCE_WAITING_MAIXCAM,
    BALL_SEQUENCE_GRAB_RUNNING,
    BALL_SEQUENCE_WAITING_RFID,
    BALL_SEQUENCE_RETURN_RUNNING,
    BALL_SEQUENCE_COMPLETE,
    BALL_SEQUENCE_TIMEOUT,
    BALL_SEQUENCE_CANCELED,
    BALL_SEQUENCE_ERROR
} BallSequenceState;

typedef enum
{
    BALL_SEQUENCE_OK = 0,
    BALL_SEQUENCE_CANCELED_BY_STOP,
    BALL_SEQUENCE_ERROR_MAIX_UART,
    BALL_SEQUENCE_ERROR_MAIX_TIMEOUT,
    BALL_SEQUENCE_ERROR_SERVO,
    BALL_SEQUENCE_ERROR_TURNTABLE,
    BALL_SEQUENCE_ERROR_GRAY_ALIGN,
    BALL_SEQUENCE_ERROR_RFID_TIMEOUT
} BallSequenceStatus;

extern volatile BallSequenceState BallSequence_State;
extern volatile BallSequenceStatus BallSequence_LastStatus;
extern volatile uint8_t BallSequence_Round;
extern uint32_t grabbed_ball_id[BALL_GRAB_MAX];
extern uint8_t grabbed_ball_count;

void BallSequence_Init(void);
BallSequenceStatus BallSequence_Run(void);
uint32_t *BALL_Get_Grabbed_ID(void);
uint32_t *BALL_Get_ID_List(void);
const char *BallSequence_StateName(BallSequenceState state);
const char *BallSequence_StatusName(BallSequenceStatus status);

#endif /* BALL_SEQUENCE_H */
