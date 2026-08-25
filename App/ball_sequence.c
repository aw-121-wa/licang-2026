#include "ball_sequence.h"
#include "cmsis_os.h"
#include "maixcam_link.h"
#include "motion_control.h"
#include "servo_action.h"

#define BALL_SEQUENCE_WAIT_PERIOD_MS        10U

volatile BallSequenceState BallSequence_State = BALL_SEQUENCE_IDLE;
volatile BallSequenceStatus BallSequence_LastStatus = BALL_SEQUENCE_OK;
volatile uint8_t BallSequence_Round = 0U;

static BallSequenceStatus BallSequence_WaitForMaixCam(void)
{
    uint32_t start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) < MAIXCAM_REQUEST_TIMEOUT_MS)
    {
        if (MotionControl_StopRequested != 0U)
        {
            return BALL_SEQUENCE_CANCELED_BY_STOP;
        }
        if (MaixCamLink_TakeReply() != 0U)
        {
            return BALL_SEQUENCE_OK;
        }
        osDelay(BALL_SEQUENCE_WAIT_PERIOD_MS);
    }

    MaixCamLink_RecordTimeout();
    return BALL_SEQUENCE_ERROR_MAIX_TIMEOUT;
}

void BallSequence_Init(void)
{
    BallSequence_State = BALL_SEQUENCE_IDLE;
    BallSequence_LastStatus = BALL_SEQUENCE_OK;
    BallSequence_Round = 0U;
}

BallSequenceStatus BallSequence_Run(void)
{
    uint8_t round;
    BallSequenceStatus status;
    ServoActionStatus servo_status;

    BallSequence_LastStatus = BALL_SEQUENCE_OK;
    BallSequence_Round = 0U;

    for (round = 1U; round <= BALL_SEQUENCE_ROUND_COUNT; round++)
    {
        BallSequence_Round = round;
        BallSequence_State = BALL_SEQUENCE_WAITING_MAIXCAM;
        if (MaixCamLink_SendRequest() != MAIXCAM_LINK_OK)
        {
            BallSequence_State = BALL_SEQUENCE_ERROR;
            BallSequence_LastStatus = BALL_SEQUENCE_ERROR_MAIX_UART;
            return BallSequence_LastStatus;
        }

        status = BallSequence_WaitForMaixCam();
        if (status == BALL_SEQUENCE_CANCELED_BY_STOP)
        {
            BallSequence_State = BALL_SEQUENCE_CANCELED;
            BallSequence_LastStatus = status;
            ServoAction_SequenceState = SERVO_SEQUENCE_WAITING_MOTION;
            return status;
        }
        if (status != BALL_SEQUENCE_OK)
        {
            BallSequence_State = BALL_SEQUENCE_TIMEOUT;
            BallSequence_LastStatus = status;
            ServoAction_SequenceState = SERVO_SEQUENCE_WAITING_MOTION;
            return status;
        }

        BallSequence_State = BALL_SEQUENCE_GRAB_RUNNING;
        ServoAction_SequenceState = SERVO_SEQUENCE_GRAB_RUNNING;
        servo_status = ServoAction_RunGroup(SERVO_ACTION_GRAB_GROUP,
                                            1U,
                                            SERVO_ACTION_GRAB_TIMEOUT_MS);
        if (servo_status != SERVO_ACTION_OK)
        {
            BallSequence_State = BALL_SEQUENCE_ERROR;
            BallSequence_LastStatus = BALL_SEQUENCE_ERROR_SERVO;
            ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
            return BallSequence_LastStatus;
        }

        /* A STOP during gripping still runs the return group before aborting. */
        BallSequence_State = BALL_SEQUENCE_RETURN_RUNNING;
        ServoAction_SequenceState = SERVO_SEQUENCE_RETURN_RUNNING;
        servo_status = ServoAction_RunGroup(SERVO_ACTION_RETURN_GROUP,
                                            1U,
                                            SERVO_ACTION_RETURN_TIMEOUT_MS);
        if (servo_status != SERVO_ACTION_OK)
        {
            BallSequence_State = BALL_SEQUENCE_ERROR;
            BallSequence_LastStatus = BALL_SEQUENCE_ERROR_SERVO;
            ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
            return BallSequence_LastStatus;
        }

        ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
        if (MotionControl_StopRequested != 0U)
        {
            BallSequence_State = BALL_SEQUENCE_CANCELED;
            BallSequence_LastStatus = BALL_SEQUENCE_CANCELED_BY_STOP;
            return BallSequence_LastStatus;
        }
    }

    BallSequence_State = BALL_SEQUENCE_COMPLETE;
    BallSequence_LastStatus = BALL_SEQUENCE_OK;
    ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
    return BallSequence_LastStatus;
}

const char *BallSequence_StateName(BallSequenceState state)
{
    switch (state)
    {
    case BALL_SEQUENCE_IDLE:            return "IDLE";
    case BALL_SEQUENCE_WAITING_MAIXCAM: return "WAIT_MAIX";
    case BALL_SEQUENCE_GRAB_RUNNING:    return "GRAB";
    case BALL_SEQUENCE_RETURN_RUNNING:  return "RETURN";
    case BALL_SEQUENCE_COMPLETE:        return "COMPLETE";
    case BALL_SEQUENCE_TIMEOUT:         return "TIMEOUT";
    case BALL_SEQUENCE_CANCELED:        return "CANCELED";
    case BALL_SEQUENCE_ERROR:           return "ERROR";
    default:                            return "UNKNOWN";
    }
}

const char *BallSequence_StatusName(BallSequenceStatus status)
{
    switch (status)
    {
    case BALL_SEQUENCE_OK:                 return "OK";
    case BALL_SEQUENCE_CANCELED_BY_STOP:   return "CANCELED";
    case BALL_SEQUENCE_ERROR_MAIX_UART:    return "MAIX_UART";
    case BALL_SEQUENCE_ERROR_MAIX_TIMEOUT: return "MAIX_TIMEOUT";
    case BALL_SEQUENCE_ERROR_SERVO:        return "SERVO";
    default:                               return "UNKNOWN";
    }
}
