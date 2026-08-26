#include "ball_sequence.h"
#include "cmsis_os.h"
#include "gray_align.h"
#include "maixcam_link.h"
#include "motion_control.h"
#include "servo_action.h"
#include "warehouse_control.h"

#define BALL_SEQUENCE_WAIT_PERIOD_MS        10U
#define BALL_SEQUENCE_TARGET_COLOR          MAIXCAM_COLOR_RED

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
    uint8_t round_count;
    BallSequenceStatus status;
    ServoActionStatus servo_status;
    WarehouseStatus warehouse_status;
    GrayAlignStatus gray_status;
    uint8_t cancel_after_return;

    BallSequence_LastStatus = BALL_SEQUENCE_OK;
    BallSequence_Round = 0U;
    round_count = WarehouseControl_RemainingBallCount();
    if ((round_count == 0U) || (round_count > BALL_SEQUENCE_ROUND_COUNT))
    {
        BallSequence_State = BALL_SEQUENCE_ERROR;
        BallSequence_LastStatus = BALL_SEQUENCE_ERROR_TURNTABLE;
        return BallSequence_LastStatus;
    }

    /* Align the chassis to MID2-IN2-IN1-MID1 = 0-1-1-0 first. */
    BallSequence_State = BALL_SEQUENCE_ALIGNING;
    gray_status = GrayAlign_Run();
    if (gray_status == GRAY_ALIGN_CANCELED)
    {
        BallSequence_State = BALL_SEQUENCE_CANCELED;
        BallSequence_LastStatus = BALL_SEQUENCE_CANCELED_BY_STOP;
        return BallSequence_LastStatus;
    }
    if (gray_status != GRAY_ALIGN_OK)
    {
        BallSequence_State = BALL_SEQUENCE_ERROR;
        BallSequence_LastStatus = BALL_SEQUENCE_ERROR_GRAY_ALIGN;
        return BallSequence_LastStatus;
    }

    /* Group 1 is the return/recognition-ready posture and runs after alignment. */
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
    if (MotionControl_StopRequested != 0U)
    {
        BallSequence_State = BALL_SEQUENCE_CANCELED;
        BallSequence_LastStatus = BALL_SEQUENCE_CANCELED_BY_STOP;
        return BallSequence_LastStatus;
    }

    for (round = 1U; round <= round_count; round++)
    {
        BallSequence_Round = round;
        BallSequence_State = BALL_SEQUENCE_WAITING_MAIXCAM;
        if (MaixCamLink_SendRequest(BALL_SEQUENCE_TARGET_COLOR) != MAIXCAM_LINK_OK)
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

        /* Group 2 is the clamp action. Its real completion triggers exactly one turn. */
        warehouse_status = WarehouseControl_HandleActionGroup2Completed();
        cancel_after_return = (warehouse_status == WAREHOUSE_STATUS_CANCELED) ? 1U : 0U;

        /* A STOP during clamp/turn still runs group 1 return before aborting. */
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
        if ((warehouse_status != WAREHOUSE_STATUS_OK) &&
            (warehouse_status != WAREHOUSE_STATUS_CANCELED))
        {
            BallSequence_State = BALL_SEQUENCE_ERROR;
            BallSequence_LastStatus = BALL_SEQUENCE_ERROR_TURNTABLE;
            return BallSequence_LastStatus;
        }
        if (cancel_after_return != 0U)
        {
            BallSequence_State = BALL_SEQUENCE_CANCELED;
            BallSequence_LastStatus = BALL_SEQUENCE_CANCELED_BY_STOP;
            return BallSequence_LastStatus;
        }
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
    case BALL_SEQUENCE_ALIGNING:        return "ALIGNING";
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
    case BALL_SEQUENCE_ERROR_TURNTABLE:    return "TURNTABLE";
    case BALL_SEQUENCE_ERROR_GRAY_ALIGN:   return "GRAY_ALIGN";
    default:                               return "UNKNOWN";
    }
}
