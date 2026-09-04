#include "ball_sequence.h"
#include "cmsis_os.h"
#include "gray_align.h"
#include "maixcam_link.h"
#include "motion_control.h"
#include "rfid.h"
#include "servo_action.h"
#include "warehouse_control.h"

#define BALL_SEQUENCE_TARGET_COLOR          MAIXCAM_COLOR_RED

volatile BallSequenceState BallSequence_State = BALL_SEQUENCE_IDLE;
volatile BallSequenceStatus BallSequence_LastStatus = BALL_SEQUENCE_OK;
volatile uint8_t BallSequence_Round = 0U;
uint8_t all_ball_id[BALL_ID_MAX] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U};
uint8_t grabbed_ball_id[BALL_GRAB_MAX] = {0U};
uint8_t grabbed_ball_count = 0U;

static uint8_t last_rfid_id = 0U;

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

static BallSequenceStatus BallSequence_WaitForRfid(uint8_t *id)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t candidate_id;

    if (id == NULL)
    {
        return BALL_SEQUENCE_ERROR_RFID_TIMEOUT;
    }

    while ((uint32_t)(HAL_GetTick() - start_tick) < BALL_SEQUENCE_RFID_TIMEOUT_MS)
    {
        if (MotionControl_StopRequested != 0U)
        {
            return BALL_SEQUENCE_CANCELED_BY_STOP;
        }
        candidate_id = 0U;
        if ((RFID_Read_ID(&candidate_id) != 0U) &&
            (candidate_id >= 1U) &&
            (candidate_id <= BALL_ID_MAX) &&
            (candidate_id != last_rfid_id))
        {
            *id = candidate_id;
            return BALL_SEQUENCE_OK;
        }
        osDelay(BALL_SEQUENCE_WAIT_PERIOD_MS);
    }

    return BALL_SEQUENCE_ERROR_RFID_TIMEOUT;
}

static uint8_t BallSequence_SaveRfidId(uint8_t id)
{
    if ((id < 1U) || (id > BALL_ID_MAX) ||
        (id == last_rfid_id) ||
        (grabbed_ball_count >= BALL_GRAB_MAX))
    {
        return 0U;
    }

    grabbed_ball_id[grabbed_ball_count] = id;
    grabbed_ball_count++;
    last_rfid_id = id;
    return 1U;
}

static void BallSequence_ClearGrabbedIds(void)
{
    uint8_t index;

    for (index = 0U; index < BALL_GRAB_MAX; index++)
    {
        grabbed_ball_id[index] = 0U;
    }
    grabbed_ball_count = 0U;
    last_rfid_id = 0U;
}

void BallSequence_Init(void)
{
    BallSequence_ClearGrabbedIds();
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
    uint8_t rfid_id;

    BallSequence_LastStatus = BALL_SEQUENCE_OK;
    BallSequence_Round = 0U;
    BallSequence_ClearGrabbedIds();
    RFID_Clear();
    round_count = WarehouseControl_RemainingBallCount();
    if (round_count > BALL_GRAB_MAX)
    {
        round_count = BALL_GRAB_MAX;
    }
    if (round_count == 0U)
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

        /* Read the ID while the just-grabbed ball is still in the arm. */
        RFID_Clear();
        BallSequence_State = BALL_SEQUENCE_WAITING_RFID;
        status = BallSequence_WaitForRfid(&rfid_id);
        warehouse_status = WAREHOUSE_STATUS_OK;
        cancel_after_return = (status == BALL_SEQUENCE_OK) ? 0U : 1U;
        if (status == BALL_SEQUENCE_OK)
        {
            if (BallSequence_SaveRfidId(rfid_id) == 0U)
            {
                status = BALL_SEQUENCE_ERROR_RFID_TIMEOUT;
                cancel_after_return = 1U;
            }
            else
            {
                /* Group 2 completion triggers exactly one turn only after ID capture. */
                warehouse_status = WarehouseControl_HandleActionGroup2Completed();
                cancel_after_return =
                    (warehouse_status == WAREHOUSE_STATUS_CANCELED) ? 1U : 0U;
            }
        }

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
        if (status != BALL_SEQUENCE_OK)
        {
            BallSequence_State = (status == BALL_SEQUENCE_CANCELED_BY_STOP) ?
                                 BALL_SEQUENCE_CANCELED : BALL_SEQUENCE_TIMEOUT;
            BallSequence_LastStatus = status;
            return status;
        }
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

uint8_t *BALL_Get_Grabbed_ID(void)
{
    return grabbed_ball_id;
}

uint8_t *BALL_Get_ID_List(void)
{
    return BALL_Get_Grabbed_ID();
}

const char *BallSequence_StateName(BallSequenceState state)
{
    switch (state)
    {
    case BALL_SEQUENCE_IDLE:            return "IDLE";
    case BALL_SEQUENCE_ALIGNING:        return "ALIGNING";
    case BALL_SEQUENCE_WAITING_MAIXCAM: return "WAIT_MAIX";
    case BALL_SEQUENCE_GRAB_RUNNING:    return "GRAB";
    case BALL_SEQUENCE_WAITING_RFID:    return "WAIT_RFID";
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
    case BALL_SEQUENCE_ERROR_RFID_TIMEOUT: return "RFID_TIMEOUT";
    default:                               return "UNKNOWN";
    }
}
