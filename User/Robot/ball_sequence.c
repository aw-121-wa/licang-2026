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
uint32_t grabbed_ball_id[BALL_GRAB_MAX] = {0U};
uint8_t grabbed_ball_count = 0U;

static uint8_t BallSequence_IsNewId(uint32_t id)
{
    uint8_t index;
    for (index = 0U; index < grabbed_ball_count; index++)
    {
        if (grabbed_ball_id[index] == id) return 0U;
    }
    return 1U;
}

static BallSequenceStatus BallSequence_Finish(BallSequenceStatus status)
{
    BallSequence_LastStatus = status;
    if (status == BALL_SEQUENCE_OK) BallSequence_State = BALL_SEQUENCE_COMPLETE;
    else if (status == BALL_SEQUENCE_CANCELED_BY_STOP) BallSequence_State = BALL_SEQUENCE_CANCELED;
    else if ((status == BALL_SEQUENCE_ERROR_RFID_TIMEOUT) ||
             (status == BALL_SEQUENCE_ERROR_MAIX_TIMEOUT)) BallSequence_State = BALL_SEQUENCE_TIMEOUT;
    else BallSequence_State = BALL_SEQUENCE_ERROR;
    return status;
}

static ServoActionStatus BallSequence_RunGroup(uint8_t group)
{
    uint8_t is_grab = (group == SERVO_ACTION_GRAB_GROUP);
    ServoActionStatus status;
    BallSequence_State = is_grab ? BALL_SEQUENCE_GRAB_RUNNING : BALL_SEQUENCE_RETURN_RUNNING;
    ServoAction_SequenceState = is_grab ? SERVO_SEQUENCE_GRAB_RUNNING : SERVO_SEQUENCE_RETURN_RUNNING;
    status = ServoAction_RunGroup(group, 1U, is_grab ?
        SERVO_ACTION_GRAB_TIMEOUT_MS : SERVO_ACTION_RETURN_TIMEOUT_MS);
    ServoAction_SequenceState = (status == SERVO_ACTION_OK) ? SERVO_SEQUENCE_DONE : SERVO_SEQUENCE_ERROR;
    return status;
}

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

static BallSequenceStatus BallSequence_WaitForRfid(uint32_t *id)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t candidate_id;

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
            (BallSequence_IsNewId(candidate_id) != 0U))
        {
            *id = candidate_id;
            return BALL_SEQUENCE_OK;
        }
        osDelay(BALL_SEQUENCE_WAIT_PERIOD_MS);
    }

    return BALL_SEQUENCE_ERROR_RFID_TIMEOUT;
}

static void BallSequence_ClearGrabbedIds(void)
{
    uint8_t index;

    for (index = 0U; index < BALL_GRAB_MAX; index++)
    {
        grabbed_ball_id[index] = 0U;
    }
    grabbed_ball_count = 0U;
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
    uint32_t rfid_id = 0U;
    BallSequenceStatus status;
    WarehouseStatus warehouse_status;
    GrayAlignStatus gray_status;

    BallSequence_LastStatus = BALL_SEQUENCE_OK;
    BallSequence_Round = 0U;
    /* A retry continues the cache. Only explicit initialization starts a new batch. */
    if (grabbed_ball_count >= BALL_GRAB_MAX)
    {
        return BallSequence_Finish(BALL_SEQUENCE_OK);
    }
    round_count = WarehouseControl_RemainingBallCount();
    if (round_count > (BALL_GRAB_MAX - grabbed_ball_count))
    {
        round_count = (uint8_t)(BALL_GRAB_MAX - grabbed_ball_count);
    }
    if (round_count == 0U)
    {
        return BallSequence_Finish(BALL_SEQUENCE_ERROR_TURNTABLE);
    }
    if (MotionControl_StopRequested != 0U)
    {
        return BallSequence_Finish(BALL_SEQUENCE_CANCELED_BY_STOP);
    }

    BallSequence_State = BALL_SEQUENCE_ALIGNING;
    gray_status = GrayAlign_Run();
    if (gray_status != GRAY_ALIGN_OK)
    {
        return BallSequence_Finish((gray_status == GRAY_ALIGN_CANCELED) ?
            BALL_SEQUENCE_CANCELED_BY_STOP : BALL_SEQUENCE_ERROR_GRAY_ALIGN);
    }
    if (BallSequence_RunGroup(SERVO_ACTION_RETURN_GROUP) != SERVO_ACTION_OK)
    {
        return BallSequence_Finish(BALL_SEQUENCE_ERROR_SERVO);
    }

    for (round = 0U; round < round_count; round++)
    {
        if (MotionControl_StopRequested != 0U)
        {
            return BallSequence_Finish(BALL_SEQUENCE_CANCELED_BY_STOP);
        }
        BallSequence_Round = (uint8_t)(grabbed_ball_count + 1U);
        BallSequence_State = BALL_SEQUENCE_WAITING_MAIXCAM;
        if (MaixCamLink_SendRequest(BALL_SEQUENCE_TARGET_COLOR) != MAIXCAM_LINK_OK)
        {
            return BallSequence_Finish(BALL_SEQUENCE_ERROR_MAIX_UART);
        }
        status = BallSequence_WaitForMaixCam();
        if (status != BALL_SEQUENCE_OK) return BallSequence_Finish(status);

        /* Retain a one-shot report received DURING the clamp action. */
        RFID_Clear();
        if (BallSequence_RunGroup(SERVO_ACTION_GRAB_GROUP) != SERVO_ACTION_OK)
        {
            return BallSequence_Finish(BALL_SEQUENCE_ERROR_SERVO);
        }
        BallSequence_State = BALL_SEQUENCE_WAITING_RFID;
        status = BallSequence_WaitForRfid(&rfid_id);
        if (status == BALL_SEQUENCE_OK)
        {
            warehouse_status = WarehouseControl_HandleActionGroup2Completed();
            if (warehouse_status == WAREHOUSE_STATUS_OK)
            {
                /* Match the existing warehouse completion criterion; not a hardware acknowledgement. */
                grabbed_ball_id[grabbed_ball_count++] = rfid_id;
            }
            else
            {
                status = (warehouse_status == WAREHOUSE_STATUS_CANCELED) ?
                    BALL_SEQUENCE_CANCELED_BY_STOP : BALL_SEQUENCE_ERROR_TURNTABLE;
            }
        }
        /* Always return after a completed clamp, including RFID timeout/STOP. */
        if (BallSequence_RunGroup(SERVO_ACTION_RETURN_GROUP) != SERVO_ACTION_OK)
        {
            return BallSequence_Finish(BALL_SEQUENCE_ERROR_SERVO);
        }
        if (status != BALL_SEQUENCE_OK) return BallSequence_Finish(status);
        if (MotionControl_StopRequested != 0U)
        {
            return BallSequence_Finish(BALL_SEQUENCE_CANCELED_BY_STOP);
        }
    }
    return BallSequence_Finish(BALL_SEQUENCE_OK);
}

uint32_t *BALL_Get_Grabbed_ID(void)
{
    return grabbed_ball_id;
}

uint32_t *BALL_Get_ID_List(void)
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
