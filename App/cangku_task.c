#include "cangku_task.h"
#include "gray_align.h"
#include "motion_control.h"
#include "servo_action.h"
#include "turntable_control.h"

#define CANGKU_MOVE_DISTANCE_MM       200U
#define CANGKU_SERVO_TIMEOUT_MS     20000U

#define CANGKU_GROUP_13               13U
#define CANGKU_GROUP_14               14U
#define CANGKU_GROUP_15               15U

volatile CangkuSequenceState CangkuSequence_State = CANGKU_STATE_IDLE;
volatile CangkuSequenceStatus CangkuSequence_LastStatus = CANGKU_STATUS_OK;

static uint8_t CangkuSequence_StopRequested(void)
{
    return (MotionControl_StopRequested != 0U) ? 1U : 0U;
}

static CangkuSequenceStatus CangkuSequence_FromMotionStatus(
    MotionControlStatus status)
{
    if (status == MOTION_ERROR_IMU_LOST)
    {
        return CANGKU_STATUS_ERROR_IMU;
    }
    if (status == MOTION_ERROR_ROTATE_TIMEOUT)
    {
        return CANGKU_STATUS_ERROR_ROTATE;
    }
    if (status != MOTION_STATUS_FINISHED)
    {
        return CANGKU_STATUS_ERROR_MOTION;
    }
    return CANGKU_STATUS_OK;
}

static CangkuSequenceStatus CangkuSequence_RunMove(float angle_deg)
{
    MotionControlStatus motion_status;

    if (CangkuSequence_StopRequested() != 0U)
    {
        return CANGKU_STATUS_CANCELED;
    }
    motion_status = MotionControl_MovePolarSegmentMm(
        CANGKU_MOVE_DISTANCE_MM, angle_deg, 0.0f, MOTION_CRUISE_RPM, 0.0f);
    if (MotionControl_WasStopped() != 0U)
    {
        return CANGKU_STATUS_CANCELED;
    }
    return CangkuSequence_FromMotionStatus(motion_status);
}

static CangkuSequenceStatus CangkuSequence_RunServoGroup(uint8_t group)
{
    if (CangkuSequence_StopRequested() != 0U)
    {
        return CANGKU_STATUS_CANCELED;
    }
    ServoAction_SequenceState = SERVO_SEQUENCE_GRAB_RUNNING;
    if (ServoAction_RunGroup(group, 1U, CANGKU_SERVO_TIMEOUT_MS) !=
        SERVO_ACTION_OK)
    {
        ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
        return CANGKU_STATUS_ERROR_SERVO;
    }
    if (CangkuSequence_StopRequested() != 0U)
    {
        return CANGKU_STATUS_CANCELED;
    }
    return CANGKU_STATUS_OK;
}

static CangkuSequenceStatus CangkuSequence_RunReverseTurntable(void)
{
    TurntableStatus turntable_status;

    if (CangkuSequence_StopRequested() != 0U)
    {
        return CANGKU_STATUS_CANCELED;
    }
    turntable_status = Turntable_MoveOneSlotReverseAndWait(
        CangkuSequence_StopRequested);
    if (turntable_status == TURNTABLE_STATUS_CANCELED)
    {
        return CANGKU_STATUS_CANCELED;
    }
    if (turntable_status != TURNTABLE_STATUS_OK)
    {
        return CANGKU_STATUS_ERROR_TURNTABLE;
    }
    return CANGKU_STATUS_OK;
}

static CangkuSequenceStatus Cangku_Action13(void)
{
    return CangkuSequence_RunServoGroup(CANGKU_GROUP_13);
}

static CangkuSequenceStatus Cangku_Action14(void)
{
    return CangkuSequence_RunServoGroup(CANGKU_GROUP_14);
}

static CangkuSequenceStatus Cangku_Action15(void)
{
    return CangkuSequence_RunServoGroup(CANGKU_GROUP_15);
}

void CangkuSequence_Init(void)
{
    CangkuSequence_State = CANGKU_STATE_IDLE;
    CangkuSequence_LastStatus = CANGKU_STATUS_OK;
}

CangkuSequenceStatus CangkuSequence_Run(void)
{
    CangkuSequenceStatus status;
    GrayAlignStatus gray_status;
    MotionControlStatus motion_status;

    CangkuSequence_LastStatus = CANGKU_STATUS_OK;

    CangkuSequence_State = CANGKU_STATE_ROTATE;
    motion_status = MotionControl_RotateDeg(180.0f);
    if (MotionControl_WasStopped() != 0U)
    {
        status = CANGKU_STATUS_CANCELED;
        goto canceled;
    }
    status = CangkuSequence_FromMotionStatus(motion_status);
    if (status != CANGKU_STATUS_OK)
    {
        goto failed;
    }

    CangkuSequence_State = CANGKU_STATE_FIND_LINE;
    if (CangkuSequence_StopRequested() != 0U)
    {
        status = CANGKU_STATUS_CANCELED;
        goto canceled;
    }
    gray_status = GrayAlign_Run();
    if (gray_status == GRAY_ALIGN_CANCELED)
    {
        status = CANGKU_STATUS_CANCELED;
        goto canceled;
    }
    if (gray_status == GRAY_ALIGN_ERROR_IMU)
    {
        status = CANGKU_STATUS_ERROR_IMU;
        goto failed;
    }
    if (gray_status != GRAY_ALIGN_OK)
    {
        status = CANGKU_STATUS_ERROR_GRAY_ALIGN;
        goto failed;
    }

    CangkuSequence_State = CANGKU_STATE_BACKWARD_1;
    status = CangkuSequence_RunMove(180.0f);
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action13();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action14();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = CangkuSequence_RunReverseTurntable();
    }
    if (status != CANGKU_STATUS_OK)
    {
        goto cangku_status;
    }

    CangkuSequence_State = CANGKU_STATE_BACKWARD_2;
    status = CangkuSequence_RunMove(180.0f);
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action14();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action13();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = CangkuSequence_RunReverseTurntable();
    }
    if (status != CANGKU_STATUS_OK)
    {
        goto cangku_status;
    }

    CangkuSequence_State = CANGKU_STATE_BACKWARD_3;
    status = CangkuSequence_RunMove(180.0f);
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action14();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action13();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = CangkuSequence_RunReverseTurntable();
    }
    if (status != CANGKU_STATUS_OK)
    {
        goto cangku_status;
    }

    CangkuSequence_State = CANGKU_STATE_ACTION_15;
    status = Cangku_Action15();
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action13();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = CangkuSequence_RunReverseTurntable();
    }
    if (status != CANGKU_STATUS_OK)
    {
        goto cangku_status;
    }

    CangkuSequence_State = CANGKU_STATE_FORWARD_1;
    status = CangkuSequence_RunMove(0.0f);
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action15();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action13();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = CangkuSequence_RunReverseTurntable();
    }
    if (status != CANGKU_STATUS_OK)
    {
        goto cangku_status;
    }

    CangkuSequence_State = CANGKU_STATE_FORWARD_2;
    status = CangkuSequence_RunMove(0.0f);
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action15();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = Cangku_Action13();
    }
    if (status == CANGKU_STATUS_OK)
    {
        status = CangkuSequence_RunReverseTurntable();
    }

cangku_status:
    if (status == CANGKU_STATUS_CANCELED)
    {
        goto canceled;
    }
    if (status != CANGKU_STATUS_OK)
    {
        goto failed;
    }
    CangkuSequence_State = CANGKU_STATE_DONE;
    CangkuSequence_LastStatus = CANGKU_STATUS_OK;
    ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
    return CangkuSequence_LastStatus;

canceled:
    (void)MotionControl_SetBodySpeed(0.0f, 0.0f, 0.0f);
    (void)Turntable_Stop();
    ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
    CangkuSequence_State = CANGKU_STATE_CANCELED;
    CangkuSequence_LastStatus = CANGKU_STATUS_CANCELED;
    return CangkuSequence_LastStatus;

failed:
    (void)MotionControl_SetBodySpeed(0.0f, 0.0f, 0.0f);
    CangkuSequence_State = CANGKU_STATE_ERROR;
    CangkuSequence_LastStatus = status;
    return CangkuSequence_LastStatus;
}

const char *CangkuSequence_StateName(CangkuSequenceState state)
{
    switch (state)
    {
    case CANGKU_STATE_IDLE:       return "IDLE";
    case CANGKU_STATE_ROTATE:     return "ROTATE";
    case CANGKU_STATE_FIND_LINE:  return "FIND_LINE";
    case CANGKU_STATE_BACKWARD_1: return "BACKWARD_1";
    case CANGKU_STATE_BACKWARD_2: return "BACKWARD_2";
    case CANGKU_STATE_BACKWARD_3: return "BACKWARD_3";
    case CANGKU_STATE_ACTION_15:  return "ACTION_15";
    case CANGKU_STATE_FORWARD_1:  return "FORWARD_1";
    case CANGKU_STATE_FORWARD_2:  return "FORWARD_2";
    case CANGKU_STATE_DONE:       return "DONE";
    case CANGKU_STATE_CANCELED:   return "CANCELED";
    case CANGKU_STATE_ERROR:      return "ERROR";
    default:                      return "UNKNOWN";
    }
}

const char *CangkuSequence_StatusName(CangkuSequenceStatus status)
{
    switch (status)
    {
    case CANGKU_STATUS_OK:               return "OK";
    case CANGKU_STATUS_CANCELED:         return "CANCELED";
    case CANGKU_STATUS_ERROR_IMU:        return "IMU";
    case CANGKU_STATUS_ERROR_ROTATE:     return "ROTATE";
    case CANGKU_STATUS_ERROR_GRAY_ALIGN: return "GRAY_ALIGN";
    case CANGKU_STATUS_ERROR_MOTION:     return "MOTION";
    case CANGKU_STATUS_ERROR_SERVO:      return "SERVO";
    case CANGKU_STATUS_ERROR_TURNTABLE:  return "TURNTABLE";
    default:                             return "UNKNOWN";
    }
}
