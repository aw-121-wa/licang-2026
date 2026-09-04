#include "stair_sequence.h"
#include "cmsis_os.h"
#include "gray_align.h"
#include "maixcam_link.h"
#include "motor_control.h"
#include "motion_control.h"
#include "servo_action.h"
#include "turntable_control.h"

typedef enum
{
    STAIR_BALL_NOT_FOUND = 0,
    STAIR_BALL_FOUND,
    STAIR_BALL_CANCELED,
    STAIR_BALL_MOTOR_ERROR,
    STAIR_BALL_MAIX_UART_ERROR,
    STAIR_BALL_IMU_ERROR,
    STAIR_BALL_SERVO_ERROR,
    STAIR_BALL_TURNTABLE_ERROR
} StairBallResult;

typedef enum
{
    STAIR_PART2_MOVE_DISTANCE_DONE = 0,
    STAIR_PART2_MOVE_VISION_STOP,
    STAIR_PART2_MOVE_CANCELED,
    STAIR_PART2_MOVE_MOTOR_ERROR,
    STAIR_PART2_MOVE_MAIX_UART_ERROR,
    STAIR_PART2_MOVE_IMU_ERROR
} StairPart2MoveResult;

volatile StairSequenceState StairSequence_State = STAIR_STATE_IDLE;
volatile StairSequenceStatus StairSequence_LastStatus = STAIR_SEQUENCE_OK;

static volatile uint8_t stair_move_visual_hit = 0U;

static StairSequenceStatus StairSequence_Finalize(StairSequenceStatus status)
{
    StairSequence_LastStatus = status;
    if (status == STAIR_SEQUENCE_OK)
    {
        StairSequence_State = STAIR_STATE_DONE;
    }
    else if (status == STAIR_SEQUENCE_CANCELED_BY_STOP)
    {
        StairSequence_State = STAIR_STATE_CANCELED;
    }
    else
    {
        StairSequence_State = STAIR_STATE_ERROR;
    }
    return status;
}

static StairSequenceStatus StairSequence_FromMotionStatus(
    MotionControlStatus status)
{
    if (status == MOTION_ERROR_IMU_STARTUP ||
        status == MOTION_ERROR_IMU_LOST)
    {
        return STAIR_SEQUENCE_ERROR_IMU;
    }
    if (status == MOTION_ERROR_MOTOR_UART)
    {
        return STAIR_SEQUENCE_ERROR_MOTOR;
    }
    if (status >= MOTION_ERROR_IMU_STARTUP)
    {
        return STAIR_SEQUENCE_ERROR_MOTOR;
    }
    return STAIR_SEQUENCE_OK;
}

static StairSequenceStatus StairSequence_StopChassis(void)
{
    HAL_StatusTypeDef status = MotionControl_SetBodySpeed(
        0.0f, 0.0f, 0.0f);

    if (status != HAL_OK)
    {
        (void)MotorControl_StopAll();
        return STAIR_SEQUENCE_ERROR_MOTOR;
    }
    return STAIR_SEQUENCE_OK;
}

static StairSequenceStatus StairSequence_CheckStop(void)
{
    if (MotionControl_StopRequested == 0U)
    {
        return STAIR_SEQUENCE_OK;
    }
    return (StairSequence_StopChassis() == STAIR_SEQUENCE_OK) ?
           STAIR_SEQUENCE_CANCELED_BY_STOP : STAIR_SEQUENCE_ERROR_MOTOR;
}

static uint8_t StairSequence_TurntableCancelCheck(void)
{
    return (MotionControl_StopRequested != 0U) ? 1U : 0U;
}

static StairBallResult StairSequence_CheckRedBallStopped(void)
{
    uint32_t start_tick;

    if (StairSequence_StopChassis() != STAIR_SEQUENCE_OK)
    {
        return STAIR_BALL_MOTOR_ERROR;
    }
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_BALL_CANCELED;
    }
    if (MaixCamLink_SendRequest(MAIXCAM_COLOR_RED) != MAIXCAM_LINK_OK)
    {
        return STAIR_BALL_MAIX_UART_ERROR;
    }

    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) <
           STAIR_VISION_TIMEOUT_MS)
    {
        if (MotionControl_StopRequested != 0U)
        {
            (void)StairSequence_StopChassis();
            return STAIR_BALL_CANCELED;
        }
        if (MaixCamLink_TakeReply() != 0U)
        {
            return STAIR_BALL_FOUND;
        }
        osDelay(STAIR_VISION_POLL_MS);
    }
    if (MotionControl_StopRequested != 0U)
    {
        (void)StairSequence_StopChassis();
        return STAIR_BALL_CANCELED;
    }
    return STAIR_BALL_NOT_FOUND;
}

static uint8_t StairSequence_VisionEarlyStopCheck(void)
{
    if (MaixCamLink_TakeReply() != 0U)
    {
        stair_move_visual_hit = 1U;
        return 1U;
    }
    return 0U;
}

static StairSequenceStatus StairSequence_RunCameraPoseGroup(uint8_t group);

static StairBallResult StairSequence_Move90ThenCheck(
    StairSequenceState move_state,
    StairSequenceState check_state)
{
    MotionControlStatus motion_status;
    StairSequenceStatus stair_status;

    StairSequence_State = move_state;
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_BALL_CANCELED;
    }

    motion_status = MotionControl_MovePolarSegmentMm(
        90U,
        180.0f,
        0.0f,
        STAIR_FORWARD_RPM,
        0.0f);
    if (MotionControl_WasStopped() != 0U)
    {
        return STAIR_BALL_CANCELED;
    }
    stair_status = StairSequence_FromMotionStatus(motion_status);
    if (stair_status == STAIR_SEQUENCE_ERROR_IMU)
    {
        return STAIR_BALL_IMU_ERROR;
    }
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return STAIR_BALL_MOTOR_ERROR;
    }
    if (StairSequence_StopChassis() != STAIR_SEQUENCE_OK)
    {
        return STAIR_BALL_MOTOR_ERROR;
    }
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_BALL_CANCELED;
    }

    StairSequence_State = check_state;
    switch (StairSequence_CheckRedBallStopped())
    {
    case STAIR_BALL_FOUND:          return STAIR_BALL_FOUND;
    case STAIR_BALL_NOT_FOUND:      return STAIR_BALL_NOT_FOUND;
    case STAIR_BALL_CANCELED:       return STAIR_BALL_CANCELED;
    case STAIR_BALL_MAIX_UART_ERROR:return STAIR_BALL_MAIX_UART_ERROR;
    case STAIR_BALL_IMU_ERROR:      return STAIR_BALL_IMU_ERROR;
    case STAIR_BALL_MOTOR_ERROR:    return STAIR_BALL_MOTOR_ERROR;
    default:                        return STAIR_BALL_MOTOR_ERROR;
    }
}

/*
 * Part 2 uses the moving camera request only as a parking aid.  It must not
 * be converted to a business-level FOUND result, and it must not start an
 * action group while the chassis is moving.  The next loop iteration handles
 * the point pose and sends a fresh request for the formal stationary check.
 */
static StairPart2MoveResult StairSequence_Part2MoveToNextPoint(void)
{
    MotionControlStatus motion_status;
    StairSequenceStatus stair_status;
    uint8_t early_stopped = 0U;

    stair_move_visual_hit = 0U;
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_PART2_MOVE_CANCELED;
    }
    if (MaixCamLink_SendRequest(MAIXCAM_COLOR_RED) != MAIXCAM_LINK_OK)
    {
        return STAIR_PART2_MOVE_MAIX_UART_ERROR;
    }

    motion_status = MotionControl_MovePolarSegmentMmUntil(
        90U,
        180.0f,
        0.0f,
        STAIR_FORWARD_RPM,
        0.0f,
        StairSequence_VisionEarlyStopCheck,
        &early_stopped);
    if (MotionControl_WasStopped() != 0U)
    {
        return STAIR_PART2_MOVE_CANCELED;
    }
    stair_status = StairSequence_FromMotionStatus(motion_status);
    if (stair_status == STAIR_SEQUENCE_ERROR_IMU)
    {
        return STAIR_PART2_MOVE_IMU_ERROR;
    }
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return STAIR_PART2_MOVE_MOTOR_ERROR;
    }

    /* The motion API already sends zero at completion/early-stop.  Repeat it
       here so the point-state transition always begins from a stopped base. */
    stair_status = StairSequence_StopChassis();
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return STAIR_PART2_MOVE_MOTOR_ERROR;
    }
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_PART2_MOVE_CANCELED;
    }
    return ((early_stopped != 0U) || (stair_move_visual_hit != 0U)) ?
           STAIR_PART2_MOVE_VISION_STOP : STAIR_PART2_MOVE_DISTANCE_DONE;
}

static StairSequenceState StairSequence_Part2PointState(uint8_t point)
{
    switch (point)
    {
    case 0U: return STAIR_STATE_PART2_P1;
    case 1U: return STAIR_STATE_PART2_P2;
    case 2U: return STAIR_STATE_PART2_P3;
    default:return STAIR_STATE_PART2_P4;
    }
}

static StairSequenceState StairSequence_Part2MoveState(uint8_t point)
{
    switch (point)
    {
    case 1U: return STAIR_STATE_PART2_MOVE_TO_P2;
    case 2U: return STAIR_STATE_PART2_MOVE_TO_P3;
    default:return STAIR_STATE_PART2_MOVE_TO_P4;
    }
}

static StairSequenceStatus StairSequence_MapPart2MoveResult(
    StairPart2MoveResult result)
{
    switch (result)
    {
    case STAIR_PART2_MOVE_DISTANCE_DONE:
    case STAIR_PART2_MOVE_VISION_STOP:
        return STAIR_SEQUENCE_OK;
    case STAIR_PART2_MOVE_CANCELED:
        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    case STAIR_PART2_MOVE_IMU_ERROR:
        return STAIR_SEQUENCE_ERROR_IMU;
    case STAIR_PART2_MOVE_MAIX_UART_ERROR:
        return STAIR_SEQUENCE_ERROR_MAIX_UART;
    default:
        return STAIR_SEQUENCE_ERROR_MOTOR;
    }
}

static StairSequenceStatus StairSequence_RunCameraPoseGroup(uint8_t group)
{
    uint32_t start_tick;

    if ((group != STAIR_GROUP_5) &&
        (group != STAIR_GROUP_8) &&
        (group != STAIR_GROUP_11))
    {
        return STAIR_SEQUENCE_ERROR_SERVO;
    }
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    }
    if (ServoAction_StartGroupNoWait(group, 1U) != SERVO_ACTION_OK)
    {
        return STAIR_SEQUENCE_ERROR_SERVO;
    }

    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) <
           STAIR_CAMERA_POSE_WAIT_MS)
    {
        if (MotionControl_StopRequested != 0U)
        {
            return STAIR_SEQUENCE_CANCELED_BY_STOP;
        }
        osDelay(10U);
    }
    return STAIR_SEQUENCE_OK;
}

static StairSequenceStatus StairSequence_RunServoGroup(uint8_t group)
{
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    }
    if (ServoAction_RunGroup(group, 1U, STAIR_SERVO_TIMEOUT_MS) !=
        SERVO_ACTION_OK)
    {
        return STAIR_SEQUENCE_ERROR_SERVO;
    }
    return StairSequence_CheckStop();
}

static StairSequenceStatus StairSequence_RunGrabAndTurn(
    uint8_t group,
    StairSequenceState grab_state,
    StairSequenceState turn_state)
{
    TurntableStatus turntable_status;
    StairSequenceStatus status;

    StairSequence_State = grab_state;
    status = StairSequence_RunServoGroup(group);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }

    StairSequence_State = turn_state;
    status = StairSequence_CheckStop();
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }
    turntable_status = Turntable_MoveOneSlotAndWait(
        StairSequence_TurntableCancelCheck);
    if (turntable_status == TURNTABLE_STATUS_CANCELED)
    {
        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    }
    if (turntable_status != TURNTABLE_STATUS_OK)
    {
        return STAIR_SEQUENCE_ERROR_TURNTABLE;
    }
    return StairSequence_CheckStop();
}

static StairSequenceStatus StairSequence_RunTransitionGroup(
    uint8_t group,
    StairSequenceState state)
{
    StairSequence_State = state;
    return StairSequence_RunServoGroup(group);
}

static StairSequenceStatus StairSequence_Move117(
    StairSequenceState state)
{
    MotionControlStatus motion_status;
    StairSequenceStatus stair_status;

    StairSequence_State = state;
    motion_status = MotionControl_MovePolarSegmentMm(
        117U, 180.0f, 0.0f, STAIR_FORWARD_RPM, 0.0f);
    if (MotionControl_WasStopped() != 0U)
    {
        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    }
    stair_status = StairSequence_FromMotionStatus(motion_status);
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return stair_status;
    }
    stair_status = StairSequence_StopChassis();
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return stair_status;
    }
    return StairSequence_CheckStop();
}

static StairSequenceStatus StairSequence_Move20AfterAlign(void)
{
    MotionControlStatus motion_status;
    StairSequenceStatus stair_status;

    StairSequence_State = STAIR_STATE_MOVE20_AFTER_ALIGN;
    motion_status = MotionControl_MovePolarSegmentMm(
        STAIR_INITIAL_BACKWARD_MM,
        180.0f,
        0.0f,
        STAIR_FORWARD_RPM,
        0.0f);
    if (MotionControl_WasStopped() != 0U)
    {
        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    }
    stair_status = StairSequence_FromMotionStatus(motion_status);
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return stair_status;
    }
    stair_status = StairSequence_StopChassis();
    if (stair_status != STAIR_SEQUENCE_OK)
    {
        return stair_status;
    }
    return StairSequence_CheckStop();
}

static StairSequenceStatus StairSequence_MapBallError(
    StairBallResult result)
{
    switch (result)
    {
    case STAIR_BALL_CANCELED:        return STAIR_SEQUENCE_CANCELED_BY_STOP;
    case STAIR_BALL_IMU_ERROR:       return STAIR_SEQUENCE_ERROR_IMU;
    case STAIR_BALL_MOTOR_ERROR:     return STAIR_SEQUENCE_ERROR_MOTOR;
    case STAIR_BALL_MAIX_UART_ERROR: return STAIR_SEQUENCE_ERROR_MAIX_UART;
    case STAIR_BALL_SERVO_ERROR:     return STAIR_SEQUENCE_ERROR_SERVO;
    case STAIR_BALL_TURNTABLE_ERROR: return STAIR_SEQUENCE_ERROR_TURNTABLE;
    default:                         return STAIR_SEQUENCE_OK;
    }
}

static StairSequenceStatus StairSequence_RunPart1(void)
{
    StairSequenceStatus status;
    StairBallResult ball_result;

    StairSequence_State = STAIR_STATE_PART1_G5;
    status = StairSequence_RunCameraPoseGroup(STAIR_GROUP_5);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }

    StairSequence_State = STAIR_STATE_PART1_CHECK1;
    ball_result = StairSequence_CheckRedBallStopped();
    if (ball_result == STAIR_BALL_FOUND)
    {
        status = StairSequence_RunGrabAndTurn(
            STAIR_GROUP_6,
            STAIR_STATE_PART1_G6,
            STAIR_STATE_PART1_TURN);
        if (status != STAIR_SEQUENCE_OK)
        {
            return status;
        }
    }
    else if (ball_result != STAIR_BALL_NOT_FOUND)
    {
        return StairSequence_MapBallError(ball_result);
    }

    ball_result = StairSequence_Move90ThenCheck(
        STAIR_STATE_PART1_MOVE90,
        STAIR_STATE_PART1_CHECK2);
    if (ball_result == STAIR_BALL_FOUND)
    {
        status = StairSequence_RunGrabAndTurn(
            STAIR_GROUP_6,
            STAIR_STATE_PART1_G6,
            STAIR_STATE_PART1_TURN);
        if (status != STAIR_SEQUENCE_OK)
        {
            return status;
        }
    }
    else if (ball_result != STAIR_BALL_NOT_FOUND)
    {
        return StairSequence_MapBallError(ball_result);
    }

    status = StairSequence_RunTransitionGroup(
        STAIR_GROUP_0, STAIR_STATE_PART1_G0);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }
    return STAIR_SEQUENCE_OK;
}

static StairSequenceStatus StairSequence_RunPart2(void)
{
    StairSequenceStatus status;
    StairBallResult ball_result;

    uint8_t point;
    StairPart2MoveResult move_result;

    /* The preceding Backward117 ends at P1.  G8 is the single setup action for the
       complete Part 2 search; P2/P3/P4 keep the resulting search posture. */
    StairSequence_State = STAIR_STATE_PART2_G8;
    status = StairSequence_RunCameraPoseGroup(STAIR_GROUP_8);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }

    /* P1 is already reached; P2/P3/P4 each begin with one 90 mm move. */
    for (point = 0U; point < 4U; point++)
    {
        if (point > 0U)
        {
            StairSequence_State = StairSequence_Part2MoveState(point);
            move_result = StairSequence_Part2MoveToNextPoint();
            status = StairSequence_MapPart2MoveResult(move_result);
            if (status != STAIR_SEQUENCE_OK)
            {
                return status;
            }
        }

        StairSequence_State = StairSequence_Part2PointState(point);
        ball_result = StairSequence_CheckRedBallStopped();
        if (ball_result == STAIR_BALL_FOUND)
        {
            status = StairSequence_RunGrabAndTurn(
                STAIR_GROUP_9,
                STAIR_STATE_PART2_G9,
                STAIR_STATE_PART2_TURN);
            if (status != STAIR_SEQUENCE_OK)
            {
                return status;
            }
        }
        else if (ball_result != STAIR_BALL_NOT_FOUND)
        {
            return StairSequence_MapBallError(ball_result);
        }
    }

    status = StairSequence_RunTransitionGroup(
        STAIR_GROUP_7, STAIR_STATE_PART2_G7);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }
    return StairSequence_Move117(STAIR_STATE_PART2_MOVE117);
}

static StairSequenceStatus StairSequence_RunPart3(void)
{
    StairSequenceStatus status;
    StairBallResult ball_result;

    StairSequence_State = STAIR_STATE_PART3_G11;
    status = StairSequence_RunCameraPoseGroup(STAIR_GROUP_11);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }

    StairSequence_State = STAIR_STATE_PART3_CHECK1;
    ball_result = StairSequence_CheckRedBallStopped();
    if (ball_result == STAIR_BALL_FOUND)
    {
        status = StairSequence_RunGrabAndTurn(
            STAIR_GROUP_12,
            STAIR_STATE_PART3_G12,
            STAIR_STATE_PART3_TURN);
        if (status != STAIR_SEQUENCE_OK)
        {
            return status;
        }
    }
    else if (ball_result != STAIR_BALL_NOT_FOUND)
    {
        return StairSequence_MapBallError(ball_result);
    }

    /* The first point is always followed by one 90 mm move and a second check. */
    ball_result = StairSequence_Move90ThenCheck(
        STAIR_STATE_PART3_MOVE90,
        STAIR_STATE_PART3_CHECK2);
    if (ball_result == STAIR_BALL_FOUND)
    {
        status = StairSequence_RunGrabAndTurn(
            STAIR_GROUP_12,
            STAIR_STATE_PART3_G12,
            STAIR_STATE_PART3_TURN);
        if (status != STAIR_SEQUENCE_OK)
        {
            return status;
        }
    }
    else if (ball_result != STAIR_BALL_NOT_FOUND)
    {
        return StairSequence_MapBallError(ball_result);
    }

    status = StairSequence_RunTransitionGroup(
        STAIR_GROUP_10, STAIR_STATE_PART3_G10);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }
    return StairSequence_Move117(STAIR_STATE_PART3_MOVE117);
}

StairSequenceStatus StairSequence_Run(void)
{
    GrayAlignStatus gray_status;
    StairSequenceStatus status;

    StairSequence_State = STAIR_STATE_IDLE;
    StairSequence_LastStatus = STAIR_SEQUENCE_OK;

    if (Turntable_IsReady() == 0U)
    {
        return StairSequence_Finalize(STAIR_SEQUENCE_ERROR_TURNTABLE);
    }

    StairSequence_State = STAIR_STATE_ALIGNING;
    gray_status = GrayAlign_RunUnlimited();
    if (gray_status == GRAY_ALIGN_CANCELED)
    {
        return StairSequence_Finalize(STAIR_SEQUENCE_CANCELED_BY_STOP);
    }
    if (gray_status == GRAY_ALIGN_ERROR_IMU)
    {
        return StairSequence_Finalize(STAIR_SEQUENCE_ERROR_IMU);
    }
    if (gray_status == GRAY_ALIGN_ERROR_MOTOR_UART)
    {
        return StairSequence_Finalize(STAIR_SEQUENCE_ERROR_MOTOR);
    }
    if (gray_status != GRAY_ALIGN_OK)
    {
        return StairSequence_Finalize(STAIR_SEQUENCE_ERROR_GRAY_ALIGN);
    }

    status = StairSequence_Move20AfterAlign();
    if (status != STAIR_SEQUENCE_OK)
    {
        return StairSequence_Finalize(status);
    }

    status = StairSequence_RunPart3();
    if (status != STAIR_SEQUENCE_OK)
    {
        return StairSequence_Finalize(status);
    }
    status = StairSequence_RunPart2();
    if (status != STAIR_SEQUENCE_OK)
    {
        return StairSequence_Finalize(status);
    }
    status = StairSequence_RunPart1();
    return StairSequence_Finalize(status);
}

const char *StairSequence_StateName(StairSequenceState state)
{
    switch (state)
    {
    case STAIR_STATE_IDLE:          return "IDLE";
    case STAIR_STATE_ALIGNING:      return "ALIGNING";
    case STAIR_STATE_MOVE20_AFTER_ALIGN:return "MOVE20_AFTER_ALIGN";
    case STAIR_STATE_PART1_G5:      return "PART1_G5";
    case STAIR_STATE_PART1_CHECK1:  return "PART1_CHECK1";
    case STAIR_STATE_PART1_MOVE90: return "PART1_MOVE90";
    case STAIR_STATE_PART1_G6:      return "PART1_G6";
    case STAIR_STATE_PART1_TURN:    return "PART1_TURN";
    case STAIR_STATE_PART1_CHECK2:  return "PART1_CHECK2";
    case STAIR_STATE_PART1_G7:      return "PART1_G7";
    case STAIR_STATE_PART1_MOVE117:return "PART1_MOVE117";
    case STAIR_STATE_PART1_G0:      return "PART1_G0";
    case STAIR_STATE_PART2_G8:      return "PART2_G8";
    case STAIR_STATE_PART2_P1:      return "PART2_P1";
    case STAIR_STATE_PART2_MOVE_TO_P2:return "PART2_MOVE_TO_P2";
    case STAIR_STATE_PART2_P2:      return "PART2_P2";
    case STAIR_STATE_PART2_MOVE_TO_P3:return "PART2_MOVE_TO_P3";
    case STAIR_STATE_PART2_P3:      return "PART2_P3";
    case STAIR_STATE_PART2_MOVE_TO_P4:return "PART2_MOVE_TO_P4";
    case STAIR_STATE_PART2_P4:      return "PART2_P4";
    case STAIR_STATE_PART2_G9:      return "PART2_G9";
    case STAIR_STATE_PART2_TURN:    return "PART2_TURN";
    case STAIR_STATE_PART2_G10:     return "PART2_G10";
    case STAIR_STATE_PART2_MOVE117:return "PART2_MOVE117";
    case STAIR_STATE_PART2_G7:      return "PART2_G7";
    case STAIR_STATE_PART3_G11:     return "PART3_G11";
    case STAIR_STATE_PART3_CHECK1: return "PART3_CHECK1";
    case STAIR_STATE_PART3_MOVE90: return "PART3_MOVE90";
    case STAIR_STATE_PART3_CHECK2: return "PART3_CHECK2";
    case STAIR_STATE_PART3_G12:     return "PART3_G12";
    case STAIR_STATE_PART3_TURN:   return "PART3_TURN";
    case STAIR_STATE_PART3_G10:    return "PART3_G10";
    case STAIR_STATE_PART3_MOVE117:return "PART3_MOVE117";
    case STAIR_STATE_DONE:          return "DONE";
    case STAIR_STATE_CANCELED:      return "CANCELED";
    case STAIR_STATE_ERROR:         return "ERROR";
    default:                        return "UNKNOWN";
    }
}

const char *StairSequence_StatusName(StairSequenceStatus status)
{
    switch (status)
    {
    case STAIR_SEQUENCE_OK:                 return "OK";
    case STAIR_SEQUENCE_CANCELED_BY_STOP:  return "CANCELED";
    case STAIR_SEQUENCE_ERROR_GRAY_ALIGN:  return "GRAY_ALIGN";
    case STAIR_SEQUENCE_ERROR_IMU:         return "IMU";
    case STAIR_SEQUENCE_ERROR_MOTOR:       return "MOTOR";
    case STAIR_SEQUENCE_ERROR_SERVO:       return "SERVO";
    case STAIR_SEQUENCE_ERROR_TURNTABLE:  return "TURNTABLE";
    case STAIR_SEQUENCE_ERROR_MAIX_UART:  return "MAIX_UART";
    default:                               return "UNKNOWN";
    }
}
