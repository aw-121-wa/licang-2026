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

static StairBallResult StairSequence_Move90WithVisionAssist(
    StairSequenceState move_state,
    StairSequenceState check_state,
    uint8_t rearm_camera_before_static_check)
{
    MotionControlStatus motion_status;
    StairSequenceStatus stair_status;
    uint8_t early_stopped = 0U;

    StairSequence_State = move_state;
    stair_move_visual_hit = 0U;
    if (MotionControl_StopRequested != 0U)
    {
        return STAIR_BALL_CANCELED;
    }
    if (MaixCamLink_SendRequest(MAIXCAM_COLOR_RED) != MAIXCAM_LINK_OK)
    {
        return STAIR_BALL_MAIX_UART_ERROR;
    }

    motion_status = MotionControl_MovePolarSegmentMmUntil(
        90U,
        0.0f,
        0.0f,
        STAIR_FORWARD_RPM,
        0.0f,
        StairSequence_VisionEarlyStopCheck,
        &early_stopped);
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
    if ((early_stopped != 0U) || (stair_move_visual_hit != 0U))
    {
        return STAIR_BALL_FOUND;
    }

    if (rearm_camera_before_static_check != 0U)
    {
        StairSequence_State = STAIR_STATE_PART2_G8;
        stair_status = StairSequence_RunCameraPoseGroup(STAIR_GROUP_8);
        if (stair_status == STAIR_SEQUENCE_CANCELED_BY_STOP)
        {
            return STAIR_BALL_CANCELED;
        }
        if (stair_status != STAIR_SEQUENCE_OK)
        {
            return STAIR_BALL_SERVO_ERROR;
        }
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
        117U, 0.0f, 0.0f, STAIR_FORWARD_RPM, 0.0f);
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

    ball_result = StairSequence_Move90WithVisionAssist(
        STAIR_STATE_PART1_MOVE90,
        STAIR_STATE_PART1_CHECK2,
        0U);
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
        STAIR_GROUP_7, STAIR_STATE_PART1_G7);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }
    return StairSequence_Move117(STAIR_STATE_PART1_MOVE117);
}

static StairSequenceStatus StairSequence_RunPart2(void)
{
    StairSequenceStatus status;
    StairBallResult ball_result;
    uint8_t search_move;

    StairSequence_State = STAIR_STATE_PART2_G8;
    status = StairSequence_RunCameraPoseGroup(STAIR_GROUP_8);
    if (status != STAIR_SEQUENCE_OK)
    {
        return status;
    }

    StairSequence_State = STAIR_STATE_PART2_CHECK;
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
    else
    {
        /* Three 90 mm moves produce P1, P2 and P3 after P0. */
        for (search_move = 0U; search_move < 3U; search_move++)
        {
            ball_result = StairSequence_Move90WithVisionAssist(
                STAIR_STATE_PART2_MOVE90,
                STAIR_STATE_PART2_CHECK,
                1U);
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
                break;
            }
            if (ball_result != STAIR_BALL_NOT_FOUND)
            {
                return StairSequence_MapBallError(ball_result);
            }
        }
    }

    status = StairSequence_RunTransitionGroup(
        STAIR_GROUP_10, STAIR_STATE_PART2_G10);
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

    /* P0 never ends Part 3; the second point is always searched once. */
    ball_result = StairSequence_Move90WithVisionAssist(
        STAIR_STATE_PART3_MOVE90,
        STAIR_STATE_PART3_CHECK2,
        0U);
    if (ball_result == STAIR_BALL_FOUND)
    {
        return StairSequence_RunGrabAndTurn(
            STAIR_GROUP_12,
            STAIR_STATE_PART3_G12,
            STAIR_STATE_PART3_TURN);
    }
    if (ball_result != STAIR_BALL_NOT_FOUND)
    {
        return StairSequence_MapBallError(ball_result);
    }
    return STAIR_SEQUENCE_OK;
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
    gray_status = GrayAlign_Run();
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

    status = StairSequence_RunPart1();
    if (status != STAIR_SEQUENCE_OK)
    {
        return StairSequence_Finalize(status);
    }
    status = StairSequence_RunPart2();
    if (status != STAIR_SEQUENCE_OK)
    {
        return StairSequence_Finalize(status);
    }
    status = StairSequence_RunPart3();
    return StairSequence_Finalize(status);
}

const char *StairSequence_StateName(StairSequenceState state)
{
    switch (state)
    {
    case STAIR_STATE_IDLE:          return "IDLE";
    case STAIR_STATE_ALIGNING:      return "ALIGNING";
    case STAIR_STATE_PART1_G5:      return "PART1_G5";
    case STAIR_STATE_PART1_CHECK1:  return "PART1_CHECK1";
    case STAIR_STATE_PART1_MOVE90: return "PART1_MOVE90";
    case STAIR_STATE_PART1_G6:      return "PART1_G6";
    case STAIR_STATE_PART1_TURN:    return "PART1_TURN";
    case STAIR_STATE_PART1_CHECK2:  return "PART1_CHECK2";
    case STAIR_STATE_PART1_G7:      return "PART1_G7";
    case STAIR_STATE_PART1_MOVE117:return "PART1_MOVE117";
    case STAIR_STATE_PART2_G8:      return "PART2_G8";
    case STAIR_STATE_PART2_CHECK:   return "PART2_CHECK";
    case STAIR_STATE_PART2_MOVE90:  return "PART2_MOVE90";
    case STAIR_STATE_PART2_G9:      return "PART2_G9";
    case STAIR_STATE_PART2_TURN:    return "PART2_TURN";
    case STAIR_STATE_PART2_G10:     return "PART2_G10";
    case STAIR_STATE_PART2_MOVE117:return "PART2_MOVE117";
    case STAIR_STATE_PART3_G11:     return "PART3_G11";
    case STAIR_STATE_PART3_CHECK1: return "PART3_CHECK1";
    case STAIR_STATE_PART3_MOVE90: return "PART3_MOVE90";
    case STAIR_STATE_PART3_CHECK2: return "PART3_CHECK2";
    case STAIR_STATE_PART3_G12:     return "PART3_G12";
    case STAIR_STATE_PART3_TURN:   return "PART3_TURN";
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
