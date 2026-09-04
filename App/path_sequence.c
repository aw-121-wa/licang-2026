#include "path_sequence.h"
#include "motor_control.h"
#include "servo_action.h"
#include "warehouse_control.h"

/*
 * =====================================================
 * COMPETITION PATH COMMAND QUEUE
 * 修改比赛路线时只修改这里
 * =====================================================
 */
static const PathStep PathSequence_CommandQueue[] =
{
    /* STEP 0: 左前20°，1800 mm */
    {
        PATH_STEP_MOVE,
        1800U,
        20.0f,
        MOTION_DIAGONAL_CRUISE_RPM,
        0U
    },

    /* STEP 1: 前进2300 mm */
    {
        PATH_STEP_MOVE,
        2300U,
        0.0f,
        MOTION_CRUISE_RPM,
        0U
    },

    /* STEP 2: 原地逆时针旋转178° */
    {
        PATH_STEP_ROTATE,
        0U,
        178.0f,
        0.0f,
        0U
    },

    /* STEP 3: 当前已有 BALL 流程 */
    {
        PATH_STEP_BALL,
        0U,
        0.0f,
        0.0f,
        0U
    },

    /* STEP 4: BALL 成功后再次原地逆时针旋转178° */
    {
        PATH_STEP_ROTATE,
        0U,
        178.0f,
        0.0f,
        0U
    },

    /* STEP 5: 后退1810 mm，180°表示后退方向 */
    {
        PATH_STEP_MOVE,
        1810U,
        180.0f,
        MOTION_CRUISE_RPM,
        0U
    },

    /* STEP 6: 当前已有 RZ 流程 */
    {
        PATH_STEP_RZ,
        0U,
        0.0f,
        0.0f,
        0U
    },

    /* STEP 7: RZ成功后阻塞等待第一次动作组0回位完成 */
    {
        PATH_STEP_SERVO_GROUP,
        0U,
        0.0f,
        0.0f,
        SERVO_ACTION_START_GROUP
    },

    /* STEP 8: 第一次动作组0完成后前进330 mm */
    {
        PATH_STEP_MOVE,
        330U,
        0.0f,
        MOTION_CRUISE_RPM,
        0U
    },

    /* STEP 9: 执行完整 STAIR 流程 */
    {
        PATH_STEP_STAIR,
        0U,
        0.0f,
        0.0f,
        0U
    },

    /* STEP 10: STAIR成功后阻塞等待第二次动作组0回位完成 */
    {
        PATH_STEP_SERVO_GROUP,
        0U,
        0.0f,
        0.0f,
        SERVO_ACTION_START_GROUP
    },

    /* STEP 11: 第二次动作组0完成后向左横移1600 mm */
    {
        PATH_STEP_MOVE,
        1600U,
        90.0f,
        MOTION_CRUISE_RPM,
        0U
    },

    /* STEP 12: 完成现有路径后执行仓库搬运流程 */
    {
        PATH_STEP_CANGKU,
        0U,
        0.0f,
        0.0f,
        0U
    }
};

#define PATH_SEQUENCE_STEP_COUNT \
    (sizeof(PathSequence_CommandQueue) / \
     sizeof(PathSequence_CommandQueue[0]))

volatile PathSequenceState PathSequence_State = PATH_SEQUENCE_IDLE;
volatile uint8_t PathSequence_CurrentStep = 0U;
volatile PathSequenceStatus PathSequence_LastStatus = PATH_SEQUENCE_OK;
volatile BallSequenceStatus PathSequence_LastBallStatus = BALL_SEQUENCE_OK;
volatile RoundPillarStatus PathSequence_LastRzStatus = ROUND_PILLAR_OK;
volatile StairSequenceStatus PathSequence_LastStairStatus = STAIR_SEQUENCE_OK;
volatile CangkuSequenceStatus PathSequence_LastCangkuStatus = CANGKU_STATUS_OK;
volatile MotionControlStatus PathSequence_LastMotionStatus = MOTION_STATUS_IDLE;

static void PathSequence_SetStepState(uint32_t step_index)
{
    switch (step_index)
    {
    case 0U: PathSequence_State = PATH_SEQUENCE_LF20_1800; break;
    case 1U: PathSequence_State = PATH_SEQUENCE_F2300;      break;
    case 2U: PathSequence_State = PATH_SEQUENCE_ROTATE1_178;break;
    case 3U: PathSequence_State = PATH_SEQUENCE_BALL;       break;
    case 4U: PathSequence_State = PATH_SEQUENCE_ROTATE2_178;break;
    case 5U: PathSequence_State = PATH_SEQUENCE_BACK1820;   break;
    case 6U: PathSequence_State = PATH_SEQUENCE_RZ;         break;
    case 7U: PathSequence_State = PATH_SEQUENCE_GROUP0;      break;
    case 8U: PathSequence_State = PATH_SEQUENCE_F330;        break;
    case 9U: PathSequence_State = PATH_SEQUENCE_STAIR;       break;
    case 10U:PathSequence_State = PATH_SEQUENCE_GROUP0;      break;
    case 11U:PathSequence_State = PATH_SEQUENCE_LEFT_2000;   break;
    case 12U:PathSequence_State = PATH_SEQUENCE_CANGKU;      break;
    default: PathSequence_State = PATH_SEQUENCE_IDLE;       break;
    }
}

static void PathSequence_StopChassis(void)
{
    if (MotionControl_SetBodySpeed(0.0f, 0.0f, 0.0f) != HAL_OK)
    {
        (void)MotorControl_StopAll();
    }
}

static PathSequenceStatus PathSequence_Cancel(void)
{
    PathSequence_StopChassis();
    PathSequence_State = PATH_SEQUENCE_CANCELED;
    PathSequence_LastStatus = PATH_SEQUENCE_STATUS_CANCELED;
    return PATH_SEQUENCE_STATUS_CANCELED;
}

static PathSequenceStatus PathSequence_Fail(PathSequenceStatus status)
{
    PathSequence_StopChassis();
    PathSequence_State = PATH_SEQUENCE_ERROR;
    PathSequence_LastStatus = status;
    return status;
}

static uint8_t PathSequence_StopPending(void)
{
    return ((MotionControl_StopRequested != 0U) ||
            (MotionControl_WasStopped() != 0U)) ? 1U : 0U;
}

static uint8_t PathSequence_IsReadyForAction(void)
{
    return ((WarehouseControl_IsReadyForAction() != 0U) &&
            ((ServoAction_SequenceState == SERVO_SEQUENCE_WAITING_MOTION) ||
             (ServoAction_SequenceState == SERVO_SEQUENCE_DONE))) ? 1U : 0U;
}

PathSequenceStatus PathSequence_Run(void)
{
    uint32_t i;

    PathSequence_State = PATH_SEQUENCE_IDLE;
    PathSequence_CurrentStep = 0U;
    PathSequence_LastStatus = PATH_SEQUENCE_OK;
    PathSequence_LastBallStatus = BALL_SEQUENCE_OK;
    PathSequence_LastRzStatus = ROUND_PILLAR_OK;
    PathSequence_LastStairStatus = STAIR_SEQUENCE_OK;
    PathSequence_LastCangkuStatus = CANGKU_STATUS_OK;
    PathSequence_LastMotionStatus = MOTION_STATUS_IDLE;

    /* This is the only warehouse/servo readiness check for the whole PATH. */
    if (PathSequence_IsReadyForAction() == 0U)
    {
        return PathSequence_Fail(PATH_SEQUENCE_ERROR_BALL);
    }

    for (i = 0U; i < PATH_SEQUENCE_STEP_COUNT; i++)
    {
        const PathStep *step = &PathSequence_CommandQueue[i];

        PathSequence_CurrentStep = (uint8_t)i;
        PathSequence_SetStepState(i);

        /* Never clear the stop request here: STOP cancels the whole PATH. */
        if (PathSequence_StopPending() != 0U)
        {
            return PathSequence_Cancel();
        }

        switch (step->type)
        {
        case PATH_STEP_MOVE:
            PathSequence_LastMotionStatus =
                MotionControl_MovePolarSegmentMm(
                    step->distance_mm,
                    step->angle_deg,
                    0.0f,
                    step->cruise_rpm,
                    0.0f);
            if (MotionControl_WasStopped() != 0U)
            {
                return PathSequence_Cancel();
            }
            if (PathSequence_LastMotionStatus >= MOTION_ERROR_IMU_STARTUP)
            {
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_MOTION);
            }
            break;

        case PATH_STEP_ROTATE:
            PathSequence_LastMotionStatus =
                MotionControl_RotateDeg(step->angle_deg);
            if (MotionControl_WasStopped() != 0U)
            {
                return PathSequence_Cancel();
            }
            if (PathSequence_LastMotionStatus >= MOTION_ERROR_IMU_STARTUP)
            {
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_ROTATE);
            }
            break;

        case PATH_STEP_BALL:
            PathSequence_LastBallStatus = BallSequence_Run();
            if (PathSequence_LastBallStatus == BALL_SEQUENCE_CANCELED_BY_STOP)
            {
                return PathSequence_Cancel();
            }
            if (PathSequence_LastBallStatus != BALL_SEQUENCE_OK)
            {
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_BALL);
            }
            /* BALL OK unconditionally advances to the next PATH step. */
            break;

        case PATH_STEP_RZ:
            PathSequence_LastRzStatus = RoundPillar_Run();
            if (PathSequence_LastRzStatus == ROUND_PILLAR_CANCELED)
            {
                return PathSequence_Cancel();
            }
            if (PathSequence_LastRzStatus != ROUND_PILLAR_OK)
            {
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_RZ);
            }
            break;

        case PATH_STEP_STAIR:
            PathSequence_LastStairStatus = StairSequence_Run();
            if (PathSequence_LastStairStatus == STAIR_SEQUENCE_CANCELED_BY_STOP)
            {
                return PathSequence_Cancel();
            }
            if (PathSequence_LastStairStatus != STAIR_SEQUENCE_OK)
            {
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_STAIR);
            }
            break;

        case PATH_STEP_CANGKU:
            PathSequence_LastCangkuStatus = CangkuSequence_Run();
            if (PathSequence_LastCangkuStatus == CANGKU_STATUS_CANCELED)
            {
                return PathSequence_Cancel();
            }
            if (PathSequence_LastCangkuStatus != CANGKU_STATUS_OK)
            {
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_CANGKU);
            }
            break;

        case PATH_STEP_SERVO_GROUP:
        {
            ServoActionStatus servo_status;

            ServoAction_SequenceState = SERVO_SEQUENCE_RETURN_RUNNING;
            servo_status = ServoAction_RunGroup(
                step->servo_group,
                1U,
                SERVO_ACTION_START_TIMEOUT_MS);
            if (PathSequence_StopPending() != 0U)
            {
                return PathSequence_Cancel();
            }
            if (servo_status != SERVO_ACTION_OK)
            {
                ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
                return PathSequence_Fail(PATH_SEQUENCE_ERROR_SERVO);
            }
            ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
            break;
        }

        default:
            return PathSequence_Fail(PATH_SEQUENCE_ERROR_MOTION);
        }

        /* A STOP received between two synchronous application calls also
           cancels the remaining steps.  Do not reset the request. */
        if (PathSequence_StopPending() != 0U)
        {
            return PathSequence_Cancel();
        }
    }

    /* All steps completed; stop once more as the final PATH safety check. */
    PathSequence_StopChassis();
    PathSequence_State = PATH_SEQUENCE_DONE;
    PathSequence_LastStatus = PATH_SEQUENCE_OK;
    return PATH_SEQUENCE_OK;
}

const char *PathSequence_StateName(PathSequenceState state)
{
    switch (state)
    {
    case PATH_SEQUENCE_IDLE:          return "IDLE";
    case PATH_SEQUENCE_LF20_1800:     return "LF20_1800";
    case PATH_SEQUENCE_F2300:         return "F2300";
    case PATH_SEQUENCE_ROTATE1_178:   return "ROTATE1_178";
    case PATH_SEQUENCE_BALL:          return "BALL";
    case PATH_SEQUENCE_ROTATE2_178:   return "ROTATE2_178";
    case PATH_SEQUENCE_BACK1820:      return "BACK1820";
    case PATH_SEQUENCE_RZ:            return "RZ";
    case PATH_SEQUENCE_GROUP0:        return "GROUP0";
    case PATH_SEQUENCE_F330:          return "F330";
    case PATH_SEQUENCE_STAIR:         return "STAIR";
    case PATH_SEQUENCE_LEFT_2000:     return "LEFT_2000";
    case PATH_SEQUENCE_CANGKU:        return "CANGKU";
    case PATH_SEQUENCE_DONE:          return "DONE";
    case PATH_SEQUENCE_CANCELED:      return "CANCELED";
    case PATH_SEQUENCE_ERROR:         return "ERROR";
    default:                          return "UNKNOWN";
    }
}

const char *PathSequence_StatusName(PathSequenceStatus status)
{
    switch (status)
    {
    case PATH_SEQUENCE_OK:             return "OK";
    case PATH_SEQUENCE_STATUS_CANCELED:return "CANCELED";
    case PATH_SEQUENCE_ERROR_MOTION:   return "MOTION";
    case PATH_SEQUENCE_ERROR_ROTATE:   return "ROTATE";
    case PATH_SEQUENCE_ERROR_BALL:     return "BALL";
    case PATH_SEQUENCE_ERROR_RZ:       return "RZ";
    case PATH_SEQUENCE_ERROR_SERVO:    return "SERVO";
    case PATH_SEQUENCE_ERROR_STAIR:    return "STAIR";
    case PATH_SEQUENCE_ERROR_CANGKU:   return "CANGKU";
    default:                           return "UNKNOWN";
    }
}
