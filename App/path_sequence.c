#include "path_sequence.h"
#include "ball_sequence.h"
#include "motor_control.h"
#include "motion_control.h"
#include "round_pillar.h"
#include "warehouse_control.h"

typedef struct
{
    PathStepType type;
    uint32_t distance_mm;
    float angle_deg;
    float cruise_rpm;
} PathStep;

/*
 * ===============================
 * COMPETITION PATH - EDIT HERE
 * ===============================
 */
static const PathStep PathSequence_Steps[] =
{
    /* 1. 左前45度前进1850 mm */
    {
        PATH_STEP_MOVE_POLAR,
        1850U,
        45.0f,
        MOTION_DIAGONAL_CRUISE_RPM
    },

    /* 2. 正前方前进2300 mm */
    {
        PATH_STEP_MOVE_POLAR,
        2300U,
        0.0f,
        MOTION_CRUISE_RPM
    },

    /* 3. 原地逆时针旋转178度 */
    {
        PATH_STEP_ROTATE,
        0U,
        178.0f,
        0.0f
    },

    /* 4. 执行当前已有BALL完整流程 */
    {
        PATH_STEP_BALL,
        0U,
        0.0f,
        0.0f
    },

    /* 5. BALL完成后原地逆时针旋转180度 */
    {
        PATH_STEP_ROTATE,
        0U,
        180.0f,
        0.0f
    },

    /* 6. 后退1820 mm */
    {
        PATH_STEP_MOVE_POLAR,
        1820U,
        180.0f,
        MOTION_CRUISE_RPM
    },

    /* 7. 执行当前已有RZ完整流程 */
    {
        PATH_STEP_RZ,
        0U,
        0.0f,
        0.0f
    }
};

#define PATH_STEP_COUNT ((uint8_t)(sizeof(PathSequence_Steps) / sizeof(PathSequence_Steps[0])))

volatile PathSequenceState PathSequence_State = PATH_SEQUENCE_IDLE;
volatile PathSequenceStatus PathSequence_LastStatus = PATH_SEQUENCE_OK;
volatile uint8_t PathSequence_CurrentStep = 0U;

static PathSequenceState PathSequence_StateForStep(PathStepType type,
                                                    uint8_t step_index)
{
    if (type == PATH_STEP_MOVE_POLAR)
    {
        if (step_index == 0U)
        {
            return PATH_SEQUENCE_MOVE_LF_1850;
        }
        if (step_index == 1U)
        {
            return PATH_SEQUENCE_MOVE_F_2300;
        }
        return PATH_SEQUENCE_BACK_1820;
    }
    if (type == PATH_STEP_ROTATE)
    {
        return (step_index == 2U) ? PATH_SEQUENCE_ROTATE_178 :
                                    PATH_SEQUENCE_ROTATE_180;
    }
    if (type == PATH_STEP_BALL)
    {
        return PATH_SEQUENCE_BALL;
    }
    return PATH_SEQUENCE_RZ;
}

static PathSequenceStatus PathSequence_Finalize(PathSequenceStatus status)
{
    if (MotionControl_SetBodySpeed(0.0f, 0.0f, 0.0f) != HAL_OK)
    {
        (void)MotorControl_StopAll();
        if (status == PATH_SEQUENCE_OK)
        {
            status = PATH_SEQUENCE_ERROR_MOTION;
        }
    }

    PathSequence_LastStatus = status;
    PathSequence_State = (status == PATH_SEQUENCE_OK) ?
                         PATH_SEQUENCE_DONE :
                         ((status == PATH_SEQUENCE_CANCELED) ?
                          PATH_SEQUENCE_STATE_CANCELED : PATH_SEQUENCE_ERROR);
    return status;
}

static PathSequenceStatus PathSequence_MapMotionStatus(
    MotionControlStatus status)
{
    if ((MotionControl_StopRequested != 0U) ||
        (MotionControl_WasStopped() != 0U))
    {
        return PATH_SEQUENCE_CANCELED;
    }
    if (status < MOTION_ERROR_IMU_STARTUP)
    {
        return PATH_SEQUENCE_OK;
    }
    if ((status == MOTION_ERROR_IMU_STARTUP) ||
        (status == MOTION_ERROR_IMU_LOST))
    {
        return PATH_SEQUENCE_ERROR_IMU;
    }
    if (status == MOTION_ERROR_ROTATE_TIMEOUT)
    {
        return PATH_SEQUENCE_ERROR_ROTATE;
    }
    return PATH_SEQUENCE_ERROR_MOTION;
}

static PathSequenceStatus PathSequence_MapBallStatus(
    BallSequenceStatus status)
{
    switch (status)
    {
    case BALL_SEQUENCE_OK:
        return PATH_SEQUENCE_OK;
    case BALL_SEQUENCE_CANCELED_BY_STOP:
        return PATH_SEQUENCE_CANCELED;
    case BALL_SEQUENCE_ERROR_MAIX_UART:
        return PATH_SEQUENCE_ERROR_BALL_MAIX_UART;
    case BALL_SEQUENCE_ERROR_MAIX_TIMEOUT:
        return PATH_SEQUENCE_ERROR_BALL_MAIX_TIMEOUT;
    case BALL_SEQUENCE_ERROR_SERVO:
        return PATH_SEQUENCE_ERROR_BALL_SERVO;
    case BALL_SEQUENCE_ERROR_TURNTABLE:
        return PATH_SEQUENCE_ERROR_BALL_TURNTABLE;
    case BALL_SEQUENCE_ERROR_GRAY_ALIGN:
        return PATH_SEQUENCE_ERROR_BALL_GRAY_ALIGN;
    default:
        return PATH_SEQUENCE_ERROR_MOTION;
    }
}

static PathSequenceStatus PathSequence_MapRzStatus(RoundPillarStatus status)
{
    switch (status)
    {
    case ROUND_PILLAR_OK:
        return PATH_SEQUENCE_OK;
    case ROUND_PILLAR_CANCELED:
        return PATH_SEQUENCE_CANCELED;
    case ROUND_PILLAR_ERROR_IMU:
        return PATH_SEQUENCE_ERROR_RZ_IMU;
    case ROUND_PILLAR_ERROR_MOTOR:
    case ROUND_PILLAR_ERROR_APPROACH_TIMEOUT:
        return (status == ROUND_PILLAR_ERROR_MOTOR) ?
               PATH_SEQUENCE_ERROR_RZ_MOTOR : PATH_SEQUENCE_ERROR_RZ_TIMEOUT;
    case ROUND_PILLAR_ERROR_SERVO:
        return PATH_SEQUENCE_ERROR_RZ_SERVO;
    case ROUND_PILLAR_ERROR_TURNTABLE:
        return PATH_SEQUENCE_ERROR_RZ_TURNTABLE;
    case ROUND_PILLAR_ERROR_MAIX_UART:
        return PATH_SEQUENCE_ERROR_RZ_MAIX_UART;
    case ROUND_PILLAR_ERROR_MAIX_TIMEOUT:
        return PATH_SEQUENCE_ERROR_RZ_MAIX_TIMEOUT;
    case ROUND_PILLAR_ERROR_ORBIT_TIMEOUT:
        return PATH_SEQUENCE_ERROR_RZ_TIMEOUT;
    default:
        return PATH_SEQUENCE_ERROR_RZ_TIMEOUT;
    }
}

static PathSequenceStatus PathSequence_RunStep(const PathStep *step)
{
    MotionControlStatus motion_status;
    BallSequenceStatus ball_status;
    RoundPillarStatus rz_status;

    if (step->type == PATH_STEP_MOVE_POLAR)
    {
        motion_status = MotionControl_MovePolarSegmentMm(
            step->distance_mm,
            step->angle_deg,
            0.0f,
            step->cruise_rpm,
            0.0f);
        return PathSequence_MapMotionStatus(motion_status);
    }
    if (step->type == PATH_STEP_ROTATE)
    {
        motion_status = MotionControl_RotateDeg(step->angle_deg);
        return PathSequence_MapMotionStatus(motion_status);
    }
    if (step->type == PATH_STEP_BALL)
    {
        ball_status = BallSequence_Run();
        return PathSequence_MapBallStatus(ball_status);
    }

    rz_status = RoundPillar_Run();
    return PathSequence_MapRzStatus(rz_status);
}

PathSequenceStatus PathSequence_Run(void)
{
    uint8_t step_index;
    PathSequenceStatus path_status;

    PathSequence_State = PATH_SEQUENCE_IDLE;
    PathSequence_LastStatus = PATH_SEQUENCE_OK;
    PathSequence_CurrentStep = 0U;

    if (WarehouseControl_IsReadyForAction() == 0U)
    {
        return PathSequence_Finalize(PATH_SEQUENCE_ERROR_NOT_READY);
    }
    if ((MotionControl_StopRequested != 0U) ||
        (MotionControl_WasStopped() != 0U))
    {
        return PathSequence_Finalize(PATH_SEQUENCE_CANCELED);
    }

    for (step_index = 0U; step_index < PATH_STEP_COUNT; step_index++)
    {
        PathSequence_CurrentStep = step_index;
        PathSequence_State = PathSequence_StateForStep(
            PathSequence_Steps[step_index].type, step_index);

        if ((MotionControl_StopRequested != 0U) ||
            (MotionControl_WasStopped() != 0U))
        {
            return PathSequence_Finalize(PATH_SEQUENCE_CANCELED);
        }

        path_status = PathSequence_RunStep(&PathSequence_Steps[step_index]);
        if (path_status != PATH_SEQUENCE_OK)
        {
            return PathSequence_Finalize(path_status);
        }
        if ((MotionControl_StopRequested != 0U) ||
            (MotionControl_WasStopped() != 0U))
        {
            return PathSequence_Finalize(PATH_SEQUENCE_CANCELED);
        }
    }

    return PathSequence_Finalize(PATH_SEQUENCE_OK);
}

MotionControlStatus PathSequence_ToMotionControlStatus(
    PathSequenceStatus status)
{
    switch (status)
    {
    case PATH_SEQUENCE_OK:
        return MOTION_STATUS_FINISHED;
    case PATH_SEQUENCE_CANCELED:
        return MOTION_STATUS_FINISHED;
    case PATH_SEQUENCE_ERROR_IMU:
    case PATH_SEQUENCE_ERROR_RZ_IMU:
        return MOTION_ERROR_IMU_LOST;
    case PATH_SEQUENCE_ERROR_ROTATE:
        return MOTION_ERROR_ROTATE_TIMEOUT;
    case PATH_SEQUENCE_ERROR_BALL_MAIX_TIMEOUT:
    case PATH_SEQUENCE_ERROR_RZ_MAIX_TIMEOUT:
        return MOTION_ERROR_MAIX_TIMEOUT;
    case PATH_SEQUENCE_ERROR_BALL_MAIX_UART:
    case PATH_SEQUENCE_ERROR_RZ_MAIX_UART:
        return MOTION_ERROR_MAIX_UART;
    case PATH_SEQUENCE_ERROR_BALL_GRAY_ALIGN:
        return MOTION_ERROR_GRAY_ALIGN;
    case PATH_SEQUENCE_ERROR_MOTION:
    case PATH_SEQUENCE_ERROR_NOT_READY:
    case PATH_SEQUENCE_ERROR_BALL_SERVO:
    case PATH_SEQUENCE_ERROR_BALL_TURNTABLE:
    case PATH_SEQUENCE_ERROR_RZ_MOTOR:
    case PATH_SEQUENCE_ERROR_RZ_SERVO:
    case PATH_SEQUENCE_ERROR_RZ_TURNTABLE:
    case PATH_SEQUENCE_ERROR_RZ_TIMEOUT:
    default:
        return MOTION_ERROR_MOTOR_UART;
    }
}
