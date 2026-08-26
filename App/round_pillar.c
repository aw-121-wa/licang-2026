#include "round_pillar.h"
#include "cmsis_os.h"
#include "jy61p.h"
#include "maixcam_link.h"
#include "motor_control.h"
#include "motion_control.h"
#include "servo_action.h"
#include "turntable_control.h"

static uint8_t RoundPillar_IrDetected(void)
{
    return (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) ==
            RZ_IR_DETECTED_LEVEL) ? 1U : 0U;
}

static HAL_StatusTypeDef RoundPillar_Stop(void)
{
    HAL_StatusTypeDef status = MotionControl_SetBodySpeed(0.0f, 0.0f, 0.0f);

    if (status != HAL_OK)
    {
        (void)MotorControl_StopAll();
    }
    return status;
}

static RoundPillarStatus RoundPillar_CheckStopAndImu(void)
{
    if (MotionControl_StopRequested != 0U)
    {
        (void)RoundPillar_Stop();
        return ROUND_PILLAR_CANCELED;
    }
    if (Jy61P_IsOnline(500U) == 0U)
    {
        (void)RoundPillar_Stop();
        return ROUND_PILLAR_ERROR_IMU;
    }
    return ROUND_PILLAR_OK;
}

static RoundPillarStatus RoundPillar_WaitSettled(uint32_t duration_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) < duration_ms)
    {
        RoundPillarStatus status = RoundPillar_CheckStopAndImu();

        if (status != ROUND_PILLAR_OK)
        {
            return status;
        }
        osDelay(RZ_PERIOD_MS);
    }
    return ROUND_PILLAR_OK;
}

static RoundPillarStatus RoundPillar_WaitCameraRaise(void)
{
    uint32_t start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) <
           RZ_CAMERA_RAISE_WAIT_MS)
    {
        if (MotionControl_StopRequested != 0U)
        {
            return ROUND_PILLAR_CANCELED;
        }
        osDelay(RZ_PERIOD_MS);
    }
    return ROUND_PILLAR_OK;
}

static uint8_t RoundPillar_TurntableCancelCheck(void)
{
    return (MotionControl_StopRequested != 0U) ? 1U : 0U;
}

static RoundPillarStatus RoundPillar_MapMotionStatus(
    MotionControlStatus status)
{
    if ((status == MOTION_ERROR_IMU_LOST) ||
        (status == MOTION_ERROR_IMU_STARTUP))
    {
        return ROUND_PILLAR_ERROR_IMU;
    }
    if (status == MOTION_ERROR_MOTOR_UART)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    if (MotionControl_WasStopped() != 0U)
    {
        return ROUND_PILLAR_CANCELED;
    }
    return ROUND_PILLAR_ERROR_MOTOR;
}

static RoundPillarStatus RoundPillar_Approach(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t ir_detect_since = 0U;
    uint8_t ir_stable = 0U;

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        RoundPillarStatus status;
        float correction;

        status = RoundPillar_CheckStopAndImu();
        if (status != ROUND_PILLAR_OK)
        {
            return status;
        }
        if ((uint32_t)(now - start_tick) >= RZ_APPROACH_TIMEOUT_MS)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_APPROACH_TIMEOUT;
        }

        if (RoundPillar_IrDetected() != 0U)
        {
            if (ir_stable == 0U)
            {
                ir_stable = 1U;
                ir_detect_since = now;
            }
            if ((uint32_t)(now - ir_detect_since) >= RZ_IR_STABLE_MS)
            {
                return ROUND_PILLAR_OK;
            }
        }
        else
        {
            ir_stable = 0U;
        }

        correction = MotionControl_GetHeadingCorrection(RZ_APPROACH_RPM);
        if (MotionControl_SetBodySpeed(0.0f,
                                       -RZ_APPROACH_RPM,
                                       correction) != HAL_OK)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_MOTOR;
        }
        osDelay(RZ_PERIOD_MS);
    }
}

static RoundPillarStatus RoundPillar_HandleDetectedBall(
    uint8_t *grab_count)
{
    ServoActionStatus servo_status;
    TurntableStatus turntable_status;
    RoundPillarStatus status;

    if (RoundPillar_Stop() != HAL_OK)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    status = RoundPillar_WaitSettled(RZ_STOP_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    ServoAction_SequenceState = SERVO_SEQUENCE_GRAB_RUNNING;
    servo_status = ServoAction_RunGroup(
        SERVO_ACTION_PILLAR_GRAB_GROUP,
        1U,
        SERVO_ACTION_PILLAR_GRAB_TIMEOUT_MS);
    if (servo_status != SERVO_ACTION_OK)
    {
        ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
        return ROUND_PILLAR_ERROR_SERVO;
    }

    turntable_status = Turntable_MoveOneSlotAndWait(
        RoundPillar_TurntableCancelCheck);
    if (turntable_status == TURNTABLE_STATUS_CANCELED)
    {
        return ROUND_PILLAR_CANCELED;
    }
    if (turntable_status != TURNTABLE_STATUS_OK)
    {
        return ROUND_PILLAR_ERROR_TURNTABLE;
    }

    (*grab_count)++;
    ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
    if (MotionControl_StopRequested != 0U)
    {
        return ROUND_PILLAR_CANCELED;
    }
    if (*grab_count < RZ_GRAB_COUNT)
    {
        if (MaixCamLink_SendRequest(MAIXCAM_COLOR_RED) != MAIXCAM_LINK_OK)
        {
            return ROUND_PILLAR_ERROR_MAIX_UART;
        }
    }
    return ROUND_PILLAR_OK;
}

static RoundPillarStatus RoundPillar_OrbitAndGrab(void)
{
    uint32_t orbit_start_tick;
    uint32_t handling_start_tick;
    float reverse_start_yaw;
    float reverse_target_yaw;
    float current_yaw;
    uint8_t grab_count = 0U;
    RoundPillarStatus status;

    MotionControl_ResetHeadingReference();
    orbit_start_tick = HAL_GetTick();
    MotionControl_State = MOTION_STATUS_ROTATING;
    if (MaixCamLink_SendRequest(MAIXCAM_COLOR_RED) != MAIXCAM_LINK_OK)
    {
        return ROUND_PILLAR_ERROR_MAIX_UART;
    }
    current_yaw = Jy61P_GetContinuousYaw();

    while (current_yaw > RZ_CW_TARGET_DEG)
    {
        status = RoundPillar_CheckStopAndImu();
        if (status != ROUND_PILLAR_OK)
        {
            return status;
        }
        if ((uint32_t)(HAL_GetTick() - orbit_start_tick) >=
            RZ_ORBIT_TIMEOUT_MS)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_ORBIT_TIMEOUT;
        }

        current_yaw = Jy61P_GetContinuousYaw();
        if ((grab_count < RZ_GRAB_COUNT) &&
            (MaixCamLink_TakeReply() != 0U))
        {
            handling_start_tick = HAL_GetTick();
            status = RoundPillar_HandleDetectedBall(&grab_count);
            orbit_start_tick += HAL_GetTick() - handling_start_tick;
            if (status != ROUND_PILLAR_OK)
            {
                return status;
            }
            continue;
        }
        if (current_yaw <= RZ_CW_TARGET_DEG)
        {
            break;
        }
        if (MotionControl_SetBodySpeed(RZ_ORBIT_FORWARD_RPM,
                                       0.0f,
                                       -RZ_ORBIT_OMEGA_RPM) != HAL_OK)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_MOTOR;
        }
        osDelay(RZ_PERIOD_MS);
    }

    if (RoundPillar_Stop() != HAL_OK)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    status = RoundPillar_WaitSettled(RZ_STOP_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    /* Keep the continuous yaw so the return follows the same orbit backwards. */
    reverse_start_yaw = Jy61P_GetContinuousYaw();
    reverse_target_yaw = reverse_start_yaw + RZ_CCW_REVERSE_DEG;
    current_yaw = reverse_start_yaw;

    while (current_yaw < reverse_target_yaw)
    {
        status = RoundPillar_CheckStopAndImu();
        if (status != ROUND_PILLAR_OK)
        {
            return status;
        }
        if ((uint32_t)(HAL_GetTick() - orbit_start_tick) >=
            RZ_ORBIT_TIMEOUT_MS)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_ORBIT_TIMEOUT;
        }

        current_yaw = Jy61P_GetContinuousYaw();
        if ((grab_count < RZ_GRAB_COUNT) &&
            (MaixCamLink_TakeReply() != 0U))
        {
            handling_start_tick = HAL_GetTick();
            status = RoundPillar_HandleDetectedBall(&grab_count);
            orbit_start_tick += HAL_GetTick() - handling_start_tick;
            if (status != ROUND_PILLAR_OK)
            {
                return status;
            }
            continue;
        }
        if (current_yaw >= reverse_target_yaw)
        {
            break;
        }
        if (MotionControl_SetBodySpeed(-RZ_ORBIT_FORWARD_RPM,
                                       0.0f,
                                       RZ_ORBIT_OMEGA_RPM) != HAL_OK)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_MOTOR;
        }
        osDelay(RZ_PERIOD_MS);
    }

    if (RoundPillar_Stop() != HAL_OK)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    status = RoundPillar_WaitSettled(RZ_STOP_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }
    if (grab_count != RZ_GRAB_COUNT)
    {
        return ROUND_PILLAR_ERROR_MAIX_TIMEOUT;
    }
    MotionControl_ResetHeadingReference();
    MotionControl_State = MOTION_STATUS_FINISHED;
    return ROUND_PILLAR_OK;
}

RoundPillarStatus RoundPillar_Run(void)
{
    MotionControlStatus move_status;
    RoundPillarStatus status;
    ServoActionStatus servo_status;

    if (Jy61P_IsOnline(500U) == 0U)
    {
        (void)RoundPillar_Stop();
        return ROUND_PILLAR_ERROR_IMU;
    }

    MotionControl_ResetHeadingReference();
    status = RoundPillar_Approach();
    if (status != ROUND_PILLAR_OK)
    {
        (void)RoundPillar_Stop();
        return status;
    }

    if (RoundPillar_Stop() != HAL_OK)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    status = RoundPillar_WaitSettled(RZ_STOP_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    /* Keep the RZ-start yaw reference.  This move is still locked to zero. */
    move_status = MotionControl_MovePolarSegmentMm(
        RZ_AFTER_IR_DISTANCE_MM,
        -90.0f,
        0.0f,
        RZ_FINE_APPROACH_RPM,
        0.0f);
    if (move_status >= MOTION_ERROR_IMU_STARTUP)
    {
        return RoundPillar_MapMotionStatus(move_status);
    }
    if (MotionControl_WasStopped() != 0U)
    {
        (void)RoundPillar_Stop();
        return ROUND_PILLAR_CANCELED;
    }

    if (RoundPillar_Stop() != HAL_OK)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    status = RoundPillar_WaitSettled(RZ_STOP_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    ServoAction_SequenceState = SERVO_SEQUENCE_RETURN_RUNNING;
    servo_status = ServoAction_StartGroupNoWait(
        SERVO_ACTION_PILLAR_CAMERA_GROUP,
        1U);
    if (servo_status != SERVO_ACTION_OK)
    {
        ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
        return ROUND_PILLAR_ERROR_SERVO;
    }

    status = RoundPillar_WaitCameraRaise();
    ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    return RoundPillar_OrbitAndGrab();
}
