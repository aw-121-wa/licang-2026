#include "round_pillar.h"
#include "cmsis_os.h"
#include "jy61p.h"
#include "mecanum_kinematics.h"
#include "motor_control.h"
#include "motion_control.h"

static uint8_t RoundPillar_IrDetected(void)
{
    return (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) ==
            RZ_IR_DETECTED_LEVEL) ? 1U : 0U;
}

static HAL_StatusTypeDef RoundPillar_SendBodySpeed(float forward,
                                                    float left,
                                                    float counter_clockwise)
{
    MecanumWheelValues wheels;
    MotorWheelSpeedsRpmX10 speeds;

    MecanumKinematics_Solve(forward, left, counter_clockwise, &wheels);
    MecanumKinematics_DesaturateWithScale(&wheels, MOTOR_SPEED_LIMIT_RPM);
    speeds.front_left = (int16_t)(wheels.front_left *
                                  MOTOR_SPEED_COMMAND_SCALE);
    speeds.front_right = (int16_t)(wheels.front_right *
                                   MOTOR_SPEED_COMMAND_SCALE);
    speeds.rear_left = (int16_t)(wheels.rear_left *
                                 MOTOR_SPEED_COMMAND_SCALE);
    speeds.rear_right = (int16_t)(wheels.rear_right *
                                  MOTOR_SPEED_COMMAND_SCALE);
    return MotorControl_SetWheelSpeeds(&speeds);
}

static HAL_StatusTypeDef RoundPillar_Stop(void)
{
    return MotorControl_StopAll();
}

static float RoundPillar_HeadingCorrection(float locked_yaw,
                                            float *previous_error)
{
    float current_yaw = Jy61P_GetContinuousYaw();
    float error = locked_yaw - current_yaw;
    float correction;

    correction = (RZ_HEADING_KP * error) +
                 (RZ_HEADING_KD * (error - *previous_error));
    *previous_error = error;

    if ((error <= RZ_HEADING_DEADBAND_DEG) &&
        (error >= -RZ_HEADING_DEADBAND_DEG))
    {
        correction = 0.0f;
    }
    else if (correction > RZ_HEADING_MAX_RPM)
    {
        correction = RZ_HEADING_MAX_RPM;
    }
    else if (correction < -RZ_HEADING_MAX_RPM)
    {
        correction = -RZ_HEADING_MAX_RPM;
    }

    return correction;
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

static RoundPillarStatus RoundPillar_Approach(float locked_yaw)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t ir_detect_since = 0U;
    float previous_error = 0.0f;
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

        correction = RoundPillar_HeadingCorrection(locked_yaw,
                                                    &previous_error);
        if (RoundPillar_SendBodySpeed(0.0f,
                                      -RZ_APPROACH_RPM,
                                      correction) != HAL_OK)
        {
            (void)RoundPillar_Stop();
            return ROUND_PILLAR_ERROR_MOTOR;
        }
        osDelay(RZ_PERIOD_MS);
    }
}

static RoundPillarStatus RoundPillar_Orbit(void)
{
    uint32_t orbit_start_tick;
    float reverse_start_yaw;
    float reverse_target_yaw;
    RoundPillarStatus status;

    Jy61P_ResetContinuousYaw();
    orbit_start_tick = HAL_GetTick();
    MotionControl_State = MOTION_STATUS_ROTATING;

    for (;;)
    {
        float current_yaw;

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
        if (current_yaw <= RZ_CW_TARGET_DEG)
        {
            break;
        }
        if (RoundPillar_SendBodySpeed(-RZ_ORBIT_FORWARD_RPM,
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
    status = RoundPillar_WaitSettled(RZ_DIRECTION_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    /* Do not reset here.  The reverse target is measured from the actual
     * settled yaw so inertia cannot turn the requested 90 degrees into 93. */
    reverse_start_yaw = Jy61P_GetContinuousYaw();
    reverse_target_yaw = reverse_start_yaw + RZ_CCW_REVERSE_DEG;

    for (;;)
    {
        float current_yaw;

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
        if (current_yaw >= reverse_target_yaw)
        {
            break;
        }
        if (RoundPillar_SendBodySpeed(RZ_ORBIT_FORWARD_RPM,
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
    status = RoundPillar_WaitSettled(RZ_DIRECTION_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }
    Jy61P_ResetContinuousYaw();
    MotionControl_State = MOTION_STATUS_FINISHED;
    return ROUND_PILLAR_OK;
}

RoundPillarStatus RoundPillar_Run(void)
{
    MotionControlStatus move_status;
    RoundPillarStatus status;
    const float locked_yaw = 0.0f;

    if (Jy61P_IsOnline(500U) == 0U)
    {
        (void)RoundPillar_Stop();
        return ROUND_PILLAR_ERROR_IMU;
    }

    Jy61P_ResetContinuousYaw();
    status = RoundPillar_Approach(locked_yaw);
    if (status != ROUND_PILLAR_OK)
    {
        (void)RoundPillar_Stop();
        return status;
    }

    if (RoundPillar_Stop() != HAL_OK)
    {
        return ROUND_PILLAR_ERROR_MOTOR;
    }
    status = RoundPillar_WaitSettled(RZ_DIRECTION_SETTLE_MS);
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
    status = RoundPillar_WaitSettled(RZ_DIRECTION_SETTLE_MS);
    if (status != ROUND_PILLAR_OK)
    {
        return status;
    }

    return RoundPillar_Orbit();
}
