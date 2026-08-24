#include "motion_control.h"
#include "jy61p.h"
#include "mecanum_kinematics.h"
#include "motor_control.h"

#define DEFAULT_MOVE_DISTANCE_MM          1000
#define DEFAULT_PAUSE_MS                  500U
#define GYRO_STARTUP_TIMEOUT_MS           2000U
#define GYRO_ONLINE_TIMEOUT_MS            500U

#define MOTION_REQUIRE_IMU_AT_STARTUP        1U
#define MOTION_STOP_IF_IMU_LOST              1U

/* F6 speed-mode control: software ramp and command-RPM time integration. */
#define MOTION_CONTROL_PERIOD_MS           20U
#define MOTION_RAMP_TIME_MS               300U
#define MOTION_CRUISE_RPM                100.0f
#define MOTION_PI                          3.1415926f

#define FORWARD_DISTANCE_GAIN              1.000f
#define LEFT_DISTANCE_GAIN                 1.000f
#define FRONT_LEFT_GAIN                    1.000f
#define FRONT_RIGHT_GAIN                   1.000f
#define REAR_LEFT_GAIN                     1.000f
#define REAR_RIGHT_GAIN                    1.000f

#define HEADING_KP_RPM_PER_DEG             2.0f
#define HEADING_KD_RPM_PER_DEG             0.20f
#define HEADING_DEADBAND_DEG                0.15f
#define HEADING_MAX_CORRECTION_RPM         20.0f
#define HEADING_MAX_TRANSLATION_RATIO       0.25f
#define HEADING_CORRECTION_SIGN             1.0f

volatile MotionControlStatus MotionControl_State = MOTION_STATUS_IDLE;
volatile float MotionControl_HeadingErrorDeg = 0.0f;
volatile float MotionControl_HeadingCorrectionRpm = 0.0f;
volatile uint8_t MotionControl_ImuHeadingHoldActive = 0U;
volatile uint32_t MotionControl_PeriodOverrunCount = 0U;

static float previous_heading_error = 0.0f;

static float Motion_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t Motion_RoundToInt(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static float Motion_HeadingCorrectionRpm(float translation_rpm)
{
    float error = -Jy61P_GetContinuousYaw();
    float correction;
    float relative_limit = Motion_Absolute(translation_rpm) *
                           HEADING_MAX_TRANSLATION_RATIO;
    float limit = HEADING_MAX_CORRECTION_RPM;

    if (Motion_Absolute(error) <= HEADING_DEADBAND_DEG)
    {
        error = 0.0f;
    }
    correction = (HEADING_KP_RPM_PER_DEG * error) +
                 (HEADING_KD_RPM_PER_DEG *
                  (error - previous_heading_error));
    correction *= HEADING_CORRECTION_SIGN;
    previous_heading_error = error;

    if (relative_limit < limit) { limit = relative_limit; }
    if (correction > limit) { correction = limit; }
    else if (correction < -limit) { correction = -limit; }

    MotionControl_HeadingErrorDeg = error;
    MotionControl_HeadingCorrectionRpm = correction;
    return correction;
}

static void Motion_ApplyWheelCalibration(MecanumWheelValues *wheels)
{
    wheels->front_left *= FRONT_LEFT_GAIN;
    wheels->front_right *= FRONT_RIGHT_GAIN;
    wheels->rear_left *= REAR_LEFT_GAIN;
    wheels->rear_right *= REAR_RIGHT_GAIN;
}

static HAL_StatusTypeDef Motion_SendChassisSpeed(float forward_rpm,
                                                  float left_rpm,
                                                  float correction_rpm)
{
    MecanumWheelValues wheel_values;
    MotorWheelSpeedsRpmX10 wheel_speeds;

    MecanumKinematics_Solve(forward_rpm, left_rpm, correction_rpm,
                            &wheel_values);
    Motion_ApplyWheelCalibration(&wheel_values);
    MecanumKinematics_Desaturate(&wheel_values,
                                 (float)MOTOR_SPEED_LIMIT_RPM);

    wheel_speeds.front_left = (int16_t)Motion_RoundToInt(
        wheel_values.front_left * (float)MOTOR_SPEED_COMMAND_SCALE);
    wheel_speeds.front_right = (int16_t)Motion_RoundToInt(
        wheel_values.front_right * (float)MOTOR_SPEED_COMMAND_SCALE);
    wheel_speeds.rear_left = (int16_t)Motion_RoundToInt(
        wheel_values.rear_left * (float)MOTOR_SPEED_COMMAND_SCALE);
    wheel_speeds.rear_right = (int16_t)Motion_RoundToInt(
        wheel_values.rear_right * (float)MOTOR_SPEED_COMMAND_SCALE);
    return MotorControl_SetWheelSpeeds(&wheel_speeds);
}

static void Motion_WaitControlPeriod(uint32_t *next_tick)
{
    uint32_t now;
    int32_t remaining;

    *next_tick += MOTION_CONTROL_PERIOD_MS;
    now = HAL_GetTick();
    remaining = (int32_t)(*next_tick - now);
    if (remaining > 0)
    {
        HAL_Delay((uint32_t)remaining);
    }
    else
    {
        MotionControl_PeriodOverrunCount++;
        *next_tick = now;
    }
}

void MotionControl_Init(UART_HandleTypeDef *motor_uart,
                        UART_HandleTypeDef *imu_uart)
{
    MotorControl_Init(motor_uart);
    Jy61P_Init(imu_uart);
    previous_heading_error = 0.0f;
    MotionControl_HeadingErrorDeg = 0.0f;
    MotionControl_HeadingCorrectionRpm = 0.0f;
    MotionControl_ImuHeadingHoldActive = 0U;
    MotionControl_PeriodOverrunCount = 0U;
    MotionControl_State = MOTION_STATUS_IDLE;
}

MotionControlStatus MotionControl_MoveMm(int32_t forward_mm, int32_t left_mm)
{
    float target_mm;
    float forward_unit = 0.0f;
    float left_unit = 0.0f;
    float peak_rpm = MOTION_CRUISE_RPM;
    float wheel_mm_per_rpm_ms;
    float full_ramp_distance;
    float decel_distance;
    float traveled_mm = 0.0f;
    float previous_base_rpm = 0.0f;
    uint32_t stage_start;
    uint32_t last_integral_tick;
    uint32_t next_tick;
    uint8_t stage = 0U; /* 0 accelerate, 1 cruise, 2 decelerate. */

    if ((forward_mm == 0) && (left_mm == 0))
    {
        return MotionControl_State;
    }
    if ((forward_mm != 0) && (left_mm != 0))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }

    if (forward_mm != 0)
    {
        target_mm = (float)((forward_mm < 0) ?
            -(int64_t)forward_mm : forward_mm) * FORWARD_DISTANCE_GAIN;
        forward_unit = (forward_mm < 0) ? -1.0f : 1.0f;
    }
    else
    {
        target_mm = (float)((left_mm < 0) ?
            -(int64_t)left_mm : left_mm) * LEFT_DISTANCE_GAIN;
        left_unit = (left_mm < 0) ? -1.0f : 1.0f;
    }

    wheel_mm_per_rpm_ms = MOTION_PI * (float)MOTOR_WHEEL_DIAMETER_MM /
                          60000.0f;
    full_ramp_distance = peak_rpm * wheel_mm_per_rpm_ms *
                         (float)MOTION_RAMP_TIME_MS;
    if (target_mm < full_ramp_distance)
    {
        peak_rpm = target_mm /
                   (wheel_mm_per_rpm_ms * (float)MOTION_RAMP_TIME_MS);
    }
    decel_distance = peak_rpm * wheel_mm_per_rpm_ms *
                     (float)MOTION_RAMP_TIME_MS * 0.5f;

    stage_start = HAL_GetTick();
    last_integral_tick = stage_start;
    next_tick = stage_start;

    while (stage < 3U)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - last_integral_tick;
        uint32_t stage_elapsed = now - stage_start;
        float base_rpm;
        float correction_rpm = 0.0f;

        traveled_mm += Motion_Absolute(previous_base_rpm) *
                       wheel_mm_per_rpm_ms * (float)elapsed_ms;
        last_integral_tick = now;

        if (stage == 0U)
        {
            if (stage_elapsed >= MOTION_RAMP_TIME_MS)
            {
                base_rpm = peak_rpm;
                stage = 1U;
            }
            else
            {
                base_rpm = peak_rpm * (float)stage_elapsed /
                           (float)MOTION_RAMP_TIME_MS;
            }
        }
        else if (stage == 1U)
        {
            base_rpm = peak_rpm;
            if (traveled_mm >= (target_mm - decel_distance))
            {
                stage = 2U;
                stage_start = now;
            }
        }
        else
        {
            if ((stage_elapsed >= MOTION_RAMP_TIME_MS) ||
                (traveled_mm >= target_mm))
            {
                stage = 3U;
                break;
            }
            base_rpm = peak_rpm *
                       (1.0f - ((float)stage_elapsed /
                                (float)MOTION_RAMP_TIME_MS));
        }

        if ((MotionControl_ImuHeadingHoldActive != 0U) &&
            (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U))
        {
#if MOTION_STOP_IF_IMU_LOST
            (void)Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f);
            MotionControl_State = MOTION_ERROR_IMU_LOST;
            return MotionControl_State;
#else
            MotionControl_ImuHeadingHoldActive = 0U;
#endif
        }
        if (MotionControl_ImuHeadingHoldActive != 0U)
        {
            correction_rpm = Motion_HeadingCorrectionRpm(base_rpm);
        }

        if (Motion_SendChassisSpeed(forward_unit * base_rpm,
                                    left_unit * base_rpm,
                                    correction_rpm) != HAL_OK)
        {
            (void)MotorControl_StopAll();
            MotionControl_State = MOTION_ERROR_MOTOR_UART;
            return MotionControl_State;
        }
        previous_base_rpm = base_rpm;
        Motion_WaitControlPeriod(&next_tick);
    }

    if (Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f) != HAL_OK)
    {
        (void)MotorControl_StopAll();
        MotionControl_State = MOTION_ERROR_MOTOR_UART;
    }
    return MotionControl_State;
}

MotionControlStatus MotionControl_RunDefaultSequence(void)
{
    if (Jy61P_WaitData(GYRO_STARTUP_TIMEOUT_MS) == 0U)
    {
#if MOTION_REQUIRE_IMU_AT_STARTUP
        MotionControl_State = MOTION_ERROR_IMU_STARTUP;
        (void)MotorControl_StopAll();
        return MotionControl_State;
#else
        MotionControl_ImuHeadingHoldActive = 0U;
#endif
    }
    else
    {
        MotionControl_ImuHeadingHoldActive = 1U;
    }

    if (MotorControl_EnableAll() != HAL_OK)
    {
        MotionControl_State = MOTION_ERROR_MOTOR_UART;
        return MotionControl_State;
    }
    HAL_Delay(500U);

    if (MotionControl_ImuHeadingHoldActive != 0U)
    {
        Jy61P_ResetContinuousYaw();
    }
    previous_heading_error = 0.0f;
    HAL_Delay(100U);

    MotionControl_State = MOTION_STATUS_FORWARD;
    if (MotionControl_MoveMm(DEFAULT_MOVE_DISTANCE_MM, 0) >=
        MOTION_ERROR_IMU_STARTUP)
    {
        return MotionControl_State;
    }

    MotionControl_State = MOTION_STATUS_PAUSE;
    HAL_Delay(DEFAULT_PAUSE_MS);

    MotionControl_State = MOTION_STATUS_LEFT;
    if (MotionControl_MoveMm(0, DEFAULT_MOVE_DISTANCE_MM) >=
        MOTION_ERROR_IMU_STARTUP)
    {
        return MotionControl_State;
    }

    if (MotorControl_StopAll() != HAL_OK)
    {
        MotionControl_State = MOTION_ERROR_MOTOR_UART;
        return MotionControl_State;
    }
    MotionControl_State = MOTION_STATUS_FINISHED;
    return MotionControl_State;
}
