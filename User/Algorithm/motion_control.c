#include "motion_control.h"
#include "jy61p.h"
#include "mecanum_kinematics.h"
#include "motor_control.h"
#include <math.h>
#include <float.h>

#define GYRO_STARTUP_TIMEOUT_MS           2000U
#define GYRO_ONLINE_TIMEOUT_MS            500U

#define MOTION_REQUIRE_IMU_AT_STARTUP        1U
#define MOTION_STOP_IF_IMU_LOST              1U

/* F6 speed-mode control: software ramp and command-RPM time integration. */
#define MOTION_CONTROL_PERIOD_MS           20U
#define MOTION_PI                          3.1415926f

/* IMU closed-loop in-place rotation. Positive omega is counter-clockwise. */
#define ROTATE_CRUISE_RPM                  60.0f
#define ROTATE_APPROACH_RPM                15.0f
#define ROTATE_MIN_EFFECTIVE_RPM            8.0f
#define ROTATE_DECEL_START_DEG             30.0f
#define ROTATE_FINE_START_DEG              10.0f
#define ROTATE_TOLERANCE_DEG                0.8f
#define ROTATE_SETTLE_CYCLES                5U
#define ROTATE_RAMP_TIME_MS               250U
#define ROTATE_TIMEOUT_MS                8000U
#define ROTATE_MAX_ANGLE_DEG              360.0f

#define HEADING_KP_RPM_PER_DEG             2.0f
#define HEADING_KD_RPM_PER_DEG_PER_S       0.08f
#define HEADING_RATE_FILTER_MS             40.0f
#define HEADING_RATE_MAX_GAP_MS             200U
#define HEADING_DEADBAND_DEG                0.15f
#define HEADING_MAX_CORRECTION_RPM          8.0f
#define HEADING_MAX_TRANSLATION_RATIO       0.25f
#define HEADING_CORRECTION_SIGN             1.0f

volatile MotionControlStatus MotionControl_State = MOTION_STATUS_IDLE;
volatile uint8_t MotionControl_StopRequested = 0U;
volatile uint8_t MotionControl_StoppedByRequest = 0U;
volatile float MotionControl_HeadingErrorDeg = 0.0f;
volatile float MotionControl_HeadingCorrectionRpm = 0.0f;
volatile uint8_t MotionControl_ImuHeadingHoldActive = 0U;
volatile uint32_t MotionControl_PeriodOverrunCount = 0U;
volatile float MotionControl_TargetAngleDeg = 0.0f;
volatile float MotionControl_ForwardUnit = 0.0f;
volatile float MotionControl_LeftUnit = 0.0f;
volatile float MotionControl_BaseRpm = 0.0f;
volatile float MotionControl_EffectiveBaseRpm = 0.0f;
volatile float MotionControl_LastFrontLeftRpm = 0.0f;
volatile float MotionControl_LastFrontRightRpm = 0.0f;
volatile float MotionControl_LastRearLeftRpm = 0.0f;
volatile float MotionControl_LastRearRightRpm = 0.0f;
volatile float MotionControl_WheelScale = 1.0f;
volatile float MotionControl_TraveledMm = 0.0f;
volatile float MotionControl_TargetDistanceMm = 0.0f;
volatile float MotionControl_RotateTargetDeg = 0.0f;
volatile float MotionControl_RotateCurrentDeg = 0.0f;
volatile float MotionControl_RotateErrorDeg = 0.0f;
volatile float MotionControl_RotateCommandRpm = 0.0f;
volatile uint8_t MotionControl_RotateSettleCount = 0U;
volatile uint32_t MotionControl_RotateElapsedMs = 0U;

static float previous_heading_error = 0.0f;
static float heading_rate_deg_s;
static uint32_t heading_sample_tick;
static uint8_t heading_sample_valid;

/* Applied command velocity, not encoder feedback. Updated only on successful sync. */
static float applied_forward_rpm;
static float applied_left_rpm;
static float applied_omega_rpm;
static uint8_t distance_tracking;
static uint32_t distance_tick;

static void Motion_IntegrateUntil(uint32_t tick)
{
    if (distance_tracking != 0U)
    {
        float along_rpm = applied_forward_rpm * MotionControl_ForwardUnit +
                          applied_left_rpm * MotionControl_LeftUnit;
        MotionControl_TraveledMm += along_rpm *
            (MOTION_PI * MOTOR_WHEEL_DIAMETER_MM / 60000.0f) *
            (float)(tick - distance_tick);
        distance_tick = tick;
    }
}

static float Motion_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t Motion_RoundToInt(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static float Motion_Approach(float value, float target, float step)
{
    if (value < target) { return (value + step < target) ? value + step : target; }
    return (value - step > target) ? value - step : target;
}

static float Motion_HeadingCorrection(float translation_rpm, float minimum_limit)
{
    float error;
    uint32_t sample_tick;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    error = -Jy61P_GetContinuousYaw();
    sample_tick = Jy61P_GetLastTick();
    if (primask == 0U) { __enable_irq(); }
    float correction;
    float relative_limit = Motion_Absolute(translation_rpm) *
                           HEADING_MAX_TRANSLATION_RATIO;
    float limit = HEADING_MAX_CORRECTION_RPM;

    /* Differentiate only fresh samples, using their actual arrival interval.
       Keep rate damping between samples; repeated control calls are not data. */
    if ((heading_sample_valid == 0U) || (sample_tick != heading_sample_tick))
    {
        uint32_t dt_ms = sample_tick - heading_sample_tick;
        if ((heading_sample_valid != 0U) && (dt_ms > 0U) &&
            (dt_ms <= HEADING_RATE_MAX_GAP_MS))
        {
            float rate = (error - previous_heading_error) * 1000.0f / (float)dt_ms;
            float alpha = (float)dt_ms / (HEADING_RATE_FILTER_MS + (float)dt_ms);
            heading_rate_deg_s += alpha * (rate - heading_rate_deg_s);
        }
        else { heading_rate_deg_s = 0.0f; }
        previous_heading_error = error;
        heading_sample_tick = sample_tick;
        heading_sample_valid = 1U;
    }
    if ((uint32_t)(HAL_GetTick() - sample_tick) > HEADING_RATE_MAX_GAP_MS)
    {
        heading_rate_deg_s = 0.0f;
    }
    if (Motion_Absolute(error) <= HEADING_DEADBAND_DEG) { error = 0.0f; }
    correction = HEADING_KP_RPM_PER_DEG * error +
                 HEADING_KD_RPM_PER_DEG_PER_S * heading_rate_deg_s;
    correction *= HEADING_CORRECTION_SIGN;

    if (relative_limit < minimum_limit) { relative_limit = minimum_limit; }
    if (relative_limit < limit) { limit = relative_limit; }
    if (correction > limit) { correction = limit; }
    else if (correction < -limit) { correction = -limit; }

    MotionControl_HeadingErrorDeg = error;
    MotionControl_HeadingCorrectionRpm = correction;
    return correction;
}

float MotionControl_GetHeadingCorrection(float translation_rpm)
{
    /* Preserve the existing limit for gray alignment and pillar control. */
    return Motion_HeadingCorrection(translation_rpm, 0.0f);
}

static HAL_StatusTypeDef MotionControl_SetBodySpeedWithScale(
    float forward_rpm,
    float left_rpm,
    float omega_rpm,
    float *wheel_scale)
{
    MecanumWheelValues wheel_values;
    MotorWheelSpeedsRpmX10 wheel_speeds;
    HAL_StatusTypeDef status;

    if ((forward_rpm == 0.0f) && (left_rpm == 0.0f) && (omega_rpm == 0.0f))
    {
        HAL_StatusTypeDef stopped;
        status = MotorControl_StopAll();
        stopped = status;
        if (stopped != HAL_OK) { stopped = MotorControl_StopAll(); }
        if (stopped == HAL_OK)
        {
            Motion_IntegrateUntil(MotorControl_LastSpeedSyncTick);
            applied_forward_rpm = applied_left_rpm = applied_omega_rpm = 0.0f;
            MotionControl_LastFrontLeftRpm = MotionControl_LastFrontRightRpm = 0.0f;
            MotionControl_LastRearLeftRpm = MotionControl_LastRearRightRpm = 0.0f;
            MotionControl_BaseRpm = MotionControl_EffectiveBaseRpm = 0.0f;
            MotionControl_HeadingCorrectionRpm = 0.0f;
            MotionControl_WheelScale = 1.0f;
            if (wheel_scale != 0) { *wheel_scale = 1.0f; }
        }
        /* A recovered transmission still reports the original fault. */
        return status;
    }

    MecanumKinematics_Solve(forward_rpm, left_rpm,
                            omega_rpm * MOTION_OMEGA_TO_WHEEL_SIGN,
                            &wheel_values);
    if (wheel_scale != 0)
    {
        *wheel_scale = MecanumKinematics_DesaturateWithScale(
            &wheel_values, (float)MOTOR_SPEED_LIMIT_RPM);
    }
    else
    {
        (void)MecanumKinematics_DesaturateWithScale(
            &wheel_values, (float)MOTOR_SPEED_LIMIT_RPM);
    }

    MotionControl_LastFrontLeftRpm = wheel_values.front_left;
    MotionControl_LastFrontRightRpm = wheel_values.front_right;
    MotionControl_LastRearLeftRpm = wheel_values.rear_left;
    MotionControl_LastRearRightRpm = wheel_values.rear_right;

    wheel_speeds.front_left = (int16_t)Motion_RoundToInt(
        wheel_values.front_left * (float)MOTOR_SPEED_COMMAND_SCALE);
    wheel_speeds.front_right = (int16_t)Motion_RoundToInt(
        wheel_values.front_right * (float)MOTOR_SPEED_COMMAND_SCALE);
    wheel_speeds.rear_left = (int16_t)Motion_RoundToInt(
        wheel_values.rear_left * (float)MOTOR_SPEED_COMMAND_SCALE);
    wheel_speeds.rear_right = (int16_t)Motion_RoundToInt(
        wheel_values.rear_right * (float)MOTOR_SPEED_COMMAND_SCALE);
    status = MotorControl_SetWheelSpeeds(&wheel_speeds);
    if (status == HAL_OK)
    {
        /* Old command remains active while all four wheel frames are queued. */
        Motion_IntegrateUntil(MotorControl_LastSpeedSyncTick);
        applied_forward_rpm = (wheel_speeds.front_left + wheel_speeds.front_right +
            wheel_speeds.rear_left + wheel_speeds.rear_right) /
            (4.0f * MOTOR_SPEED_COMMAND_SCALE);
        applied_left_rpm = (-wheel_speeds.front_left + wheel_speeds.front_right +
            wheel_speeds.rear_left - wheel_speeds.rear_right) /
            (4.0f * MOTOR_SPEED_COMMAND_SCALE);
        applied_omega_rpm = (-wheel_speeds.front_left + wheel_speeds.front_right -
            wheel_speeds.rear_left + wheel_speeds.rear_right) /
            (4.0f * MOTOR_SPEED_COMMAND_SCALE * MOTION_OMEGA_TO_WHEEL_SIGN);
    }
    return status;
}

HAL_StatusTypeDef MotionControl_SetBodySpeed(float forward_rpm,
                                              float left_rpm,
                                              float omega_rpm)
{
    return MotionControl_SetBodySpeedWithScale(
        forward_rpm, left_rpm, omega_rpm, 0);
}

void MotionControl_ResetHeadingReference(void)
{
    Jy61P_ResetContinuousYaw();
    previous_heading_error = 0.0f;
    heading_rate_deg_s = 0.0f;
    heading_sample_valid = 0U;
    MotionControl_HeadingErrorDeg = 0.0f;
    MotionControl_HeadingCorrectionRpm = 0.0f;
}

/* Returns 0 when no request is pending, 1 when stopped, and 2 on UART error. */
static uint8_t MotionControl_HandleStopRequest(void)
{
    if (MotionControl_StopRequested == 0U)
    {
        return 0U;
    }

    MotionControl_StopRequested = 0U;
    if (MotionControl_SetBodySpeedWithScale(0.0f, 0.0f, 0.0f, 0) != HAL_OK)
    {
        (void)MotorControl_StopAll();
        MotionControl_State = MOTION_ERROR_MOTOR_UART;
        return 2U;
    }

    MotionControl_StoppedByRequest = 1U;
    MotionControl_State = MOTION_STATUS_FINISHED;
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    return 1U;
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
    distance_tracking = 0U;
    applied_forward_rpm = 0.0f;
    applied_left_rpm = 0.0f;
    applied_omega_rpm = 0.0f;
    MotorControl_Init(motor_uart);
    Jy61P_Init(imu_uart);
    previous_heading_error = 0.0f;
    heading_rate_deg_s = 0.0f;
    heading_sample_valid = 0U;
    MotionControl_HeadingErrorDeg = 0.0f;
    MotionControl_HeadingCorrectionRpm = 0.0f;
    MotionControl_ImuHeadingHoldActive = 0U;
    MotionControl_PeriodOverrunCount = 0U;
    MotionControl_TargetAngleDeg = 0.0f;
    MotionControl_ForwardUnit = 0.0f;
    MotionControl_LeftUnit = 0.0f;
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    MotionControl_LastFrontLeftRpm = 0.0f;
    MotionControl_LastFrontRightRpm = 0.0f;
    MotionControl_LastRearLeftRpm = 0.0f;
    MotionControl_LastRearRightRpm = 0.0f;
    MotionControl_WheelScale = 1.0f;
    MotionControl_TraveledMm = 0.0f;
    MotionControl_TargetDistanceMm = 0.0f;
    MotionControl_RotateTargetDeg = 0.0f;
    MotionControl_RotateCurrentDeg = 0.0f;
    MotionControl_RotateErrorDeg = 0.0f;
    MotionControl_RotateCommandRpm = 0.0f;
    MotionControl_RotateSettleCount = 0U;
    MotionControl_RotateElapsedMs = 0U;
    MotionControl_State = MOTION_STATUS_IDLE;
    MotionControl_StopRequested = 0U;
    MotionControl_StoppedByRequest = 0U;
}

void MotionControl_RequestStop(void)
{
    MotionControl_StopRequested = 1U;
}

void MotionControl_ClearStopRequest(void)
{
    MotionControl_StopRequested = 0U;
    MotionControl_StoppedByRequest = 0U;
}

uint8_t MotionControl_WasStopped(void)
{
    return MotionControl_StoppedByRequest;
}

static uint8_t Motion_SegmentAngleValid(float angle_deg)
{
    return ((angle_deg >= -180.0f) && (angle_deg <= 180.0f)) ? 1U : 0U;
}

static MotionControlStatus Motion_SegmentFail(MotionControlStatus error)
{
    (void)MotionControl_SetBodySpeedWithScale(0.0f, 0.0f, 0.0f, 0);
    (void)MotorControl_StopAll();
    MotionControl_State = error;
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    return MotionControl_State;
}

/* In RPM*ms units: reserve the slow tail before solving the fast brake phase. */
static float Motion_BrakingSpeed(float remaining_rpm_ms, float end_rpm,
                                 float horizon_ms)
{
    float decel = MOTION_DECELERATION_RPM_PER_S / 1000.0f;
    float terminal_squared = end_rpm * end_rpm;
    if (end_rpm <= 0.0001f)
    {
        float tail_decel = MOTION_FINAL_DECEL_RPM_PER_S / 1000.0f;
        float tail_speed = MOTION_FINAL_APPROACH_RPM;
        float tail_distance = tail_speed * tail_speed / (2.0f * tail_decel);
        if (remaining_rpm_ms > tail_distance + tail_speed * horizon_ms)
        {
            remaining_rpm_ms -= tail_distance;
            terminal_squared = tail_speed * tail_speed;
        }
        else { decel = tail_decel; }
    }
    return sqrtf(decel * decel * horizon_ms * horizon_ms + terminal_squared +
                 2.0f * decel * remaining_rpm_ms) - decel * horizon_ms;
}

static MotionControlStatus MotionControl_RunPolarSegment(
    float distance_mm,
    float forward_unit,
    float left_unit,
    float start_rpm,
    float cruise_rpm,
    float end_rpm,
    MotionControlStatus move_status,
    MotionControlEarlyStopCheck early_stop_check,
    uint8_t *early_stopped)
{
    const float wheel_mm_per_rpm_ms =
        MOTION_PI * (float)MOTOR_WHEEL_DIAMETER_MM / 60000.0f;
    float target_mm;
    float base_rpm = start_rpm;
    float final_scale = 1.0f;
    float send_time_ms = 20.0f;
    float correction_rpm = applied_omega_rpm;
    uint32_t last_control_tick;
    uint32_t next_tick;

    if (!(distance_mm > 0.0f) ||
        ((forward_unit * forward_unit) +
         (left_unit * left_unit) <= 0.0001f) ||
        !(start_rpm >= 0.0f) || !(cruise_rpm >= 0.1f) ||
        !(cruise_rpm <= FLT_MAX) ||
        !(start_rpm <= MOTOR_SPEED_LIMIT_RPM) ||
        !(end_rpm <= MOTOR_SPEED_LIMIT_RPM) ||
        !(end_rpm >= 0.0f) || (cruise_rpm < start_rpm) ||
        (cruise_rpm < end_rpm))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }

    target_mm = distance_mm;
    MotionControl_State = move_status;
    MotionControl_TargetAngleDeg = atan2f(left_unit, forward_unit) *
                                   180.0f / MOTION_PI;
    MotionControl_ForwardUnit = forward_unit;
    MotionControl_LeftUnit = left_unit;
    MotionControl_BaseRpm = start_rpm;
    MotionControl_EffectiveBaseRpm = start_rpm;
    MotionControl_WheelScale = 1.0f;
    MotionControl_TraveledMm = 0.0f;
    MotionControl_TargetDistanceMm = target_mm;

    last_control_tick = HAL_GetTick();
    next_tick = last_control_tick;
    distance_tick = last_control_tick;
    distance_tracking = 1U;

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - last_control_tick;
        uint32_t send_start;
        float wheel_scale = 1.0f;
        float remaining_mm;
        float brake_limit;
        float horizon_ms;
        uint8_t stop_result;

        Motion_IntegrateUntil(now);
        last_control_tick = now;
        stop_result = MotionControl_HandleStopRequest();
        if (stop_result != 0U) { return MotionControl_State; }

        /* Application callbacks run only after the normal STOP check. */
        if ((early_stop_check != 0) && (early_stop_check() != 0U))
        {
            if (MotionControl_SetBodySpeedWithScale(
                    0.0f, 0.0f, 0.0f, 0) != HAL_OK)
            {
                (void)MotorControl_StopAll();
                MotionControl_State = MOTION_ERROR_MOTOR_UART;
                MotionControl_BaseRpm = 0.0f;
                MotionControl_EffectiveBaseRpm = 0.0f;
                return MotionControl_State;
            }
            if (early_stopped != 0)
            {
                *early_stopped = 1U;
            }
            /* This was an application event, not a user STOP request. */
            MotionControl_State = MOTION_STATUS_FINISHED;
            MotionControl_BaseRpm = 0.0f;
            MotionControl_EffectiveBaseRpm = 0.0f;
            return MotionControl_State;
        }

        remaining_mm = target_mm - MotionControl_TraveledMm;
        if (remaining_mm <= MOTION_DISTANCE_TOLERANCE_MM) { break; }

        /* Brake from remaining distance, including one control/transport interval.
           This remains valid when wheel saturation stretches the movement time. */
        horizon_ms = (send_time_ms > MOTION_CONTROL_PERIOD_MS) ?
                      send_time_ms : (float)MOTION_CONTROL_PERIOD_MS;
        horizon_ms += send_time_ms;
        brake_limit = Motion_BrakingSpeed(remaining_mm / wheel_mm_per_rpm_ms,
                                          end_rpm, horizon_ms);
        if (brake_limit < end_rpm) { brake_limit = end_rpm; }
        base_rpm += MOTION_ACCELERATION_RPM_PER_S * (float)elapsed_ms / 1000.0f;
        if (base_rpm > cruise_rpm) { base_rpm = cruise_rpm; }
        if (base_rpm > brake_limit) { base_rpm = brake_limit; }

        if ((MotionControl_ImuHeadingHoldActive != 0U) &&
            (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U))
        {
            return Motion_SegmentFail(MOTION_ERROR_IMU_LOST);
        }
        if (MotionControl_ImuHeadingHoldActive != 0U)
        {
            float desired = MotionControl_GetHeadingCorrection(base_rpm);
            /* Release obsolete correction immediately; slew only its buildup. */
            if (correction_rpm * desired <= 0.0f) { correction_rpm = 0.0f; }
            else if (Motion_Absolute(desired) < Motion_Absolute(correction_rpm))
            { correction_rpm = desired; }
            correction_rpm = Motion_Approach(correction_rpm, desired,
                MOTION_HEADING_SLEW_RPM_PER_S * (float)elapsed_ms / 1000.0f);
            /* Slew limiting must not retain a large correction as speed falls. */
            float limit = base_rpm * HEADING_MAX_TRANSLATION_RATIO;
            if (correction_rpm > limit) { correction_rpm = limit; }
            if (correction_rpm < -limit) { correction_rpm = -limit; }
            MotionControl_HeadingCorrectionRpm = correction_rpm;
        }

        send_start = HAL_GetTick();
        if (MotionControl_SetBodySpeedWithScale(
                base_rpm * MotionControl_ForwardUnit,
                base_rpm * MotionControl_LeftUnit,
                correction_rpm,
                &wheel_scale) != HAL_OK)
        {
            return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
        }
        send_time_ms = (float)(HAL_GetTick() - send_start);
        MotionControl_BaseRpm = base_rpm;
        MotionControl_WheelScale = wheel_scale;
        MotionControl_EffectiveBaseRpm = applied_forward_rpm * forward_unit +
                                         applied_left_rpm * left_unit;
        Motion_WaitControlPeriod(&next_tick);
    }

    if (MotionControl_HandleStopRequest() != 0U)
    {
        return MotionControl_State;
    }

    if (end_rpm <= 0.0001f)
    {
        if (MotionControl_SetBodySpeedWithScale(0.0f, 0.0f, 0.0f, 0) != HAL_OK)
        {
            return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
        }
        MotionControl_State = MOTION_STATUS_FINISHED;
        return MotionControl_State;
    }

    /* Publish the terminal speed without inserting a zero-speed gap. */
    {
        float final_correction_rpm = 0.0f;

        if ((MotionControl_ImuHeadingHoldActive != 0U) &&
            (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U))
        {
            return Motion_SegmentFail(MOTION_ERROR_IMU_LOST);
        }
        if (MotionControl_ImuHeadingHoldActive != 0U)
        {
            final_correction_rpm = MotionControl_GetHeadingCorrection(end_rpm);
        }
        if (MotionControl_SetBodySpeedWithScale(
                end_rpm * MotionControl_ForwardUnit,
                end_rpm * MotionControl_LeftUnit,
                final_correction_rpm,
                &final_scale) != HAL_OK)
        {
            return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
        }
    }

    MotionControl_BaseRpm = end_rpm;
    MotionControl_WheelScale = final_scale;
    MotionControl_EffectiveBaseRpm = applied_forward_rpm * forward_unit +
                                     applied_left_rpm * left_unit;
    return MotionControl_State;
}

static void Motion_ApplyLateralCompensation(float *forward_unit,
                                             float *left_unit)
{
    float magnitude;

    if ((Motion_Absolute(*forward_unit) > 0.0001f) ||
        (Motion_Absolute(*left_unit) <= 0.0001f) ||
        (Motion_Absolute(LATERAL_FORWARD_COMPENSATION) <= 0.0001f))
    {
        return;
    }

    *forward_unit = LATERAL_FORWARD_COMPENSATION * (*left_unit);
    magnitude = sqrtf((*forward_unit * *forward_unit) +
                      (*left_unit * *left_unit));
    if (magnitude > 0.0001f)
    {
        *forward_unit /= magnitude;
        *left_unit /= magnitude;
    }
}

MotionControlStatus MotionControl_MovePolarSegmentMmUntil(
    uint32_t distance_mm,
    float angle_deg,
    float start_rpm,
    float cruise_rpm,
    float end_rpm,
    MotionControlEarlyStopCheck early_stop_check,
    uint8_t *early_stopped)
{
    float radians;
    float corrected_forward;
    float corrected_left;
    float corrected_distance;
    float forward_unit;
    float left_unit;
    MotionControlStatus move_status;

    if (early_stopped != 0)
    {
        *early_stopped = 0U;
    }

    if (Motion_SegmentAngleValid(angle_deg) == 0U)
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    radians = angle_deg * MOTION_PI / 180.0f;
    forward_unit = cosf(radians);
    left_unit = sinf(radians);
    Motion_ApplyLateralCompensation(&forward_unit, &left_unit);
    corrected_forward = forward_unit * (float)distance_mm *
                        FORWARD_DISTANCE_GAIN;
    corrected_left = left_unit * (float)distance_mm * LEFT_DISTANCE_GAIN;
    corrected_distance = sqrtf((corrected_forward * corrected_forward) +
                               (corrected_left * corrected_left));
    if (corrected_distance <= 0.0f)
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    forward_unit = corrected_forward / corrected_distance;
    left_unit = corrected_left / corrected_distance;
    move_status = ((Motion_Absolute(forward_unit) > 0.0001f) &&
                   (Motion_Absolute(left_unit) > 0.0001f)) ?
                  MOTION_STATUS_DIAGONAL : MOTION_STATUS_POLAR_MOVE;
    move_status = MotionControl_RunPolarSegment(
        corrected_distance, forward_unit, left_unit,
        start_rpm, cruise_rpm, end_rpm, move_status,
        early_stop_check, early_stopped);
    Motion_IntegrateUntil(HAL_GetTick());
    distance_tracking = 0U;
    return move_status;
}

MotionControlStatus MotionControl_MovePolarSegmentMm(
    uint32_t distance_mm,
    float angle_deg,
    float start_rpm,
    float cruise_rpm,
    float end_rpm)
{
    return MotionControl_MovePolarSegmentMmUntil(
        distance_mm, angle_deg, start_rpm, cruise_rpm, end_rpm,
        0, 0);
}

MotionControlStatus MotionControl_RotateDeg(float angle_deg)
{
    uint32_t start_tick;
    uint32_t next_tick;

    if ((Motion_Absolute(angle_deg) <= 0.0f) ||
        (Motion_Absolute(angle_deg) > ROTATE_MAX_ANGLE_DEG))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    if (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U)
    {
        MotionControl_State = MOTION_ERROR_IMU_LOST;
        return MotionControl_State;
    }

    /* Each rotation is measured relative to the heading at its start. */
    MotionControl_State = MOTION_STATUS_ROTATING;
    MotionControl_RotateTargetDeg = angle_deg;
    MotionControl_RotateCurrentDeg = 0.0f;
    MotionControl_RotateErrorDeg = angle_deg;
    MotionControl_RotateCommandRpm = 0.0f;
    MotionControl_RotateSettleCount = 0U;
    MotionControl_RotateElapsedMs = 0U;
    MotionControl_ResetHeadingReference();
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    MotionControl_WheelScale = 1.0f;

    start_tick = HAL_GetTick();
    next_tick = start_tick;
    for (;;)
    {
        uint32_t now = HAL_GetTick();
        float error_deg;
        float absolute_error_deg;
        float magnitude_rpm;
        float ramp_limit_rpm;
        float wheel_scale = 1.0f;
        uint8_t stop_result;

        MotionControl_RotateElapsedMs = now - start_tick;
        stop_result = MotionControl_HandleStopRequest();
        if (stop_result != 0U)
        {
            if (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) != 0U)
            {
                MotionControl_ResetHeadingReference();
            }
            MotionControl_RotateTargetDeg = 0.0f;
            MotionControl_RotateCurrentDeg = 0.0f;
            MotionControl_RotateErrorDeg = 0.0f;
            MotionControl_RotateCommandRpm = 0.0f;
            MotionControl_RotateSettleCount = 0U;
            MotionControl_RotateElapsedMs = 0U;
            return MotionControl_State;
        }
        if (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U)
        {
            MotionControl_RotateCommandRpm = 0.0f;
            return Motion_SegmentFail(MOTION_ERROR_IMU_LOST);
        }
        if (MotionControl_RotateElapsedMs >= ROTATE_TIMEOUT_MS)
        {
            MotionControl_RotateCommandRpm = 0.0f;
            MotionControl_State = Motion_SegmentFail(MOTION_ERROR_ROTATE_TIMEOUT);
            if (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) != 0U)
            {
                MotionControl_ResetHeadingReference();
            }
            MotionControl_RotateTargetDeg = 0.0f;
            MotionControl_RotateCurrentDeg = 0.0f;
            MotionControl_RotateErrorDeg = 0.0f;
            MotionControl_RotateCommandRpm = 0.0f;
            MotionControl_RotateSettleCount = 0U;
            MotionControl_RotateElapsedMs = 0U;
            return MotionControl_State;
        }

        MotionControl_RotateCurrentDeg = Jy61P_GetContinuousYaw();
        error_deg = MotionControl_RotateTargetDeg -
                    MotionControl_RotateCurrentDeg;
        absolute_error_deg = Motion_Absolute(error_deg);
        MotionControl_RotateErrorDeg = error_deg;

        if (absolute_error_deg <= ROTATE_TOLERANCE_DEG)
        {
            MotionControl_RotateCommandRpm = 0.0f;
            if (MotionControl_SetBodySpeedWithScale(0.0f, 0.0f, 0.0f,
                                                    &wheel_scale) != HAL_OK)
            {
                return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
            }
            MotionControl_WheelScale = wheel_scale;
            MotionControl_RotateSettleCount++;
            if (MotionControl_RotateSettleCount >= ROTATE_SETTLE_CYCLES)
            {
                /* The settled heading becomes the reference for the next move. */
                MotionControl_ResetHeadingReference();
                MotionControl_RotateTargetDeg = 0.0f;
                MotionControl_RotateCurrentDeg = 0.0f;
                MotionControl_RotateErrorDeg = 0.0f;
                MotionControl_RotateCommandRpm = 0.0f;
                MotionControl_RotateSettleCount = 0U;
                MotionControl_RotateElapsedMs = 0U;
                MotionControl_State = MOTION_STATUS_FINISHED;
                return MotionControl_State;
            }
        }
        else
        {
            MotionControl_RotateSettleCount = 0U;
            if (absolute_error_deg >= ROTATE_DECEL_START_DEG)
            {
                magnitude_rpm = ROTATE_CRUISE_RPM;
            }
            else if (absolute_error_deg >= ROTATE_FINE_START_DEG)
            {
                magnitude_rpm = ROTATE_APPROACH_RPM +
                    ((ROTATE_CRUISE_RPM - ROTATE_APPROACH_RPM) *
                     ((absolute_error_deg - ROTATE_FINE_START_DEG) /
                      (ROTATE_DECEL_START_DEG - ROTATE_FINE_START_DEG)));
            }
            else
            {
                magnitude_rpm = ROTATE_APPROACH_RPM *
                                (absolute_error_deg / ROTATE_FINE_START_DEG);
            }

            if (MotionControl_RotateElapsedMs >= ROTATE_RAMP_TIME_MS)
            {
                ramp_limit_rpm = ROTATE_CRUISE_RPM;
            }
            else
            {
                ramp_limit_rpm = ROTATE_CRUISE_RPM *
                    ((float)MotionControl_RotateElapsedMs /
                     (float)ROTATE_RAMP_TIME_MS);
            }
            if (magnitude_rpm > ramp_limit_rpm)
            {
                magnitude_rpm = ramp_limit_rpm;
            }
            if ((ramp_limit_rpm >= ROTATE_MIN_EFFECTIVE_RPM) &&
                (magnitude_rpm < ROTATE_MIN_EFFECTIVE_RPM))
            {
                magnitude_rpm = ROTATE_MIN_EFFECTIVE_RPM;
            }

            MotionControl_RotateCommandRpm = (error_deg > 0.0f) ?
                                              magnitude_rpm : -magnitude_rpm;
            if (MotionControl_SetBodySpeedWithScale(
                    0.0f, 0.0f, MotionControl_RotateCommandRpm,
                    &wheel_scale) != HAL_OK)
            {
                MotionControl_RotateCommandRpm = 0.0f;
                return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
            }
            MotionControl_WheelScale = wheel_scale;
        }

        Motion_WaitControlPeriod(&next_tick);
    }
}

MotionControlStatus MotionControl_PrepareForMove(void)
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
        MotionControl_ResetHeadingReference();
    }
    else
    {
        previous_heading_error = 0.0f;
    heading_rate_deg_s = 0.0f;
    heading_sample_valid = 0U;
    }
    HAL_Delay(100U);

    return MotionControl_State;
}
