#include "motion_control.h"
#include "jy61p.h"
#include "mecanum_kinematics.h"
#include "motor_control.h"
#include <math.h>

#define GYRO_STARTUP_TIMEOUT_MS           2000U
#define GYRO_ONLINE_TIMEOUT_MS            500U

#define MOTION_REQUIRE_IMU_AT_STARTUP        1U
#define MOTION_STOP_IF_IMU_LOST              1U

/* F6 speed-mode control: software ramp and command-RPM time integration. */
#define MOTION_CONTROL_PERIOD_MS           20U
#define MOTION_RAMP_TIME_MS               300U
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

#define FORWARD_DISTANCE_GAIN              1.000f
#define LEFT_DISTANCE_GAIN                 1.000f

#define HEADING_KP_RPM_PER_DEG             2.0f
#define HEADING_KD_RPM_PER_DEG             0.15f
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

static float Motion_Absolute(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t Motion_RoundToInt(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

float MotionControl_GetHeadingCorrection(float translation_rpm)
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

static HAL_StatusTypeDef MotionControl_SetBodySpeedWithScale(
    float forward_rpm,
    float left_rpm,
    float omega_rpm,
    float *wheel_scale)
{
    MecanumWheelValues wheel_values;
    MotorWheelSpeedsRpmX10 wheel_speeds;

    MecanumKinematics_Solve(forward_rpm, left_rpm, omega_rpm,
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
    return MotorControl_SetWheelSpeeds(&wheel_speeds);
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
    MotorControl_Init(motor_uart);
    Jy61P_Init(imu_uart);
    previous_heading_error = 0.0f;
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
    float peak_rpm;
    float acceleration_time_ms;
    float deceleration_time_ms;
    float acceleration_distance;
    float deceleration_distance;
    float previous_effective_base_rpm = 0.0f;
    float final_scale = 1.0f;
    uint32_t stage_start;
    uint32_t last_integral_tick;
    uint32_t next_tick;
        uint8_t stage = 0U; /* 0 accelerate, 1 cruise, 2 decelerate. */

    if (!(distance_mm > 0.0f) ||
        ((forward_unit * forward_unit) +
         (left_unit * left_unit) <= 0.0001f) ||
        !(start_rpm >= 0.0f) || !(cruise_rpm > 0.0f) ||
        !(end_rpm >= 0.0f) || (cruise_rpm < start_rpm) ||
        (cruise_rpm < end_rpm))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }

    target_mm = distance_mm;
    peak_rpm = cruise_rpm;
    acceleration_time_ms = (peak_rpm > start_rpm) ?
                           (float)MOTION_RAMP_TIME_MS : 0.0f;
    deceleration_time_ms = (peak_rpm > end_rpm) ?
                           (float)MOTION_RAMP_TIME_MS : 0.0f;
    acceleration_distance = ((start_rpm + peak_rpm) * 0.5f) *
                             wheel_mm_per_rpm_ms * acceleration_time_ms;
    deceleration_distance = ((peak_rpm + end_rpm) * 0.5f) *
                             wheel_mm_per_rpm_ms * deceleration_time_ms;

    /* For short segments, lower the peak while preserving both terminal speeds. */
    if (target_mm < (acceleration_distance + deceleration_distance) &&
        ((acceleration_time_ms + deceleration_time_ms) > 0.0f))
    {
        peak_rpm = ((2.0f * target_mm / wheel_mm_per_rpm_ms) -
                    (start_rpm * acceleration_time_ms) -
                    (end_rpm * deceleration_time_ms)) /
                   (acceleration_time_ms + deceleration_time_ms);
        if (peak_rpm < start_rpm) { peak_rpm = start_rpm; }
        if (peak_rpm < end_rpm) { peak_rpm = end_rpm; }
        if (peak_rpm > cruise_rpm) { peak_rpm = cruise_rpm; }
        acceleration_time_ms = (peak_rpm > start_rpm) ?
                               (float)MOTION_RAMP_TIME_MS : 0.0f;
        deceleration_time_ms = (peak_rpm > end_rpm) ?
                               (float)MOTION_RAMP_TIME_MS : 0.0f;
        acceleration_distance = ((start_rpm + peak_rpm) * 0.5f) *
                                 wheel_mm_per_rpm_ms * acceleration_time_ms;
        deceleration_distance = ((peak_rpm + end_rpm) * 0.5f) *
                                 wheel_mm_per_rpm_ms * deceleration_time_ms;
    }

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

    stage_start = HAL_GetTick();
    last_integral_tick = stage_start;
    next_tick = stage_start;

    while (stage < 3U)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - last_integral_tick;
        uint32_t stage_elapsed_ms = now - stage_start;
        float base_rpm;
        float correction_rpm = 0.0f;
        float wheel_scale = 1.0f;
        uint8_t stop_result;

        stop_result = MotionControl_HandleStopRequest();
        if (stop_result != 0U)
        {
            return MotionControl_State;
        }

        MotionControl_TraveledMm +=
            Motion_Absolute(previous_effective_base_rpm) *
            wheel_mm_per_rpm_ms * (float)elapsed_ms;
        last_integral_tick = now;

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

        if (stage == 0U)
        {
            if ((acceleration_time_ms == 0.0f) ||
                ((float)stage_elapsed_ms >= acceleration_time_ms))
            {
                base_rpm = peak_rpm;
                stage = 1U;
            }
            else
            {
                base_rpm = start_rpm +
                           ((peak_rpm - start_rpm) *
                            ((float)stage_elapsed_ms / acceleration_time_ms));
            }
        }
        else if (stage == 1U)
        {
            base_rpm = peak_rpm;
            if (MotionControl_TraveledMm >=
                (target_mm - deceleration_distance))
            {
                stage = 2U;
                stage_start = now;
            }
        }
        else
        {
            if ((deceleration_time_ms == 0.0f) ||
                ((float)stage_elapsed_ms >= deceleration_time_ms) ||
                (MotionControl_TraveledMm >= target_mm))
            {
                stage = 3U;
                break;
            }
            base_rpm = peak_rpm +
                       ((end_rpm - peak_rpm) *
                        ((float)stage_elapsed_ms / deceleration_time_ms));
        }

        if ((MotionControl_ImuHeadingHoldActive != 0U) &&
            (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U))
        {
            return Motion_SegmentFail(MOTION_ERROR_IMU_LOST);
        }
        if (MotionControl_ImuHeadingHoldActive != 0U)
        {
            correction_rpm = MotionControl_GetHeadingCorrection(base_rpm);
        }

        if (MotionControl_SetBodySpeedWithScale(
                base_rpm * MotionControl_ForwardUnit,
                base_rpm * MotionControl_LeftUnit,
                correction_rpm,
                &wheel_scale) != HAL_OK)
        {
            return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
        }
        previous_effective_base_rpm = base_rpm * wheel_scale;
        MotionControl_BaseRpm = base_rpm;
        MotionControl_WheelScale = wheel_scale;
        MotionControl_EffectiveBaseRpm = previous_effective_base_rpm;
        Motion_WaitControlPeriod(&next_tick);
    }

    if (MotionControl_HandleStopRequest() != 0U)
    {
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
    MotionControl_EffectiveBaseRpm = end_rpm * final_scale;
    if (end_rpm <= 0.0001f)
    {
        MotionControl_State = MOTION_STATUS_FINISHED;
        MotionControl_BaseRpm = 0.0f;
        MotionControl_EffectiveBaseRpm = 0.0f;
    }
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
    return MotionControl_RunPolarSegment(
        corrected_distance, forward_unit, left_unit,
        start_rpm, cruise_rpm, end_rpm, move_status,
        early_stop_check, early_stopped);
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
    }
    HAL_Delay(100U);

    return MotionControl_State;
}
