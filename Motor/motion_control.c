#include "motion_control.h"
#include "jy61p.h"
#include "mecanum_kinematics.h"
#include "motor_control.h"
#include <math.h>

#define DEFAULT_MOVE_DISTANCE_MM          1000
#define DEFAULT_PAUSE_MS                  500U
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
                                                  float correction_rpm,
                                                  float *wheel_scale)
{
    MecanumWheelValues wheel_values;
    MotorWheelSpeedsRpmX10 wheel_speeds;

    MecanumKinematics_Solve(forward_rpm, left_rpm, correction_rpm,
                            &wheel_values);
    Motion_ApplyWheelCalibration(&wheel_values);
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

/* Returns 0 when no request is pending, 1 when stopped, and 2 on UART error. */
static uint8_t MotionControl_HandleStopRequest(void)
{
    if (MotionControl_StopRequested == 0U)
    {
        return 0U;
    }

    MotionControl_StopRequested = 0U;
    if (Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f, 0) != HAL_OK)
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

static MotionControlStatus MotionControl_MoveVectorMm(float forward_mm,
                                                       float left_mm,
                                                       float target_angle_deg,
                                                       MotionControlStatus move_status)
{
    float target_mm;
    float corrected_forward_mm;
    float corrected_left_mm;
    float corrected_length_mm;
    float forward_unit;
    float left_unit;
    float peak_rpm;
    float wheel_mm_per_rpm_ms;
    float full_ramp_distance;
    float decel_distance;
    float previous_effective_base_rpm = 0.0f;
    float previous_wheel_scale = 1.0f;
    uint32_t stage_start;
    uint32_t last_integral_tick;
    uint32_t next_tick;
    uint8_t stage = 0U; /* 0 accelerate, 1 cruise, 2 decelerate. */

    if ((forward_mm == 0.0f) && (left_mm == 0.0f))
    {
        return MotionControl_State;
    }

    corrected_forward_mm = forward_mm * FORWARD_DISTANCE_GAIN;
    corrected_left_mm = left_mm * LEFT_DISTANCE_GAIN;
    corrected_length_mm = sqrtf((corrected_forward_mm * corrected_forward_mm) +
                                (corrected_left_mm * corrected_left_mm));
    if (corrected_length_mm <= 0.0f)
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }

    target_mm = corrected_length_mm;
    forward_unit = corrected_forward_mm / corrected_length_mm;
    left_unit = corrected_left_mm / corrected_length_mm;
    MotionControl_State = move_status;
    MotionControl_TargetAngleDeg = target_angle_deg;
    MotionControl_ForwardUnit = forward_unit;
    MotionControl_LeftUnit = left_unit;
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    MotionControl_WheelScale = 1.0f;
    MotionControl_TraveledMm = 0.0f;
    MotionControl_TargetDistanceMm = target_mm;

    peak_rpm = ((Motion_Absolute(forward_unit) > 0.0001f) &&
                (Motion_Absolute(left_unit) > 0.0001f)) ?
               MOTION_DIAGONAL_CRUISE_RPM : MOTION_CRUISE_RPM;

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
        float wheel_scale = 1.0f;
        uint8_t stop_result;

        stop_result = MotionControl_HandleStopRequest();
        if (stop_result != 0U)
        {
            return MotionControl_State;
        }

        MotionControl_TraveledMm += Motion_Absolute(previous_effective_base_rpm) *
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
            if (MotionControl_TraveledMm >= (target_mm - decel_distance))
            {
                stage = 2U;
                stage_start = now;
            }
        }
        else
        {
            if ((stage_elapsed >= MOTION_RAMP_TIME_MS) ||
                (MotionControl_TraveledMm >= target_mm))
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
            (void)Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f, 0);
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
                                    correction_rpm,
                                    &wheel_scale) != HAL_OK)
        {
            (void)Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f, 0);
            (void)MotorControl_StopAll();
            MotionControl_State = MOTION_ERROR_MOTOR_UART;
            return MotionControl_State;
        }
        previous_wheel_scale = wheel_scale;
        previous_effective_base_rpm = base_rpm * previous_wheel_scale;
        MotionControl_BaseRpm = base_rpm;
        MotionControl_WheelScale = wheel_scale;
        MotionControl_EffectiveBaseRpm = previous_effective_base_rpm;
        Motion_WaitControlPeriod(&next_tick);
    }

    if (MotionControl_HandleStopRequest() != 0U)
    {
        return MotionControl_State;
    }
    if (Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f, 0) != HAL_OK)
    {
        (void)MotorControl_StopAll();
        MotionControl_State = MOTION_ERROR_MOTOR_UART;
    }
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    return MotionControl_State;
}

static uint8_t Motion_SegmentAngleValid(float angle_deg)
{
    return ((angle_deg >= -180.0f) && (angle_deg <= 180.0f)) ? 1U : 0U;
}

static float Motion_SegmentAngle(float start_angle_deg,
                                 float end_angle_deg,
                                 uint32_t elapsed_ms,
                                 uint32_t blend_time_ms)
{
    float progress;

    if (blend_time_ms == 0U)
    {
        return end_angle_deg;
    }
    if (elapsed_ms >= blend_time_ms)
    {
        return end_angle_deg;
    }
    progress = (float)elapsed_ms / (float)blend_time_ms;
    return start_angle_deg +
           ((end_angle_deg - start_angle_deg) * progress);
}

static MotionControlStatus Motion_SegmentFail(MotionControlStatus error)
{
    (void)Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f, 0);
    (void)MotorControl_StopAll();
    MotionControl_State = error;
    MotionControl_BaseRpm = 0.0f;
    MotionControl_EffectiveBaseRpm = 0.0f;
    return MotionControl_State;
}

static MotionControlStatus MotionControl_RunPolarSegment(
    uint32_t distance_mm,
    float start_angle_deg,
    float end_angle_deg,
    uint32_t blend_time_ms,
    float start_rpm,
    float cruise_rpm,
    float end_rpm,
    MotionControlStatus move_status)
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
    uint32_t segment_start;
    uint32_t stage_start;
    uint32_t last_integral_tick;
    uint32_t next_tick;
        uint8_t stage = 0U; /* 0 accelerate, 1 cruise, 2 decelerate. */

    if ((distance_mm == 0U) ||
        (Motion_SegmentAngleValid(start_angle_deg) == 0U) ||
        (Motion_SegmentAngleValid(end_angle_deg) == 0U) ||
        !(start_rpm >= 0.0f) || !(cruise_rpm > 0.0f) ||
        !(end_rpm >= 0.0f) || (cruise_rpm < start_rpm) ||
        (cruise_rpm < end_rpm) ||
        ((blend_time_ms == 0U) &&
         (Motion_Absolute(start_angle_deg - end_angle_deg) > 0.0001f)))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }

    target_mm = (float)distance_mm;
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
    MotionControl_TargetAngleDeg = start_angle_deg;
    MotionControl_ForwardUnit = cosf(start_angle_deg * MOTION_PI / 180.0f);
    MotionControl_LeftUnit = sinf(start_angle_deg * MOTION_PI / 180.0f);
    MotionControl_BaseRpm = start_rpm;
    MotionControl_EffectiveBaseRpm = start_rpm;
    MotionControl_WheelScale = 1.0f;
    MotionControl_TraveledMm = 0.0f;
    MotionControl_TargetDistanceMm = target_mm;

    segment_start = HAL_GetTick();
    stage_start = segment_start;
    last_integral_tick = segment_start;
    next_tick = segment_start;

    while (stage < 3U)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - last_integral_tick;
        uint32_t stage_elapsed_ms = now - stage_start;
        uint32_t segment_elapsed_ms = now - segment_start;
        float base_rpm;
        float current_angle_deg;
        float radians;
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
            correction_rpm = Motion_HeadingCorrectionRpm(base_rpm);
        }

        current_angle_deg = Motion_SegmentAngle(
            start_angle_deg, end_angle_deg,
            segment_elapsed_ms, blend_time_ms);
        radians = current_angle_deg * MOTION_PI / 180.0f;
        MotionControl_TargetAngleDeg = current_angle_deg;
        MotionControl_ForwardUnit = cosf(radians);
        MotionControl_LeftUnit = sinf(radians);

        if (Motion_SendChassisSpeed(base_rpm * MotionControl_ForwardUnit,
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
        float radians = end_angle_deg * MOTION_PI / 180.0f;

        if ((MotionControl_ImuHeadingHoldActive != 0U) &&
            (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U))
        {
            return Motion_SegmentFail(MOTION_ERROR_IMU_LOST);
        }
        if (MotionControl_ImuHeadingHoldActive != 0U)
        {
            final_correction_rpm = Motion_HeadingCorrectionRpm(end_rpm);
        }
        MotionControl_TargetAngleDeg = end_angle_deg;
        MotionControl_ForwardUnit = cosf(radians);
        MotionControl_LeftUnit = sinf(radians);
        if (Motion_SendChassisSpeed(end_rpm * MotionControl_ForwardUnit,
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

MotionControlStatus MotionControl_MoveMm(int32_t forward_mm, int32_t left_mm)
{
    float forward = (float)forward_mm;
    float left = (float)left_mm;
    float angle_deg;

    if ((forward_mm == 0) && (left_mm == 0))
    {
        return MotionControl_State;
    }
    angle_deg = (180.0f / MOTION_PI) * atan2f(left, forward);
    return MotionControl_MoveVectorMm(
        forward, left, angle_deg,
        ((forward_mm != 0) && (left_mm != 0)) ?
            MOTION_STATUS_DIAGONAL :
            ((forward_mm != 0) ? MOTION_STATUS_FORWARD : MOTION_STATUS_LEFT));
}

MotionControlStatus MotionControl_MovePolarMm(uint32_t distance_mm,
                                               float angle_deg)
{
    float radians;
    float distance;
    float forward_mm;
    float left_mm;
    MotionControlStatus move_status;

    if (distance_mm == 0U)
    {
        return MotionControl_State;
    }
    if (!(angle_deg >= -180.0f) || !(angle_deg <= 180.0f))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    distance = (float)distance_mm;
    radians = angle_deg * MOTION_PI / 180.0f;
    forward_mm = distance * cosf(radians);
    left_mm = distance * sinf(radians);
    move_status = ((Motion_Absolute(forward_mm) > 0.0001f) &&
                   (Motion_Absolute(left_mm) > 0.0001f)) ?
                  MOTION_STATUS_DIAGONAL : MOTION_STATUS_POLAR_MOVE;
    return MotionControl_MoveVectorMm(forward_mm,
                                      left_mm,
                                      angle_deg,
                                      move_status);
}

MotionControlStatus MotionControl_MovePolarSegmentMm(
    uint32_t distance_mm,
    float angle_deg,
    float start_rpm,
    float cruise_rpm,
    float end_rpm)
{
    float radians;
    float forward_unit;
    float left_unit;
    MotionControlStatus move_status;

    if (Motion_SegmentAngleValid(angle_deg) == 0U)
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    radians = angle_deg * MOTION_PI / 180.0f;
    forward_unit = cosf(radians);
    left_unit = sinf(radians);
    move_status = ((Motion_Absolute(forward_unit) > 0.0001f) &&
                   (Motion_Absolute(left_unit) > 0.0001f)) ?
                  MOTION_STATUS_DIAGONAL : MOTION_STATUS_POLAR_MOVE;
    return MotionControl_RunPolarSegment(
        distance_mm, angle_deg, angle_deg, 0U,
        start_rpm, cruise_rpm, end_rpm, move_status);
}

MotionControlStatus MotionControl_MovePolarBlendSegmentMm(
    uint32_t distance_mm,
    float start_angle_deg,
    float end_angle_deg,
    uint32_t blend_time_ms,
    float start_rpm,
    float cruise_rpm,
    float end_rpm)
{
    float start_radians;
    float end_radians;
    float start_forward_unit;
    float start_left_unit;
    float end_forward_unit;
    float end_left_unit;
    MotionControlStatus move_status;

    if ((Motion_SegmentAngleValid(start_angle_deg) == 0U) ||
        (Motion_SegmentAngleValid(end_angle_deg) == 0U))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    start_radians = start_angle_deg * MOTION_PI / 180.0f;
    end_radians = end_angle_deg * MOTION_PI / 180.0f;
    start_forward_unit = cosf(start_radians);
    start_left_unit = sinf(start_radians);
    end_forward_unit = cosf(end_radians);
    end_left_unit = sinf(end_radians);
    move_status = (((Motion_Absolute(start_forward_unit) > 0.0001f) &&
                    (Motion_Absolute(start_left_unit) > 0.0001f)) ||
                   ((Motion_Absolute(end_forward_unit) > 0.0001f) &&
                    (Motion_Absolute(end_left_unit) > 0.0001f))) ?
                  MOTION_STATUS_DIAGONAL : MOTION_STATUS_POLAR_MOVE;
    return MotionControl_RunPolarSegment(
        distance_mm, start_angle_deg, end_angle_deg, blend_time_ms,
        start_rpm, cruise_rpm, end_rpm, move_status);
}

static uint8_t Motion_IsWrapperAngleValid(float angle_deg)
{
    return ((angle_deg >= 0.0f) && (angle_deg <= 90.0f)) ? 1U : 0U;
}

MotionControlStatus MotionControl_MoveLeftFrontMm(uint32_t distance_mm,
                                                   float angle_deg)
{
    if (!Motion_IsWrapperAngleValid(angle_deg))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    return MotionControl_MovePolarMm(distance_mm, angle_deg);
}

MotionControlStatus MotionControl_MoveRightFrontMm(uint32_t distance_mm,
                                                    float angle_deg)
{
    if (!Motion_IsWrapperAngleValid(angle_deg))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    return MotionControl_MovePolarMm(distance_mm, -angle_deg);
}

MotionControlStatus MotionControl_MoveLeftRearMm(uint32_t distance_mm,
                                                  float angle_deg)
{
    if (!Motion_IsWrapperAngleValid(angle_deg))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    return MotionControl_MovePolarMm(distance_mm, 180.0f - angle_deg);
}

MotionControlStatus MotionControl_MoveRightRearMm(uint32_t distance_mm,
                                                   float angle_deg)
{
    if (!Motion_IsWrapperAngleValid(angle_deg))
    {
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        return MotionControl_State;
    }
    return MotionControl_MovePolarMm(distance_mm, -(180.0f - angle_deg));
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
    Jy61P_ResetContinuousYaw();
    MotionControl_State = MOTION_STATUS_ROTATING;
    MotionControl_RotateTargetDeg = angle_deg;
    MotionControl_RotateCurrentDeg = 0.0f;
    MotionControl_RotateErrorDeg = angle_deg;
    MotionControl_RotateCommandRpm = 0.0f;
    MotionControl_RotateSettleCount = 0U;
    MotionControl_RotateElapsedMs = 0U;
    MotionControl_HeadingErrorDeg = 0.0f;
    MotionControl_HeadingCorrectionRpm = 0.0f;
    previous_heading_error = 0.0f;
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
            MotionControl_RotateCommandRpm = 0.0f;
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
            return Motion_SegmentFail(MOTION_ERROR_ROTATE_TIMEOUT);
        }

        MotionControl_RotateCurrentDeg = Jy61P_GetContinuousYaw();
        error_deg = MotionControl_RotateTargetDeg -
                    MotionControl_RotateCurrentDeg;
        absolute_error_deg = Motion_Absolute(error_deg);
        MotionControl_RotateErrorDeg = error_deg;

        if (absolute_error_deg <= ROTATE_TOLERANCE_DEG)
        {
            MotionControl_RotateCommandRpm = 0.0f;
            if (Motion_SendChassisSpeed(0.0f, 0.0f, 0.0f,
                                        &wheel_scale) != HAL_OK)
            {
                return Motion_SegmentFail(MOTION_ERROR_MOTOR_UART);
            }
            MotionControl_WheelScale = wheel_scale;
            MotionControl_RotateSettleCount++;
            if (MotionControl_RotateSettleCount >= ROTATE_SETTLE_CYCLES)
            {
                /* The settled heading becomes the reference for the next move. */
                Jy61P_ResetContinuousYaw();
                MotionControl_RotateCurrentDeg = MotionControl_RotateTargetDeg;
                MotionControl_RotateErrorDeg = 0.0f;
                previous_heading_error = 0.0f;
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
            if (Motion_SendChassisSpeed(0.0f, 0.0f,
                                        MotionControl_RotateCommandRpm,
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
        Jy61P_ResetContinuousYaw();
    }
    previous_heading_error = 0.0f;
    HAL_Delay(100U);

    return MotionControl_State;
}

MotionControlStatus MotionControl_RunDefaultSequence(void)
{
    if (MotionControl_PrepareForMove() >= MOTION_ERROR_IMU_STARTUP)
    {
        return MotionControl_State;
    }

#if DIAGONAL_TEST_ENABLE
    {
        MotionControlStatus status;

#if DIAGONAL_TEST_DIRECTION == DIAGONAL_TEST_LEFT_FRONT
        status = MotionControl_MoveLeftFrontMm(
            DIAGONAL_TEST_DISTANCE_MM, DIAGONAL_TEST_ANGLE_DEG);
#elif DIAGONAL_TEST_DIRECTION == DIAGONAL_TEST_RIGHT_FRONT
        status = MotionControl_MoveRightFrontMm(
            DIAGONAL_TEST_DISTANCE_MM, DIAGONAL_TEST_ANGLE_DEG);
#elif DIAGONAL_TEST_DIRECTION == DIAGONAL_TEST_LEFT_REAR
        status = MotionControl_MoveLeftRearMm(
            DIAGONAL_TEST_DISTANCE_MM, DIAGONAL_TEST_ANGLE_DEG);
#elif DIAGONAL_TEST_DIRECTION == DIAGONAL_TEST_RIGHT_REAR
        status = MotionControl_MoveRightRearMm(
            DIAGONAL_TEST_DISTANCE_MM, DIAGONAL_TEST_ANGLE_DEG);
#else
        MotionControl_State = MOTION_ERROR_INVALID_ARGUMENT;
        status = MotionControl_State;
#endif
        if (status >= MOTION_ERROR_IMU_STARTUP)
        {
            return MotionControl_State;
        }
    }
#else
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
#endif

    if (MotorControl_StopAll() != HAL_OK)
    {
        MotionControl_State = MOTION_ERROR_MOTOR_UART;
        return MotionControl_State;
    }
    MotionControl_State = MOTION_STATUS_FINISHED;
    return MotionControl_State;
}
