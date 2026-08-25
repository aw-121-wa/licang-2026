#include "gray_align.h"
#include "cmsis_os.h"
#include "jy61p.h"
#include "mecanum_kinematics.h"
#include "motor_control.h"
#include "motion_control.h"

typedef struct
{
    uint8_t mid2;
    uint8_t in2;
    uint8_t in1;
    uint8_t mid1;
} GrayAlignSample;

static uint8_t GrayAlign_ReadOnLine(GPIO_TypeDef *port, uint16_t pin)
{
    /* The gray modules use pull-ups and assert the line with a low level. */
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static GrayAlignSample GrayAlign_ReadSensors(void)
{
    GrayAlignSample sample;

    sample.mid2 = GrayAlign_ReadOnLine(GPIOD, GPIO_PIN_8);
    sample.in2 = GrayAlign_ReadOnLine(GPIOD, GPIO_PIN_0);
    sample.in1 = GrayAlign_ReadOnLine(GPIOD, GPIO_PIN_1);
    sample.mid1 = GrayAlign_ReadOnLine(GPIOD, GPIO_PIN_3);
    return sample;
}

static HAL_StatusTypeDef GrayAlign_SendVelocity(float forward,
                                                 float left,
                                                 float counter_clockwise)
{
    MecanumWheelValues wheels;
    MotorWheelSpeedsRpmX10 speeds;

    MecanumKinematics_Solve(forward, left, counter_clockwise, &wheels);
    MecanumKinematics_DesaturateWithScale(&wheels, MOTOR_SPEED_LIMIT_RPM);
    speeds.front_left = (int16_t)(wheels.front_left * MOTOR_SPEED_COMMAND_SCALE);
    speeds.front_right = (int16_t)(wheels.front_right * MOTOR_SPEED_COMMAND_SCALE);
    speeds.rear_left = (int16_t)(wheels.rear_left * MOTOR_SPEED_COMMAND_SCALE);
    speeds.rear_right = (int16_t)(wheels.rear_right * MOTOR_SPEED_COMMAND_SCALE);
    return MotorControl_SetWheelSpeeds(&speeds);
}

static HAL_StatusTypeDef GrayAlign_Stop(void)
{
    return MotorControl_StopAll();
}

static float GrayAlign_HeadingCorrection(float locked_yaw,
                                          float *previous_error)
{
    float current_yaw = Jy61P_GetContinuousYaw();
    float error = locked_yaw - current_yaw;
    float correction;

    correction = (GRAY_ALIGN_HEADING_KP * error) +
                 (GRAY_ALIGN_HEADING_KD * (error - *previous_error));
    *previous_error = error;

    if ((error <= GRAY_ALIGN_HEADING_DEADBAND_DEG) &&
        (error >= -GRAY_ALIGN_HEADING_DEADBAND_DEG))
    {
        correction = 0.0f;
    }
    else if (correction > GRAY_ALIGN_HEADING_MAX_RPM)
    {
        correction = GRAY_ALIGN_HEADING_MAX_RPM;
    }
    else if (correction < -GRAY_ALIGN_HEADING_MAX_RPM)
    {
        correction = -GRAY_ALIGN_HEADING_MAX_RPM;
    }

    return correction;
}

GrayAlignStatus GrayAlign_Run(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t stable_since = 0U;
    float locked_yaw;
    float previous_error = 0.0f;
    uint8_t stable = 0U;

    if (Jy61P_IsOnline(500U) == 0U)
    {
        (void)GrayAlign_Stop();
        return GRAY_ALIGN_ERROR_IMU;
    }
    locked_yaw = Jy61P_GetContinuousYaw();

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        GrayAlignSample sample = GrayAlign_ReadSensors();
        float left;
        float heading_correction;

        if (MotionControl_StopRequested != 0U)
        {
            if (GrayAlign_Stop() != HAL_OK)
            {
                return GRAY_ALIGN_ERROR_MOTOR_UART;
            }
            return GRAY_ALIGN_CANCELED;
        }
        if ((uint32_t)(now - start_tick) >= GRAY_ALIGN_TIMEOUT_MS)
        {
            (void)GrayAlign_Stop();
            return GRAY_ALIGN_ERROR_TIMEOUT;
        }
        if (Jy61P_IsOnline(500U) == 0U)
        {
            (void)GrayAlign_Stop();
            return GRAY_ALIGN_ERROR_IMU;
        }

        if ((sample.mid2 == 0U) &&
            (sample.in2 != 0U) &&
            (sample.in1 != 0U) &&
            (sample.mid1 == 0U))
        {
            if (stable == 0U)
            {
                stable = 1U;
                stable_since = now;
            }
            if (GrayAlign_Stop() != HAL_OK)
            {
                return GRAY_ALIGN_ERROR_MOTOR_UART;
            }
            if ((uint32_t)(now - stable_since) >= GRAY_ALIGN_STABLE_MS)
            {
                Jy61P_ResetContinuousYaw();
                return GRAY_ALIGN_OK;
            }
        }
        else
        {
            stable = 0U;
            left = ((sample.mid2 != 0U) || (sample.mid1 != 0U)) ?
                   GRAY_ALIGN_RETREAT_RPM : -GRAY_ALIGN_APPROACH_RPM;
            heading_correction = GrayAlign_HeadingCorrection(
                locked_yaw, &previous_error);
            if (GrayAlign_SendVelocity(0.0f, left,
                                       heading_correction) != HAL_OK)
            {
                return GRAY_ALIGN_ERROR_MOTOR_UART;
            }
        }

        osDelay(GRAY_ALIGN_PERIOD_MS);
    }
}
