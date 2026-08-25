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

static HAL_StatusTypeDef GrayAlign_Command(const GrayAlignSample *sample)
{
    float left = 0.0f;
    float counter_clockwise = 0.0f;

    if ((sample->mid2 != 0U) && (sample->mid1 != 0U))
    {
        /* Both outside sensors are on the line: retreat before re-aligning. */
        left = GRAY_ALIGN_RETREAT_RPM;
    }
    else if (sample->mid2 != 0U)
    {
        /* The left side is too deep; rotate away from MID2. */
        counter_clockwise = GRAY_ALIGN_ROTATE_RPM;
    }
    else if (sample->mid1 != 0U)
    {
        /* The right side is too deep; rotate away from MID1. */
        counter_clockwise = -GRAY_ALIGN_ROTATE_RPM;
    }
    else if ((sample->in2 != 0U) && (sample->in1 == 0U))
    {
        /* IN2 arrived first; rotate until IN1 reaches the line. */
        counter_clockwise = -GRAY_ALIGN_ROTATE_RPM;
    }
    else if ((sample->in1 != 0U) && (sample->in2 == 0U))
    {
        /* IN1 arrived first; rotate in the opposite direction. */
        counter_clockwise = GRAY_ALIGN_ROTATE_RPM;
    }
    else
    {
        /* No sensor is on the line: approach it slowly along the left axis. */
        left = -GRAY_ALIGN_APPROACH_RPM;
    }

    return GrayAlign_SendVelocity(0.0f, left, counter_clockwise);
}

GrayAlignStatus GrayAlign_Run(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t stable_since = 0U;
    uint8_t stable = 0U;

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        GrayAlignSample sample = GrayAlign_ReadSensors();

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
            if (GrayAlign_Command(&sample) != HAL_OK)
            {
                return GRAY_ALIGN_ERROR_MOTOR_UART;
            }
        }

        osDelay(GRAY_ALIGN_PERIOD_MS);
    }
}
