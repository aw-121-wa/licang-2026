#include "gray_align.h"
#include "cmsis_os.h"
#include "jy61p.h"
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

static HAL_StatusTypeDef GrayAlign_Stop(void)
{
    HAL_StatusTypeDef status = MotionControl_SetBodySpeed(0.0f, 0.0f, 0.0f);

    if (status != HAL_OK)
    {
        (void)MotorControl_StopAll();
    }
    return status;
}

GrayAlignStatus GrayAlign_Run(void)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t stable_since = 0U;
    uint8_t stable = 0U;

    if (Jy61P_IsOnline(500U) == 0U)
    {
        (void)GrayAlign_Stop();
        return GRAY_ALIGN_ERROR_IMU;
    }
    MotionControl_ResetHeadingReference();

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
                MotionControl_ResetHeadingReference();
                return GRAY_ALIGN_OK;
            }
        }
        else
        {
            stable = 0U;
            left = ((sample.mid2 != 0U) || (sample.mid1 != 0U)) ?
                   GRAY_ALIGN_RETREAT_RPM : -GRAY_ALIGN_APPROACH_RPM;
            heading_correction = MotionControl_GetHeadingCorrection(
                GRAY_ALIGN_APPROACH_RPM);
            if (MotionControl_SetBodySpeed(0.0f, left,
                                           heading_correction) != HAL_OK)
            {
                return GRAY_ALIGN_ERROR_MOTOR_UART;
            }
        }

        osDelay(GRAY_ALIGN_PERIOD_MS);
    }
}
