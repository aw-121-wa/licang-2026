#include "jy61p.h"
#include "maixcam_link.h"
#include "servo_action.h"
#include "uart_command.h"

static UART_HandleTypeDef *jy61p_uart = NULL;
static uint8_t jy61p_rx_byte = 0U;
static uint8_t rx_buffer[11];
static uint8_t rx_state = 0U;
static uint8_t rx_index = 0U;
static float jy61p_last_raw_yaw = 0.0f;
static uint8_t jy61p_continuous_valid = 0U;

volatile float Roll = 0.0f;
volatile float Pitch = 0.0f;
volatile float Yaw = 0.0f;
volatile float Jy61P_ContinuousYaw = 0.0f;
volatile uint8_t Jy61P_UpdateFlag = 0U;
volatile uint32_t Jy61P_LastUpdateTick = 0U;
volatile uint32_t Jy61P_FrameCount = 0U;
volatile uint32_t Jy61P_RxByteCount = 0U;
volatile uint32_t Jy61P_ChecksumErrorCount = 0U;

static int16_t Jy61P_MakeInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static void Jy61P_UpdateContinuousYaw(float current_yaw)
{
    float delta;

    if (jy61p_continuous_valid == 0U)
    {
        jy61p_last_raw_yaw = current_yaw;
        Jy61P_ContinuousYaw = 0.0f;
        jy61p_continuous_valid = 1U;
        return;
    }

    delta = current_yaw - jy61p_last_raw_yaw;

    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    Jy61P_ContinuousYaw += delta;
    jy61p_last_raw_yaw = current_yaw;
}

void Jy61P_Init(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    jy61p_uart = huart;
    rx_state = 0U;
    rx_index = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    Roll = 0.0f;
    Pitch = 0.0f;
    Yaw = 0.0f;
    Jy61P_ContinuousYaw = 0.0f;
    jy61p_last_raw_yaw = 0.0f;
    jy61p_continuous_valid = 0U;
    Jy61P_UpdateFlag = 0U;
    Jy61P_LastUpdateTick = 0U;
    Jy61P_FrameCount = 0U;
    Jy61P_RxByteCount = 0U;
    Jy61P_ChecksumErrorCount = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }

    if (jy61p_uart != NULL)
    {
        (void)HAL_UART_Receive_IT(jy61p_uart, &jy61p_rx_byte, 1U);
    }
}

void Jy61P_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((jy61p_uart != NULL) && (huart == jy61p_uart))
    {
        Jy61P_ReceiveData(jy61p_rx_byte);
        (void)HAL_UART_Receive_IT(jy61p_uart, &jy61p_rx_byte, 1U);
    }
}

void Jy61P_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((jy61p_uart != NULL) && (huart == jy61p_uart))
    {
        rx_state = 0U;
        rx_index = 0U;
        __HAL_UART_CLEAR_OREFLAG(jy61p_uart);
        jy61p_uart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(jy61p_uart, &jy61p_rx_byte, 1U);
    }
}

void Jy61P_ReceiveData(uint8_t rx_data)
{
    uint8_t i;
    uint8_t sum;
    int16_t raw_roll;
    int16_t raw_pitch;
    int16_t raw_yaw;
    float new_yaw;

    Jy61P_RxByteCount++;

    if (rx_state == 0U)
    {
        if (rx_data == 0x55U)
        {
            rx_buffer[0] = rx_data;
            rx_index = 1U;
            rx_state = 1U;
        }
        return;
    }

    if (rx_state == 1U)
    {
        if (rx_data == 0x53U)
        {
            rx_buffer[1] = rx_data;
            rx_index = 2U;
            rx_state = 2U;
        }
        else if (rx_data == 0x55U)
        {
            rx_buffer[0] = rx_data;
            rx_index = 1U;
        }
        else
        {
            rx_state = 0U;
            rx_index = 0U;
        }
        return;
    }

    rx_buffer[rx_index++] = rx_data;

    if (rx_index < 11U)
    {
        return;
    }

    sum = 0U;
    for (i = 0U; i < 10U; i++)
    {
        sum = (uint8_t)(sum + rx_buffer[i]);
    }

    if (sum == rx_buffer[10])
    {
        raw_roll = Jy61P_MakeInt16(rx_buffer[2], rx_buffer[3]);
        raw_pitch = Jy61P_MakeInt16(rx_buffer[4], rx_buffer[5]);
        raw_yaw = Jy61P_MakeInt16(rx_buffer[6], rx_buffer[7]);
        Roll = (float)raw_roll * (180.0f / 32768.0f);
        Pitch = (float)raw_pitch * (180.0f / 32768.0f);
        new_yaw = (float)raw_yaw * (180.0f / 32768.0f);
        Yaw = new_yaw;
        Jy61P_UpdateContinuousYaw(new_yaw);
        Jy61P_UpdateFlag = 1U;
        Jy61P_LastUpdateTick = HAL_GetTick();
        Jy61P_FrameCount++;
    }
    else
    {
        Jy61P_ChecksumErrorCount++;
    }

    rx_state = 0U;
    rx_index = 0U;
}

uint8_t Jy61P_IsOnline(uint32_t timeout_ms)
{
    uint32_t last_tick = Jy61P_LastUpdateTick;

    if ((last_tick == 0U) || ((HAL_GetTick() - last_tick) > timeout_ms))
    {
        return 0U;
    }
    return 1U;
}

uint8_t Jy61P_WaitData(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if (Jy61P_LastUpdateTick != 0U)
        {
            return 1U;
        }
        HAL_Delay(5U);
    }
    return 0U;
}

float Jy61P_GetYaw(void)
{
    float value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = Yaw;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return value;
}

void Jy61P_ResetContinuousYaw(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    jy61p_last_raw_yaw = Yaw;
    Jy61P_ContinuousYaw = 0.0f;
    jy61p_continuous_valid = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

float Jy61P_GetContinuousYaw(void)
{
    float value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = Jy61P_ContinuousYaw;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return value;
}

uint32_t Jy61P_GetLastTick(void)
{
    return Jy61P_LastUpdateTick;
}

uint32_t Jy61P_GetFrameCount(void)
{
    return Jy61P_FrameCount;
}

uint32_t Jy61P_GetRxByteCount(void)
{
    return Jy61P_RxByteCount;
}

uint32_t Jy61P_GetChecksumErrorCount(void)
{
    return Jy61P_ChecksumErrorCount;
}

/* Keep the UART interrupt plumbing inside the IMU module. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Jy61P_UartRxCpltCallback(huart);
    MaixCamLink_UartRxCpltCallback(huart);
    UartCommand_UartRxCpltCallback(huart);
    ServoAction_UartRxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Jy61P_UartErrorCallback(huart);
    MaixCamLink_UartErrorCallback(huart);
    UartCommand_UartErrorCallback(huart);
    ServoAction_UartErrorCallback(huart);
}
