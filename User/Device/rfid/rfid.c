#include "rfid.h"
#include "usart.h"

static uint8_t rfid_rx_byte = 0U;
static uint8_t rfid_id = 0U;
static uint8_t rfid_flag = 0U;

static uint8_t RFID_IsValidID(uint8_t id)
{
    return ((id >= 1U) && (id <= 9U)) ? 1U : 0U;
}

void RFID_Init(void)
{
    rfid_rx_byte = 0U;
    RFID_Clear();
    (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
}

void RFID_Clear(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    rfid_id = 0U;
    rfid_flag = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t RFID_Read_ID(uint8_t *id)
{
    uint32_t primask;
    uint8_t result = 0U;

    if ((id == NULL) || (rfid_flag == 0U))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (rfid_flag != 0U)
    {
        *id = rfid_id;
        rfid_flag = 0U;
        result = 1U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return result;
}

uint8_t RFID_Get_ID(uint8_t *id)
{
    return RFID_Read_ID(id);
}

void RFID_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == &huart8))
    {
        if (RFID_IsValidID(rfid_rx_byte) != 0U)
        {
            rfid_id = rfid_rx_byte;
            rfid_flag = 1U;
        }
        (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
    }
}

void RFID_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == &huart8))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart8);
        huart8.ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
    }
}
