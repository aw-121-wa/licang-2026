#include "rfid.h"
#include "usart.h"

static uint8_t rfid_rx_byte = 0U;
static volatile uint32_t rfid_id = 0U;
static volatile uint8_t rfid_flag = 0U;
static volatile RfidStatus rfid_status;

#define RFID_FRAME_MAX 28U
static uint8_t frame[RFID_FRAME_MAX];
static uint8_t frame_length;
static uint32_t last_byte_tick;
static void RFID_AcceptFrame(void)
{
    uint8_t i;
    uint8_t checksum = 0U;
    uint32_t uid = 0U;
    for (i = 0U; i < frame[1]; i++) checksum ^= frame[i];
    if (checksum != 0xFFU)
    {
        rfid_status.invalid_count++;
        return;
    }
    rfid_status.frame_count++;
    rfid_status.reader_status = frame[4];
    if (frame[4] != 0U) return;
    if (!(((frame[0] == 1U) && (frame[2] == 0xA1U) && (frame[1] == 12U)) ||
          ((frame[0] == 4U) && (frame[2] == 2U) && (frame[1] == 12U)) ||
          ((frame[0] == 4U) && (frame[2] == 4U) && (frame[1] == 28U)))) return;
    for (i = 7U; i < 11U; i++) uid = (uid << 8) | frame[i];
    rfid_status.uid = uid;
    rfid_status.tag_count++;
    if (rfid_flag == 0U)
    {
        rfid_id = uid;
        rfid_flag = 1U;
    }
}

static void RFID_ReceiveByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_byte_tick) > RFID_FRAME_GAP_MS) frame_length = 0U;
    last_byte_tick = now;
    if (frame_length == 0U)
    {
        if ((byte != 1U) && (byte != 4U)) return;
    }
    else if (frame_length == 1U)
    {
        if ((byte != 8U) && (byte != 12U) && (byte != 22U) && (byte != 28U))
        {
            frame_length = ((byte == 1U) || (byte == 4U)) ? 1U : 0U;
            frame[0] = byte;
            rfid_status.invalid_count++;
            return;
        }
    }
    else if ((frame_length == 3U) && (byte != RFID_READER_ADDRESS))
    {
        frame_length = 0U;
        rfid_status.invalid_count++;
        return;
    }
    frame[frame_length++] = byte;
    if ((frame_length >= 2U) && (frame_length == frame[1]))
    {
        RFID_AcceptFrame();
        frame_length = 0U;
    }
}

void RFID_Init(void)
{
    rfid_rx_byte = 0U;
    RFID_Clear();
    (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
}

/* Recover a failed receive arm without blocking or configuring reader EEPROM. */
void RFID_Poll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (huart8.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
    }
    if (primask == 0U) __enable_irq();
}

void RFID_GetStatus(RfidStatus *status)
{
    uint32_t primask;
    if (status == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    *status = rfid_status;
    status->pending = rfid_flag;
    status->receiving = (huart8.RxState == HAL_UART_STATE_BUSY_RX) ? 1U : 0U;
    if (primask == 0U) __enable_irq();
}

void RFID_Clear(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    rfid_id = 0U;
    rfid_flag = 0U;
    frame_length = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t RFID_Read_ID(uint32_t *id)
{
    uint32_t primask;
    uint8_t result = 0U;

    if (id == NULL)
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

uint8_t RFID_Get_ID(uint32_t *id)
{
    return RFID_Read_ID(id);
}

void RFID_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == &huart8))
    {
        rfid_status.rx_count++;
        rfid_status.last_byte = rfid_rx_byte;
        /* HAL may deliver RX completion before its error callback. */
        if (huart->ErrorCode != HAL_UART_ERROR_NONE)
        {
            rfid_status.last_error = huart->ErrorCode;
            return;
        }
        RFID_ReceiveByte(rfid_rx_byte);
        (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
    }
}

void RFID_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == &huart8))
    {
        rfid_status.last_error = huart->ErrorCode;
        RFID_Clear();
        __HAL_UART_CLEAR_OREFLAG(&huart8);
        huart8.ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(&huart8, &rfid_rx_byte, 1U);
    }
}
