#include "maixcam_link.h"

#define MAIXCAM_RX_LINE_MAX                 8U

static UART_HandleTypeDef *maixcam_uart = 0;
static uint8_t maixcam_rx_byte = 0U;
static char maixcam_rx_line[MAIXCAM_RX_LINE_MAX];
static uint8_t maixcam_rx_length = 0U;
static uint8_t maixcam_rx_overflow = 0U;
static volatile uint8_t maixcam_reply_pending = 0U;
static volatile uint8_t maixcam_request_active = 0U;

static void MaixCamLink_ResetLine(void)
{
    maixcam_rx_length = 0U;
    maixcam_rx_overflow = 0U;
    maixcam_rx_line[0] = '\0';
}

static void MaixCamLink_CompleteLine(void)
{
    if ((maixcam_rx_overflow == 0U) &&
        (maixcam_rx_length == 1U) &&
        (maixcam_rx_line[0] == '1'))
    {
        if (maixcam_request_active != 0U)
        {
            maixcam_reply_pending = 1U;
            maixcam_request_active = 0U;
        }
    }
    MaixCamLink_ResetLine();
}

void MaixCamLink_Init(UART_HandleTypeDef *huart)
{
    maixcam_uart = huart;
    maixcam_rx_byte = 0U;
    maixcam_reply_pending = 0U;
    maixcam_request_active = 0U;
    MaixCamLink_ResetLine();
    if (maixcam_uart != 0)
    {
        (void)HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
    }
}

MaixCamLinkStatus MaixCamLink_SendRequest(MaixCamColor color)
{
    uint8_t command;

    if (maixcam_uart == 0)
    {
        return MAIXCAM_LINK_ERROR_ARGUMENT;
    }

    if (color == MAIXCAM_COLOR_RED)
    {
        command = '1';
    }
    else if (color == MAIXCAM_COLOR_BLUE)
    {
        command = '2';
    }
    else
    {
        return MAIXCAM_LINK_ERROR_ARGUMENT;
    }

    /* A reply received before this request belongs to an older transaction. */
    maixcam_reply_pending = 0U;
    maixcam_request_active = 0U;
    MaixCamLink_ResetLine();
    if (HAL_UART_Transmit(maixcam_uart,
                          &command,
                          1U,
                          100U) != HAL_OK)
    {
        return MAIXCAM_LINK_ERROR_UART;
    }

    maixcam_request_active = 1U;
    return MAIXCAM_LINK_OK;
}

uint8_t MaixCamLink_TakeReply(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t reply;

    __disable_irq();
    reply = maixcam_reply_pending;
    maixcam_reply_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return reply;
}

void MaixCamLink_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((maixcam_uart == 0) || (huart != maixcam_uart))
    {
        return;
    }

    if (maixcam_rx_byte == '\r')
    {
        /* CR is accepted as the first half of the required CRLF delimiter. */
    }
    else if (maixcam_rx_byte == '\n')
    {
        MaixCamLink_CompleteLine();
    }
    else if (maixcam_rx_overflow == 0U)
    {
        if (maixcam_rx_length < (MAIXCAM_RX_LINE_MAX - 1U))
        {
            maixcam_rx_line[maixcam_rx_length++] = (char)maixcam_rx_byte;
            maixcam_rx_line[maixcam_rx_length] = '\0';
        }
        else
        {
            maixcam_rx_overflow = 1U;
        }
    }

    (void)HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
}

void MaixCamLink_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((maixcam_uart == 0) || (huart != maixcam_uart))
    {
        return;
    }

    MaixCamLink_ResetLine();
    __HAL_UART_CLEAR_OREFLAG(maixcam_uart);
    maixcam_uart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(maixcam_uart, &maixcam_rx_byte, 1U);
}
