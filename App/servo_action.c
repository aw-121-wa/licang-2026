#include "servo_action.h"
#include "cmsis_os.h"

#define SERVO_ACTION_FRAME_HEADER       0x55U
#define SERVO_ACTION_CMD_RUN            0x06U
#define SERVO_ACTION_CMD_COMPLETE       0x08U
#define SERVO_ACTION_RX_DATA_MAX        8U

typedef enum
{
    SERVO_RX_WAIT_HEADER_1 = 0,
    SERVO_RX_WAIT_HEADER_2,
    SERVO_RX_WAIT_LENGTH,
    SERVO_RX_READ_DATA
} ServoRxState;

static UART_HandleTypeDef *servo_uart = 0;
static uint8_t servo_rx_byte = 0U;
static uint8_t servo_rx_data[SERVO_ACTION_RX_DATA_MAX];
static uint8_t servo_rx_length = 0U;
static uint8_t servo_rx_index = 0U;
static ServoRxState servo_rx_state = SERVO_RX_WAIT_HEADER_1;
static volatile uint8_t servo_completed_group = 0xFFU;

volatile ServoActionStatus ServoAction_LastStatus = SERVO_ACTION_ERROR_UART;
volatile ServoActionSequenceState ServoAction_SequenceState = SERVO_SEQUENCE_STARTING;

static void ServoAction_ResetParser(void)
{
    servo_rx_length = 0U;
    servo_rx_index = 0U;
    servo_rx_state = SERVO_RX_WAIT_HEADER_1;
}

static void ServoAction_ParseByte(uint8_t value)
{
    switch (servo_rx_state)
    {
    case SERVO_RX_WAIT_HEADER_1:
        if (value == SERVO_ACTION_FRAME_HEADER)
        {
            servo_rx_state = SERVO_RX_WAIT_HEADER_2;
        }
        break;

    case SERVO_RX_WAIT_HEADER_2:
        if (value == SERVO_ACTION_FRAME_HEADER)
        {
            servo_rx_state = SERVO_RX_WAIT_LENGTH;
        }
        else
        {
            servo_rx_state = SERVO_RX_WAIT_HEADER_1;
        }
        break;

    case SERVO_RX_WAIT_LENGTH:
        /* Length includes the length byte and command byte. */
        if ((value >= 2U) && (value <= (SERVO_ACTION_RX_DATA_MAX + 1U)))
        {
            servo_rx_length = value;
            servo_rx_index = 0U;
            servo_rx_state = SERVO_RX_READ_DATA;
        }
        else
        {
            ServoAction_ResetParser();
        }
        break;

    case SERVO_RX_READ_DATA:
        servo_rx_data[servo_rx_index++] = value;
        if (servo_rx_index >= (uint8_t)(servo_rx_length - 1U))
        {
            if ((servo_rx_length == 5U) &&
                (servo_rx_data[0] == SERVO_ACTION_CMD_COMPLETE))
            {
                servo_completed_group = servo_rx_data[1];
            }
            ServoAction_ResetParser();
        }
        break;

    default:
        ServoAction_ResetParser();
        break;
    }
}

void ServoAction_Init(UART_HandleTypeDef *huart)
{
    servo_uart = huart;
    servo_completed_group = 0xFFU;
    ServoAction_LastStatus = SERVO_ACTION_ERROR_UART;
    ServoAction_SequenceState = SERVO_SEQUENCE_STARTING;
    ServoAction_ResetParser();
    if (servo_uart != 0)
    {
        if (HAL_UART_Receive_IT(servo_uart, &servo_rx_byte, 1U) == HAL_OK)
        {
            ServoAction_LastStatus = SERVO_ACTION_OK;
        }
    }
}

ServoActionStatus ServoAction_RunGroup(uint8_t group,
                                       uint16_t repeat_count,
                                       uint32_t timeout_ms)
{
    uint32_t start_tick;

    if ((servo_uart == 0) || (timeout_ms == 0U))
    {
        ServoAction_LastStatus = SERVO_ACTION_ERROR_ARGUMENT;
        return ServoAction_LastStatus;
    }

    if (ServoAction_StartGroupNoWait(group, repeat_count) != SERVO_ACTION_OK)
    {
        return ServoAction_LastStatus;
    }

    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) < timeout_ms)
    {
        if (servo_completed_group == group)
        {
            ServoAction_LastStatus = SERVO_ACTION_OK;
            return ServoAction_LastStatus;
        }
        osDelay(10U);
    }

    ServoAction_LastStatus = SERVO_ACTION_ERROR_TIMEOUT;
    return ServoAction_LastStatus;
}

ServoActionStatus ServoAction_StartGroupNoWait(uint8_t group,
                                               uint16_t repeat_count)
{
    uint8_t frame[7];

    if (servo_uart == 0)
    {
        ServoAction_LastStatus = SERVO_ACTION_ERROR_ARGUMENT;
        return ServoAction_LastStatus;
    }

    /* Discard an old completion event before issuing the new request. */
    servo_completed_group = 0xFFU;
    frame[0] = SERVO_ACTION_FRAME_HEADER;
    frame[1] = SERVO_ACTION_FRAME_HEADER;
    frame[2] = 0x05U;
    frame[3] = SERVO_ACTION_CMD_RUN;
    frame[4] = group;
    frame[5] = (uint8_t)(repeat_count & 0xFFU);
    frame[6] = (uint8_t)((repeat_count >> 8) & 0xFFU);

    if (HAL_UART_Transmit(servo_uart, frame, sizeof(frame), 100U) != HAL_OK)
    {
        ServoAction_LastStatus = SERVO_ACTION_ERROR_UART;
        return ServoAction_LastStatus;
    }

    ServoAction_LastStatus = SERVO_ACTION_OK;
    return ServoAction_LastStatus;
}

void ServoAction_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((servo_uart == 0) || (huart != servo_uart))
    {
        return;
    }

    ServoAction_ParseByte(servo_rx_byte);
    (void)HAL_UART_Receive_IT(servo_uart, &servo_rx_byte, 1U);
}

void ServoAction_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if ((servo_uart == 0) || (huart != servo_uart))
    {
        return;
    }

    ServoAction_ResetParser();
    __HAL_UART_CLEAR_OREFLAG(servo_uart);
    servo_uart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(servo_uart, &servo_rx_byte, 1U);
}

const char *ServoAction_SequenceStateName(ServoActionSequenceState state)
{
    switch (state)
    {
    case SERVO_SEQUENCE_STARTING:       return "STARTING";
    case SERVO_SEQUENCE_WAITING_MOTION: return "WAIT_MOTION";
    case SERVO_SEQUENCE_GRAB_RUNNING:   return "GRAB_RUNNING";
    case SERVO_SEQUENCE_RETURN_RUNNING: return "RETURN_RUNNING";
    case SERVO_SEQUENCE_DONE:           return "DONE";
    case SERVO_SEQUENCE_ERROR:          return "ERROR";
    default:                            return "UNKNOWN";
    }
}
