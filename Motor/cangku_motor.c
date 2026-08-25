#include "cangku_motor.h"

/* These bytes are the same Emm V5.0/x42 commands used by Motor/motor_control.c. */
#define ZDT_CHECK_BYTE                 0x6BU
#define ZDT_SYNC_WAIT                  0x01U
#define ZDT_POSITION_RELATIVE          0x00U
#define ZDT_SPEED_SCALE                10U
#define ZDT_UART_TIMEOUT_MS            100U
#define ZDT_INTERFRAME_DELAY_MS          2U

static UART_HandleTypeDef *zdt_uart = 0;

volatile uint32_t ZdtMotor_TxCount = 0U;
volatile HAL_StatusTypeDef ZdtMotor_LastUartStatus = HAL_ERROR;

static HAL_StatusTypeDef ZDT_Send(const uint8_t *frame, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((zdt_uart == 0) || (frame == 0) || (length == 0U))
    {
        ZdtMotor_LastUartStatus = HAL_ERROR;
        return HAL_ERROR;
    }

    status = HAL_UART_Transmit(zdt_uart,
                               (uint8_t *)frame,
                               length,
                               ZDT_UART_TIMEOUT_MS);
    ZdtMotor_LastUartStatus = status;
    if (status == HAL_OK)
    {
        ZdtMotor_TxCount++;
        /* Match the proven chassis-driver inter-frame spacing for Emm V5.0. */
        HAL_Delay(ZDT_INTERFRAME_DELAY_MS);
    }
    return status;
}

static HAL_StatusTypeDef ZDT_SyncTrigger(void)
{
    const uint8_t command[4] = {0x00U, 0xFFU, 0x66U, ZDT_CHECK_BYTE};
    return ZDT_Send(command, sizeof(command));
}

void ZDT_Init(UART_HandleTypeDef *huart)
{
    zdt_uart = huart;
    ZdtMotor_TxCount = 0U;
    ZdtMotor_LastUartStatus = (huart != 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef ZDT_Enable(void)
{
    const uint8_t command[6] = {
        ZDT_MOTOR_ADDR, 0xF3U, 0xABU, 0x01U, ZDT_SYNC_WAIT, ZDT_CHECK_BYTE
    };

    if (ZDT_Send(command, sizeof(command)) != HAL_OK)
    {
        return ZdtMotor_LastUartStatus;
    }
    return ZDT_SyncTrigger();
}

HAL_StatusTypeDef ZDT_Disable(void)
{
    const uint8_t command[6] = {
        ZDT_MOTOR_ADDR, 0xF3U, 0xABU, 0x00U, ZDT_SYNC_WAIT, ZDT_CHECK_BYTE
    };

    if (ZDT_Send(command, sizeof(command)) != HAL_OK)
    {
        return ZdtMotor_LastUartStatus;
    }
    return ZDT_SyncTrigger();
}

HAL_StatusTypeDef ZDT_Stop(void)
{
    const uint8_t command[5] = {
        ZDT_MOTOR_ADDR, 0xFEU, 0x98U, ZDT_SYNC_WAIT, ZDT_CHECK_BYTE
    };

    if (ZDT_Send(command, sizeof(command)) != HAL_OK)
    {
        return ZdtMotor_LastUartStatus;
    }
    return ZDT_SyncTrigger();
}

HAL_StatusTypeDef ZDT_RunSpeed(ZdtDirection direction,
                                uint16_t speed_rpm,
                                uint8_t acceleration)
{
    uint8_t command[8];
    uint32_t speed_command;

    if ((direction != ZDT_DIR_CW) && (direction != ZDT_DIR_CCW))
    {
        ZdtMotor_LastUartStatus = HAL_ERROR;
        return HAL_ERROR;
    }

    speed_command = (uint32_t)speed_rpm * ZDT_SPEED_SCALE;
    if (speed_command > 0xFFFFU)
    {
        ZdtMotor_LastUartStatus = HAL_ERROR;
        return HAL_ERROR;
    }

    command[0] = ZDT_MOTOR_ADDR;
    command[1] = 0xF6U;
    command[2] = (uint8_t)direction;
    command[3] = (uint8_t)(speed_command >> 8);
    command[4] = (uint8_t)speed_command;
    command[5] = acceleration;
    command[6] = ZDT_SYNC_WAIT;
    command[7] = ZDT_CHECK_BYTE;

    if (ZDT_Send(command, sizeof(command)) != HAL_OK)
    {
        return ZdtMotor_LastUartStatus;
    }
    return ZDT_SyncTrigger();
}

HAL_StatusTypeDef ZDT_MoveRelative(ZdtDirection direction,
                                    uint16_t speed_rpm,
                                    uint8_t acceleration,
                                    uint32_t pulses)
{
    uint8_t command[13];
    uint32_t speed_command;

    if (((direction != ZDT_DIR_CW) && (direction != ZDT_DIR_CCW)) ||
        (speed_rpm == 0U) || (pulses == 0U))
    {
        ZdtMotor_LastUartStatus = HAL_ERROR;
        return HAL_ERROR;
    }

    speed_command = (uint32_t)speed_rpm * ZDT_SPEED_SCALE;
    if (speed_command > 0xFFFFU)
    {
        ZdtMotor_LastUartStatus = HAL_ERROR;
        return HAL_ERROR;
    }

    command[0] = ZDT_MOTOR_ADDR;
    command[1] = 0xFDU;
    command[2] = (uint8_t)direction;
    command[3] = (uint8_t)(speed_command >> 8);
    command[4] = (uint8_t)speed_command;
    command[5] = acceleration;
    command[6] = (uint8_t)(pulses >> 24);
    command[7] = (uint8_t)(pulses >> 16);
    command[8] = (uint8_t)(pulses >> 8);
    command[9] = (uint8_t)pulses;
    command[10] = ZDT_POSITION_RELATIVE;
    command[11] = ZDT_SYNC_WAIT;
    command[12] = ZDT_CHECK_BYTE;

    if (ZDT_Send(command, sizeof(command)) != HAL_OK)
    {
        return ZdtMotor_LastUartStatus;
    }
    return ZDT_SyncTrigger();
}
