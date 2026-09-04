#include "motor_control.h"

#define MOTOR_LF 1U
#define MOTOR_RF 2U
#define MOTOR_LR 3U
#define MOTOR_RR 4U
#define MOTOR_CW 0U
#define MOTOR_CCW 1U
#define DIR_LF_FORWARD MOTOR_CCW
#define DIR_RF_FORWARD MOTOR_CW
#define DIR_LR_FORWARD MOTOR_CCW
#define DIR_RR_FORWARD MOTOR_CW
#define ZDT_CHECK_BYTE 0x6BU
#define ZDT_SYNC_WAIT 0x01U
#define ZDT_POSITION_RELATIVE 0x00U
#define PI_SCALED 314159ULL
#define PI_SCALE 100000ULL

static UART_HandleTypeDef *motor_uart = NULL;
volatile uint32_t MotorControl_TxCount = 0U;
volatile HAL_StatusTypeDef MotorControl_LastUartStatus = HAL_OK;

static HAL_StatusTypeDef Motor_Send(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    if (motor_uart == NULL)
    {
        MotorControl_LastUartStatus = HAL_ERROR;
        return HAL_ERROR;
    }
    status = HAL_UART_Transmit(motor_uart, (uint8_t *)data, len, 100U);
    MotorControl_LastUartStatus = status;
    if (status == HAL_OK) { MotorControl_TxCount++; }
    HAL_Delay(2U);
    return status;
}

static HAL_StatusTypeDef Motor_Enable(uint8_t address)
{
    const uint8_t command[6] = {address, 0xF3U, 0xABU, 0x01U, ZDT_SYNC_WAIT, ZDT_CHECK_BYTE};
    return Motor_Send(command, sizeof(command));
}

static HAL_StatusTypeDef Motor_Position(uint8_t address, uint8_t forward_direction,
                                        int32_t signed_pulses)
{
    uint8_t command[13];
    uint8_t direction = forward_direction;
    uint32_t magnitude;
    if (signed_pulses < 0)
    {
        direction ^= 1U;
        magnitude = (uint32_t)(-(int64_t)signed_pulses);
    }
    else { magnitude = (uint32_t)signed_pulses; }

    command[0] = address;
    command[1] = 0xFDU;
    command[2] = direction;
    {
        uint16_t speed_command = MOTOR_MOVE_SPEED_RPM *
                                 MOTOR_SPEED_COMMAND_SCALE;
        command[3] = (uint8_t)(speed_command >> 8);
        command[4] = (uint8_t)speed_command;
    }
    command[5] = MOTOR_MOVE_ACCELERATION;
    command[6] = (uint8_t)(magnitude >> 24);
    command[7] = (uint8_t)(magnitude >> 16);
    command[8] = (uint8_t)(magnitude >> 8);
    command[9] = (uint8_t)magnitude;
    command[10] = ZDT_POSITION_RELATIVE;
    command[11] = ZDT_SYNC_WAIT;
    command[12] = ZDT_CHECK_BYTE;
    return Motor_Send(command, sizeof(command));
}

static HAL_StatusTypeDef Motor_Speed(uint8_t address,
                                     uint8_t forward_direction,
                                     int16_t signed_rpm_x10)
{
    uint8_t command[8];
    uint8_t direction = forward_direction;
    uint16_t magnitude;

    if (signed_rpm_x10 < 0)
    {
        direction ^= 1U;
        magnitude = (uint16_t)(-(int32_t)signed_rpm_x10);
    }
    else
    {
        magnitude = (uint16_t)signed_rpm_x10;
    }
    if (magnitude > (MOTOR_SPEED_LIMIT_RPM * MOTOR_SPEED_COMMAND_SCALE))
    {
        magnitude = MOTOR_SPEED_LIMIT_RPM * MOTOR_SPEED_COMMAND_SCALE;
    }

    command[0] = address;
    command[1] = 0xF6U;
    command[2] = direction;
    command[3] = (uint8_t)(magnitude >> 8);
    command[4] = (uint8_t)magnitude;
    command[5] = MOTOR_SPEED_ACCELERATION;
    command[6] = ZDT_SYNC_WAIT;
    command[7] = ZDT_CHECK_BYTE;
    return Motor_Send(command, sizeof(command));
}

static HAL_StatusTypeDef Motor_Stop(uint8_t address)
{
    const uint8_t command[5] = {address, 0xFEU, 0x98U, ZDT_SYNC_WAIT, ZDT_CHECK_BYTE};
    return Motor_Send(command, sizeof(command));
}

static HAL_StatusTypeDef Motor_SyncTrigger(void)
{
    const uint8_t command[4] = {0x00U, 0xFFU, 0x66U, ZDT_CHECK_BYTE};
    return Motor_Send(command, sizeof(command));
}

static uint32_t Motor_AbsolutePulses(int32_t pulses)
{
    return (pulses < 0) ? (uint32_t)(-(int64_t)pulses) : (uint32_t)pulses;
}

static void Motor_WaitSegment(uint32_t pulses)
{
    uint64_t numerator = (uint64_t)pulses * 60000ULL;
    uint32_t denominator = MOTOR_MOVE_SPEED_RPM * MOTOR_PULSES_PER_REV;
    uint32_t move_ms = (uint32_t)((numerator + denominator - 1U) / denominator);
    HAL_Delay(move_ms + MOTOR_SEGMENT_SETTLE_MARGIN_MS);
}

void MotorControl_Init(UART_HandleTypeDef *huart)
{
    motor_uart = huart;
    MotorControl_TxCount = 0U;
    MotorControl_LastUartStatus = HAL_OK;
}

HAL_StatusTypeDef MotorControl_EnableAll(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    if (Motor_Enable(MOTOR_LF) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_Enable(MOTOR_RF) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_Enable(MOTOR_LR) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_Enable(MOTOR_RR) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_SyncTrigger() != HAL_OK) { status = HAL_ERROR; }
    return status;
}

HAL_StatusTypeDef MotorControl_StopAll(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    if (Motor_Stop(MOTOR_LF) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_Stop(MOTOR_RF) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_Stop(MOTOR_LR) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_Stop(MOTOR_RR) != HAL_OK) { status = HAL_ERROR; }
    if (Motor_SyncTrigger() != HAL_OK) { status = HAL_ERROR; }
    return status;
}

uint32_t MotorControl_DistanceMmToPulses(uint32_t distance_mm)
{
    uint64_t numerator = (uint64_t)distance_mm * MOTOR_PULSES_PER_REV;
    uint64_t denominator;
    numerator *= (uint64_t)MOTOR_DRIVE_GEAR_RATIO_NUM * PI_SCALE;
    denominator = (uint64_t)MOTOR_WHEEL_DIAMETER_MM * PI_SCALED;
    denominator *= MOTOR_DRIVE_GEAR_RATIO_DEN;
    return (uint32_t)((numerator + denominator / 2ULL) / denominator);
}

HAL_StatusTypeDef MotorControl_MoveWheels(const MotorWheelPulses *pulses)
{
    uint32_t maximum;
    uint32_t value;
    if (pulses == NULL) { return HAL_ERROR; }

    /* Never trigger a partially queued four-wheel movement. */
    if (Motor_Position(MOTOR_LF, DIR_LF_FORWARD, pulses->front_left) != HAL_OK) { return HAL_ERROR; }
    if (Motor_Position(MOTOR_RF, DIR_RF_FORWARD, pulses->front_right) != HAL_OK) { return HAL_ERROR; }
    if (Motor_Position(MOTOR_LR, DIR_LR_FORWARD, pulses->rear_left) != HAL_OK) { return HAL_ERROR; }
    if (Motor_Position(MOTOR_RR, DIR_RR_FORWARD, pulses->rear_right) != HAL_OK) { return HAL_ERROR; }
    if (Motor_SyncTrigger() != HAL_OK) { return HAL_ERROR; }

    maximum = Motor_AbsolutePulses(pulses->front_left);
    value = Motor_AbsolutePulses(pulses->front_right); if (value > maximum) { maximum = value; }
    value = Motor_AbsolutePulses(pulses->rear_left); if (value > maximum) { maximum = value; }
    value = Motor_AbsolutePulses(pulses->rear_right); if (value > maximum) { maximum = value; }
    Motor_WaitSegment(maximum);
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_SetWheelSpeeds(
    const MotorWheelSpeedsRpmX10 *speeds)
{
    if (speeds == NULL) { return HAL_ERROR; }

    /* Queue all four F6 commands; do not trigger a partial speed update. */
    if (Motor_Speed(MOTOR_LF, DIR_LF_FORWARD, speeds->front_left) != HAL_OK) { return HAL_ERROR; }
    if (Motor_Speed(MOTOR_RF, DIR_RF_FORWARD, speeds->front_right) != HAL_OK) { return HAL_ERROR; }
    if (Motor_Speed(MOTOR_LR, DIR_LR_FORWARD, speeds->rear_left) != HAL_OK) { return HAL_ERROR; }
    if (Motor_Speed(MOTOR_RR, DIR_RR_FORWARD, speeds->rear_right) != HAL_OK) { return HAL_ERROR; }
    return Motor_SyncTrigger();
}
