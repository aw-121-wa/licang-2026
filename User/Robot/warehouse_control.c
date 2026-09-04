#include "warehouse_control.h"
#include "cmsis_os.h"
#include "motion_control.h"

volatile uint8_t Warehouse_BallCount = 0U;
volatile uint8_t Warehouse_State = WAREHOUSE_STATE_IDLE;
volatile WarehouseStatus Warehouse_LastStatus = WAREHOUSE_STATUS_ERROR_INIT;

static uint8_t WarehouseControl_StopRequested(void)
{
    return (MotionControl_StopRequested != 0U) ? 1U : 0U;
}

static WarehouseStatus WarehouseControl_FromTurntableStatus(TurntableStatus status)
{
    if (status == TURNTABLE_STATUS_OK)
    {
        return WAREHOUSE_STATUS_OK;
    }
    if (status == TURNTABLE_STATUS_CANCELED)
    {
        return WAREHOUSE_STATUS_CANCELED;
    }
    if (status == TURNTABLE_STATUS_ERROR_TIMEOUT)
    {
        return WAREHOUSE_STATUS_ERROR_TIMEOUT;
    }
    return WAREHOUSE_STATUS_ERROR_UART;
}

WarehouseStatus WarehouseControl_Init(UART_HandleTypeDef *huart)
{
    TurntableStatus turntable_status;

    Warehouse_BallCount = 0U;
    Warehouse_State = WAREHOUSE_STATE_IDLE;
    Warehouse_LastStatus = WAREHOUSE_STATUS_ERROR_INIT;

    turntable_status = Turntable_Init(huart);
    if (turntable_status == TURNTABLE_STATUS_OK)
    {
        turntable_status = Turntable_Enable();
    }
    Warehouse_LastStatus = WarehouseControl_FromTurntableStatus(turntable_status);
    if (Warehouse_LastStatus != WAREHOUSE_STATUS_OK)
    {
        Warehouse_State = WAREHOUSE_STATE_ERROR;
        return Warehouse_LastStatus;
    }

    osDelay(WAREHOUSE_ENABLE_SETTLE_MS);
    Warehouse_State = WAREHOUSE_STATE_WAIT_ARM_GROUP2;
    return Warehouse_LastStatus;
}

WarehouseStatus WarehouseControl_HandleActionGroup2Completed(void)
{
    TurntableStatus turntable_status;

    if ((Warehouse_State != WAREHOUSE_STATE_WAIT_ARM_GROUP2) &&
        (Warehouse_State != WAREHOUSE_STATE_WAIT_NEXT_BALL))
    {
        Warehouse_LastStatus = (Warehouse_State == WAREHOUSE_STATE_FINISHED) ?
                               WAREHOUSE_STATUS_FINISHED : WAREHOUSE_STATUS_ERROR_INIT;
        return Warehouse_LastStatus;
    }
    if (Warehouse_BallCount >= WAREHOUSE_TOTAL_BALLS)
    {
        Warehouse_State = WAREHOUSE_STATE_FINISHED;
        Warehouse_LastStatus = WAREHOUSE_STATUS_FINISHED;
        return Warehouse_LastStatus;
    }

    /* The caller reaches here only after ServoAction_RunGroup(2) has received 0x08/2. */
    if (WarehouseControl_StopRequested() != 0U)
    {
        Warehouse_State = WAREHOUSE_STATE_CANCELED;
        Warehouse_LastStatus = WAREHOUSE_STATUS_CANCELED;
        return Warehouse_LastStatus;
    }
    if ((WAREHOUSE_TURN_AFTER_LAST_BALL == 0U) &&
        ((Warehouse_BallCount + 1U) >= WAREHOUSE_TOTAL_BALLS))
    {
        Warehouse_BallCount++;
        Warehouse_State = WAREHOUSE_STATE_FINISHED;
        Warehouse_LastStatus = WAREHOUSE_STATUS_OK;
        return Warehouse_LastStatus;
    }
    Warehouse_State = WAREHOUSE_STATE_TURNTABLE_MOVING;
    turntable_status = Turntable_MoveOneSlotAndWait(
        WarehouseControl_StopRequested);
    Warehouse_LastStatus = WarehouseControl_FromTurntableStatus(turntable_status);

    if (Warehouse_LastStatus == WAREHOUSE_STATUS_OK)
    {
        Warehouse_BallCount++;
        Warehouse_State = (Warehouse_BallCount >= WAREHOUSE_TOTAL_BALLS) ?
                          WAREHOUSE_STATE_FINISHED : WAREHOUSE_STATE_WAIT_NEXT_BALL;
        return Warehouse_LastStatus;
    }
    if (Warehouse_LastStatus == WAREHOUSE_STATUS_CANCELED)
    {
        Warehouse_State = WAREHOUSE_STATE_CANCELED;
        return Warehouse_LastStatus;
    }

    (void)Turntable_Stop();
    Warehouse_State = WAREHOUSE_STATE_ERROR;
    return Warehouse_LastStatus;
}

uint8_t WarehouseControl_IsReadyForAction(void)
{
    return ((Warehouse_State == WAREHOUSE_STATE_WAIT_ARM_GROUP2) ||
            (Warehouse_State == WAREHOUSE_STATE_WAIT_NEXT_BALL)) ? 1U : 0U;
}

uint8_t WarehouseControl_RemainingBallCount(void)
{
    return (Warehouse_BallCount < WAREHOUSE_TOTAL_BALLS) ?
           (uint8_t)(WAREHOUSE_TOTAL_BALLS - Warehouse_BallCount) : 0U;
}

const char *WarehouseControl_StateName(WarehouseState state)
{
    switch (state)
    {
    case WAREHOUSE_STATE_IDLE:            return "IDLE";
    case WAREHOUSE_STATE_WAIT_ARM_GROUP2: return "WAIT_ARM_G2";
    case WAREHOUSE_STATE_TURNTABLE_MOVING:return "TURNTABLE";
    case WAREHOUSE_STATE_WAIT_NEXT_BALL:  return "WAIT_NEXT";
    case WAREHOUSE_STATE_FINISHED:        return "FINISHED";
    case WAREHOUSE_STATE_ERROR:           return "ERROR";
    case WAREHOUSE_STATE_CANCELED:        return "CANCELED";
    default:                              return "UNKNOWN";
    }
}

const char *WarehouseControl_StatusName(WarehouseStatus status)
{
    switch (status)
    {
    case WAREHOUSE_STATUS_OK:            return "OK";
    case WAREHOUSE_STATUS_FINISHED:      return "FINISHED";
    case WAREHOUSE_STATUS_CANCELED:      return "CANCELED";
    case WAREHOUSE_STATUS_ERROR_INIT:    return "INIT";
    case WAREHOUSE_STATUS_ERROR_UART:    return "UART";
    case WAREHOUSE_STATUS_ERROR_TIMEOUT: return "TIMEOUT";
    default:                             return "UNKNOWN";
    }
}
