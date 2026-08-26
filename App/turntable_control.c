#include "turntable_control.h"
#include "cmsis_os.h"

#define TURNTABLE_WAIT_PERIOD_MS            10U

volatile TurntableState Turntable_State = TURNTABLE_STATE_UNINITIALIZED;
volatile TurntableStatus Turntable_LastStatus = TURNTABLE_STATUS_ERROR_ARGUMENT;
volatile uint32_t Turntable_LastExpectedMoveMs = 0U;

static TurntableStatus Turntable_FromHalStatus(HAL_StatusTypeDef status)
{
    return (status == HAL_OK) ? TURNTABLE_STATUS_OK : TURNTABLE_STATUS_ERROR_UART;
}

static uint32_t Turntable_EstimateMoveTimeMs(uint32_t pulses)
{
    uint64_t numerator = (uint64_t)pulses * 60000ULL;
    uint32_t denominator = TURNTABLE_MOVE_SPEED_RPM * TURNTABLE_PULSES_PER_REV;
    uint32_t travel_ms;

    if (denominator == 0U)
    {
        return TURNTABLE_MOVE_TIMEOUT_MS;
    }
    travel_ms = (uint32_t)((numerator + denominator - 1U) / denominator);
    return travel_ms + TURNTABLE_SETTLE_MARGIN_MS;
}

TurntableStatus Turntable_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0)
    {
        Turntable_State = TURNTABLE_STATE_ERROR;
        Turntable_LastStatus = TURNTABLE_STATUS_ERROR_ARGUMENT;
        return Turntable_LastStatus;
    }

    ZDT_Init(huart);
    Turntable_LastExpectedMoveMs = Turntable_EstimateMoveTimeMs(
        TURNTABLE_ONE_SLOT_PULSES);
    Turntable_State = TURNTABLE_STATE_UNINITIALIZED;
    Turntable_LastStatus = TURNTABLE_STATUS_OK;
    return Turntable_LastStatus;
}

TurntableStatus Turntable_Enable(void)
{
    Turntable_LastStatus = Turntable_FromHalStatus(ZDT_Enable());
    Turntable_State = (Turntable_LastStatus == TURNTABLE_STATUS_OK) ?
                      TURNTABLE_STATE_READY : TURNTABLE_STATE_ERROR;
    return Turntable_LastStatus;
}

TurntableStatus Turntable_MoveOneSlot(void)
{
    if (Turntable_State != TURNTABLE_STATE_READY)
    {
        Turntable_LastStatus = TURNTABLE_STATUS_ERROR_ARGUMENT;
        return Turntable_LastStatus;
    }

    Turntable_LastStatus = Turntable_FromHalStatus(
        ZDT_MoveRelative(TURNTABLE_SLOT_DIRECTION,
                         TURNTABLE_MOVE_SPEED_RPM,
                         TURNTABLE_MOVE_ACCELERATION,
                         TURNTABLE_ONE_SLOT_PULSES));
    if (Turntable_LastStatus == TURNTABLE_STATUS_OK)
    {
        Turntable_State = TURNTABLE_STATE_MOVING;
    }
    else
    {
        Turntable_State = TURNTABLE_STATE_ERROR;
    }
    return Turntable_LastStatus;
}

TurntableStatus Turntable_MoveOneSlotAndWait(TurntableCancelCheck cancel_check)
{
    TurntableStatus status = Turntable_MoveOneSlot();

    if (status != TURNTABLE_STATUS_OK)
    {
        return status;
    }
    return Turntable_WaitComplete(cancel_check);
}

TurntableStatus Turntable_WaitComplete(TurntableCancelCheck cancel_check)
{
    uint32_t start_tick;

    if (Turntable_State != TURNTABLE_STATE_MOVING)
    {
        Turntable_LastStatus = TURNTABLE_STATUS_ERROR_ARGUMENT;
        return Turntable_LastStatus;
    }

    start_tick = HAL_GetTick();
    for (;;)
    {
        uint32_t elapsed = HAL_GetTick() - start_tick;

        if ((cancel_check != 0) && (cancel_check() != 0U))
        {
            (void)ZDT_Stop();
            Turntable_State = TURNTABLE_STATE_CANCELED;
            Turntable_LastStatus = TURNTABLE_STATUS_CANCELED;
            return Turntable_LastStatus;
        }
        if (elapsed >= Turntable_LastExpectedMoveMs)
        {
            Turntable_State = TURNTABLE_STATE_READY;
            Turntable_LastStatus = TURNTABLE_STATUS_OK;
            return Turntable_LastStatus;
        }
        if (elapsed >= TURNTABLE_MOVE_TIMEOUT_MS)
        {
            (void)ZDT_Stop();
            Turntable_State = TURNTABLE_STATE_ERROR;
            Turntable_LastStatus = TURNTABLE_STATUS_ERROR_TIMEOUT;
            return Turntable_LastStatus;
        }
        osDelay(TURNTABLE_WAIT_PERIOD_MS);
    }
}

TurntableStatus Turntable_Stop(void)
{
    Turntable_LastStatus = Turntable_FromHalStatus(ZDT_Stop());
    Turntable_State = (Turntable_LastStatus == TURNTABLE_STATUS_OK) ?
                      TURNTABLE_STATE_READY : TURNTABLE_STATE_ERROR;
    return Turntable_LastStatus;
}

uint8_t Turntable_IsReady(void)
{
    return (Turntable_State == TURNTABLE_STATE_READY) ? 1U : 0U;
}

const char *Turntable_StateName(TurntableState state)
{
    switch (state)
    {
    case TURNTABLE_STATE_UNINITIALIZED: return "UNINITIALIZED";
    case TURNTABLE_STATE_READY:         return "READY";
    case TURNTABLE_STATE_MOVING:        return "MOVING";
    case TURNTABLE_STATE_ERROR:         return "ERROR";
    case TURNTABLE_STATE_CANCELED:      return "CANCELED";
    default:                            return "UNKNOWN";
    }
}

const char *Turntable_StatusName(TurntableStatus status)
{
    switch (status)
    {
    case TURNTABLE_STATUS_OK:             return "OK";
    case TURNTABLE_STATUS_ERROR_ARGUMENT: return "ARGUMENT";
    case TURNTABLE_STATUS_ERROR_UART:     return "UART";
    case TURNTABLE_STATUS_ERROR_TIMEOUT:  return "TIMEOUT";
    case TURNTABLE_STATUS_CANCELED:       return "CANCELED";
    default:                              return "UNKNOWN";
    }
}
