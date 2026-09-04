#ifndef TURNTABLE_CONTROL_H
#define TURNTABLE_CONTROL_H

#include "cangku_motor.h"
#include "robot_config.h"

/* USART6 uses the standard Emm V5.0/x42 baud rate used by this project. */
#define TURNTABLE_SLOT_DIRECTION       ZDT_DIR_CW

typedef enum
{
    TURNTABLE_STATUS_OK = 0,
    TURNTABLE_STATUS_ERROR_ARGUMENT,
    TURNTABLE_STATUS_ERROR_UART,
    TURNTABLE_STATUS_ERROR_TIMEOUT,
    TURNTABLE_STATUS_CANCELED
} TurntableStatus;

typedef enum
{
    TURNTABLE_STATE_UNINITIALIZED = 0,
    TURNTABLE_STATE_READY,
    TURNTABLE_STATE_MOVING,
    TURNTABLE_STATE_ERROR,
    TURNTABLE_STATE_CANCELED
} TurntableState;

typedef uint8_t (*TurntableCancelCheck)(void);

extern volatile TurntableState Turntable_State;
extern volatile TurntableStatus Turntable_LastStatus;
extern volatile uint32_t Turntable_LastExpectedMoveMs;

TurntableStatus Turntable_Init(UART_HandleTypeDef *huart);
TurntableStatus Turntable_Enable(void);
TurntableStatus Turntable_MoveOneSlot(void);
TurntableStatus Turntable_WaitComplete(TurntableCancelCheck cancel_check);
TurntableStatus Turntable_MoveOneSlotAndWait(TurntableCancelCheck cancel_check);
TurntableStatus Turntable_MoveOneSlotReverseAndWait(
    TurntableCancelCheck cancel_check);
TurntableStatus Turntable_Stop(void);
uint8_t Turntable_IsReady(void);
const char *Turntable_StateName(TurntableState state);
const char *Turntable_StatusName(TurntableStatus status);

#endif /* TURNTABLE_CONTROL_H */
