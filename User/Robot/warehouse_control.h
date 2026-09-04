#ifndef WAREHOUSE_CONTROL_H
#define WAREHOUSE_CONTROL_H

#include "turntable_control.h"

#define WAREHOUSE_TOTAL_BALLS               6U
#define WAREHOUSE_TURN_AFTER_LAST_BALL      1U
#define WAREHOUSE_ENABLE_SETTLE_MS        200U

typedef enum
{
    WAREHOUSE_STATE_IDLE = 0,
    WAREHOUSE_STATE_WAIT_ARM_GROUP2,
    WAREHOUSE_STATE_TURNTABLE_MOVING,
    WAREHOUSE_STATE_WAIT_NEXT_BALL,
    WAREHOUSE_STATE_FINISHED,
    WAREHOUSE_STATE_ERROR,
    WAREHOUSE_STATE_CANCELED
} WarehouseState;

typedef enum
{
    WAREHOUSE_STATUS_OK = 0,
    WAREHOUSE_STATUS_FINISHED,
    WAREHOUSE_STATUS_CANCELED,
    WAREHOUSE_STATUS_ERROR_INIT,
    WAREHOUSE_STATUS_ERROR_UART,
    WAREHOUSE_STATUS_ERROR_TIMEOUT
} WarehouseStatus;

/* Count means a processed group-2 (clamp) event and its successful one-slot turn. */
extern volatile uint8_t Warehouse_BallCount;
extern volatile uint8_t Warehouse_State;
extern volatile WarehouseStatus Warehouse_LastStatus;

WarehouseStatus WarehouseControl_Init(UART_HandleTypeDef *huart);
WarehouseStatus WarehouseControl_HandleActionGroup2Completed(void);
uint8_t WarehouseControl_IsReadyForAction(void);
uint8_t WarehouseControl_RemainingBallCount(void);
const char *WarehouseControl_StateName(WarehouseState state);
const char *WarehouseControl_StatusName(WarehouseStatus status);

#endif /* WAREHOUSE_CONTROL_H */
