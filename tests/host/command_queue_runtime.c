/* Include the real parser to exercise its private submission boundary. */
#include "../../User/Task/uart_command.c"
#include <assert.h>

static ChassisCommand items[4];
static unsigned count, cleared;
static unsigned stopped, stop_on_receive;
volatile ServoActionSequenceState ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
uint8_t WarehouseControl_IsReadyForAction(void) { return 0; }
uint8_t Turntable_IsReady(void) { return 0; }
void MotionControl_ClearStopRequest(void) { cleared++; stopped = 0; }
void MotionControl_RequestStop(void) { stopped = 1; }
BaseType_t xQueueReset(QueueHandle_t q) { (void)q; count = 0; return pdPASS; }
BaseType_t xQueueReceive(QueueHandle_t q, void *p, TickType_t t)
{
    (void)q; (void)t;
    if (!count) return 0;
    *(ChassisCommand *)p = items[0];
    memmove(items, items + 1, (--count) * sizeof(items[0]));
    if (stop_on_receive) { stop_on_receive = 0; UartCommand_StopQueue(); }
    return pdPASS;
}
BaseType_t xQueueSend(QueueHandle_t q, const void *p, TickType_t t)
{
    (void)q; (void)t;
    if (count == 4) return 0;
    items[count++] = *(const ChassisCommand *)p;
    return pdPASS;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) { (void)q; return count; }
int main(void)
{
    ChassisCommand c = {0};
    ChassisCommandQueue = items;
    ChassisTask_Ready = 0;
    ChassisCommand_Busy = 1;
    c.type = CHASSIS_CMD_FORWARD; c.distance_mm = 1000;
    assert(UartCommand_SubmitMotion(&c));
    c.type = CHASSIS_CMD_GRAB;
    assert(UartCommand_SubmitMotion(&c));
    c.type = CHASSIS_CMD_PATH;
    assert(UartCommand_SubmitMotion(&c));
    c.type = CHASSIS_CMD_CANGKU;
    assert(UartCommand_SubmitMotion(&c));
    assert(!UartCommand_SubmitMotion(&c));
    assert(count == 4 && items[0].type == CHASSIS_CMD_FORWARD);
    assert(cleared == 0); /* Enqueue must never cancel an active STOP. */
    UartCommand_StopQueue();
    assert(count == 0 && stopped && ChassisCommand_Busy);
    assert(UartCommand_SubmitMotion(&c));
    assert(stopped); /* New input during cancellation cannot restart the active motion. */
    assert(UartCommand_WaitNext(&c));
    assert(!stopped && cleared == 1 && c.type == CHASSIS_CMD_CANGKU);
    assert(UartCommand_SubmitMotion(&c));
    stop_on_receive = 1; /* STOP between dequeue and claiming the command. */
    assert(!UartCommand_WaitNext(&c));
    assert(stopped && cleared == 1);
    return 0;
}
