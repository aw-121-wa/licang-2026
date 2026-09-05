#include <stdio.h>
#include <string.h>
#include "rfid.h"
#include "usart.h"
#include "ball_sequence.h"
#include "servo_action.h"
#include "maixcam_link.h"
#include "gray_align.h"
#include "warehouse_control.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); return 1; } } while (0)
UART_HandleTypeDef huart8;
uint32_t test_primask;
volatile uint8_t MotionControl_StopRequested;
volatile ServoActionSequenceState ServoAction_SequenceState;
static uint8_t *rx;
static uint32_t tick;
static const char *scenario;
static unsigned clamps, returns, turns, requests;
static uint8_t warehouse_count;
static uint8_t next_id = 1U;
static uint8_t fail_arm;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{
    (void)n;
    if (fail_arm) { fail_arm = 0; return HAL_ERROR; }
    if (h->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
    rx = p;
    h->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}
uint32_t HAL_GetTick(void) { return tick; }
static void feed(uint8_t value)
{
    *rx = value;
    huart8.RxState = HAL_UART_STATE_READY;
    RFID_UartRxCpltCallback(&huart8);
}
static void feed_uid(uint32_t uid)
{
    uint8_t packet[12] = {4, 12, 2, 0x20, 0, 4, 0, 0, 0, 0, 0, 0};
    uint8_t i, sum = 0;
    packet[7] = (uint8_t)(uid >> 24); packet[8] = (uint8_t)(uid >> 16);
    packet[9] = (uint8_t)(uid >> 8); packet[10] = (uint8_t)uid;
    for (i = 0; i < 11; i++) sum ^= packet[i];
    packet[11] = (uint8_t)~sum;
    for (i = 0; i < 12; i++) feed(packet[i]);
}
void osDelay(uint32_t ms)
{
    tick += ms;
    if (BallSequence_State != BALL_SEQUENCE_WAITING_RFID) return;
    if (!strcmp(scenario, "timeout") || !strcmp(scenario, "clamp")) return;
    if (!strcmp(scenario, "stop")) { MotionControl_StopRequested = 1U; return; }
    feed_uid(0xA1230000U + next_id);
}
GrayAlignStatus GrayAlign_Run(void) { return GRAY_ALIGN_OK; }
MaixCamLinkStatus MaixCamLink_SendRequest(MaixCamColor color)
{
    (void)color;
    requests++;
    if (!strcmp(scenario, "retry") && requests == 2U) return MAIXCAM_LINK_ERROR_UART;
    return MAIXCAM_LINK_OK;
}
uint8_t MaixCamLink_TakeReply(void) { return 1U; }
ServoActionStatus ServoAction_RunGroup(uint8_t group, uint16_t repeat, uint32_t timeout)
{
    (void)repeat; (void)timeout;
    if (group == 2U) {
        clamps++;
        next_id = (uint8_t)clamps;
        if (!strcmp(scenario, "duplicate") && clamps == 3U) next_id = 1U;
        if (!strcmp(scenario, "clamp")) feed_uid(0xA1230000U + next_id);
    } else { returns++; }
    return SERVO_ACTION_OK;
}
uint8_t WarehouseControl_RemainingBallCount(void) { return (uint8_t)(6U - warehouse_count); }
WarehouseStatus WarehouseControl_HandleActionGroup2Completed(void)
{
    turns++;
    if (!strcmp(scenario, "turn_fail")) return WAREHOUSE_STATUS_ERROR_UART;
    warehouse_count++;
    return WAREHOUSE_STATUS_OK;
}
int main(int argc, char **argv)
{
    uint32_t id = 0U;
    BallSequenceStatus status;
    if (argc != 2) return 2;
    scenario = argv[1];
    if (!strcmp(scenario, "rearm")) fail_arm = 1;
    RFID_Init();
    BallSequence_Init();
    if (!strcmp(scenario, "rearm")) {
        RfidStatus snapshot;
        RFID_GetStatus(&snapshot); CHECK(!snapshot.receiving);
        RFID_Poll(); RFID_GetStatus(&snapshot); CHECK(snapshot.receiving);
        feed_uid(0x11223344); CHECK(RFID_Read_ID(&id) && id == 0x11223344); return 0;
    }
    if (!strcmp(scenario, "raw")) {
        static const uint8_t sample[] = {4,12,2,0x20,0,4,0,0x45,0x96,0xB7,0x8A,0x3F};
        unsigned i;
        RfidStatus snapshot;
        feed(0); feed('1'); feed(10); CHECK(!RFID_Read_ID(&id));
        for (i = 0; i < sizeof(sample); i++) feed(sample[i]);
        RFID_GetStatus(&snapshot);
        CHECK(snapshot.uid == 0x4596B78AU && snapshot.frame_count == 1 && snapshot.pending);
        CHECK(!RFID_Read_ID(NULL)); CHECK(RFID_Read_ID(&id) && id == 0x4596B78AU);
        CHECK(!RFID_Read_ID(&id));
        test_primask = 1; RFID_Clear(); CHECK(test_primask == 1);
        return 0;
    }
    if (!strcmp(scenario, "first")) {
        feed_uid(0x12345678); feed_uid(0x87654321); CHECK(RFID_Read_ID(&id) && id == 0x12345678); return 0;
    }
    if (!strcmp(scenario, "corrupt")) {
        huart8.ErrorCode = 4; feed_uid(0x12345678); CHECK(!RFID_Read_ID(&id));
        RFID_UartErrorCallback(&huart8); feed_uid(0x87654321); CHECK(RFID_Read_ID(&id) && id == 0x87654321); return 0;
    }
    if (!strcmp(scenario, "frames")) {
        uint8_t bad[] = {4,12,2,0x20,0,4,0,0x45,0x96,0xB7,0x8A,0x3E};
        unsigned i;
        for(i=0;i<sizeof(bad);i++) feed(bad[i]);
        CHECK(!RFID_Read_ID(&id));
        feed(4); feed(255); CHECK(!RFID_Read_ID(&id));
        feed(4); feed(12); tick += 101;
        feed_uid(0xFFFFFFFFU); CHECK(RFID_Read_ID(&id) && id == 0xFFFFFFFFU);
        feed_uid(0); CHECK(RFID_Read_ID(&id) && id == 0);
        return 0;
    }
    if (!strcmp(scenario, "variants")) {
        /* Vendor A1 response, failed auto response and auto UID+block sample. */
        static const uint8_t query[] = {1,12,0xA1,0x20,0,4,0,0x0A,0xDC,0xEF,0xF9,0xB7};
        static const uint8_t failed[] = {1,8,0xA1,0x20,1,0,0,0x76};
        static const uint8_t combined[] = {4,28,4,0x20,0,4,0,0x45,0x96,0xB7,0x8A,
            0,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x29};
        unsigned i;
        for (i=0;i<sizeof(query);i++) feed(query[i]);
        CHECK(RFID_Read_ID(&id) && id == 0x0ADCEFF9);
        for (i=0;i<sizeof(failed);i++) feed(failed[i]);
        CHECK(!RFID_Read_ID(&id));
        for (i=0;i<sizeof(combined);i++) feed(combined[i]);
        CHECK(RFID_Read_ID(&id) && id == 0x4596B78A);
        return 0;
    }
    status = BallSequence_Run();
    if (!strcmp(scenario, "clamp")) CHECK(status == BALL_SEQUENCE_OK && grabbed_ball_count == 5);
    if (!strcmp(scenario, "duplicate")) CHECK(status == BALL_SEQUENCE_ERROR_RFID_TIMEOUT && grabbed_ball_count == 2);
    if (!strcmp(scenario, "turn_fail")) CHECK(status == BALL_SEQUENCE_ERROR_TURNTABLE && grabbed_ball_count == 0 && returns == 2);
    if (!strcmp(scenario, "stop")) CHECK(status == BALL_SEQUENCE_CANCELED_BY_STOP && turns == 0 && returns == 2);
    if (!strcmp(scenario, "timeout")) CHECK(status == BALL_SEQUENCE_ERROR_RFID_TIMEOUT && turns == 0 && returns == 2);
    if (!strcmp(scenario, "retry")) {
        CHECK(status == BALL_SEQUENCE_ERROR_MAIX_UART && grabbed_ball_count == 1);
        CHECK(BallSequence_Run() == BALL_SEQUENCE_OK);
        CHECK(grabbed_ball_count == 5 && grabbed_ball_id[0] == 0xA1230001U && grabbed_ball_id[4] == 0xA1230005U && turns == 5);
    }
    if (!strcmp(scenario, "capacity")) {
        CHECK(status == BALL_SEQUENCE_OK && turns == 5);
        CHECK(BallSequence_Run() == BALL_SEQUENCE_OK && turns == 5 && clamps == 5 && grabbed_ball_count == 5);
    }
    return 0;
}
