#include <motion_control.h>
#include "motor_control.h"
#include "jy61p.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

uint32_t test_primask;
static uint32_t tick, elapsed, calls, fail_call;
static float wheels[4], queued[4];
static double x, y;
static uint32_t stop_at, offline_at;
static unsigned tx_ms = 1;
static unsigned jitter;
static float yaw;
static unsigned brake_test, yaw_test, yaw_injected, tail_correction;
static unsigned tail_abort;
static float brake_start_x = 1000, last_speed;
static uint32_t sync_tick;
static void advance(uint32_t ms)
{
    const double k = 0.003926990817;
    x += (wheels[0]+wheels[1]+wheels[2]+wheels[3])*0.25f*k*ms;
    y += (-wheels[0]+wheels[1]+wheels[2]-wheels[3])*0.25f*k*ms;
    if (yaw_test) yaw += (-wheels[0]+wheels[1]-wheels[2]+wheels[3])*0.25f*0.001f*ms;
    tick += ms; elapsed += ms;
    if (stop_at && elapsed >= stop_at) MotionControl_RequestStop();
    assert(elapsed < 120000);
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { advance(ms); }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p,
                                    uint16_t n, uint32_t timeout)
{
    (void)h; (void)n; (void)timeout;
    advance(jitter ? 1 + calls % 4 : tx_ms);
    if (++calls == fail_call) return HAL_ERROR;
    if (p[1] == 0xF6) {
        /* Physical address map: 1 RF, 2 LF, 3 LR, 4 RR. */
        unsigned i = p[0] == 1 ? 1 : p[0] == 2 ? 0 : p[0]-1;
        float v = (float)((p[3]<<8)|p[4])/10;
        assert(v <= 150.0f);
        queued[i] = p[2] == ((i==0 || i==2) ? 1 : 0) ? v : -v;
    } else if (p[1] == 0xFE) queued[p[0] == 1 ? 1 : p[0] == 2 ? 0 : p[0]-1] = 0;
    else if (p[1] == 0xFF) {
        float v = (queued[0]+queued[1]+queued[2]+queued[3])*0.25f;
        float omega = (-queued[0]+queued[1]-queued[2]+queued[3])*0.25f;
        if (brake_test && last_speed > v + 0.11f) {
            if (last_speed > 100 && brake_start_x == 1000) brake_start_x = (float)x;
            float allowed = (last_speed <= 20.1f ? 40.0f : 140.0f) * (tick-sync_tick)/1000.0f + 0.11f;
            assert(last_speed-v <= allowed);
        }
        if (yaw_test && v == 0 && x > 900) {
            if (!yaw_injected) {
                yaw = 1; yaw_injected = 1;
                if (tail_abort == 1) stop_at = elapsed + 50;
                if (tail_abort == 2) offline_at = elapsed + 50;
                if (tail_abort == 3) fail_call = calls + 2;
            }
            if (omega < -0.1f) tail_correction = 1;
            assert(fabsf(omega) <= 3.1f);
        }
        last_speed = v; sync_tick = tick;
        memcpy(wheels, queued, sizeof(wheels));
    }
    return HAL_OK;
}
void Jy61P_Init(UART_HandleTypeDef *h) { (void)h; }
void Jy61P_ResetContinuousYaw(void) {}
float Jy61P_GetContinuousYaw(void) { return yaw; }
uint8_t Jy61P_IsOnline(uint32_t t) { (void)t; return !offline_at || elapsed < offline_at; }
uint8_t Jy61P_WaitData(uint32_t t) { (void)t; return 1; }
static uint8_t early(void) { return elapsed >= 400; }
int main(int argc, char **argv)
{
    UART_HandleTypeDef h = {0};
    float angle = 0, speed = 130, distance = 1000;
    uint8_t hit = 0;
    assert(argc == 2 || argc == 5);
    if (argc == 5) { distance = strtof(argv[2], 0); angle = strtof(argv[3], 0); speed = strtof(argv[4], 0); }
    if (!strcmp(argv[1], "short")) distance = 18;
    if (!strcmp(argv[1], "lateral")) angle = 90;
    if (!strcmp(argv[1], "diagonal")) { angle = 45; speed = 220; }
    if (!strcmp(argv[1], "slow")) speed = 25;
    if (!strcmp(argv[1], "overrun")) tx_ms = 5;
    if (!strcmp(argv[1], "jitter")) jitter = 1;
    if (!strcmp(argv[1], "wrap")) tick = 0xFFFFFF00U;
    if (!strcmp(argv[1], "stop")) stop_at = 400;
    if (!strcmp(argv[1], "offline")) offline_at = 400;
    if (!strcmp(argv[1], "uart")) fail_call = 12;
    if (!strcmp(argv[1], "sync_fail")) fail_call = 5;
    if (!strcmp(argv[1], "heading")) { yaw = 3; angle = 30; speed = 180; }
    if (!strcmp(argv[1], "brake")) brake_test = 1;
    if (!strcmp(argv[1], "settle")) yaw_test = 1;
    if (!strcmp(argv[1], "settle_stop")) { yaw_test = 1; tail_abort = 1; }
    if (!strcmp(argv[1], "settle_offline")) { yaw_test = 1; tail_abort = 2; }
    if (!strcmp(argv[1], "settle_uart")) { yaw_test = 1; tail_abort = 3; }
    MotionControl_Init(&h, &h);
    MotionControl_ImuHeadingHoldActive = 1;
    if (!strcmp(argv[1], "invalid")) {
        assert(MotionControl_MovePolarSegmentMm(1000, 0, 0, INFINITY, 0) == MOTION_ERROR_INVALID_ARGUMENT);
        assert(MotionControl_MovePolarSegmentMm(1000, 0, 0, 0.001f, 0) == MOTION_ERROR_INVALID_ARGUMENT);
        assert(calls == 0);
        return 0;
    }
    if (!strcmp(argv[1], "terminal")) {
        MotionControlStatus first = MotionControl_MovePolarSegmentMm(1000, 0, 0, 130, 30);
        assert(first == MOTION_STATUS_POLAR_MOVE);
        assert(wheels[0] == 30 && wheels[3] == 30);
        assert(MotionControl_MovePolarSegmentMm(1000, 0, 30, 130, 0) == MOTION_STATUS_FINISHED);
        assert(fabs(x-2000) < 6);
        assert(wheels[0] == 0 && wheels[3] == 0);
        return 0;
    }
    MotionControlStatus result = MotionControl_MovePolarSegmentMmUntil(
        (uint32_t)distance, angle, 0, speed, 0,
        !strcmp(argv[1], "early") ? early : 0, &hit);
    float actual = (float)sqrt(x*x+y*y);
    printf("%s: result=%u distance=%.3f estimate=%.3f elapsed=%lu\n",
           argv[1], result, actual, MotionControl_TraveledMm, (unsigned long)elapsed);
    assert(fabsf(wheels[0])+fabsf(wheels[1])+fabsf(wheels[2])+fabsf(wheels[3]) == 0);
    if (brake_test) { printf("braking begins at %.1f mm\n", brake_start_x); assert(brake_start_x < 800); }
    if (yaw_test && !tail_abort) { printf("settled yaw %.3f deg\n", yaw); assert(tail_correction); assert(yaw < 0.9f); }
    if (stop_at) { assert(MotionControl_WasStopped()); assert(actual < (tail_abort ? 1001 : 300)); }
    else if (offline_at) assert(result == MOTION_ERROR_IMU_LOST);
    else if (fail_call) assert(result == MOTION_ERROR_MOTOR_UART);
    else if (!strcmp(argv[1], "early")) { assert(hit); assert(actual < 300); }
    else {
        assert(result == MOTION_STATUS_FINISHED);
        assert(fabsf(actual-distance) <= 1.0f);
        /* Each encoded wheel may round by 0.05 RPM; allow its accumulated
           lateral component while still checking the actual direction. */
        double quantization_mm = elapsed * 0.003926990817 * 0.1 + 0.05;
        assert(fabs(x-actual*cos(angle*3.141592653589793/180.0)) < quantization_mm);
        assert(fabs(y-actual*sin(angle*3.141592653589793/180.0)) < quantization_mm);
        assert(fabsf(actual-MotionControl_TraveledMm) <= 0.1f);
    }
    return 0;
}
