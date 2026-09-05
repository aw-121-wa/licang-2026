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
static float physical_yaw;
static uint32_t sample_tick;
static unsigned manual_sample;
static unsigned brake_test, yaw_test;
static unsigned hard_stops, last_command;
static float brake_start_x = 1000, last_speed;
static float peak_wheel_rpm;
static uint32_t sync_tick;
static void advance(uint32_t ms)
{
    const double k = 0.003926990817;
    x += (wheels[0]+wheels[1]+wheels[2]+wheels[3])*0.25f*k*ms;
    y += (-wheels[0]+wheels[1]+wheels[2]-wheels[3])*0.25f*k*ms;
    /* Measured chassis response: old positive wheel omega turns right. */
    if (yaw_test) {
        float delta = (wheels[0]-wheels[1]+wheels[2]-wheels[3])*0.25f*0.001f*ms;
        yaw += delta;
        physical_yaw += delta;
    }
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
    last_command = p[1];
    if (p[1] == 0xF6) {
        /* Physical address map: 1 RF, 2 LF, 3 LR, 4 RR. */
        unsigned i = p[0] == 1 ? 1 : p[0] == 2 ? 0 : p[0]-1;
        float v = (float)((p[3]<<8)|p[4])/10;
        assert(v <= MOTOR_SPEED_LIMIT_RPM);
        if (v > peak_wheel_rpm) peak_wheel_rpm = v;
        queued[i] = p[2] == ((i==0 || i==2) ? 1 : 0) ? v : -v;
    } else if (p[1] == 0xFE) {
        assert(n == 5 && p[0] == 0 && p[2] == 0x98 && p[3] == 0 && p[4] == 0x6B);
        memset(wheels, 0, sizeof(wheels));
        memset(queued, 0, sizeof(queued));
        hard_stops++;
    }
    else if (p[1] == 0xFF) {
        float v = (queued[0]+queued[1]+queued[2]+queued[3])*0.25f;
        if (brake_test && last_speed > v + 0.11f) {
            if (brake_start_x == 1000) brake_start_x = (float)x;
            float allowed = (last_speed <= 20.1f ? 40.0f : 140.0f) * (tick-sync_tick)/1000.0f + 0.11f;
            assert(last_speed-v <= allowed);
        }
        last_speed = v; sync_tick = tick;
        memcpy(wheels, queued, sizeof(wheels));
    }
    return HAL_OK;
}
void Jy61P_Init(UART_HandleTypeDef *h) { (void)h; }
void Jy61P_ResetContinuousYaw(void) { yaw = 0; }
float Jy61P_GetContinuousYaw(void) { return yaw; }
uint32_t Jy61P_GetLastTick(void) { return manual_sample ? sample_tick : tick; }
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
    if (!strcmp(argv[1], "fast")) { distance = 10000; speed = MOTION_CRUISE_RPM; }
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
    if (!strcmp(argv[1], "hard_heading")) { yaw_test = 1; yaw = 3; }
    MotionControl_Init(&h, &h);
    MotionControl_ImuHeadingHoldActive = 1;
    if (!strcmp(argv[1], "heading_damping")) {
        manual_sample = 1;
        tick = sample_tick = 100;
        yaw = 2;
        assert(fabsf(MotionControl_GetHeadingCorrection(450) + 3.7f) < 0.01f);
        tick = sample_tick = 120;
        yaw = 1;
        float damped = MotionControl_GetHeadingCorrection(450);
        assert(isfinite(damped));
        tick += 20; /* Same IMU frame: do not recompute a zero derivative. */
        assert(fabsf(MotionControl_GetHeadingCorrection(450) - damped) < 0.01f);
        tick += 201;
        assert(fabsf(MotionControl_GetHeadingCorrection(450) + 1.85f) < 0.2f);
        MotionControl_ResetHeadingReference();
        assert(fabsf(MotionControl_GetHeadingCorrection(450)) < 0.01f);
        return 0;
    }
    if (!strcmp(argv[1], "rotate_ccw") || !strcmp(argv[1], "rotate_cw")) {
        float target = !strcmp(argv[1], "rotate_ccw") ? 10.0f : -10.0f;
        yaw_test = 1;
        assert(MotionControl_RotateDeg(target) == MOTION_STATUS_FINISHED);
        assert(fabsf(physical_yaw - target) < 0.9f);
        assert(last_command == 0xFE);
        return 0;
    }
    if (!strcmp(argv[1], "stop_protocol")) {
        MotorWheelSpeedsRpmX10 command = {300, 400, -500, -600};
        assert(MotorControl_SetWheelSpeeds(&command) == HAL_OK);
        unsigned before = calls;
        assert(MotionControl_SetBodySpeed(0, 0, 0) == HAL_OK);
        assert(calls == before + 1 && hard_stops == 1 && last_command == 0xFE);
        for (unsigned i = 0; i < 4; ++i) assert(wheels[i] == 0);
        /* A partial cached speed update must not be released by stopping. */
        fail_call = calls + 3;
        assert(MotorControl_SetWheelSpeeds(&command) == HAL_ERROR);
        before = calls;
        assert(MotionControl_SetBodySpeed(0, 0, 0) == HAL_OK);
        assert(calls == before + 1 && last_command == 0xFE);
        /* A failed stop is retried, but is never reported as success. */
        fail_call = calls + 1;
        before = calls;
        assert(MotionControl_SetBodySpeed(0, 0, 0) == HAL_ERROR);
        assert(calls == before + 2 && last_command == 0xFE);
        for (unsigned i = 0; i < 4; ++i) assert(wheels[i] == 0);
        return 0;
    }
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
        assert(fabs(x-2000*FORWARD_DISTANCE_GAIN) < 6);
        assert(wheels[0] == 0 && wheels[3] == 0);
        return 0;
    }
    MotionControlStatus result = MotionControl_MovePolarSegmentMmUntil(
        (uint32_t)distance, angle, 0, speed, 0,
        !strcmp(argv[1], "early") ? early : 0, &hit);
    float actual = (float)sqrt(x*x+y*y);
    distance = MotionControl_TargetDistanceMm;
    printf("%s: result=%u distance=%.3f estimate=%.3f elapsed=%lu target=%.3f\n",
           argv[1], result, actual, MotionControl_TraveledMm, (unsigned long)elapsed, distance);
    assert(fabsf(wheels[0])+fabsf(wheels[1])+fabsf(wheels[2])+fabsf(wheels[3]) == 0);
    if (brake_test) { printf("braking begins at %.1f mm\n", brake_start_x); assert(brake_start_x < distance); }
    assert(hard_stops > 0 && last_command == 0xFE);
    if (!strcmp(argv[1], "fast")) { assert(peak_wheel_rpm >= 449.9f); }
    if (yaw_test) { assert(yaw >= 0 && yaw < 3.0f); }
    if (stop_at) { assert(MotionControl_WasStopped()); assert(actual < 300); }
    else if (offline_at) assert(result == MOTION_ERROR_IMU_LOST);
    else if (fail_call) assert(result == MOTION_ERROR_MOTOR_UART);
    else if (!strcmp(argv[1], "early")) { assert(hit); assert(actual < 300); }
    else {
        assert(result == MOTION_STATUS_FINISHED);
        assert(fabsf(actual-distance) <= 1.0f);
        /* Each encoded wheel may round by 0.05 RPM; allow its accumulated
           lateral component while still checking the actual direction. */
        double quantization_mm = elapsed * 0.003926990817 * 0.1 + 0.05;
        double calibrated_angle = atan2(sin(angle*3.141592653589793/180.0)*LEFT_DISTANCE_GAIN,
                                        cos(angle*3.141592653589793/180.0)*FORWARD_DISTANCE_GAIN);
        assert(fabs(x-actual*cos(calibrated_angle)) < quantization_mm);
        assert(fabs(y-actual*sin(calibrated_angle)) < quantization_mm);
        assert(fabsf(actual-MotionControl_TraveledMm) <= 0.1f);
    }
    return 0;
}
