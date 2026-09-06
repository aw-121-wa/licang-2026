#include "../../User/Algorithm/motion_control.h"
#include "path_sequence.h"
#include "servo_action.h"
#include <assert.h>
#include <math.h>
#include <string.h>

typedef struct
{
    uint32_t distance_mm;
    float angle_deg;
    float cruise_rpm;
    uint8_t rotate;
} RecordedAction;

static RecordedAction actions[16];
static uint8_t action_count;
volatile MotionControlStatus MotionControl_State;
volatile uint8_t MotionControl_StopRequested;
volatile uint8_t MotionControl_StoppedByRequest;
volatile ServoActionSequenceState ServoAction_SequenceState;

HAL_StatusTypeDef MotorControl_StopAll(void) { return HAL_OK; }
HAL_StatusTypeDef MotionControl_SetBodySpeed(float forward, float left, float omega)
{
    (void)forward; (void)left; (void)omega;
    return HAL_OK;
}
uint8_t MotionControl_WasStopped(void) { return MotionControl_StoppedByRequest; }
MotionControlStatus MotionControl_MovePolarSegmentMm(uint32_t distance_mm,
    float angle_deg, float start_rpm, float cruise_rpm, float end_rpm)
{
    (void)start_rpm; (void)end_rpm;
    actions[action_count++] = (RecordedAction){distance_mm, angle_deg, cruise_rpm, 0U};
    return MOTION_STATUS_FINISHED;
}
MotionControlStatus MotionControl_RotateDeg(float angle_deg)
{
    actions[action_count++] = (RecordedAction){0U, angle_deg, 0.0f, 1U};
    return MOTION_STATUS_FINISHED;
}
uint8_t WarehouseControl_IsReadyForAction(void) { return 1U; }
BallSequenceStatus BallSequence_Run(void) { return BALL_SEQUENCE_OK; }
RoundPillarStatus RoundPillar_Run(void) { return ROUND_PILLAR_OK; }
StairSequenceStatus StairSequence_Run(void) { return STAIR_SEQUENCE_OK; }
CangkuSequenceStatus CangkuSequence_Run(void) { return CANGKU_STATUS_OK; }
ServoActionStatus ServoAction_RunGroup(uint8_t group, uint16_t repeat, uint32_t timeout)
{
    (void)group; (void)repeat; (void)timeout;
    return SERVO_ACTION_OK;
}

#include "../../User/Robot/path_sequence.c"

static void ExpectAction(uint8_t index, uint32_t distance_mm,
                         float angle_deg, float cruise_rpm, uint8_t rotate)
{
    assert(actions[index].distance_mm == distance_mm);
    assert(fabsf(actions[index].angle_deg - angle_deg) < 0.01f);
    assert(fabsf(actions[index].cruise_rpm - cruise_rpm) < 0.01f);
    assert(actions[index].rotate == rotate);
}

int main(void)
{
    assert(PathSequence_RunHome() == PATH_SEQUENCE_ERROR_HOME_NOT_READY);
    assert(PathSequence_Run() == PATH_SEQUENCE_OK);
    assert(PathSequence_IsHomeReady() == 1U);
    assert(action_count == 6U);
    assert(PathSequence_RunHome() == PATH_SEQUENCE_OK);
    assert(PathSequence_IsHomeReady() == 0U);
    assert(action_count == 12U);
    ExpectAction(6U, 1360U, 180.0f, MOTION_CRUISE_RPM, 0U);
    ExpectAction(7U, 1100U, -90.0f, MOTION_CRUISE_RPM, 0U);
    ExpectAction(8U, 0U, -178.0f, 0.0f, 1U);
    ExpectAction(9U, 0U, -178.0f, 0.0f, 1U);
    ExpectAction(10U, 3850U, 0.0f, MOTION_CRUISE_RPM, 0U);
    ExpectAction(11U, 550U, 90.0f, MOTION_CRUISE_RPM, 0U);

    assert(PathSequence_RunHome() == PATH_SEQUENCE_ERROR_HOME_NOT_READY);
    assert(PathSequence_Run() == PATH_SEQUENCE_OK);
    assert(PathSequence_IsHomeReady() == 1U);
    PathSequence_InvalidateHome();
    assert(PathSequence_IsHomeReady() == 0U);
    assert(PathSequence_RunHome() == PATH_SEQUENCE_ERROR_HOME_NOT_READY);
    return 0;
}
