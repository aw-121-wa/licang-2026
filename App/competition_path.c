#include "competition_path.h"
#include <string.h>

/* The compiled path remains available through PATH LOAD DEFAULT. */
#define DEFAULT_PATH_DIAGONAL_DISTANCE_MM  1800U
#define DEFAULT_PATH_DIAGONAL_ANGLE_DEG    30.0f
#define PATH_MAX_DISTANCE_MM              10000U
#define PATH_TRANSITION_RPM               60.0f
#define PATH_DIRECTION_BLEND_MS           150U

volatile uint8_t CompetitionPath_Started = 0U;
volatile uint8_t CompetitionPath_Finished = 0U;
volatile uint32_t CompetitionPath_CurrentStep = 0U;
volatile MotionControlStatus CompetitionPath_LastStatus = MOTION_STATUS_IDLE;
volatile uint8_t CompetitionPath_UserCount = 0U;
volatile uint8_t CompetitionPath_UserCurrentStep = 0U;

static CompetitionPathSegment competition_path_user[
    COMPETITION_PATH_MAX_SEGMENTS];

static const CompetitionPathSegment competition_path_default[] =
{
    {PATH_MOVE_LEFT_FRONT,
     DEFAULT_PATH_DIAGONAL_DISTANCE_MM,
     DEFAULT_PATH_DIAGONAL_ANGLE_DEG}
};

static uint8_t CompetitionPath_StatusIsError(MotionControlStatus status)
{
    return (status >= MOTION_ERROR_IMU_STARTUP) ? 1U : 0U;
}

static uint8_t CompetitionPath_IsDiagonal(CompetitionPathMotionType type)
{
    return ((type == PATH_MOVE_LEFT_FRONT) ||
            (type == PATH_MOVE_RIGHT_FRONT) ||
            (type == PATH_MOVE_LEFT_REAR) ||
            (type == PATH_MOVE_RIGHT_REAR)) ? 1U : 0U;
}

static uint8_t CompetitionPath_IsRotate(CompetitionPathMotionType type)
{
    return (type == PATH_MOVE_ROTATE) ? 1U : 0U;
}

static float CompetitionPath_ToPolarAngle(const CompetitionPathSegment *segment)
{
    switch (segment->type)
    {
    case PATH_MOVE_FORWARD:      return 0.0f;
    case PATH_MOVE_BACKWARD:     return 180.0f;
    case PATH_MOVE_LEFT:         return 90.0f;
    case PATH_MOVE_RIGHT:        return -90.0f;
    case PATH_MOVE_LEFT_FRONT:   return segment->angle_deg;
    case PATH_MOVE_RIGHT_FRONT:  return -segment->angle_deg;
    case PATH_MOVE_LEFT_REAR:    return 180.0f - segment->angle_deg;
    case PATH_MOVE_RIGHT_REAR:   return -(180.0f - segment->angle_deg);
    default:                     return 0.0f;
    }
}

static MotionControlStatus CompetitionPath_RunSegments(
    const CompetitionPathSegment *segments,
    uint8_t count,
    uint8_t user_path)
{
    MotionControlStatus status;
    uint8_t index = 0U;

    if ((segments == 0) || (count == 0U))
    {
        CompetitionPath_LastStatus = MOTION_ERROR_INVALID_ARGUMENT;
        return CompetitionPath_LastStatus;
    }

    CompetitionPath_Started = 1U;
    CompetitionPath_Finished = 0U;
    CompetitionPath_CurrentStep = 0U;
    CompetitionPath_UserCurrentStep = 0U;
    CompetitionPath_LastStatus = MOTION_STATUS_IDLE;

    /* Preserve the verified non-zero-speed transition for a diagonal + F pair. */
    if ((count >= 2U) &&
        (CompetitionPath_IsDiagonal(segments[0].type) != 0U) &&
        (segments[1].type == PATH_MOVE_FORWARD))
    {
        CompetitionPath_CurrentStep = 1U;
        CompetitionPath_UserCurrentStep = user_path ? 1U : 0U;
        status = MotionControl_MovePolarSegmentMm(
            segments[0].distance_mm,
            CompetitionPath_ToPolarAngle(&segments[0]),
            0.0f,
            MOTION_DIAGONAL_CRUISE_RPM,
            PATH_TRANSITION_RPM);
        CompetitionPath_LastStatus = status;
        if (CompetitionPath_StatusIsError(status) != 0U)
        {
            return status;
        }
        if (MotionControl_WasStopped() != 0U)
        {
            CompetitionPath_Finished = 1U;
            CompetitionPath_LastStatus = MOTION_STATUS_FINISHED;
            return CompetitionPath_LastStatus;
        }

        CompetitionPath_CurrentStep = 2U;
        CompetitionPath_UserCurrentStep = user_path ? 2U : 0U;
        status = MotionControl_MovePolarBlendSegmentMm(
            segments[1].distance_mm,
            CompetitionPath_ToPolarAngle(&segments[0]),
            0.0f,
            PATH_DIRECTION_BLEND_MS,
            PATH_TRANSITION_RPM,
            MOTION_CRUISE_RPM,
            0.0f);
        CompetitionPath_LastStatus = status;
        if (CompetitionPath_StatusIsError(status) != 0U)
        {
            return status;
        }
        if (MotionControl_WasStopped() != 0U)
        {
            CompetitionPath_Finished = 1U;
            CompetitionPath_LastStatus = MOTION_STATUS_FINISHED;
            return CompetitionPath_LastStatus;
        }
        index = 2U;
    }

    for (; index < count; index++)
    {
        float angle_deg = CompetitionPath_ToPolarAngle(&segments[index]);
        float cruise_rpm = (CompetitionPath_IsDiagonal(segments[index].type) != 0U) ?
                           MOTION_DIAGONAL_CRUISE_RPM : MOTION_CRUISE_RPM;

        CompetitionPath_CurrentStep = (uint32_t)index + 1U;
        CompetitionPath_UserCurrentStep = user_path ? (uint8_t)(index + 1U) : 0U;
        if (CompetitionPath_IsRotate(segments[index].type) != 0U)
        {
            /* Rotation is a path boundary; it starts only after prior motion ends. */
            status = MotionControl_RotateDeg(segments[index].angle_deg);
        }
        else
        {
            status = MotionControl_MovePolarSegmentMm(
                segments[index].distance_mm,
                angle_deg,
                0.0f,
                cruise_rpm,
                0.0f);
        }
        CompetitionPath_LastStatus = status;
        if (CompetitionPath_StatusIsError(status) != 0U)
        {
            return status;
        }
        if (MotionControl_WasStopped() != 0U)
        {
            CompetitionPath_Finished = 1U;
            CompetitionPath_LastStatus = MOTION_STATUS_FINISHED;
            return CompetitionPath_LastStatus;
        }
    }

    CompetitionPath_Finished = 1U;
    CompetitionPath_CurrentStep = (uint32_t)count + 1U;
    CompetitionPath_UserCurrentStep = user_path ? count : 0U;
    CompetitionPath_LastStatus = MOTION_STATUS_FINISHED;
    return CompetitionPath_LastStatus;
}

MotionControlStatus CompetitionPath_RunOnce(void)
{
    return CompetitionPath_RunSegments(
        competition_path_default,
        (uint8_t)(sizeof(competition_path_default) /
                  sizeof(competition_path_default[0])),
        0U);
}

MotionControlStatus CompetitionPath_RunUserPath(void)
{
    return CompetitionPath_RunSegments(
        competition_path_user,
        CompetitionPath_UserCount,
        1U);
}

void CompetitionPath_ClearUser(void)
{
    (void)memset(competition_path_user, 0, sizeof(competition_path_user));
    CompetitionPath_UserCount = 0U;
    CompetitionPath_UserCurrentStep = 0U;
}

void CompetitionPath_LoadDefault(void)
{
    (void)memset(competition_path_user, 0, sizeof(competition_path_user));
    (void)memcpy(competition_path_user,
                 competition_path_default,
                 sizeof(competition_path_default));
    CompetitionPath_UserCount = (uint8_t)(sizeof(competition_path_default) /
                                          sizeof(competition_path_default[0]));
    CompetitionPath_UserCurrentStep = 0U;
}

CompetitionPathEditResult CompetitionPath_AddUserSegment(
    CompetitionPathMotionType type,
    uint32_t distance_mm,
    float angle_deg)
{
    if (type > PATH_MOVE_ROTATE)
    {
        return COMPETITION_PATH_EDIT_INVALID;
    }
    if (CompetitionPath_IsRotate(type) != 0U)
    {
        if ((distance_mm != 0U) || (angle_deg == 0.0f) ||
            (angle_deg < -360.0f) || (angle_deg > 360.0f))
        {
            return COMPETITION_PATH_EDIT_INVALID;
        }
    }
    else if ((distance_mm == 0U) || (distance_mm > PATH_MAX_DISTANCE_MM))
    {
        return COMPETITION_PATH_EDIT_INVALID;
    }
    if ((CompetitionPath_IsRotate(type) == 0U) &&
        (CompetitionPath_IsDiagonal(type) != 0U))
    {
        if (!((angle_deg > 0.0f) && (angle_deg <= 90.0f)))
        {
            return COMPETITION_PATH_EDIT_INVALID;
        }
    }
    else if ((CompetitionPath_IsRotate(type) == 0U) && (angle_deg != 0.0f))
    {
        return COMPETITION_PATH_EDIT_INVALID;
    }
    if (CompetitionPath_UserCount >= COMPETITION_PATH_MAX_SEGMENTS)
    {
        return COMPETITION_PATH_EDIT_FULL;
    }

    competition_path_user[CompetitionPath_UserCount].type = type;
    competition_path_user[CompetitionPath_UserCount].distance_mm = distance_mm;
    competition_path_user[CompetitionPath_UserCount].angle_deg = angle_deg;
    CompetitionPath_UserCount++;
    return COMPETITION_PATH_EDIT_OK;
}

uint8_t CompetitionPath_GetUserCount(void)
{
    return CompetitionPath_UserCount;
}

uint8_t CompetitionPath_GetUserSegment(uint8_t index,
                                       CompetitionPathSegment *segment)
{
    if ((segment == 0) || (index >= CompetitionPath_UserCount))
    {
        return 0U;
    }
    *segment = competition_path_user[index];
    return 1U;
}
