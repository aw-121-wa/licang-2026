#ifndef COMPETITION_PATH_H
#define COMPETITION_PATH_H

#include "motion_control.h"

#define COMPETITION_PATH_MAX_SEGMENTS 20U

typedef enum
{
    PATH_MOVE_FORWARD = 0,
    PATH_MOVE_BACKWARD,
    PATH_MOVE_LEFT,
    PATH_MOVE_RIGHT,
    PATH_MOVE_LEFT_FRONT,
    PATH_MOVE_RIGHT_FRONT,
    PATH_MOVE_LEFT_REAR,
    PATH_MOVE_RIGHT_REAR
} CompetitionPathMotionType;

typedef struct
{
    CompetitionPathMotionType type;
    uint32_t distance_mm;
    float angle_deg;
} CompetitionPathSegment;

typedef enum
{
    COMPETITION_PATH_EDIT_OK = 0,
    COMPETITION_PATH_EDIT_FULL,
    COMPETITION_PATH_EDIT_INVALID
} CompetitionPathEditResult;

/* Keil Watch-visible state for the compiled and user-editable paths. */
extern volatile uint8_t CompetitionPath_Started;
extern volatile uint8_t CompetitionPath_Finished;
extern volatile uint32_t CompetitionPath_CurrentStep;
extern volatile MotionControlStatus CompetitionPath_LastStatus;
extern volatile uint8_t CompetitionPath_UserCount;
extern volatile uint8_t CompetitionPath_UserCurrentStep;

MotionControlStatus CompetitionPath_RunOnce(void);
MotionControlStatus CompetitionPath_RunUserPath(void);

void CompetitionPath_ClearUser(void);
void CompetitionPath_LoadDefault(void);
CompetitionPathEditResult CompetitionPath_AddUserSegment(
    CompetitionPathMotionType type,
    uint32_t distance_mm,
    float angle_deg);
uint8_t CompetitionPath_GetUserCount(void);
uint8_t CompetitionPath_GetUserSegment(uint8_t index,
                                       CompetitionPathSegment *segment);

#endif /* COMPETITION_PATH_H */
