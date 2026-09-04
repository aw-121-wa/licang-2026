#include "state_machine.h"
#include "usart.h"
#include "servo_action.h"
#include "maixcam_link.h"
#include "rfid.h"
#include "ball_sequence.h"

void RobotUser_Init(void)
{
    ServoAction_Init(&huart7);
    MaixCamLink_Init(&huart4);
    RFID_Init();
    BallSequence_Init();
}
