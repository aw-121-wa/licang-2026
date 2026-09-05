#include "state_machine.h"
#include "jy61p.h"
#include "uart_command.h"
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

/* Board-level UART routing belongs to Robot integration, not a device driver. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Jy61P_UartRxCpltCallback(huart);
    MaixCamLink_UartRxCpltCallback(huart);
    UartCommand_UartRxCpltCallback(huart);
    ServoAction_UartRxCpltCallback(huart);
    RFID_UartRxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Jy61P_UartErrorCallback(huart);
    MaixCamLink_UartErrorCallback(huart);
    UartCommand_UartErrorCallback(huart);
    ServoAction_UartErrorCallback(huart);
    RFID_UartErrorCallback(huart);
}
