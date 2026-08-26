#ifndef MAIXCAM_LINK_H
#define MAIXCAM_LINK_H

#include "main.h"

#define MAIXCAM_BAUDRATE                    115200U
#define MAIXCAM_REQUEST_TIMEOUT_MS          10000U

typedef enum
{
    MAIXCAM_COLOR_RED = 1,
    MAIXCAM_COLOR_BLUE = 2
} MaixCamColor;

typedef enum
{
    MAIXCAM_LINK_OK = 0,
    MAIXCAM_LINK_ERROR_ARGUMENT,
    MAIXCAM_LINK_ERROR_UART
} MaixCamLinkStatus;

void MaixCamLink_Init(UART_HandleTypeDef *huart);
MaixCamLinkStatus MaixCamLink_SendRequest(MaixCamColor color);
uint8_t MaixCamLink_TakeReply(void);
void MaixCamLink_UartRxCpltCallback(UART_HandleTypeDef *huart);
void MaixCamLink_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* MAIXCAM_LINK_H */
