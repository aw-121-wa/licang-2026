#ifndef MAIXCAM_LINK_H
#define MAIXCAM_LINK_H

#include "main.h"

#define MAIXCAM_BAUDRATE                    115200U
#define MAIXCAM_REQUEST_TIMEOUT_MS          10000U

typedef enum
{
    MAIXCAM_LINK_OK = 0,
    MAIXCAM_LINK_ERROR_ARGUMENT,
    MAIXCAM_LINK_ERROR_UART
} MaixCamLinkStatus;

extern volatile uint32_t MaixCamLink_TxRequestCount;
extern volatile uint32_t MaixCamLink_RxReplyCount;
extern volatile uint32_t MaixCamLink_InvalidFrameCount;
extern volatile uint32_t MaixCamLink_TimeoutCount;
extern volatile uint32_t MaixCamLink_UartErrorCount;

void MaixCamLink_Init(UART_HandleTypeDef *huart);
MaixCamLinkStatus MaixCamLink_SendRequest(void);
uint8_t MaixCamLink_TakeReply(void);
void MaixCamLink_RecordTimeout(void);
void MaixCamLink_UartRxCpltCallback(UART_HandleTypeDef *huart);
void MaixCamLink_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* MAIXCAM_LINK_H */
