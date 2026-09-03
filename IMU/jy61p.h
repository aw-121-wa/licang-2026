#ifndef __JY61P_H__
#define __JY61P_H__

#include "main.h"

/* Legacy Jy61P_* API retained for JY60/JY61P compatibility. */
/* Current hardware: JY60 at 9600 baud; standard 11-byte 55 53 angle frame. */
void Jy61P_Init(UART_HandleTypeDef *huart);
void Jy61P_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Jy61P_UartErrorCallback(UART_HandleTypeDef *huart);
void Jy61P_ReceiveData(uint8_t rx_data);

uint8_t Jy61P_IsOnline(uint32_t timeout_ms);
uint8_t Jy61P_WaitData(uint32_t timeout_ms);
float Jy61P_GetYaw(void);
void Jy61P_ResetContinuousYaw(void);
float Jy61P_GetContinuousYaw(void);
uint32_t Jy61P_GetLastTick(void);
uint32_t Jy61P_GetFrameCount(void);
uint32_t Jy61P_GetRxByteCount(void);
uint32_t Jy61P_GetChecksumErrorCount(void);

extern volatile float Roll;
extern volatile float Pitch;
extern volatile float Yaw;
extern volatile float Jy61P_ContinuousYaw;
extern volatile uint8_t Jy61P_UpdateFlag;
extern volatile uint32_t Jy61P_LastUpdateTick;
extern volatile uint32_t Jy61P_FrameCount;
extern volatile uint32_t Jy61P_RxByteCount;
extern volatile uint32_t Jy61P_ChecksumErrorCount;

#endif
