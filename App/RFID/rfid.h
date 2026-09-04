#ifndef RFID_H
#define RFID_H

#include "main.h"

#define RFID_BAUDRATE 115200U

void RFID_Init(void);
uint8_t RFID_Read_ID(uint8_t *id);
uint8_t RFID_Get_ID(uint8_t *id);
void RFID_Clear(void);
void RFID_UartRxCpltCallback(UART_HandleTypeDef *huart);
void RFID_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* RFID_H */
