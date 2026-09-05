#ifndef RFID_H
#define RFID_H

#include "main.h"
#include "robot_config.h"

/* Yumin V1.0.5 framed UART protocol. UIDs are four bytes in wire order. */
typedef struct
{
    uint32_t rx_count;
    uint32_t invalid_count;
    uint32_t last_error;
    uint32_t frame_count;
    uint32_t tag_count;
    uint32_t uid;
    uint8_t reader_status;
    uint8_t last_byte;
    uint8_t pending;
    uint8_t receiving;
} RfidStatus;

void RFID_Init(void);
void RFID_Poll(void);
void RFID_GetStatus(RfidStatus *status);
uint8_t RFID_Read_ID(uint32_t *id);
uint8_t RFID_Get_ID(uint32_t *id);
void RFID_Clear(void);
void RFID_UartRxCpltCallback(UART_HandleTypeDef *huart);
void RFID_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* RFID_H */
