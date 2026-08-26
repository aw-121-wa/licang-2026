#ifndef CANGKU_MOTOR_H
#define CANGKU_MOTOR_H

#include "main.h"

/* The warehouse turntable is on USART6 only; chassis motors remain 1..4. */
#define ZDT_MOTOR_ADDR                 0x05U

/* Emm V5.0/x42 direction encoding. Keep the direction choice in turntable_control.h. */
typedef enum
{
    ZDT_DIR_CW = 0U,
    ZDT_DIR_CCW = 1U
} ZdtDirection;

extern volatile uint32_t ZdtMotor_TxCount;
extern volatile HAL_StatusTypeDef ZdtMotor_LastUartStatus;

void ZDT_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef ZDT_Enable(void);
HAL_StatusTypeDef ZDT_Disable(void);
HAL_StatusTypeDef ZDT_Stop(void);
HAL_StatusTypeDef ZDT_RunSpeed(ZdtDirection direction,
                                uint16_t speed_rpm,
                                uint8_t acceleration);
HAL_StatusTypeDef ZDT_MoveRelative(ZdtDirection direction,
                                    uint16_t speed_rpm,
                                    uint8_t acceleration,
                                    uint32_t pulses);

#endif /* CANGKU_MOTOR_H */
