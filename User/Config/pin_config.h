#ifndef ROBOT_PIN_CONFIG_H
#define ROBOT_PIN_CONFIG_H

/* Semantic map only; CubeMX-generated GPIO/UART setup remains in Core. */
#define GRAY_MID2_GPIO_PORT_NAME "GPIOD"
#define GRAY_MID2_PIN_NAME       "PD8"
#define GRAY_IN2_GPIO_PORT_NAME  "GPIOD"
#define GRAY_IN2_PIN_NAME        "PD0"
#define GRAY_IN1_GPIO_PORT_NAME  "GPIOD"
#define GRAY_IN1_PIN_NAME        "PD1"
#define GRAY_MID1_GPIO_PORT_NAME "GPIOD"
#define GRAY_MID1_PIN_NAME       "PD3"
#define RZ_IR_GPIO_PORT_NAME     "GPIOD"
#define RZ_IR_PIN_NAME           "PD10"

#define CHASSIS_MOTOR_UART_NAME  "USART3"
#define JY61P_UART_NAME          "USART2"
#define MAIXCAM_UART_NAME        "UART4"
#define COMMAND_UART_NAME        "UART5"
#define SERVO_UART_NAME          "UART7"
#define RFID_UART_NAME           "UART8"
#define TURNTABLE_UART_NAME      "USART6"

#endif /* ROBOT_PIN_CONFIG_H */
