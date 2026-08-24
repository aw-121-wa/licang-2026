/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include "motion_control.h"
#include "competition_path.h"
#include "uart_command.h"
#include "servo_action.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartCommandTask */
osThreadId_t UartCommandTaskHandle;
const osThreadAttr_t UartCommandTask_attributes = {
  .name = "UartCommandTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartChassisTask(void *argument);
void StartUartCommandTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  UartCommand_CreateQueues();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of UartCommandTask */
  UartCommandTaskHandle = osThreadNew(StartUartCommandTask, NULL,
                                      &UartCommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  MotionControlStatus result;
  ServoActionStatus servo_result;
  ChassisCommand command;

  (void)argument;
  ChassisCommand_Busy = 1U;
  ChassisTask_Ready = 0U;
  ChassisCommand_Mode = CHASSIS_MODE_IDLE;
  ChassisCommand_LastStatus = MOTION_STATUS_IDLE;
  ServoAction_MotionCompletedCount = 0U;
  ServoAction_SequenceState = SERVO_SEQUENCE_STARTING;

  MotionControl_Init(&huart3, &huart2);
  osDelay(100);

  /* 出发姿态必须完成后，才允许外部发送底盘运动命令。 */
  servo_result = ServoAction_RunGroup(SERVO_ACTION_START_GROUP,
                                       1U,
                                       SERVO_ACTION_START_TIMEOUT_MS);
  if (servo_result != SERVO_ACTION_OK)
  {
    ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
    ChassisTask_Ready = 0U;
    ChassisCommand_Busy = 0U;
    ChassisCommand_LastStatus = MOTION_ERROR_MOTOR_UART;
    for (;;)
    {
      osDelay(1000U);
    }
  }

  result = MotionControl_PrepareForMove();
  ChassisCommand_LastStatus = result;
  CompetitionPath_LastStatus = result;
  if (result < MOTION_ERROR_IMU_STARTUP)
  {
    ServoAction_SequenceState = SERVO_SEQUENCE_WAITING_MOTION;
    ChassisTask_Ready = 1U;
    ChassisCommand_Busy = 0U;
    MotionControl_State = MOTION_STATUS_IDLE;
  }
  else
  {
    ChassisCommand_Busy = 0U;
  }

  for (;;)
  {
    if ((ChassisCommandQueue != 0) &&
        (xQueueReceive(ChassisCommandQueue, &command, portMAX_DELAY) == pdPASS))
    {
      if (ChassisTask_Ready == 0U)
      {
        ChassisCommand_Busy = 0U;
        ChassisCommand_LastStatus = MotionControl_State;
        continue;
      }

      ChassisCommand_Mode = (command.type == CHASSIS_CMD_RUN_PATH) ?
                            CHASSIS_MODE_PATH : CHASSIS_MODE_MANUAL;
      if (command.type == CHASSIS_CMD_RUN_PATH)
      {
        result = CompetitionPath_RunUserPath();
      }
      else
      {
        float angle_deg = 0.0f;
        float cruise_rpm = MOTION_CRUISE_RPM;

        switch (command.type)
        {
        case CHASSIS_CMD_FORWARD:     angle_deg = 0.0f;   break;
        case CHASSIS_CMD_BACKWARD:    angle_deg = 180.0f; break;
        case CHASSIS_CMD_LEFT:        angle_deg = 90.0f;  break;
        case CHASSIS_CMD_RIGHT:       angle_deg = -90.0f; break;
        case CHASSIS_CMD_LEFT_FRONT:  angle_deg = command.angle_deg;
                                      cruise_rpm = MOTION_DIAGONAL_CRUISE_RPM; break;
        case CHASSIS_CMD_RIGHT_FRONT: angle_deg = -command.angle_deg;
                                      cruise_rpm = MOTION_DIAGONAL_CRUISE_RPM; break;
        case CHASSIS_CMD_LEFT_REAR:   angle_deg = 180.0f - command.angle_deg;
                                      cruise_rpm = MOTION_DIAGONAL_CRUISE_RPM; break;
        case CHASSIS_CMD_RIGHT_REAR:  angle_deg = -(180.0f - command.angle_deg);
                                      cruise_rpm = MOTION_DIAGONAL_CRUISE_RPM; break;
        default:                      angle_deg = 0.0f; break;
        }
        result = MotionControl_MovePolarSegmentMm(
            command.distance_mm, angle_deg, 0.0f, cruise_rpm, 0.0f);
      }
      ChassisCommand_LastStatus = result;
      CompetitionPath_LastStatus = result;
      if ((result < MOTION_ERROR_IMU_STARTUP) &&
          (MotionControl_WasStopped() == 0U))
      {
        ServoAction_MotionCompletedCount++;
      }
      if ((result < MOTION_ERROR_IMU_STARTUP) ||
          (MotionControl_WasStopped() != 0U))
      {
        MotionControl_State = MOTION_STATUS_IDLE;
      }
      ChassisCommand_Mode = CHASSIS_MODE_IDLE;
      ChassisCommand_Busy = 0U;

      if (ServoAction_MotionCompletedCount >= 2U)
      {
        ServoAction_SequenceState = SERVO_SEQUENCE_DISK_RUNNING;
        ChassisCommand_Busy = 1U;
        servo_result = ServoAction_RunGroup(SERVO_ACTION_DISK_GROUP,
                                             1U,
                                             SERVO_ACTION_DISK_TIMEOUT_MS);
        if (servo_result == SERVO_ACTION_OK)
        {
          ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
        }
        else
        {
          ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
        }
        /* 圆盘机动作结束后锁定流程，后续底盘指令不再接受。 */
        ChassisTask_Ready = 0U;
        ChassisCommand_Mode = CHASSIS_MODE_IDLE;
        ChassisCommand_Busy = 0U;
      }
    }
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartUartCommandTask */
/* USER CODE END Header_StartUartCommandTask */
void StartUartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartUartCommandTask */
  UartCommand_Init(&huart5);
  UartCommand_Task(argument);
  /* USER CODE END StartUartCommandTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

