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
#include "ball_sequence.h"
#include "uart_command.h"
#include "servo_action.h"
#include "warehouse_control.h"
#include "round_pillar.h"
#include "stair_sequence.h"

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
  BallSequenceStatus ball_result;
  RoundPillarStatus rz_result = ROUND_PILLAR_OK;
  StairSequenceStatus stair_result = STAIR_SEQUENCE_OK;
  ServoActionStatus servo_result;
  WarehouseStatus warehouse_result;
  ChassisCommand command;

  (void)argument;
  ChassisCommand_Busy = 1U;
  ChassisTask_Ready = 0U;
  ChassisCommand_LastStatus = MOTION_STATUS_IDLE;
  ServoAction_SequenceState = SERVO_SEQUENCE_STARTING;

  MotionControl_Init(&huart3, &huart2);
  osDelay(100);

  /* Warehouse motor is isolated on USART6; an error must not disable the chassis. */
  warehouse_result = WarehouseControl_Init(&huart6);
  (void)warehouse_result;

  /* 发送出发姿态，但不依赖舵控板的完成回传；部分舵控板不提供该帧。 */
  servo_result = ServoAction_StartGroupNoWait(SERVO_ACTION_START_GROUP, 1U);
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

      if (command.type == CHASSIS_CMD_ROTATE)
      {
        result = MotionControl_RotateDeg(command.angle_deg);
      }
      else if (command.type == CHASSIS_CMD_BALL)
      {
        ball_result = BallSequence_Run();
        if ((ball_result == BALL_SEQUENCE_OK) ||
            (ball_result == BALL_SEQUENCE_CANCELED_BY_STOP))
        {
          result = MOTION_STATUS_FINISHED;
          ChassisTask_Ready = 1U;
        }
        else if (ball_result == BALL_SEQUENCE_ERROR_SERVO)
        {
          result = MOTION_ERROR_MOTOR_UART;
          ChassisTask_Ready = 0U;
        }
        else if (ball_result == BALL_SEQUENCE_ERROR_MAIX_TIMEOUT)
        {
          result = MOTION_ERROR_MAIX_TIMEOUT;
          ChassisTask_Ready = 1U;
        }
        else if (ball_result == BALL_SEQUENCE_ERROR_TURNTABLE)
        {
          result = MOTION_ERROR_MOTOR_UART;
          ChassisTask_Ready = 1U;
        }
        else if (ball_result == BALL_SEQUENCE_ERROR_GRAY_ALIGN)
        {
          result = MOTION_ERROR_GRAY_ALIGN;
          ChassisTask_Ready = 1U;
        }
        else
        {
          result = MOTION_ERROR_MAIX_UART;
          ChassisTask_Ready = 1U;
        }
      }
      else if (command.type == CHASSIS_CMD_RZ)
      {
        rz_result = RoundPillar_Run();
        if ((rz_result == ROUND_PILLAR_OK) ||
            (rz_result == ROUND_PILLAR_CANCELED))
        {
          result = MOTION_STATUS_FINISHED;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_IMU)
        {
          result = MOTION_ERROR_IMU_LOST;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_MOTOR)
        {
          result = MOTION_ERROR_MOTOR_UART;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_SERVO)
        {
          result = MOTION_ERROR_MOTOR_UART;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_TURNTABLE)
        {
          result = MOTION_ERROR_MOTOR_UART;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_MAIX_UART)
        {
          result = MOTION_ERROR_MAIX_UART;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_MAIX_TIMEOUT)
        {
          result = MOTION_ERROR_MAIX_TIMEOUT;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_APPROACH_TIMEOUT)
        {
          result = MOTION_ERROR_RZ_TIMEOUT;
          ChassisTask_Ready = 1U;
        }
        else if (rz_result == ROUND_PILLAR_ERROR_ORBIT_TIMEOUT)
        {
          result = MOTION_ERROR_RZ_TIMEOUT;
          ChassisTask_Ready = 1U;
        }
        else
        {
          result = MOTION_ERROR_RZ_TIMEOUT;
          ChassisTask_Ready = 1U;
        }
      }
      else if (command.type == CHASSIS_CMD_STAIR)
      {
        stair_result = StairSequence_Run();
        if ((stair_result == STAIR_SEQUENCE_OK) ||
            (stair_result == STAIR_SEQUENCE_CANCELED_BY_STOP))
        {
          result = MOTION_STATUS_FINISHED;
        }
        else if (stair_result == STAIR_SEQUENCE_ERROR_GRAY_ALIGN)
        {
          result = MOTION_ERROR_GRAY_ALIGN;
        }
        else if (stair_result == STAIR_SEQUENCE_ERROR_IMU)
        {
          result = MOTION_ERROR_IMU_LOST;
        }
        else if (stair_result == STAIR_SEQUENCE_ERROR_MAIX_UART)
        {
          result = MOTION_ERROR_MAIX_UART;
        }
        else
        {
          result = MOTION_ERROR_MOTOR_UART;
        }
        /* STAIR is a retryable test flow; keep the chassis command path open. */
        ChassisTask_Ready = 1U;
      }
      else if (command.type == CHASSIS_CMD_GRAB)
      {
        /* GRAB is the operator's explicit trigger for action group 2 (clamp). */
        result = MOTION_STATUS_FINISHED;
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
      if ((result < MOTION_ERROR_IMU_STARTUP) ||
          (MotionControl_WasStopped() != 0U))
      {
        MotionControl_State = MOTION_STATUS_IDLE;
      }
      if (command.type != CHASSIS_CMD_GRAB)
      {
        ChassisCommand_Busy = 0U;
      }

      if (command.type == CHASSIS_CMD_GRAB)
      {
        ServoAction_SequenceState = SERVO_SEQUENCE_GRAB_RUNNING;
        ChassisCommand_Busy = 1U;
        servo_result = ServoAction_RunGroup(SERVO_ACTION_GRAB_GROUP,
                                             1U,
                                             SERVO_ACTION_GRAB_TIMEOUT_MS);
        if (servo_result == SERVO_ACTION_OK)
        {
          /* Group 2 is the clamp action; only its UART7 0x08 completion turns the table. */
          warehouse_result = WarehouseControl_HandleActionGroup2Completed();
          ServoAction_SequenceState = SERVO_SEQUENCE_RETURN_RUNNING;
          servo_result = ServoAction_RunGroup(SERVO_ACTION_RETURN_GROUP,
                                               1U,
                                               SERVO_ACTION_RETURN_TIMEOUT_MS);
          if (servo_result == SERVO_ACTION_OK)
          {
            ServoAction_SequenceState = SERVO_SEQUENCE_DONE;
            if ((warehouse_result == WAREHOUSE_STATUS_OK) ||
                (warehouse_result == WAREHOUSE_STATUS_CANCELED))
            {
              ChassisCommand_LastStatus = MOTION_STATUS_FINISHED;
            }
            else
            {
              ChassisCommand_LastStatus = MOTION_ERROR_MOTOR_UART;
            }
            ChassisTask_Ready = 1U;
          }
          else
          {
            ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
            ChassisCommand_LastStatus = MOTION_ERROR_MOTOR_UART;
            ChassisTask_Ready = 0U;
          }
        }
        else
        {
          ServoAction_SequenceState = SERVO_SEQUENCE_ERROR;
          ChassisCommand_LastStatus = MOTION_ERROR_MOTOR_UART;
          ChassisTask_Ready = 0U;
        }
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
