/* User task implementations extracted from CubeMX freertos.c. */
#include "task_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "motion_control.h"
#include "ball_sequence.h"
#include "uart_command.h"
#include "servo_action.h"
#include "warehouse_control.h"
#include "round_pillar.h"
#include "stair_sequence.h"
#include "path_sequence.h"
#include "cangku_task.h"

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
  PathSequenceStatus path_result = PATH_SEQUENCE_OK;
  CangkuSequenceStatus cangku_result = CANGKU_STATUS_OK;
  ServoActionStatus servo_result;
  WarehouseStatus warehouse_result;
  ChassisCommand command;

  (void)argument;
  ChassisCommand_Busy = 1U;
  ChassisTask_Ready = 0U;
  ChassisCommand_LastStatus = MOTION_STATUS_IDLE;
  ServoAction_SequenceState = SERVO_SEQUENCE_STARTING;
  CangkuSequence_Init();

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
      else if (command.type == CHASSIS_CMD_PATH)
      {
        path_result = PathSequence_Run();
        if ((path_result == PATH_SEQUENCE_OK) ||
            (path_result == PATH_SEQUENCE_STATUS_CANCELED))
        {
          result = MOTION_STATUS_FINISHED;
        }
        else if ((path_result == PATH_SEQUENCE_ERROR_MOTION) ||
                 (path_result == PATH_SEQUENCE_ERROR_ROTATE))
        {
          result = PathSequence_LastMotionStatus;
        }
        else if (path_result == PATH_SEQUENCE_ERROR_BALL)
        {
          if (PathSequence_LastBallStatus == BALL_SEQUENCE_ERROR_SERVO)
          {
            result = MOTION_ERROR_MOTOR_UART;
          }
          else if (PathSequence_LastBallStatus == BALL_SEQUENCE_ERROR_MAIX_TIMEOUT)
          {
            result = MOTION_ERROR_MAIX_TIMEOUT;
          }
          else if (PathSequence_LastBallStatus == BALL_SEQUENCE_ERROR_TURNTABLE)
          {
            result = MOTION_ERROR_MOTOR_UART;
          }
          else if (PathSequence_LastBallStatus == BALL_SEQUENCE_ERROR_GRAY_ALIGN)
          {
            result = MOTION_ERROR_GRAY_ALIGN;
          }
          else if (PathSequence_LastBallStatus == BALL_SEQUENCE_ERROR_RFID_TIMEOUT)
          {
            result = MOTION_ERROR_RFID_TIMEOUT;
          }
          else
          {
            result = MOTION_ERROR_MAIX_UART;
          }
        }
        else if (path_result == PATH_SEQUENCE_ERROR_STAIR)
        {
          if (PathSequence_LastStairStatus == STAIR_SEQUENCE_ERROR_GRAY_ALIGN)
          {
            result = MOTION_ERROR_GRAY_ALIGN;
          }
          else if (PathSequence_LastStairStatus == STAIR_SEQUENCE_ERROR_IMU)
          {
            result = MOTION_ERROR_IMU_LOST;
          }
          else if (PathSequence_LastStairStatus == STAIR_SEQUENCE_ERROR_MAIX_UART)
          {
            result = MOTION_ERROR_MAIX_UART;
          }
          else
          {
            result = MOTION_ERROR_MOTOR_UART;
            }
        }
        else if (path_result == PATH_SEQUENCE_ERROR_CANGKU)
        {
          if (PathSequence_LastCangkuStatus == CANGKU_STATUS_ERROR_IMU)
          {
            result = MOTION_ERROR_IMU_LOST;
          }
          else if (PathSequence_LastCangkuStatus == CANGKU_STATUS_ERROR_ROTATE)
          {
            result = MOTION_ERROR_ROTATE_TIMEOUT;
          }
          else if (PathSequence_LastCangkuStatus == CANGKU_STATUS_ERROR_GRAY_ALIGN)
          {
            result = MOTION_ERROR_GRAY_ALIGN;
          }
          else
          {
            result = MOTION_ERROR_MOTOR_UART;
          }
        }
        else if (path_result == PATH_SEQUENCE_ERROR_SERVO)
        {
          /* PATH keeps the servo failure distinct; the command API exposes the UART fault class. */
          result = MOTION_ERROR_MOTOR_UART;
        }
        else
        {
          if (PathSequence_LastRzStatus == ROUND_PILLAR_ERROR_IMU)
          {
            result = MOTION_ERROR_IMU_LOST;
          }
          else if ((PathSequence_LastRzStatus == ROUND_PILLAR_ERROR_MOTOR) ||
                   (PathSequence_LastRzStatus == ROUND_PILLAR_ERROR_SERVO) ||
                   (PathSequence_LastRzStatus == ROUND_PILLAR_ERROR_TURNTABLE))
          {
            result = MOTION_ERROR_MOTOR_UART;
          }
          else if (PathSequence_LastRzStatus == ROUND_PILLAR_ERROR_MAIX_UART)
          {
            result = MOTION_ERROR_MAIX_UART;
          }
          else if (PathSequence_LastRzStatus == ROUND_PILLAR_ERROR_MAIX_TIMEOUT)
          {
            result = MOTION_ERROR_MAIX_TIMEOUT;
          }
          else
          {
            result = MOTION_ERROR_RZ_TIMEOUT;
          }
        }

        /* Keep PATH retryable unless an action-group driver entered ERROR. */
        ChassisTask_Ready =
            (ServoAction_SequenceState == SERVO_SEQUENCE_ERROR) ? 0U : 1U;
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
        else if (ball_result == BALL_SEQUENCE_ERROR_RFID_TIMEOUT)
        {
          result = MOTION_ERROR_RFID_TIMEOUT;
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
      else if (command.type == CHASSIS_CMD_CANGKU)
      {
        cangku_result = CangkuSequence_Run();
        if ((cangku_result == CANGKU_STATUS_OK) ||
            (cangku_result == CANGKU_STATUS_CANCELED))
        {
          result = MOTION_STATUS_FINISHED;
        }
        else if (cangku_result == CANGKU_STATUS_ERROR_IMU)
        {
          result = MOTION_ERROR_IMU_LOST;
        }
        else if (cangku_result == CANGKU_STATUS_ERROR_ROTATE)
        {
          result = MOTION_ERROR_ROTATE_TIMEOUT;
        }
        else if (cangku_result == CANGKU_STATUS_ERROR_GRAY_ALIGN)
        {
          result = MOTION_ERROR_GRAY_ALIGN;
        }
        else if (cangku_result == CANGKU_STATUS_ERROR_SERVO)
        {
          result = MOTION_ERROR_MOTOR_UART;
        }
        else
        {
          result = MOTION_ERROR_MOTOR_UART;
        }
        ChassisTask_Ready =
            (ServoAction_SequenceState == SERVO_SEQUENCE_ERROR) ? 0U : 1U;
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
