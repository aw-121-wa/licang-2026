#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Current project values centralized without retuning runtime behavior. */

/* Chassis motor and mechanical parameters. */
#define MOTOR_WHEEL_DIAMETER_MM          75U
#define MOTOR_PULSES_PER_REV              3200U
#define MOTOR_DRIVE_GEAR_RATIO_NUM          1U
#define MOTOR_DRIVE_GEAR_RATIO_DEN          1U
#define MOTOR_MOVE_SPEED_RPM              100U
#define MOTOR_MOVE_ACCELERATION             0U
#define MOTOR_SEGMENT_SETTLE_MARGIN_MS    100U
#define MOTOR_SPEED_ACCELERATION            0U
#define MOTOR_SPEED_LIMIT_RPM             150U
#define MOTOR_SPEED_COMMAND_SCALE          10U

/* Body motion parameters. */
#define MOTION_CRUISE_RPM          130.0f
#define MOTION_DIAGONAL_CRUISE_RPM  85.0f
#define LATERAL_FORWARD_COMPENSATION       0.0f

/* Servo action groups and waits. */
#define SERVO_ACTION_BAUDRATE             9600U
#define SERVO_ACTION_START_GROUP             0U
#define SERVO_ACTION_RETURN_GROUP            1U
#define SERVO_ACTION_GRAB_GROUP              2U
#define SERVO_ACTION_PILLAR_CAMERA_GROUP     3U
#define SERVO_ACTION_PILLAR_GRAB_GROUP       4U
#define SERVO_ACTION_START_TIMEOUT_MS     3000U
#define SERVO_ACTION_GRAB_TIMEOUT_MS     15000U
#define SERVO_ACTION_RETURN_TIMEOUT_MS   15000U
#define SERVO_ACTION_PILLAR_CAMERA_TIMEOUT_MS 15000U
#define SERVO_ACTION_PILLAR_GRAB_TIMEOUT_MS   20000U

/* Camera, RFID, BALL and gray alignment. */
#define MAIXCAM_BAUDRATE                 115200U
#define MAIXCAM_REQUEST_TIMEOUT_MS       10000U
#define RFID_BAUDRATE                    115200U
#define BALL_ID_MAX                          9U
#define BALL_GRAB_MAX                        5U
#define BALL_SEQUENCE_ROUND_COUNT       BALL_GRAB_MAX
#define BALL_SEQUENCE_WAIT_PERIOD_MS         1U
#define BALL_SEQUENCE_RFID_TIMEOUT_MS    10000U
#define GRAY_ALIGN_STABLE_MS                50U
#define GRAY_ALIGN_TIMEOUT_MS             5000U
#define GRAY_ALIGN_PERIOD_MS                20U
#define GRAY_ALIGN_APPROACH_RPM           25.0f
#define GRAY_ALIGN_RETREAT_RPM            25.0f

/* RZ approach and orbit parameters. */
#define RZ_PERIOD_MS                        20U
#define RZ_APPROACH_RPM                   25.0f
#define RZ_IR_STABLE_MS                    30U
#define RZ_APPROACH_TIMEOUT_MS           5000U
#define RZ_STOP_SETTLE_MS                  80U
#define RZ_CAMERA_RAISE_WAIT_MS          1000U
#define RZ_GRAB_COUNT                       4U
#define RZ_ORBIT_FORWARD_RPM             62.0f
#define RZ_ORBIT_OMEGA_RPM                49.0f
#define RZ_ORBIT_TARGET_DEG              (352.0f)
#define RZ_ORBIT_TIMEOUT_MS             15000U

/* STAIR parameters. */
#define STAIR_GROUP_0                        0U
#define STAIR_GROUP_5                        5U
#define STAIR_GROUP_6                        6U
#define STAIR_GROUP_7                        7U
#define STAIR_GROUP_8                        8U
#define STAIR_GROUP_9                        9U
#define STAIR_GROUP_10                      10U
#define STAIR_GROUP_11                      11U
#define STAIR_GROUP_12                      12U
#define STAIR_CAMERA_POSE_WAIT_MS          1000U
#define STAIR_SERVO_TIMEOUT_MS             20000U
#define STAIR_VISION_TIMEOUT_MS            1000U
#define STAIR_VISION_POLL_MS                  1U
#define STAIR_FORWARD_RPM                  40.0f
#define STAIR_INITIAL_BACKWARD_MM            18U

/* CANGKU parameters. */
#define CANGKU_MOVE_DISTANCE_MM            200U
#define CANGKU_AFTER_LINE_LEFT_DISTANCE_MM  50U
#define CANGKU_SERVO_TIMEOUT_MS          20000U
#define CANGKU_GROUP_13                     13U
#define CANGKU_GROUP_14                     14U
#define CANGKU_GROUP_15                     15U

/* UART5 command queue parameters. */
#define UART_CMD_BUFFER_SIZE                96U
#define UART_CMD_LINE_QUEUE_LENGTH           4U
#define CHASSIS_COMMAND_QUEUE_LENGTH         4U

/* Turntable parameters independent of its direction enum. */
#define TURNTABLE_UART_BAUDRATE         115200U
#define TURNTABLE_ONE_SLOT_PULSES         1280U
#define TURNTABLE_MOVE_SPEED_RPM            100U
#define TURNTABLE_MOVE_ACCELERATION          0U
#define TURNTABLE_PULSES_PER_REV           3200U
#define TURNTABLE_SETTLE_MARGIN_MS         600U
#define TURNTABLE_MOVE_TIMEOUT_MS         1500U

#endif /* ROBOT_CONFIG_H */
