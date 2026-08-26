# 当前需求与验收标准

## 当前比赛动作

1. 等待 JY60/JY61P 输出有效角度帧。
2. 使能四个电机并建立当前车头航向基准。
3. 等待 UART5 现场命令；普通平移、ROT、BALL、GRAB 和 RZ 均由 `ChassisTask` 顺序执行。
4. 平移过程中保持同一航向基准，实时执行统一的航向 PD 修正。
5. 完成动作后保持停止并返回可接收命令状态。

## 控制要求

- 前进、横移和斜线均使用四麦轮运动学解算。
- 行走使用 `F6` 连续速度模式，每20 ms读取一次当前航向并更新四轮速度，实现运动途中实时锁头。
- 指定距离按下发RPM、实际运行时间和75 mm轮径进行软件积分；保持开环，不读取电机位置、到位或其他返回值。
- 起步和停车使用软件速度斜坡，不通过多条短 `FD` 位置命令实现距离。
- 电机命令采用 x42_v1.3 / Emm V5.0 协议和四轴同步触发。
- 任一位置命令未成功进入发送队列时，不得触发该组运动。
- IMU 无有效数据时禁止启动；运动中 IMU 丢失时立即停车，避免在“实时锁头”失效后继续运行。
- 电机串口异常时进入错误状态并尝试停车。
- `main.c` 不放置麦轮公式、PID、距离换算或驱动协议代码；`gray_align.c` 和 `round_pillar.c` 不重复实现麦轮解算。
- `MotionControl_SetBodySpeed()` 是唯一的 BodySpeed 到四轮速度发送入口；`MotionControl_ResetHeadingReference()` 和 `MotionControl_GetHeadingCorrection()` 是唯一的航向锁定入口。
- 航向 PD 统一参数为 `KP=2.0`、`KD=0.15`、死区 `0.15°`、最大修正 `8 RPM`，并保留相对平移速度限幅。
- 纯横移使用 `LATERAL_FORWARD_COMPENSATION` 增加前后方向补偿，初始值为 `0.0f`；该值必须通过 `L 2000`、`R 2000` 实车标定，不用四个轮子 gain 掩盖轨迹偏差。

## UART5 PC/VOFA 调试接口

- UART5 使用 PC12=TX、PD2=RX，115200、8N1、无硬件流控、无 DMA，UART5 IRQ 优先级为 6。
- UART5 只接收 ASCII 命令并返回文本，不得直接发送张大头电机协议。
- UART5 中断只负责单字节接收、行缓冲和投递；所有运动函数只能由 `ChassisTask` 在任务上下文调用。
- `STOP` 通过 `MotionControl_RequestStop()` 进入当前 20 ms 控制周期，不能依赖普通运动队列排队后才生效。
- `BALL` 是无参数的 UART5 ASCII 命令；仅在底盘空闲、仓库转盘已准备好且尚有球位时接受，并运行当前六球批次的剩余 MaixCAM 握手流程。
- 动态路径使用静态 RAM 表，最多 20 段；路径编辑只允许在底盘 IDLE 时进行。
- 当前默认编译路径与用户 RAM 路径分离，`PATH LOAD DEFAULT` 负责载入默认路径。

## FreeRTOS 命令执行

- `main.c` 只负责硬件初始化（包括 UART5）、`osKernelInitialize()`、`MX_FREERTOS_Init()` 和 `osKernelStart()`。
- `ChassisTask` 只初始化一次 `MotionControl`，完成 IMU 等待、四轮使能和锁头基准建立后等待命令队列；所有运动函数只能由该任务在任务上下文执行。
- `MotionControl_PrepareForMove()` 负责启动时 IMU 有效性检查、四轮使能和统一航向基准建立；不再保留独立上电运动测试。
- 正式平移只保留 `MotionControl_MovePolarSegmentMm()`；起步、停车、距离积分和轮速限幅均由 `motion_control` 负责。
- 角度定义为：0°前进，+45°左前，+90°左移，+135°左后，180°后退，-45°右前，-90°右移，-135°右后；距离表示沿归一化轨迹方向的目标长度。

## VOFA 命令

- 运动：`F <mm>`、`B <mm>`、`L <mm>`、`R <mm>`、`LF <mm> <deg>`、`RF <mm> <deg>`、`LR <mm> <deg>`、`RR <mm> <deg>`。
- 旋转：`ROT CCW <deg>`、`ROT CW <deg>`。
- 动作：`BALL`、`GRAB`、`RZ`。
- 控制和查询：`STOP`、`STATUS`、`HELP`。
- 已删除动态 PATH 编辑器、PATH LOAD DEFAULT、旧独立测试和未引用的运动包装接口。

## 构建验收

- 在项目根目录运行 `.vscode/build.ps1` 应生成：
  `MDK-ARM/chassis_motor/chassis_motor.hex`。
- Keil 构建日志必须为 `0 Error(s), 0 Warning(s)`。
- VS Code 打开项目根目录后，`Ctrl+Shift+B` 默认运行同一个 Keil 构建任务。

## 尚需实车验收

- 左移时轮向应为：1号反、2号正、3号正、4号反。
- 校准实际轮径、每转脉冲数、前进增益、横移增益和唯一的 `LATERAL_FORWARD_COMPENSATION`。
- 确认航向修正符号；若偏差被放大，应翻转 `HEADING_CORRECTION_SIGN`。
- 标定开环速度时间积分与实际地面距离的误差，尤其是麦轮横移滑移误差。
- 观察 `MotionControl_PeriodOverrunCount`；正常应保持为0，否则需要降低控制频率或优化串口发送。

## Servo action-group sequence (2026-08-24)

- On boot, UART7 (PE7/PE8, 9600-8-N-1) sends action group 0 (`出发姿态.rob`) once, but does not wait for a completion frame.
- UART5 chassis commands become available after the normal IMU/motor preparation. The operator must wait for the physical start pose to finish before sending `GRAB` or `BALL`, because group 0 completion is no longer observable by the MCU.
- Chassis motion commands are not limited by count. A STOP or motion error does not trigger the arm sequence.
- When the chassis is idle, UART5 command `GRAB` runs action group 2 (`8.25-圆盘机夹.rob`), turns the warehouse one slot only after group 2 really completes, then runs action group 1 (`8.25-圆盘机回位.rob`).
- The three `.rob` files must be downloaded to controller slots 0, 1 and 2; the MCU sends only invocation frames.

## MaixCAM six-ball sequence (2026-08-25)

- UART4 uses PC10=TX, PC11=RX, 115200-8-N-1, no flow control and IRQ reception. MaixCAM must use 3.3 V TTL, crossed TX/RX and common ground.
- `BALL` first runs group 1 (return/recognition posture). Each of its six possible rounds then sends the configured one-byte color request (`1` for red; the current no-argument BALL path selects red), waits no more than 10 s for MaixCAM's valid ASCII line `1`, runs group 2 (clamp), turns the warehouse one slot, then runs group 1 (return). MaixCAM may reply only after a new request, 150 ms of visual settling, a qualified target inside its configured trigger zone, and three consecutive frames; one frame of colour-threshold noise must not trigger a clamp. If manual `GRAB` cycles already consumed slots, `BALL` runs only the remaining count.
- `STATUS` reports the compact fields `STATE`, `IMU`, `YAW`, `HEAD_ERR`, `HEAD_CORR`, `DIST`, `TARGET`, `LAST`, `BALL_STATE`, `BALL_ROUND`, `WAREHOUSE_BALL` and `STOP`; MaixCAM/servo/turntable error handling remains active but its debug counters are not exposed.
- A MaixCAM timeout or UART4 send failure does not move the arm and allows retrying `BALL`; action-group failures retain arm error lock. `STOP` ends a waiting round immediately; a STOP during group 2/turntable still waits for group 1 return to complete, then cancels the rest of the batch.

## BALL 灰度校准（2026-08-25）

- `BALL` 必须先执行灰度校准，再允许动作组 1、MaixCAM 识别和动作组 2。四路从左到右为 `MID2 IN2 IN1 MID1`，实际 STM32 引脚分别为 `PD8 PD0 PD1 PD3`。
- 灰度输入使用上拉，低电平表示压线。唯一成功状态是逻辑 `0 1 1 0`：`IN1/IN2` 同时在线，`MID1/MID2` 同时离线，并且连续稳定 50 ms；`MID1/MID2` 同时在线不能判定成功。
- 进入校准时锁定当前 JY61P 连续航向；校准期间只允许横向移动，不允许灰度状态触发原地旋转。横移过程中使用 `KP=2.0`、`KD=0.15`、最大 8 RPM 的航向 PD 纠偏保持锁定角度。
- `MID1/MID2` 均离线且未达到目标时以 25 RPM 靠近；任一外侧传感器在线时以 25 RPM 反向退出。IN1/IN2 先后在线只影响是否达到目标，不改变航向。5 s 内未达到稳定目标返回灰度校准错误；IMU 失联立即停车并返回 IMU 错误，不降级为纯灰度控制。
- 校准成功后停车并清零 JY61P 连续航向，再进入已有的动作组 1 → MaixCAM → 动作组 2 → 转盘流程。

## RZ 靠桩抓球动作（2026-08-26）

- UART5 无参数命令 `RZ` 必须经 `ChassisCommandQueue` 投递并由 `ChassisTask` 执行。RZ 先完成 PD10 靠桩、锁角、红外稳定确认、现有红外后的极坐标靠近和停车稳定；底盘定位后不再运动。
- RZ 使用 PD10 单红外输入；第一版配置为 GPIO 输入、无上下拉、低电平有效，实际有效电平只允许修改 `RZ_IR_DETECTED_LEVEL`。靠桩阶段保留 30 ms 稳定判断、5 s 超时、STOP 响应和 IMU 在线检查。
- 底盘定位完成后使用 `ServoAction_StartGroupNoWait()` 启动 `SERVO_ACTION_PILLAR_CAMERA_GROUP`（动作组 3）一次；发送失败返回 `ROUND_PILLAR_ERROR_SERVO`，不依赖完成回包，并固定等待 `RZ_CAMERA_RAISE_WAIT_MS=1000 ms` 的机械动作时间后进入 `RoundPillar_OrbitAndGrab()`。
- `RoundPillar_OrbitAndGrab()` 先发送第一个红球请求，再在顺时针 720°（`forward=+63 RPM`、`omega=-49 RPM`）和同轨迹反向 90°（两个量同时反号）的 20 ms 控制循环中非阻塞轮询 `MaixCamLink_TakeReply()`。识别成功立即停车稳定，执行 Group4，完成后保持当前 ContinuousYaw 继续绕桩并发送下一次请求；CW 720°和反向 90°共用 30000 ms 绕桩保护时间。
- 视觉阶段固定使用 `MaixCamLink_SendRequest(MAIXCAM_COLOR_RED)`。每轮必须重新发送请求并等待 `MaixCamLink_TakeReply()` 的当前请求有效回复；不能复用旧回复或一次请求等待四次回复。
- `RZ_GRAB_COUNT` 固定为 4。每轮顺序为“红球请求 → 有效 `1` 回复 → `SERVO_ACTION_PILLAR_GRAB_GROUP`（动作组 4）”；Group4 完整包含夹球、放球和重新架摄像头，Group4 完成后才允许下一轮请求。
- RZ 不调用 `BALL`、`WarehouseControl`、转盘、动作组 1 或动作组 2，不增加 `Warehouse_BallCount`。Group4 一旦开始必须完整执行；Group3 完成后、MaixCAM 等待期间和 Group4 完成后均检查 STOP，取消时不进入下一轮。
- RZ 结果映射为：靠桩超时 `MOTION_ERROR_RZ_TIMEOUT`、舵机错误 `MOTION_ERROR_MOTOR_UART`、MaixCAM UART 错误 `MOTION_ERROR_MAIX_UART`、MaixCAM 超时 `MOTION_ERROR_MAIX_TIMEOUT`；不再保留旧 Orbit 状态。

## 仓库转盘协同（2026-08-25）

- 仓库电机固定使用 USART6：PC6=TX、PC7=RX、115200、8N1、无硬件流控；地址必须为 `ZDT_MOTOR_ADDR = 0x05`。UART5 仍只用于 VOFA，USART3 仍只用于底盘地址 1–4。
- `Turntable_MoveOneSlot()` 必须调用 `ZDT_MoveRelative(TURNTABLE_SLOT_DIRECTION, TURNTABLE_MOVE_SPEED_RPM, TURNTABLE_MOVE_ACCELERATION, 1280U)`，使用相对位置，不得自行拼接协议帧。
- 只能在机械臂动作组 2（圆盘机夹）的真实 UART7 完成反馈之后转盘；动作组命令成功发送不是完成条件。动作组 1（圆盘机回位）或其他组完成不得触发转盘。
- 现阶段未解析转盘驱动器到位反馈，`Turntable_WaitComplete()` 以 1280 脉冲、100 RPM、3200 脉冲/圈计算运行时间并加 600 ms 裕量，1500 ms 后超时。超时或 UART 错误必须执行停止、进入 `WAREHOUSE_STATE_ERROR`，且不得增加球计数。
- `Warehouse_BallCount` 表示“组 2（夹取）已真实完成且对应一格转盘已成功完成”的数量。最大为 6；`WAREHOUSE_TURN_AFTER_LAST_BALL=1U`，因此第 6 球后默认也转一格，总理论相对脉冲为 `6 × 1280 = 7680`。
- 若动作组 2 完成时已经收到 `STOP`，不得再下发新的转盘命令；仍须执行动作组 1 回位后取消本批。转盘等待期间收到 `STOP` 时必须向地址 5 发送停止命令，且不得增加球计数。

## 原地旋转任意角度（2026-08-25）

- 提供 `MotionControl_RotateDeg(float angle_deg)`：正角为逆时针左转，负角为顺时针右转，允许绝对值 `1..360` 度（0 度无效）。
- 必须以 JY61P `ContinuousYaw` 闭环判断旋转角度；旋转期间每 20 ms 检查 IMU 在线和 STOP 请求，IMU 失联、串口发送失败或超过 8000 ms 均安全停机并返回错误。
- 四轮速度必须继续通过 `MecanumKinematics_Solve(0, 0, omega)` 和 `MotorControl_SetWheelSpeeds()` 发送，保持原有四轮同步触发；不改动电机协议、安装方向、麦轮公式、JY61P解析或 FreeRTOS 配置。
- 默认参数：巡航 50 RPM、接近 15 RPM、最小有效 8 RPM；剩余 30 度开始减速、10 度进入低速区；进入 ±0.8 度后发送 0 RPM，连续稳定 5 个控制周期（约100 ms）才算完成。
- 成功完成后调用 `Jy61P_ResetContinuousYaw()`，使后续前进/横移/斜线以旋转后的车头作为新的锁头参考。
- UART5：`ROT CCW <deg>`、`ROT CW <deg>`；旋转直接由 `ChassisTask` 执行，不再经过路径编辑器。
