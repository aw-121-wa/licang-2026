# 当前需求与验收标准

## 当前比赛动作

1. 等待 JY60/JY61P 输出有效角度帧。
2. 使能四个电机并建立当前车头航向基准。
3. 以左前 `+30°` 方向斜线运行 1800 mm，起始速度为 0，最后软减速到 0。
4. 运动过程中保持同一航向基准，实时执行锁头修正。
5. 完成路径后保持停止。

## 控制要求

- 前进和横移均使用四麦轮运动学解算。
- 行走使用 `F6` 连续速度模式，每20 ms读取一次当前航向并更新四轮速度，实现运动途中实时锁头。
- 指定距离按下发RPM、实际运行时间和75 mm轮径进行软件积分；保持开环，不读取电机位置、到位或其他返回值。
- 起步和停车使用软件速度斜坡，不通过多条短 `FD` 位置命令实现距离。
- 电机命令采用 x42_v1.3 / Emm V5.0 协议和四轴同步触发。
- 任一位置命令未成功进入发送队列时，不得触发该组运动。
- IMU 无有效数据时禁止启动；运动中 IMU 丢失时立即停车，避免在“实时锁头”失效后继续运行。
- 电机串口异常时进入错误状态并尝试停车。
- `main.c` 不放置麦轮公式、PID、距离换算或驱动协议代码。

## UART5 PC/VOFA 调试接口

- UART5 使用 PC12=TX、PD2=RX，115200、8N1、无硬件流控、无 DMA，UART5 IRQ 优先级为 6。
- UART5 只接收 ASCII 命令并返回文本，不得直接发送张大头电机协议。
- UART5 中断只负责单字节接收、行缓冲和投递；所有运动函数只能由 `ChassisTask` 在任务上下文调用。
- `STOP` 通过 `MotionControl_RequestStop()` 进入当前 20 ms 控制周期，不能依赖普通运动队列排队后才生效。
- `BALL` 是无参数的 UART5 ASCII 命令；仅在底盘空闲、仓库转盘已准备好且尚有球位时接受，并运行当前六球批次的剩余 MaixCAM 握手流程。
- 动态路径使用静态 RAM 表，最多 20 段；路径编辑只允许在底盘 IDLE 时进行。
- 当前默认编译路径与用户 RAM 路径分离，`PATH LOAD DEFAULT` 负责载入默认路径。

## FreeRTOS 比赛路径模块

- `main.c` 只负责硬件初始化（包括 UART5）、`osKernelInitialize()`、`MX_FREERTOS_Init()` 和 `osKernelStart()`。
- `ChassisTask` 只初始化一次 `MotionControl`，在任务中完成 IMU 等待、四轮使能和锁头基准建立后等待命令队列；普通运动和 `CompetitionPath_RunUserPath()` 均只能由该任务执行，不得无限重复。
- 比赛路径位于 `App/competition_path.c/.h`，只编排现有 `MotionControl` 公共接口；电机协议、麦轮解算、IMU 数据解析和距离积分继续由原模块负责。
- 路径步骤之间使用 `osDelay()`，不在应用层直接发送电机命令或调用 UART HAL。
- `MotionControl_PrepareForMove()` 仅复用原有 IMU 有效性检查、四轮使能和航向基准建立流程，由 `ChassisTask` 在调度器启动后调用；原 `MotionControl_RunDefaultSequence()` 仍保留用于底盘单独测试。
- `MotionControl_MovePolarSegmentMm()` 和 `MotionControl_MovePolarBlendSegmentMm()` 只扩展连续段速度/方向轨迹，原有单段运动 API 必须继续保留。

## 任意角度斜线运动

- `MotionControl_MovePolarMm(distance_mm, angle_deg)` 支持 -180°～+180° 的任意平移角度。
- 角度定义为：0°前进，+45°左前，+90°左移，+135°左后，180°后退，-45°右前，-90°右移，-135°右后。
- `distance_mm` 表示沿该方向的实际轨迹长度，不是前进和横移分量的简单相加。
- `MotionControl_MoveMm(forward_mm, left_mm)` 保留原接口，并允许两个分量同时非零；目标距离按二维向量长度计算。
- 四个包装接口接收 0°～90° 的相对角度：`MoveLeftFrontMm`、`MoveRightFrontMm`、`MoveLeftRearMm`、`MoveRightRearMm`。
- 斜线运动使用 `MOTION_DIAGONAL_CRUISE_RPM`，当前为 85 RPM；纯轴向运动继续使用 100 RPM。
- 麦轮限幅后的比例必须进入距离积分，避免 45° 或加入锁头修正后软件距离与实际下发速度不一致。

## 独立底盘测试流程

- `DIAGONAL_TEST_ENABLE=1` 时，上电流程为：等待 IMU、使能四轮、等待 500 ms、建立航向基准、等待 100 ms、执行一次测试平移、停车。
- `DIAGONAL_TEST_DISTANCE_MM`、`DIAGONAL_TEST_ANGLE_DEG` 和 `DIAGONAL_TEST_DIRECTION` 位于 `Motor/motion_control.h`，用于现场快速切换测试距离、角度和左前/右前/左后/右后方向。
- `DIAGONAL_TEST_ENABLE` 只影响 `MotionControl_RunDefaultSequence()` 独立测试，不影响正式 `CompetitionPath_RunOnce()` 路径。

## VOFA 命令

- 运动：`F <mm>`、`B <mm>`、`L <mm>`、`R <mm>`、`LF <mm> <deg>`、`RF <mm> <deg>`、`LR <mm> <deg>`、`RR <mm> <deg>`。
- 控制和查询：`STOP`、`STATUS`、`HELP`。
- 动态路径：`PATH CLEAR`、`PATH ADD ...`、`PATH SHOW`、`PATH RUN`、`PATH LOAD DEFAULT`。
- 当路径首段为斜线、第二段为前进时，执行器复用现有非零末速度和 150 ms 方向 Blend 接口，不在两段之间人为停到 0。

## 构建验收

- 在项目根目录运行 `.vscode/build.ps1` 应生成：
  `MDK-ARM/chassis_motor/chassis_motor.hex`。
- Keil 构建日志必须为 `0 Error(s), 0 Warning(s)`。
- VS Code 打开项目根目录后，`Ctrl+Shift+B` 默认运行同一个 Keil 构建任务。

## 尚需实车验收

- 左移时轮向应为：1号反、2号正、3号正、4号反。
- 校准实际轮径、每转脉冲数、前进增益、横移增益和四轮独立增益。
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
- `STATUS` reports `BALL_STATE`, `BALL_STATUS`, `BALL_ROUND`, `MAIX_TX`, `MAIX_RX`, `MAIX_INVALID`, `MAIX_TIMEOUT` and `MAIX_UART_ERR`.
- A MaixCAM timeout or UART4 send failure does not move the arm and allows retrying `BALL`; action-group failures retain arm error lock. `STOP` ends a waiting round immediately; a STOP during group 2/turntable still waits for group 1 return to complete, then cancels the rest of the batch.

## BALL 灰度校准（2026-08-25）

- `BALL` 必须先执行灰度校准，再允许动作组 1、MaixCAM 识别和动作组 2。四路从左到右为 `MID2 IN2 IN1 MID1`，实际 STM32 引脚分别为 `PD8 PD0 PD1 PD3`。
- 灰度输入使用上拉，低电平表示压线。唯一成功状态是逻辑 `0 1 1 0`：`IN1/IN2` 同时在线，`MID1/MID2` 同时离线，并且连续稳定 50 ms；`MID1/MID2` 同时在线不能判定成功。
- 进入校准时锁定当前 JY61P 连续航向；校准期间只允许横向移动，不允许灰度状态触发原地旋转。横移过程中使用 `KP=2.0`、`KD=0.15`、最大 8 RPM 的航向 PD 纠偏保持锁定角度。
- `MID1/MID2` 均离线且未达到目标时以 25 RPM 靠近；任一外侧传感器在线时以 25 RPM 反向退出。IN1/IN2 先后在线只影响是否达到目标，不改变航向。5 s 内未达到稳定目标返回灰度校准错误；IMU 失联立即停车并返回 IMU 错误，不降级为纯灰度控制。
- 校准成功后停车并清零 JY61P 连续航向，再进入已有的动作组 1 → MaixCAM → 动作组 2 → 转盘流程。

## RZ 绕桩动作（2026-08-25）

- UART5 新增无参数命令 `RZ`，必须经 `ChassisCommandQueue` 投递并由 `ChassisTask` 执行；不得触发机械臂、BALL、MaixCAM 或转盘。
- RZ 使用 PD10 单红外输入。第一版配置为 GPIO 输入、无上下拉、低电平有效；有效电平必须集中在 `RZ_IR_DETECTED_LEVEL`，实车若相反只允许修改该定义。
- RZ 开始时检查 JY61P 在线并清零 ContinuousYaw，随后锁定 0°，只向左横移寻找 PD10；PD10 连续检测 30 ms 才确认，找桩超时为 5 s。找桩阶段使用 `KP=2.0`、`KD=0.15`、死区 0.15°、最大航向修正 8 RPM。
- 红外触发后不重新找桩，使用 `MotionControl_MovePolarSegmentMm(60, -90°, 0, 20, 0)` 继续右移 60 mm；该阶段不得重置 Yaw，且必须响应 STOP 与 IMU 失联。
- 绕桩开始前仅再清零一次 ContinuousYaw。顺时针 360°固定发送 `forward=-40 RPM`、`left=0`、`omega=-60 RPM`，以 ContinuousYaw `<= -360°` 为完成条件；绕桩总超时为 15 s。
- 顺时针 360°后必须停车并等待稳定，但禁止清零 Yaw。读取实际稳定角 `reverse_start_yaw`，反向目标为 `reverse_start_yaw + 90°`，随后发送 `forward=+40 RPM`、`left=0`、`omega=+60 RPM`，达到目标后停车、等待稳定、清零 Yaw 并完成。禁止用 `MotionControl_RotateDeg(90°)` 替代。
- RZ 取消或失败不永久锁死 `ChassisTask`；RZ 完成按一次独立底盘动作统计，取消不计入动作完成计数。

## 仓库转盘协同（2026-08-25）

- 仓库电机固定使用 USART1：PA9=TX、PA10=RX、115200、8N1、无硬件流控；地址必须为 `ZDT_MOTOR_ADDR = 0x05`。UART5 仍只用于 VOFA，USART3 仍只用于底盘地址 1–4。
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
- UART5：`ROT CCW <deg>`、`ROT CW <deg>`；路径：`PATH ADD ROT CCW <deg>`、`PATH ADD ROT CW <deg>`。`PATH SHOW` 将显示友好的 CW/CCW 方向；旋转段不参与速度 Blend。
