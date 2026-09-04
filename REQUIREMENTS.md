# 当前需求与验收标准

## 当前比赛动作

1. 等待 JY60 输出有效角度帧（软件兼容 API 仍为 `Jy61P_*`）。
2. 使能四个电机并建立当前车头航向基准。
3. 等待 UART5 现场命令；普通平移、ROT、BALL、GRAB、RZ、STAIR 和固定比赛路线 PATH 均由 `ChassisTask` 顺序执行。
4. 平移过程中保持同一航向基准，实时执行统一的航向 PD 修正。
5. 完成动作后保持停止并返回可接收命令状态。

## 当前 IMU 硬件

- 当前硬件为 JY60，使用 USART2：PD5=TX、PD6=RX，9600、8N1、无硬件流控。
- 软件继续保留 `IMU/jy61p.*` 和 `Jy61P_*` 作为兼容接口；解析器只消费标准 11 字节 `0x55 0x53` 角度帧，并保留校验和、连续航向角及在线超时语义。

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
- `PATH` 是无参数的固定比赛命令；仅在底盘空闲、仓库转盘已准备好且舵机状态可用时接受，并由 `App/path_sequence.c` 的编译期静态指令表逐项执行。
- PATH 通过 `ChassisCommandQueue` 只投递一条 `CHASSIS_CMD_PATH`；PATH 内部不向现有 FreeRTOS 队列追加七条命令，运行期间 `ChassisCommand_Busy` 保持为 1。

## FreeRTOS 命令执行

- `main.c` 只负责硬件初始化（包括 UART5）、`osKernelInitialize()`、`MX_FREERTOS_Init()` 和 `osKernelStart()`。
- `ChassisTask` 只初始化一次 `MotionControl`，完成 IMU 等待、四轮使能和锁头基准建立后等待命令队列；所有运动函数只能由该任务在任务上下文执行，`CHASSIS_CMD_PATH` 由该任务同步调用 `PathSequence_Run()`。
- `MotionControl_PrepareForMove()` 负责启动时 IMU 有效性检查、四轮使能和统一航向基准建立；不再保留独立上电运动测试。
- 正式平移只保留 `MotionControl_MovePolarSegmentMm()`；起步、停车、距离积分和轮速限幅均由 `motion_control` 负责。
- 角度定义为：0°前进，+45°左前，+90°左移，+135°左后，180°后退，-45°右前，-90°右移，-135°右后；距离表示沿归一化轨迹方向的目标长度。

## VOFA 命令

- 运动：`F <mm>`、`B <mm>`、`L <mm>`、`R <mm>`、`LF <mm> <deg>`、`RF <mm> <deg>`、`LR <mm> <deg>`、`RR <mm> <deg>`。
- 旋转：`ROT CCW <deg>`、`ROT CW <deg>`。
- 动作：`BALL`、`GRAB`、`RZ`、`STAIR`、`PATH`。
- 控制和查询：`STOP`、`STATUS`、`HELP`。
- 已删除动态 PATH 编辑器、`PATH ADD`/`PATH CLEAR`/`PATH SHOW`/`PATH LOAD DEFAULT`/`PATH RUN` 等子命令；当前 PATH 仅运行固定编译期比赛表。

## 固定比赛路线 PATH

- 顺序必须为：`Move(1800, +20°)` → `Move(2300, 0°)` → `Rotate(+178°)` → `BallSequence_Run()` → `Rotate(+178°)` → `Move(1810, 180°)` → `RoundPillar_Run()` → `ServoAction_RunGroup(SERVO_ACTION_START_GROUP, 1U, SERVO_ACTION_START_TIMEOUT_MS)` → `Move(330, 0°)` → `StairSequence_Run(Part3 → Part2 → Part1)` → `ServoAction_RunGroup(SERVO_ACTION_START_GROUP, 1U, SERVO_ACTION_START_TIMEOUT_MS)` → `Move(2000, +90°, MOTION_CRUISE_RPM)` → 停车并进入 `DONE`。最终 PATH 共 12 个 step，最后一步为向左横移 2000 mm；按极坐标定义必须使用 +90°，不再执行左后斜向移动。
- 每个同步 API 成功返回后才进入下一步；RZ 成功后必须等待第一次动作组 0 返回 `SERVO_ACTION_OK`，再执行前进 330 mm；STAIR 必须按第三、第二、第一部分完整返回 `STAIR_SEQUENCE_OK` 后，才允许等待第二次动作组 0，第二次动作组 0 成功后才允许执行最后的 2000 mm 移动。任一动作组失败映射为独立 `PATH_SEQUENCE_ERROR_SERVO` 并立即结束 PATH；任一步 STOP、运动错误、BALL/RZ/STAIR 错误都立即结束整条 PATH，不执行后续步骤；最后的 2000 mm 移动期间仍可由 STOP 立即取消并停车。
- PATH 状态通过 `PATH_STATE`、`PATH_STEP`、`PATH_LAST` 和 `PATH_BALL_LAST` 暴露在 `STATUS` 中；同时输出 `WAREHOUSE_STATE`、`WAREHOUSE_BALL`、`STOP` 和 `STOPPED` 便于区分 BALL 完成、仓库 FINISHED 与真实 STOP。成功结束状态为 `DONE`，STOP 状态为 `CANCELED`，其他失败状态为 `ERROR`。

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

## STAIR 阶梯测试验收（2026-08-29）

- `STAIR` 必须首先调用 `GrayAlign_RunUnlimited()`；只有 `GRAY_ALIGN_OK` 才允许以极坐标 180°后退 20 mm，且该移动完整成功后才允许发送第一段动作组 G11。灰度成功后的实际顺序为后退 20 mm、第一段 G11、第二段 G8、第三段 G5。20 mm 移动期间 STOP、IMU 和电机错误必须按现有 STAIR 规则退出。无限找线仍按 20 ms 周期检查 STOP、IMU 在线和电机发送错误；灰度逻辑、BALL、RZ、GRAB、Group0–4 和普通移动行为不得被修改。
- 通用 `GrayAlign_Run()` 仍保留 5000 ms 超时，BALL 继续使用该接口；只有 STAIR 使用无正常时间上限的 `GrayAlign_RunUnlimited()`。
- 搜索姿态为 G11/G8/G5，均使用 `ServoAction_StartGroupNoWait()` 后等待 `STAIR_CAMERA_POSE_WAIT_MS=1000 ms`，等待期间每 10 ms 检查 STOP；抓取 G12/G9/G6、过渡 G10/G7 及第三段末尾 G0 使用 `ServoAction_RunGroup()`，超时为 20000 ms。
- 第一部分：G11 → 静止识别；找到则 G12 → 转盘 1280，随后后退 90 mm，未找到则直接后退 90 mm；90 mm 完成后再次静止识别，找到则 G12 → 转盘 1280 → G10，未找到则直接 G10；最后后退 117 mm。
- 第二部分：第一部分末尾后退 117 mm 的终点就是第 1 点；在该点执行 G8 后开始四个点位的逐点静止识别。第 1 点后退 90 mm 到第 2 点，第 2 点后退 90 mm 到第 3 点，第 3 点后退 90 mm 到第 4 点，总共只执行三次点间 90 mm 移动。命中点执行 G9 → 转盘 1280；第四点完成后，命中则在 G9 完成后执行 G7，未命中直接执行 G7，最后后退 117 mm。
- 第三部分：G5 → 静止识别；找到则 G6 → 转盘 1280；无论是否找到都后退 90 mm，完成后再次静止识别；找到则 G6 → 转盘 1280 → G0，未找到直接 G0，第三部分结束。
- 所有静止红球识别超时固定为 1000 ms，每次检测必须重新 `MaixCamLink_SendRequest(MAIXCAM_COLOR_RED)`；超时仅表示 `NOT_FOUND`，不映射为错误。
- 第二段的 90 mm 移动先发红球请求，再调用带可选 early-stop callback 的运动 API；视觉提前命中时立即停车，保留当前 `MotionControl_TraveledMm`，不补足剩余 90 mm。第一段和第三段的 90 mm 均以后退极坐标 180°完整走完后停车，再重新发请求静止识别；117 mm 过渡以后退 180°执行且不使用视觉提前截断。
- STAIR 直接调用 `Turntable_MoveOneSlotAndWait()`，每个成功 G6/G9/G12 恰好转一次 1280 脉冲；不调用 `WarehouseControl_HandleActionGroup2Completed()`，不改变 `Warehouse_BallCount` 或 `Warehouse_State`。
- `STATUS` 至少显示 `STAIR_STATE` 和 `STAIR_LAST`；STAIR 的 STOP、灰度、IMU、运动、电机、舵机、转盘和 MaixCAM UART 错误必须能从状态中区分。STAIR 联调失败后，在底层仍可用时保持 `ChassisTask_Ready=1` 以便重试。

## Servo action-group sequence (2026-08-24)

- On boot, UART7 (PE7/PE8, 9600-8-N-1) sends action group 0 (`出发姿态.rob`) once, but does not wait for a completion frame.
- UART5 chassis commands become available after the normal IMU/motor preparation. The operator must wait for the physical start pose to finish before sending `GRAB` or `BALL`, because group 0 completion is no longer observable by the MCU.
- Chassis motion commands are not limited by count. A STOP or motion error does not trigger the arm sequence.
- When the chassis is idle, UART5 command `GRAB` runs action group 2 (`8.25-圆盘机夹.rob`), turns the warehouse one slot only after group 2 really completes, then runs action group 1 (`8.25-圆盘机回位.rob`).
- The three `.rob` files must be downloaded to controller slots 0, 1 and 2; the MCU sends only invocation frames.

## MaixCAM six-ball sequence (2026-08-25)

- UART4 uses PC10=TX, PC11=RX, 115200-8-N-1, no flow control and IRQ reception. MaixCAM must use 3.3 V TTL, crossed TX/RX and common ground.
- `BALL` first runs group 1 (return/recognition posture). Each of its six possible rounds then sends the configured one-byte color request (`1` for red; the current no-argument BALL path selects red), waits no more than 10 s for MaixCAM's valid ASCII line `1`, runs group 2 (clamp), turns the warehouse one slot, then runs group 1 (return). MaixCAM recognizes only inside the yellow ROI and replies on the first frame containing a complete calibrated target ball; the ball bounding box must be fully inside the ROI, with no green trigger zone, disappearance trigger, extra inner margin, or multi-frame confirmation. AUTO calibration updates only the standard ball dimensions and center and leaves the ROI unchanged; MANUAL ROI uses two touch points to define the yellow search area directly. The AUTO-calibrated size limits apply regardless of whether the ROI is AUTO or MANUAL. If manual `GRAB` cycles already consumed slots, `BALL` runs only the remaining count.
- `STATUS` reports the compact fields `STATE`, `IMU`, `YAW`, `HEAD_ERR`, `HEAD_CORR`, `DIST`, `TARGET`, `LAST`, `BALL_STATE`, `BALL_ROUND`, `PATH_STATE`, `PATH_STEP`, `PATH_LAST`, `PATH_BALL_LAST`, `WAREHOUSE_STATE`, `WAREHOUSE_BALL`, `STOP`, `STOPPED`, `STAIR_STATE`, `STAIR_LAST`, `TURNTABLE_STATE` and `TURNTABLE_LAST`; MaixCAM/servo/turntable error handling remains active but its debug counters are not exposed.
- A MaixCAM timeout or UART4 send failure does not move the arm and allows retrying `BALL`; action-group failures retain arm error lock. `STOP` ends a waiting round immediately; a STOP during group 2/turntable still waits for group 1 return to complete, then cancels the rest of the batch.

## BALL 灰度校准（2026-08-25）

- `BALL` 必须先执行灰度校准，再允许动作组 1、MaixCAM 识别和动作组 2。四路从左到右为 `MID2 IN2 IN1 MID1`，实际 STM32 引脚分别为 `PD8 PD0 PD1 PD3`。
- 灰度输入使用上拉，低电平表示压线。唯一成功状态是逻辑 `0 1 1 0`：`IN1/IN2` 同时在线，`MID1/MID2` 同时离线，并且连续稳定 50 ms；`MID1/MID2` 同时在线不能判定成功。
- 进入校准时锁定当前 JY61P 连续航向；校准期间只允许横向移动，不允许灰度状态触发原地旋转。横移过程中使用 `KP=2.0`、`KD=0.15`、最大 8 RPM 的航向 PD 纠偏保持锁定角度。
- `MID1/MID2` 均离线且未达到目标时以 25 RPM 靠近；任一外侧传感器在线时以 25 RPM 反向退出。IN1/IN2 先后在线只影响是否达到目标，不改变航向。5 s 内未达到稳定目标返回灰度校准错误；IMU 失联立即停车并返回 IMU 错误，不降级为纯灰度控制。
- 校准成功后停车并清零 JY61P 连续航向，再进入已有的动作组 1 → MaixCAM → 动作组 2 → 转盘流程。

## RZ 靠桩抓球动作（2026-08-26）

- UART5 无参数命令 `RZ` 必须经 `ChassisCommandQueue` 投递并由 `ChassisTask` 执行。RZ 先完成 PD10 靠桩、锁角、红外稳定确认和停车稳定；红外稳定触发后不再额外靠近，直接进入相机动作组和绕桩流程。
- RZ 使用 PD10 单红外输入；第一版配置为 GPIO 输入、无上下拉、低电平有效，实际有效电平只允许修改 `RZ_IR_DETECTED_LEVEL`。靠桩阶段保留 30 ms 稳定判断、5 s 超时、STOP 响应和 IMU 在线检查。
- 底盘定位完成后使用 `ServoAction_StartGroupNoWait()` 启动 `SERVO_ACTION_PILLAR_CAMERA_GROUP`（动作组 3）一次；发送失败返回 `ROUND_PILLAR_ERROR_SERVO`，不依赖完成回包，并固定等待 `RZ_CAMERA_RAISE_WAIT_MS=1000 ms` 的机械动作时间后进入 `RoundPillar_OrbitAndGrab()`。
- `RoundPillar_OrbitAndGrab()` 先发送第一个红球请求，再以反向的逆时针方向（`forward=-62 RPM`、`omega=+49 RPM`）在 20 ms 控制循环中非阻塞轮询 `MaixCamLink_TakeReply()`；从当前航向基准连续绕行至配置的 `RZ_ORBIT_TARGET_DEG=+352°` 后直接结束。识别成功立即停车稳定，执行 Group4，完成后保持当前 ContinuousYaw 继续绕桩并发送下一次请求；单向绕桩使用现有 15000 ms 保护时间。
- 视觉阶段固定使用 `MaixCamLink_SendRequest(MAIXCAM_COLOR_RED)`。每轮必须重新发送请求并等待 `MaixCamLink_TakeReply()` 的当前请求有效回复；不能复用旧回复或一次请求等待四次回复。
- `RZ_GRAB_COUNT` 固定为 4，仅表示本次 RZ 最多处理 4 个识别到的球，不是 RZ 成功门槛。每轮顺序为“红球请求 → 有效 `1` 回复 → `SERVO_ACTION_PILLAR_GRAB_GROUP`（动作组 4）”；Group4 完整包含夹球、放球和重新架摄像头，Group4 完成后才允许下一轮请求。只要达到配置的 `RZ_ORBIT_TARGET_DEG` 且无真实错误，即使抓到 0–3 个球也返回 `ROUND_PILLAR_OK`。
- RZ 不调用 `BALL`、`WarehouseControl`、转盘、动作组 1 或动作组 2，不增加 `Warehouse_BallCount`。Group4 一旦开始必须完整执行；Group3 完成后、MaixCAM 等待期间和 Group4 完成后均检查 STOP，取消时不进入下一轮。
- RZ 结果映射为：靠桩超时 `MOTION_ERROR_RZ_TIMEOUT`、舵机错误 `MOTION_ERROR_MOTOR_UART`、MaixCAM UART 错误 `MOTION_ERROR_MAIX_UART`、MaixCAM 超时 `MOTION_ERROR_MAIX_TIMEOUT`；不再保留旧 Orbit 状态。

## 仓库转盘协同（2026-08-25）

- 仓库电机固定使用 USART6：PC6=TX、PC7=RX、115200、8N1、无硬件流控；地址必须为 `ZDT_MOTOR_ADDR = 0x05`。UART5 仍只用于 VOFA，USART3 仍只用于底盘地址 1–4。
- `Turntable_MoveOneSlot()` 必须调用 `ZDT_MoveRelative(TURNTABLE_SLOT_DIRECTION, TURNTABLE_MOVE_SPEED_RPM, TURNTABLE_MOVE_ACCELERATION, 1280U)`，使用相对位置，不得自行拼接协议帧。
- 只能在机械臂动作组 2（圆盘机夹）的真实 UART7 完成反馈之后转盘；动作组命令成功发送不是完成条件。动作组 1（圆盘机回位）或其他组完成不得触发转盘。
- 现阶段未解析转盘驱动器到位反馈，`Turntable_WaitComplete()` 以 1280 脉冲、100 RPM、3200 脉冲/圈计算运行时间并加 600 ms 裕量，1500 ms 后超时。超时或 UART 错误必须执行停止、进入 `WAREHOUSE_STATE_ERROR`，且不得增加球计数。
- `Warehouse_BallCount` 表示“组 2（夹取）已真实完成且对应一格转盘已成功完成”的数量。最大为 6；`WAREHOUSE_TURN_AFTER_LAST_BALL=1U`，因此第 6 球后默认也转一格，总理论相对脉冲为 `6 × 1280 = 7680`。
- 若动作组 2 完成时已经收到 `STOP`，不得再下发新的转盘命令；仍须执行动作组 1 回位后取消本批。转盘等待期间收到 `STOP` 时必须向地址 5 发送停止命令，且不得增加球计数。

## CANGKU 仓库搬运流程（2026-09-04）

- UART5 无参数命令 `CANGKU` 进入 `ChassisCommandQueue`，由 `ChassisTask` 执行；串口解析任务不直接执行阻塞动作。
- CANGKU 先原地逆时针旋转 180 度，再复用 `GrayAlign_Run()` 对齐灰度目标 `MID2 IN2 IN1 MID1 = 0 1 1 0`，稳定后继续后续搬运流程。
- 搬运顺序固定为：后退 200 mm 后执行 G13、G14、反向转盘一格；后退 200 mm 后执行 G14、G13、反向转盘一格；再次后退 200 mm 后执行 G14、G13、反向转盘一格；执行 G15、G13、反向转盘一格；前进 200 mm 后执行 G15、G13、反向转盘一格；再次前进 200 mm 后执行 G15、G13、反向转盘一格。
- 反向转盘仍使用现有一格 1280 脉冲、速度和加速度参数，仅取反 `TURNTABLE_SLOT_DIRECTION`；CANGKU 不改变 `Warehouse_BallCount`。
- STOP、IMU、底盘、动作组和转盘错误均在 CANGKU 流程中停止并返回对应状态；动作组 13、14、15 必须预先写入舵机控制器。

## 原地旋转任意角度（2026-08-25）

- 提供 `MotionControl_RotateDeg(float angle_deg)`：正角为逆时针左转，负角为顺时针右转，允许绝对值 `1..360` 度（0 度无效）。
- 必须以 JY61P `ContinuousYaw` 闭环判断旋转角度；旋转期间每 20 ms 检查 IMU 在线和 STOP 请求，IMU 失联、串口发送失败或超过 8000 ms 均安全停机并返回错误。
- 四轮速度必须继续通过 `MecanumKinematics_Solve(0, 0, omega)` 和 `MotorControl_SetWheelSpeeds()` 发送，保持原有四轮同步触发；不改动电机协议、安装方向、麦轮公式、JY61P解析或 FreeRTOS 配置。
- 默认参数：巡航 50 RPM、接近 15 RPM、最小有效 8 RPM；剩余 30 度开始减速、10 度进入低速区；进入 ±0.8 度后发送 0 RPM，连续稳定 5 个控制周期（约100 ms）才算完成。
- 成功完成后调用 `Jy61P_ResetContinuousYaw()`，使后续前进/横移/斜线以旋转后的车头作为新的锁头参考。
- UART5：`ROT CCW <deg>`、`ROT CW <deg>`；旋转直接由 `ChassisTask` 执行，不再经过路径编辑器。
