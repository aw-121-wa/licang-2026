# chassis_motor 项目说明

## 项目目标

基于 STM32F750V8Tx、JY60/JY61P 陀螺仪和 x42_v1.3 张大头闭环步进驱动板，实现四麦克纳姆轮底盘的开环距离控制与实时航向保持。

## 硬件与接口

- 1号电机：左前轮。
- 2号电机：右前轮。
- 3号电机：左后轮。
- 4号电机：右后轮。
- USART3，115200：四台电机驱动器通信。
- USART2，PD5/PD6，9600：JY60/JY61P 通信。
- UART5，PC12/PD2，115200：PC/VOFA 命令和路径调试接口（PC12=TX，PD2=RX）。
- 麦轮直径：75 mm。
- 麦轮坐标：车头方向为 `+X`，车体左侧为 `+Y`，逆时针旋转为 `+Omega`。
- 当前 X 型麦轮方向矩阵：前进 `++++`，左移 `-++-`，逆时针 `-+-+`。

## 模块结构

- `Core/`：CubeMX 系统、时钟、GPIO、串口和 FreeRTOS 初始化。
- `App/competition_path.*`：比赛路径编排层，只按顺序调用 `MotionControl` 公共接口，不包含驱动协议、麦轮公式或 IMU 解析。
- `Motor/motor_control.*`：张大头驱动协议、地址和安装方向、`F6/FD` 命令、四轴缓存与同步触发。
- `Motor/mecanum_kinematics.*`：不依赖 HAL 的麦轮运动学解算和等比例限幅。
- `Motor/motion_control.*`：速度时间积分距离、软件加减速、20 ms实时航向 PD、任意角度平移、故障停车和动作序列。
- `IMU/jy61p.*`：JY60/JY61P 数据帧解析、连续航向角和串口中断接收。
- `App/uart_command.*`：UART5 ASCII 命令接收、解析、路径编辑、状态查询和 FreeRTOS 命令投递。
- `.vscode/`：IntelliSense 与 Keil 构建任务。

## 已确定的设计决策

- 行走使用 `F6` 速度模式，每20 ms更新四轮速度；不用多条短 `FD` 位置命令，避免反复减速到位造成卡顿。
- 四台驱动器的 `S_Vel_IS` 已由用户开启；程序直接按0.1 RPM单位编码，100 RPM编码为1000，不在上电时重复修改驱动器配置。
- 指定距离采用已下发平移RPM和实际时间的软件积分，按75 mm轮径换算，不读取电机返回值。
- 四个速度命令先缓存，全部发送成功后才发送广播同步触发。
- 锁头修正每20 ms叠加到麦轮逆解旋转分量，并限制绝对值和相对平移速度比例。
- 软件线性加减速用于降低速度模式启停冲击。
- IMU 启动无数据、运动中离线或电机串口发送失败时停止运动。
- 通用麦轮限幅保持四轮比例。
- 平移方向统一使用极坐标：0°前进、+90°左移、180°后退、-90°右移，角度范围为 -180°～+180°。
- 指定的极坐标距离表示实际平移轨迹长度；每个控制周期把轮速限幅后的有效平移 RPM 纳入距离积分。
- `MotionControl_RunDefaultSequence()` 的独立测试仍由 `DIAGONAL_TEST_ENABLE` 控制；正式 FreeRTOS 路径由 `App/competition_path.c` 管理。
- 当前正式路径为左前 `+30°` 斜线运行 1800 mm，并软减速到 0。
- FreeRTOS 启动后由 `ChassisTask` 完成 `MotionControl_Init`、IMU 等待、四轮使能和锁头基准建立，再等待 UART5 命令；`main.c` 不再直接执行底盘动作。
- UART5 接收使用单字节 `HAL_UART_Receive_IT()`；UART5 中断只收字节和投递行缓冲，运动命令统一由 `ChassisTask` 执行。
- 当前正式路径保留在编译代码中，用户路径使用独立的静态 RAM 表，最多 20 段；`PATH LOAD DEFAULT` 将当前编译默认路径载入 RAM。

## 用户工作偏好

- 倾向直接实施并验证，不只给示例代码或口头建议。
- `main.c` 尽量简洁，业务逻辑独立成模块。
- 重视多份现有代码和公开优秀实现的对比，但不能忽略本车实际安装关系。
- 希望主动发现隐藏问题，并清楚区分“已验证”“需要实车标定”和“尚未实现”。
- 交付时说明修改文件、构建结果、风险和下一步实车检查点。

## Servo action-group sequence (2026-08-24)

- UART7 PE7=RX and PE8=TX, 9600-8-N-1, Hiwonder/Lobot action-group controller.
- Action group 0 is `出发姿态.rob`; action group 1 is `8.25-圆盘机夹.rob`; action group 2 is `8.25-圆盘机回位.rob`.
- The controller must be preloaded with the three `.rob` files in slots 0, 1 and 2; the MCU sends only the action-group invocation frame.
- At boot the MCU sends group 0 once but does not require a completion reply, because the installed controller does not provide a usable completion frame for it. Chassis commands become available after normal IMU/motor preparation; do not issue `GRAB` or `BALL` until the physical start pose has finished. Chassis motions are not count-limited; when the chassis is idle, UART5 command `GRAB` explicitly runs group 1 and, after its completion, group 2. After group 2 completes successfully, chassis motion commands remain available; repeated `GRAB` commands are rejected.

## MaixCAM ball handshake (2026-08-25)

- UART4 connects to MaixCAM at 115200-8-N-1: PC10=TX and PC11=RX. Use 3.3 V TTL, cross-connect TX/RX and share GND.
- `App/maixcam_link.*` owns UART4 byte reception and accepts only an ASCII reply line `1` as a MaixCAM acknowledgement; outgoing request frame is always `1\r\n`.
- UART5 command `BALL` runs five complete rounds: request MaixCAM, wait up to 10 s for reply `1`, run action group 1 (grab), then action group 2 (return). The fifth return ends the batch without a sixth request.
- `GRAB` remains a separate single action-group 1 then 2 operation. While a BALL batch runs, all ordinary chassis, `GRAB` and new `BALL` commands remain busy.
- MaixCAM timeout or UART4 transmission failure starts no servo action and allows a later `BALL` retry. A group 1/2 communication failure retains the existing arm error lock. `STOP` cancels an acknowledgement wait immediately; during a grab it finishes group 1 and group 2 return before ending the remaining batch.

## IMU closed-loop in-place rotation (2026-08-25)

- `MotionControl_RotateDeg(angle_deg)` rotates about the chassis centre using the existing mecanum inverse kinematics: positive angle is counter-clockwise/left and negative angle is clockwise/right.
- Rotation is measured only by `Jy61P_GetContinuousYaw()`; it has no time- or encoder-pulse-based completion estimate. The heading baseline is reset at the start and after a settled successful rotation, so following translation holds the new vehicle heading.
- UART5 accepts `ROT CCW <deg>` / `ROT CW <deg>` (1..360 degrees). User paths accept `PATH ADD ROT CCW <deg>` / `PATH ADD ROT CW <deg>`; a rotation is a hard path boundary rather than a blended translation segment.
- Default parameters are 50 RPM cruise, 15 RPM approach, 8 RPM minimum effective speed, deceleration from 30 degrees, fine control from 10 degrees, 0.8-degree tolerance, five 20-ms settle periods, 250-ms ramp, and 8-s timeout.
