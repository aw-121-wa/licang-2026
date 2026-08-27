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
- UART5，PC12/PD2，115200：PC/VOFA ASCII 比赛调试接口（PC12=TX，PD2=RX）。
- USART6，PC6/PC7，115200：仓库转盘专用张大头闭环步进电机（PC6=TX，PC7=RX，地址 `0x05`）。
- 麦轮直径：75 mm。
- 麦轮坐标：车头方向为 `+X`，车体左侧为 `+Y`，逆时针旋转为 `+Omega`。
- 当前 X 型麦轮方向矩阵：前进 `++++`，左移 `-++-`，逆时针 `-+-+`。

## 模块结构

- `Core/`：CubeMX 系统、时钟、GPIO、串口和 FreeRTOS 初始化。
- `Motor/motor_control.*`：张大头驱动协议、地址和安装方向、`F6/FD` 命令、四轴缓存与同步触发。
- `Motor/mecanum_kinematics.*`：不依赖 HAL 的麦轮运动学解算和等比例限幅。
- `Motor/motion_control.*`：速度时间积分距离、软件加减速、20 ms实时航向 PD、任意角度平移、故障停车和动作序列。
- `IMU/jy61p.*`：JY60/JY61P 数据帧解析、连续航向角和串口中断接收。
- `App/uart_command.*`：UART5 ASCII 命令接收、解析、精简状态查询和 FreeRTOS 命令投递。
- `Motor/cangku_motor.*`：仓库转盘的单电机 Emm V5.0/x42 协议层；只操作 USART1 地址 `0x05`，不与底盘四轮共用状态。
- `App/turntable_control.*`：仓库转盘一格相对运动、启用、停止及集中等待策略。
- `App/warehouse_control.*`：机械臂组 2（夹取）完成后的仓库协同和六球计数状态机，由现有 `ChassisTask` 调用，不新建重复任务。
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
- 现场运动测试只通过 UART5 命令完成；不再保留 PATH 编辑器、独立上电测试和旧的运动包装接口。
- UART5 保留 F/B/L/R、LF/RF/LR/RR、ROT、BALL、GRAB、RZ、STOP、STATUS、HELP。
- 平移统一使用 `MotionControl_MovePolarSegmentMm()`；纯横移额外使用唯一的 `LATERAL_FORWARD_COMPENSATION` 前后偏差补偿，初值为 `0.0f`。
- `MotionControl_SetBodySpeed()` 和 `MotionControl_GetHeadingCorrection()` 是灰度校准、RZ 与普通平移共用的底盘速度/航向接口；航向 PD 参数只在 `Motor/motion_control.c` 保留一套。
- FreeRTOS 启动后由 `ChassisTask` 完成 `MotionControl_Init`、IMU 等待、四轮使能和锁头基准建立，再等待 UART5 命令；`main.c` 不再直接执行底盘动作。
- UART5 接收使用单字节 `HAL_UART_Receive_IT()`；UART5 中断只收字节和投递行缓冲，运动命令统一由 `ChassisTask` 执行。任务只维护 Ready、Busy 和 LastStatus。

## UART5 现场 STATUS

- `STATUS` 只输出 `STATE`、`IMU`、`YAW`、`HEAD_ERR`、`HEAD_CORR`、`DIST`、`TARGET`、`LAST`、`BALL_STATE`、`BALL_ROUND`、`WAREHOUSE_BALL` 和 `STOP`。
- MaixCAM、机械臂和转盘的实际错误处理仍保留，但不再为 STATUS 保存或输出仅用于调试的收发计数、动作计数和预计时间统计。

## 用户工作偏好

- 倾向直接实施并验证，不只给示例代码或口头建议。
- `main.c` 尽量简洁，业务逻辑独立成模块。
- 重视多份现有代码和公开优秀实现的对比，但不能忽略本车实际安装关系。
- 希望主动发现隐藏问题，并清楚区分“已验证”“需要实车标定”和“尚未实现”。
- 交付时说明修改文件、构建结果、风险和下一步实车检查点。

## Servo action-group sequence (2026-08-24)

- UART7 PE7=RX and PE8=TX, 9600-8-N-1, Hiwonder/Lobot action-group controller.
- Action group 0 is `出发姿态.rob`; action group 1 is `8.25-圆盘机回位.rob`; action group 2 is `8.25-圆盘机夹.rob`.
- The controller must be preloaded with the three `.rob` files in slots 0, 1 and 2; the MCU sends only the action-group invocation frame.
- At boot the MCU sends group 0 once but does not require a completion reply, because the installed controller does not provide a usable completion frame for it. Chassis commands become available after normal IMU/motor preparation; do not issue `GRAB` or `BALL` until the physical start pose has finished. `GRAB` runs group 2 (clamp), then one turntable slot, then group 1 (return).

## MaixCAM ball handshake (2026-08-25)

- UART4 connects to MaixCAM at 115200-8-N-1: PC10=TX and PC11=RX. Use 3.3 V TTL, cross-connect TX/RX and share GND.
- `App/maixcam_link.*` owns UART4 byte reception and accepts only an ASCII reply line `1` as a MaixCAM acknowledgement; outgoing request is one byte, `1` for red or `2` for blue. The deployed `licang_BLUE_RED_BALL.py` accepts a new command after the prior request state is cleared and checks from the next frame for a color/shape-qualified complete target inside the yellow ROI. AUTO calibration updates only the measured ball dimensions and center, leaving the ROI unchanged; MANUAL ROI defines the yellow search area directly with two touch points. AUTO-calibrated size limits apply in both ROI modes. No green target rectangle or multi-frame confirmation is used.
- UART5 command `BALL` first runs action group 1 (return/recognition posture). It then runs the remaining rounds of the current six-ball warehouse batch: request MaixCAM, wait up to 10 s for reply `1`, run action group 2 (clamp), turn the warehouse one slot, then run action group 1 (return). The sixth group-2 completion also turns one slot and is followed by group 1 return.
- `GRAB` remains a separate single cycle: group 2 (clamp) -> one turntable slot -> group 1 (return). While a BALL batch runs, all ordinary chassis, `GRAB` and new `BALL` commands remain busy.
- MaixCAM timeout or UART4 transmission failure starts no servo action and allows a later `BALL` retry. A group 1/2 communication failure retains the existing arm error lock. `STOP` cancels an acknowledgement wait immediately; during group 2/turntable it still completes group 1 (return) before ending the remaining batch.

## BALL gray alignment (2026-08-25)

- The four gray sensors are ordered from left to right as `MID2`, `IN2`, `IN1`, `MID1`: `MID2=PD8`, `IN2=PD0`, `IN1=PD1`, `MID1=PD3`.
- Inputs use GPIO pull-ups and active-low line detection. The logical `OnLine` order is therefore `0 1 1 0` for the only valid alignment state: both inner sensors on the line and both outer sensors off the line.
- `App/gray_align.*` runs before BALL action group 1. It locks the current continuous JY61P yaw at entry, moves only along the left/right axis at 25 RPM, and applies a small heading PD correction (`KP=2.0`, `KD=0.15`, limit 8 RPM) during the lateral move. It holds the exact target for 50 ms, stops all wheels, and resets the continuous JY61P yaw before returning success. The alignment timeout is 5 s.
- `MID1`/`MID2` are overshoot protection sensors, not completion sensors. If either is on, the chassis retreats while maintaining the locked yaw; otherwise it approaches. IN1/IN2 appearing in sequence never commands a chassis rotation. IMU loss during alignment stops the chassis and returns an alignment error.

## RZ pillar ball sequence (2026-08-26)

- UART5 command `RZ` enters the existing `ChassisCommandQueue` and is executed by `ChassisTask`; it first uses PD10 to approach the pillar, locks the yaw during chassis positioning, then stops and settles before any arm or vision action. After the stable IR trigger, RZ starts the camera group directly without an extra approach move.
- After the chassis is positioned, RZ starts `SERVO_ACTION_PILLAR_CAMERA_GROUP` (group 3) once without requiring a completion frame, waits the fixed `RZ_CAMERA_RAISE_WAIT_MS` mechanical interval, and then starts `RoundPillar_OrbitAndGrab()`. The integrated routine sends the first red request and keeps the single-direction pillar orbit active while polling the MaixCAM reply.
- A valid reply stops and settles the chassis before `SERVO_ACTION_PILLAR_GRAB_GROUP` (group 4); after Group4 completes, the next red request is sent and the orbit resumes from the current continuous yaw. The RZ orbit now uses one clockwise direction with the corrected circle-centre side (`forward=+62 RPM`, `omega=-49 RPM`) and continues from its reset heading to `-360 degrees`; it completes directly at that target after four grabs, with no reverse stage.
- Group 4 contains clamp, release and camera re-positioning, so group 3 is never repeated. RZ does not call `BALL`, group 1, group 2 or `WarehouseControl`; after each successful Group4 it advances the turntable one slot, requests the next red ball and continues the current 450-degree orbit.
- STOP during approach or MaixCAM waiting cancels before the next arm action. Once group 3 or group 4 starts, that group is allowed to finish; a pending STOP is handled before the next vision request. MaixCAM and servo failures map to their dedicated RZ result statuses and do not trigger later group 4 actions.

## Warehouse turntable coordination (2026-08-25)

- `ZDT_MOTOR_ADDR = 0x05U` is the single authoritative warehouse-motor address. Chassis addresses 1–4 on USART3 remain unchanged.
- One slot is a relative `FD` move of `1280` pulses. `TURNTABLE_SLOT_DIRECTION` is currently `ZDT_DIR_CW`, speed is 100 RPM and acceleration is 0; all are centralized in `App/turntable_control.h` for hardware calibration.
- `ServoAction_RunGroup(2)` waits for the real UART7 completion frame (`55 55 05 08 02 ...`). Only after that frame does `WarehouseControl_HandleActionGroup2Completed()` issue one turntable move. A command transmission is not considered completion.
- The installed turntable driver has no returned completion/status parser. The initial completion criterion is its calculated 240 ms running time plus 600 ms settling margin (840 ms total), bounded by a 1500 ms timeout. This must be verified on hardware before competition.
- A successful group-2/turntable pair increments `Warehouse_BallCount`. The counter is capped at six; `WAREHOUSE_TURN_AFTER_LAST_BALL` is `1U`, so the sixth ball also triggers a turn. A UART failure or timeout stops the turntable and enters the warehouse error state without incrementing the count.
- If `STOP` is already pending when group 2 completes, no new turntable command is sent; group 1 still runs to return the arm, then the remaining warehouse batch is canceled. During a turntable timing wait, `STOP` sends the driver stop command and likewise does not increment the count.

## IMU closed-loop in-place rotation (2026-08-25)

- `MotionControl_RotateDeg(angle_deg)` rotates about the chassis centre using the existing mecanum inverse kinematics: positive angle is counter-clockwise/left and negative angle is clockwise/right.
- Rotation is measured only by `Jy61P_GetContinuousYaw()`; it has no time- or encoder-pulse-based completion estimate. The heading baseline is reset at the start and after a settled successful rotation, so following translation holds the new vehicle heading.
- UART5 accepts `ROT CCW <deg>` / `ROT CW <deg>` (1..360 degrees)；旋转由 `ChassisTask` 直接执行，不再经过路径编辑器。
- Default parameters are 50 RPM cruise, 15 RPM approach, 8 RPM minimum effective speed, deceleration from 30 degrees, fine control from 10 degrees, 0.8-degree tolerance, five 20-ms settle periods, 250-ms ramp, and 8-s timeout.
