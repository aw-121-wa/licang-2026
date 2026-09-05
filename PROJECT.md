# chassis_motor 项目说明

## User 层工程结构（2026-09-04）

- `Core/`、`Drivers/`、`Middlewares/` 保持 CubeMX/HAL/CMSIS/FreeRTOS 系统代码不变。
- `User/BSP/` 保存底盘电机和底层板级电机接口；`User/Device/` 保存舵机、RFID、IMU、MaixCAM 和转盘设备接口。
- `User/Algorithm/` 保存麦轮解算、运动控制和灰度对线；`User/Robot/` 保存 BALL、RZ、STAIR、PATH、CANGKU 等比赛流程。
- `User/Task/` 保存串口命令任务、底盘任务实现和任务接口；`User/Config/` 保存当前硬件映射及既有参数集中定义。
- 头文件仍使用原有 basename include 方式，由 CMake、Keil 和 VS Code include path 解析；通信协议、控制参数和动作顺序保持不变。

## 项目目标

基于 STM32F750V8Tx、JY60 陀螺仪和张大头闭环步进驱动板，实现四麦克纳姆轮底盘的开环距离控制与实时航向保持。2026-09-05 用户确认底盘固件已更换为 X42S，支持同步收发指令。

## 硬件与接口

- 1号电机：右前轮。
- 2号电机：左前轮。
- 3号电机：左后轮。
- 4号电机：右后轮。
- USART3，115200：四台电机驱动器通信。
- USART2，PD5/PD6：JY60 通信（软件层保留 `Jy61P_*` 兼容 API）。当前用户将波特率改为 115200，旧资料/CubeMX 配置为 9600；实际 IMU 配置尚待确认，本次保留用户设置。
- UART5，PC12/PD2，115200：PC/VOFA ASCII 比赛调试接口（PC12=TX，PD2=RX）。
- USART6，PC6/PC7，115200：仓库转盘专用张大头闭环步进电机（PC6=TX，PC7=RX，地址 `0x05`）。
- 麦轮直径：75 mm。
- 麦轮坐标：车头方向为 `+X`，车体左侧为 `+Y`，逆时针旋转为 `+Omega`。
- 通用麦轮解算矩阵按左前、右前、左后、右后排列：前进 `++++`，左移 `-++-`，名义旋转 `-+-+`。用户实测 ROT CCW 10 实际向右，且右转 YAW 为负，因此车体接口使用 `MOTION_OMEGA_TO_WHEEL_SIGN=-1` 校正安装方向；车体逆时针对应逻辑轮速 `+-+-`，按地址 1、2、3、4 为 `-++-`。平移分量不变。
- 旋转方向校正在共用 SetBodySpeed 接口实施，量化轮速反解旋转分量使用同一符号；ROT、行驶锁头、灰度对线和 RZ 均遵循左转为正。IMU 符号及 PD 增益不反转。实车仍需复验左右转与混合运动。
- BSP 将逻辑左前映射到地址 2、右前映射到地址 1；前进编码保持左侧 CCW、右侧 CW。

## 模块结构

- `Core/`：CubeMX 系统、时钟、GPIO、串口和 FreeRTOS 初始化。
- `User/BSP/motor_control.*`：张大头驱动协议、地址和安装方向、`F6/FD` 命令、四轴缓存与同步触发。
- `User/Algorithm/mecanum_kinematics.*`：不依赖 HAL 的麦轮运动学解算和等比例限幅。
- `User/Algorithm/motion_control.*`：速度时间积分距离、软件加减速、20 ms实时航向 PD、任意角度平移、故障停车和动作序列。
- `User/Device/imu/jy61p.*`：JY60/JY61P 标准角度帧解析、连续航向角和串口中断接收；文件名及 `Jy61P_*` 接口保留用于兼容历史工程。
- `User/Task/uart_command.*`：UART5 ASCII 命令接收、解析、精简状态查询和 FreeRTOS 命令投递。
- `User/BSP/cangku_motor.*`：仓库转盘的单电机 Emm V5.0/x42 协议层；只操作 USART1 地址 `0x05`，不与底盘四轮共用状态。
- `User/Device/turntable/turntable_control.*`：仓库转盘一格相对运动、启用、停止及集中等待策略。
- `User/Robot/warehouse_control.*`：机械臂组 2（夹取）完成后的仓库协同和六球计数状态机，由现有 `ChassisTask` 调用，不新建重复任务。
- `User/Robot/stair_sequence.*`：独立 UART5 `STAIR` 阶梯测试流程；复用灰度校准、MaixCAM、舵机组 5–12 和转盘接口，不进入仓库球计数状态机。
- `User/Robot/path_sequence.*`：固定比赛路线的一站式动作编排；由一个 `PATH` 命令触发，内部按编译期静态表顺序调用既有运动、BALL 和 RZ API。
- PATH 现有固定步骤完成后追加 `CANGKU` 仓库搬运流程；该步骤直接调用 `CangkuSequence_Run()`，不重复展开仓库动作。
- PATH 在 RZ 成功后阻塞等待第一次动作组 0 回位完成，再前进 330 mm、同步执行完整 STAIR（第三、第二、第一部分）；STAIR 成功完成后再次阻塞等待动作组 0 回位完成，再以极坐标 +90° 向左横移 1600 mm，随后执行 `CANGKU` 仓库搬运流程并进入 DONE。任一动作组或 CANGKU 步骤失败时 PATH 立即结束并报告对应错误。
- `.vscode/`：IntelliSense 与 Keil 构建任务。

## 已确定的设计决策

- 用户确认旋转方向修正后不再越纠越偏。按最新提速要求，普通平移巡航由 150 提至 450 RPM、斜行由 85 提至 255 RPM；单轮限速 460 RPM，为直行航向修正留余量。影响 UART5 平移及引用巡航参数的 PATH/CANGKU；ROT、灰度、STAIR 搜索和 RZ 专用速度沿用原值。S_Vel_IS 保持 Enable，加减速度和广播硬停沿用现值，短距离不保证达到巡航速度。

- 行走使用 `F6` 速度模式，每20 ms更新四轮速度；不用多条短 `FD` 位置命令，避免反复减速到位造成卡顿。
- 四台驱动器的 `S_Vel_IS` 已由用户开启；程序直接按0.1 RPM单位编码，100 RPM编码为1000，不在上电时重复修改驱动器配置。
- 指定距离采用已下发平移RPM和实际时间的软件积分，按75 mm轮径换算，不读取电机返回值。
- 四个速度命令先缓存，全部发送成功后才发送广播同步触发。
- 锁头修正每20 ms叠加到麦轮逆解旋转分量，并限制绝对值和相对平移速度比例。
- 软件线性加减速用于降低速度模式启停冲击。
- 距离控制保留剩余距离减速包络：按用户最新要求，加速度由 300 提至 600 RPM/s；常规减速度 120 RPM/s，20 RPM 以下 40 RPM/s。进入 0.5 mm 积分容差后立即广播硬停。
- 2026-09-05 根据实车回滚反馈，删除定距结束后的 300 ms 航向修正和残余速度软停。所有零车体速度请求使用 USART3 单帧 `00 FE 98 00 6B` 广播立即停止，不逐轮停车、不失能、不追加 FF 同步帧。行驶航向修正随平移速度限幅，保留 30 RPM/s 变化率，但降速时立即收紧限幅，避免低速残留过大修正。
- 停车发送失败最多重试一次，仍向调用者报告首次通信错误；成功发送时清零已应用速度和轮速诊断量。没有驱动器接收/执行确认，不能保证断线或驱动故障时四轮实际停住。普通 F6 更新仍四轮缓存后广播同步；非零末速度继续衔接。
- 距离积分按四轮同步发送完成时刻切换速度，并从实际编码的 0.1 RPM 轮速反解平移分量；发送期间仍积分上一条速度。该时刻不等于驱动器确认，仍未读取编码器反馈。
- `robot_config.h` 中 `FORWARD_DISTANCE_GAIN` 和 `LEFT_DISTANCE_GAIN` 分别用于纵向、横向距离标定；用户已将两轴恢复为 1.0，本轮保留，尚未提供尺量记录。操作见 [底盘距离标定](docs/CHASSIS_CALIBRATION.md)。
- IMU 启动无数据、运动中离线或电机串口发送失败时停止运动。
- 通用麦轮限幅保持四轮比例。
- 平移方向统一使用极坐标：0°前进、+90°左移、180°后退、-90°右移，角度范围为 -180°～+180°。
- 指定的极坐标距离表示实际平移轨迹长度；每个控制周期把轮速限幅后的有效平移 RPM 纳入距离积分。
- 现场运动测试只通过 UART5 命令完成；不再保留 PATH 编辑器、独立上电测试和旧的运动包装接口。
- UART5 保留 F/B/L/R、LF/RF/LR/RR、ROT、BALL、GRAB、RZ、STAIR、PATH、STOP、STATUS、HELP。
- `PATH` 不是动态路径编辑器；它只运行 `User/Robot/path_sequence.c` 中的固定比赛指令表，并以单条 `CHASSIS_CMD_PATH` 占用底盘命令队列。
- 平移统一使用 `MotionControl_MovePolarSegmentMm()`；纯横移额外使用唯一的 `LATERAL_FORWARD_COMPENSATION` 前后偏差补偿，初值为 `0.0f`。
- `MotionControl_SetBodySpeed()` 和 `MotionControl_GetHeadingCorrection()` 是灰度校准、RZ 与普通平移共用的底盘速度/航向接口；航向 PD 参数只在 `User/Algorithm/motion_control.c` 保留一套。
- FreeRTOS 启动后由 `ChassisTask` 完成 `MotionControl_Init`、IMU 等待、四轮使能和锁头基准建立，再等待 UART5 命令；`main.c` 不再直接执行底盘动作。
- UART5 接收使用单字节 `HAL_UART_Receive_IT()`；UART5 中断只收字节和投递行缓冲，运动命令统一由 `ChassisTask` 执行。Ready、Busy 和 LastStatus 仅作诊断，不拦截入队。
- 2026-09-05 删除舵机启动失败时底盘任务的永久等待，以及舵机/仓库/转盘状态对指令入队的跨模块拦截；运行中可排队，队列容量仍为 4，满时回复 ERR QUEUE_FULL。OK 表示已入队，不代表完成。
- STOP 清空待执行队列，保留当前动作的停止请求；只有底盘任务领取 STOP 之后的新命令时才清理旧请求。出队与 STOP 使用代次校验，避免被清空的旧命令开始执行。
- 启动时 IMU/电机准备失败不再永久锁住任务，后续命令可重试准备；实际运动的 IMU、UART、STOP、参数校验和动作完成判断继续有效。

## UART5 现场 STATUS

- 2026-09-05 修复 GCC newlib-nano 未链接浮点 printf 时 YAW/HEAD_ERR/HEAD_CORR/DIST/TARGET 为空：使用整数格式化输出小数，角度与修正保留两位、距离取整，不依赖 `_printf_float`。非有限或超范围数据输出 INVALID。
- 用户已确认 X42S 使用 Emm 固件，保持 8 字节 F6 格式和 S_Vel_IS=Enable 的 0.1 RPM 编码；不是 X 固件的 9 字节 F6。

- `STATUS` 只输出 `STATE`、`IMU`、`YAW`、`HEAD_ERR`、`HEAD_CORR`、`DIST`、`TARGET`、`LAST`、`BALL_STATE`、`BALL_ROUND`、`PATH_STATE`、`PATH_STEP`、`PATH_LAST`、`PATH_BALL_LAST`、`WAREHOUSE_STATE`、`WAREHOUSE_BALL`、`STOP`、`STOPPED`、`STAIR_STATE`、`STAIR_LAST`、`TURNTABLE_STATE` 和 `TURNTABLE_LAST`。
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
- `User/Device/camera/maixcam_link.*` owns UART4 byte reception and accepts only an ASCII reply line `1` as a MaixCAM acknowledgement; outgoing request is one byte, `1` for red or `2` for blue. The deployed `licang_BLUE_RED_BALL.py` accepts a new command after the prior request state is cleared and checks from the next frame for a color/shape-qualified complete target inside the yellow ROI. AUTO calibration updates only the measured ball dimensions and center, leaving the ROI unchanged; MANUAL ROI defines the yellow search area directly with two touch points. AUTO-calibrated size limits apply in both ROI modes. No green target rectangle or multi-frame confirmation is used.
- UART5 command `BALL` first runs action group 1 (return/recognition posture). It then runs the remaining rounds of the current six-ball warehouse batch: request MaixCAM, wait up to 10 s for reply `1`, run action group 2 (clamp), turn the warehouse one slot, then run action group 1 (return). The sixth group-2 completion also turns one slot and is followed by group 1 return.
- `GRAB` remains a separate single cycle: group 2 (clamp) -> one turntable slot -> group 1 (return). During BALL, later commands may queue and execute serially after it finishes.
- MaixCAM timeout or UART4 transmission failure starts no servo action and allows a later `BALL` retry. A group 1/2 communication failure retains the existing arm error lock. `STOP` cancels an acknowledgement wait immediately; during group 2/turntable it still completes group 1 (return) before ending the remaining batch.

## BALL gray alignment (2026-08-25)

- The four gray sensors are ordered from left to right as `MID2`, `IN2`, `IN1`, `MID1`: `MID2=PD8`, `IN2=PD0`, `IN1=PD1`, `MID1=PD3`.
- Inputs use GPIO pull-ups and active-low line detection. The logical `OnLine` order is therefore `0 1 1 0` for the only valid alignment state: both inner sensors on the line and both outer sensors off the line.
- `User/Algorithm/gray_align.*` runs before BALL action group 1. It locks the current continuous JY61P yaw at entry, moves only along the left/right axis at 25 RPM, and applies a small heading PD correction (`KP=2.0`, `KD=0.15`, limit 8 RPM) during the lateral move. It holds the exact target for 50 ms, stops all wheels, and resets the continuous JY61P yaw before returning success. The alignment timeout is 5 s.
- `MID1`/`MID2` are overshoot protection sensors, not completion sensors. If either is on, the chassis retreats while maintaining the locked yaw; otherwise it approaches. IN1/IN2 appearing in sequence never commands a chassis rotation. IMU loss during alignment stops the chassis and returns an alignment error.

## RZ pillar ball sequence (2026-08-26)

- UART5 command `RZ` enters the existing `ChassisCommandQueue` and is executed by `ChassisTask`; it first uses PD10 to approach the pillar, locks the yaw during chassis positioning, then stops and settles before any arm or vision action. After the stable IR trigger, RZ starts the camera group directly without an extra approach move.
- After the chassis is positioned, RZ starts `SERVO_ACTION_PILLAR_CAMERA_GROUP` (group 3) once without requiring a completion frame, waits the fixed `RZ_CAMERA_RAISE_WAIT_MS` mechanical interval, and then starts `RoundPillar_OrbitAndGrab()`. The integrated routine sends the first red request and keeps the single-direction pillar orbit active while polling the MaixCAM reply.
- RZ succeeds when the configured `RZ_ORBIT_TARGET_DEG` is reached and the chassis completes its stop/settle sequence; `RZ_GRAB_COUNT` only limits the maximum balls processed, so finding fewer balls does not fail RZ.
- A valid reply stops and settles the chassis before `SERVO_ACTION_PILLAR_GRAB_GROUP` (group 4); after Group4 completes, the next red request is sent and the orbit resumes from the current continuous yaw. The RZ orbit now uses the reversed counter-clockwise direction (`forward=-62 RPM`, `omega=+49 RPM`) and continues from its reset heading to the configured `+352 degrees`; it completes directly at that target after four grabs, with no reverse stage.
- Group 4 contains clamp, release and camera re-positioning, so group 3 is never repeated. RZ does not call `BALL`, group 1, group 2 or `WarehouseControl`; after each successful Group4 it advances the turntable one slot, requests the next red ball and continues the current 355-degree orbit.
- STOP during approach or MaixCAM waiting cancels before the next arm action. Once group 3 or group 4 starts, that group is allowed to finish; a pending STOP is handled before the next vision request. MaixCAM and servo failures map to their dedicated RZ result statuses and do not trigger later group 4 actions.

## Warehouse turntable coordination (2026-08-25)

- `ZDT_MOTOR_ADDR = 0x05U` is the single authoritative warehouse-motor address. Chassis addresses 1–4 on USART3 remain unchanged.
- One slot is a relative `FD` move of `1280` pulses. `TURNTABLE_SLOT_DIRECTION` is currently `ZDT_DIR_CW`, speed is 100 RPM and acceleration is 0; all are centralized in `User/Device/turntable/turntable_control.h` for hardware calibration.
- `ServoAction_RunGroup(2)` waits for the real UART7 completion frame (`55 55 05 08 02 ...`). Only after that frame does `WarehouseControl_HandleActionGroup2Completed()` issue one turntable move. A command transmission is not considered completion.
- The installed turntable driver has no returned completion/status parser. The initial completion criterion is its calculated 240 ms running time plus 600 ms settling margin (840 ms total), bounded by a 1500 ms timeout. This must be verified on hardware before competition.
- A successful group-2/turntable pair increments `Warehouse_BallCount`. The counter is capped at six; `WAREHOUSE_TURN_AFTER_LAST_BALL` is `1U`, so the sixth ball also triggers a turn. A UART failure or timeout stops the turntable and enters the warehouse error state without incrementing the count.
- If `STOP` is already pending when group 2 completes, no new turntable command is sent; group 1 still runs to return the arm, then the remaining warehouse batch is canceled. During a turntable timing wait, `STOP` sends the driver stop command and likewise does not increment the count.

## IMU closed-loop in-place rotation (2026-08-25)

- `MotionControl_RotateDeg(angle_deg)` rotates about the chassis centre using the existing mecanum inverse kinematics: positive angle is counter-clockwise/left and negative angle is clockwise/right.
- Rotation is measured only by `Jy61P_GetContinuousYaw()`; it has no time- or encoder-pulse-based completion estimate. The heading baseline is reset at the start and after a settled successful rotation, so following translation holds the new vehicle heading.
- UART5 accepts `ROT CCW <deg>` / `ROT CW <deg>` (1..360 degrees)；旋转由 `ChassisTask` 直接执行，不再经过路径编辑器。
- Default parameters are 50 RPM cruise, 15 RPM approach, 8 RPM minimum effective speed, deceleration from 30 degrees, fine control from 10 degrees, 0.8-degree tolerance, five 20-ms settle periods, 250-ms ramp, and 8-s timeout.

## CANGKU 仓库搬运流程（2026-09-04）

- `User/Robot/cangku_task.*`：`CANGKU` 仓库搬运流程；由 `ChassisTask` 执行旋转、0110 灰度对齐、向左横移 50 mm、移动、动作组和反向转盘步骤。

## RFID 自动卡号采集（2026-09-05）

- UART8 PE0=RX、PE1=TX，9600-8-N-1，地址 0x20，匹配厂家 V1.0.5 手册默认参数。
- 读写器预设为自动读卡号、主动上传，推荐一直读；固件不在上电时修改读写器 EEPROM。
- 使用完整帧的长度、命令、地址、状态和异或取反校验，保存真实 32 位 UID；不限制为 1..9，不建立固定编号映射。
- BALL 在夹取组 2 前清除旧数据，保留夹取期间的卡号，整批去重，最多 5 个 UID；重试保留成功记录，转盘步骤成功后才追加卡号。
- `BALL_Get_Grabbed_ID()` / `BALL_Get_ID_List()` 返回 `uint32_t *`。仓库机械计数仍为 6；CANGKU 尚未按 UID 分拣。
- UART5 `RFID` 提供不消费数据的接收、卡号和批次查询。配置和实物验收见 [RFID.md](docs/RFID.md)。
- 全局 HAL UART 回调由 `User/Robot/state_machine.c` 分发，各 Device 模块只负责自身设备。

## STAIR 阶梯测试流程（2026-08-29）

- UART5 无参数命令 `STAIR` 由 `ChassisTask` 串行执行；开始前仅检查 `Turntable_IsReady()`，不初始化或修改仓库状态。
- 流程首先使用仅供 STAIR 的 `GrayAlign_RunUnlimited()`；灰度成功后先以极坐标 180° 后退 20 mm，再实际依次执行第一段 `G11`、第二段 `G8`、第三段 `G5`。灰度失败、IMU/电机/舵机/转盘/MaixCAM 通信失败均有独立状态，1 秒未找到红球是正常的 `NOT_FOUND` 分支。BALL 仍使用普通 `GrayAlign_Run()`。
- 搜索姿态组 G5/G8/G11 使用 `ServoAction_StartGroupNoWait()` 并等待 1000 ms；抓取组 G6/G9/G12、过渡组 G7/G10 和第三段末尾的 G0 使用 20 s 真实完成回包等待。
- 第一段：G11 后静止识别；命中执行 G12，随后后退 90 mm；未命中直接后退 90 mm；再次静止识别，命中执行 G12，随后执行 G10，未命中直接执行 G10，最后后退 117 mm。该 117 mm 的终点就是第二段第 1 点；第二段在此执行 G8/G9 搜索，再依次后退 90 mm 到第 2、3、4 点，因此四个点位之间总共只有三次 90 mm 移动；第四点结束后执行 G7 再后退 117 mm。第三段：G5 后识别，命中执行 G6；后退 90 mm 后再次静止识别，命中执行 G6 后执行 G0，未命中直接执行 G0。
- 每次成功 G6、G9 或 G12 后，直接调用现有 `Turntable_MoveOneSlotAndWait()` 转动一格（1280 脉冲）；不调用 `WarehouseControl`，不增加 `Warehouse_BallCount`。
- 第二段的 90 mm 搜索移动使用 STAIR 专用 40 RPM，并在运动控制 20 ms 周期中轮询当前 MaixCAM 请求，视觉提前命中即停车且不补足剩余距离；第一段和第三段均完成后退 90 mm，再重新发请求静止识别 1 s。117 mm 过渡均以后退 180°执行，不使用视觉截断。
- `STATUS` 增加 `STAIR_STATE`、`STAIR_LAST`、`TURNTABLE_STATE` 和 `TURNTABLE_LAST`，便于区分灰度、动作组、视觉和转盘阶段。
