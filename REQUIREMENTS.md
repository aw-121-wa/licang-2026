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

- On boot, UART7 (PE7/PE8, 9600-8-N-1) runs action group 0 (start pose) and waits for its completion frame.
- UART5 chassis commands remain rejected until the start pose completes.
- Two successfully completed chassis commands are accepted; a STOP or motion error does not increment the count.
- After the second successful command, action group 1 (8.24 disk-machine) runs once. On completion the sequence is locked.
- The two `.rob` files must be downloaded to controller slots 0 and 1; the MCU sends only invocation frames.
