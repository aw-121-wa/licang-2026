# 最近工作状态

基线日期：2026-08-24

## 已完成

- 电机、麦轮解算、运动控制和 IMU 已独立成模块。
- `DIAGONAL_TEST_ENABLE=1` 时默认测试为左前 45°、500 mm 单次平移，并使用 JY60/JY61P 锁头；设为 0 时恢复前进 1 m、停止 0.5 s、左移 1 m。
- 四轴位置命令全部发送成功后才同步触发，避免部分电机先动。
- VS Code 已配置 ARMCC IntelliSense 和默认 Keil Build Task。
- `.vscode/build.ps1` 已验证通过，Keil 结果为 0 error、0 warning。
- 修正了“JY60 无有效角度帧导致整车完全不启动”：默认降级为无锁头运动，并增加 IMU 原始字节数和校验错误计数。
- 修复电机一卡一卡：取消把 1 m 拆成 10 条短位置命令，恢复为每个动作一条四轴同步 `FD` 位置命令。
- 按新要求改为 `F6` 速度模式实时控制：20 ms更新四轮速度和IMU锁头，指定距离按下发RPM与时间开环积分，不读取电机返回值；轮径改为75 mm。
- 用户确认四台驱动器的 `S_Vel_IS` 已开启；程序保留0.1 RPM编码，但不再上电重复发送配置命令。
- 新增 `MotionControl_MovePolarMm()` 和四个前/后左右包装接口，支持 -180°～+180° 任意平移角度；`MotionControl_MoveMm()` 现在支持前进和横移分量同时非零。
- 新增 `MecanumKinematics_DesaturateWithScale()`，将轮速限幅比例纳入距离积分，并增加极坐标、单位向量、有效 RPM、轨迹距离等 Keil Watch 诊断量。
- `App/competition_path.c/.h` 当前正式路径改为左前 `+30°` 斜线运行 1800 mm，使用 F6 速度模式并软减速到 0。
- 本次路径为单段斜线，不使用方向 Blend；IMU 等待、四轮使能、锁头基准、20 ms 锁头控制和开环距离积分仍由原有模块负责。
- `main.c` 不再初始化或执行底盘动作；`ChassisTask` 在 FreeRTOS 启动后初始化 `MotionControl`，完成 IMU 等待、四轮使能和锁头基准建立，再等待 UART5 命令。
- 从 `MotionControl_RunDefaultSequence()` 抽取 `MotionControl_PrepareForMove()`，保留原有 IMU 等待、四轮使能和锁头基准建立流程，并供应用层路径复用。
- 新增 UART5（PC12/PD2，115200）PC/VOFA 命令接口、单字节 IT 接收、CommandTask 和 ChassisCommandQueue；UART5 不直接发送电机协议。
- 新增静态 RAM 比赛路径表，最多 20 段，支持 `PATH CLEAR/ADD/SHOW/RUN/LOAD DEFAULT`；`LF + F` 组合复用非零末速度和方向 Blend。
- ChassisTask 不再上电自动执行默认路径，改为等待 UART5 的手动命令或 `PATH RUN`；默认路径通过 `PATH LOAD DEFAULT` 载入。
- 增加 `MotionControl_RequestStop()` 停止请求，STOP 在 CommandTask 中快速置位，并由运动控制周期处理。

## 当前已知风险

- 上电等待 IMU 最多2秒；无有效角度帧不启动，运动中IMU掉线立即停车。
- 尚未读取驱动器应答、报警和真实位置完成状态。
- 距离由指令RPM和时间积分估算，未使用电机反馈；电压、负载和麦轮横移打滑会形成实车距离误差。
- 对角线方向在理论上会产生两轮主导的轮速组合，但仍需架空车轮和实车确认方向、距离及限幅后的轨迹误差。
- 航向PD、修正符号及横移增益仍需实车标定。
- 路径动作已移入 FreeRTOS `ChassisTask`；底层单段运动内部仍使用既有 20 ms 控制周期和 HAL 时间基准。
- 正式路径采用 0→85→0 RPM；正常结束时发送软减速后的 0 RPM，不额外调用 `MotorControl_StopAll()`。
- UART5 接收采用单字节 `HAL_UART_Receive_IT()`；当前命令行队列长度 4，ChassisCommandQueue 长度 4，路径表最大 20 段。

## 下次工作入口

先架空车轮验证四轮方向，再落地标定前进/横移距离和航向修正符号。若需要让其他任务与底盘同时运行，应优先把阻塞动作序列改成 FreeRTOS 状态机任务。
