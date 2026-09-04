# User Architecture Refactor Design

## Goal

在 `refactor-architecture` 分支中整理 STM32 机器人工程目录，只改变源文件归属和构建收录路径，不改变比赛流程、控制参数、通信协议、任务周期、动作组顺序或运行时语义。

## Constraints

- `Core/`、`Drivers/`、`Middlewares/` 保留为 CubeMX/HAL/CMSIS/FreeRTOS 目录。
- 生成代码 `main.c`、`freertos.c`、`usart.c`、`gpio.c`、中断和时钟文件仍保留在 `Core`；只把其中的用户初始化和任务实现移到用户层。
- 用户头文件的 basename 保持不变，例如仍使用 `#include "motion_control.h"`，不改成目录前缀形式。
- CMake、ARMCC CMake、Keil 工程和 VS Code IntelliSense 同步新的用户目录。
- `.vscode/build.ps1`、`tasks.json`、`settings.json` 保留，因为仓库规则和 VS Code 任务依赖它们。
- 当前缺失的 `App/competition_path.c` 不补功能；移除其陈旧的构建引用并记录为基线清理。

## Directory mapping

| Current file | New location | Responsibility |
| --- | --- | --- |
| `Motor/motor_control.*` | `User/BSP/` | 四轮驱动器协议和底盘电机收发 |
| `Motor/cangku_motor.*` | `User/BSP/` | 转盘步进电机协议 |
| `Motor/mecanum_kinematics.*` | `User/Algorithm/` | 麦轮逆解和限幅 |
| `Motor/motion_control.*` | `User/Algorithm/` | 速度、距离、航向和旋转控制 |
| `App/gray_align.*` | `User/Algorithm/` | 灰度对线控制 |
| `App/servo_action.*` | `User/Device/servo/` | 舵机动作组协议 |
| `App/RFID/rfid.*` | `User/Device/rfid/` | RFID UART8 接收 |
| `IMU/jy61p.*` | `User/Device/imu/` | JY61P 数据解析 |
| `App/maixcam_link.*` | `User/Device/camera/` | MaixCAM UART4 链路 |
| `App/turntable_control.*` | `User/Device/turntable/` | 转盘一格运动和等待 |
| `App/ball_sequence.*` | `User/Robot/` | BALL 比赛流程 |
| `App/round_pillar.*` | `User/Robot/` | RZ 绕桩流程 |
| `App/stair_sequence.*` | `User/Robot/` | STAIR 流程 |
| `App/path_sequence.*` | `User/Robot/` | PATH 固定流程 |
| `App/cangku_task.*` | `User/Robot/` | CANGKU 流程 |
| `App/warehouse_control.*` | `User/Robot/` | 仓库计数和转盘协同 |
| `App/uart_command.*` | `User/Task/` | UART5 命令接收和解析 |
| 用户初始化调用 | `User/Robot/state_machine.*` | 初始化现有设备状态，不新增比赛行为 |
| `StartChassisTask` 等实现 | `User/Task/task_control.*` | FreeRTOS 用户任务实现 |

## Configuration

`User/Config/robot_config.h` 保存现有运动、机构、动作组和超时参数的原值；原模块头文件通过兼容宏继续暴露旧名称。`User/Config/pin_config.h` 只描述已经在 CubeMX 生成文件中使用的语义引脚映射，不替换 `gpio.c/usart.c` 的初始化代码。

## Runtime data flow

`main.c` 只保留系统/HAL/CubeMX 初始化和 `MX_FREERTOS_Init()`。用户外设初始化在 `MX_FREERTOS_Init()` 创建任务前由用户层初始化函数执行，调用顺序保持：舵机、MaixCAM、RFID、BALL 状态初始化。`freertos.c` 只保留线程属性、队列创建和线程创建；`StartChassisTask`、`StartUartCommandTask` 的实现移动到 `User/Task/task_control.c`，函数签名不变。

## Verification

- 静态架构契约测试验证目录、构建清单、Keil 路径、主任务边界和配置值。
- 原有 Python 合约测试继续运行；已知两个历史参数不一致单独报告，不在本次修复。
- GCC CMake 完整编译并检查产物。
- `.vscode/build.ps1` 必须执行；若环境仍缺少 `G:\Keil_v5\UV4\UV4.exe`，报告环境阻塞而不伪装为通过。
- 使用 `git diff --check` 检查空白错误，并检查迁移前后的函数/宏/API 名称没有被改写。
