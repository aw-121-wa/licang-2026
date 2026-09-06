# 全局航向目标控制设计

## 目标

将底盘从每个动作重新清零 IMU 连续航向的控制方式，改为启动后建立一次全局连续航向，并由 `HeadingTarget` 跨普通移动、灰度对齐、旋转和绕柱持续保持车头方向。

## 约束

- 保留现有前进和横移分离的 Heading PD 参数、滤波、预测、死区、限幅与麦轮解算。
- 不改 JY61P 串口解析、X42S 协议或路径动作顺序。
- `Jy61P_ResetContinuousYaw()` 只允许在运动系统完成 IMU 启动后建立世界参考时调用。
- 普通移动和 GrayAlign 只读取 `HeadingTarget` 产生 omega；主动旋转和绕柱自行输出 omega，二者不并行叠加。

## 接口与状态

`motion_control` 对外提供：

- `MotionControl_HeadingTargetDeg`：当前全局保持目标。
- `MotionControl_CaptureHeadingTarget()`：IMU 在线时把当前连续航向作为目标，并清除 PD 历史。
- `MotionControl_SetHeadingTarget(float heading_deg)`：写入理论目标，并清除 PD 历史。
- `MotionControl_GetHeadingTarget()`：读取目标。

控制器内部单独维护目标是否有效。IMU 数据尚未就绪时，航向修正输出为零，避免虚假的零度目标驱动车身旋转。

`MotionControl_ResetHeadingReference()` 保留为兼容 API，但语义改为捕获当前航向；它不再清除 ContinuousYaw。新的内部函数只清 Heading PD 的误差、滤波和调试历史。

## 数据流

启动时 `MotionControl_PrepareForMove()` 在确认 IMU 在线后清除一次 ContinuousYaw，并建立 `HeadingTarget=0`。随后普通移动的误差为：

```
heading_error = HeadingTarget - ContinuousYaw
```

旋转使用：

```
rotate_target = ContinuousYaw_at_start + requested_delta
```

旋转在稳定完成时将理论 `rotate_target` 写入 HeadingTarget；收到 STOP、超时或电机通信失败时，若 IMU 仍在线，则捕获实际航向。这样恢复普通移动后不会回拉到旋转前方向。

绕柱同样记录 `orbit_start_yaw` 与 `orbit_target_yaw`，根据 `RZ_ORBIT_TARGET_DEG` 正负判断是否到达。完成后写入理论目标；中断或失败时捕获实际航向。

## 模块边界

- `motion_control.[ch]`：目标状态、控制器生命周期、旋转目标和公开 API。
- `gray_align.c`：移除与航向坐标重置有关的调用，继续使用既有航向修正。
- `round_pillar.c`：实现全局相对绕柱目标和失败收敛。
- `uart_command.c`：STATUS 输出 `HEAD_TARGET`。
- `tests/host/chassis_runtime.c` 与静态测试：验证新语义并防止重引入局部清零。

## 验证

主机测试覆盖：目标误差、连续两次旋转、旋转成功的理论目标、STOP 后捕获当前航向。静态测试确认 GrayAlign、RoundPillar 和旋转路径不再清零 ContinuousYaw。CMake 与 Keil 构建分别检查 GCC 主机固件和目标工程。
