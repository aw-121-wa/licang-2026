# CMake 构建说明

本工程同时保留两种构建入口，二者编译的是同一套业务、HAL 和 FreeRTOS 源文件：

- Keil：`MDK-ARM/chassis_motor.uvprojx`，原有使用方式不变。
- CMake：根目录 `CMakeLists.txt`，使用 ARM Compiler 5，因此复用现有 `RVDS/ARM_CM7/r0p1` FreeRTOS 移植层和 Keil 启动文件，避免更换编译器后引入运行时差异。CMake 也对齐 Keil 的 Cortex-M7 单精度 FPU、`--split_sections` 和 64 KiB Flash/320 KiB RAM 限制。

## 前提

- CMake 3.21 或更高。
- Ninja。
- MDK ARM Compiler 5。默认路径为 `G:/Keil_v5/ARM/ARMCC`；若安装目录不同，在 PowerShell 中设置 `KEIL_ARMCC_ROOT` 为 ARMCC 目录后再配置，例如：

```powershell
$env:KEIL_ARMCC_ROOT = 'D:/Keil_v5/ARM/ARMCC'
```

## 构建

在工程根目录执行：

```powershell
cmake --preset armcc-debug
cmake --build --preset armcc-debug
```

调试构建产物为：

```text
build/armcc-debug/chassis_motor.axf
build/armcc-debug/chassis_motor.hex
```

发布构建使用 `armcc-release` 预设。CMake 输出固定在 `build/`，不会覆盖 `MDK-ARM/chassis_motor/` 内 Keil 的 `.axf` 或 `.hex`。

## VS Code

安装并启用 CMake Tools 后，选择 `armcc-debug` 或 `armcc-release` 预设即可配置和构建。原有 `Ctrl+Shift+B` 默认任务仍是 Keil 构建。
