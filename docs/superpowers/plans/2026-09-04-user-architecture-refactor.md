# User Architecture Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将用户代码迁移到 `User/BSP、Device、Algorithm、Robot、Task、Config`，保持现有 STM32 比赛逻辑和运行时行为不变。

**Architecture:** 保留 `Core/Drivers/Middlewares` 作为 CubeMX 和系统层，用户源文件按职责使用 Git rename 迁移且保留 basename。`main.c/freertos.c` 只保留系统初始化、RTOS 对象创建和线程入口壳，用户任务实现放入 `User/Task`；CMake、ARMCC CMake、Keil 和 VS Code 同步路径。

**Tech Stack:** STM32F750 HAL、FreeRTOS/CMSIS-RTOS2、C99、CMake/Ninja、ARMCC/Keil MDK、Python `unittest` 静态契约测试。

## Global Constraints

- 保留 `Core/`、`Drivers/`、`Middlewares/`，不修改 HAL、CMSIS、启动文件或 CubeMX 外设初始化逻辑。
- 不修改比赛算法参数、任务周期、通信协议、动作组编号、运动方向或流程顺序。
- 保留 `.vscode/build.ps1`、`.vscode/tasks.json`、`.vscode/settings.json`，因为仓库规则和构建任务使用它们。
- 头文件 basename 和公共函数名保持不变；只通过 include path 维持现有 `#include "xxx.h"`。
- 不补写缺失的 `App/competition_path.c`；从 ARMCC 清单移除陈旧引用。
- 每个迁移阶段都先运行针对性测试，再运行 GCC CMake；最终必须执行 `.vscode/build.ps1` 并报告真实结果。

---

### Task 1: 建立基线和架构目录契约

**Files:**
- Create: `backup_before_refactor/BASELINE_COMMIT.txt`
- Create: `docs/superpowers/specs/2026-09-04-user-architecture-design.md`
- Create: `tests/test_user_architecture.py`
- Modify: none in production code

**Interfaces:**
- Produces: static contract names for all `User` destinations and retained build tooling.

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class UserArchitectureContractTest(unittest.TestCase):
    def test_user_directory_layout_and_moved_sources_exist(self):
        expected = (
            "User/BSP/motor_control.c",
            "User/BSP/cangku_motor.c",
            "User/Algorithm/mecanum_kinematics.c",
            "User/Algorithm/motion_control.c",
            "User/Algorithm/gray_align.c",
            "User/Device/servo/servo_action.c",
            "User/Device/rfid/rfid.c",
            "User/Device/imu/jy61p.c",
            "User/Device/camera/maixcam_link.c",
            "User/Device/turntable/turntable_control.c",
            "User/Robot/ball_sequence.c",
            "User/Robot/round_pillar.c",
            "User/Robot/stair_sequence.c",
            "User/Robot/path_sequence.c",
            "User/Robot/cangku_task.c",
            "User/Robot/warehouse_control.c",
            "User/Task/uart_command.c",
            "User/Task/task_control.c",
            "User/Config/robot_config.h",
            "User/Config/pin_config.h",
        )
        for relative in expected:
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_cube_mx_directories_and_build_tools_are_retained(self):
        for relative in (
            "Core/Src/main.c",
            "Core/Src/freertos.c",
            "Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal.h",
            "Middlewares/Third_Party/FreeRTOS/Source/tasks.c",
            ".vscode/build.ps1",
            ".vscode/tasks.json",
            ".vscode/settings.json",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest discover -s tests -p test_user_architecture.py -v`

Expected: FAIL because the `User` destination files do not exist yet.

- [ ] **Step 3: Record the clean baseline**

Run: `git rev-parse HEAD`, `git branch --show-current`, `python -m unittest discover -s tests -v`, and `cmake --build build-gcc -- -j 4`.

Expected: baseline commit is `6c43dc02094d089f557b278fd7a6fe5280b4fd45`; existing test failures are only the documented `1600U/1650U` and `20U/18U` mismatches; GCC build completes with the pre-existing linker warnings.

- [ ] **Step 4: Commit the baseline contract**

```powershell
git add backup_before_refactor/BASELINE_COMMIT.txt docs/superpowers/specs/2026-09-04-user-architecture-design.md tests/test_user_architecture.py
git commit -m "refactor: create user architecture"
```

### Task 2: Move BSP and device modules without changing APIs

**Files:**
- Move: `Motor/motor_control.c/h` -> `User/BSP/`
- Move: `Motor/cangku_motor.c/h` -> `User/BSP/`
- Move: `App/servo_action.c/h` -> `User/Device/servo/`
- Move: `App/RFID/rfid.c/h` -> `User/Device/rfid/`
- Move: `IMU/jy61p.c/h` -> `User/Device/imu/`
- Move: `App/maixcam_link.c/h` -> `User/Device/camera/`
- Move: `App/turntable_control.c/h` -> `User/Device/turntable/`
- Modify: `CMakeLists.txt`, `CMakeLists_armcc.txt`, `MDK-ARM/chassis_motor.uvprojx`, `.vscode/c_cpp_properties.json`
- Test: `tests/test_user_architecture.py`

**Interfaces:**
- Consumes: existing public headers and function names.
- Produces: same headers/functions at new paths, with all build systems resolving them.

- [ ] **Step 1: Add failing build-manifest assertions**

```python
    def test_build_manifests_use_user_paths_for_bsp_and_devices(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        armcc = (ROOT / "CMakeLists_armcc.txt").read_text(encoding="utf-8")
        for source in (
            "User/BSP/motor_control.c",
            "User/BSP/cangku_motor.c",
            "User/Device/servo/servo_action.c",
            "User/Device/rfid/rfid.c",
            "User/Device/imu/jy61p.c",
            "User/Device/camera/maixcam_link.c",
            "User/Device/turntable/turntable_control.c",
        ):
            self.assertIn(source, cmake)
            self.assertIn(source, armcc)
        self.assertNotIn("App/competition_path.c", armcc)
```

- [ ] **Step 2: Run the targeted test and observe the expected path failures**

Run: `python -m unittest discover -s tests -p test_user_architecture.py -v`

Expected: FAIL on the new `User` source paths and old build-manifest entries.

- [ ] **Step 3: Move files with Git rename semantics**

Create these directories and move each source/header pair while preserving its basename. Do not edit file contents in this step. The intended command shape is:

```powershell
New-Item -ItemType Directory -Force User/BSP,User/Device/servo,User/Device/rfid,User/Device/imu,User/Device/camera,User/Device/turntable | Out-Null
git mv Motor/motor_control.c Motor/motor_control.h User/BSP/
git mv Motor/cangku_motor.c Motor/cangku_motor.h User/BSP/
git mv App/servo_action.c App/servo_action.h User/Device/servo/
git mv App/RFID/rfid.c App/RFID/rfid.h User/Device/rfid/
git mv IMU/jy61p.c IMU/jy61p.h User/Device/imu/
git mv App/maixcam_link.c App/maixcam_link.h User/Device/camera/
git mv App/turntable_control.c App/turntable_control.h User/Device/turntable/
```

- [ ] **Step 4: Update build manifests and include search paths**

Replace only path strings in CMake/Keil/IntelliSense. Keep source order and compiler flags. Add:

```text
User/BSP
User/Device/servo
User/Device/rfid
User/Device/imu
User/Device/camera
User/Device/turntable
```

Remove only the nonexistent `App/competition_path.c` line from `CMakeLists_armcc.txt`.

- [ ] **Step 5: Run the targeted test and GCC build**

Run: `python -m unittest discover -s tests -p test_user_architecture.py -v` and `cmake --build build-gcc -- -j 4`.

Expected: architecture test passes and GCC exits 0 with only the baseline linker warnings.

- [ ] **Step 6: Commit the BSP/device migration**

```powershell
git add User CMakeLists.txt CMakeLists_armcc.txt MDK-ARM/chassis_motor.uvprojx .vscode/c_cpp_properties.json tests/test_user_architecture.py
git commit -m "refactor: move device drivers"
```

### Task 3: Move algorithm and robot modules, then centralize unchanged parameters

**Files:**
- Move: `Motor/mecanum_kinematics.c/h`, `Motor/motion_control.c/h`, `App/gray_align.c/h` -> `User/Algorithm/`
- Move: `App/ball_sequence.c/h`, `App/round_pillar.c/h`, `App/stair_sequence.c/h`, `App/path_sequence.c/h`, `App/cangku_task.c/h`, `App/warehouse_control.c/h` -> `User/Robot/`
- Create: `User/Config/robot_config.h`, `User/Config/pin_config.h`
- Modify: module headers only where a value is moved to the config header; `CMakeLists.txt`, `CMakeLists_armcc.txt`, `MDK-ARM/chassis_motor.uvprojx`, `.vscode/c_cpp_properties.json`, `PROJECT.md`, `REQUIREMENTS.md`
- Test: `tests/test_user_architecture.py`

**Interfaces:**
- Consumes: the public APIs from Task 2 and existing module include basenames.
- Produces: unchanged motion/robot APIs and a config header containing the existing values, including wheel diameter `75U`, motion cruise `130.0f`, diagonal cruise `85.0f`, and current RZ/turntable parameters.

- [ ] **Step 1: Add failing assertions for algorithm/robot paths and config values**

```python
    def test_algorithm_robot_and_config_contract(self):
        for relative in (
            "User/Algorithm/mecanum_kinematics.h",
            "User/Algorithm/motion_control.h",
            "User/Algorithm/gray_align.h",
            "User/Robot/ball_sequence.h",
            "User/Robot/round_pillar.h",
            "User/Robot/stair_sequence.h",
            "User/Robot/path_sequence.h",
            "User/Robot/cangku_task.h",
            "User/Robot/warehouse_control.h",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)
        config = (ROOT / "User/Config/robot_config.h").read_text(encoding="utf-8")
        self.assertIn("#define MOTOR_WHEEL_DIAMETER_MM          75U", config)
        self.assertIn("#define MOTION_CRUISE_RPM          130.0f", config)
        self.assertIn("#define MOTION_DIAGONAL_CRUISE_RPM  85.0f", config)
        pins = (ROOT / "User/Config/pin_config.h").read_text(encoding="utf-8")
        self.assertIn("PD8", pins)
        self.assertIn("PD0", pins)
        self.assertIn("PD1", pins)
        self.assertIn("PD3", pins)
```

- [ ] **Step 2: Run the targeted test to verify the expected failures**

Run: `python -m unittest discover -s tests -p test_user_architecture.py -v`

Expected: FAIL because the algorithm/robot files and config headers have not moved or been created.

- [ ] **Step 3: Move the algorithm and robot files preserving file contents**

```powershell
New-Item -ItemType Directory -Force User/Algorithm,User/Robot,User/Config | Out-Null
git mv Motor/mecanum_kinematics.c Motor/mecanum_kinematics.h User/Algorithm/
git mv Motor/motion_control.c Motor/motion_control.h User/Algorithm/
git mv App/gray_align.c App/gray_align.h User/Algorithm/
git mv App/ball_sequence.c App/ball_sequence.h User/Robot/
git mv App/round_pillar.c App/round_pillar.h User/Robot/
git mv App/stair_sequence.c App/stair_sequence.h User/Robot/
git mv App/path_sequence.c App/path_sequence.h User/Robot/
git mv App/cangku_task.c App/cangku_task.h User/Robot/
git mv App/warehouse_control.c App/warehouse_control.h User/Robot/
```

- [ ] **Step 4: Create config files using current values only**

`robot_config.h` must contain the current public tuning values copied from the existing headers; it must not introduce new values or retune anything. Module headers may include it and retain compatibility macro names. `pin_config.h` must document `MID2=PD8`, `IN2=PD0`, `IN1=PD1`, `MID1=PD3`, `RZ_IR=PD10`, and the existing UART mappings without modifying CubeMX-generated pin initialization.

- [ ] **Step 5: Update all source/build paths and run tests/build**

Update only path entries in the two CMake manifests, Keil project, and `.vscode/c_cpp_properties.json`. Run:

```powershell
python -m unittest discover -s tests -p test_user_architecture.py -v
cmake --build build-gcc -- -j 4
```

Expected: targeted architecture tests pass and the firmware target exits 0.

- [ ] **Step 6: Commit algorithm/robot/config migration**

```powershell
git add User CMakeLists.txt CMakeLists_armcc.txt MDK-ARM/chassis_motor.uvprojx .vscode/c_cpp_properties.json PROJECT.md REQUIREMENTS.md tests/test_user_architecture.py
git commit -m "refactor: separate robot modules"
```

### Task 4: Extract user FreeRTOS task code and initialization

**Files:**
- Create: `User/Task/task_control.c`, `User/Task/task_control.h`
- Move: `App/uart_command.c/h` -> `User/Task/`
- Modify: `Core/Src/freertos.c`, `Core/Src/main.c`, `CMakeLists.txt`, `CMakeLists_armcc.txt`, `MDK-ARM/chassis_motor.uvprojx`, `.vscode/c_cpp_properties.json`
- Create: `User/Robot/state_machine.c`, `User/Robot/state_machine.h`
- Test: `tests/test_user_architecture.py`

**Interfaces:**
- Produces: `void StartChassisTask(void *argument)`, `void StartUartCommandTask(void *argument)`, and `void RobotUser_Init(void)` with the same call order as the current main initialization.

- [ ] **Step 1: Add failing task-boundary tests**

```python
    def test_main_and_freertos_keep_only_system_shells(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        freertos = (ROOT / "Core/Src/freertos.c").read_text(encoding="utf-8")
        task = (ROOT / "User/Task/task_control.c").read_text(encoding="utf-8")
        state = (ROOT / "User/Robot/state_machine.c").read_text(encoding="utf-8")
        self.assertNotIn("ServoAction_Init", main)
        self.assertNotIn("MaixCamLink_Init", main)
        self.assertNotIn("RFID_Init", main)
        self.assertNotIn("BallSequence_Init", main)
        self.assertNotIn("void StartChassisTask(void *argument)", freertos)
        self.assertIn("void StartChassisTask(void *argument)", task)
        self.assertIn("void RobotUser_Init(void)", state)
        self.assertIn("ServoAction_Init", state)
        self.assertIn("MaixCamLink_Init", state)
        self.assertIn("RFID_Init", state)
        self.assertIn("BallSequence_Init", state)
```

- [ ] **Step 2: Run the targeted test to verify it fails**

Run: `python -m unittest discover -s tests -p test_user_architecture.py -v`

Expected: FAIL because task code and initialization are still in `Core`/`main.c`.

- [ ] **Step 3: Extract the existing task bodies without editing logic**

Move the complete current definitions of `StartChassisTask` and `StartUartCommandTask` into `User/Task/task_control.c`, preserving local declarations, branches, error mappings, delays, queue calls, and function signatures. `task_control.h` exposes only the two task prototypes. `Core/Src/freertos.c` includes that header and retains task attributes, `MX_FREERTOS_Init`, queue creation, and `osThreadNew` calls.

- [ ] **Step 4: Extract initialization in the existing order**

Implement:

```c
void RobotUser_Init(void)
{
    ServoAction_Init(&huart7);
    MaixCamLink_Init(&huart4);
    RFID_Init();
    BallSequence_Init();
}
```

Call `RobotUser_Init()` from the `USER CODE` section of `MX_FREERTOS_Init()` before `osThreadNew`. Remove only the four corresponding application calls/includes from `main.c`; preserve all CubeMX initialization order and generated code.

- [ ] **Step 5: Move `uart_command` and update project manifests**

Move `App/uart_command.c/h` to `User/Task/`, add all new task/state sources to both CMake lists and the Keil project, and add `User/Task`, `User/Robot`, `User/Algorithm`, and all device include directories to IntelliSense. Keep `Core/Src/freertos.c` and `Core/Src/main.c` in their existing build positions.

- [ ] **Step 6: Run tests and GCC build**

Run: `python -m unittest discover -s tests -p test_user_architecture.py -v` and `cmake --build build-gcc -- -j 4`.

Expected: task-boundary test passes; GCC exits 0 with only previously known linker warnings.

- [ ] **Step 7: Commit the task split**

```powershell
git add Core/Src/main.c Core/Src/freertos.c User CMakeLists.txt CMakeLists_armcc.txt MDK-ARM/chassis_motor.uvprojx .vscode/c_cpp_properties.json tests/test_user_architecture.py
git commit -m "refactor: separate robot modules"
```

### Task 5: Final project documentation and verification

**Files:**
- Modify: `PROJECT.md`, `REQUIREMENTS.md`, `docs/worklogs/LATEST.md`
- Modify: `.vscode/c_cpp_properties.json` only if an include path is missing
- Test: `tests/test_user_architecture.py` and all existing `tests/`

**Interfaces:**
- Produces: documented directory map, unchanged hardware mapping, and verified build results.

- [ ] **Step 1: Add final static checks**

```python
    def test_old_user_source_paths_are_not_in_build_manifests(self):
        for relative in ("CMakeLists.txt", "CMakeLists_armcc.txt"):
            text = (ROOT / relative).read_text(encoding="utf-8")
            for old in ("App/ball_sequence.c", "App/path_sequence.c", "Motor/motion_control.c", "IMU/jy61p.c"):
                self.assertNotIn(old, text)

    def test_public_header_names_and_tooling_are_preserved(self):
        self.assertIn('#include "motion_control.h"', (ROOT / "User/Robot/path_sequence.c").read_text(encoding="utf-8"))
        self.assertIn('"${workspaceFolder}/User/Robot"', (ROOT / ".vscode/c_cpp_properties.json").read_text(encoding="utf-8"))
```

- [ ] **Step 2: Run the complete test suite**

Run: `$env:PYTHONDONTWRITEBYTECODE='1'; python -m unittest discover -s tests -v`

Expected: all architecture, RFID, CANGKU, and existing passing contracts pass; report the two known historical failures separately if they remain.

- [ ] **Step 3: Run CMake verification through the build skill path**

Run: `python C:/Users/lovec/.codex/skills/build-cmake/scripts/cmake_builder.py --source D:/stm32project/中国机器人大赛立体仓库/licang-2026 --build-dir D:/stm32project/中国机器人大赛立体仓库/licang-2026/build-gcc --generator Ninja --build-type Debug`

Expected: configure/build exit 0 and `build-gcc/chassis_motor.elf`, `.hex`, and `.bin` are present. Count compiler errors and linker warnings separately.

- [ ] **Step 4: Run the required Keil/VS Code build script**

Run: `powershell.exe -ExecutionPolicy Bypass -File .\.vscode\build.ps1`

Expected: either Keil reports 0 errors/0 warnings, or the command reports the exact missing Keil executable path without being misreported as a source failure.

- [ ] **Step 5: Review diff and commit documentation**

Run: `git diff --check` and `git status --short`. Confirm no deleted CubeMX/HAL/CMSIS files, no changed public function signatures, and no changed numerical behavior. Then commit:

```powershell
git add PROJECT.md REQUIREMENTS.md docs/worklogs/LATEST.md tests/test_user_architecture.py
git commit -m "refactor: cleanup project files"
```

