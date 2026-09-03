# UART5 PATH 固定比赛流程 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增一个 UART5 `PATH` 命令，用现有运动、BALL 和 RZ API 原子执行固定比赛路线。

**Architecture:** UART5 只向现有底盘命令队列投递一个 `CHASSIS_CMD_PATH`。`App/path_sequence.c` 内部按静态 `PathStep` 表同步调用现有 `MotionControl_MovePolarSegmentMm()`、`MotionControl_RotateDeg()`、`BallSequence_Run()` 和 `RoundPillar_Run()`，不复制任何既有业务流程。FreeRTOS 任务只负责分发 PATH 并映射结果，STOP 继续由现有停止请求通道处理。

**Tech Stack:** STM32F750 C99、FreeRTOS、CMSIS-RTOS、CMake/Keil MDK 工程、Python `unittest` 源码契约测试。

## Global Constraints

- 不执行 `git reset --hard`、`git checkout -- .` 或 `git clean`，保留用户其它修改。
- 不修改 `BallSequence_Run()`、`RoundPillar_Run()`、`GrayAlign_Run()`、STAIR、GRAB、Warehouse/Turntable、MotionControl 原 API 和普通命令行为。
- 不恢复旧动态 PATH 编辑器；UART5 `PATH` 只运行编译期固定静态路径表。
- PATH 运行期间保持 `ChassisCommand_Busy=1`；STOP 必须能中止整条路径且不能继续后续步骤。
- PATH 默认顺序固定为：LF +45° 1850 mm → F 2300 mm → ROT +178° → BALL → ROT +180° → B 1820 mm → RZ → 停车。
- 修改后执行 `.vscode/build.ps1`，报告实际 Error/Warning 数量。

---

### Task 1: PATH 源码契约测试

**Files:**
- Modify: `tests/test_competition_cleanup.py`

**Interfaces:**
- Consumes: `App/path_sequence.c/.h`、`App/uart_command.*`、`Core/Src/freertos.c`、构建文件和 Keil 工程。
- Produces: 能检查固定步骤顺序、错误短路、STOP/BUSY 接入和两个构建系统收录情况的测试。

- [ ] **Step 1: Write the failing tests**

  将原先“PATH 必须不存在”的清理契约改为“旧 `competition_path` 不存在、固定 `path_sequence` 存在”，并增加以下源码契约断言：

  ```python
  def test_path_sequence_is_static_and_wired(self):
      path_h = self.read("App/path_sequence.h")
      path_c = self.read("App/path_sequence.c")
      uart_h = self.read("App/uart_command.h")
      uart_c = self.read("App/uart_command.c")
      freertos_c = self.read("Core/Src/freertos.c")
      cmake_c = self.read("CMakeLists.txt")
      cmake_armcc = self.read("CMakeLists_armcc.txt")
      uvprojx = self.read("MDK-ARM/chassis_motor.uvprojx")

      self.assertFalse((ROOT / "App/competition_path.c").exists())
      self.assertFalse((ROOT / "App/competition_path.h").exists())
      self.assertIn("PATH_STEP_MOVE_POLAR", path_h)
      self.assertIn("PATH_STEP_ROTATE", path_h)
      self.assertIn("PATH_STEP_BALL", path_h)
      self.assertIn("PATH_STEP_RZ", path_h)
      for token in ("1850U", "45.0f", "2300U", "178.0f", "180.0f", "1820U"):
          self.assertIn(token, path_c)
      self.assertIn("BallSequence_Run()", path_c)
      self.assertIn("RoundPillar_Run()", path_c)
      self.assertIn("CHASSIS_CMD_PATH", uart_h + uart_c + freertos_c)
      self.assertIn('"PATH\\r\\n"', uart_c)
      self.assertIn("PathSequence_Run()", freertos_c)
      self.assertIn("App/path_sequence.c", cmake_c)
      self.assertIn("App/path_sequence.c", cmake_armcc)
      self.assertIn("<FileName>path_sequence.c</FileName>", uvprojx)
      self.assertNotIn("PATH CLEAR", uart_c)
      self.assertNotIn("PATH ADD", uart_c)
      self.assertNotIn("PATH LOAD DEFAULT", uart_c)

  def test_path_sequence_stops_on_failure_before_later_steps(self):
      path_c = self.read("App/path_sequence.c")
      self.assertIn("return PathSequence_Finalize(PATH_SEQUENCE_CANCELED)", path_c)
      self.assertIn("return PathSequence_Finalize(path_status)", path_c)
      self.assertIn("MotionControl_WasStopped()", path_c)
      self.assertLess(path_c.index("BallSequence_Run()"), path_c.index("MotionControl_RotateDeg(180.0f)"))
      self.assertLess(path_c.index("MotionControl_RotateDeg(180.0f)"), path_c.index("RoundPillar_Run()"))
  ```

- [ ] **Step 2: Run the focused tests and verify RED**

  Run:

  ```text
  python -m unittest tests.test_competition_cleanup -v
  ```

  Expected: FAIL because `App/path_sequence.c/.h` and `CHASSIS_CMD_PATH` do not yet exist.

### Task 2: PATH 编排模块

**Files:**
- Create: `App/path_sequence.h`
- Create: `App/path_sequence.c`

**Interfaces:**
- Consumes: 现有四类同步 API 和 `MotionControl_StopRequested`/`MotionControl_WasStopped()`。
- Produces: `PathSequence_Run()`、可在 Watch 中查看的状态、按固定表执行和错误映射。

- [ ] **Step 1: Define the static step/state/status interfaces**
- [ ] **Step 2: Implement the seven-step table and synchronous executor**
- [ ] **Step 3: Map BALL/RZ/Motion 错误并在 STOP/失败时立即终止**
- [ ] **Step 4: Run focused tests and verify GREEN**

### Task 3: UART5/FreeRTOS/构建工程接入

**Files:**
- Modify: `App/uart_command.h`
- Modify: `App/uart_command.c`
- Modify: `Core/Src/freertos.c`
- Modify: `CMakeLists.txt`
- Modify: `CMakeLists_armcc.txt`
- Modify: `MDK-ARM/chassis_motor.uvprojx`

**Interfaces:**
- Consumes: `PathSequence_Run()` and `PathSequenceStatus`。
- Produces: 无参数 `PATH\r\n` 命令、`OK PATH`/`ERR FORMAT`/`ERR BUSY`/`ERR PATH_NOT_READY` 响应和持续 Busy 语义。

- [ ] **Step 1: Add `CHASSIS_CMD_PATH` without changing existing enum values**
- [ ] **Step 2: Parse/submit PATH with readiness checks and add HELP text**
- [ ] **Step 3: Dispatch PATH from `ChassisTask` and map result to existing motion status**
- [ ] **Step 4: Add the source to GCC, ARMCC CMake and Keil App group**
- [ ] **Step 5: Run the complete source tests**

### Task 4: 文档同步和固件构建

**Files:**
- Modify: `PROJECT.md`
- Modify: `REQUIREMENTS.md`

- [ ] **Step 1: Document static PATH semantics and exact default route**
- [ ] **Step 2: Run `.vscode/build.ps1`**
- [ ] **Step 3: Verify 0 errors/0 warnings and inspect the final diff**

