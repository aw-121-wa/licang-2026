# Global Heading Target Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep a persistent global heading target across chassis actions while retaining the tuned heading controller.

**Architecture:** `motion_control` owns ContinuousYaw initialization, HeadingTarget and heading-controller history. Normal translation and GrayAlign obtain omega from HeadingTarget; active rotations and pillar orbit use their own global relative targets, then hand the final heading back to `motion_control`.

**Tech Stack:** STM32 HAL, FreeRTOS, CMake/Ninja host build, Keil MDK project, Python unittest host checks.

## Global Constraints

- Do not modify mecanum kinematics, X42S protocol, JY61P frame parsing, or existing Heading PD tuning constants.
- `Jy61P_ResetContinuousYaw()` may only establish the global reference after IMU startup.
- Positive rotation remains counter-clockwise; all targets use continuous, unnormalized yaw degrees.
- A successful active rotation writes its theoretical target; cancellation and actionable failures capture actual yaw only when IMU is online.
- Preserve user-owned unrelated changes and avoid CRLF-only diffs.

---

### Task 1: Specify and test the persistent heading API

**Files:**
- Modify: `User/Algorithm/motion_control.h`
- Modify: `tests/host/chassis_runtime.c`
- Modify: `tests/test_competition_cleanup.py`

**Interfaces:**
- Produces: `extern volatile float MotionControl_HeadingTargetDeg`
- Produces: `void MotionControl_CaptureHeadingTarget(void)`
- Produces: `void MotionControl_SetHeadingTarget(float heading_deg)`
- Produces: `float MotionControl_GetHeadingTarget(void)`

- [ ] **Step 1: Write the failing host assertion**

```c
yaw = 12.0f;
MotionControl_SetHeadingTarget(15.0f);
assert(fabsf(MotionControl_GetHeadingTarget() - 15.0f) < 0.01f);
MotionControl_ResetHeadingReference();
assert(fabsf(yaw - 12.0f) < 0.01f);
assert(fabsf(MotionControl_GetHeadingTarget() - 12.0f) < 0.01f);
```

- [ ] **Step 2: Run the test and verify it fails under reset-to-zero semantics**

Run: `python -B -m unittest discover -s tests -p test_chassis_runtime.py -v`

Expected: FAIL in the reset assertion.

- [ ] **Step 3: Replace obsolete static expectations**

```python
self.assertNotIn("MotionControl_ResetHeadingReference", gray_c)
self.assertIn("orbit_target_yaw", rz_c)
self.assertIn("MotionControl_SetHeadingTarget(orbit_target_yaw)", rz_c)
self.assertNotIn("Jy61P_ResetContinuousYaw", rz_c)
```

- [ ] **Step 4: Run the static test and verify it fails until the target API exists**

Run: `python -B -m unittest tests/test_competition_cleanup.py -v`

Expected: FAIL because the target API and orbit assignment are absent.

### Task 2: Implement global target ownership and translation correction

**Files:**
- Modify: `User/Algorithm/motion_control.c`
- Modify: `User/Algorithm/motion_control.h`

**Interfaces:**
- Consumes: API declarations and host expectations from Task 1.
- Produces: persistent heading target initialization and target-relative Heading PD error.

- [ ] **Step 1: Add state and history reset**

```c
volatile float MotionControl_HeadingTargetDeg = 0.0f;
static uint8_t motion_heading_target_valid = 0U;

static void MotionControl_ResetHeadingControllerState(void)
{
    previous_heading_error = 0.0f;
    heading_rate_deg_s = 0.0f;
    heading_sample_valid = 0U;
    MotionControl_HeadingErrorDeg = 0.0f;
    MotionControl_HeadingCorrectionRpm = 0.0f;
}
```

- [ ] **Step 2: Implement the target API and compatibility function**

```c
void MotionControl_CaptureHeadingTarget(void)
{
    if (Jy61P_IsOnline(GYRO_ONLINE_TIMEOUT_MS) == 0U) { return; }
    MotionControl_HeadingTargetDeg = Jy61P_GetContinuousYaw();
    motion_heading_target_valid = 1U;
    MotionControl_ResetHeadingControllerState();
}

void MotionControl_ResetHeadingReference(void)
{
    MotionControl_CaptureHeadingTarget();
}
```

- [ ] **Step 3: Establish global yaw once in PrepareForMove**

```c
Jy61P_ResetContinuousYaw();
MotionControl_HeadingTargetDeg = 0.0f;
motion_heading_target_valid = 1U;
MotionControl_ResetHeadingControllerState();
```

Place it only in the successful IMU branch of `MotionControl_PrepareForMove()`.

- [ ] **Step 4: Change only the source of Heading PD error**

```c
if ((MotionControl_ImuHeadingHoldActive == 0U) ||
    (motion_heading_target_valid == 0U)) { return 0.0f; }
error = MotionControl_HeadingTargetDeg - Jy61P_GetContinuousYaw();
```

Retain all existing gain selection, filtering, prediction, deadband and limits.

- [ ] **Step 5: Run targeted tests**

Run: `python -B -m unittest discover -s tests -p 'test_*.py' -v`

Expected: target API test passes; tests awaiting active-orbit work may still fail.

### Task 3: Make RotateDeg global and failure-safe

**Files:**
- Modify: `User/Algorithm/motion_control.c`
- Modify: `tests/host/chassis_runtime.c`

**Interfaces:**
- Consumes: `MotionControl_SetHeadingTarget()` and `MotionControl_CaptureHeadingTarget()`.
- Produces: global rotation target and safe target handling for completion, STOP, timeout and motor UART failure.

- [ ] **Step 1: Add failing consecutive-rotation assertions**

```c
assert(MotionControl_RotateDeg(90.0f) == MOTION_STATUS_FINISHED);
assert(fabsf(MotionControl_GetHeadingTarget() - 90.0f) < 0.01f);
assert(MotionControl_RotateDeg(90.0f) == MOTION_STATUS_FINISHED);
assert(fabsf(MotionControl_GetHeadingTarget() - 180.0f) < 0.01f);
```

- [ ] **Step 2: Run it and verify old local targets fail**

Run: `python -B -m unittest discover -s tests -p test_chassis_runtime.py -v`

Expected: FAIL because completion clears the local target.

- [ ] **Step 3: Implement relative global rotation**

```c
rotate_start_heading = Jy61P_GetContinuousYaw();
MotionControl_RotateTargetDeg = rotate_start_heading + angle_deg;
```

Use global ContinuousYaw as current rotation angle. On settle call `MotionControl_SetHeadingTarget(MotionControl_RotateTargetDeg)` before clearing rotate-only debug values.

- [ ] **Step 4: Capture actual yaw for active-rotation interruption**

On STOP, timeout and motor UART error, stop motors first and then call `MotionControl_CaptureHeadingTarget()` only if IMU remains online. Keep target unchanged after IMU loss.

- [ ] **Step 5: Run host tests**

Run: `python -B -m unittest discover -s tests -p test_chassis_runtime.py -v`

Expected: PASS, including two consecutive rotations and hard-stop behavior.

### Task 4: Preserve heading through GrayAlign and RoundPillar

**Files:**
- Modify: `User/Algorithm/gray_align.c`
- Modify: `User/Robot/round_pillar.c`
- Modify: `tests/test_competition_cleanup.py`

**Interfaces:**
- Consumes: target API from Task 2.
- Produces: GrayAlign without coordinate reset plus global relative pillar orbit logic.

- [ ] **Step 1: Extend static tests for normal and active actions**

Use the Task 1 Python assertions, plus checks that the orbit direction compares current yaw with `orbit_target_yaw` for both signs of `RZ_ORBIT_TARGET_DEG`.

- [ ] **Step 2: Remove GrayAlign reset calls**

Keep `MotionControl_GetHeadingCorrection()` and its existing lateral command unchanged at entry, while searching, and after stable line detection.

- [ ] **Step 3: Implement global relative orbit target**

```c
orbit_start_yaw = Jy61P_GetContinuousYaw();
orbit_target_yaw = orbit_start_yaw + RZ_ORBIT_TARGET_DEG;
reached = (RZ_ORBIT_TARGET_DEG >= 0.0f) ?
          (current_yaw >= orbit_target_yaw) :
          (current_yaw <= orbit_target_yaw);
```

Keep existing nonzero orbit omega as the only heading command. On settled success call `MotionControl_SetHeadingTarget(orbit_target_yaw)`.

- [ ] **Step 4: Handle interrupted orbit**

For stop, timeout, motor, camera, servo or turntable failures after orbit begins, stop chassis and capture actual yaw only while the IMU is online.

- [ ] **Step 5: Run static tests**

Run: `python -B -m unittest tests/test_competition_cleanup.py -v`

Expected: PASS with no state-machine continuous-yaw reset.

### Task 5: Expose, document, build, and inspect

**Files:**
- Modify: `User/Task/uart_command.c`
- Modify: `PROJECT.md`
- Modify: `REQUIREMENTS.md`

**Interfaces:**
- Consumes: `MotionControl_GetHeadingTarget()`.
- Produces: `HEAD_TARGET=<degrees>` in STATUS and matching acceptance documentation.

- [ ] **Step 1: Add the STATUS field**

```c
UartCommand_FormatNumber(numbers[5], sizeof(numbers[5]),
                         MotionControl_GetHeadingTarget(), 2);
```

Insert `HEAD_TARGET=%s` after `YAW=%s` and expand the numbers array.

- [ ] **Step 2: Update documentation**

Document global yaw initialization, persistent target, active completion and cancellation rules, and the STATUS field.

- [ ] **Step 3: Check whitespace**

Run: `git diff --check`

Expected: no whitespace errors in source and documentation changes.

- [ ] **Step 4: Build target projects**

Run: `.vscode/build.ps1`

Run: `python C:/Users/lovec/.codex/skills/build-cmake/scripts/build_cmake.py --source . --build-dir build-gcc`

Expected: Keil reports environment status or a zero-error build; CMake produces `build-gcc/chassis_motor.elf` with zero compilation errors and no new warnings.

- [ ] **Step 5: Run tests and inspect final call sites**

Run: `python -B -m unittest discover -s tests -p 'test_*.py' -v`

Run: `rg -n "Jy61P_ResetContinuousYaw|MotionControl_SetBodySpeed" User`

Expected: reset remains only in global initialization; active nonzero omega is limited to `MotionControl_RotateDeg` and `RoundPillar_OrbitAndGrab`.

- [ ] **Step 6: Commit the focused change**

```bash
git add User/Algorithm/motion_control.c User/Algorithm/motion_control.h User/Algorithm/gray_align.c User/Robot/round_pillar.c User/Task/uart_command.c PROJECT.md REQUIREMENTS.md tests/host/chassis_runtime.c tests/test_competition_cleanup.py docs/superpowers
git commit -m "feat: add persistent global heading target control"
```
