# Ball Grab Handshake Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a BALL round execute action group 2 only after a fresh, color-qualified MaixCAM2 success reply.

**Architecture:** Keep the existing `BallSequence` ordering and servo/warehouse modules. MaixCAM2 will gate recognition by command, a 150 ms arm delay, trigger-zone membership, and three consecutive valid frames. `MaixCamLink` will send one color byte and accept a `1\n` line only after that request has been transmitted successfully.

**Tech Stack:** STM32 HAL/C with FreeRTOS, MaixCAM2 MicroPython, existing Python selftest, Keil build via `.vscode/build.ps1`.

## Global Constraints

- Only change code directly involved in MaixCAM2 ball recognition and the STM32 ball handshake.
- Preserve the existing action groups, turntable behavior, chassis, IMU, path, and FreeRTOS architecture.
- Do not add debug variables, counters, `printf`, logs, status fields, test commands, or protocol messages.
- Preserve the `1`/`2` request and `1\n` success protocol; no ACK/READY/START/DONE/OK/ERROR additions.
- Group 2 is legal only after group 1 succeeds, the current request is sent, and a fresh valid reply is received.

---

### Task 1: MaixCAM2 regression behavior

**Files:**
- Modify: `licang_BLUE_RED_BALL.py` selftest section

**Interfaces:**
- Consumes: `set_mode()`, `update_detection()`, `ticks_ms()`.
- Produces: executable assertions for command gating, 150 ms delay, trigger-zone gating, and three-frame confirmation.

- [x] **Step 1: Write the failing assertions**

  Replace the old single-frame assertions with these required behaviors:

  ```python
  process_command_bytes(b"2")
  assert recognition_armed is True
  assert update_detection(valid, serial, now_ms=0) is None
  assert serial.sent == []
  assert update_detection(valid, serial, now_ms=VISION_ARM_DELAY_MS - 1) is None
  assert serial.sent == []
  assert update_detection(valid, serial, now_ms=VISION_ARM_DELAY_MS) is None
  assert serial.sent == []
  assert update_detection(outside_trigger, serial, now_ms=VISION_ARM_DELAY_MS + 1) is None
  assert serial.sent == []
  update_detection(valid, serial, now_ms=VISION_ARM_DELAY_MS + 2)
  update_detection(valid, serial, now_ms=VISION_ARM_DELAY_MS + 3)
  assert serial.sent == []
  update_detection(valid, serial, now_ms=VISION_ARM_DELAY_MS + 4)
  assert serial.sent == [b"1\n"]
  ```

- [ ] **Step 2: Run the selftest and verify it fails for the old behavior**

  Run `python licang_BLUE_RED_BALL.py --selftest`.

  Expected result: failure because the old `update_detection()` has no `now_ms` parameter and emits on the first valid frame.

### Task 2: MaixCAM2 recognition gate

**Files:**
- Modify: `licang_BLUE_RED_BALL.py`

**Interfaces:**
- Consumes: the existing single-byte `1`/`2` commands and blob detection path.
- Produces: one `1\n` reply only after delay, trigger-zone membership, and three consecutive valid frames.

- [ ] **Step 1: Add only the required constants/state**

  Add `VISION_ARM_DELAY_MS = 150`, `DETECT_CONFIRM_FRAMES = 3`, and a command-arm timestamp. Reset the timestamp, latch, and streak in the UART branch of `set_mode()`.

- [ ] **Step 2: Implement the minimal gate**

  Make `update_detection()` reject frames before the delay, reject blobs outside `TRIGGER_ZONE`, reset the streak on invalid frames, increment it on valid frames, and send/latch exactly once at three frames.

- [ ] **Step 3: Run the selftest and verify it passes**

  Run `python licang_BLUE_RED_BALL.py --selftest` and require exit code 0.

### Task 3: STM32 color request and fresh-reply gate

**Files:**
- Modify: `App/maixcam_link.h`
- Modify: `App/maixcam_link.c`
- Modify: `App/ball_sequence.c`

**Interfaces:**
- Consumes: existing UART4 interrupt reception and `BallSequence_Run()`.
- Produces: `MaixCamColor`, `MaixCamLink_SendRequest(MaixCamColor)`, and a reply accepted only while the current request is active.

- [ ] **Step 1: Add the color enum and change the request signature**

  Define `MAIXCAM_COLOR_RED = 1` and `MAIXCAM_COLOR_BLUE = 2`; map them to one-byte `'1'`/`'2'` requests and reject other values.

- [ ] **Step 2: Add the business transaction gate**

  Clear the pending reply and line buffer before sending. Set the request-active flag only after `HAL_UART_Transmit()` returns `HAL_OK`. In line completion, accept `1` only when the request-active flag is set, then clear the flag.

- [ ] **Step 3: Make BALL select the existing target color**

  Keep the current single-round structure and action groups; use a local compile-time target color set to `MAIXCAM_COLOR_RED` because the existing `BALL` command has no color parameter.

- [ ] **Step 4: Verify failure paths**

  Confirm by inspection that UART send failure, timeout, STOP, and group-1 failure return before `ServoAction_RunGroup(SERVO_ACTION_GRAB_GROUP, ...)`.

### Task 4: Documentation and verification

**Files:**
- Modify: `PROJECT.md` and `REQUIREMENTS.md` only where their MaixCAM protocol description is stale.

- [ ] **Step 1: Synchronize the protocol wording**

  Document one-byte `1`/`2` requests, `1\n` replies, fresh-request gating, and the 150 ms/three-frame/trigger-zone conditions without adding new runtime behavior.

- [ ] **Step 2: Run the required build**

  Run `.vscode/build.ps1`; require `MDK-ARM/chassis_motor/chassis_motor.hex` and 0 errors/0 warnings.

- [ ] **Step 3: Run scope checks**

  Review `git diff --check`, verify no forbidden debug additions or protocol tokens were introduced, and report hardware-only tests that remain.
