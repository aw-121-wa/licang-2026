# RFID Ball-ID Capture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Add UART8 RFID ID capture to the existing BALL sequence, keep the complete ID domain at 9 values, and cap each BALL batch at 5 saved IDs.

**Architecture:** Add a small `App/RFID` driver that owns one-byte UART8 interrupt reception on PE0/PE1 and exposes clear/read operations. Extend the existing `ball_sequence` module instead of creating a second BALL state machine: after group 2 completes, clear stale RFID data, wait for a valid non-duplicate ID, save it, then continue the existing turntable/return flow. The existing six-ball `WarehouseControl` accounting remains unchanged so its mechanical turntable state is not silently redefined.

**Tech Stack:** STM32F750 HAL, FreeRTOS/CMSIS-RTOS, C99, CMake GCC/ARMCC projects, Keil UV project, Python `unittest` static contract tests.

## Global Constraints

- UART8 uses PE0 as RX and PE1 as TX.
- UART8 uses 115200 baud, 8 data bits, no parity, one stop bit, TX/RX mode, and no hardware flow control.
- RFID input is a single byte; only IDs `1` through `9` are valid.
- `BALL_ID_MAX` is `9U`; `BALL_GRAB_MAX` is `5U`.
- The existing MaixCAM, RZ, STAIR, CANGKU, turntable, and warehouse modules keep their protocols and behavior.
- No debug counters, debug framework, or blocking UART polling are added.
- All UART receive callbacks remain routed through the existing callback fan-out in `IMU/jy61p.c`.
- Run `.vscode/build.ps1` after source changes; if the Keil executable is unavailable, also run the GCC CMake build and report the exact limitation.

---

### Task 1: Add failing RFID/BALL contract tests

**Files:**
- Create: `tests/test_ball_rfid.py`

**Interfaces:**
- Tests the planned public symbols and integration points without requiring a target board.

- [ ] **Step 1: Write the failing tests**

Add tests that assert all of the following are present:

```python
assert "BALL_ID_MAX" and "BALL_GRAB_MAX" in App/ball_sequence.h
assert "all_ball_id" and "grabbed_ball_id" and "grabbed_ball_count" in App/ball_sequence.h
assert "BALL_SEQUENCE_WAITING_RFID" in App/ball_sequence.h
assert "RFID_Read_ID" and "RFID_Clear" in App/RFID/rfid.h
assert "UART8" and "PE0" and "PE1" and "115200" in the UART sources
assert "RFID_UartRxCpltCallback" in IMU/jy61p.c
assert "RFID_Read_ID" and "BALL_GRAB_MAX" in App/ball_sequence.c
assert "App/RFID/rfid.c" in both CMake source lists
```

Also assert that `App/warehouse_control.h` still contains `WAREHOUSE_TOTAL_BALLS 6U`, proving the five-ID capture cap is not a silent warehouse accounting change.

- [ ] **Step 2: Run the test and verify the expected failure**

Run:

```powershell
python -m pytest tests/test_ball_rfid.py -q
```

Expected: FAIL because the RFID directory, UART8 symbols, and BALL RFID state do not yet exist.

### Task 2: Implement the standalone RFID receiver

**Files:**
- Create: `App/RFID/rfid.h`
- Create: `App/RFID/rfid.c`

**Interfaces:**
- Produces `void RFID_Init(void)`.
- Produces `uint8_t RFID_Read_ID(uint8_t *id)`.
- Produces `uint8_t RFID_Get_ID(uint8_t *id)` as a compatibility alias to `RFID_Read_ID`.
- Produces `void RFID_Clear(void)`.
- Produces `void RFID_UartRxCpltCallback(UART_HandleTypeDef *huart)`.
- Produces `void RFID_UartErrorCallback(UART_HandleTypeDef *huart)`.

- [ ] **Step 1: Implement the receive state**

Use one static receive byte, one pending ID byte, and one pending flag. `RFID_Init()` clears the pending state and starts `HAL_UART_Receive_IT(&huart8, &rx_byte, 1U)`. The completion callback must accept only `&huart8`, copy a valid raw byte into the pending ID, set the flag, and immediately arm the next one-byte reception. Invalid values are discarded while reception continues.

- [ ] **Step 2: Implement clear/read behavior**

`RFID_Clear()` removes any pending ID before a BALL wait. `RFID_Read_ID()` returns `1U` only when a valid pending ID exists, writes it to the caller, and consumes it; null pointers and empty state return `0U`. `RFID_Get_ID()` calls `RFID_Read_ID()`.

- [ ] **Step 3: Implement UART error recovery**

For UART8 errors, clear the overrun flag, clear `ErrorCode`, and restart one-byte interrupt reception. Do not add logging or counters.

- [ ] **Step 4: Run the contract test**

Run:

```powershell
python -m pytest tests/test_ball_rfid.py -q
```

Expected: the RFID-file assertions pass; UART8 and BALL integration assertions remain red.

### Task 3: Configure UART8 on the STM32F750

**Files:**
- Modify: `Core/Inc/usart.h`
- Modify: `Core/Src/usart.c`
- Modify: `Core/Inc/stm32f7xx_it.h`
- Modify: `Core/Src/stm32f7xx_it.c`
- Modify: `Core/Src/main.c`
- Modify: `chassis_motor.ioc`

**Interfaces:**
- Produces global `UART_HandleTypeDef huart8`.
- Produces `void MX_UART8_Init(void)`.
- Produces `UART8_IRQHandler(void)`.

- [ ] **Step 1: Add the UART8 handle and initializer declaration**

Declare `huart8` and `MX_UART8_Init()` in `usart.h`; define the handle and initializer in `usart.c` with `115200`, `UART_WORDLENGTH_8B`, `UART_STOPBITS_1`, `UART_PARITY_NONE`, `UART_MODE_TX_RX`, `UART_HWCONTROL_NONE`, and oversampling 16.

- [ ] **Step 2: Add PE0/PE1 MSP configuration**

In `HAL_UART_MspInit`, configure `RCC_PERIPHCLK_UART8` from PCLK1, enable UART8/GPIOE clocks, configure PE0 and PE1 as AF push-pull with `GPIO_AF8_UART8`, and enable `UART8_IRQn`. Add the matching deinit branch.

- [ ] **Step 3: Add the IRQ handler**

Declare and implement `UART8_IRQHandler()` to call `HAL_UART_IRQHandler(&huart8)`.

- [ ] **Step 4: Start UART8 reception during system initialization**

Call `MX_UART8_Init()` with the other generated peripheral initializers and call `RFID_Init()` after UART initialization and before the scheduler starts. Include `RFID/rfid.h` through the existing `App` include root.

- [ ] **Step 5: Route callbacks through the existing fan-out**

In `IMU/jy61p.c`, include `RFID/rfid.h` and call both RFID callback functions from the existing `HAL_UART_RxCpltCallback()` and `HAL_UART_ErrorCallback()` fan-out. Do not create another global HAL callback.

- [ ] **Step 6: Update the CubeMX project description**

Add PE0/PE1 UART8 signals, UART8 asynchronous mode, 115200 baud, and UART8 global interrupt entries to `chassis_motor.ioc` while preserving all existing peripheral mappings.

### Task 4: Integrate five-ID capture into the existing BALL sequence

**Files:**
- Modify: `App/ball_sequence.h`
- Modify: `App/ball_sequence.c`

**Interfaces:**
- Produces `all_ball_id[9]`, `grabbed_ball_id[5]`, and `grabbed_ball_count`.
- Produces `uint8_t *BALL_Get_Grabbed_ID(void)` and `uint8_t *BALL_Get_ID_List(void)`.
- Extends `BallSequenceState` with `BALL_SEQUENCE_WAITING_RFID`.
- Extends `BallSequenceStatus` with `BALL_SEQUENCE_ERROR_RFID_TIMEOUT`.

- [ ] **Step 1: Add the separated ID storage**

Define:

```c
#define BALL_ID_MAX 9U
#define BALL_GRAB_MAX 5U

extern uint8_t all_ball_id[BALL_ID_MAX];
extern uint8_t grabbed_ball_id[BALL_GRAB_MAX];
extern uint8_t grabbed_ball_count;
```

Initialize `all_ball_id` to `1` through `9`; clear the five-entry cache and count in `BallSequence_Init()` and at the start of every new `BallSequence_Run()`.

- [ ] **Step 2: Cap only the current BALL batch at five**

Keep `WAREHOUSE_TOTAL_BALLS` at six. Set the local run count to the smaller of `WarehouseControl_RemainingBallCount()` and `BALL_GRAB_MAX`; reject zero remaining balls as before. This makes the first BALL batch capture five IDs without rewriting warehouse accounting.

- [ ] **Step 3: Add the RFID wait state after group 2 completion**

After `ServoAction_RunGroup(SERVO_ACTION_GRAB_GROUP, ...)` succeeds, call `RFID_Clear()`, enter `BALL_SEQUENCE_WAITING_RFID`, and wait in the task context with `osDelay(1U)` until `RFID_Read_ID()` supplies a valid ID or the existing BALL timeout/STOP condition occurs. No turntable move happens before the ID is accepted.

- [ ] **Step 4: Save IDs with immediate duplicate suppression**

Maintain a private `last_rfid_id`, initialized to `0U` for each batch. Accept only IDs in `1..9`; ignore an ID equal to the immediately previous accepted ID and continue waiting. On acceptance, append to `grabbed_ball_id`, update `last_rfid_id`, and increment `grabbed_ball_count` without exceeding `BALL_GRAB_MAX`.

- [ ] **Step 5: Preserve cleanup and existing mechanics**

On a valid ID, call the existing `WarehouseControl_HandleActionGroup2Completed()` exactly once, then run the existing group 1 return action. On RFID STOP or timeout after group 2, skip the turntable, still run group 1 return, and return the corresponding BALL status. If the fifth ID is saved, finish after the normal group 2/turntable/return cleanup; do not start a sixth visual request in that batch.

- [ ] **Step 6: Expose the cache to CANGKU**

Return `grabbed_ball_id` from both `BALL_Get_Grabbed_ID()` and the compatibility name `BALL_Get_ID_List()`. Do not add CANGKU routing logic in this task.

- [ ] **Step 7: Update state/status names**

Return `"WAIT_RFID"` for the new state and `"RFID_TIMEOUT"` for the new status. Keep existing status names and error semantics unchanged.

### Task 5: Add build-system and project-file source membership

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `CMakeLists_armcc.txt`
- Modify: `MDK-ARM/chassis_motor.uvprojx`

**Interfaces:**
- Both CMake builds and the Keil project compile `App/RFID/rfid.c`.

- [ ] **Step 1: Add the RFID source to GCC CMake**

Insert `App/RFID/rfid.c` beside the other application sources in `CMakeLists.txt`.

- [ ] **Step 2: Add the RFID source to ARMCC CMake**

Insert `App/RFID/rfid.c` beside `App/ball_sequence.c` in `CMakeLists_armcc.txt`.

- [ ] **Step 3: Add RFID files to the Keil project**

Add `rfid.c` and `rfid.h` under the application source group with paths `../App/RFID/rfid.c` and `../App/RFID/rfid.h`.

### Task 6: Verify the complete integration

**Files:**
- Test: `tests/test_ball_rfid.py`
- Check: all modified C/H, CMake, IOC, and UV project files

- [ ] **Step 1: Run the new contract tests**

```powershell
python -m pytest tests/test_ball_rfid.py -q
```

Expected: PASS.

- [ ] **Step 2: Run the full Python contract suite**

```powershell
python -m pytest tests -q
```

Expected: no new failures attributable to RFID/BALL changes. Record any pre-existing failure separately.

- [ ] **Step 3: Run the GCC CMake build**

```powershell
cmake --build build-gcc -- -j 4
```

Expected: firmware compiles and links; report any existing linker warnings exactly.

- [ ] **Step 4: Run the required project build script**

```powershell
& .\.vscode\build.ps1
```

Expected: report actual error/warning counts. If the script cannot find the configured Keil executable, state that limitation rather than claiming a Keil build passed.

- [ ] **Step 5: Perform a final scope check**

Confirm that no MaixCAM, RZ, STAIR, CANGKU, turntable, warehouse-total, or ordinary motion source was behaviorally changed, and that `grabbed_ball_id` remains capped at five while valid RFID IDs remain `1..9`.
