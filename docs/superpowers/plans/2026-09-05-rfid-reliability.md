# RFID automatic UID capture and focused cleanup

**Goal:** Automatically receive real tag UIDs using the supplied vendor protocol.

**Architecture:** Retain Device/Robot/Task boundaries. Device validates complete UART frames; BALL stores at most five complete UIDs; Robot routes global UART callbacks.

**Constraints:** No fixed IDs or UID mapping. UART8 defaults to 9600 8N1, address 0x20. Reader must be configured for automatic active upload. No EEPROM writes, flashing or physical motion in this change.

- [x] Read project, vendor V1.0.5 manual and STM32 auto-read example.
- [x] Reproduce capture timing, duplicate, retry, capacity and premature commit bugs using real C sources.
- [x] Parse full frames and preserve 32-bit UIDs, including zero; recover receive arming.
- [x] Consolidate BALL completion/return handling, deduplicate full batch and commit after turn success.
- [x] Move global callbacks into Robot and share BALL/PATH status mapping.
- [x] Add non-consuming RFID diagnostics and update project, requirements and bench guide.
- [x] Final behavioral tests, existing contracts, required Keil script, GCC build and diff review.

## Verification

- Host GCC `-O2 -Wall -Wextra -Werror`: 13 RFID/BALL runtime scenarios pass.
- Full Python suite: 31 tests, 29 passed, 2 existing failures (PATH move-count expectation 5 versus current 6; STAIR timeout expectation 500 versus current 1000 ms). Missing REQUIREMENTS error resolved.
- Required `.vscode/build.ps1` executed with process execution-policy override: configured `G:\Keil_v5\UV4\UV4.exe` missing, no Keil compiler error/warning counts available.
- CMake skill helper could not import `tool_config`; direct existing `cmake --build build-gcc -- -j 4` succeeded: 0 errors, 10 warnings (existing nested PATH comment, 8 newlib syscall stubs, RWX load segment).
- Source/document diff whitespace check passed. Generated linker map retains tool-generated trailing spaces.
- No flashing or real reader/robot test performed. Reader automatic upload configuration and physical capture position remain bench checks.
