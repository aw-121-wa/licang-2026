@echo off
REM Flash using OpenOCD with DAPLink

echo Flashing chassis_motor.hex via OpenOCD...
openocd -f interface/cmsis-dap.cfg -f target/stm32f7x.cfg -c "program build-gcc/chassis_motor.hex verify reset exit"

if %ERRORLEVEL% EQU 0 (
    echo Flash successful!
) else (
    echo Flash failed!
    exit /b 1
)
