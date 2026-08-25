@echo off
REM Flash using pyOCD with DAPLink

echo Flashing chassis_motor.hex via pyOCD...
pyocd flash -t stm32f750v8tx build-gcc/chassis_motor.hex

if %ERRORLEVEL% EQU 0 (
    echo Flash successful!
    pyocd reset
) else (
    echo Flash failed!
    exit /b 1
)
