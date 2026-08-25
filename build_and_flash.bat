@echo off
REM Build and Flash script for STM32F750 with GCC + DAPLink

setlocal enabledelayedexpansion

echo ========================================
echo STM32F750 Build and Flash Tool
echo ========================================

REM Check for GCC
where arm-none-eabi-gcc >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: arm-none-eabi-gcc not found in PATH!
    echo Please install ARM GNU Toolchain and add to PATH.
    exit /b 1
)

REM Check for OpenOCD or pyOCD
set FLASH_TOOL=
where openocd >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set FLASH_TOOL=openocd
    goto :found_tool
)

where pyocd >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set FLASH_TOOL=pyocd
    goto :found_tool
)

echo ERROR: Neither OpenOCD nor pyOCD found in PATH!
echo Please install one of them:
echo   - OpenOCD: https://github.com/openocd-org/openocd
echo   - pyOCD: pip install pyocd
exit /b 1

:found_tool
echo Using flash tool: %FLASH_TOOL%

REM Switch to GCC CMakeLists if needed
if exist CMakeLists.txt (
    findstr /C:"ARMCC" CMakeLists.txt >nul
    if %ERRORLEVEL% EQU 0 (
        echo Switching to GCC CMakeLists...
        ren CMakeLists.txt CMakeLists_armcc.txt
        ren CMakeLists_gcc.txt CMakeLists.txt
    )
)

REM Build
echo.
echo [1/2] Building with GCC...
if not exist build-gcc mkdir build-gcc
cd build-gcc

cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/gcc-arm.cmake ..
if %ERRORLEVEL% NEQ 0 (
    echo CMake configuration failed!
    exit /b 1
)

ninja
if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    exit /b 1
)

cd ..

REM Flash
echo.
echo [2/2] Flashing via DAPLink...

if "%FLASH_TOOL%"=="openocd" (
    echo Using OpenOCD...
    openocd -f interface/cmsis-dap.cfg -f target/stm32f7x.cfg -c "program build-gcc/chassis_motor.elf verify reset exit"
) else (
    echo Using pyOCD...
    pyocd flash -t stm32f750v8tx build-gcc/chassis_motor.hex
    pyocd reset
)

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Build and flash successful!
    echo ========================================
) else (
    echo.
    echo ========================================
    echo Flash failed! Check DAPLink connection.
    echo ========================================
    exit /b 1
)

endlocal
