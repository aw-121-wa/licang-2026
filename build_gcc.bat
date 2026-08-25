@echo off
REM Build with GCC using CMake

echo Building with GCC...

if not exist build-gcc mkdir build-gcc
cd build-gcc

cmake -G "MSYS Makefiles" -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/gcc-arm.cmake ..
if %ERRORLEVEL% NEQ 0 (
    echo CMake configuration failed!
    exit /b 1
)

make -j%NUMBER_OF_PROCESSORS%
if %ERRORLEVEL% NEQ 0 (
    echo Build failed!
    exit /b 1
)

cd ..
echo Build successful! Output: build-gcc/chassis_motor.hex
