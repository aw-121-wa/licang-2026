# GCC Build and DAPLink Flash Guide

## Prerequisites

1. **ARM GNU Toolchain** (arm-none-eabi-gcc)
   - Download: https://developer.arm.com/downloads/-/gnu-rm
   - Add to PATH

2. **MinGW** (for mingw32-make)
   - Or use Ninja instead

3. **OpenOCD** or **pyOCD** (for flashing)
   - OpenOCD: https://github.com/openocd-org/openocd
   - pyOCD: `pip install pyocd`

4. **CMake** 3.21+

## Quick Start

### Option 1: One-click build and flash
```batch
build_and_flash.bat
```

### Option 2: Step by step

#### Build
```batch
build_gcc.bat
```

Or manually:
```batch
mkdir build-gcc
cd build-gcc
cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/gcc-arm.cmake ..
mingw32-make
```

#### Flash
Using OpenOCD:
```batch
flash_openocd.bat
```

Using pyOCD:
```batch
flash_pyocd.bat
```

## Files Created

| File | Description |
|------|-------------|
| `cmake/toolchains/gcc-arm.cmake` | GCC ARM toolchain config |
| `cmake/startup_stm32f750xx.s` | GCC startup file |
| `cmake/STM32F750V8Tx_FLASH.ld` | GCC linker script |
| `CMakeLists_gcc.txt` | CMakeLists for GCC (rename to CMakeLists.txt to use) |
| `build_gcc.bat` | Build script |
| `flash_openocd.bat` | Flash with OpenOCD |
| `flash_pyocd.bat` | Flash with pyOCD |
| `build_and_flash.bat` | Combined build and flash |

## Using CMakeLists_gcc.txt

To use the GCC build, rename the files:
```batch
ren CMakeLists.txt CMakeLists_armcc.txt
ren CMakeLists_gcc.txt CMakeLists.txt
```

## Troubleshooting

### "arm-none-eabi-gcc not found"
Add ARM GNU Toolchain to PATH:
```batch
set PATH=%PATH%;C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin
```

### "mingw32-make not found"
Use Ninja instead:
```batch
cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/gcc-arm.cmake ..
ninja
```

### Flash fails
- Check DAPLink connection
- Try: `pyocd list` to see connected probes
- Try: `openocd -f interface/cmsis-dap.cfg -f target/stm32f7x.cfg` to test
