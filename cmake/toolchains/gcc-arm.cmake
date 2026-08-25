# GCC ARM Toolchain for STM32F750
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Find GCC in PATH or set custom path
find_program(ARM_GCC_C_COMPILER arm-none-eabi-gcc)
if(NOT ARM_GCC_C_COMPILER)
    message(FATAL_ERROR "arm-none-eabi-gcc not found. Install ARM GNU Toolchain and add to PATH.")
endif()

get_filename_component(ARM_GCC_BIN_DIR "${ARM_GCC_C_COMPILER}" DIRECTORY)

set(CMAKE_C_COMPILER "${ARM_GCC_BIN_DIR}/arm-none-eabi-gcc.exe" CACHE FILEPATH "GCC C compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_GCC_BIN_DIR}/arm-none-eabi-gcc.exe" CACHE FILEPATH "GCC ASM compiler" FORCE)
set(CMAKE_AR "${ARM_GCC_BIN_DIR}/arm-none-eabi-ar.exe" CACHE FILEPATH "GCC archiver" FORCE)
set(CMAKE_LINKER "${ARM_GCC_BIN_DIR}/arm-none-eabi-ld.exe" CACHE FILEPATH "GCC linker" FORCE)
set(CMAKE_OBJCOPY "${ARM_GCC_BIN_DIR}/arm-none-eabi-objcopy.exe" CACHE FILEPATH "GCC objcopy" FORCE)
set(CMAKE_OBJDUMP "${ARM_GCC_BIN_DIR}/arm-none-eabi-objdump.exe" CACHE FILEPATH "GCC objdump" FORCE)
set(CMAKE_SIZE "${ARM_GCC_BIN_DIR}/arm-none-eabi-size.exe" CACHE FILEPATH "GCC size" FORCE)

# Cortex-M7 with single-precision FPU
set(CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${CPU_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_C_FLAGS_DEBUG_INIT "-Os -g -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE_INIT "-Os -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_FLAGS} -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs")
