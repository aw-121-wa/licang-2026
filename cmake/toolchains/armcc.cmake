# ARM Compiler 5 toolchain for the existing STM32F750V8Tx firmware.
# Set KEIL_ARMCC_ROOT before configuring if Keil is installed elsewhere.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ENV{KEIL_ARMCC_ROOT} AND NOT "$ENV{KEIL_ARMCC_ROOT}" STREQUAL "")
    set(ARMCC_ROOT "$ENV{KEIL_ARMCC_ROOT}")
else()
    set(ARMCC_ROOT "G:/Keil_v5/ARM/ARMCC")
endif()

file(TO_CMAKE_PATH "${ARMCC_ROOT}" ARMCC_ROOT)
if(NOT EXISTS "${ARMCC_ROOT}/Bin/armcc.exe")
    message(FATAL_ERROR
        "ARM Compiler 5 was not found at ${ARMCC_ROOT}. Set KEIL_ARMCC_ROOT "
        "to the directory containing Bin/armcc.exe.")
endif()

set(CMAKE_C_COMPILER "${ARMCC_ROOT}/Bin/armcc.exe" CACHE FILEPATH "ARMCC C compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARMCC_ROOT}/Bin/armasm.exe" CACHE FILEPATH "ARMCC assembler" FORCE)
set(CMAKE_AR "${ARMCC_ROOT}/Bin/armar.exe" CACHE FILEPATH "ARMCC archiver" FORCE)
set(CMAKE_LINKER "${ARMCC_ROOT}/Bin/armlink.exe" CACHE FILEPATH "ARMCC linker" FORCE)

# Match the Keil target: Cortex-M7 single-precision FPU, C99 and Thumb interwork.
set(CMAKE_C_FLAGS_INIT "--cpu=Cortex-M7.fp.sp --c99 --apcs=interwork")
set(CMAKE_ASM_FLAGS_INIT "--cpu=Cortex-M7.fp.sp")
# The Keil target uses -Otime even when debug information is enabled. Keeping
# that setting is required for this 64 KiB Flash device to fit its firmware.
set(CMAKE_C_FLAGS_DEBUG_INIT "-Otime -g")
set(CMAKE_C_FLAGS_DEBUG "-Otime -g" CACHE STRING
    "Debug flags matching the Keil target" FORCE)
