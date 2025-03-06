include(${CMAKE_CURRENT_LIST_DIR}/miosix.toolchain.cmake)

set(MIOSIX_OPT_BOARD stm32f756zg_nucleo)
set(MIOSIX_ARCH_NAME cortexM7_stm32f7)
set(MIOSIX_CPU_MICROARCH armv7m)

set(MIOSIX_BSP_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bsps/${MIOSIX_OPT_BOARD})
set(MIOSIX_LINKER_SCRIPT ${MIOSIX_BSP_PATH}/stm32_1m+256k_rom.ld)

# Basic flags
set(FLAGS_BASE "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -D_DEFAULT_SOURCE=1")
set(LFLAGS_BASE "-Wl,--gc-sections -Wl,-T${MIOSIX_LINKER_SCRIPT} -nostdlib")

# Flags for C/C++
string(TOUPPER ${MIOSIX_OPT_BOARD} BOARD_UPPER)

set(CMAKE_C_FLAGS_INIT "${FLAGS_BASE}")
set(CMAKE_CXX_FLAGS_INIT "${CMAKE_C_FLAGS_INIT}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${LFLAGS_BASE}")

include(CMakePrintHelpers)

cmake_print_variables(CMAKE_CURRENT_LIST_DIR)
cmake_print_variables(MIOSIX_LINKER_SCRIPT)
