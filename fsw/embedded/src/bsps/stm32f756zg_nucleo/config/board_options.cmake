# Copyright (C) 2023 by Skyward
# Copyright (C) 2025 Luca Erbetta
#
# This program is free software; you can redistribute it and/or 
# it under the terms of the GNU General Public License as published 
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# As a special exception, if other files instantiate templates or use
# macros or inline functions from this file, or you compile this file
# and link it with other works to produce a work based on this file,
# this file does not by itself cause the resulting work to be covered
# by the GNU General Public License. However the source code for this
# file must still be made available in accordance with the GNU 
# Public License. This exception does not invalidate any other 
# why a work based on this file might be covered by the GNU General
# Public License.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>

set(MIOSIX_OPT_BOARD stm32f756zg_nucleo)
set(MIOSIX_ARCH_NAME cortexM7_stm32f7)
set(MIOSIX_CPU_MICROARCH armv7m)

# Base directories with header files for this board
set(ARCH_PATH ${KPATH}/arch/${MIOSIX_ARCH_NAME})
set(BOARD_PATH ${PROJECT_PATH}/src/bsps/${MIOSIX_OPT_BOARD})
set(BOARD_CONFIG_PATH ${PROJECT_PATH}/src/bsps/${MIOSIX_OPT_BOARD}/config)

set(MIOSIX_LINKER_SCRIPT ${BOARD_PATH}/stm32_1m+256k_rom.ld)

# Select clock frequency (HSE_VALUE is the xtal on board, fixed)
set(CLOCK_FREQ -DHSE_VALUE=8000000 -DSYSCLK_FREQ_216MHz=216000000)

# Basic flags
set(FLAGS_BASE -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Flags for ASM and linker
set(MIOSIX_ARCH_AFLAGS ${FLAGS_BASE})
set(MIOSIX_ARCH_LFLAGS ${FLAGS_BASE})

# Flags for C/C++
string(TOUPPER ${MIOSIX_OPT_BOARD} BOARD_UPPER)
set(MIOSIX_ARCH_CFLAGS
    -D_BOARD_${BOARD_UPPER}
    -D_MIOSIX_BOARDNAME=\"${MIOSIX_OPT_BOARD}\"
    -D_ARCH_CORTEXM7_STM32F7 
    ${CLOCK_FREQ}
    ${FLAGS_BASE}
)
set(MIOSIX_ARCH_CXXFLAGS ${MIOSIX_ARCH_CFLAGS})


set(MIOSIX_ARCH_INC
    ${ARCH_PATH}/common
    ${BOARD_PATH}
    ${BOARD_PATH}/config
    ${KPATH}/arch/cpu/${MIOSIX_CPU_MICROARCH}
)

# Select architecture specific files
set(MIOSIX_ARCH_SRC
    ${BOARD_PATH}/boot.cpp
    ${BOARD_PATH}/interfaces-impl/bsp.cpp

    ${ARCH_PATH}/common/interfaces-impl/delays.cpp

    ${KPATH}/arch/cpu/common/cortexMx_interrupts.cpp
    ${KPATH}/arch/cpu/common/cortexMx_userspace.cpp
    ${KPATH}/arch/cpu/${MIOSIX_CPU_MICROARCH}/interfaces-impl/cpu.cpp
    
    ${KPATH}/arch/common/sleep/cortexMx_sleep.cpp
    ${KPATH}/arch/common/cache/cortexMx_cache.cpp
    ${KPATH}/arch/common/drivers/stm32_serial_common.cpp
    ${KPATH}/arch/common/drivers/stm32f7_serial.cpp
    ${KPATH}/arch/common/drivers/sd_stm32f2_f4_f7.cpp
    ${KPATH}/arch/common/drivers/dcc.cpp
    ${KPATH}/arch/common/drivers/stm32_gpio.cpp
    ${KPATH}/arch/common/os_timer/stm32_32bit_os_timer.cpp
    ${KPATH}/arch/common/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
)
