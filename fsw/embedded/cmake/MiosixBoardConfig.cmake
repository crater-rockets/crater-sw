set(KPATH ${CMAKE_CURRENT_SOURCE_DIR}/miosix-kernel/miosix)

if(MIOSIX_OPT_BOARD STREQUAL "stm32f756zg_nucleo")
    # Base directories with header files for this board
    set(ARCH_PATH ${KPATH}/arch/${MIOSIX_ARCH_NAME})
    set(BOARD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bsps/${MIOSIX_OPT_BOARD})
    set(BOARD_CONFIG_PATH ${PROJECT_PATH}/bsps/${MIOSIX_OPT_BOARD}/config)

    set(FLAGS_BASE 
            -D_BOARD_${BOARD_UPPER}
            -D_MIOSIX_BOARDNAME=\"${MIOSIX_OPT_BOARD}\"
            -D_ARCH_CORTEXM7_STM32F7
            -DHSE_VALUE=8000000
            -DSYSCLK_FREQ_216MHz=216000000
        )

    set(MIOSIX_ARCH_CFLAGS
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
else()
    message(FATAL_ERROR "Unsupported board '${MIOSIX_OPT_BOARD}'" )
endif()