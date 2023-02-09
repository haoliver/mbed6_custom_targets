# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(samcortexm0p-drivers INTERFACE)

target_compile_options(samcortexm0p-drivers INTERFACE
    -Wno-expansion-to-defined
)

target_include_directories(samcortexm0p-drivers
    INTERFACE
        .

        ../config/${SAM_TARGET_NAME}

        adc
        adc/${SAM_TARGET_NAME}
        dac
        dac/${SAM_TARGET_NAME}
        dma
        extint
        extint/${SAM_TARGET_NAME}
        port 
        rtc
        rtc/${SAM_TARGET_NAME}
        sercom
        sercom/i2c
        sercom/i2c/sam0
        sercom/usart
        services
        services/delay
        services/delay/sam0
        system
        system/clock
        system/clock/${SAM_TARGET_NAME}
        system/interrupt
        system/interrupt/${SAM_TARGET_NAME}
        system/pinmux
        system/power
        system/power/${SAM_TARGET_NAME}
        system/reset
        system/reset/${SAM_TARGET_NAME}
        tc
        tc/${SAM_TARGET_NAME}
        tcc
)

target_sources(samcortexm0p-drivers
    INTERFACE
        adc/${SAM_TARGET_NAME}/adc.c
        dac/${SAM_TARGET_NAME}/dac.c
        dma/dma.c
        extint/extint_callback.c
        extint/${SAM_TARGET_NAME}/extint.c
        port/port.c
        rtc/${SAM_TARGET_NAME}/rtc_count.c
        sercom/sercom_interrupt.c
        sercom/sercom.c
        sercom/i2c/i2c_sam0/i2c_master.c
        sercom/i2c/i2c_sam0/i2c_slave.c
        sercom/i2c/i2c_sam0/i2c_master_interrupt.c
        sercom/usart/usart.c
        sercom/usart/usart_interrupt.c
        services/delay/sam0/systick_counter.c
        system/system.c
        system/clock/${SAM_TARGET_NAME}/clock.c
        system/clock/${SAM_TARGET_NAME}/gclk.c
        system/interrupt/system_interrupt.c
        system/pinmux/pinmux.c
        tc/${SAM_TARGET_NAME}/tc.c
        tc/tc_interrupt.c
)

target_link_libraries(samcortexm0p-drivers
    INTERFACE 
        samcortexm0p-utils
)


