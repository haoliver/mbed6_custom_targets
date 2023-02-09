# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(${MBED_TOOLCHAIN} STREQUAL "GCC_ARM")
    set(LINKER_FILE device/TOOLCHAIN_GCC_ARM/TARGET_LPC11U68/LPC11U68.ld)
    set(STARTUP_FILE device/TOOLCHAIN_GCC_ARM/TARGET_LPC11U68/startup_LPC11U68.S)
endif()

add_library(mbed-lpc11u6x INTERFACE)

target_include_directories(mbed-lpc11u6x
    INTERFACE
        .
        device
)

target_sources(mbed-lpc11u6x
    INTERFACE
        analogin_api.c
        gpio_api.c
        gpio_irq_api.c
        i2c_api.c
        pinmap.c
        pwmout_api.c
        rtc_api.c
        serial_api.c
        sleep.c
        spi_api.c
        us_ticker.c
        device/system_LPC11U6x.c
        USBPhyHw.cpp
        ${STARTUP_FILE}
)

target_link_libraries(mbed-lpc11u6x INTERFACE mbed-legacy-nxp mbed-cmsis-cortex-m)

add_library(mbed-lpc11u68 INTERFACE)

target_link_libraries(mbed-lpc11u68 INTERFACE mbed-lpc11u6x)

mbed_set_linker_script(mbed-lpc11u68 ${CMAKE_CURRENT_SOURCE_DIR}/${LINKER_FILE})
