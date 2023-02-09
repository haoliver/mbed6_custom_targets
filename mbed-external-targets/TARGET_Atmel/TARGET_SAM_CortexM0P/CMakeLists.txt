# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(utils EXCLUDE_FROM_ALL)
add_subdirectory(drivers EXCLUDE_FROM_ALL)
add_subdirectory(TARGET_SAMD21G18A EXCLUDE_FROM_ALL)

add_library(mbed-samcortexm0p INTERFACE)

target_include_directories(mbed-samcortexm0p
    INTERFACE
        .
        ${SAM_TARGET_NAME}/device
)

target_sources(mbed-samcortexm0p
    INTERFACE
        analogin_api.c
        dma_api.c
        gpio_api.c
        gpio_irq_api.c
        i2c_api.c
        pinmap.c
        pinmap_function.c
        port_api.c
        pwmout_api.c
        rtc_api.c
        sercom_dma.c
        serial_api.c
        sleep_api.c
        spi_api.c
        us_ticker.c
        USBPhy_SAMD21.cpp
)

target_link_libraries(mbed-samcortexm0p
    INTERFACE
        mbed-atmel 
        samcortexm0p-utils
        samcortexm0p-drivers
)


