# Copyright (c) 2022 Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(TARGET_XIAO_SAMD21 EXCLUDE_FROM_ALL)

if(${MBED_TOOLCHAIN} STREQUAL "GCC_ARM")
    set(LINKER_FILE  device/TOOLCHAIN_GCC_ARM/samd21g18a.ld)
    set(STARTUP_FILE device/TOOLCHAIN_GCC_ARM/startup_samd21.c)
endif()

add_library(mbed-mcu-samd21g18a INTERFACE)
mbed_set_linker_script(mbed-mcu-samd21g18a ${CMAKE_CURRENT_SOURCE_DIR}/${LINKER_FILE})

target_include_directories(mbed-mcu-samd21g18a
    INTERFACE
        .
)

target_sources(mbed-mcu-samd21g18a
    INTERFACE
        analogout_api.c
        PeripheralPins.c

        ${STARTUP_FILE}
)

target_link_libraries(mbed-mcu-samd21g18a
    INTERFACE 
        mbed-samcortexm0p
)


