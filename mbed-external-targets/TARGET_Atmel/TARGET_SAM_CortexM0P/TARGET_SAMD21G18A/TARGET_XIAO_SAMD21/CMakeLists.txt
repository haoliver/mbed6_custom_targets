# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-xiao-samd21 INTERFACE)

target_include_directories(mbed-xiao-samd21
    INTERFACE
        .
)

target_link_libraries(mbed-xiao-samd21 INTERFACE mbed-mcu-samd21g18a)

target_sources(mbed-xiao-samd21
    INTERFACE
        mbed_overrides.c
)
