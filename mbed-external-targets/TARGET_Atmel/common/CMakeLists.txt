# Copyright (c) 2022 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

include(mbed_set_linker_script)

add_subdirectory(TARGET_SAM_CortexM0P EXCLUDE_FROM_ALL)

add_library(mbed-atmel INTERFACE)

target_include_directories(mbed-atmel
    INTERFACE
        .
        common/utils
        common/utils/interrupt
)

target_sources(mbed-atmel
    INTERFACE
        common/utils/interrupt/interrupt_sam_nvic.c
)

target_link_libraries(mbed-atmel INTERFACE mbed-cmsis-cortex-m)
