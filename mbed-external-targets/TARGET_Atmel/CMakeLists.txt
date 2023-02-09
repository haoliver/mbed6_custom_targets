# Copyright (c) 2022 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

include(mbed_set_linker_script)

set(SAM_TARGET_NAME TARGET_SAMD21)

add_subdirectory(TARGET_SAM_CortexM0P EXCLUDE_FROM_ALL)

add_library(mbed-atmel INTERFACE)

target_include_directories(mbed-atmel
    INTERFACE
        .
)

