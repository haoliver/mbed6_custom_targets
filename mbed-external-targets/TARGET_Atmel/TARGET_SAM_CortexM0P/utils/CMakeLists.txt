# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(samcortexm0p-utils INTERFACE)

target_compile_options(samcortexm0p-utils INTERFACE
    -Wno-expansion-to-defined
)

target_include_directories(samcortexm0p-utils
    INTERFACE
        .

        ../../common/utils
        ../../common/utils/interrupt

        preprocessor
        header_files

        cmsis/${SAM_TARGET_NAME}/include
        cmsis/${SAM_TARGET_NAME}/source
        cmsis/${SAM_TARGET_NAME}/include/component
        cmsis/${SAM_TARGET_NAME}/include/instance
        cmsis/${SAM_TARGET_NAME}/include/pio

)

target_sources(samcortexm0p-utils
    INTERFACE
        cmsis/${SAM_TARGET_NAME}/source/system_samd21.c
        ../../common/utils/interrupt/interrupt_sam_nvic.c
)

target_link_libraries(samcortexm0p-utils INTERFACE mbed-cmsis-cortex-m)

