# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/TARGET_LPC11U6X)

add_library(mbed-legacy-nxp INTERFACE)

target_include_directories(mbed-legacy-nxp
    INTERFACE
        .
)

