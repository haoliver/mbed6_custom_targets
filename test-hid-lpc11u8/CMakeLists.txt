# Copyright (c) 2020-2021 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")
set(APP_TARGET test-hid-lpc11u8)

include(${MBED_PATH}/tools/cmake/app.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../mbed-external-targets ${CMAKE_CURRENT_BINARY_DIR}/mbed-external-targets)
add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os)

add_executable(${APP_TARGET}
    main.cpp
)

project(${APP_TARGET})

target_link_libraries(${APP_TARGET} mbed-baremetal mbed-events mbed-usb)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()