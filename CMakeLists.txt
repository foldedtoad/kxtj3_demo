#
# Copyright (c) 2024  Callender Consulting LLC
#
# SPDX-License-Identifier: Apache-2.0
#
cmake_minimum_required(VERSION 3.20.)

set(BOARD xiao_ble)

set(DTS_ROOT_SYSTEM_INCLUDE_DIRS "dts/bindings/sensor/")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(kxtj3_demo)

add_subdirectory(drivers)

include_directories(app PRIVATE src)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})

zephyr_compile_options(-save-temps)
