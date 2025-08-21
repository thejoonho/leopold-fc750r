# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(leopold_fc750r)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/fuel_gauge.c)

target_sources(app PRIVATE ${app_sources})
include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
FILE(GLOB app_sources ${ZEPHYR_BASE}/samples/subsys/usb/hid-keyboard/src/*.c)
FILE(GLOB app_sources src/*.c)