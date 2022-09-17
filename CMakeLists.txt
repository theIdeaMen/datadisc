#
# Copyright (c) 2020-2022 Griffin Adams
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.4)

set(ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/drivers/kx134 ${CMAKE_CURRENT_LIST_DIR}/drivers/bm1422)
#set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/pm_static_${BOARD}.yml)
set(CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/prj.conf)
list(APPEND CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/kx134.conf)
list(APPEND CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/bm1422.conf)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/usb.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(datadisc)

# Point to DataDisc root directory.
set(DATADISC_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "DataDisc root directory")

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Include UART ASYNC API adapter
target_sources_ifdef(CONFIG_BT_NUS_UART_ASYNC_ADAPTER app PRIVATE
  src/uart_async_adapter.c
)

zephyr_library_include_directories(include)