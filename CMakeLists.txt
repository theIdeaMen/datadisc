# SPDX-License-Identifier: Apache-2.0
set(ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/drivers/kx134 ${CMAKE_CURRENT_LIST_DIR}/drivers/bm1422)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
list(APPEND CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/prj.conf
						${CMAKE_CURRENT_LIST_DIR}/kx134.conf
						${CMAKE_CURRENT_LIST_DIR}/bm1422.conf)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/drivers)
list(APPEND DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/usb.overlay)
						#${CMAKE_CURRENT_LIST_DIR}/qspi.overlay)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(datadisc)

# Point to DataDisc root directory.
set(DATADISC_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "DataDisc root directory")

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
