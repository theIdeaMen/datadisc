#
#  Copyright (c) 2021-2022 Griffin Adams
#
#  SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.4)

if(CONFIG_CH201)
include_directories(.)
target_sources(app PRIVATE bm1422.c bm1422_trigger.c)
endif()