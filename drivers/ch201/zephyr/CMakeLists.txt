#
#  Copyright (c) 2021-2022 Griffin Adams
#
#  SPDX-License-Identifier: Apache-2.0
#

if(CONFIG_CH201)
    # Add driver to the set of global include paths.
    zephyr_include_directories(.)

    zephyr_library()
    zephyr_library_sources(
        ch201.c
        ch201_trigger.c
        )
endif()