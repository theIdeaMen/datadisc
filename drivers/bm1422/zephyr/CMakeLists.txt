#
#  Copyright (c) 2021-2022 Griffin Adams
#
#  SPDX-License-Identifier: Apache-2.0
#

if(CONFIG_BM1422)
    # Add driver to the set of global include paths.
    zephyr_include_directories(.)

    zephyr_library()
    zephyr_library_sources(
        bm1422.c
        bm1422_trigger.c
        )
endif()