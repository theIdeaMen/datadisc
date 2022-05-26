#
#  Copyright (c) 2021 Griffin Adams
#
#  SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_CH201 ch201.c)
zephyr_library_sources_ifdef(CONFIG_CH201_TRIGGER ch201_trigger.c)
