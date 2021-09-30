#
#  Copyright (c) 2021 Griffin Adams
#
#  SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_BM1422 bm1422.c)
zephyr_library_sources_ifdef(CONFIG_BM1422_TRIGGER bm1422_trigger.c)
