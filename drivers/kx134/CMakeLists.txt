#
#  Copyright (c) 2021 Griffin Adams
#
#  SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_KX134 kx134.c)
zephyr_library_sources_ifdef(CONFIG_KX134_TRIGGER kx134_trigger.c)
