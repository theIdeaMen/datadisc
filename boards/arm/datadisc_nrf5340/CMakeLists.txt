# SPDX-License-Identifier: Apache-2.0


if (CONFIG_BOARD_DATADISC_NRF5340_CPUAPP AND CONFIG_BOARD_ENABLE_CPUNET)
  zephyr_library()
  zephyr_library_sources(datadisc_nrf5340_cpunet_reset.c)
endif()
