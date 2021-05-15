# ST Microelectronics LIS2DTW12 3-axis accelerometer driver
#
# Copyright (c) 2019 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LIS2DTW12            lis2dtw12)
zephyr_library_sources_ifdef(CONFIG_LIS2DTW12            lis2dtw12_i2c.c)
zephyr_library_sources_ifdef(CONFIG_LIS2DTW12            lis2dtw12_spi.c)
zephyr_library_sources_ifdef(CONFIG_LIS2DTW12_TRIGGER    lis2dtw12_trigger.c)
