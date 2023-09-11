# Copyright 2023 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BSC_RZA2	bsc_rza2.c)
