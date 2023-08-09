# Copyright (c) 2023 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(soc.c)
zephyr_library_sources(mmu_regions.c)
zephyr_library_sources_ifdef(CONFIG_SOC_R7S9210 pfc_r7s9210.c)
