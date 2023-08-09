/*
 * Copyright 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/aarch32/mmu/arm_mmu.h>
#include <zephyr/devicetree.h>

static const struct arm_mmu_region mmu_regions[] = {
	/* ARM Arch timer, GIC are covered by the MPCore mapping */
	MMU_REGION_FLAT_ENTRY("vectors",
			      0x0,
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
