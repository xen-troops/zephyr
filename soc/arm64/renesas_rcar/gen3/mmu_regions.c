/*
 * Copyright 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/arch/arm64/arm_mmu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 0),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 0),
			      MT_DEVICE_nGnRnE | MT_RW | MT_NS),

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 1),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 1),
			      MT_DEVICE_nGnRnE | MT_RW | MT_NS),

	MMU_REGION_FLAT_ENTRY("UART",
			      DT_REG_ADDR(DT_INST(0, renesas_rcar_scif)),
			      ROUND_UP(DT_REG_SIZE(DT_INST(0, renesas_rcar_scif)), 4096),
			      MT_DEVICE_nGnRnE | MT_RW | MT_NS),

	MMU_REGION_FLAT_ENTRY("CPG",
			      DT_REG_ADDR(DT_INST(0, renesas_r8a7795_cpg_mssr)),
			      DT_REG_SIZE(DT_INST(0, renesas_r8a7795_cpg_mssr)),
			      MT_DEVICE_nGnRnE | MT_RW | MT_NS),

	MMU_REGION_FLAT_ENTRY("PFC",
			      DT_REG_ADDR(DT_INST(0, renesas_rcar_pfc)), 4096,
			      MT_DEVICE_nGnRnE | MT_RW | MT_NS),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
