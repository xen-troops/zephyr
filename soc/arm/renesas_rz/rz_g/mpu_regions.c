/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/slist.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/arch/arm/mpu/arm_mpu.h>

#include <zephyr/arch/arm/cortex_m/arm_mpu_mem_cfg.h>

static const struct arm_mpu_region mpu_regions[] = {
#if defined(CONFIG_NULL_POINTER_EXCEPTION_DETECTION_MPU_RZ_G)
	MPU_REGION_ENTRY("NULL", 0x0,
			 REGION_FLASH_ATTR(0x0, CONFIG_CORTEX_M_NULL_POINTER_EXCEPTION_PAGE_SIZE)),
#endif /* CONFIG_NULL_POINTER_EXCEPTION_DETECTION_MPU_RZ_G */
	MPU_REGION_ENTRY("vector", (uintptr_t)_vector_start,
			 REGION_RAM_TEXT_ATTR((uintptr_t)_vector_end)),

	MPU_REGION_ENTRY("SRAM_TEXT", (uintptr_t)__text_region_start,
			 REGION_RAM_TEXT_ATTR((uintptr_t)__rodata_region_start)),

	MPU_REGION_ENTRY("SRAM_RODATA", (uintptr_t)__rodata_region_start,
			 REGION_RAM_RO_ATTR((uintptr_t)__rodata_region_end)),

	MPU_REGION_ENTRY("SRAM_DATA", (uintptr_t)__rom_region_end,
			 REGION_RAM_ATTR_NO_EXEC((uintptr_t)__kernel_ram_end)),

#if defined(CONFIG_MPU_DISABLE_BACKGROUND_MAP)
#define DEVICE_REGION_START 0x40000000UL
#define DEVICE_REGION_END   0x5FFFFFFFUL
	MPU_REGION_ENTRY("DEVICE", DEVICE_REGION_START, REGION_DEVICE_ATTR(DEVICE_REGION_END)),
#endif /* CONFIG_MPU_DISABLE_BACKGROUND_MAP */
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
