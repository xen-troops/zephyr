/*
 * Copyright (c) 2026 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/xen/generic.h>

#include <zephyr/arch/common/init.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(xen_fdt);

#define FDT_MAGIC 0xd00dfeed

/* x0 from bootloader stored on boot */
__noinit uintptr_t xen_fdt_addr;

__noinit uint32_t xen_fdt_size;

/* buffer for device tree blob */
uint8_t xen_fdt[CONFIG_XEN_FDT_MAX_SIZE] __aligned(XEN_PAGE_SIZE);

void xen_copy_fdt(void)
{
	const uint8_t *src = (const uint8_t *)xen_fdt_addr;
	uint32_t magic = sys_get_be32(src);
	uint32_t size = sys_be32_to_cpu(xen_fdt_size);

	if (magic != FDT_MAGIC) {
		LOG_ERR("Invalid Xen device tree magic: 0x%x", magic);
		k_panic();
	}

	if (size > CONFIG_XEN_FDT_MAX_SIZE) {
		LOG_ERR("Xen device tree size 0x%x exceeds max 0x%x",
			size, CONFIG_XEN_FDT_MAX_SIZE);
		k_panic();
	}

	arch_early_memset(xen_fdt, 0, CONFIG_XEN_FDT_MAX_SIZE);
	arch_early_memcpy(xen_fdt, src, size);
}

static int xen_print_fdt(void)
{
	LOG_DBG("Saved Xen device tree address is 0x%lx, size = 0x%x",
		xen_fdt_addr, sys_be32_to_cpu(xen_fdt_size));
	return 0;
}

SYS_INIT(xen_print_fdt, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
