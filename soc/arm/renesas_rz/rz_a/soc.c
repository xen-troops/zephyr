/*
 * Copyright (c) 2023 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <cmsis_core.h>
#include <zephyr/arch/arm/cortex_a_r/lib_helpers.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include "soc.h"

#ifdef CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER
void z_soc_irq_init(void)
{}

void z_soc_irq_enable(unsigned int irq)
{
	const struct device *lvl2_intc;

	switch (irq_get_level(irq)) {
	case 1:
		arm_gic_irq_enable(irq);
		break;
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r7s9210_intc));
		irq_enable_next_level(lvl2_intc, irq);
		break;
	default:
		break;
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	const struct device *lvl2_intc;

	switch (irq_get_level(irq)) {
	case 1:
		arm_gic_irq_disable(irq);
		break;
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r7s9210_intc));
		irq_disable_next_level(lvl2_intc, irq);
		break;
	default:
		break;
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	const struct device *lvl2_intc;

	switch (irq_get_level(irq)) {
	case 1:
		return arm_gic_irq_is_enabled(irq);
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r7s9210_intc));
		return irq_line_is_enabled_next_level(lvl2_intc, irq);
	default:
		break;
	}

	return 0;
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
	const struct device *lvl2_intc;

	switch (irq_get_level(irq)) {
	case 1:
		arm_gic_irq_set_priority(irq, prio, flags);
		break;
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r7s9210_intc));
		irq_set_priority_next_level(lvl2_intc, irq, prio, flags);
		break;
	default:
		break;
	}
}

unsigned int z_soc_irq_get_active(void)
{
	/* irqs from r7s9210 intc mapped 1:1 on arm gic */
	return arm_gic_get_active();
}

/* for now, eoi for lvl2 is performed in the end of the r7s9210 isr */
void z_soc_irq_eoi(unsigned int irq)
{
	switch (irq_get_level(irq)) {
	case 1:
		arm_gic_eoi(irq);
		break;
	default:
		break;
	}
}
#endif

static int soc_rz_init(void)
{
	return 0;
}
SYS_INIT(soc_rz_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

extern char _vector_start[];

void relocate_vector_table(void)
{
#if defined(CONFIG_XIP) && (CONFIG_FLASH_BASE_ADDRESS != 0) || \
	!defined(CONFIG_XIP) && (CONFIG_SRAM_BASE_ADDRESS != 0)
	write_sctlr(read_sctlr() & ~HIVECS);
#elif defined(CONFIG_SW_VECTOR_RELAY) || defined(CONFIG_SW_VECTOR_RELAY_CLIENT)
	_vector_table_pointer = _vector_start;
#endif
	__set_VBAR(POINTER_TO_UINT(_vector_start));
}

/* Platform-specific early initialization */
void z_arm_platform_init(void) {}
