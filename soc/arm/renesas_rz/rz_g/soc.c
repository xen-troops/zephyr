/*
 * Copyright (c) 2024 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <cmsis_core.h>
#include <zephyr/arch/arm/cortex_m/nvic.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include "soc.h"

#ifdef CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER

#define NUM_IRQS_PER_REG 32
#define REG_FROM_IRQ(irq) ((irq) / NUM_IRQS_PER_REG)
#define BIT_FROM_IRQ(irq) ((irq) % NUM_IRQS_PER_REG)

void z_soc_irq_enable(unsigned int irq)
{
	const struct device *lvl2_intc;

	switch (irq_get_level(irq)) {
	case 1:
		NVIC_EnableIRQ((IRQn_Type)irq);
		break;
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r9a08g045_intc));
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
		NVIC_DisableIRQ((IRQn_Type)irq);
		break;
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r9a08g045_intc));
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
		return NVIC->ISER[REG_FROM_IRQ(irq)] & BIT(BIT_FROM_IRQ(irq));
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r9a08g045_intc));
		return irq_line_is_enabled_next_level(lvl2_intc, irq);
	default:
		break;
	}

	return 0;
}

static void z_arm_nvic_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	/* The kernel may reserve some of the highest priority levels.
	 * So we offset the requested priority level with the number
	 * of priority levels reserved by the kernel.
	 */

	/* If we have zero latency interrupts, those interrupts will
	 * run at a priority level which is not masked by irq_lock().
	 * Our policy is to express priority levels with special properties
	 * via flags
	 */
	if (IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) && (flags & IRQ_ZERO_LATENCY)) {
		if (ZERO_LATENCY_LEVELS == 1) {
			prio = _EXC_ZERO_LATENCY_IRQS_PRIO;
		} else {
			/* Use caller supplied prio level as-is */
		}
	} else {
		prio += _IRQ_PRIO_OFFSET;
	}

	/* The last priority level is also used by PendSV exception, but
	 * allow other interrupts to use the same level, even if it ends up
	 * affecting performance (can still be useful on systems with a
	 * reduced set of priorities, like Cortex-M0/M0+).
	 */
	__ASSERT(prio <= (BIT(NUM_IRQ_PRIO_BITS) - 1),
		 "invalid priority %d for %d irq! values must be less than %lu\n",
		 prio - _IRQ_PRIO_OFFSET, irq,
		 BIT(NUM_IRQ_PRIO_BITS) - (_IRQ_PRIO_OFFSET));
	NVIC_SetPriority((IRQn_Type)irq, prio);
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
	const struct device *lvl2_intc;

	switch (irq_get_level(irq)) {
	case 1:
		z_arm_nvic_irq_priority_set(irq, prio, flags);
		break;
	case 2:
		lvl2_intc = DEVICE_DT_GET(DT_INST(0, renesas_r9a08g045_intc));
		irq_set_priority_next_level(lvl2_intc, irq, prio, flags);
		break;
	default:
		break;
	}
}

#endif /* CONFIG_ARM_CUSTOM_INTERRUPT_CONTROLLER */

static int soc_rz_init(void)
{
	return 0;
}

SYS_INIT(soc_rz_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/* Platform-specific early initialization */
void z_arm_platform_init(void) {}
