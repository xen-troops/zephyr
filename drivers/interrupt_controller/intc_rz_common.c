/*
 * Copyright (c) 2024 EPAM Systemss
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>

#include "intc_rz_common.h"

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_r7s9210_intc)
#include <zephyr/drivers/interrupt_controller/gic.h>

#define INTC_RZ_INT_BASE	(GIC_SPI_INT_BASE)
#else
#define INTC_RZ_INT_BASE	(0)
#endif

/* is this irq belongs to this intc if yes - return parent irq */
unsigned int intc_rz_intr_get_parent_irq(const struct device *dev, unsigned int irq)
{
	const struct intc_rz_cfg *cfg = (const struct intc_rz_cfg *)dev->config;
	unsigned int parent_line = irq_parent_level_2(irq) + INTC_RZ_INT_BASE;
	unsigned int line;

	irq = irq_from_level_2(irq);
	parent_line += irq;

	for (line = 0; line < cfg->num_lines; line++) {
		if (cfg->line_map[line].line != irq) {
			continue;
		}

		if (cfg->line_map[line].parent_line == parent_line) {
			return parent_line;
		}
	}

	return line;
}

/* The driver can't enable IRQ by this driver, so lets request it from the parent driver. */
void intc_rz_intr_enable(const struct device *dev, unsigned int irq)
{
	const struct intc_rz_cfg *cfg = (const struct intc_rz_cfg *)dev->config;
	unsigned int parent_irq = intc_rz_intr_get_parent_irq(dev, irq);

	if (parent_irq >= cfg->num_lines) {
		return;
	}

	irq_enable(parent_irq);
}

/* The driver can't disable IRQ by this driver, so lets request it from the parent driver. */
void intc_rz_intr_disable(const struct device *dev, unsigned int irq)
{
	const struct intc_rz_cfg *cfg = (const struct intc_rz_cfg *)dev->config;
	unsigned int parent_irq = intc_rz_intr_get_parent_irq(dev, irq);

	if (parent_irq >= cfg->num_lines) {
		return;
	}

	irq_disable(parent_irq);
}

/**
 * The driver can't determine the status of whether the IRQs on lines are enabled or not.
 * So, let's request information from the parent driver.
 */
unsigned int intc_rz_intr_get_state(const struct device *dev)
{
	const struct intc_rz_cfg *cfg = (const struct intc_rz_cfg *)dev->config;

	for (unsigned int line = 0; line < cfg->num_lines; line++) {
		if (irq_is_enabled(cfg->line_map[line].parent_line)) {
			return 1;
		}
	}

	return 0;
}

/**
 * The driver can't determine the status of whether the IRQ on this line is enabled or not.
 * Therefore, let's request this information from the parent driver.
 */
int intc_rz_intr_get_line_state(const struct device *dev, unsigned int irq)
{
	const struct intc_rz_cfg *cfg = (const struct intc_rz_cfg *)dev->config;
	unsigned int parent_irq = intc_rz_intr_get_parent_irq(dev, irq);

	if (parent_irq >= cfg->num_lines) {
		return 0;
	}

	return irq_is_enabled(parent_irq);
}
