/*
 * Copyright (c) 2024 EPAM Systemss
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r9a08g045_intc

/**
 * @file
 * @brief r9a08g045 interrupt controller driver
 *
 * This driver provides support for level 2 interrupts on the r9a08g045
 * SoC. The r9a08g045 interrupt controller is a front-end for the NVIC found
 * on Renesas RZ/G3S SoCs: IRQ sense select for 8 external pin interrupts,
 * 1:1-mapped to 8 NVIC interrupts and NMI edge select.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include <zephyr/dt-bindings/interrupt-controller/r7s9210-intc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(r9a08g045_intc, CONFIG_INTC_LOG_LEVEL);

#include "intc_rz_common.h"

#define R9A08G045_NMI_NSCR  0x00
#define R9A08G045_NMI_NITSR 0x04

#define R9A08G045_INTC_ISCR  0x10
#define R9A08G045_INTC_IITSR 0x14

/* NSCR NMI Status Control Register */
#define R9A08G045_NMI_NSCR_NSTAT BIT(0)
#define R9A08G045_NMI_NSCR_NSMON BIT(16)

/* NITSR NMI Interrupt Type Selection Register
 * 0: Falling-edge detection.
 * 1: Rising-edge detection.
 */
#define R9A08G045_NMI_NITSR_NTSEL BIT(0)

/* ISCR IRQ Status Control Register */
#define R9A08G045_INTC_ISCR_ISTAT(n) BIT(n)

/* IITSR IRQ Interrupt Type Selection Register */
#define R9A08G045_INTC_IITSR_IITSEL(n, t)        (((t) & 0x3) << ((n) << 1))
#define R9A08G045_INTC_IITSR_IITSEL_LOW_LEVEL    0x0
#define R9A08G045_INTC_IITSR_IITSEL_EDGE_FALLING 0x1
#define R9A08G045_INTC_IITSR_IITSEL_EDGE_RISING  0x2
#define R9A08G045_INTC_IITSR_IITSEL_EDGE_BOTH    0x3

struct r9a08g045_intc_data {
	struct intc_rz_data rz_data;
	uint8_t f_irq_lvl;
};

static void r9a08g045_intr_eoi(const struct device *dev, uint32_t line)
{
	struct r9a08g045_intc_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t reg_val;

	if (data->f_irq_lvl & BIT(line)) {
		return;
	}

	/* clear non-level irq status only - invalid to clear for lvl */
	key = k_spin_lock(&data->rz_data.lock);
	reg_val = sys_read32(DEVICE_MMIO_GET(dev) + R9A08G045_INTC_ISCR);
	if (reg_val & BIT(line)) {
		reg_val &= ~R9A08G045_INTC_ISCR_ISTAT(line);
		sys_write32(reg_val, DEVICE_MMIO_GET(dev) + R9A08G045_INTC_ISCR);
		reg_val = sys_read32(DEVICE_MMIO_GET(dev) + R9A08G045_INTC_ISCR);
	}
	k_spin_unlock(&data->rz_data.lock, key);
}

static void r9a08g045_isr(const void *arg)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	const uint32_t line = POINTER_TO_UINT(arg);
	const struct _isr_table_entry *entry;
	uint32_t reg_val;

	reg_val = sys_read32(DEVICE_MMIO_GET(dev) + R9A08G045_INTC_ISCR);
	if (!(reg_val & BIT(line))) {
		LOG_DBG("intc: spurious irq %u", line);
		return;
	}


	entry = &_sw_isr_table[CONFIG_2ND_LVL_ISR_TBL_OFFSET + line];
	entry->isr(entry->arg);

	r9a08g045_intr_eoi(dev, line);
}

static void r9a08g045_intr_set_priority(const struct device *dev, unsigned int irq,
					unsigned int prio, uint32_t flags)
{
	struct r9a08g045_intc_data *data = (struct r9a08g045_intc_data *)dev->data;
	const struct intc_rz_cfg *cfg = (const struct intc_rz_cfg *)dev->config;
	unsigned int parent_irq = intc_rz_intr_get_parent_irq(dev, irq);
	k_spinlock_key_t key;
	uint32_t reg_val;

	if (parent_irq >= cfg->num_lines) {
		return;
	}

	irq = irq_from_level_2(irq);
	data->f_irq_lvl &= ~BIT(irq);

	switch (flags) {
	case IRQ_TYPE_LOW_LEVEL:
		flags = R9A08G045_INTC_IITSR_IITSEL_LOW_LEVEL;
		data->f_irq_lvl |= BIT(irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		flags = R9A08G045_INTC_IITSR_IITSEL_EDGE_FALLING;
		break;
	case IRQ_TYPE_EDGE_RISING:
		flags = R9A08G045_INTC_IITSR_IITSEL_EDGE_RISING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		flags = R9A08G045_INTC_IITSR_IITSEL_EDGE_BOTH;
		break;
	default:
		break;
	}

	key = k_spin_lock(&data->rz_data.lock);
	reg_val = sys_read32(DEVICE_MMIO_GET(dev) + R9A08G045_INTC_IITSR);
	reg_val &= ~R9A08G045_INTC_IITSR_IITSEL(irq, 0x3);
	reg_val |= R9A08G045_INTC_IITSR_IITSEL(irq, flags);
	sys_write32(reg_val, DEVICE_MMIO_GET(dev) + R9A08G045_INTC_IITSR);
	k_spin_unlock(&data->rz_data.lock, key);

	r9a08g045_intr_eoi(dev, irq);
}

static const struct irq_next_level_api r9a08g045_intc_next_lvl = {
	.intr_enable = intc_rz_intr_enable,
	.intr_disable = intc_rz_intr_disable,
	.intr_get_state = intc_rz_intr_get_state,
	.intr_set_priority = r9a08g045_intr_set_priority,
	.intr_get_line_state = intc_rz_intr_get_line_state,
};

#define IRQ_CONFIGURE(n, inst, _isr)                                                               \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority), _isr, \
		    DT_INST_PROP_BY_IDX(inst, map, n), 0);

#define CONFIGURE_ALL_IRQS(inst, n, _isr) LISTIFY(n, IRQ_CONFIGURE, (), inst, _isr)

static int r9a08g045_intc_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	CONFIGURE_ALL_IRQS(0, DT_NUM_IRQS(DT_DRV_INST(0)), r9a08g045_isr);

	return 0;
}

BUILD_ASSERT(DT_NUM_IRQS(DT_DRV_INST(0)) == DT_INST_PROP_LEN(0, map),
	     "Number of items in interrupts and map should be the same");

static const struct intc_rz_cfg drv_cfg = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),

	.num_lines = DT_NUM_IRQS(DT_DRV_INST(0)),
	.line_map = FILL_LINE_MAP_CONFIG(0, DT_NUM_IRQS(DT_DRV_INST(0))),
};

static struct r9a08g045_intc_data drv_data;

DEVICE_DT_INST_DEFINE(0, &r9a08g045_intc_init, NULL,
		      &drv_data, &drv_cfg, PRE_KERNEL_1,
		      CONFIG_INTC_R9A08G045_INIT_PRIORITY, &r9a08g045_intc_next_lvl);
