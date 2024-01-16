/*
 * Copyright (c) 2023 EPAM Systemss
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_intc

/**
 * @file
 * @brief r7s9210 interrupt controller driver
 *
 * This driver provides support for level 2 interrupts on the r7s9210
 * SoC. The r7s9210 interrupt controller is a front-end for the GIC found
 * on Renesas RZ/A2 SoCs: IRQ sense select for 8 external interrupts,
 * 1:1-mapped to 8 GIC SPI interrupts and NMI edge select.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq_nextlevel.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/dt-bindings/interrupt-controller/r7s9210-intc.h>

#include "intc_rz_common.h"

/**
 * CR0 is a 16-bit register that sets the input signal detection mode for the
 * external interrupt input pin NMI, and indicates the input level at the NMI pin.
 */
#define R7S9210_INTC_CR0 0x0
#define R7S9210_INTC_CR0_NMIF        BIT(1)   /* NMI Input Level */
#define R7S9210_INTC_CR0_NMIE        BIT(8)   /* NMI Edge Select */
#define R7S9210_INTC_CR0_NMIE_OFFSET 8
#define R7S9210_INTC_CR0_NMIL        BIT(15)  /* NMI Interrupt Request */

/**
 * CR1 is a 16-bit register that specifies the detection mode for external interrupt
 * input pins IRQ7 to IRQ0 individually: low level, falling edge, rising edge, or
 * both edges
 */
#define R7S9210_INTC_CR1 0x2

/**
 * Possible values for type should be taken from r7s9210-intc.h
 */
#define R7S9210_SET_DETECTION_MODE(reg, irq, type) \
	reg = (reg & ~(0x3U << (irq * 2U))) | (type << (irq * 2U))

/**
 * RR is a 16-bit register that indicates interrupt requests from external input
 * pins IRQ7 to IRQ0
 */
#define R7S9210_INTC_RR 0x4

/**
 * MSK is a bit to set release, when the IRQ signal is used as a software standby
 * cancel source signal. 0 - masked, 1 - unmasked. When MSK is 1 (unmasked), the
 * IRQ signal can be used as a software standby cancel source.
 */
#define R7S9210_INTC_RR_MSK BIT(15)

static void r7s9210_isr(const void *arg)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct intc_rz_data *data = (struct intc_rz_data *)dev->data;
	k_spinlock_key_t key;
	const uint32_t line = POINTER_TO_UINT(arg);
	const struct _isr_table_entry *entry = &_sw_isr_table[CONFIG_2ND_LVL_ISR_TBL_OFFSET + line];
	uint16_t rr;

	entry->isr(entry->arg);

	key = k_spin_lock(&data->lock);
	/*
	 * For non-level detection, we should execute a clear sequence for the IRQ line
	 * but let's save time in the ISR and perform the clear sequence for low-level mode too.
	 */
	rr = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_RR);
	rr &= ~BIT(line);
	sys_write16(rr, DEVICE_MMIO_GET(dev) + R7S9210_INTC_RR);
	k_spin_unlock(&data->lock, key);
}

static void r7s9210_intr_set_priority(const struct device *dev, unsigned int irq,
				      unsigned int prio, uint32_t flags)
{
	uint16_t cr1;
	struct intc_rz_data *data = (struct intc_rz_data *)dev->data;
	k_spinlock_key_t key;
	unsigned int parent_irq = intc_rz_intr_get_parent_irq(dev, irq);

	if (!parent_irq) {
		return;
	}

	irq = irq_from_level_2(irq);

	switch (flags) {
	case IRQ_TYPE_LOW_LEVEL:
		flags = 0;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		flags = 1;
		break;
	case IRQ_TYPE_EDGE_RISING:
		flags = 2;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		flags = 3;
		break;
	default:
		break;
	}

	key = k_spin_lock(&data->lock);
	cr1 = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR1);
	R7S9210_SET_DETECTION_MODE(cr1, irq, flags);
	sys_write16(cr1, DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR1);
	k_spin_unlock(&data->lock, key);
}

static const struct irq_next_level_api r7s9210_intc_next_lvl = {
	.intr_enable = intc_rz_intr_enable,
	.intr_disable = intc_rz_intr_disable,
	.intr_get_state = intc_rz_intr_get_state,
	.intr_set_priority = r7s9210_intr_set_priority,
	.intr_get_line_state = intc_rz_intr_get_line_state,
};

BUILD_ASSERT(DT_NUM_IRQS(DT_DRV_INST(0)) == DT_INST_PROP_LEN(0, map),
	     "Number of items in interrupts and map should be the same");

static inline void r7s9210_intc_setup_nmi_edge(const struct device *dev)
{
	k_spinlock_key_t key;
	uint16_t cr0;
	struct intc_rz_data *data = (struct intc_rz_data *)dev->data;
	uint16_t nmi_edge = DT_INST_PROP_OR(0, nmi_edge, IRQ_TYPE_EDGE_FALLING) >> 2;

	nmi_edge <<= R7S9210_INTC_CR0_NMIE_OFFSET;

	key = k_spin_lock(&data->lock);
	cr0 = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
	cr0 &= R7S9210_INTC_CR0_NMIE;
	cr0 |= nmi_edge;
	sys_write16(cr0, DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
	k_spin_unlock(&data->lock, key);
}

/*
 * note: This function only makes sense when 'z_arm_nmi_set_handler' is called since the default
 *       NMI handler never exits. Moreover, NMI handled incorrectly for Cortex A/R platforms:
 *         * LR isn't saved before entering NMI handler (look at 'z_arm_nmi'), so dabort happened;
 *         * interrupt nesting counter is never incremented for NMI, but decremented on exit.
 */
void z_arm_nmi_eoi(void)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	uint16_t cr0;

	/* cancel current NMI, this function is called after nmi handler */
	cr0 = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
	cr0 &= R7S9210_INTC_CR0_NMIF;
	sys_write16(cr0, DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
}

static int r7s9210_intc_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	CONFIGURE_ALL_IRQS(0, DT_NUM_IRQS(DT_DRV_INST(0)), r7s9210_isr);

	r7s9210_intc_setup_nmi_edge(dev);
	return 0;
}

static const struct intc_rz_cfg drv_cfg = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),

	.num_lines = DT_NUM_IRQS(DT_DRV_INST(0)),
	.line_map = FILL_LINE_MAP_CONFIG(0, DT_NUM_IRQS(DT_DRV_INST(0))),
};

static struct intc_rz_data drv_data;

DEVICE_DT_INST_DEFINE(0, &r7s9210_intc_init, NULL,
		      &drv_data, &drv_cfg, PRE_KERNEL_1,
		      CONFIG_INTC_R7S9210_INIT_PRIORITY, &r7s9210_intc_next_lvl);
