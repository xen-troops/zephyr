/*
 * Copyright (c) 2024 EPAM Systemss
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_RZ_COMMON_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_RZ_COMMON_H_

struct intc_rz_line_cfg {
	unsigned int line;
	unsigned int parent_line;
};

struct intc_rz_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	struct intc_rz_line_cfg *line_map;
	unsigned int num_lines;
};

struct intc_rz_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct k_spinlock lock;
};

#define IRQ_CONFIGURE(n, inst, _isr)                                                               \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority), _isr, \
		    DT_INST_PROP_BY_IDX(inst, map, n),                                             \
		    COND_CODE_1(DT_INST_IRQ_HAS_CELL_AT_IDX(inst, n, flags),                       \
				(DT_INST_IRQ_BY_IDX(inst, n, flags)), (0)));

#define CONFIGURE_ALL_IRQS(inst, n, _isr) LISTIFY(n, IRQ_CONFIGURE, (), inst, _isr)

#define FILL_ONE_LINE_MAP(n, inst)                        \
	{                                                 \
		DT_INST_PROP_BY_IDX(inst, map, n),        \
		DT_INST_IRQ_BY_IDX(inst, n, irq),         \
	}
#define FILL_LINE_MAP_CONFIG(inst, n) \
	(struct intc_rz_line_cfg []){LISTIFY(n, FILL_ONE_LINE_MAP, (,), inst)}

unsigned int intc_rz_intr_get_parent_irq(const struct device *dev, unsigned int irq);
void intc_rz_intr_enable(const struct device *dev, unsigned int irq);
void intc_rz_intr_disable(const struct device *dev, unsigned int irq);
unsigned int intc_rz_intr_get_state(const struct device *dev);
int intc_rz_intr_get_line_state(const struct device *dev, unsigned int irq);

#endif /* ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_RZ_COMMON_H_ */
