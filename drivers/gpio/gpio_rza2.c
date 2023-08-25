/*
 * Copyright 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_gpio

#include <soc.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#include "gpio_rza2.h"

#define TINT0  480
#define TINT1  481
#define TINT2  482
#define TINT3  483
#define TINT4  484
#define TINT5  485
#define TINT6  486
#define TINT7  487
#define TINT8  488
#define TINT9  489
#define TINT10 490
#define TINT11 491
#define TINT12 492
#define TINT13 493
#define TINT14 494
#define TINT15 495
#define TINT16 496
#define TINT17 497
#define TINT18 498
#define TINT19 499
#define TINT20 500
#define TINT21 501
#define TINT22 502
#define TINT23 503
#define TINT24 504
#define TINT25 505
#define TINT26 506
#define TINT27 507
#define TINT28 508
#define TINT29 509
#define TINT30 510
#define TINT31 511

#define RZA2_PODR(port) (0x0000 + (port))     /* Output Data 8-bit */
#define RZA2_PIDR(port) (0x0020 + (port))     /* Input Data 8-bit */
#define RZA2_DSCR(port) (0x0000 + (port) * 2) /* DSCR 16-bit */

static struct rza2_gpio_data {
	mm_reg_t io_addr;
	mm_reg_t dscr_addr;
} rza2_gpio_data;

#define RZA2_IO_REG    DT_INST_REG_ADDR_BY_NAME(0, io)
#define RZA2_IO_SIZE   DT_INST_REG_SIZE_BY_NAME(0, io)
#define RZA2_DSCR_REG  DT_INST_REG_ADDR_BY_NAME(0, dscr)
#define RZA2_DSCR_SIZE DT_INST_REG_SIZE_BY_NAME(0, dscr)

#define RZA2_DSCR_MASK 0x03

#if DT_NODE_HAS_PROP(DT_DRV_INST(0), interrupts)
#define MAX_PORTS_FOR_INT 2
static const struct device *sub_gpio_ports[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_DRV_INST(0), DEVICE_DT_GET, (,))};

static const uint32_t sub_gpio_port_regs[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_DRV_INST(0), DT_REG_ADDR, (,))};

/* TINT pin map (According to the Table 51.37 of HW manual. 0xff indicates unused slot*/
static const struct gpio_port_int_map {
	uint32_t irq;
	uint32_t ports[MAX_PORTS_FOR_INT];
	uint16_t masks[MAX_PORTS_FOR_INT];
} gpio_port_map[] = {{TINT31, {0x0, 0xff}, {0x7f}},    {TINT30, {0x1, 0xff}, {0x1f}},
		     {TINT29, {0x2, 0x3}, {0xf, 0x1}}, {TINT28, {0x3, 0x4}, {0x3e, 0x1}},
		     {TINT27, {0x4, 0xff}, {0xfe}},    {TINT26, {0x5, 0xff}, {0xf}},
		     {TINT25, {0x5, 0xff}, {0xf0}},    {TINT24, {0x6, 0xff}, {0xf}},
		     {TINT23, {0x6, 0xff}, {0xf0}},    {TINT22, {0x7, 0xff}, {0xf}},
		     {TINT21, {0x7, 0xff}, {0xf0}},    {TINT20, {0x8, 0xff}, {0xf}},
		     {TINT19, {0x8, 0xff}, {0xf0}},    {TINT18, {0x9, 0xff}, {0xf}},
		     {TINT17, {0x9, 0xff}, {0xf0}},    {TINT16, {0xa, 0xff}, {0xf}},
		     {TINT15, {0xa, 0xff}, {0xf0}},    {TINT14, {0xb, 0xff}, {0x3f}},
		     {TINT13, {0xc, 0xff}, {0xf}},     {TINT12, {0xc, 0xff}, {0xf0}},
		     {TINT11, {0xd, 0xff}, {0xf}},     {TINT10, {0xd, 0xff}, {0xf0}},
		     {TINT9, {0xe, 0xff}, {0x7f}},     {TINT8, {0xf, 0xff}, {0xf}},
		     {TINT7, {0xf, 0xff}, {0xf0}},     {TINT6, {0x10, 0xff}, {0xf}},
		     {TINT5, {0x10, 0xff}, {0xf0}},    {TINT4, {0x11, 0xff}, {0x7f}},
		     {TINT3, {0x12, 0xff}, {0xf}},     {TINT2, {0x12, 0xff}, {0xf0}},
		     {TINT1, {0x13, 0xff}, {0x3f}},    {TINT0, {0x14, 0x15}, {0x1f, 0x1}}};

#endif /* DT_NODE_HAS_PROP(DT_DRV_INST(0), interrupts) */

uint8_t rza2_in_get(uint8_t port)
{
	return sys_read8(rza2_gpio_data.io_addr + RZA2_PIDR(port));
}

uint8_t rza2_out_get(uint8_t port)
{
	return sys_read8(rza2_gpio_data.io_addr + RZA2_PODR(port));
}

void rza2_out_set(uint8_t port, uint8_t value)
{
	sys_write8(value, rza2_gpio_data.io_addr + RZA2_PODR(port));
}

/*
 * GPIO HIGH is only possible for
 * PG_2, PG_3, PG_4, PG_5, PG_6, PG_7, PJ_0, PJ_1, PJ_2, PJ_3, PJ_4, PJ_5, and PJ_6
 * see 51.3.5 section of HW Manual
 */
static struct gpio_high_pins {
	uint8_t port;
	uint8_t mask;
} allowed_pins[] = {
	{0x10, BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7)},
	{0x12, BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6)},
};

static bool high_is_allowed(uint8_t port, uint8_t pin)
{
	int len = ARRAY_SIZE(allowed_pins);
	int i;

	for (i = 0; i < len; i++) {
		if (allowed_pins[i].port == port && (allowed_pins[i].mask & BIT(pin))) {
			return true;
		}
	}

	return false;
}

/* FIXME: There should be possible to set drive high for non GPIO pins */
int rza2_drive_set(uint8_t port, uint8_t pin, uint8_t flag)
{
	uint16_t reg16;
	uint16_t mask16;

	if (flag == RZA2_GPIO_LEVEL_HIGH && !high_is_allowed(port, pin)) {
		return -ENOTSUP;
	}

	reg16 = sys_read16(rza2_gpio_data.dscr_addr + RZA2_DSCR(port));
	mask16 = RZA2_DSCR_MASK << (pin * 2);
	reg16 &= ~mask16;
	reg16 |= flag << (pin * 2);

	sys_write16(reg16, rza2_gpio_data.dscr_addr + RZA2_DSCR(port));

	return 0;
}

bool rza2_pin_int_is_allowed(uint32_t port, uint8_t pin)
{
#if DT_NODE_HAS_PROP(DT_DRV_INST(0), interrupts)
	int i, j;

	for (i = 0; i < ARRAY_SIZE(gpio_port_map); i++) {
		for (j = 0; j < MAX_PORTS_FOR_INT; j++) {
			if (port == gpio_port_map[i].ports[j] &&
			    (gpio_port_map[i].masks[j] & BIT(pin))) {
				return true;
			}
		}
	}
#endif
	return false;
}

#if DT_NODE_HAS_PROP(DT_DRV_INST(0), interrupts)
static void fire_int_handler(uint32_t port, uint16_t mask)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sub_gpio_port_regs); i++) {
		if (sub_gpio_port_regs[i] == port) {
			rza2_isr_handler(sub_gpio_ports[i], mask);
		}
	}
}

static void gpio_rza2_isr_common(uint16_t idx)
{
	int j;

	for (j = 0; j < MAX_PORTS_FOR_INT; j++) {
		fire_int_handler(gpio_port_map[idx].ports[j], gpio_port_map[idx].masks[j]);
	}
}

#define IRQ_DECLARE_ISR(n, inst)                                                                   \
	static void gpio_rza2_##n##_isr(void *param)                                               \
	{                                                                                          \
		gpio_rza2_isr_common(ARRAY_SIZE(gpio_port_map) + TINT0 -                           \
				     DT_INST_IRQ_BY_IDX(inst, n, irq) - 1);                        \
	}

/*
 * FIXME: There is no need to keep irq line enable all the time.
 * irq_enable should be called when the fist pin is set to irq mode
 * and disabled for all related pins.
 */
#define IRQ_CONFIGURE(n, inst)                                                                     \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    gpio_rza2_##n##_isr, NULL, DT_INST_IRQ_BY_IDX(inst, n, flags));                \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define DECLARE_ALL_IRQS(inst, n)   LISTIFY(n, IRQ_DECLARE_ISR, (), inst)
#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

DECLARE_ALL_IRQS(0, DT_NUM_IRQS(DT_DRV_INST(0)))
#else
#define CONFIGURE_ALL_IRQS(inst, n)
#endif /* DT_NODE_HAS_PROP(DT_DRV_INST(0), interrupts) */

__boot_func static int gpio_rza2_driver_init(void)
{
	device_map(&rza2_gpio_data.io_addr, RZA2_IO_REG, RZA2_IO_SIZE, K_MEM_CACHE_NONE);
	device_map(&rza2_gpio_data.dscr_addr, RZA2_DSCR_REG, RZA2_DSCR_SIZE, K_MEM_CACHE_NONE);

	CONFIGURE_ALL_IRQS(0, DT_NUM_IRQS(DT_DRV_INST(0)))
	return 0;
}

SYS_INIT(gpio_rza2_driver_init, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY);
