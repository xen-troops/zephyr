/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_pinctrl

#include <errno.h>
#include <soc.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#define RZA2_PDR(port)      (0x0000 + (port) * 2)           /* Direction 16-bit */
#define RZA2_PMR(port)      (0x0000 + (port))               /* Mode 8-bit */
#define RZA2_PFS(port, pin) (0x0180 + ((port) * 8) + (pin)) /* Fnct 8-bit */

#define RZA2_PWPR 0x027f /* Write Protect 8-bit */

#define RZA2_PDR_INPUT  0x02
#define RZA2_PDR_OUTPUT 0x03
#define RZA2_PDR_MASK   0x03

#define PWPR_PFSWE BIT(6) /* PFSWE Bit Enable */
#define PWPR_B0WI  BIT(7) /* PFSWE Bit Disable */

#define RZA2_PDR_REG      DT_INST_REG_ADDR_BY_NAME(0, pdr)
#define RZA2_PDR_SIZE     DT_INST_REG_SIZE_BY_NAME(0, pdr)
#define RZA2_PINCTRL_REG  DT_INST_REG_ADDR_BY_NAME(0, pinctrl)
#define RZA2_PINCTRL_SIZE DT_INST_REG_SIZE_BY_NAME(0, pinctrl)

static struct rza2_pinctrl_data {
	mm_reg_t pdr_addr;
	mm_reg_t pinctrl_addr;
	struct k_mutex lock;
} rza2_pinctrl_data;

static void rza2_pin_write_prot(uint8_t mask)
{
	/* PFS Register Write Protect set*/
	sys_write8(0x00, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);
	sys_write8(mask, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);
}

static void rza2_set_pin_function(uint8_t port, uint8_t pin, uint8_t func)
{
	uint16_t mask16;
	uint16_t reg16;
	uint8_t reg8;

	k_mutex_lock(&rza2_pinctrl_data.lock, K_FOREVER);

	/* Set pin to 'Non-use (Hi-z input protection)'  */
	reg16 = sys_read8(rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));
	mask16 = RZA2_PDR_MASK << (pin * 2);
	reg16 &= ~mask16;
	sys_write16(reg16, rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));

	/* Temporarily switch to GPIO */
	reg8 = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));
	reg8 &= ~BIT(pin);
	sys_write8(reg8, rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));

	/* PFS Register Write Protect : OFF */
	rza2_pin_write_prot(PWPR_PFSWE); /* B0WI=0, PFSWE=1 */

	/* Set Pin function (interrupt disabled, ISEL=0) */
	sys_write8(func, rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));

	/* PFS Register Write Protect : ON */
	rza2_pin_write_prot(PWPR_B0WI); /* B0WI=1, PFSWE=0 */

	/* Port Mode  : Peripheral module pin functions */
	reg8 = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));
	reg8 |= BIT(pin);

	sys_write8(reg8, rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));

	k_mutex_unlock(&rza2_pinctrl_data.lock);
}

static void rza2_pin_to_gpio(uint8_t port, uint8_t pin, uint8_t dir)
{
	uint16_t mask16;
	uint16_t reg16;
	uint8_t reg8;

	k_mutex_lock(&rza2_pinctrl_data.lock, K_FOREVER);

	reg16 = sys_read16(rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));
	mask16 = RZA2_PDR_MASK << (pin * 2);
	reg16 &= ~mask16;
	reg16 |= dir << (pin * 2);

	sys_write16(reg16, rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));

	reg8 = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));
	reg8 &= ~BIT(pin);
	sys_write8(reg8, rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));

	k_mutex_unlock(&rza2_pinctrl_data.lock);
}

static void rza2_set_gpio_int(uint8_t port, uint8_t pin, bool int_en)
{
	uint8_t value;
	/* PFS Register Write Protect : OFF */
	rza2_pin_write_prot(PWPR_PFSWE); /* B0WI=0, PFSWE=1 */

	if (int_en) {
		/* Enable interrupt, ISEL=1 */
		sys_write8(BIT(6), rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));
	} else {
		value = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));
		value &= ~BIT(6);
		sys_write8(value, rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));
	}
	/* PFS Register Write Protect : ON */
	rza2_pin_write_prot(PWPR_B0WI); /* B0WI=1, PFSWE=0 */
}

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pin)
{
	if (pin.func & FUNC_GPIO_INPUT) {
		rza2_pin_to_gpio(pin.port, pin.pin, RZA2_PDR_INPUT);
	} else if (pin.func & FUNC_GPIO_OUTPUT) {
		rza2_pin_to_gpio(pin.port, pin.pin, RZA2_PDR_OUTPUT);
	} else if (pin.func & FUNC_GPIO_INT_EN) {
		rza2_set_gpio_int(pin.port, pin.pin, true);
	} else if (pin.func & FUNC_GPIO_INT_DIS) {
		rza2_set_gpio_int(pin.port, pin.pin, false);
	} else {
		rza2_set_pin_function(pin.port, pin.pin, pin.func);
	}
	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);
	while (pin_cnt-- > 0U) {
		ret = pinctrl_configure_pin(*pins++);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

__boot_func static int pinctrl_rza2_driver_init(void)
{
	device_map(&rza2_pinctrl_data.pdr_addr, RZA2_PDR_REG, RZA2_PDR_SIZE, K_MEM_CACHE_NONE);
	device_map(&rza2_pinctrl_data.pinctrl_addr, RZA2_PINCTRL_REG, RZA2_PINCTRL_SIZE,
		   K_MEM_CACHE_NONE);

	k_mutex_init(&rza2_pinctrl_data.lock);
	return 0;
}

SYS_INIT(pinctrl_rza2_driver_init, PRE_KERNEL_1, CONFIG_PINCTRL_RZA2_INIT_PRIORITY);
