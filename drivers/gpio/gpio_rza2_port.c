/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_r7s9210_gpio_port

#include <errno.h>
#include <soc.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/gpio/rza2-gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/slist.h>

#include "gpio_rza2.h"

LOG_MODULE_REGISTER(rza2, CONFIG_GPIO_LOG_LEVEL);

/* We can't set interrupt EDGE or LEVEL mode for each pin */
#if !defined(CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT)
#error "GPIO support only enable/disable of interrupt"
#endif

#define RZA2_GPIO_VALID_FLAGS                                                                      \
	RZA2_GPIO_DRIVE_MSK | GPIO_INPUT | GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH |                   \
		GPIO_OUTPUT_INIT_LOW

struct gpio_rza2_port_state {
	enum gpio_int_mode mode;
	bool configured;
	uint32_t direction;
};

struct gpio_rza2_port_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint8_t port;
	uint8_t ngpios;
};

struct gpio_rza2_port_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	struct gpio_rza2_port_state *states;
	uint8_t mask_irq_en;
};

static void rza2_pin_to_gpio(uint8_t port, uint8_t pin, uint8_t dir)
{
	pinctrl_soc_pin_t p;

	p.pin = pin;
	p.port = port;
	p.func = dir;

	pinctrl_configure_pins(&p, 1, PINCTRL_REG_NONE);
}

static int rza2_gpio_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_out_get(config->port);

	rza2_out_set(config->port, (base_value | pins));
	return 0;
}

static int rza2_gpio_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_out_get(config->port);

	rza2_out_set(config->port, (base_value & ~pins));
	return 0;
}

static int rza2_gpio_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	int ret = 0;
	const struct gpio_rza2_port_config *config = port->config;
	struct gpio_rza2_port_data *data = port->data;

	if (pin >= config->ngpios) {
		LOG_ERR("provided pin %d > %d (ngpios)", pin, config->ngpios);
		return -EINVAL;
	}

	if (!(flags ^ (RZA2_GPIO_VALID_FLAGS))) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) && (flags & GPIO_INPUT)) {
		/* Pin cannot be configured as input and output */
		return -ENOTSUP;
	} else if (!(flags & (GPIO_INPUT | GPIO_OUTPUT))) {
		/* Pin has to be configured as input or output */
		return -ENOTSUP;
	}

	if (flags & GPIO_INPUT) {
		rza2_pin_to_gpio(config->port, pin, FUNC_GPIO_INPUT);
		data->states[pin].direction = GPIO_INPUT;
	} else {
		rza2_pin_to_gpio(config->port, pin, FUNC_GPIO_OUTPUT);
		data->states[pin].direction = GPIO_OUTPUT;
	}

	if ((flags & RZA2_GPIO_DRIVE_MSK) == RZA2_GPIO_DRIVE_HIGH) {
		ret = rza2_drive_set(config->port, pin, RZA2_GPIO_LEVEL_HIGH);
	} else {
		ret = rza2_drive_set(config->port, pin, RZA2_GPIO_LEVEL_NORMAL);
	};

	if (ret) {
		LOG_ERR("unable to set gpio drive level");
		return ret;
	}

	if (flags & GPIO_OUTPUT_INIT_HIGH) {
		ret = rza2_gpio_port_set_bits_raw(port, BIT(pin));
	} else if (flags & GPIO_OUTPUT_INIT_LOW) {
		ret = rza2_gpio_port_clear_bits_raw(port, BIT(pin));
	}

	data->states[pin].configured = true;
	return ret;
}

static uint8_t rza2_get_dir_mask(const struct device *port, uint32_t direction)
{
	int i;
	uint8_t ret = 0;
	const struct gpio_rza2_port_config *config = port->config;
	struct gpio_rza2_port_data *data = port->data;

	for (i = 0; i < config->ngpios; i++) {
		if (data->states[i].configured && data->states[i].direction == direction) {
			ret |= BIT(i);
		}
	}

	return ret;
}

static int rza2_gpio_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t mask = rza2_get_dir_mask(port, GPIO_INPUT);

	*value = rza2_in_get(config->port) & mask;

	return 0;
}

static int rza2_gpio_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_out_get(config->port);

	rza2_out_set(config->port, (base_value & ~mask) | (value & mask));
	return 0;
}

static int rza2_gpio_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_out_get(config->port);

	rza2_out_set(config->port, (base_value ^ pins));
	return 0;
}

static int rza2_gpio_manage_callback(const struct device *dev, struct gpio_callback *callback,
				     bool set)
{
	struct gpio_rza2_port_data *data = dev->data;

	gpio_manage_callback(&data->callbacks, callback, set);
	return 0;
}

static int rza2_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_rza2_port_config *config = dev->config;
	struct gpio_rza2_port_data *data = dev->data;
	int flag;

	/* We only set Enable/disable flag. Trigger is ignored because only high is supported */
	ARG_UNUSED(trig);

	if (pin > config->ngpios) {
		return -EINVAL;
	}

	if (!data->states[pin].configured || data->states[pin].direction != GPIO_INPUT) {
		return -EINVAL;
	}

	if (!rza2_pin_int_is_allowed(config->port, pin)) {
		return -EINVAL;
	}

	if (mode & GPIO_INT_MODE_ENABLE_ONLY) {
		flag = FUNC_GPIO_INT_EN;
		data->mask_irq_en |= BIT(pin);
	} else if (mode & GPIO_INT_MODE_DISABLE_ONLY) {
		flag = FUNC_GPIO_INT_DIS;
		data->mask_irq_en &= ~BIT(pin);
	} else {
		return -EINVAL;
	}

	data->states[pin].mode = mode;

	rza2_pin_to_gpio(config->port, pin, flag);
	return 0;
}

static const struct gpio_driver_api rza2_gpio_driver_api = {
	.pin_configure = rza2_gpio_configure,
	.port_get_raw = rza2_gpio_port_get_raw,
	.port_set_masked_raw = rza2_gpio_port_set_masked_raw,
	.port_set_bits_raw = rza2_gpio_port_set_bits_raw,
	.port_clear_bits_raw = rza2_gpio_port_clear_bits_raw,
	.port_toggle_bits = rza2_gpio_port_toggle_bits,
	.pin_interrupt_configure = rza2_gpio_pin_interrupt_configure,
	.manage_callback = rza2_gpio_manage_callback,
};

static int get_high_pin(struct gpio_rza2_port_data *data, uint8_t value)
{
	return find_lsb_set(data->mask_irq_en & value) - 1;
}

void rza2_isr_handler(const struct device *port, uint8_t mask)
{
	const struct gpio_rza2_port_config *config = port->config;
	struct gpio_rza2_port_data *data = port->data;
	uint8_t value = rza2_in_get(config->port);
	int pin = get_high_pin(data, value & mask);

	if (pin < 0) {
		return;
	}

	gpio_fire_callbacks(&data->callbacks, port, BIT(pin));
}

static int gpio_rza2_port_init(const struct device *port)
{
	return 0;
}

#define GPIO_RZA2_PORT_INIT(inst)                                                                  \
	static struct gpio_rza2_port_state gpio_##inst##_states[DT_INST_PROP(inst, ngpios)] = {0}; \
	static const struct gpio_rza2_port_config gpio_rza2_cfg_##inst = {                         \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.port = DT_INST_REG_ADDR(inst),                                                    \
		.ngpios = DT_INST_PROP(inst, ngpios),                                              \
	};                                                                                         \
	static struct gpio_rza2_port_data gpio_rza2_data_##inst = {                                \
		.states = (struct gpio_rza2_port_state *)&gpio_##inst##_states, .mask_irq_en = 0}; \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_rza2_port_init, NULL, &gpio_rza2_data_##inst,             \
			      &gpio_rza2_cfg_##inst, POST_KERNEL,                                  \
			      CONFIG_GPIO_RZA2_PORT_INIT_PRIORITY, &rza2_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_RZA2_PORT_INIT)
