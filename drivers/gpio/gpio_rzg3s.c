/*
 * Copyright 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r9a08g045_gpio

#include <soc.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/gpio/rzg3s-gpio.h>
#include <zephyr/dt-bindings/pinctrl/renesas/pinctrl-r9a08g045.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rzg3s_gpio, CONFIG_GPIO_LOG_LEVEL);

#define LOG_DEV_ERR(dev, format, ...) LOG_ERR("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...) LOG_WRN("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...) LOG_INF("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...) LOG_DBG("%s:" #format, (dev)->name, ##__VA_ARGS__)

#define R_P(port)   (0x0000 + (port))  /* Port Register, Output Data 8-bit */
#define R_PIN(port) (0x0800 + (port))  /* Port Input Register, Input Data 8-bit */
#define R_PM(n)     (0x0100 + (n) * 2) /* Port Mode 16-bit */
#define R_ISEL(n)   (0x2C00 + (n) * 8) /* Interrupt Enable Control 32-bit */

/* R_PM Port Mode Register */
#define R_PM_MODE_MASK          GENMASK(1, 0)
#define R_PM_MODE_VAL(pin, val) (((val) & R_PM_MODE_MASK) << ((pin) << 1))

#define R_PM_MODE_HI_Z   0x0
#define R_PM_MODE_IN     BIT(0)
#define R_PM_MODE_OUT    BIT(1)
#define R_PM_MODE_OUT_IN (R_PM_MODE_IN | R_PM_MODE_OUT)

#define R_ISEL_MODE_MASK          BIT(1)
#define R_ISEL_MODE_VAL(pin, val) (((val) & R_ISEL_MODE_MASK) << ((pin) * 8))

#define RZG3S_PORT_NUM 19

#define RZG3S_GPIO_VALID_FLAGS                                                                     \
	(GPIO_INPUT | GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOW | GPIO_PULL_UP |  \
	 GPIO_PULL_DOWN)

/*
 * num_pins: indicates number of pins in the port
 * reg_ofs: the register index
 * irq_ofs: GPIO irq offset within TINT
 */
#define RZG3S_GPIO_PORT_DEF(num_pins, reg_ofs, irq_ofs)                                            \
	(((num_pins) << 16) | (reg_ofs) | ((irq_ofs) << 24))
#define RZG3S_GPIO_PORT_GET_PINCNT(p)  (((p) & GENMASK(23, 16)) >> 16)
#define RZG3S_GPIO_PORT_GET_REG(p)     (((p) & GENMASK(15, 0)))
#define RZG3S_GPIO_PORT_GET_IRQ_OFS(p) (((p) & GENMASK(31, 24)) >> 24)

static const uint32_t rzg3s_gpio_ports[RZG3S_PORT_NUM] = {
	RZG3S_GPIO_PORT_DEF(4, 0x20, 0),
	RZG3S_GPIO_PORT_DEF(5, 0x30, 4),
	RZG3S_GPIO_PORT_DEF(4, 0x31, 9),
	RZG3S_GPIO_PORT_DEF(4, 0x32, 13),
	RZG3S_GPIO_PORT_DEF(6, 0x33, 17),
	RZG3S_GPIO_PORT_DEF(5, 0x21, 23),
	RZG3S_GPIO_PORT_DEF(5, 0x22, 28),
	RZG3S_GPIO_PORT_DEF(5, 0x34, 33),
	RZG3S_GPIO_PORT_DEF(5, 0x35, 38),
	RZG3S_GPIO_PORT_DEF(4, 0x36, 43),
	RZG3S_GPIO_PORT_DEF(5, 0x37, 47),
	RZG3S_GPIO_PORT_DEF(4, 0x23, 52),
	RZG3S_GPIO_PORT_DEF(2, 0x24, 56),
	RZG3S_GPIO_PORT_DEF(5, 0x25, 58),
	RZG3S_GPIO_PORT_DEF(3, 0x26, 63),
	RZG3S_GPIO_PORT_DEF(4, 0x27, 66),
	RZG3S_GPIO_PORT_DEF(2, 0x28, 70),
	RZG3S_GPIO_PORT_DEF(4, 0x29, 72),
	RZG3S_GPIO_PORT_DEF(6, 0x2A, 76),
};

/* TINT */
#define R_TINT_TSCR       0x0
#define R_TINT_TITSR(irq) (0x4 + 0x4 * ((irq) / 16))
#define R_TINT_TSSR(irq)  (0x10 + 0x4 * ((irq) / 4))

/* TINT Interrupt Type Selection Register */
#define R_TITSR_TITSEL_MASK          GENMASK(1, 0)
#define R_TITSR_TITSEL_VAL(irq, val) (((val) & R_TITSR_TITSEL_MASK) << (((irq) % 16) * 2))

#define R_TITSR_TITSEL_EDGE_RISING  0x0
#define R_TITSR_TITSEL_EDGE_FALLING 0x1
#define R_TITSR_TITSEL_LEVEL_HIGH   0x2
#define R_TITSR_TITSEL_LEVEL_LOW    0x3

/* TINT Source Selection Register */
#define R_TSSR_TSSEL_MASK          GENMASK(6, 0)
#define R_TSSR_TSSEL_VAL(irq, val) (((val) & R_TSSR_TSSEL_MASK) << (((irq) % 4) * 8))
#define R_TSSR_TIEN_MASK          BIT(7)
#define R_TSSR_TIEN_VAL(irq, val) (((val) & R_TSSR_TIEN_MASK) << (((irq) % 4) * 8))

struct rzg3s_gpio_port_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint8_t port;
	uint8_t ngpios;
	const struct device *pinctrl_dev;
	const struct device *ctrl_dev;
};

struct rzg3s_gpio_port_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	uint8_t		mask_dir_out;
	uint8_t		mask_dir_in;
	uint8_t		mask_irq_en;

	struct k_spinlock lock;
};

#define TINT_NUM_IRQ	32

struct rzg3s_gpio_ctrl_config {
	DEVICE_MMIO_NAMED_ROM(tint);
	const struct device	*pinctrl_dev;
	unsigned int		parent_irqs[TINT_NUM_IRQ];
	void			(*irq_config)(void);
};

struct rzg3s_gpio_tint_irq_data {
	const struct device *port_dev;
	uint8_t pin;    /* pin within port */
	uint8_t pin_id; /* global pin number 0..81 */
};

struct rzg3s_gpio_ctrl_data {
	DEVICE_MMIO_NAMED_ROM(tint);
	mm_reg_t gpio_addr;
	struct rzg3s_gpio_tint_irq_data tints[TINT_NUM_IRQ];
	uint32_t free_irqs_mask;
	uint32_t edge_irqs_mask;
	struct k_spinlock tint_lock;
} rza2_gpio_data;

#define DEV_DATA(dev) ((struct rzg3s_gpio_ctrl_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct rzg3s_gpio_ctrl_config *)((dev)->config))

static int rzg3s_gpio_tint_free(const struct device *dev, const struct device *port_dev,
				uint8_t pin);

static int rzg3s_gpio_tint_allocen(const struct device *dev, const struct device *port_dev,
				   uint8_t pin, uint8_t pin_id, uint8_t irq_type);

static uint16_t rzg3s_pinctrl_port_get_reg(uint8_t port)
{
	return RZG3S_GPIO_PORT_GET_REG(rzg3s_gpio_ports[port]);
}

static uint16_t __maybe_unused rzg3s_pinctrl_port_get_pinnum(uint8_t port)
{
	return RZG3S_GPIO_PORT_GET_PINCNT(rzg3s_gpio_ports[port]);
}

static uint8_t rzg3s_gpio_in_get(mm_reg_t base, uint8_t port)
{
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(port);

	return sys_read8(base + R_PIN(port_reg));
}

static uint8_t rzg3s_gpio_out_get(mm_reg_t base, uint8_t port)
{
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(port);

	return sys_read8(base + R_P(port_reg));
}

static void rzg3s_gpio_out_set(mm_reg_t base, uint8_t port, uint8_t value)
{
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(port);

	sys_write8(value, base + R_P(port_reg));
}

static int rzg3s_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	struct rzg3s_gpio_port_data *data = dev->data;

	*value = rzg3s_gpio_in_get(base, cfg->port) & data->mask_dir_in;

	return 0;
}

static int rzg3s_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	struct rzg3s_gpio_port_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t reg8;

	key = k_spin_lock(&data->lock);
	reg8 = rzg3s_gpio_out_get(base, cfg->port);
	reg8 &= ~mask;
	reg8 |= ((value & data->mask_dir_out) & mask);
	rzg3s_gpio_out_set(base, cfg->port, reg8);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int rzg3s_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	struct rzg3s_gpio_port_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t reg8;

	key = k_spin_lock(&data->lock);
	reg8 = rzg3s_gpio_out_get(base, cfg->port);
	reg8 |= pins & data->mask_dir_out;
	rzg3s_gpio_out_set(base, cfg->port, reg8);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int rzg3s_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	struct rzg3s_gpio_port_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t reg8;

	key = k_spin_lock(&data->lock);
	reg8 = rzg3s_gpio_out_get(base, cfg->port);
	reg8 &= ~(pins & data->mask_dir_out);
	rzg3s_gpio_out_set(base, cfg->port, reg8);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int rzg3s_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	struct rzg3s_gpio_port_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t reg8;

	key = k_spin_lock(&data->lock);
	reg8 = rzg3s_gpio_out_get(base, cfg->port);
	reg8 ^= (pins & data->mask_dir_out);
	rzg3s_gpio_out_set(base, cfg->port, reg8);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static void rzg3s_gpio_port_set_dir(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(cfg->port);
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	uint16_t reg16;

	reg16 = sys_read16(base + R_PM(port_reg));

	reg16 &= ~R_PM_MODE_VAL(pin, R_PM_MODE_MASK);
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == (GPIO_INPUT | GPIO_OUTPUT)) {
		reg16 |= R_PM_MODE_VAL(pin, R_PM_MODE_OUT_IN);
	} else if (flags & GPIO_INPUT) {
		reg16 |= R_PM_MODE_VAL(pin, R_PM_MODE_IN);
	} else if (flags & GPIO_OUTPUT) {
		reg16 |= R_PM_MODE_VAL(pin, R_PM_MODE_OUT);
	}

	sys_write16(reg16, base + R_PM(port_reg));
}

static int rzg3s_gpio_port_pin_to_gpio(uint8_t port, gpio_pin_t pin, gpio_flags_t flags)
{
	pinctrl_soc_pin_t pin_cfg = { 0 };

	pin_cfg.type = PINCTRL_RZG3S_TYPE_PINMUX;
	pin_cfg.pinmux.port = port;
	pin_cfg.pinmux.pin = pin;
	pin_cfg.pinmux.func = PINCTRL_RZG3S_FUNC_GPIO;

	if (flags & GPIO_PULL_DOWN) {
		pin_cfg.pinmux.pull_down = 1;
	} else if (flags & GPIO_PULL_UP) {
		pin_cfg.pinmux.pull_up = 1;
	}

	if (flags & RZG3S_GPIO_DRIVE_IOLH_PRESENT) {
		pin_cfg.pinmux.drive_strength = 1;
		pin_cfg.pinmux.drive_strength_microamp = RZG3S_GPIO_DRIVE_IOLH_GET(flags);
	}

	return pinctrl_configure_pins(&pin_cfg, 1, PINCTRL_REG_NONE);
}

static int rzg3s_gpio_port_pin_cfg(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	struct rzg3s_gpio_port_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	if (pin >= cfg->ngpios) {
		LOG_DEV_ERR(dev, "port:%u provided pin %d > %d (ngpios)",
			    cfg->port, pin, cfg->ngpios);
		return -EINVAL;
	}

	if (!(flags ^ (RZG3S_GPIO_VALID_FLAGS))) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	data->mask_dir_in &= ~BIT(pin);
	data->mask_dir_out &= ~BIT(pin);
	if (flags & GPIO_INPUT) {
		data->mask_dir_in |= BIT(pin);
	}
	if (flags & GPIO_OUTPUT) {
		data->mask_dir_out = BIT(pin);
	}

	ret = rzg3s_gpio_port_pin_to_gpio(cfg->port, pin, flags);
	if (ret) {
		k_spin_unlock(&data->lock, key);
		LOG_DEV_ERR(dev, "port:%u pin %d pincfg failed %d", cfg->port, pin, ret);
		return ret;
	}
	rzg3s_gpio_port_set_dir(dev, pin, flags);
	k_spin_unlock(&data->lock, key);

	if (flags & GPIO_OUTPUT_INIT_HIGH) {
		ret = rzg3s_gpio_port_set_bits_raw(dev, BIT(pin));
	} else if (flags & GPIO_OUTPUT_INIT_LOW) {
		ret = rzg3s_gpio_port_clear_bits_raw(dev, BIT(pin));
	}

	return ret;
}

static int rzg3s_gpio_port_manage_callback(const struct device *dev, struct gpio_callback *callback,
					   bool set)
{
	struct rzg3s_gpio_port_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static void rzg3s_gpio_isel_set(const struct device *dev, uint8_t pin, bool on)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(cfg->port);
	mm_reg_t base = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	uint8_t isel = 0;
	uint32_t reg32;

	if (on) {
		isel = BIT(1);
	}

	/* handle _L/_H for 32-bit register read/write */
	if (pin >= 4) {
		pin -= 4;
		port_reg += 4;
	}

	reg32 = sys_read32(base + R_ISEL(port_reg));
	reg32 &= ~R_ISEL_MODE_VAL(pin, R_ISEL_MODE_MASK);
	reg32 |= R_ISEL_MODE_VAL(pin, isel);
	sys_write32(reg32, base + R_ISEL(port_reg));
}

static int rzg3s_gpio_pin_irq_cfg(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode,
				  enum gpio_int_trig trig)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;
	struct rzg3s_gpio_port_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t irq_type;
	uint8_t pin_id;
	int ret = 0;

	if (pin >= cfg->ngpios) {
		LOG_DEV_ERR(dev, "port:%u provided pin %d > %d (ngpios)",
			    cfg->port, pin, cfg->ngpios);
		return -EINVAL;
	}

	if (trig == GPIO_INT_TRIG_BOTH) {
		return -ENOTSUP;
	}

	pin_id = RZG3S_GPIO_PORT_GET_IRQ_OFS(rzg3s_gpio_ports[cfg->port]) + pin;

	key = k_spin_lock(&data->lock);

	if (mode == GPIO_INT_MODE_DISABLED) {
		if (!(data->mask_irq_en & BIT(pin))) {
			/* not enabled - just exit */
			goto exit_unlock;
		}
		rzg3s_gpio_isel_set(dev, pin, false);
		ret = rzg3s_gpio_tint_free(cfg->ctrl_dev, dev, pin);
		data->mask_irq_en &= ~BIT(pin);
		goto exit_unlock;
	}

	if (!(data->mask_dir_in & BIT(pin))) {
		LOG_DEV_ERR(dev, "port:%u pin:%d not cfg as input", cfg->port, pin);
		ret = -EINVAL;
		goto exit_unlock;
	}

	if (mode == GPIO_INT_MODE_EDGE) {
		irq_type = R_TITSR_TITSEL_EDGE_RISING;
		if (trig == GPIO_INT_TRIG_LOW) {
			irq_type = R_TITSR_TITSEL_EDGE_FALLING;
		}
	} else {
		irq_type = R_TITSR_TITSEL_LEVEL_HIGH;
		if (trig == GPIO_INT_TRIG_LOW) {
			irq_type = R_TITSR_TITSEL_LEVEL_LOW;
		}
	}

	rzg3s_gpio_isel_set(dev, pin, true);
	ret = rzg3s_gpio_tint_allocen(cfg->ctrl_dev, dev, pin, pin_id, irq_type);
	if (ret) {
		rzg3s_gpio_isel_set(dev, pin, false);
	} else {
		data->mask_irq_en |= BIT(pin);
	}

exit_unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static const struct gpio_driver_api rzg3s_gpio_port_api = {
	.port_get_raw = rzg3s_gpio_port_get_raw,
	.port_set_masked_raw = rzg3s_gpio_port_set_masked_raw,
	.port_set_bits_raw = rzg3s_gpio_port_set_bits_raw,
	.port_clear_bits_raw = rzg3s_gpio_port_clear_bits_raw,
	.port_toggle_bits = rzg3s_gpio_port_toggle_bits,
	.pin_configure = rzg3s_gpio_port_pin_cfg,
	.manage_callback = rzg3s_gpio_port_manage_callback,
	.pin_interrupt_configure = rzg3s_gpio_pin_irq_cfg,
};

static void rzg3s_gpio_port_isr(const struct device *dev, uint8_t pin)
{
	struct rzg3s_gpio_port_data *data = dev->data;

	gpio_fire_callbacks(&data->callbacks, dev, BIT(pin));
}

static int rzg3s_gpio_port_init(const struct device *dev)
{
	const struct rzg3s_gpio_port_config *cfg = dev->config;

	ARG_UNUSED(cfg);
	__ASSERT(cfg->port < RZG3S_PORT_NUM, "gpio port%u incorrect, check DT", cfg->port);
	__ASSERT(cfg->ngpios == rzg3s_pinctrl_port_get_pinnum(cfg->port),
		 "gpio port%u incorrect ngpios %u, check DT", cfg->port, cfg->ngpios);

	return 0;
}

static void rzg3s_gpio_tint_free_raw(const struct device *dev, uint8_t tirq)
{
	uint16_t port_reg = R_TINT_TSSR(tirq);
	uint32_t reg32;

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
	reg32 &= ~R_TSSR_TSSEL_VAL(tirq, R_TSSR_TSSEL_MASK);
	reg32 &= ~R_TSSR_TIEN_VAL(tirq, R_TSSR_TIEN_MASK);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);

	/* clean status */
	port_reg = R_TINT_TSCR;
	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
	reg32 &= ~BIT(tirq);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
}

static void rzg3s_gpio_tint_allocen_raw(const struct device *dev, uint8_t tirq, uint8_t pin_id)
{
	struct rzg3s_gpio_ctrl_data *data = dev->data;
	uint16_t port_reg = R_TINT_TSSR(tirq);
	uint32_t reg32;

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
	reg32 &= ~R_TSSR_TSSEL_VAL(tirq, R_TSSR_TSSEL_MASK);
	reg32 |= R_TSSR_TSSEL_VAL(tirq, pin_id);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);

	k_busy_wait(100);

	/* clean status for edge */
	if (data->edge_irqs_mask & BIT(tirq)) {
		port_reg = R_TINT_TSCR;
		reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
		if (reg32 & BIT(tirq)) {
			reg32 &= ~BIT(tirq);
			sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
		}
	}

	port_reg = R_TINT_TSSR(tirq);
	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
	reg32 |= R_TSSR_TIEN_VAL(tirq, R_TSSR_TIEN_MASK);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
}

static void rzg3s_gpio_tint_set_type_raw(const struct device *dev, uint8_t tirq, uint8_t irq_type)
{
	struct rzg3s_gpio_ctrl_data *data = dev->data;
	uint16_t port_reg = R_TINT_TITSR(tirq);
	uint32_t reg32;

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
	reg32 &= ~R_TITSR_TITSEL_VAL(tirq, R_TITSR_TITSEL_MASK);
	reg32 |= R_TITSR_TITSEL_VAL(tirq, irq_type);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);

	if (irq_type == R_TITSR_TITSEL_EDGE_RISING ||
	    irq_type == R_TITSR_TITSEL_EDGE_FALLING) {
		data->edge_irqs_mask |= BIT(tirq);
	}
}

static uint8_t rzg3s_gpio_tint_find(const struct device *dev, const struct device *port_dev,
				    uint8_t pin)
{
	struct rzg3s_gpio_ctrl_data *data = dev->data;
	uint8_t i;

	for (i = 0; i < TINT_NUM_IRQ; i++) {
		if (data->tints[i].port_dev == port_dev && data->tints[i].pin == pin) {
			break;
		}
	}

	return i;
}

static int rzg3s_gpio_tint_free(const struct device *dev, const struct device *port_dev,
				uint8_t pin)
{
	const struct rzg3s_gpio_ctrl_config *cfg = dev->config;
	struct rzg3s_gpio_ctrl_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t tirq;

	key = k_spin_lock(&data->tint_lock);

	tirq = rzg3s_gpio_tint_find(dev, port_dev, pin);
	if (tirq == TINT_NUM_IRQ) {
		LOG_DEV_DBG(dev, "tint: %s:pin:%u not enabled", port_dev->name, pin);
		k_spin_unlock(&data->tint_lock, key);
		return -ENOENT;
	}

	irq_disable(cfg->parent_irqs[tirq]);

	rzg3s_gpio_tint_free_raw(dev, tirq);
	data->tints[tirq].port_dev = NULL;
	data->tints[tirq].pin = UINT8_MAX;
	data->tints[tirq].pin_id = UINT8_MAX;
	data->free_irqs_mask |= BIT(tirq);
	data->edge_irqs_mask &= ~BIT(tirq);

	k_spin_unlock(&data->tint_lock, key);
	return 0;
}

static int rzg3s_gpio_tint_allocen(const struct device *dev, const struct device *port_dev,
				   uint8_t pin, uint8_t pin_id, uint8_t irq_type)
{
	const struct rzg3s_gpio_ctrl_config *cfg = dev->config;
	struct rzg3s_gpio_ctrl_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t tirq;
	int ret = 0;

	key = k_spin_lock(&data->tint_lock);

	tirq = rzg3s_gpio_tint_find(dev, port_dev, pin);
	if (tirq != TINT_NUM_IRQ) {
		LOG_DEV_DBG(dev, "tint: %s:pin:%u already in use", port_dev->name, pin);
		ret = -ENOENT;
		goto exit_unlock;
	}

	tirq = find_lsb_set(data->free_irqs_mask);
	if (!tirq) {
		LOG_DEV_DBG(dev, "tint: no free irqs");
		ret = -EBUSY;
		goto exit_unlock;

	}
	tirq--;

	rzg3s_gpio_tint_set_type_raw(dev, tirq, irq_type);
	rzg3s_gpio_tint_allocen_raw(dev, tirq, pin_id);
	data->tints[tirq].port_dev = port_dev;
	data->tints[tirq].pin = pin;
	data->tints[tirq].pin_id = pin_id;
	data->free_irqs_mask &= ~BIT(tirq);
	irq_enable(cfg->parent_irqs[tirq]);

exit_unlock:
	k_spin_unlock(&data->tint_lock, key);
	return ret;
}

static void rzg3s_gpio_tint_isr(uint32_t p_irq, void *param)
{
	const struct rzg3s_gpio_ctrl_config *cfg;
	struct rzg3s_gpio_ctrl_data *data;
	const struct device *dev = param;
	uint16_t port_reg = R_TINT_TSCR;
	uint32_t reg32;
	uint8_t tirq;

	cfg = dev->config;
	data = dev->data;
	tirq = p_irq - cfg->parent_irqs[0];
	__ASSERT_NO_MSG(tirq < TINT_NUM_IRQ);

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);

	if (!(reg32 & BIT(tirq))) {
		/* For unknown reasons, the spurious irq generated with TSTATn == 0
		 * when TINT IRQ configured and enabled for the first time.
		 * Same is not seen for further IRQ events.
		 */
		LOG_DEV_DBG(dev, "tint:%u spurious irq, status 0", tirq);
		return;
	}

	if (!data->tints[tirq].port_dev) {
		k_spinlock_key_t key;

		/* spurious irq disable it */
		key = k_spin_lock(&data->tint_lock);
		rzg3s_gpio_tint_free_raw(dev, tirq);
		k_spin_unlock(&data->tint_lock, key);
		LOG_DEV_ERR(dev, "tint:%u spurious irq, disable it", tirq);
		return;
	}

	if (data->edge_irqs_mask & BIT(tirq)) {
		/* clean status for Edge irq types */
		reg32 &= ~BIT(tirq);
		sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, tint) + port_reg);
	}

	rzg3s_gpio_port_isr(data->tints[tirq].port_dev, data->tints[tirq].pin);
}

static int rzg3s_gpio_ctrl_init(const struct device *dev)
{
	const struct rzg3s_gpio_ctrl_config *cfg = dev->config;
	struct rzg3s_gpio_ctrl_data *data = dev->data;

	/* Get IO addr from parent */
	data->gpio_addr = DEVICE_MMIO_GET(cfg->pinctrl_dev);
	data->free_irqs_mask = UINT32_MAX;
	DEVICE_MMIO_NAMED_MAP(dev, tint, K_MEM_CACHE_NONE);

	cfg->irq_config();
	return 0;
}

#define GPIO_RZG3S_PORT_DATA_NAME(node_id) _CONCAT(rzg3s_gpio_port_data, DT_DEP_ORD(node_id))
#define GPIO_RZG3S_PORT_CFG_NAME(node_id) _CONCAT(rzg3s_gpio_port_cfg, DT_DEP_ORD(node_id))

#define GPIO_RZG3S_PORT_INIT(p_node)                                                               \
	static const struct rzg3s_gpio_port_config GPIO_RZG3S_PORT_CFG_NAME(p_node) = {            \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(p_node),          \
			},                                                                         \
		.port = DT_REG_ADDR(p_node),                                                       \
		.ngpios = DT_PROP(p_node, ngpios),                                                 \
		.pinctrl_dev = DEVICE_DT_GET(DT_PARENT(DT_PARENT(p_node))),                        \
		.ctrl_dev = DEVICE_DT_GET(DT_PARENT(p_node)),                                      \
	};                                                                                         \
	static struct rzg3s_gpio_port_data GPIO_RZG3S_PORT_DATA_NAME(p_node) = {};                 \
                                                                                                   \
	DEVICE_DT_DEFINE(p_node, rzg3s_gpio_port_init, NULL, &GPIO_RZG3S_PORT_DATA_NAME(p_node),   \
			 &GPIO_RZG3S_PORT_CFG_NAME(p_node), POST_KERNEL,                           \
			 CONFIG_GPIO_RZG3S_PORT_INIT_PRIORITY, &rzg3s_gpio_port_api);

#define GPIO_RZG3S_IRQ_DECLARE_ISR(n, inst)                                                        \
	static void rzg3s_gpio_tint_##n##_isr(void *param)                                         \
	{                                                                                          \
		rzg3s_gpio_tint_isr(DT_INST_IRQ_BY_IDX(inst, n, irq), param);                      \
	}
#define GPIO_RZG3S_DECLARE_ALL_ISRS(inst, n) LISTIFY(n, GPIO_RZG3S_IRQ_DECLARE_ISR, (), inst)

#define GPIO_RZG3S_IRQ_CONFIGURE(n, inst)                                                          \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    rzg3s_gpio_tint_##n##_isr, DEVICE_DT_INST_GET(inst), 0);

#define GPIO_RZG3S_IRQ_CONFIG_FUNC(inst)                                                           \
	static void rzg3s_gpio_tint_cfg_func_##inst(void)                                          \
	{                                                                                          \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(inst)), GPIO_RZG3S_IRQ_CONFIGURE, (), inst)        \
	}

#define GPIO_RZG3S_FILL_ONE_P_IRQ(n, inst)  DT_INST_IRQ_BY_IDX(inst, n, irq),
#define GPIO_RZG3S_FILL_ALL_P_IRQS(inst, n) LISTIFY(n, GPIO_RZG3S_FILL_ONE_P_IRQ, (), inst)

#define GPIO_RZG3S_CTRL_INIT(inst)                                                                 \
	GPIO_RZG3S_DECLARE_ALL_ISRS(inst, DT_NUM_IRQS(DT_DRV_INST(inst)))                          \
	GPIO_RZG3S_IRQ_CONFIG_FUNC(inst)                                                           \
	static const struct rzg3s_gpio_ctrl_config rzg3s_gpio_ctrl_cfg##inst = {                   \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(tint, DT_DRV_INST(inst)),                       \
		.pinctrl_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                \
		.parent_irqs = {GPIO_RZG3S_FILL_ALL_P_IRQS(inst, DT_NUM_IRQS(DT_DRV_INST(inst)))}, \
		.irq_config = rzg3s_gpio_tint_cfg_func_##inst,                                     \
	};                                                                                         \
	static struct rzg3s_gpio_ctrl_data rzg3s_gpio_ctrl_data##inst = {};                        \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, rzg3s_gpio_ctrl_init, NULL, &rzg3s_gpio_ctrl_data##inst,       \
			      &rzg3s_gpio_ctrl_cfg##inst, POST_KERNEL,                             \
			      CONFIG_GPIO_RZG3S_INIT_PRIORITY, NULL);                              \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, GPIO_RZG3S_PORT_INIT);

DT_INST_FOREACH_STATUS_OKAY(GPIO_RZG3S_CTRL_INIT)
