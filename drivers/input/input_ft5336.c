/*
 * Copyright (c) 2020,2023 NXP
 * Copyright (c) 2020 Mark Olsson <mark@markolsson.se>
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT focaltech_ft5336

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ft5336, CONFIG_INPUT_LOG_LEVEL);

#include <zephyr/input/ft5336.h>

#define VENDOR_ID		0x79

/* FT5336 used registers */
#define REG_DEVICE_MODE		0x00
#define REG_GEST_ID		0x01
#define REG_TD_STATUS		0x02

/* Base address for touch registers
 * Sequence of registers:
 * Xh, Xl, Yh, Yl, weight, misc
 */
#define REG_TOUCH		0x03

#define REG_ID_G_THGROUP	0x80
#define REG_ID_G_CTRL		0x86
#define REG_ID_G_PERIODACTIVE	0x88
#define REG_ID_G_AUTO_CLB_MODE	0xa0
#define REG_ID_G_LIB_VERSION	0xa1
#define REG_ID_G_CIPHER		0xa3
#define REG_ID_G_MODE		0xa4
#define REG_ID_G_PMODE		0xa5
#define REG_ID_G_FIRMID		0xa6
#define REG_ID_G_VENDORID	0xa8

/* REG_TD_STATUS: Touch points. */
#define TOUCH_POINTS_POS	0U
#define TOUCH_POINTS_MSK	0x0F

/* REG_Pn_XH: Events. */
#define EVENT_POS		6U
#define EVENT_MSK		0x03

/* Touch events */
#define TOUCH_EVENT(x)		(((x) >> 6) & 0x3)
#define TOUCH_ID(x)		(((x) >> 4) & 0xf)
#define TOUCH_AREA(x)		(((x) >> 4) & 0xf)
#define TOUCH_DIR(x)		(((x) >> 2) & 0x3)
#define TOUCH_SPEED(x)		((x) & 0x3)
#define COORD_X_MASK		0x3ff
#define COORD_Y_MASK		0xFff
#define EVENT_PRESS_DOWN	0x00
#define EVENT_LIFT_UP		0x01
#define EVENT_CONTACT		0x02
#define EVENT_NONE		0x03

/* REG_Pn_XH: Position */
#define POSITION_H_MSK		0x0FU

/** FT5336 configuration (DT). */
struct ft5336_config {
	/** I2C bus. */
	struct i2c_dt_spec bus;
	struct gpio_dt_spec reset_gpio;
	int tag;
#ifdef CONFIG_INPUT_FT5336_INTERRUPT
#ifdef CONFIG_INPUT_FT5335_GPIO_INTERRUPT
	/** Interrupt GPIO information. */
	struct gpio_dt_spec int_gpio;
#else
	const struct pinctrl_dev_config *pcfg;
	int irq_num;
	int irq_prio;
	int irq_flags;
#endif
#endif
};

/** FT5336 data. */
struct ft5336_data {
	/** Device pointer. */
	const struct device *dev;
	/** Work queue (for deferred read). */
	struct k_work work;
#ifdef CONFIG_INPUT_FT5336_INTERRUPT
	/** Interrupt GPIO callback. */
	struct gpio_callback int_gpio_cb;
#else
	/** Timer (polling mode). */
	struct k_timer timer;
#endif
	/** Last pressed state. */
	bool pressed_old;
};

static void ft5336_process(const struct device *dev)
{
	const struct ft5336_config *config = dev->config;
	struct ft5336_data *data = dev->data;
	int r;
	uint8_t points;
	uint8_t gest;
	uint8_t coords[6];
	uint8_t evt, touch_id, weight, area, dir, speed;
	uint16_t x, y;
	bool pressed;

	r = i2c_reg_read_byte_dt(&config->bus, REG_TD_STATUS, &points);
	if (r < 0) {
		return;
	}

	points = (points >> TOUCH_POINTS_POS) & TOUCH_POINTS_MSK;
	if (!points) {
		return;
	}

	for (int finger = 0; finger < points; finger++) {
		r = i2c_burst_read_dt(&config->bus, REG_TOUCH + finger * 6, coords, sizeof(coords));
		if (r < 0) {
			return;
		}
		x = ((coords[0] << 8) | coords[1]) & COORD_X_MASK;
		y = ((coords[2] << 8) | coords[3]) & COORD_Y_MASK;
		evt = TOUCH_EVENT(coords[0]);
		touch_id = TOUCH_ID(coords[3]);
		weight = coords[4];
		area = TOUCH_AREA(coords[5]);
		dir = TOUCH_DIR(coords[5]);
		speed = TOUCH_SPEED(coords[5]);
		pressed = (evt == EVENT_PRESS_DOWN) || (evt == EVENT_CONTACT);

		if (pressed) {
			input_report_abs(dev, INPUT_ABS_X, x, false, K_FOREVER);
			input_report_abs(dev, INPUT_ABS_Y, y, false, K_FOREVER);
			input_report(dev, INPUT_EV_MSC, INPUT_MSC_WEIGHT, weight, false, K_FOREVER);
			input_report(dev, INPUT_EV_MSC, INPUT_MSC_DIRECTION, dir, false, K_FOREVER);
			input_report(dev, INPUT_EV_MSC, INPUT_MSC_AREA, area, false, K_FOREVER);
			input_report(dev, INPUT_EV_MSC, INPUT_MSC_SPEED, speed, false, K_FOREVER);
			input_report(dev, INPUT_EV_KEY, INPUT_BTN_TOUCH,
				     0x10 | finger, true, K_FOREVER);
		} else if (data->pressed_old && !pressed) {
			input_report(dev, INPUT_EV_KEY, INPUT_BTN_TOUCH, finger, true, K_FOREVER);
		}
		data->pressed_old = pressed;
	}

	r = i2c_reg_read_byte_dt(&config->bus, REG_GEST_ID, &gest);
	if (r < 0) {
		return;
	}
	if (gest) {
		input_report(dev, INPUT_EV_MSC, INPUT_MSC_GESTURE, gest, false, K_FOREVER);
	}
}

static void ft5336_work_handler(struct k_work *work)
{
	struct ft5336_data *data = CONTAINER_OF(work, struct ft5336_data, work);

	ft5336_process(data->dev);
}

#ifdef CONFIG_INPUT_FT5336_INTERRUPT
static void ft5336_isr_handler(const void *param)
{
	const struct device *dev = (const struct device *)param;
	struct ft5336_data *data = (struct ft5336_data *)dev->data;

	k_work_submit(&data->work);
}
#else
static void ft5336_timer_handler(struct k_timer *timer)
{
	struct ft5336_data *data = CONTAINER_OF(timer, struct ft5336_data, timer);

	k_work_submit(&data->work);
}
#endif

static int check_vendor(const struct device *dev)
{
	const struct ft5336_config *config = dev->config;
	uint8_t id;
	int r;

	r = i2c_reg_read_byte_dt(&config->bus, REG_ID_G_VENDORID, &id);
	if (r) {
		return -EIO;
	}

	return (id == VENDOR_ID) ? 0 : -1;
}

static int ft5336_init(const struct device *dev)
{
	const struct ft5336_config *config = dev->config;
	struct ft5336_data *data = dev->data;
	int r;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C controller device not ready");
		return -ENODEV;
	}

	data->dev = dev;

	k_work_init(&data->work, ft5336_work_handler);

	if (check_vendor(dev)) {
		return -ENODEV;
	}
	/* normal operating mode */
	r = i2c_reg_write_byte_dt(&config->bus, REG_DEVICE_MODE, 0);
	if (r) {
		return -EIO;
	}
	/* trigger or polling mode */
#ifdef CONFIG_INPUT_FT5336_INTERRUPT
	r = i2c_reg_write_byte_dt(&config->bus, REG_ID_G_MODE, 1);
#else
	r = i2c_reg_write_byte_dt(&config->bus, REG_ID_G_MODE, 0);
#endif
	if (r) {
		return -EIO;
	}
	/* touching detect threshold */
	r = i2c_reg_write_byte_dt(&config->bus, REG_ID_G_THGROUP, 22);
	if (r) {
		return -EIO;
	}
	/* period of active status */
	r = i2c_reg_write_byte_dt(&config->bus, REG_ID_G_PERIODACTIVE, 14);
	if (r) {
		return -EIO;
	}
	/* power consumption mode */
	r = i2c_reg_write_byte_dt(&config->bus, REG_ID_G_PMODE, 0);
	if (r) {
		return -EIO;
	}
	/* enable auto calibration */
	r = i2c_reg_write_byte_dt(&config->bus, REG_ID_G_AUTO_CLB_MODE, 0);
	if (r) {
		return -EIO;
	}

	if (config->reset_gpio.port != NULL) {
		/* Enable reset GPIO, and pull down */
		r = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (r < 0) {
			LOG_ERR("Could not enable reset GPIO");
			return r;
		}
		/*
		 * Datasheet requires reset be held low 1 ms, or
		 * 1 ms + 100us if powering on controller. Hold low for
		 * 5 ms to be safe.
		 */
		k_sleep(K_MSEC(5));
		/* Pull reset pin high to complete reset sequence */
		r = gpio_pin_set_dt(&config->reset_gpio, 1);
		if (r < 0) {
			return r;
		}
	}

#ifdef CONFIG_INPUT_FT5336_INTERRUPT
#ifdef CONFIG_INPUT_FT5335_GPIO_INTERRUPT
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("Interrupt GPIO controller device not ready");
		return -ENODEV;
	}

	r = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin");
		return r;
	}
#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	r = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_MODE_ENABLE_ONLY);
#else
	r = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
#endif
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO interrupt.");
		return r;
	}

	gpio_init_callback(&data->int_gpio_cb, ft5336_isr_handler,
			   BIT(config->int_gpio.pin));
	r = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
	if (r < 0) {
		LOG_ERR("Could not set gpio callback");
		return r;
	}
#else
	/* Configure dt provided device signals when available */
	r = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (r < 0) {
		return r;
	}
	r = irq_connect_dynamic(config->irq_num, config->irq_prio, ft5336_isr_handler,
				dev, config->irq_flags);
	if (r < 0) {
		return r;
	}
	irq_enable(config->irq_num);
#endif
#else
	k_timer_init(&data->timer, ft5336_timer_handler, NULL);
	k_timer_start(&data->timer, K_MSEC(CONFIG_INPUT_FT5336_PERIOD),
		      K_MSEC(CONFIG_INPUT_FT5336_PERIOD));
#endif
	return 0;
}

#define IF_DISABLED(_flag, _code) COND_CODE_1(_flag, (), _code)

#ifdef CONFIG_INPUT_FT5336_INTERRUPT
#ifdef CONFIG_INPUT_FT5335_GPIO_INTERRUPT
#define INTERRUPT_DEF(index) \
			.int_gpio = GPIO_DT_SPEC_INST_GET(index, int_gpios),
#else
#define INTERRUPT_DEF(index) \
			.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
			.irq_num = DT_INST_IRQN(index),					\
			.irq_prio = COND_CODE_1(DT_INST_IRQ_HAS_CELL(index, priority),  \
						(DT_INST_IRQ(index, priority)), (0)),	\
			.irq_flags = DT_INST_IRQ(index, flags),
#endif
#endif

#define FT5336_INIT(index)								\
	IF_ENABLED(CONFIG_INPUT_FT5336_INTERRUPT, (					\
	  IF_DISABLED(CONFIG_INPUT_FT5335_GPIO_INTERRUPT, (				\
	    PINCTRL_DT_INST_DEFINE(index);))						\
	  )										\
	)										\
	static const struct ft5336_config ft5336_config_##index = {			\
		.bus = I2C_DT_SPEC_INST_GET(index),					\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(index, reset_gpios, {0}),	\
		IF_ENABLED(CONFIG_INPUT_FT5336_INTERRUPT, (INTERRUPT_DEF(index)))	\
	};										\
	static struct ft5336_data ft5336_data_##index;					\
	DEVICE_DT_INST_DEFINE(index, ft5336_init, NULL,					\
			      &ft5336_data_##index, &ft5336_config_##index,		\
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(FT5336_INIT)
