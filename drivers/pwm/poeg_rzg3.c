/*
 * Copyright (c) 2024, EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_rzg3s_poeg

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/drivers/poeg.h>
#ifdef CONFIG_RESET
#include <zephyr/drivers/reset.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(poeg_rzg3s, CONFIG_PWM_LOG_LEVEL);

#define LOG_DEV_ERR(dev, format, ...) LOG_ERR("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...) LOG_WRN("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...) LOG_INF("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...) LOG_DBG("%s:" #format, (dev)->name, ##__VA_ARGS__)

#define PIDF     BIT(0)
#define IOCF     BIT(1)
#define SSF      BIT(3)
#define PIDE     BIT(4)
#define IOCE     BIT(5)
#define ST       BIT(16)
#define INV      BIT(28)
#define NFEN     BIT(29)
#define NFCS_MSK BIT(31) | BIT(30)
#define NFCS_OFF (30)

#define POEG_STATUS_FLAGS (SSF | IOCF | PIDF)
#define POEG_INITIAL_CONF (NFEN | PIDE | IOCE)
struct poeg_rzg3_config {
	DEVICE_MMIO_ROM; /* Must be first */
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
#ifdef CONFIG_RESET
	const struct reset_dt_spec reset;
#endif
	void (*cfg_irqs)(void);
	int irq_line;
	int invert;
};

struct poeg_rzg3_data {
	DEVICE_MMIO_RAM; /* Must be first */
	void (*cb)(void *args);
	void *args;
};

#define DEV_DATA(dev) ((struct poeg_rzg3_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct poeg_rzg3_config *)((dev)->config))

static void poeg_rzg3_isr(struct device *dev)
{
	struct poeg_rzg3_data *data = DEV_DATA(dev);

	if (data->cb) {
		data->cb(data->args);
	}
}

static void poeg_rzg3_set_bits(const struct device *dev, uint32_t data)
{
	uint32_t reg32 = sys_read32(DEVICE_MMIO_GET(dev));

	sys_write32(reg32 | data, DEVICE_MMIO_GET(dev));
}

static void poeg_rzg3_clear_bits(const struct device *dev, uint32_t data)
{
	uint32_t reg32 = sys_read32(DEVICE_MMIO_GET(dev));

	sys_write32(reg32 & ~(data), DEVICE_MMIO_GET(dev));
}

void poeg_rzg3_reset(const struct device *dev)
{
	LOG_DEV_DBG(dev, "reset POEG device");
	poeg_rzg3_clear_bits(dev, POEG_STATUS_FLAGS);
}

uint32_t poeg_rzg3_status(const struct device *dev)
{
	return sys_read32(DEVICE_MMIO_GET(dev)) & POEG_STATUS_FLAGS;
}

void poeg_rzg3_cb_set(const struct device *dev, void (*cb)(void *), void *args)
{
	struct poeg_rzg3_data *data = DEV_DATA(dev);
	const struct poeg_rzg3_config *config = DEV_CFG(dev);

	data->cb = cb;
	data->args = args;

	if (cb) {
		irq_enable(config->irq_line);
	} else {
		irq_disable(config->irq_line);
	}
}

static int poeg_rzg3_init(const struct device *dev)
{
	int ret;
	const struct poeg_rzg3_config *config = DEV_CFG(dev);
	uint32_t poeg_config_reg = POEG_INITIAL_CONF;

#ifdef CONFIG_PINCTRL
	if (config->pcfg) {
		/* Configure dt provided device signals when available */
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "can't apply default pincontrol state");
			return ret;
		}
	}
#endif

	if (!device_is_ready(config->clock_dev)) {
		LOG_DEV_ERR(dev, "cpg isn't ready");
		return -ENODEV;
	}

#ifdef CONFIG_RESET
	if (device_is_ready(config->reset.dev)) {
		(void)reset_line_deassert_dt(&config->reset);
	}
#endif
	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "can't enable module clock, ret: %d", ret);
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	config->cfg_irqs();

	/* POEG Initial configuration valus can be set only once after reset */
	if (config->invert) {
		poeg_config_reg |= INV;
	}

	poeg_rzg3_set_bits(dev, poeg_config_reg);
	LOG_DEV_DBG(dev, "initialization finished");
	return 0;
}

static const struct poeg_driver_api poeg_rzg3_driver_api = {
	.reset = poeg_rzg3_reset,
	.cb_set = poeg_rzg3_cb_set,
	.status = poeg_rzg3_status,
};

#ifdef CONFIG_RESET
#define POEG_SET_RESET(n)                                                                          \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n),))
#else
#define POEG_SET_RESET(n)
#endif

#ifdef CONFIG_PINCTRL
#define POEG_PINCTRL_DT_INST_DEV_CONFIG_GET(n)                                                     \
	.pcfg = COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),    \
			    NULL)

#define POEG_PINCTRL_DT_INST_DEFINE(n)                                                             \
	COND_CODE_1(DT_INST_NUM_PINCTRL_STATES(n), (PINCTRL_DT_INST_DEFINE(n);), (EMPTY))
#else
#define POEG_PINCTRL_DT_INST_DEV_CONFIG_GET(n)
#define POEG_PINCTRL_DT_INST_DEFINE(n)
#endif

#define POEG_DEVICE_INIT_RZG3S(n)                                                                  \
	POEG_PINCTRL_DT_INST_DEFINE(n)                                                             \
                                                                                                   \
	static void config_isr_##n(void)                                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), poeg_rzg3_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	}                                                                                          \
                                                                                                   \
	static const struct poeg_rzg3_config poeg_rzg3_cfg_##n = {                                 \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		POEG_PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                            \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),                        \
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),                        \
		.cfg_irqs = config_isr_0,                                                          \
		.irq_line = DT_INST_IRQN(n),                                                       \
		.invert = DT_INST_PROP_OR(n, renesas_invert, 0),                                   \
		POEG_SET_RESET(n)};                                                                \
                                                                                                   \
	struct poeg_rzg3_data poeg_rzg3_data_##n;                                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &poeg_rzg3_init, NULL, &poeg_rzg3_data_##n, &poeg_rzg3_cfg_##n,   \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &poeg_rzg3_driver_api);

DT_INST_FOREACH_STATUS_OKAY(POEG_DEVICE_INIT_RZG3S)
