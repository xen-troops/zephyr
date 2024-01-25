/*
 * Copyright (C) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rzg_wdt

/*
 * *************************** INCLUDES *****************************
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

/*
 * *************************** CONSTANTS *****************************
 */
/* WDT registers */
#define WDTCNT				0x00
#define WDTSET				0x04
#define WDTTIM				0x08
#define WDTINT				0x0C
#define PECR				0x10
#define PEEN				0x14

/* WDT Register bits */
#define WDTCNT_WDTEN			BIT(0)
#define WDTINT_INTDISP			BIT(0)
#define PEEN_FORCE			BIT(0)

/* System controller registers */
#define SYS_WDT1_CTRL			0x260
#define SYS_WDT2_CTRL			0x270

/* System controller register bits */
#define WDTN_CTRL_WDTSTOPMASK		BIT(16)

/* CPG WDT registers */
#define CPG_WDTOVF_RST			0x0
#define CPG_WDTRST_SEL			0x4

/* CPG WDT register bits */
#define CPG_WDTRST_SEL1			BIT(1)
#define CPG_WDTRST_SEL2			BIT(2)
#define CPG_WDTRST_SEL9			BIT(9)
#define CPG_WDTRST_SEL10		BIT(10)
#define CPG_WDTRST_SEL1_WEN		BIT(17)
#define CPG_WDTRST_SEL2_WEN		BIT(18)
#define CPG_WDTRST_SEL9_WEN		BIT(25)
#define CPG_WDTRST_SEL10_WEN		BIT(26)

/* Other constants */
#define WDTSET_COUNTER_VAL(f)		((f) << 20)

/*
 * **************************** MACROS *******************************
 */
LOG_MODULE_REGISTER(wdt_rzg, CONFIG_WDT_LOG_LEVEL);

#define DEV_DATA(dev)			((struct wdt_rzg_data *)((dev)->data))
#define DEV_CFG(dev)			((struct wdt_rzg_config *)((dev)->config))

#define RZG_WDT_REG(dev)		DEVICE_MMIO_NAMED_GET(dev, wdt_reg)
#define RZG_CPG_REG(dev)		DEVICE_MMIO_NAMED_GET(dev, cpg_reg)

/*
 * *********************** TYPE DECLARATIONS *************************
 */
enum wdt_timer_idx {
	WDT1_IDX = 1,
	WDT2_IDX = 2
};

struct wdt_rzg_config {
	/* Must be first */
	DEVICE_MMIO_NAMED_ROM(wdt_reg);
	DEVICE_MMIO_NAMED_ROM(cpg_reg);
	/* pointer to clock device for WDT */
	const struct device *cpg;
	/* pointer to IRQ configuration function */
	void (*irq_config)(void);
	/* WDT clocks */
	struct renesas_cpg_clk pclk;
	struct renesas_cpg_clk oscclk;
	/* reset controller */
	struct reset_dt_spec wdt_rstn;
	/* pointer to system controller */
	const struct device *syscon;
	/* timer index */
	const enum wdt_timer_idx timer_idx;
	/* WDT IRQ number */
	const uint8_t irq_num;
};

struct wdt_rzg_data {
	/* Must be first */
	DEVICE_MMIO_NAMED_RAM(wdt_reg);
	DEVICE_MMIO_NAMED_RAM(cpg_reg);
	uint32_t clk_rate;
	/* user callback */
	wdt_callback_t callback;
};

/*
 * ************************* PRIVATE FUNCTIONS ***************************
 */

/**
 * @brief Function calculate WDTTIME value for WDTSET_n register
 *
 * @param wdt_clk - Watchdog clock rate in Hz
 * @param wdt_cycle_ms - Required watchdog cycle time in ms
 *
 * @return WDTTIME value. If value < 0 or > 0xFFF this means an error
 */
static int32_t wdt_rzg_get_wdttime(uint32_t wdt_clk, uint32_t wdt_cycle_ms)
{
	return (wdt_clk / 1000) * wdt_cycle_ms / (1024 * 1024) - 1;
}

/**
 * @brief Function performs write value to RZ/G WDT register
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param value - Value to write
 * @param reg - WDT register offset
 *
 * @return none
 */
static void wdt_rzg_write(const struct device *dev, uint32_t value, uint32_t reg)
{
	sys_write32(value, RZG_WDT_REG(dev) + reg);

	/* Registers other than the WDTINT is always synchronized with WDT_CLK */
	if (reg != WDTINT) {
		k_busy_wait(1);
	}
}

/**
 * @brief Function performs write value to RZ/G CPG register related to WDT
 *	  Used for accessing to CPG_WDTOVF_RST and CPG_WDTRST_SEL registers
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param value - Value to write
 * @param reg - CPG register offset (start from CPG_WDTOVF_RST)
 *
 * @return none
 */
static void wdt_cpg_rzg_write(const struct device *dev, uint32_t value, uint32_t reg)
{
	sys_write32(value, RZG_CPG_REG(dev) + reg);
}

/**
 * @brief Function performs read value from RZ/G CPG register related to WDT
 *	  Used for accessing to CPG_WDTOVF_RST and CPG_WDTRST_SEL registers
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param reg - CPG register offset (start from CPG_WDTOVF_RST)
 *
 * @return Read value
 */
static uint32_t wdt_cpg_rzg_read(const struct device *dev, uint32_t reg)
{
	return sys_read32(RZG_CPG_REG(dev) + reg);
}

/**
 * @brief Function controls the stop function of WDT
 *	  from CoreSight during debugging
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param on - if true, stops WDT counting during debugging,
 *	       if false, not stop WDT counting during debugging
 *
 * @return none
 */
static void wdt_rzg_set_stop_in_dbg(const struct device *dev, bool on)
{
	const struct wdt_rzg_config *config = dev->config;
	const struct device *syscon = config->syscon;
	uint32_t reg, val;

	/* We use timer index to identify wdt1 or wdt2 */
	switch (config->timer_idx) {
	case WDT1_IDX:
		reg = SYS_WDT1_CTRL;
		break;

	case WDT2_IDX:
		reg = SYS_WDT2_CTRL;
		break;

	default:
		LOG_ERR("%s: wrong index: %d", dev->name, config->timer_idx);
		return;
	}

	syscon_read_reg(syscon, reg, &val);
	/* WDTSTOPMASK must be 0 for stops WDT during debugging */
	if (on) {
		syscon_write_reg(syscon, reg, (val & ~WDTN_CTRL_WDTSTOPMASK));
	} else {
		syscon_write_reg(syscon, reg, (val | WDTN_CTRL_WDTSTOPMASK));
	}
}

/**
 * @brief Function enable or disable entire system reset from WDT
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param on - if true, system reset from this WDT enabled
 *	       if false, system reset from this WDT disabled
 *
 * @return none
 */
static void wdt_rzg_set_system_reset(const struct device *dev, bool on)
{
	const struct wdt_rzg_config *config = dev->config;
	uint32_t val = wdt_cpg_rzg_read(dev, CPG_WDTRST_SEL);

	/* We use timer index to identify wdt1 or wdt2 */
	switch (config->timer_idx) {
	case WDT1_IDX:
		val |= CPG_WDTRST_SEL1_WEN;
		val = (on) ? (val | CPG_WDTRST_SEL1) : (val & ~CPG_WDTRST_SEL1);
		break;

	case WDT2_IDX:
		val |= CPG_WDTRST_SEL2_WEN;
		val = (on) ? (val | CPG_WDTRST_SEL2) : (val & ~CPG_WDTRST_SEL2);
		break;

	default:
		LOG_ERR("%s: wrong index: %d", dev->name, config->timer_idx);
		return;
	}

	wdt_cpg_rzg_write(dev, val, CPG_WDTRST_SEL);
}

/**
 * @brief Function enable or disable single core cold-reset from WDT
 *        WDT1 used to reset Cortex-M33, WDT2 used to reset Cortex-M33_FPU
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param on - if true, corresponding core reset from this WDT enabled
 *	       if false, corresponding core reset from this WDT disabled
 *
 * @return none
 */
static void wdt_rzg_set_core_reset(const struct device *dev, bool on)
{
	const struct wdt_rzg_config *config = dev->config;
	uint32_t val = wdt_cpg_rzg_read(dev, CPG_WDTRST_SEL);

	/* We use timer index to identify wdt1 or wdt2 */
	switch (config->timer_idx) {
	case WDT1_IDX:
		val |= CPG_WDTRST_SEL9_WEN;
		val = (on) ? (val | CPG_WDTRST_SEL9) : (val & ~CPG_WDTRST_SEL9);
		break;

	case WDT2_IDX:
		val |= CPG_WDTRST_SEL10_WEN;
		val = (on) ? (val | CPG_WDTRST_SEL10) : (val & ~CPG_WDTRST_SEL10);
		break;

	default:
		LOG_ERR("%s: wrong index: %d", dev->name, config->timer_idx);
		return;
	}

	wdt_cpg_rzg_write(dev, val, CPG_WDTRST_SEL);
}

static int wdt_rzg_disable(const struct device *dev)
{
	const struct wdt_rzg_config *config = dev->config;

	irq_disable(config->irq_num);

	/* We have no other way to stop the WDT other than resetting it */
	return reset_line_toggle_dt(&config->wdt_rstn);
}

static int wdt_rzg_setup(const struct device *dev, uint8_t options)
{
	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		LOG_ERR("%s: pause in sleep not supported", dev->name);
		return -ENOTSUP;
	}

	/* TODO: Figure out why WDT doesn't stop during debugging! */
	wdt_rzg_set_stop_in_dbg(dev, !!(options & WDT_OPT_PAUSE_HALTED_BY_DBG));

	/* Reset watchdog counter register */
	wdt_rzg_write(dev, 0, WDTTIM);

	/* Enable watchdog timer*/
	wdt_rzg_write(dev, WDTCNT_WDTEN, WDTCNT);

	return 0;
}

/**
 * @brief This function must be called periodically to avoid WDT overflow
 *	  and system reset
 *
 * @param dev - Pointer to RZ/G watchdog device
 * @param channel_id - Not used
 * @param reg - 0 on success
 *
 * @return none
 */
static int wdt_rzg_feed(const struct device *dev, int channel_id)
{
	struct wdt_rzg_data *data = dev->data;
	const struct wdt_rzg_config *config = dev->config;

	/* Clear overflow interrupt */
	wdt_rzg_write(dev, WDTINT_INTDISP, WDTINT);

	if (data->callback) {
		/* Re-enable interrupts, if we use callback */
		irq_enable(config->irq_num);
	}

	return 0;
}

static int wdt_rzg_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *timeout)
{
	struct wdt_rzg_data *data = dev->data;
	const struct wdt_rzg_config *config = dev->config;
	int32_t wdttime;

#ifdef CONFIG_WDT_MULTISTAGE
	if (timeout->next) {
		LOG_ERR("%s: multistaging isn't supported", dev->name);
		return -ENOTSUP;
	}
#endif

	if (!timeout->window.max || timeout->window.min > timeout->window.max) {
		LOG_ERR("%s: invalid timeout val(s) min %u max %u",
			dev->name, timeout->window.min, timeout->window.max);
		return -EINVAL;
	}

	data->callback = timeout->callback;

	wdt_rzg_disable(dev);

	/* Set mode of watchdog and callback */
	wdt_rzg_set_system_reset(dev, false);
	wdt_rzg_set_core_reset(dev, false);
	switch (timeout->flags) {
	case WDT_FLAG_RESET_SOC:
		wdt_rzg_set_system_reset(dev, true);
		LOG_DBG("%s: configuring reset SOC mode", dev->name);
		break;
	case WDT_FLAG_RESET_CPU_CORE:
		wdt_rzg_set_core_reset(dev, true);
		LOG_DBG("%s: configuring reset single core mode", dev->name);
		break;
	case WDT_FLAG_RESET_NONE:
		LOG_DBG("%s: configuring non-reset mode", dev->name);
		break;
	default:
		LOG_ERR("%s: unsupported watchdog config flag (0x%08x)", dev->name, timeout->flags);
		return -EINVAL;
	}

	if (data->callback) {
		/* Enable interrupts, only if we use callback */
		irq_enable(config->irq_num);
	}

	/* 2 consecutive overflow cycle needed to trigger reset */
	wdttime = wdt_rzg_get_wdttime(data->clk_rate, timeout->window.max / 2);
	if (wdttime < 0 || wdttime > 0xFFF) {
		LOG_ERR("%s: The requested timeout %u is outside the possible limits",
			dev->name, timeout->window.max);
		return -EINVAL;
	}
	wdt_rzg_write(dev, WDTSET_COUNTER_VAL(wdttime), WDTSET);

	return 0;
}

static int wdt_rzg_init(const struct device *dev)
{
	const struct wdt_rzg_config *config = dev->config;
	struct wdt_rzg_data *data = dev->data;
	const struct device *cpg = config->cpg;
	int ret;

	if (!device_is_ready(cpg)) {
		LOG_ERR("%s: error cpg isn't ready", dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(cpg, (clock_control_subsys_t)&config->pclk);
	if (ret != 0) {
		LOG_ERR("Can't turn on pclk clock for %s\n", dev->name);
		return ret;
	}

	ret = clock_control_on(cpg, (clock_control_subsys_t)&config->oscclk);
	if (ret != 0) {
		LOG_ERR("Can't turn on oscclk clock for %s\n", dev->name);
		goto err;
	}

	/* We don't expect that someone would change the rate after initializing the CPG driver */
	ret = clock_control_get_rate(cpg, (clock_control_subsys_t)&config->oscclk,
				     &data->clk_rate);
	if (ret < 0) {
		LOG_ERR("Can't get clock rate for %s\n", dev->name);
		goto err;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* Reset device */
	(void)reset_line_toggle_dt(&config->wdt_rstn);

	config->irq_config();

	return 0;

err:
	/* On error */
	clock_control_off(cpg, (clock_control_subsys_t)&config->oscclk);
	clock_control_off(cpg, (clock_control_subsys_t)&config->pclk);
	return ret;
}

static void wdt_rgz_isr(const struct device *dev)
{
	struct wdt_rzg_data *data = dev->data;
	const struct wdt_rzg_config *config = dev->config;

	LOG_DBG("%s: wdtr irq", dev->name);
	/* Disable interrupts to avoid IRQ storm */
	irq_disable(config->irq_num);

	if (data->callback) {
		data->callback(dev, 0);
	}
}

/*
 * ************************* DRIVER INSTANTIATION SECTION ***************************
 */
static const struct wdt_driver_api wdt_rzg_api = {
	.setup            = wdt_rzg_setup,
	.disable          = wdt_rzg_disable,
	.install_timeout  = wdt_rzg_install_timeout,
	.feed             = wdt_rzg_feed,
};

#define WDT_RGZ_IRQ_CONFIG_FUNC(n)							\
	static void wdt_rgz_irq_config_##n(void)					\
	{										\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, wdt, irq),				\
			    DT_INST_IRQ_BY_NAME(n, wdt, priority),			\
			    wdt_rgz_isr,						\
			    DEVICE_DT_INST_GET(n), 0);					\
	}

#define WDT_RZG_DEFINE(n)								\
	WDT_RGZ_IRQ_CONFIG_FUNC(n)							\
	static const struct wdt_rzg_config wdt_rzg_config##n = {			\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(wdt_reg, DT_DRV_INST(n)),		\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(cpg_reg, DT_DRV_INST(n)),		\
		.cpg = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),				\
		.irq_config = wdt_rgz_irq_config_##n,					\
		.oscclk.module = DT_INST_CLOCKS_CELL_BY_NAME(n, oscclk, module),	\
		.oscclk.domain = DT_INST_CLOCKS_CELL_BY_NAME(n, oscclk, domain),	\
		.pclk.module = DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, module),		\
		.pclk.domain = DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, domain),		\
		.wdt_rstn = RESET_DT_SPEC_INST_GET_BY_IDX(n, 0),			\
		.syscon = DEVICE_DT_GET(DT_NODELABEL(syscon)),				\
		.timer_idx = DT_INST_PROP(n, timer_idx),				\
		.irq_num = DT_INST_IRQN(n),						\
	};										\
											\
	static struct wdt_rzg_data wdt_rzg_data##n;					\
	DEVICE_DT_INST_DEFINE(n, &wdt_rzg_init, NULL, &wdt_rzg_data##n,			\
			      &wdt_rzg_config##n, POST_KERNEL,				\
			      CONFIG_WDT_RZ_INIT_PRIORITY,				\
			      &wdt_rzg_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_RZG_DEFINE)
