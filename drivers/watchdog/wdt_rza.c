/*
 * Copyright (C) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rza_wdt

#include <string.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_rz, CONFIG_WDT_LOG_LEVEL);

/* Watchdog timer control/status register */
#define RZ_WTCSR_OFFSET   0x0
#define RZ_WTCSR_CSK_MASK BIT_MASK(4) /* clock Select mask */
#define RZ_WTCSR_TME      BIT(5)      /* enable timer */
#define RZ_WTCSR_TMM_WT   BIT(6)      /* timer mode watchdog, otherwise interval */

#define RZ_WTCSR_GEN_WRITE_VAL(val) (0xa500 | ((val) & 0xff))

/* Watchdog timer counter */
#define RZ_WTCNT_OFFSET 0x2
#define RZ_WTCNT_GEN_WRITE_VAL(val) (0x5a00 | ((val) & 0xff))
#define RZ_WTCNT_MAX_CNT (255)

/* Watchdog reset control/status register */
#define RZ_WRCSR_OFFSET 0x4

#define RZ_WRCSR_RSTE BIT(6) /* Reset Enable */
#define RZ_WRCSR_WOVF BIT(7) /* Watchdog Timer Overflow */

#define RZ_WRCSR_GEN_RSTE_VAL(en_res) (0x5a1f | ((en_res) ? RZ_WRCSR_RSTE : 0))
#define RZ_WRCSR_WOVF_CLEAR_VAL 0xa500

struct wdt_rz_config {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *cpg;
	const struct pinctrl_dev_config *pcfg;
	struct renesas_cpg_clk wdt_clk;
};

struct wdt_rz_data {
	DEVICE_MMIO_RAM; /* Must be first */
	uint32_t clk_rate;
	uint8_t timer_cnt_on_feed;
};

static int wdt_rz_disable(const struct device *dev)
{
	uint8_t wtcsr = sys_read8(DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);

	wtcsr &= ~RZ_WTCSR_TME;
	sys_write16(RZ_WTCSR_GEN_WRITE_VAL(wtcsr), DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);
	return 0;
}

static int wdt_rz_setup(const struct device *dev, uint8_t options)
{
#ifdef CONFIG_PINCTRL
	const struct wdt_rz_config *config = dev->config;
#endif
	uint8_t wtcsr;

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		LOG_ERR("%s: pause in sleep not supported", dev->name);
		return -ENOTSUP;
	}

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		LOG_ERR("%s: pause when halted by debugger not supported", dev->name);
		return -ENOTSUP;
	}

	wdt_rz_disable(dev);

#ifdef CONFIG_PINCTRL
	if (config->pcfg != NULL && pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
		LOG_ERR("%s: error can't apply pinctrl state", dev->name);
		return -EINVAL;
	}
#endif

	/* clear overflow bit, dummy read is needed for clearing WOVF bit */
	sys_read8(DEVICE_MMIO_GET(dev) + RZ_WRCSR_OFFSET);
	sys_write16(RZ_WRCSR_WOVF_CLEAR_VAL, DEVICE_MMIO_GET(dev) + RZ_WRCSR_OFFSET);

	/* start watchdog timer */
	wtcsr = sys_read8(DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);
	wtcsr |= RZ_WTCSR_TME;
	sys_write16(RZ_WTCSR_GEN_WRITE_VAL(wtcsr), DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);
	return 0;
}

static const uint32_t div_table[] = {
	1,      64,      128,     256,
	512,    1024,    4096,    16384,
	32768,  65536,   131072,  262144,
	524288, 1048576, 2097152, 4194304
};

static int wdt_rz_feed(const struct device *dev, int channel_id)
{
	struct wdt_rz_data *data = dev->data;
	uint16_t cnt = RZ_WTCNT_GEN_WRITE_VAL(data->timer_cnt_on_feed);

	sys_write16(cnt, DEVICE_MMIO_GET(dev) + RZ_WTCNT_OFFSET);
	return 0;
}

/* we don't need to install callback, because irq isn't present for watchdog timer mode */
static int wdt_rz_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *timeout)
{
	struct wdt_rz_data *data = dev->data;
	bool reset_lsi = false;
	uint8_t min_possible_div;
	uint64_t max_delay_us, req_max_us = timeout->window.max * 1000ULL;
	uint32_t one_cnt_tick_us;
	uint16_t cnt_ticks, wtcsr;

#ifdef CONFIG_WDT_MULTISTAGE
	if (timeout->next) {
		LOG_ERR("%s: multistaging isn't supported", dev->name);
		return -ENOTSUP;
	}
#endif

	if (timeout->window.min > timeout->window.max) {
		LOG_ERR("%s: invalid timeout val(s) min %u max %u",
			dev->name, timeout->window.min, timeout->window.max);
		return -EINVAL;
	}

	/* Set mode of watchdog and callback */
	switch (timeout->flags) {
	case WDT_FLAG_RESET_SOC:
		reset_lsi = true;
		LOG_DBG("%s: configuring reset SOC mode", dev->name);
		break;
	case WDT_FLAG_RESET_NONE:
		LOG_DBG("%s: configuring non-reset mode", dev->name);
		break;
	default:
		LOG_ERR("%s: unsupported watchdog config flag (0x%08x)", dev->name, timeout->flags);
		return -EINVAL;
	}

	/* find minimal possible divider */
	for (min_possible_div = 0; min_possible_div < ARRAY_SIZE(div_table); min_possible_div++) {
		max_delay_us = div_table[min_possible_div] * (RZ_WTCNT_MAX_CNT * 1000000ULL);

		max_delay_us /= data->clk_rate;
		if (req_max_us <= max_delay_us) {
			break;
		}
	}

	if (min_possible_div == ARRAY_SIZE(div_table)) {
		LOG_ERR("%s: exceed maximum possible timeout (%u) requested %u",
			dev->name, (uint32_t)(max_delay_us / 1000), timeout->window.max);
		return -EINVAL;
	}

	/* calculate number of counts with the chosen clk divider */
	one_cnt_tick_us = (div_table[min_possible_div] * 1000000ULL) / data->clk_rate;
	cnt_ticks = DIV_ROUND_UP(timeout->window.max * 1000, one_cnt_tick_us);

	/* shouldn't never happen */
	if (cnt_ticks > RZ_WTCNT_MAX_CNT) {
		cnt_ticks = RZ_WTCNT_MAX_CNT;
		LOG_WRN("%s: cnt_ticks greagter %u after calculation", dev->name, RZ_WTCNT_MAX_CNT);
	}

	data->timer_cnt_on_feed = RZ_WTCNT_MAX_CNT - cnt_ticks;

	/* set divider */
	wtcsr = sys_read8(DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);
	wtcsr &= ~RZ_WTCSR_CSK_MASK;
	wtcsr = RZ_WTCSR_GEN_WRITE_VAL(wtcsr | min_possible_div);
	sys_write16(wtcsr, DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);

	wdt_rz_feed(dev, 0);
	sys_write16(RZ_WRCSR_GEN_RSTE_VAL(reset_lsi), DEVICE_MMIO_GET(dev) + RZ_WRCSR_OFFSET);
	return 0;
}

static const struct wdt_driver_api wdt_rz_api = {
	.setup            = wdt_rz_setup,
	.disable          = wdt_rz_disable,
	.install_timeout  = wdt_rz_install_timeout,
	.feed             = wdt_rz_feed,
};

static int wdt_rz_init(const struct device *dev)
{
	const struct wdt_rz_config *config = dev->config;
	struct wdt_rz_data *data = dev->data;
	const struct device *cpg = config->cpg;
	uint16_t wtcsr;
	int ret;

	if (!device_is_ready(cpg)) {
		LOG_ERR("%s: error cpg isn't ready", dev->name);
		return -ENODEV;
	}

	/* We don't expect that someone would change the rate after initializing the CPG driver */
	ret = clock_control_get_rate(cpg, (clock_control_subsys_t)&config->wdt_clk,
				     &data->clk_rate);
	if (ret < 0) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

#ifdef CONFIG_WDT_DISABLE_AT_BOOT
	wdt_rz_disable(dev);
#endif

	/* setup wdt timer in watchdog mode */
	wtcsr = RZ_WTCSR_GEN_WRITE_VAL(RZ_WTCSR_TMM_WT);
	sys_write16(wtcsr, DEVICE_MMIO_GET(dev) + RZ_WTCSR_OFFSET);

	return 0;
}

#ifdef CONFIG_PINCTRL
#define RZ_PINCTRL_DT_INST_DEFINE(n) \
	COND_CODE_1(DT_INST_NUM_PINCTRL_STATES(n), (PINCTRL_DT_INST_DEFINE(n);), (EMPTY))
#define RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n) \
	COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), NULL)
#else
#define RZ_PINCTRL_DT_INST_DEFINE(n)
#define RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n) NULL
#endif

#define WDT_RZ_DEFINE(n)                                                                           \
	RZ_PINCTRL_DT_INST_DEFINE(n);                                                              \
	static const struct wdt_rz_config wdt_rz_config##n = {                                     \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.cpg = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                      \
		.wdt_clk.module = DT_INST_CLOCKS_CELL(n, module),                                  \
		.wdt_clk.domain = DT_INST_CLOCKS_CELL(n, domain),                                  \
		.pcfg = RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
	};                                                                                         \
	                                                                                           \
	static struct wdt_rz_data wdt_rz_data##n;                                                  \
	DEVICE_DT_INST_DEFINE(n, &wdt_rz_init, NULL, &wdt_rz_data##n,                              \
			      &wdt_rz_config##n, POST_KERNEL,                                      \
			      CONFIG_WDT_RZ_INIT_PRIORITY,                                         \
			      &wdt_rz_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_RZ_DEFINE)
