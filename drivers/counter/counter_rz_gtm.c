/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ostm

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#if DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045)
#include <zephyr/drivers/reset.h>
#else
#include <zephyr/drivers/interrupt_controller/gic.h>
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045) */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_rz_gtm, CONFIG_COUNTER_LOG_LEVEL);

#define LOG_DEV_ERR(dev, format, ...) LOG_ERR("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...) LOG_WRN("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...) LOG_INF("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...) LOG_DBG("%s:" #format, (dev)->name, ##__VA_ARGS__)

/*
 * GTM/OSTM counter can't support both alarms and changing top value in free-running mode,
 * so this drivers allows to use either alarms + free-running (counting up),
 * either changing top value + interval mode (counting down).
 * Therefore:
 * - driver starts with counter in free-running mode with alarms support by default
 * - set top is rejected if alarm is active
 * - set top causes counter to switch to interval mode and disables alarms
 * - set top to RZ_GTM_TIMER_TOP_VALUE switch counter back to free-running mode
 *   and enables alarms
 * Over all it's practical to use counter instance as either free-running with alarms or as
 * interval counter with supporting of changing top value, but not both.
 */

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045)
#define counter_rz_gtm_clear_pending(irq) NVIC_ClearPendingIRQ(irq)
#define counter_rz_gtm_set_pending(irq)   NVIC_SetPendingIRQ(irq)
#define counter_rz_gtm_is_pending(irq)    NVIC_GetPendingIRQ(irq)
#else
#define counter_rz_gtm_clear_pending(irq) arm_gic_irq_clear_pending(irq)
#define counter_rz_gtm_set_pending(irq)   arm_gic_irq_set_pending(irq)
#define counter_rz_gtm_is_pending(irq)    arm_gic_irq_is_pending(irq)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045) */

#define R_GTM_CMP 0x0  /* compare register */
#define R_GTM_CNT 0x4  /* counter register */
#define R_GTM_TE  0x10 /* count enable status register */
#define R_GTM_TS  0x14 /* count start trigger register */
#define R_GTM_TT  0x18 /* count stop trigger register */
#define R_GTM_CTL 0x20 /* control register */

/* count enable status register */
#define R_GTM_TE_ENABLED        BIT(0) /* timer enabled */
/* count start trigger register */
#define R_GTM_TS_START          BIT(0) /* trigger start of the timer */
/* count stop trigger register */
#define R_GTM_TT_STOP           BIT(0) /* trigger stop of the timer */
/* control register */
#define R_GTM_CTL_IRQ_START_ENABLE    BIT(0)
#define R_GTM_CTL_MODE_SHIFT    1
#define R_GTM_CTL_MODE_INTERVAL 0
#define R_GTM_CTL_MODE_FREERUN  BIT(1)

#define RZ_GTM_TIMER_TOP_VALUE	UINT32_MAX

struct counter_rz_gtm_config {
	struct counter_config_info info;

	DEVICE_MMIO_NAMED_ROM(cnt_regs);
	const struct device *clk_dev;
	struct renesas_cpg_clk clk_mod;
#if DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045)
	struct reset_dt_spec reset_z;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045) */
	uint8_t irqn;
	void (*irq_config)(const struct device *dev);
};

struct counter_rz_gtm_data {
	DEVICE_MMIO_NAMED_RAM(cnt_regs);
	/* top callback function */
	counter_top_callback_t top_cb;
	/* alarm callback function */
	counter_alarm_callback_t alarm_cb;
	void *user_data;
	uint32_t clk_freq;
	struct k_spinlock lock;
	uint32_t guard_period;
	uint32_t top_val;
	bool f_started:1;
	bool f_periodic:1;
};

#define DEV_DATA(dev) ((struct counter_rz_gtm_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct counter_rz_gtm_config *)((dev)->config))

static uint32_t counter_rz_gtm_read(const struct device *dev)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CNT);
}

static void counter_rz_gtm_stop_int(const struct device *dev)
{
	/* disable counter */
	sys_write8(R_GTM_TT_STOP, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_TT);

	while ((sys_read8(DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_TE) & R_GTM_TE_ENABLED) ==
	       R_GTM_TE_ENABLED)
		;
}

static void counter_rz_gtm_start_freerun(const struct device *dev)
{
	/* enable counter in free running mode */
	sys_write8(R_GTM_CTL_MODE_FREERUN, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CTL);

	sys_write8(R_GTM_TS_START, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_TS);
}

static void counter_rz_gtm_start_interval(const struct device *dev)
{
	struct counter_rz_gtm_data *ctx = dev->data;

	/* start timer in interval mode */
	sys_write32(ctx->top_val, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CMP);
	sys_write8(R_GTM_CTL_MODE_INTERVAL, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CTL);
	sys_write8(R_GTM_TS_START, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_TS);
}

static uint32_t counter_rz_gtm_get_freq(const struct device *dev)
{
	struct counter_rz_gtm_data *ctx = dev->data;

	return ctx->clk_freq;
}

static uint32_t counter_rz_gtm_get_top_value(const struct device *dev)
{
	struct counter_rz_gtm_data *ctx = dev->data;
	uint32_t top_val = RZ_GTM_TIMER_TOP_VALUE;

	if (ctx->f_periodic) {
		top_val = sys_read32(DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CMP);
	}

	return top_val;
}

static int counter_rz_gtm_set_top_value(const struct device *dev,
					const struct counter_top_cfg *top_cfg)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	k_spinlock_key_t key;
	uint32_t cur_tick;
	int ret = 0;

	if (!top_cfg) {
		LOG_DEV_ERR(dev, "top: invalid top value configuration");
		return -EINVAL;
	}

	/* top value cannot be updated if the alarm is active */
	if (ctx->alarm_cb) {
		LOG_DEV_ERR(dev, "top: value cannot be updated, alarm is active!");
		return -EBUSY;
	}

	key = k_spin_lock(&ctx->lock);

	if (!ctx->f_periodic && top_cfg->ticks == RZ_GTM_TIMER_TOP_VALUE) {
		goto exit_unlock;
	}

	if (top_cfg->ticks == RZ_GTM_TIMER_TOP_VALUE) {
		/* restore free running mode */
		counter_rz_gtm_stop_int(dev);
		irq_disable(cfg->irqn);
		counter_rz_gtm_clear_pending(cfg->irqn);
		ctx->top_cb = NULL;
		ctx->user_data = NULL;
		ctx->top_val = RZ_GTM_TIMER_TOP_VALUE;
		ctx->f_periodic = false;
		LOG_DEV_DBG(dev, "top: restore to free-running");

		if (ctx->f_started) {
			counter_rz_gtm_start_freerun(dev);
			counter_rz_gtm_clear_pending(cfg->irqn);
		}
		goto exit_unlock;
	}

	ctx->top_cb = top_cfg->callback;
	ctx->user_data = top_cfg->user_data;
	ctx->top_val = top_cfg->ticks;
	LOG_DEV_DBG(dev, "top: set periodic %08x irq:%d", top_cfg->ticks, !!(ctx->top_cb));

	if (!ctx->f_started) {
		ctx->f_periodic = true;
		goto exit_unlock;
	}

	if (!ctx->f_periodic) {
		/* switch to interval mode first time, restart timer */
		counter_rz_gtm_stop_int(dev);
		irq_disable(cfg->irqn);
		ctx->f_periodic = true;
		counter_rz_gtm_start_interval(dev);
		if (ctx->top_cb) {
			counter_rz_gtm_clear_pending(cfg->irqn);
			irq_enable(cfg->irqn);
		}
		LOG_DEV_DBG(dev, "top: restart timer");
		goto exit_unlock;
	}

	if (!ctx->top_cb) {
		/* new top cfg is without callback - stop IRQs */
		irq_disable(cfg->irqn);
		counter_rz_gtm_clear_pending(cfg->irqn);
	}

	/* timer already in interval mode - only change top value */
	sys_write32(ctx->top_val, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CMP);

	/* check if counter reset is required */
	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		/* Don't reset counter */
		cur_tick = counter_rz_gtm_read(dev);
		if (cur_tick >= ctx->top_val) {
			ret = -ETIME;
			LOG_DEV_DBG(dev, "top: late");
			if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
				/* Reset counter if current is late */
				sys_write8(R_GTM_TS_START,
					   DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_TS);
				LOG_DEV_DBG(dev, "top: late reset");
			}
		}
	} else {
		/* reset counter by setting TS */
		sys_write8(R_GTM_TS_START, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_TS);
		LOG_DEV_DBG(dev, "top: reset");
	}

	LOG_DEV_DBG(dev, "top: updated");

exit_unlock:
	k_spin_unlock(&ctx->lock, key);
	return ret;
}

static int counter_rz_gtm_get_value(const struct device *dev, uint32_t *ticks)
{
	if (ticks) {
		*ticks = counter_rz_gtm_read(dev);
	}

	return 0;
}

static uint32_t counter_rz_gtm_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_rz_gtm_data *ctx = dev->data;

	ARG_UNUSED(flags);
	return ctx->guard_period;
}

static int counter_rz_gtm_set_guard_period(const struct device *dev, uint32_t guard, uint32_t flags)
{
	struct counter_rz_gtm_data *ctx = dev->data;

	ARG_UNUSED(flags);
	__ASSERT_NO_MSG(guard < counter_rz_gtm_get_top_value(dev));

	ctx->guard_period = guard;
	LOG_DEV_DBG(dev, "guard_period: %08x", guard);

	return 0;
}

static int counter_rz_gtm_start(const struct device *dev)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&ctx->lock);

	if (ctx->f_started) {
		k_spin_unlock(&ctx->lock, key);
		return -EALREADY;
	}

	if (ctx->f_periodic) {
		counter_rz_gtm_start_interval(dev);
	} else {
		counter_rz_gtm_start_freerun(dev);
	}

	counter_rz_gtm_clear_pending(cfg->irqn);
	ctx->f_started = true;

	if (ctx->top_cb) {
		irq_enable(cfg->irqn);
	}

	k_spin_unlock(&ctx->lock, key);

	LOG_DEV_DBG(dev, "start");

	return 0;
}

static int counter_rz_gtm_stop(const struct device *dev)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&ctx->lock);

	if (!ctx->f_started) {
		k_spin_unlock(&ctx->lock, key);
		return 0;
	}

	/* disable counter */
	counter_rz_gtm_stop_int(dev);

	/* dis irq */
	irq_disable(cfg->irqn);
	counter_rz_gtm_clear_pending(cfg->irqn);

	ctx->top_cb = NULL;
	ctx->alarm_cb = NULL;
	ctx->user_data = NULL;

	ctx->f_started = false;

	k_spin_unlock(&ctx->lock, key);

	LOG_DEV_DBG(dev, "stop");

	return 0;
}

static int counter_rz_gtm_set_alarm(const struct device *dev, uint8_t chan,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	bool absolute = alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE;
	uint32_t val = alarm_cfg->ticks;
	k_spinlock_key_t key;
	bool irq_on_late;
	uint32_t max_rel_val;
	uint32_t now, diff;
	int err = 0;

	if (!alarm_cfg) {
		LOG_DEV_ERR(dev, "alarm: invalid configuration");
		return -EINVAL;
	}

	/* Alarm callback is mandatory */
	if (!alarm_cfg->callback) {
		LOG_DEV_ERR(dev, "alarm: callback function cannot be null");
		return -EINVAL;
	}

	LOG_DEV_DBG(dev, "alarm: set tick:%08x flags:%x", alarm_cfg->ticks, alarm_cfg->flags);

	key = k_spin_lock(&ctx->lock);

	if (!ctx->f_started) {
		k_spin_unlock(&ctx->lock, key);
		LOG_DEV_ERR(dev, "alarm: counter not started");
		return -EINVAL;
	}

	if (ctx->f_periodic) {
		k_spin_unlock(&ctx->lock, key);
		LOG_DEV_ERR(dev, "alarm: not supported in interval mode");
		return -ENOTSUP;
	}

	if (ctx->alarm_cb) {
		k_spin_unlock(&ctx->lock, key);
		LOG_DEV_DBG(dev, "alarm: already set");
		return -EBUSY;
	}

	now = counter_rz_gtm_read(dev);
	ctx->alarm_cb = alarm_cfg->callback;
	ctx->user_data = alarm_cfg->user_data;

	if (absolute) {
		max_rel_val = RZ_GTM_TIMER_TOP_VALUE - ctx->guard_period;
		irq_on_late = alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
	} else {
		/* If relative value is smaller than half of the counter range
		 * it is assumed that there is a risk of setting value too late
		 * and late detection algorithm must be applied. When late
		 * setting is detected, interrupt shall be triggered for
		 * immediate expiration of the timer. Detection is performed
		 * by limiting relative distance between CC and counter.
		 *
		 * Note that half of counter range is an arbitrary value.
		 */
		irq_on_late = val < (RZ_GTM_TIMER_TOP_VALUE / 2U);
		/* limit max to detect short relative being set too late. */
		max_rel_val = irq_on_late ? RZ_GTM_TIMER_TOP_VALUE / 2U : RZ_GTM_TIMER_TOP_VALUE;
		val = (now + val) & RZ_GTM_TIMER_TOP_VALUE;
	}

	sys_write32(val, DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CMP);

	/* Decrement value to detect also case when val == counter_rz_gtm_read(dev). Otherwise,
	 * condition would need to include comparing diff against 0.
	 */
	diff = ((val - 1U) - counter_rz_gtm_read(dev)) & RZ_GTM_TIMER_TOP_VALUE;
	if (diff > max_rel_val) {
		if (absolute) {
			err = -ETIME;
		}

		/* Interrupt is triggered always for relative alarm and
		 * for absolute depending on the flag.
		 */
		if (irq_on_late) {
			irq_enable(cfg->irqn);
			counter_rz_gtm_set_pending(cfg->irqn);
			LOG_DEV_DBG(dev, "alarm: trig1");
		} else {
			ctx->alarm_cb = NULL;
			LOG_DEV_DBG(dev, "alarm: skip");
		}
	} else {
		if (diff == 0) {
			/* RELOAD value could be set just in time for interrupt
			 * trigger or too late. In any case time is interrupt
			 * should be triggered. No need to enable interrupt
			 * on TIMER just make sure interrupt is pending.
			 */
			irq_enable(cfg->irqn);
			counter_rz_gtm_set_pending(cfg->irqn);
			LOG_DEV_DBG(dev, "alarm: trig2");
		} else {
			counter_rz_gtm_clear_pending(cfg->irqn);
			irq_enable(cfg->irqn);
		}
	}

	k_spin_unlock(&ctx->lock, key);

	LOG_DEV_DBG(dev, "alarm: set done late:%d cmp:%08x now:%08x", irq_on_late, val, now);

	return err;
}

static int counter_rz_gtm_cancel_alarm(const struct device *dev, uint8_t chan)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	k_spinlock_key_t key;

	ARG_UNUSED(chan);

	key = k_spin_lock(&ctx->lock);

	if (!ctx->f_started) {
		k_spin_unlock(&ctx->lock, key);
		return -EINVAL;
	}

	if (!ctx->alarm_cb) {
		k_spin_unlock(&ctx->lock, key);
		LOG_DEV_DBG(dev, "alarm: cancel with no alarm");
		return 0;
	}

	irq_disable(cfg->irqn);
	counter_rz_gtm_clear_pending(cfg->irqn);
	ctx->alarm_cb = NULL;
	ctx->user_data = NULL;

	k_spin_unlock(&ctx->lock, key);
	LOG_DEV_DBG(dev, "alarm: cancel");
	return 0;
}

static uint32_t counter_rz_gtm_get_pending_int(const struct device *dev)
{
	const struct counter_rz_gtm_config *config = dev->config;

	/* There is no register to check TIMER peripheral to check for interrupt
	 * pending, check directly in NVIC.
	 */
	return counter_rz_gtm_is_pending(config->irqn);
}

static const struct counter_driver_api counter_rz_gtm_driver_api = {
	.start = counter_rz_gtm_start,
	.stop = counter_rz_gtm_stop,
	.get_value = counter_rz_gtm_get_value,
	.set_alarm = counter_rz_gtm_set_alarm,
	.cancel_alarm = counter_rz_gtm_cancel_alarm,
	.set_top_value = counter_rz_gtm_set_top_value,
	.get_pending_int = counter_rz_gtm_get_pending_int,
	.get_top_value = counter_rz_gtm_get_top_value,
	.get_guard_period = counter_rz_gtm_get_guard_period,
	.set_guard_period = counter_rz_gtm_set_guard_period,
	.get_freq = counter_rz_gtm_get_freq,
};

void counter_rz_gtm_irq_handler(const struct device *dev)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	counter_alarm_callback_t alarm_callback = ctx->alarm_cb;
	k_spinlock_key_t key;
	uint32_t now;

	key = k_spin_lock(&ctx->lock);

	now = counter_rz_gtm_read(dev);
	LOG_DEV_DBG(dev, "irq_x: %08x", now);

	if (alarm_callback) {
		irq_disable(cfg->irqn);
		ctx->alarm_cb = NULL;
		alarm_callback(dev, 0, now, ctx->user_data);
		LOG_DEV_DBG(dev, "irq_alarm: now:%08x cmp:%08x", now,
			    sys_read32(DEVICE_MMIO_NAMED_GET(dev, cnt_regs) + R_GTM_CMP));
	} else if (ctx->top_cb) {
		ctx->top_cb(dev, ctx->user_data);
		LOG_DEV_DBG(dev, "irq_top:");
	}

	k_spin_unlock(&ctx->lock, key);
}

static int counter_rz_gtm_init_timer(const struct device *dev)
{
	const struct counter_rz_gtm_config *cfg = dev->config;
	struct counter_rz_gtm_data *ctx = dev->data;
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, cnt_regs, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->clk_dev)) {
		LOG_DEV_ERR(dev, "clks dev not ready");
		return -ENODEV;
	}
#if DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045)
	if (!device_is_ready(cfg->reset_z.dev)) {
		LOG_DEV_ERR(dev, "resets dev not ready");
		return -ENODEV;
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045) */

	ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_mod);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to configure clk_pclk");
		return ret;
	}

	ret = clock_control_get_rate(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_mod,
				     &ctx->clk_freq);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to get clk rate");
		return ret;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045)
	(void)reset_line_deassert_dt(&cfg->reset_z);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045) */

	/* config/enable IRQ */
	cfg->irq_config(dev);

	LOG_DEV_INF(dev, "init done freq:%u", ctx->clk_freq);
	return 0;
}

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045)
#define COUNTER_RZ_GTM_GET_RST(inst) .reset_z = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 0),
#define COUNTER_RZ_GTM_IRQ_FLAG(inst) 0
#else
#define COUNTER_RZ_GTM_GET_RST(inst)
#define COUNTER_RZ_GTM_IRQ_FLAG(inst) DT_INST_IRQ(inst, flags)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_ostm_r9a08g045) */

#define COUNTER_RZ_GTM_INIT(inst)                                                                  \
                                                                                                   \
	static struct counter_rz_gtm_data counter_data_##inst;                                     \
                                                                                                   \
	static void counter_rz_gtm_irq_cfg_##inst(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                       \
			    counter_rz_gtm_irq_handler, DEVICE_DT_INST_GET(inst),                  \
			    COUNTER_RZ_GTM_IRQ_FLAG(inst));                                        \
	}                                                                                          \
                                                                                                   \
	static const struct counter_rz_gtm_config counter_cfg_##inst = {                           \
		DEVICE_MMIO_NAMED_ROM_INIT(cnt_regs, DT_DRV_INST(inst)),                           \
		.info = {                                                                          \
				.max_top_value = RZ_GTM_TIMER_TOP_VALUE,                           \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = 1,                                                     \
			},                                                                         \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(inst, 0)),                     \
		.clk_mod.module = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, module),                     \
		.clk_mod.domain = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, domain),                     \
		COUNTER_RZ_GTM_GET_RST(inst).irq_config = counter_rz_gtm_irq_cfg_##inst,           \
		.irqn = DT_INST_IRQN(inst),                                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, counter_rz_gtm_init_timer, NULL, &counter_data_##inst,         \
			      &counter_cfg_##inst, PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,     \
			      &counter_rz_gtm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RZ_GTM_INIT)
