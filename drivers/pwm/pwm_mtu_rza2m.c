/*
 * Copyright (c) 2023, EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rza2m_mtu

#include <errno.h>
#include <soc.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/rza2m_pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_mtu_rza2m, CONFIG_PWM_LOG_LEVEL);

#define MAX_CHANNELS 9

#define TCR_V_OFFSET  0x14
#define TCR_W_OFFSET  0x24
#define TCR2_V_OFFSET 0x15
#define TCR2_W_OFFSET 0x25

#define TCR_TPCS_MASK      0x7
#define TCR2_TPCS_MASK     0x7
#define TCR_MTU5_TPCS_MASK 0x3

#define TCR_CCLR_MASK      0xe0
#define TCR_CCLR_SHIFT     5
#define TCR_CCLR_CNT_CLR_A BIT(5)            /* TCNT clr by TGRA cmp match/input cap */
#define TCR_CCLR_CNT_CLR_B BIT(6)            /* TCNT clr by TGRB cmp match/input cap */
#define TCR_CCLR_CNT_CLR_C (BIT(7) | BIT(5)) /* TCNT clr by TGRC cmp match/input cap */
#define TCR_CCLR_CNT_CLR_D (BIT(7) | BIT(6)) /* TCNT clr by TGRD cmp match/input cap */

#define TIER2_OFFSET 0x24

#define TSTRA_OFFSET 0x76 /* Timer start register A */
#define TSTRB_OFFSET 0x76 /* Timer start register B */
#define TSTR_OFFSET  0x34 /* Timer start register MTU5 */
#define TOERA_OFFSET 0x0  /* Timer output master enable register A */
#define TOERB_OFFSET 0x0  /* Timer output master enable register B */

#define TCNT_V_OFFSET 0x10
#define TCNT_W_OFFSET 0x20

#define TCNTCMPCLR_OFFSET       0x36   /* MTU5 timer compare match clear register */
#define TCNTCMPCLR_CNT_CLR_U    BIT(2) /* TCNT clr by TGRU cmp match/input cap */
#define TCNTCMPCLR_CNT_CLR_V    BIT(1) /* TCNT clr by TGRV cmp match/input cap */
#define TCNTCMPCLR_CNT_CLR_W    BIT(0) /* TCNT clr by TGRW cmp match/input cap */
#define TCNTCMPCLR_CNT_CLR_MASK 0x7

#define TIOR_IO_INIT_OUT_HIGH BIT(2)
#define TIOR_IO_LOW_OUT_CMP   BIT(0)            /* Low output at compare match */
#define TIOR_IO_HIGH_OUT_CMP  BIT(1)            /* High output at compare match */
#define TIOR_IO_TOG_OUT_CMP   (BIT(0) | BIT(1)) /* Toggle output at compare match */

#define TIOR_IO_CAP_RIS  BIT(3)
#define TIOR_IO_CAP_FAL  (BIT(3) | BIT(0))
#define TIOR_IO_CAP_BOTH (BIT(3) | BIT(1))

#define TIOR_IO_GEN_AC(flags) (flags)
#define TIOR_IO_AC_MASK       0xf
#define TIOR_IO_GEN_BD(flags) ((flags) << 4)
#define TIOR_IO_BD_MASK       0xf0

#define TIORW_OFFSET 0x26

#define DEV_CFG(_dev) ((const struct pwm_mtu_rza2m_config *)((_dev)->config))

#define DEV_DATA(_dev) ((struct pwm_mtu_rza2m_data *)((_dev)->data))

enum io_channel {
	MTU_IO_CH_A,
	MTU_IO_CH_U = MTU_IO_CH_A,
	MTU_IO_CH_B,
	MTU_IO_CH_V = MTU_IO_CH_B,
	MTU_IO_CH_C,
	MTU_IO_CH_W = MTU_IO_CH_C,
	MTU_IO_CH_D,
	MTU_IO_CH_MAX,
};

enum mtu_reg_offsets {
	REG_TCR1,
	REG_TCR2,
	REG_TCNT,
	REG_TMDR1,
	REG_TGRA,
	REG_TGRU = REG_TGRA,
	REG_TGRB,
	REG_TGRV = REG_TGRB,
	REG_TGRC,
	REG_TGRW = REG_TGRC,
	REG_TGRD,
	REG_TIORH, /* u for mtu5 */
	REG_TIORL, /* v for mtu5 */
	REG_TIER,
	REG_MAX,
};

enum isr_idx {
	ISR_TGIA,
	ISR_TGIB,
	ISR_TGIC,
	ISR_TGID,
	ISR_TGIE,
	ISR_TGIF,
	ISR_TGIU,
	ISR_TGIV,
	ISR_TGIW,
	ISR_TCIV,
	ISR_TCIU,
	ISR_MAX, /* must be last */
};

/* todo: add support of other modes if needed, currently MD_PWM_1 is only supported */
enum timer_mode {
	MD_NORMAL = 0x0,
	MD_PWM_1 = 0x2,
	MD_PWM_2 = 0x3,
	MD_PHASE_CNT_1 = 0x4,
	MD_PHASE_CNT_2 = 0x5,
	MD_PHASE_CNT_3 = 0x6,
	MD_PHASE_CNT_4 = 0x7,
	MD_PWM_RESET_SYNC = 0x8,
	MD_PHASE_CNT_5 = 0x9,
	MD_PWM_COMPL_1 = 0xD,
	MD_PWM_COMPL_2 = 0xE,
	MD_PWM_COMPL_3 = 0xF,
};

struct pwm_mtu_rza2m_config {
	DEVICE_MMIO_NAMED_ROM(mtu);
	DEVICE_MMIO_NAMED_ROM(mtu_cmn_a);
	DEVICE_MMIO_NAMED_ROM(mtu_cmn_b);
	uint32_t channel_id;
	uint32_t divider;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;

	uint32_t irq_lines[ISR_MAX];

	void (*cfg_irqs)(const struct device *dev);
};

struct pwm_mtu_channel_pin {
	bool is_input;
	bool initialized;
	pwm_flags_t flags;
#ifdef CONFIG_PWM_CAPTURE
	bool first_isr_trig_after_start;
	bool cap_started;
	bool is_continuous;
	pwm_capture_callback_handler_t cb;
	void *user_data;
#endif /* CONFIG_PWM_CAPTURE */
};

struct pwm_mtu_rza2m_data {
	DEVICE_MMIO_NAMED_RAM(mtu);
	DEVICE_MMIO_NAMED_RAM(mtu_cmn_a);
	DEVICE_MMIO_NAMED_RAM(mtu_cmn_b);
	struct k_spinlock lock;

	struct pwm_mtu_channel_pin pins[MTU_IO_CH_MAX];
	uint32_t pin_en_mask;
	bool is_ovf;
};

/* todo: there are four extra TCR regs for ch5, add offsets for them too */
static const int32_t reg_offs[MAX_CHANNELS][REG_MAX] = {
	/*CR1 CR2   CNT   MDR1 GRA   GRB   GRC   GRD   IORH IORL  IER  */
	{0x0, 0x28, 0x6,  0x1, 0x8,  0xa,  0xc,  0xe,  0x2, 0x3,  0x4,  },
	{0x0, 0x14, 0x6,  0x1, 0x8,  0xa,  -1,   -1,   0x2, -1,   0x4,  },
	{0x0, 0xc,  0x6,  0x1, 0x8,  0xa,  -1,   -1,   0x2, -1,   0x4,  },
	{0x0, 0x4c, 0x10, 0x2, 0x18, 0x1a, 0x1c, 0x1e, 0x4, 0x5,  0x8,  },
	{0x0, 0x4c, 0x11, 0x2, 0x1b, 0x1d, 0x27, 0x29, 0x5, 0x6,  0x8,  },
	{0x4, 0x5,  0x10, -1,  0x2,  0x12, 0x22, -1,   0x6, 0x16, 0x32, },
	{0x0, 0x4c, 0x10, 0x2, 0x18, 0x1a, 0x24, 0x26, 0x4, 0x5,  0x8,  },
	{0x0, 0x4c, 0x11, 0x2, 0x1b, 0x1d, 0x27, 0x29, 0x5, 0x6,  0x8,  },
	{0x0, 0x6,  0x8,  0x1, 0xc,  0x10, 0x14, 0x18, 0x2, 0x3,  0x4,  },
};

static void rza2m_mtu_pwm_start_stop_cnt(const struct device *dev, uint32_t ch_io, bool start)
{
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;
	uint8_t reg;
	uint32_t base;
	static const uint8_t tstra_start_map[] = {
		BIT(0), BIT(1), BIT(2), BIT(6), BIT(7), 0, 0, 0, BIT(3),
	};
	static const uint8_t tstr_start_map[] = {
		BIT(2), BIT(1), BIT(0), /* u, v, w */
	};

	switch (ch) {
	case 0 ... 4:
	case 8:
		base = DEVICE_MMIO_NAMED_GET(dev, mtu_cmn_a);
		reg = sys_read8(base + TSTRA_OFFSET);
		reg &= ~tstra_start_map[ch];
		if (start) {
			reg |= tstra_start_map[ch];
		}
		sys_write8(reg, base + TSTRA_OFFSET);
		break;
	case 5:
		base = DEVICE_MMIO_NAMED_GET(dev, mtu);
		reg = sys_read8(base + TSTR_OFFSET);

		/* special case init of timer channel */
		if (ch_io == 0xff) {
			reg &= ~tstr_start_map[0];
			reg &= ~tstr_start_map[1];
			reg &= ~tstr_start_map[2];
		} else {
			reg &= ~tstr_start_map[ch_io];
		}

		if (start) {
			reg |= tstr_start_map[0];
		}
		sys_write8(reg, base + TSTR_OFFSET);
		break;
	case 6 ... 7:
		base = DEVICE_MMIO_NAMED_GET(dev, mtu_cmn_b);
		reg = sys_read8(base + TSTRB_OFFSET);
		reg &= ~BIT(ch);
		if (start) {
			reg |= BIT(ch);
		}
		sys_write8(reg, base + TSTRB_OFFSET);
		break;
	default:
		k_panic();
	}
}

static void rza2m_mtu_pwm_output_enable(const struct device *dev, uint32_t ch_io, bool enable)
{
	uint32_t base;
	uint8_t reg;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;
	int8_t toer = -1;
	static const int8_t toer_bit[2][4] = {
		{-1, 0, -1, 3},
		{1, 2, 4, 5},
	};

	switch (ch) {
	case 3:
	case 4:
		base = DEVICE_MMIO_NAMED_GET(dev, mtu_cmn_a) + TOERA_OFFSET;
		toer = toer_bit[ch - 3][ch_io];
		break;
	case 6:
	case 7:
		base = DEVICE_MMIO_NAMED_GET(dev, mtu_cmn_b) + TOERB_OFFSET;
		toer = toer_bit[ch - 6][ch_io];
		break;
	}

	if (toer > -1) {
		reg = sys_read8(base);
		reg &= ~BIT(toer);
		if (enable) {
			reg |= BIT(toer);
		}
		sys_write8(reg, base);
	}
}

static void rza2m_mtu_pwm_set_duty_cyc_config_io(mm_reg_t base, uint32_t ch_pwm, uint32_t ch_io,
						 uint32_t period_cyc, uint32_t pulse_cyc,
						 bool is_inv)
{
	uint32_t period_off, pulse_off;
	uint8_t tior_cmp_out;
	bool corner_case = false;
	static const uint8_t tior_cmp_out_100 =
		TIOR_IO_GEN_AC(TIOR_IO_HIGH_OUT_CMP) |
		TIOR_IO_GEN_BD(TIOR_IO_HIGH_OUT_CMP | TIOR_IO_INIT_OUT_HIGH);
	static const uint8_t tior_cmp_out_0 =
		TIOR_IO_GEN_AC(TIOR_IO_LOW_OUT_CMP) |
		TIOR_IO_GEN_BD(TIOR_IO_LOW_OUT_CMP | TIOR_IO_INIT_OUT_HIGH);

	if (is_inv) {
		tior_cmp_out =
			TIOR_IO_GEN_AC(TIOR_IO_HIGH_OUT_CMP) | TIOR_IO_GEN_BD(TIOR_IO_LOW_OUT_CMP);
	} else {
		tior_cmp_out =
			TIOR_IO_GEN_AC(TIOR_IO_LOW_OUT_CMP) | TIOR_IO_GEN_BD(TIOR_IO_HIGH_OUT_CMP);
	}

	/* handle corner cases */
	if (pulse_cyc == period_cyc) {
		tior_cmp_out = is_inv ? tior_cmp_out_0 : tior_cmp_out_100;
		corner_case = true;
	} else if (pulse_cyc == 0) {
		tior_cmp_out = is_inv ? tior_cmp_out_100 : tior_cmp_out_0;
		corner_case = true;
	}

	if (ch_io == MTU_IO_CH_A) {
		sys_write8(tior_cmp_out, base + reg_offs[ch_pwm][REG_TIORH]);
	} else {
		sys_write8(tior_cmp_out, base + reg_offs[ch_pwm][REG_TIORL]);
	}

	if (corner_case == false) {
		if (ch_io == MTU_IO_CH_A) {
			period_off = reg_offs[ch_pwm][REG_TGRA];
			pulse_off = reg_offs[ch_pwm][REG_TGRB];
		} else {
			period_off = reg_offs[ch_pwm][REG_TGRC];
			pulse_off = reg_offs[ch_pwm][REG_TGRD];
		}

		/* except mtu5, period is equal to value in appropriate trg + 1 */
		period_cyc -= 1;
		sys_write16(period_cyc, base + period_off);
		sys_write16(period_cyc - pulse_cyc, base + pulse_off);
	}
}

static int rza2m_mtu_pwm_set_cycles(const struct device *dev, uint32_t ch_io, uint32_t period_cyc,
				    uint32_t pulse_cyc, pwm_flags_t flags)
{
	const struct pwm_mtu_rza2m_config *config = dev->config;
	struct pwm_mtu_rza2m_data *data = dev->data;
	struct pwm_mtu_channel_pin *pin_cfg;
	mm_reg_t base;
	uint8_t reg;
	int ret = 0;
	k_spinlock_key_t key;
	bool is_inv = ((flags & PWM_POLARITY_INVERTED) == PWM_POLARITY_INVERTED);

	if (ch_io != MTU_IO_CH_A && ch_io != MTU_IO_CH_C) {
		LOG_ERR("%s: A and C IOs are only supported now (restriction of pwm mode 1)",
			dev->name);
		return -ENOTSUP;
	}

	if (ch_io >= MTU_IO_CH_MAX || reg_offs[config->channel_id][REG_TGRA + ch_io] < 0) {
		LOG_ERR("%s: requested IO (%u) isn't supported by timer", dev->name, ch_io);
		return -ENOTSUP;
	}

	/*
	 * Note: in theory we can perfom pwm operation without pwm mode 1,
	 * update TGR reg in cmp ISRs or using DMA. But due delay of IRQs
	 * it can produce a wrong timing of pulses.
	 */
	if (config->channel_id == 5 || config->channel_id == 8) {
		LOG_ERR("%s: this channel doesn't support PWM output", dev->name);
		return -ENOTSUP;
	}

	if (period_cyc == 0) {
		LOG_ERR("%s: period is equal to zero", dev->name);
		return -EINVAL;
	}

	if (period_cyc < pulse_cyc) {
		LOG_ERR("%s: period duration less than pulse duration", dev->name);
		return -EINVAL;
	}

	/*
	 * Note: it is possible to handle in software by using overflow interrupt, but we can lose
	 *       accuracy, looks like it is also possible to use DMA for this purpose and it should
	 *       work faster than handling overflow IRQs. In case when DMA is used, it can be to set
	 *       values from circular buffer and just with TOGGLE pin output configuration.
	 */
	if (config->channel_id != 8 && period_cyc > 0xffff) {
		LOG_ERR("%s: it is a 16-bit channel, so period_cyc has to be less than 0xffff",
			dev->name);
		return -EINVAL;
	}

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);
	pin_cfg = &data->pins[ch_io];

	key = k_spin_lock(&data->lock);

	if (pin_cfg->initialized) {
		if (pin_cfg->is_input) {
			LOG_ERR("%s: channel has been configured as input", dev->name);
			ret = -EINVAL;
			goto exit;
		}

		rza2m_mtu_pwm_set_duty_cyc_config_io(base, config->channel_id, ch_io, period_cyc,
						     pulse_cyc, is_inv);

		pin_cfg->flags = flags;
		goto exit;
	}

	/* stop counter operation */
	rza2m_mtu_pwm_start_stop_cnt(dev, ch_io, false);

	rza2m_mtu_pwm_output_enable(dev, ch_io, true);

	/* timer counter starts from zero */
	sys_write16(0, base + reg_offs[config->channel_id][REG_TCNT]);

	/* set pwm mode 1 */
	if (reg_offs[config->channel_id][REG_TMDR1] > -1) {
		sys_write8(MD_PWM_1, base + reg_offs[config->channel_id][REG_TMDR1]);
	}

	/* clean cnt on cmp of A/C */
	reg = sys_read8(base + reg_offs[config->channel_id][REG_TCR1]);
	reg &= ~TCR_CCLR_MASK;
	reg |= (ch_io == MTU_IO_CH_A) ? TCR_CCLR_CNT_CLR_A : TCR_CCLR_CNT_CLR_C;
	sys_write8(reg, base + reg_offs[config->channel_id][REG_TCR1]);

	rza2m_mtu_pwm_set_duty_cyc_config_io(base, config->channel_id, ch_io, period_cyc, pulse_cyc,
					     is_inv);

	/* start counter operation */
	rza2m_mtu_pwm_start_stop_cnt(dev, ch_io, true);

	pin_cfg->initialized = true;
	pin_cfg->is_input = false;
	pin_cfg->flags = flags;
	data->pin_en_mask |= BIT(ch_io);
exit:
	k_spin_unlock(&data->lock, key);
	return 0;
}

static const int8_t tpcs_shift[MAX_CHANNELS][2][8] = {
	[0] = {{0, 2, 4, 6, -1, -1, -1, -1}, {1, 3, 5, 8, 10, -1, -1}},
	[1] = {{0, 2, 4, 6, -1, -1, 8, -1}, {1, 3, 5, 10, -1, -1, -1}},
	[2] = {{0, 2, 4, 6, -1, -1, -1, 10}, {1, 3, 5, 8, -1, -1, -1}},
	[3 ... 4] = {{0, 2, 4, 6, 8, 10, -1, -1}, {1, 3, 5, -1, -1, -1, -1}},
	[5] = {{0, 2, 4, 6, -1, -1, -1, -1}, {1, 3, 5, 8, 10, -1, -1}},
	[6 ... 8] = {{0, 2, 4, 6, 8, 10, -1, -1}, {1, 3, 5, -1, -1, -1, -1}},
};

static int rza2m_mtu_pwm_get_cycles_per_sec(const struct device *dev, uint32_t ch_io,
					    uint64_t *cycles)
{
	uint32_t rate;
	int ret;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;
	uint8_t tpcs2;
	int8_t shift;
	uint32_t tcr1_off, tcr2_off;

	if (ch_io >= MTU_IO_CH_MAX || reg_offs[config->channel_id][REG_TGRA + ch_io] < 0) {
		LOG_ERR("%s: requested IO (%u) isn't supported by timer", dev->name, ch_io);
		return -ENOTSUP;
	}

	ret = clock_control_get_rate(config->clock_dev, (clock_control_subsys_t)&config->bus_clk,
				     &rate);
	if (ret < 0) {
		LOG_ERR("%s: can't get rate of module id: %u", dev->name, config->bus_clk.module);
		return -EINVAL;
	}

	/*
	 * Note: at the moment, all IO channels of MTU5 reg have the same TPCS,
	 *       so we can get value from U
	 */
	tcr2_off = reg_offs[ch][REG_TCR2];
	tcr1_off = reg_offs[ch][REG_TCR1];

	if (ch == 5) {
		if (ch_io == MTU_IO_CH_V) {
			tcr1_off = TCR_V_OFFSET;
			tcr2_off = TCR2_V_OFFSET;
		} else if (ch_io == MTU_IO_CH_W) {
			tcr1_off = TCR_W_OFFSET;
			tcr2_off = TCR2_W_OFFSET;
		}
	}

	/* get tpcs */
	tpcs2 = sys_read8(DEVICE_MMIO_NAMED_GET(dev, mtu) + tcr2_off);
	tpcs2 &= TCR2_TPCS_MASK;

	/* if any bit in tpcs2 is set, ignore tpcs */
	if (tpcs2) {
		shift = tpcs_shift[ch][1][tpcs2 - 1];
	} else {
		uint8_t tpcs;

		tpcs = sys_read8(DEVICE_MMIO_NAMED_GET(dev, mtu) + tcr1_off);
		if (ch == 5U) {
			tpcs &= TCR_MTU5_TPCS_MASK;
		} else {
			tpcs &= TCR_TPCS_MASK;
		}
		shift = tpcs_shift[ch][0][tpcs];
	}

	if (shift < 0) {
		LOG_ERR("%s: prohibited value or extrenal clock in tpcs is set", dev->name);
		return -EINVAL;
	}

	*cycles = rate >> shift;
	return 0;
}

static int rza2m_mtu_pwm_set_divisor(const struct device *dev, uint32_t div)
{
	uint8_t i, j, reg;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;
	uint32_t base;

	for (i = 0; i < 2; i++) {
		bool finish_loop = false;

		for (j = 0; j < 8; j++) {
			if (div == (1U << tpcs_shift[ch][i][j])) {
				finish_loop = true;
				break;
			}
		}

		if (finish_loop) {
			break;
		}
	}

	if (i == 2) {
		LOG_ERR("%s: can't set divisor (%u) for channel", dev->name, div);
		return -ENOTSUP;
	}

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);
	if (i) {
		reg = sys_read8(base + reg_offs[ch][REG_TCR2]);
		reg &= ~TCR2_TPCS_MASK;
		reg |= j + 1;
		sys_write8(reg, base + reg_offs[ch][REG_TCR2]);

		/* TODO: add possibility for setting different freq for every MTU5 IO channel */
		if (ch == 5) {
			sys_write8(reg, base + TCR2_V_OFFSET);
			sys_write8(reg, base + TCR2_W_OFFSET);
		}
	} else {
		reg = sys_read8(base + reg_offs[ch][REG_TCR1]);
		if (ch == 5) {
			reg &= ~TCR_MTU5_TPCS_MASK;
		} else {
			reg &= ~TCR_TPCS_MASK;
		}
		reg |= j;
		sys_write8(reg, base + reg_offs[ch][REG_TCR1]);

		/* TODO: add possibility for setting different freq for every MTU5 IO channel */
		if (ch == 5) {
			sys_write8(reg, base + TCR_V_OFFSET);
			sys_write8(reg, base + TCR_W_OFFSET);

			reg = sys_read8(base + TCR2_V_OFFSET);
			reg &= ~TCR2_TPCS_MASK;
			sys_write8(reg, base + TCR2_V_OFFSET);
			reg = sys_read8(base + TCR2_W_OFFSET);
			reg &= ~TCR2_TPCS_MASK;
			sys_write8(0, base + TCR2_W_OFFSET);
		}

		/* clear TCR2.TPCS */
		reg = sys_read8(base + reg_offs[ch][REG_TCR2]);
		reg &= ~TCR2_TPCS_MASK;
		sys_write8(reg, base + reg_offs[ch][REG_TCR2]);
	}

	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
/* todo: add support of two inputs by one PWM channel */
static int rza2m_mtu_pwm_configure_capture(const struct device *dev, uint32_t ch_io,
					   pwm_flags_t flags, pwm_capture_callback_handler_t cb,
					   void *user_data)
{
	const struct pwm_mtu_rza2m_config *config = dev->config;
	struct pwm_mtu_rza2m_data *data = dev->data;
	struct pwm_mtu_channel_pin *pin_cfg;
	mm_reg_t base, tior_off = 0;
	uint8_t reg, tior = 0, tior_mask = 0;
	int ret = 0;
	k_spinlock_key_t key;

	if (ch_io >= MTU_IO_CH_MAX || reg_offs[config->channel_id][REG_TGRA + ch_io] < 0) {
		LOG_ERR("%s: requested IO (%u) isn't supported by timer", dev->name, ch_io);
		return -ENOTSUP;
	}

	/*
	 * It is not easy to detect pulses with the existing set of registers. Although we can
	 * trigger capture on every event, including rising and falling edges, it becomes
	 * challenging to determine whether it was a pulse or not. This difficulty arises because
	 * the IRQ handler can be invoked after the signal has changed on the line multiple times.
	 *
	 * In theory, one approach is to change the trigger conditions every time in the IRQ
	 * handler—start on the rising edge and then switch to the falling edge, and vice versa
	 * with each triggering of IRQ. However, this doesn't resolve the issue explained above.
	 * However, if we are confident that pulses and the differences between pulses and periods
	 * have a substantial duration, the probability of capturing incorrect data from the PWM
	 * line will be reduced.
	 */
	if ((flags & PWM_CAPTURE_TYPE_MASK) != PWM_CAPTURE_TYPE_PERIOD) {
		LOG_ERR("%s: period and pulse measurening aren't supported by this timer",
			dev->name);
		return -ENOTSUP;
	}

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);
	pin_cfg = &data->pins[ch_io];

	key = k_spin_lock(&data->lock);
	/* MTU5 has separate TCNT for every input, so we can use all inputs */
	if (config->channel_id != 5 && (data->pin_en_mask & ~BIT(ch_io))) {
		LOG_ERR("%s: currently it is possible to use only one IO pin per channel",
			dev->name);
		ret = -ENOTSUP;
		goto exit;
	}

	if (pin_cfg->initialized) {
		if (!pin_cfg->is_input) {
			LOG_ERR("%s: channel has been configured as output", dev->name);
			ret = -EINVAL;
			goto exit;
		}
	}

	if (pin_cfg->cap_started) {
		LOG_ERR("%s: capture started, pls, stop before reconfigutration", dev->name);
		ret = -EBUSY;
		goto exit;
	}

	/* stop counter operation */
	rza2m_mtu_pwm_start_stop_cnt(dev, ch_io, false);

	if (reg_offs[config->channel_id][REG_TMDR1] > -1) {
		sys_write8(MD_NORMAL, base + reg_offs[config->channel_id][REG_TMDR1]);
	}

	if (config->channel_id != 5) {
		reg = sys_read8(base + reg_offs[config->channel_id][REG_TCR1]);
		reg &= ~TCR_CCLR_MASK;
		switch (ch_io) {
		case MTU_IO_CH_A:
			reg |= TCR_CCLR_CNT_CLR_A;
			tior = TIOR_IO_GEN_AC(TIOR_IO_CAP_RIS);
			tior_mask = TIOR_IO_AC_MASK;
			tior_off = reg_offs[config->channel_id][REG_TIORH];
			break;
		case MTU_IO_CH_B:
			reg |= TCR_CCLR_CNT_CLR_B;
			tior = TIOR_IO_GEN_BD(TIOR_IO_CAP_RIS);
			tior_mask = TIOR_IO_BD_MASK;
			tior_off = reg_offs[config->channel_id][REG_TIORH];
			break;
		case MTU_IO_CH_C:
			reg |= TCR_CCLR_CNT_CLR_C;
			tior = TIOR_IO_GEN_AC(TIOR_IO_CAP_RIS);
			tior_mask = TIOR_IO_AC_MASK;
			tior_off = reg_offs[config->channel_id][REG_TIORL];
			break;
		case MTU_IO_CH_D:
			reg |= TCR_CCLR_CNT_CLR_D;
			tior = TIOR_IO_GEN_BD(TIOR_IO_CAP_RIS);
			tior_mask = TIOR_IO_BD_MASK;
			tior_off = reg_offs[config->channel_id][REG_TIORL];
			break;
		default:
			k_panic();
			break;
		}
		sys_write8(reg, base + reg_offs[config->channel_id][REG_TCR1]);
	} else {
		reg = sys_read8(base + TCNTCMPCLR_OFFSET);
		reg &= ~TCNTCMPCLR_CNT_CLR_MASK;
		tior = BIT(4) | BIT(0); /* input capture at rising edge */
		tior_mask = 0x1f;
		switch (ch_io) {
		case MTU_IO_CH_U:
			reg |= TCNTCMPCLR_CNT_CLR_U;
			tior_off = reg_offs[config->channel_id][REG_TIORH];
			break;
		case MTU_IO_CH_V:
			reg |= TCNTCMPCLR_CNT_CLR_V;
			tior_off = reg_offs[config->channel_id][REG_TIORL];
			break;
		case MTU_IO_CH_W:
			reg |= TCNTCMPCLR_CNT_CLR_W;
			tior_off = TIORW_OFFSET;
			break;
		default:
			k_panic();
			break;
		}
		sys_write8(reg, base + TCNTCMPCLR_OFFSET);
	}

	reg = sys_read8(base + tior_off);
	reg &= ~tior_mask;
	sys_write8(reg | tior, base + tior_off);

	if ((flags & PWM_CAPTURE_MODE_CONTINUOUS) == PWM_CAPTURE_MODE_CONTINUOUS) {
		pin_cfg->is_continuous = true;
	} else {
		pin_cfg->is_continuous = false;
	}

	pin_cfg->cb = cb;
	pin_cfg->user_data = user_data;
	pin_cfg->is_input = true;
	pin_cfg->initialized = true;
	pin_cfg->flags = flags;
exit:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static void rza2m_mtu_pwm_irq_en(const struct device *dev, enum isr_idx isr_idx, bool enable)
{
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;
	uint32_t base;
	uint8_t reg;

	static const int16_t isr_en_map[ISR_MAX] = {
		/* in case of e, f we need to use tier2 */
		BIT(0), BIT(1), BIT(2), BIT(3), BIT(0), BIT(1),
		BIT(2), BIT(1), BIT(0), BIT(4), BIT(5),
	};

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);

	if (isr_idx == ISR_TGIE || isr_idx == ISR_TGIF) {
		reg = sys_read8(base + TIER2_OFFSET);
	} else {
		reg = sys_read8(base + reg_offs[ch][REG_TIER]);
	}

	reg &= ~isr_en_map[isr_idx];

	if (enable) {
		reg |= isr_en_map[isr_idx];
	}

	if (isr_idx == ISR_TGIE || isr_idx == ISR_TGIF) {
		sys_write8(reg, base + TIER2_OFFSET);
	} else {
		sys_write8(reg, base + reg_offs[ch][REG_TIER]);
	}
}

static int rza2m_mtu_pwm_enable_capture(const struct device *dev, uint32_t ch_io)
{
	const struct pwm_mtu_rza2m_config *config = dev->config;
	struct pwm_mtu_rza2m_data *data = dev->data;
	struct pwm_mtu_channel_pin *pin_cfg;
	enum isr_idx isr_idx = -1;
	uint32_t tgr_off = 0, tcnt_off = REG_TCNT;
	mm_reg_t base;
	int ret = 0;
	k_spinlock_key_t key;

	if (ch_io >= MTU_IO_CH_MAX || reg_offs[config->channel_id][REG_TGRA + ch_io] < 0) {
		LOG_ERR("%s: requested IO (%u) isn't supported by timer", dev->name, ch_io);
		return -ENOTSUP;
	}

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);
	pin_cfg = &data->pins[ch_io];

	key = k_spin_lock(&data->lock);

	if (!pin_cfg->initialized) {
		LOG_ERR("%s:%s: pwm channel hasn't configured for capture", __func__, dev->name);
		ret = -EINVAL;
		goto exit;
	}

	if (pin_cfg->cap_started) {
		LOG_ERR("%s:%s: pwm channel hasn't been started", __func__, dev->name);
		ret = -EBUSY;
		goto exit;
	}

	pin_cfg->cap_started = true;
	pin_cfg->first_isr_trig_after_start = true;
	data->is_ovf = false;

	switch (ch_io) {
	case MTU_IO_CH_A:
		isr_idx = (config->channel_id == 5) ? ISR_TGIU : ISR_TGIA;
		tgr_off = (config->channel_id == 5) ? REG_TGRU : REG_TGRA;
		break;
	case MTU_IO_CH_B:
		isr_idx = (config->channel_id == 5) ? ISR_TGIV : ISR_TGIB;
		tgr_off = (config->channel_id == 5) ? REG_TGRV : REG_TGRB;
		tcnt_off = (config->channel_id == 5) ? TCNT_V_OFFSET : REG_TCNT;
		break;
	case MTU_IO_CH_C:
		isr_idx = (config->channel_id == 5) ? ISR_TGIW : ISR_TGIC;
		tgr_off = (config->channel_id == 5) ? REG_TGRW : REG_TGRC;
		tcnt_off = (config->channel_id == 5) ? TCNT_W_OFFSET : REG_TCNT;
		break;
	case MTU_IO_CH_D:
		isr_idx = ISR_TGID;
		tgr_off = REG_TGRD;
		break;
	default:
		k_panic();
		break;
	}

	irq_enable(config->irq_lines[isr_idx]);

	if (config->irq_lines[ISR_TCIV] > -1) {
		irq_enable(config->irq_lines[ISR_TCIV]);
		rza2m_mtu_pwm_irq_en(dev, ISR_TCIV, true);
	}

	/* timer counter starts from zero */
	if (config->channel_id == 8) {
		sys_write32(0, base + reg_offs[config->channel_id][REG_TCNT]);
		sys_write32(0, base + reg_offs[config->channel_id][tgr_off]);
	} else {
		sys_write16(0, base + reg_offs[config->channel_id][tcnt_off]);
		sys_write16(0, base + reg_offs[config->channel_id][tgr_off]);
	}
	rza2m_mtu_pwm_irq_en(dev, isr_idx, true);
	rza2m_mtu_pwm_start_stop_cnt(dev, ch_io, true);
exit:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int rza2m_mtu_pwm_disable_capture(const struct device *dev, uint32_t ch_io)
{
	const struct pwm_mtu_rza2m_config *config = dev->config;
	struct pwm_mtu_rza2m_data *data = dev->data;
	struct pwm_mtu_channel_pin *pin_cfg;
	enum isr_idx isr_idx = -1;
	mm_reg_t base;
	int ret = 0;
	k_spinlock_key_t key;

	if (ch_io >= MTU_IO_CH_MAX || reg_offs[config->channel_id][REG_TGRA + ch_io] < 0) {
		LOG_ERR("%s: requested IO (%u) isn't supported by timer", dev->name, ch_io);
		return -ENOTSUP;
	}

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);
	pin_cfg = &data->pins[ch_io];

	key = k_spin_lock(&data->lock);

	if (!pin_cfg->initialized) {
		LOG_ERR("%s:%s: pwm channel hasn't configured for capture", __func__, dev->name);
		ret = -EINVAL;
		goto exit;
	}

	rza2m_mtu_pwm_start_stop_cnt(dev, ch_io, false);

	switch (ch_io) {
	case MTU_IO_CH_A:
		isr_idx = (config->channel_id == 5) ? ISR_TGIU : ISR_TGIA;
		break;
	case MTU_IO_CH_B:
		isr_idx = (config->channel_id == 5) ? ISR_TGIV : ISR_TGIB;
		break;
	case MTU_IO_CH_C:
		isr_idx = (config->channel_id == 5) ? ISR_TGIW : ISR_TGIC;
		break;
	case MTU_IO_CH_D:
		isr_idx = ISR_TGID;
		break;
	default:
		k_panic();
		break;
	}

	if (config->irq_lines[ISR_TCIV] > -1) {
		rza2m_mtu_pwm_irq_en(dev, ISR_TCIV, false);
		irq_disable(config->irq_lines[ISR_TCIV]);
	}

	rza2m_mtu_pwm_irq_en(dev, isr_idx, false);
	irq_disable(config->irq_lines[isr_idx]);

	pin_cfg->cap_started = false;
	pin_cfg->first_isr_trig_after_start = false;
exit:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static inline void rza2m_mtu_pwm_cap_isr_cmn(struct device *dev,
					     struct pwm_mtu_channel_pin *pin_cfg, uint32_t tgr_off)
{
	uint32_t reg;
	k_spinlock_key_t key;
	pwm_capture_callback_handler_t cb;
	uint32_t pulse = 0, period = 0;
	void *user_data;
	mm_reg_t base;
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	bool is_u16 = (config->channel_id == 8) ? false : true;
	const uint32_t ch_io = ARRAY_INDEX(data->pins, pin_cfg);
	bool is_continuous, is_ovf;

	if (!pin_cfg->is_input) {
		return;
	}

	base = DEVICE_MMIO_NAMED_GET(dev, mtu);

	key = k_spin_lock(&data->lock);

	/*
	 * We cannot start the counter on pin change, so it was initiated before the first trigger.
	 * At the first trigger, we capture the number of ticks between the actual start and the
	 * first trigger. However, this time isn't equal to the period time, so we simply ignore it.
	 */
	if (pin_cfg->first_isr_trig_after_start) {
		pin_cfg->first_isr_trig_after_start = false;
		data->is_ovf = false;
		k_spin_unlock(&data->lock, key);
		return;
	}

	if (is_u16) {
		reg = sys_read16(base + tgr_off);
	} else {
		reg = sys_read32(base + tgr_off);
	}

	if (!pin_cfg->cap_started || reg == 0) {
		data->is_ovf = false;
		k_spin_unlock(&data->lock, key);
		return;
	}

	switch (pin_cfg->flags & PWM_CAPTURE_TYPE_MASK) {
	case PWM_CAPTURE_TYPE_PERIOD:
		period = reg;
		break;
	default:
		k_panic();
		break;
	}

	cb = pin_cfg->cb;
	user_data = pin_cfg->user_data;
	is_continuous = pin_cfg->is_continuous;
	is_ovf = data->is_ovf;

	data->is_ovf = false;
	k_spin_unlock(&data->lock, key);

	if (!is_continuous) {
		rza2m_mtu_pwm_disable_capture(dev, ch_io);
	}

	if (cb) {
		cb(dev, ch_io, period, pulse, is_ovf ? -EOVERFLOW : 0, user_data);
	}
}
#endif /* CONFIG_PWM_CAPTURE */

static const struct pwm_driver_api pwm_mtu_rza2m_driver_api = {
	.set_cycles = rza2m_mtu_pwm_set_cycles,
	.get_cycles_per_sec = rza2m_mtu_pwm_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = rza2m_mtu_pwm_configure_capture,
	.enable_capture = rza2m_mtu_pwm_enable_capture,
	.disable_capture = rza2m_mtu_pwm_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

static int pwm_mtu_rza2m_init(const struct device *dev)
{
	int ret;
	const struct pwm_mtu_rza2m_config *config = dev->config;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("%s: cpg isn't ready", dev->name);
		return -ENODEV;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: can't apply default pincontrol state", dev->name);
		return ret;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		LOG_ERR("%s: can't enable module clock", dev->name);
		return ret;
	}

	DEVICE_MMIO_NAMED_MAP(dev, mtu, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, mtu_cmn_a, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, mtu_cmn_b, K_MEM_CACHE_NONE);

	/* stop counter */
	rza2m_mtu_pwm_start_stop_cnt(dev, 0xff, false);

	if (rza2m_mtu_pwm_set_divisor(dev, config->divider) < 0) {
		return -ENOTSUP;
	}

	if (config->cfg_irqs) {
		config->cfg_irqs(dev);
	}

	return 0;
}

static void __attribute__((unused)) pwm_mtu_dummy_isr(void *arg)
{
}

static void __attribute__((unused)) pwm_rza2m_tgia_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_A], reg_offs[ch][REG_TGRA]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_tgib_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_B], reg_offs[ch][REG_TGRB]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_tgic_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_C], reg_offs[ch][REG_TGRC]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_tgid_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_D], reg_offs[ch][REG_TGRD]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_tgiu_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_U], reg_offs[ch][REG_TGRU]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_tgiv_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_V], reg_offs[ch][REG_TGRV]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_tgiw_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	const struct pwm_mtu_rza2m_config *config = dev->config;
	const uint32_t ch = config->channel_id;

	rza2m_mtu_pwm_cap_isr_cmn(dev, &data->pins[MTU_IO_CH_W], reg_offs[ch][REG_TGRW]);
#endif /* CONFIG_PWM_CAPTURE */
}

static void __attribute__((unused)) pwm_rza2m_ovf_unf_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_mtu_rza2m_data *data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	/*
	 * Underflow can only occur when we have triangle PWM waves, but they are not generated
	 * for both PWM 1 and Normal modes.
	 */
	data->is_ovf = true;
	k_spin_unlock(&data->lock, key);
#endif
	LOG_DBG("%s: overflow or underflow happened", dev->name);
}

#define DT_IRQ_BY_NAME_OR(node_id, name, cell, def_value)                                          \
	COND_CODE_1(DT_IRQ_HAS_NAME(node_id, name), (DT_IRQ_BY_NAME(node_id, name, cell)),         \
		    (def_value))

#define GEN_IRQ_LINE_ARRAY(n)                                                                      \
	{                                                                                          \
		DT_IRQ_BY_NAME_OR(n, tgia, irq, -1), DT_IRQ_BY_NAME_OR(n, tgib, irq, -1),          \
		DT_IRQ_BY_NAME_OR(n, tgic, irq, -1), DT_IRQ_BY_NAME_OR(n, tgid, irq, -1),          \
		DT_IRQ_BY_NAME_OR(n, tgie, irq, -1), DT_IRQ_BY_NAME_OR(n, tgif, irq, -1),          \
		DT_IRQ_BY_NAME_OR(n, tgiu, irq, -1), DT_IRQ_BY_NAME_OR(n, tgiv, irq, -1),          \
		DT_IRQ_BY_NAME_OR(n, tgiw, irq, -1), DT_IRQ_BY_NAME_OR(n, tciv, irq, -1),          \
		DT_IRQ_BY_NAME_OR(n, tciu, irq, -1)                                                \
	}

#define IRQ_CONNECT_IF_HAS_NAME(inst, name, isr)                                                   \
	COND_CODE_1(                                                                               \
		DT_INST_IRQ_HAS_NAME(inst, name),                                                  \
		(IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, name, irq),                                 \
			     DT_INST_IRQ_BY_NAME(inst, name, priority), isr,                       \
			     DEVICE_DT_INST_GET(inst), DT_INST_IRQ_BY_NAME(inst, name, flags))),   \
		())

#define PWM_MTU_DEVICE_INIT_RZA2M(n)                                                               \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct pwm_mtu_rza2m_data pwm_mtu_rza2m_data_##n;                                   \
                                                                                                   \
	static void config_isr_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT_IF_HAS_NAME(n, tgia, pwm_rza2m_tgia_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tgib, pwm_rza2m_tgib_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tgic, pwm_rza2m_tgic_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tgid, pwm_rza2m_tgid_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tgie, pwm_mtu_dummy_isr);                               \
		IRQ_CONNECT_IF_HAS_NAME(n, tgif, pwm_mtu_dummy_isr);                               \
		IRQ_CONNECT_IF_HAS_NAME(n, tgiu, pwm_rza2m_tgiu_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tgiv, pwm_rza2m_tgiv_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tgiw, pwm_rza2m_tgiw_isr);                              \
		IRQ_CONNECT_IF_HAS_NAME(n, tciv, pwm_rza2m_ovf_unf_isr);                           \
		IRQ_CONNECT_IF_HAS_NAME(n, tciu, pwm_mtu_dummy_isr);                               \
	}                                                                                          \
                                                                                                   \
	static const struct pwm_mtu_rza2m_config pwm_mtu_rza2m_config_##n = {                      \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(mtu, DT_DRV_INST(n)),                           \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(mtu_cmn_a, DT_DRV_INST(n)),                     \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(mtu_cmn_b, DT_DRV_INST(n)),                     \
		.channel_id = DT_INST_PROP(n, channel_id),                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),                        \
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),                        \
		.bus_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),                        \
		.bus_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),                        \
		.irq_lines = GEN_IRQ_LINE_ARRAY(DT_DRV_INST(n)),                                   \
		.cfg_irqs = config_isr_##n,                                                        \
		.divider = DT_INST_PROP_OR(n, divider, 1),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pwm_mtu_rza2m_init, NULL, &pwm_mtu_rza2m_data_##n,                \
			      &pwm_mtu_rza2m_config_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,    \
			      &pwm_mtu_rza2m_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MTU_DEVICE_INIT_RZA2M)
