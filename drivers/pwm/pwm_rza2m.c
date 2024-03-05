/*
 * Copyright (c) 2023, EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <soc.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/dt-bindings/pwm/rza2m_pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rzg3s_pwm)
#include <zephyr/drivers/reset.h>
#endif
#include <zephyr/logging/log.h>

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rza2m_pwm)
#define DT_DRV_COMPAT renesas_rza2m_pwm
#elif DT_HAS_COMPAT_STATUS_OKAY(renesas_rzg3s_pwm)
#define DT_DRV_COMPAT renesas_rzg3s_pwm
#endif

LOG_MODULE_REGISTER(pwm_rza2m, CONFIG_PWM_LOG_LEVEL);

#define GTWP_OFFSET  0x0  /* Write-Protection Register */
#define GTSTR_OFFSET 0x4  /* Software Start Register */
#define GTSTP_OFFSET 0x8  /* Software Stop Register */
#define GTCLR_OFFSET 0xc  /* Software Clear Register */

#define GTSSR_OFFSET 0x10 /* Start Source Select Register */

/*
 * The below registers bits compatible with the next GPT PWM regs:
 *   - General PWM Timer Start Source Select Register (GTSSR)
 *   - General PWM Timer Stop Source Select Register (GTPSR)
 *   - General PWM Timer Clear Source Select Register (GTCSR)
 *   - General PWM Timer Up Count Source Select Register (GTUPSR)
 *   - General PWM Timer Down Count Source Select Register (GTDNSR)
 *   - General PWM Timer Input Capture Source Select Register A (GTICASR)
 *   - General PWM Timer Input Capture Source Select Register B (GTICBSR)
 */

/* Enable action on the rising edge of GTIOCA input when GTIOCB input is 0 */
#define GT_ARBL BIT(8)
/* Enable action on the rising edge of GTIOCA input when GTIOCB input is 1 */
#define GT_ARBH BIT(9)
#define GT_AR   (GT_ARBL | GT_ARBH)
/* Enable action on the falling edge of GTIOCA input when GTIOCB input is 0 */
#define GT_AFBL BIT(10)
/* Enable action on the falling edge of GTIOCA input when GTIOCB input is 1 */
#define GT_AFBH BIT(11)
#define GT_AF   (GT_AFBL | GT_AFBH)
/* Enable action on the rising edge of GTIOCB input when GTIOCA input is 0 */
#define GT_BRAL BIT(12)
/* Enable action on the rising edge of GTIOCB input when GTIOCA input is 1 */
#define GT_BRAH BIT(13)
#define GT_BR   (GT_BRAL | GT_BRAH)
/* Enable action on the falling edge of GTIOCB input when GTIOCA input is 0 */
#define GT_BFAL BIT(14)
/* Enable action on the falling edge of GTIOCB input when GTIOCA input is 1 */
#define GT_BFAH BIT(15)
#define GT_BF   (GT_BFAL | GT_BFAH)

#define GTPSR_OFFSET 0x14 /* Stop Source Select Register */

#define GTCSR_OFFSET 0x18 /* Clear Source Select Register */

#define GTUPSR_OFFSET 0x1C /* Up Count Source Select Register */
#define GTDNSR_OFFSET 0x20 /* Down Count Source Select Register */

#define GTICASR_OFFSET 0x24 /* Input Capture Source Select Register A */
#define GTICBSR_OFFSET 0x28 /* Input Capture Source Select Register B */

#define GTCR_OFFSET        0x2c /* General PWM Timer Control Register */
#define GTCR_TPCS_SHIFT    24   /* Timer Prescaler Select */
#define GTCR_TPCS_MASK     0x7
#define GTCR_TPCS_MAX_VAL  5U
#define GTCR_GET_TPCS(reg) ((reg >> GTCR_TPCS_SHIFT) & GTCR_TPCS_MASK)
#define GTCR_SET_TPCS(reg, tpcs)                                                                   \
	((reg & ~(GTCR_TPCS_MASK << GTCR_TPCS_SHIFT)) |                                            \
	 ((tpcs & GTCR_TPCS_MASK) << GTCR_TPCS_SHIFT))
#define GTCR_MD_SHIFT 16
#define GTCR_MD_MASK  0x3
#define GTCR_SET_MD(reg, md)                                                                       \
	((reg & ~(GTCR_MD_MASK << GTCR_MD_SHIFT)) | ((md & GTCR_MD_MASK) << GTCR_MD_SHIFT))
#define GTCR_START_CNT BIT(0)

#define GTUDDTYC_OFFSET 0x30   /* Count Direction and Duty Setting Register */
#define GTUDDTYC_UD     BIT(0) /* Count Direction Setting: counts up on GTCNT */

#define GTUDDTYC_OADTY_MASK (BIT(17) | BIT(16))
#define GTUDDTYC_OADTY_0    BIT(17)
#define GTUDDTYC_OADTY_100  (BIT(17) | BIT(16))

#define GTUDDTYC_OBDTY_MASK (BIT(25) | BIT(24))
#define GTUDDTYC_OBDTY_0    BIT(25)
#define GTUDDTYC_OBDTY_100  (BIT(25) | BIT(24))

#define GTIOR_OFFSET 0x34 /* I/O Control Register */

/* levels of out on compare match A */
#define GTIOR_GTIOA_OUT_CYC_CMP_MASK   (BIT(1) | BIT(0))
#define GTIOR_GTIOA_OUT_CYC_CMP_RETAIN 0
#define GTIOR_GTIOA_OUT_CYC_CMP_LOW    BIT(0)
#define GTIOR_GTIOA_OUT_CYC_CMP_HIGH   BIT(1)
#define GTIOR_GTIOA_OUT_CYC_CMP_TOG    (BIT(1) | BIT(0))

/* levels of out on end of the cycle A */
#define GTIOR_GTIOA_OUT_CYC_END_MASK   (BIT(3) | BIT(2))
#define GTIOR_GTIOA_OUT_CYC_END_RETAIN 0
#define GTIOR_GTIOA_OUT_CYC_END_LOW    BIT(2)
#define GTIOR_GTIOA_OUT_CYC_END_HIGH   BIT(3)
#define GTIOR_GTIOA_OUT_CYC_END_TOG    (BIT(3) | BIT(2))

#define GTIOR_GTIOA_INIT_OUT_HIGH BIT(4)

/* levels of out on compare match B*/
#define GTIOR_GTIOB_OUT_CYC_CMP_MASK   (BIT(17) | BIT(16))
#define GTIOR_GTIOB_OUT_CYC_CMP_RETAIN 0
#define GTIOR_GTIOB_OUT_CYC_CMP_LOW    BIT(16)
#define GTIOR_GTIOB_OUT_CYC_CMP_HIGH   BIT(17)
#define GTIOR_GTIOB_OUT_CYC_CMP_TOG    (BIT(17) | BIT(16))

/* levels of out on end of the cycle B */
#define GTIOR_GTIOB_OUT_CYC_END_MASK   (BIT(19) | BIT(18))
#define GTIOR_GTIOB_OUT_CYC_END_RETAIN 0
#define GTIOR_GTIOB_OUT_CYC_END_LOW    BIT(18)
#define GTIOR_GTIOB_OUT_CYC_END_HIGH   BIT(19)
#define GTIOR_GTIOB_OUT_CYC_END_TOG    (BIT(19) | BIT(18))

#define GTIOR_GTIOB_INIT_OUT_HIGH BIT(20)

#define GTIOR_OAE BIT(8)  /* GTIOCA Pin Output Enable */
#define GTIOR_OBE BIT(24) /* GTIOCB Pin Output Enable */

#define GTINTAD_OFFSET 0x38   /* Interrupt Output Setting Register */
#define GTINTAD_GTINTA BIT(0) /* GTCCRA Compare Match/InputCapture Interrupt Enable */
#define GTINTAD_GTINTB BIT(1) /* GTCCRB Compare Match/InputCapture Interrupt Enable */

#define GTST_OFFSET 0x3C /* Status Register */
#define GTST_TCFA   BIT(0) /* Input capture/compare match of GTCCRA occurred */
#define GTST_TCFB   BIT(1) /* Input capture/compare match of GTCCRB occurred */

#define GTBER_OFFSET 0x40 /* Buffer Enable Register */

#define GTBER_CCRA_1_BUF (BIT(16))           /* Single buffer operation for GTCCRA */
#define GTBER_CCRA_2_BUF (BIT(17) | BIT(16)) /* Double buffer operation for GTCCRA */

#define GTBER_CCRB_1_BUF (BIT(18))           /* Single buffer operation for GTCCRB */
#define GTBER_CCRB_2_BUF (BIT(19) | BIT(18)) /* Double buffer operation for GTCCRB */

#define GTBER_PR_1_BUF (BIT(20))           /* Single buffer operation for GTPR */
#define GTBER_PR_2_BUF (BIT(21) | BIT(20)) /* Double buffer operation for GTPR */

#define GTBER_ADTTA_CREST        (BIT(24))
#define GTBER_ADTTA_TROUGH       (BIT(25))
#define GTBER_ADTTA_CREST_TROUGH (BIT(25) | BIT(24))

#define GTBER_ADTTB_CREST        (BIT(28))
#define GTBER_ADTTB_TROUGH       (BIT(29))
#define GTBER_ADTTB_CREST_TROUGH (BIT(29) | BIT(28))

#define GTBER_CCRA_1_BUF_EN (GTBER_CCRA_1_BUF | GTBER_ADTTA_CREST)
#define GTBER_CCRB_1_BUF_EN (GTBER_CCRB_1_BUF | GTBER_ADTTB_CREST)

#define GTITC_OFFSET 0x44 /* Interrupt and A/D Converter Start Request Skipping Setting Register */
#define GTITC_ITLA                  BIT(0)
#define GTITC_ITLB                  BIT(1)
#define GTITC_IVTC_MASK             (BIT(6) | BIT(7))
#define GTITC_IVTC_SAW_CREST        BIT(6)
#define GTITC_IVTC_SAW_TROUGH       BIT(7)
#define GTITC_IVTC_SAW_CREST_TROUGH (BIT(7) | BIT(6))
#define GTITC_IVTT_MASK             (BIT(10) | BIT(9) | BIT(8))
#define GTITC_IVTT_OFFSET           8
#define GTITC_IVTT_GEN(val)         (((val) << GTITC_IVTT_OFFSET) & GTITC_IVTT_MASK)

#define GTCNT_OFFSET 0x48 /* Counter */

#define GTCCRA_OFFSET 0x4C /* Compare Capture Register A */
#define GTCCRB_OFFSET 0x50 /* Compare Capture Register B */
#define GTCCRC_OFFSET 0x54 /* Compare Capture Register C */
#define GTCCRD_OFFSET 0x58 /* Compare Capture Register D */
#define GTCCRE_OFFSET 0x5C /* Compare Capture Register E */
#define GTCCRF_OFFSET 0x60 /* Compare Capture Register F */

#define GTPR_OFFSET   0x64 /* Cycle Setting Register */
#define GTPBR_OFFSET  0x68 /* Cycle Setting Buffer Register */
#define GTPDBR_OFFSET 0x6C /* Cycle Setting Double-Buffer Register */

#define GTSOS_OFFSET  0x9C /* Output Protection Function Status Register */
#define GTSOTR_OFFSET 0xA0 /* Output Protection Function Temporary Release Register */

/* todo: only saw-wave mode is supported now, add other modes */
enum pwm_modes {
	PWM_MD_SAW_WAVE = 0,
	PWM_MD_SAW_WAVE_ONE_SHOT = 1,
	PWM_MD_TRIANGLE_WAVE_1 = 4,
	PWM_MD_TRIANGLE_WAVE_2 = 5,
	PWM_MD_TRIANGLE_WAVE_3 = 6,
};

enum duty_cycle {
	DUTY_0,
	DUTY_100,
	DUTY_MAX,
};

enum pin_on_channel {
	PIN_A,
	PIN_B,
	MAX_IO_PINS_PER_CHANNEL,
};

enum isr_idx {
	ISR_CCMPA,
	ISR_CCMPB,
	ISR_CMPC,
	ISR_CMPD,
	ISR_CMPE,
	ISR_CMPF,
	ISR_ADTRGA,
	ISR_ADTRGB,
	ISR_OVF,
	ISR_UDF,
	ISR_MAX,
};

struct pwm_rza2m_config {
	DEVICE_MMIO_ROM; /* Must be first */
	uint32_t channel_id;
	uint32_t divider;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;
#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rzg3s_pwm)
	const struct reset_dt_spec reset;
#endif
	uint32_t irq_lines[ISR_MAX];
	void (*cfg_irqs)(const struct device *dev);
};

struct pwm_channel_pin {
	uint32_t requested_ch;
	bool is_input;
	bool is_inverted;
	bool initialized;
	pwm_flags_t flags;
#ifdef CONFIG_PWM_CAPTURE
	uint32_t cnt_isr_trig;
	bool cap_started;
	bool is_continuous;
	pwm_capture_callback_handler_t cb;
	void *user_data;
#endif /* CONFIG_PWM_CAPTURE */
};

struct pwm_rza2m_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct k_spinlock lock;

	/* todo: add support of external input pins */
	struct pwm_channel_pin pins[MAX_IO_PINS_PER_CHANNEL];
};

static inline void rza2m_pwm_set_duty_setting(mm_reg_t base, uint32_t period_cycles,
					      uint32_t pulse_cycles, bool is_channel_b,
					      bool is_inverted)
{
	mm_reg_t reg, new_reg;
	static const uint32_t duty_cyc_cfgs[MAX_IO_PINS_PER_CHANNEL][DUTY_MAX][2] = {
		[PIN_A] = {
				[DUTY_0] = {GTUDDTYC_OADTY_0, GTUDDTYC_OADTY_100},
				[DUTY_100] = {GTUDDTYC_OADTY_100, GTUDDTYC_OADTY_0},
			},
		[PIN_B] = {
				[DUTY_0] = {GTUDDTYC_OBDTY_0, GTUDDTYC_OBDTY_100},
				[DUTY_100] = {GTUDDTYC_OBDTY_100, GTUDDTYC_OBDTY_0},
			},
	};

	static const uint32_t duty_cyc_masks[MAX_IO_PINS_PER_CHANNEL] = {
		[PIN_A] = GTUDDTYC_OADTY_MASK,
		[PIN_B] = GTUDDTYC_OBDTY_MASK,
	};

	reg = sys_read32(base + GTUDDTYC_OFFSET);
	new_reg = reg & ~duty_cyc_masks[is_channel_b];
	if (period_cycles == pulse_cycles) {
		new_reg |= duty_cyc_cfgs[is_channel_b][DUTY_100][is_inverted];
	} else if (pulse_cycles == 0) {
		new_reg |= duty_cyc_cfgs[is_channel_b][DUTY_0][is_inverted];
	}

	if (new_reg != reg) {
		sys_write32(new_reg, base + GTUDDTYC_OFFSET);
	}
}

#define CH_A_IO_FLAGS_NORMAL (GTIOR_OAE|GTIOR_GTIOA_OUT_CYC_END_HIGH|GTIOR_GTIOA_OUT_CYC_CMP_LOW)
#define CH_A_IO_FLAGS_INV (GTIOR_OAE|GTIOR_GTIOA_OUT_CYC_END_LOW|GTIOR_GTIOA_OUT_CYC_CMP_HIGH)
#define CH_B_IO_FLAGS_NORMAL (GTIOR_OBE|GTIOR_GTIOB_OUT_CYC_END_HIGH|GTIOR_GTIOB_OUT_CYC_CMP_LOW)
#define CH_B_IO_FLAGS_INV (GTIOR_OBE|GTIOR_GTIOB_OUT_CYC_END_LOW|GTIOR_GTIOB_OUT_CYC_CMP_HIGH)

static void rza2m_pwm_cfg_io(mm_reg_t base, bool is_channel_b, bool is_inv)
{
	static const uint32_t ctior_states[MAX_IO_PINS_PER_CHANNEL][2] = {
		{CH_A_IO_FLAGS_NORMAL, CH_A_IO_FLAGS_INV},
		{CH_B_IO_FLAGS_NORMAL, CH_B_IO_FLAGS_INV},
	};

	sys_write32(ctior_states[is_channel_b][is_inv], base + GTIOR_OFFSET);
}

/* todo: add support of two in/out pins on the same channel */
static int rza2m_pwm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cyc,
				uint32_t pulse_cyc, pwm_flags_t flags)
{
	const struct pwm_rza2m_config *config = dev->config;
	struct pwm_rza2m_data *data = dev->data;
	struct pwm_channel_pin *pin_cfg;
	mm_reg_t base, reg;
	int ret = 0;
	k_spinlock_key_t key;
	bool is_channel_b = ((channel & PWM_CHANNEL_INPUT_B) == PWM_CHANNEL_INPUT_B);
	bool is_inv = ((flags & PWM_POLARITY_INVERTED) == PWM_POLARITY_INVERTED);

	if (config->channel_id != (channel & ~PWM_CHANNEL_INPUT_B)) {
		LOG_ERR("%s:%s: request channel (%u) from another pwm device", __func__, dev->name,
			channel);
		return -EINVAL;
	}

	if (period_cyc == 0) {
		LOG_ERR("%s: period is equal to zero", dev->name);
		return -EINVAL;
	}

	if (period_cyc < pulse_cyc) {
		LOG_ERR("%s: period duration less than pulse duration", dev->name);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);
	pin_cfg = &data->pins[is_channel_b];

	key = k_spin_lock(&data->lock);
	if (data->pins[!is_channel_b].initialized) {
		LOG_ERR("%s: currently it is possible to use only one IO pin per channel",
			dev->name);
		ret = -ENOTSUP;
		goto exit;
	}

	if (pin_cfg->initialized) {
		if (pin_cfg->is_input) {
			LOG_ERR("%s: channel has been configured as input", dev->name);
			ret = -EINVAL;
			goto exit;
		}

		if (is_channel_b) {
			sys_write32(pulse_cyc, base + GTCCRD_OFFSET);
		} else {
			sys_write32(pulse_cyc, base + GTCCRC_OFFSET);
		}

		sys_write32(period_cyc - 1, base + GTPBR_OFFSET);
		rza2m_pwm_set_duty_setting(base, period_cyc, pulse_cyc, is_channel_b, is_inv);

		if (pin_cfg->is_inverted != is_inv) {
			rza2m_pwm_cfg_io(base, is_channel_b, is_inv);
			pin_cfg->is_inverted = is_inv;
		}
		pin_cfg->flags = flags;
		goto exit;
	}

	/* stop counter operation */
	reg = sys_read32(base + GTCR_OFFSET);
	sys_write32(reg & ~GTCR_START_CNT, base + GTCR_OFFSET);

	/* counter goes up */
	reg = sys_read32(base + GTUDDTYC_OFFSET);
	sys_write32(reg | GTUDDTYC_UD, base + GTUDDTYC_OFFSET);

	rza2m_pwm_set_duty_setting(base, period_cyc, pulse_cyc, is_channel_b, is_inv);

	/* timer counter starts from zero */
	sys_write32(0, base + GTCNT_OFFSET);

	if (is_channel_b) {
		sys_write32(pulse_cyc, base + GTCCRD_OFFSET);
		sys_write32(pulse_cyc, base + GTCCRB_OFFSET);
	} else {
		sys_write32(pulse_cyc, base + GTCCRC_OFFSET);
		sys_write32(pulse_cyc, base + GTCCRA_OFFSET);
	}

	/* todo: add possibility to use A and B at the same time at the same PWM channel */
	sys_write32(period_cyc - 1, base + GTPR_OFFSET);
	sys_write32(period_cyc - 1, base + GTPBR_OFFSET);

	/* enable bufferization for registers GTCCRA, GTCCRB and GTPR */
	if (is_channel_b) {
		sys_write32(GTBER_CCRB_1_BUF_EN | GTBER_PR_1_BUF, base + GTBER_OFFSET);
	} else {
		sys_write32(GTBER_CCRA_1_BUF_EN | GTBER_PR_1_BUF, base + GTBER_OFFSET);
	}

	rza2m_pwm_cfg_io(base, is_channel_b, is_inv);

	/* start counter operation */
	reg = sys_read32(base + GTCR_OFFSET);
	sys_write32(reg | GTCR_START_CNT, base + GTCR_OFFSET);

	pin_cfg->initialized = true;
	pin_cfg->is_inverted = is_inv;
	pin_cfg->is_input = false;
	pin_cfg->flags = flags;

exit:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int rza2m_pwm_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					uint64_t *cycles)
{
	uint32_t shift;
	uint32_t rate;
	int ret;
	const struct pwm_rza2m_config *config = dev->config;

	ret = clock_control_get_rate(config->clock_dev, (clock_control_subsys_t)&config->bus_clk,
				     &rate);
	if (ret < 0) {
		LOG_ERR("%s: can't get rate of module id: %u", dev->name, config->bus_clk.module);
		return -EINVAL;
	}

	/* we have the same divisor for all in/out */
	shift = sys_read32(DEVICE_MMIO_GET(dev) + GTCR_OFFSET);
	shift = GTCR_GET_TPCS(shift);

	if (shift > GTCR_TPCS_MAX_VAL) {
		LOG_ERR("%s: clock shift hasn't be bigger than %u", dev->name, GTCR_TPCS_MAX_VAL);
		return -EINVAL;
	}

	shift <<= 1;
	*cycles = rate >> shift;

	LOG_DBG("%s: actual div: %u and freq: %u Hz", dev->name, (1 << shift), (uint32_t)*cycles);
	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
/* todo: add support of two inputs by one PWM channel */
static int rza2m_pwm_configure_capture(const struct device *dev, uint32_t channel,
				       pwm_flags_t flags, pwm_capture_callback_handler_t cb,
				       void *user_data)
{
	const struct pwm_rza2m_config *config = dev->config;
	struct pwm_rza2m_data *data = dev->data;
	struct pwm_channel_pin *pin_cfg;
	mm_reg_t base, reg;
	int ret = 0;
	k_spinlock_key_t key;
	const bool is_channel_b = ((channel & PWM_CHANNEL_INPUT_B) == PWM_CHANNEL_INPUT_B);
	const bool is_inverted = ((flags & PWM_POLARITY_INVERTED) == PWM_POLARITY_INVERTED);

	if (config->channel_id != (channel & ~PWM_CHANNEL_INPUT_B)) {
		LOG_ERR("%s:%s: request channel (%u) from another pwm device", __func__, dev->name,
			channel);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);
	pin_cfg = &data->pins[is_channel_b];

	key = k_spin_lock(&data->lock);
	if (data->pins[!is_channel_b].initialized) {
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
	reg = sys_read32(base + GTCR_OFFSET);
	sys_write32(reg & ~GTCR_START_CNT, base + GTCR_OFFSET);

	/* counter goes up */
	reg = sys_read32(base + GTUDDTYC_OFFSET);
	sys_write32(reg | GTUDDTYC_UD, base + GTUDDTYC_OFFSET);

	/* set maximum number cycles to 2^32 */
	sys_write32(0xffffffff, base + GTPR_OFFSET);

	/* disable interrupt skipping function */
	sys_write32(0, base + GTITC_OFFSET);

	reg = sys_read32(base + GTSSR_OFFSET);
	if (is_channel_b) {
		sys_write32(reg & ~(GT_BF | GT_BR), base + GTSSR_OFFSET);
		sys_write32(is_inverted ? GT_BF : GT_BR, base + GTCSR_OFFSET);
		sys_write32(GTBER_CCRB_1_BUF, base + GTBER_OFFSET);

		switch (flags & PWM_CAPTURE_TYPE_MASK) {
		case PWM_CAPTURE_TYPE_PERIOD:
			sys_write32(is_inverted ? GT_BF : GT_BR, base + GTICBSR_OFFSET);
			break;
		case PWM_CAPTURE_TYPE_PULSE:
			sys_write32(is_inverted ? GT_BR : GT_BF, base + GTICBSR_OFFSET);
			break;
		case PWM_CAPTURE_TYPE_BOTH:
			sys_write32(GT_BF | GT_BR, base + GTICBSR_OFFSET);
			break;
		default:
			ret = -EINVAL;
			goto exit;
		}
	} else {
		sys_write32(reg & ~(GT_AF | GT_AR), base + GTSSR_OFFSET);
		sys_write32(is_inverted ? GT_AF : GT_AR, base + GTCSR_OFFSET);
		sys_write32(GTBER_CCRA_1_BUF, base + GTBER_OFFSET);

		switch (flags & PWM_CAPTURE_TYPE_MASK) {
		case PWM_CAPTURE_TYPE_PERIOD:
			sys_write32(is_inverted ? GT_AF : GT_AR, base + GTICASR_OFFSET);
			break;
		case PWM_CAPTURE_TYPE_PULSE:
			sys_write32(is_inverted ? GT_AR : GT_AF, base + GTICASR_OFFSET);
			break;
		case PWM_CAPTURE_TYPE_BOTH:
			sys_write32(GT_AF | GT_AR, base + GTICASR_OFFSET);
			break;
		default:
			ret = -EINVAL;
			goto exit;
		}
	}

	if ((flags & PWM_CAPTURE_MODE_CONTINUOUS) == PWM_CAPTURE_MODE_CONTINUOUS) {
		pin_cfg->is_continuous = true;
	} else {
		pin_cfg->is_continuous = false;
	}

	pin_cfg->cb = cb;
	pin_cfg->user_data = user_data;
	pin_cfg->requested_ch = channel;
	pin_cfg->is_inverted = is_inverted;
	pin_cfg->is_input = true;
	pin_cfg->initialized = true;
	pin_cfg->flags = flags;
exit:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int rza2m_pwm_enable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_rza2m_config *config = dev->config;
	struct pwm_rza2m_data *data = dev->data;
	struct pwm_channel_pin *pin_cfg;
	mm_reg_t base;
	uint32_t reg, ctst, gtssr;
	int ret = 0;
	k_spinlock_key_t key;
	const bool is_channel_b = ((channel & PWM_CHANNEL_INPUT_B) == PWM_CHANNEL_INPUT_B);

	channel &= ~PWM_CHANNEL_INPUT_B;
	if (config->channel_id != channel) {
		LOG_ERR("%s:%s: request channel (%u) from another pwm device", __func__, dev->name,
			channel);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);
	pin_cfg = &data->pins[is_channel_b];

	key = k_spin_lock(&data->lock);

	if (pin_cfg->cap_started) {
		LOG_ERR("%s:%s: capture has been already started", __func__, dev->name);
		ret = -EBUSY;
		goto exit;
	}

	if (!pin_cfg->initialized) {
		LOG_ERR("%s:%s: pwm channel hasn't configured for capture", __func__, dev->name);
		ret = -EINVAL;
		goto exit;
	}

	pin_cfg->cap_started = true;
	pin_cfg->cnt_isr_trig = 0;

	sys_write32(0, base + GTCNT_OFFSET);

	/* unmask IRQ on capture for INT A/B */
	reg = sys_read32(base + GTINTAD_OFFSET);
	ctst = sys_read32(base + GTST_OFFSET);
	gtssr = sys_read32(base + GTSSR_OFFSET);
	if (is_channel_b) {
		sys_write32((pin_cfg->is_inverted ? GT_BF : GT_BR) | gtssr, base + GTSSR_OFFSET);
		/* clear capture regs and their buffers */
		sys_write32(0, base + GTCCRB_OFFSET);
		sys_write32(0, base + GTCCRD_OFFSET);
		sys_write32(reg | GTINTAD_GTINTB, base + GTINTAD_OFFSET);
		sys_write32(ctst & ~GTST_TCFB, base + GTST_OFFSET);
		irq_enable(config->irq_lines[ISR_CCMPB]);
	} else {
		sys_write32((pin_cfg->is_inverted ? GT_AF : GT_AR) | gtssr, base + GTSSR_OFFSET);
		/* clear capture regs and their buffers */
		sys_write32(0, base + GTCCRA_OFFSET);
		sys_write32(0, base + GTCCRC_OFFSET);
		sys_write32(reg | GTINTAD_GTINTA, base + GTINTAD_OFFSET);
		sys_write32(ctst & ~GTST_TCFA, base + GTST_OFFSET);
		irq_enable(config->irq_lines[ISR_CCMPA]);
	}
exit:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int rza2m_pwm_disable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_rza2m_config *config = dev->config;
	struct pwm_rza2m_data *data = dev->data;
	struct pwm_channel_pin *pin_cfg;
	mm_reg_t base;
	uint32_t reg, ctst;
	int ret = 0;
	k_spinlock_key_t key;
	const bool is_channel_b = ((channel & PWM_CHANNEL_INPUT_B) == PWM_CHANNEL_INPUT_B);

	channel &= ~PWM_CHANNEL_INPUT_B;
	if (config->channel_id != channel) {
		LOG_ERR("%s:%s: request channel (%u) from another pwm device", __func__, dev->name,
			channel);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);
	pin_cfg = &data->pins[is_channel_b];

	key = k_spin_lock(&data->lock);

	if (!pin_cfg->initialized) {
		LOG_ERR("%s:%s: pwm channel hasn't configured for capture", __func__, dev->name);
		ret = -EINVAL;
		goto exit;
	}

	pin_cfg->cap_started = false;

	/* disable auto start of cnt on input edges */
	reg = sys_read32(base + GTSSR_OFFSET);
	if (is_channel_b) {
		sys_write32(reg & ~(GT_BR | GT_BF), base + GTSSR_OFFSET);
	} else {
		sys_write32(reg & ~(GT_AR | GT_AF), base + GTSSR_OFFSET);
	}

	/* stop counter operation */
	reg = sys_read32(base + GTCR_OFFSET);
	sys_write32(reg & ~GTCR_START_CNT, base + GTCR_OFFSET);

	reg = sys_read32(base + GTINTAD_OFFSET);
	ctst = sys_read32(base + GTST_OFFSET);
	if (is_channel_b) {
		sys_write32(reg & ~GTINTAD_GTINTB, base + GTINTAD_OFFSET);
		sys_write32(ctst & ~GTST_TCFB, base + GTST_OFFSET);
		irq_disable(config->irq_lines[ISR_CCMPB]);
	} else {
		sys_write32(reg & ~GTINTAD_GTINTA, base + GTINTAD_OFFSET);
		sys_write32(ctst & ~GTST_TCFA, base + GTST_OFFSET);
		irq_disable(config->irq_lines[ISR_CCMPA]);
	}
exit:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static inline void pwm_rza2m_ccmp_isr_cmn(struct device *dev, struct pwm_channel_pin *pin_cfg,
					  bool is_channel_b)
{
	uint32_t reg;
	k_spinlock_key_t key;
	pwm_capture_callback_handler_t cb;
	uint32_t pulse = 0, period = 0;
	void *user_data;
	mm_reg_t base;
	bool transfer_capture = true;
	struct pwm_rza2m_data *data = dev->data;
	bool is_continuous;

	if (!pin_cfg->is_input) {
		return;
	}

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);
	cb = pin_cfg->cb;
	user_data = pin_cfg->user_data;
	is_continuous = pin_cfg->is_continuous;

	if (is_channel_b) {
		reg = sys_read32(base + GTCCRB_OFFSET);
	} else {
		reg = sys_read32(base + GTCCRA_OFFSET);
	}

	if (!pin_cfg->cap_started || reg == 0) {
		k_spin_unlock(&data->lock, key);
		return;
	}

	pin_cfg->cnt_isr_trig++;

	switch (pin_cfg->flags & PWM_CAPTURE_TYPE_MASK) {
	case PWM_CAPTURE_TYPE_PERIOD:
		period = reg;
		break;
	case PWM_CAPTURE_TYPE_PULSE:
		pulse = reg;
		break;
	case PWM_CAPTURE_TYPE_BOTH:
		if (is_channel_b) {
			pulse = sys_read32(base + GTCCRD_OFFSET);
		} else {
			pulse = sys_read32(base + GTCCRC_OFFSET);
		}

		period = reg;
		/* call the callback only when both period and pulse are captured */
		if ((pin_cfg->cnt_isr_trig & 1) != 0) {
			transfer_capture = false;
		}
		break;
	default:
		break;
	}

	reg = sys_read32(base + GTST_OFFSET);
	if (is_channel_b) {
		sys_write32(reg & ~GTST_TCFB, base + GTST_OFFSET);
	} else {
		sys_write32(reg & ~GTST_TCFA, base + GTST_OFFSET);
	}

	k_spin_unlock(&data->lock, key);

	if (!transfer_capture) {
		return;
	}

	if (!is_continuous) {
		rza2m_pwm_disable_capture(dev, pin_cfg->requested_ch);
	}

	if (cb) {
		cb(dev, pin_cfg->requested_ch, period, pulse, 0, user_data);
	}
}

#endif /* CONFIG_PWM_CAPTURE */

static const struct pwm_driver_api pwm_rza2m_driver_api = {
	.set_cycles = rza2m_pwm_set_cycles,
	.get_cycles_per_sec = rza2m_pwm_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = rza2m_pwm_configure_capture,
	.enable_capture = rza2m_pwm_enable_capture,
	.disable_capture = rza2m_pwm_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

static int pwm_rza2m_init(const struct device *dev)
{
	int ret;
	uint32_t tpcs;
	const struct pwm_rza2m_config *config = dev->config;
	mm_reg_t base, reg;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: can't apply default pincontrol state", dev->name);
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("%s: cpg isn't ready", dev->name);
		return -ENODEV;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rzg3s_pwm)
	if (!device_is_ready(config->reset.dev)) {
		LOG_ERR("%s: resets isn't ready", dev->name);
		return -ENODEV;
	}

	(void)reset_line_deassert_dt(&config->reset);
#endif

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		LOG_ERR("%s: can't enable module clock", dev->name);
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	base = DEVICE_MMIO_GET(dev);

	/* stop counter operation */
	reg = sys_read32(base + GTCR_OFFSET);
	sys_write32(reg & ~GTCR_START_CNT, base + GTCR_OFFSET);

	/* set saw-wave mode */
	reg = GTCR_SET_MD(reg, PWM_MD_SAW_WAVE);

	/* set divider */
	tpcs = find_lsb_set(config->divider) >> 1;
	reg = GTCR_SET_TPCS(reg, tpcs);
	sys_write32(reg, base + GTCR_OFFSET);

	if (config->cfg_irqs) {
		config->cfg_irqs(dev);
	}

	return 0;
}

static void pwm_rza2m_ccmpa_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_rza2m_data *data = dev->data;

	pwm_rza2m_ccmp_isr_cmn(dev, &data->pins[PIN_A], false);
#endif /* CONFIG_PWM_CAPTURE */
}

static void pwm_rza2m_ccmpb_isr(struct device *dev)
{
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_rza2m_data *data = dev->data;

	pwm_rza2m_ccmp_isr_cmn(dev, &data->pins[PIN_B], true);
#endif /* CONFIG_PWM_CAPTURE */
}

#define DT_IRQ_HAS_CELL_AT_NAME(node_id, name, cell)                                               \
	IS_ENABLED(DT_CAT6(node_id, _IRQ_NAME_, name, _VAL_, cell, _EXISTS))

#define DT_IRQ_FLAGS_OR_ZERO(inst, name)                                                           \
	COND_CODE_1(DT_IRQ_HAS_CELL_AT_NAME(DT_DRV_INST(inst), name, flags),                       \
		    (DT_INST_IRQ_BY_NAME(inst, name, flags)), (0))

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rzg3s_pwm)
#define PWM_SET_RESET(n)                                                                           \
	.reset = RESET_DT_SPEC_INST_GET(n),
#else
#define PWM_SET_RESET(n)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_rzg3s_pwm) */

#define IRQ_GET_LINE(n, inst) DT_INST_IRQ_BY_IDX(inst, n, irq)
#define GET_IRQ_LINES_ARRAY(inst)                                                                  \
	{                                                                                          \
		LISTIFY(DT_NUM_IRQS(DT_DRV_INST(inst)), IRQ_GET_LINE, (,), inst)                   \
	}

#define PWM_DEVICE_INIT_RZA2M(n)                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct pwm_rza2m_data pwm_rza2m_data_##n;                                           \
                                                                                                   \
	static void config_isr_##n(const struct device *dev)                                       \
	{                                                                                          \
		/* todo: add other IRQs when additional PWM modes are introduced */                \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, ccmpa, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, ccmpa, priority), pwm_rza2m_ccmpa_isr,          \
			    DEVICE_DT_INST_GET(n), DT_IRQ_FLAGS_OR_ZERO(n, ccmpa));                \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, ccmpb, irq),                                    \
			    DT_INST_IRQ_BY_NAME(n, ccmpb, priority), pwm_rza2m_ccmpb_isr,          \
			    DEVICE_DT_INST_GET(n), DT_IRQ_FLAGS_OR_ZERO(n, ccmpb));                \
	}                                                                                          \
                                                                                                   \
	static const struct pwm_rza2m_config pwm_rza2m_config_##n = {                              \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.channel_id = DT_INST_PROP(n, channel_id),                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),                        \
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),                        \
		.bus_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),                        \
		.bus_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),                        \
		.irq_lines = GET_IRQ_LINES_ARRAY(n),                                               \
		.cfg_irqs = config_isr_##n,                                                        \
		.divider = DT_INST_PROP_OR(n, divider, 1),                                         \
		PWM_SET_RESET(n)};                                                                 \
                                                                                                   \
	BUILD_ASSERT(DT_NUM_IRQS(DT_DRV_INST(n)) == ISR_MAX,                                       \
		     "Different interrupt number in dts and driver");                              \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pwm_rza2m_init, NULL, &pwm_rza2m_data_##n, &pwm_rza2m_config_##n, \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &pwm_rza2m_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT_RZA2M)
