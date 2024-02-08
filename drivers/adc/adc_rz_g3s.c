/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_rzg3s_adc

/*
 * *************************** INCLUDES *****************************
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/math_extras.h>
/* This define used for adc_context.h */
#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/*
 * *************************** CONSTANTS *****************************
 */
/* Registers */
#define RZG3S_ADM(n)			((n) * 0x4)
#define RZG3S_TSUMODE			0x10
#define RZG3S_ADINT			0x20
#define RZG3S_ADSTS			0x24
#define RZG3S_ADFIL			0x2c
#define RZG3S_ADCR(n)			(0x30 + ((n) * 0x4))

/* Register bits */
#define RZG3S_ADM0_ADCE			BIT(0)
#define RZG3S_ADM0_ADBSY		BIT(1)
#define RZG3S_ADM0_PWDWNB		BIT(2)
#define RZG3S_ADM0_SRESB		BIT(15)

#define RZG3S_ADM1_TRG			BIT(0)
#define RZG3S_ADM1_MS			BIT(2)
#define RZG3S_ADM1_BS			BIT(4)
#define RZG3S_ADM1_EGA_MASK		GENMASK(13, 12)

#define RZG3S_ADM2_CHSEL_MASK		GENMASK(8, 0)

#define RZG3S_ADM3_ADIL_MASK		GENMASK(31, 24)
#define RZG3S_ADM3_ADCMP_MASK		GENMASK(23, 16)
#define RZG3S_ADM3_ADCMP_1D		FIELD_PREP(RZG3S_ADM3_ADCMP_MASK, 0x1d)
#define RZG3S_ADM3_ADSMP_MASK		GENMASK(7, 0)

#define RZG3S_TSUMODE_TSUEN		BIT(1)

#define RZG3S_ADINT_INTEN_MASK		GENMASK(11, 0)
#define RZG3S_ADINT_CSEEN		BIT(16)
#define RZG3S_ADINT_INTS		BIT(31)

#define RZG3S_ADSTS_CSEST		BIT(16)
#define RZG3S_ADSTS_INTST_MASK		GENMASK(8, 0)

#define RZG3S_ADIVC_DIVADC_MASK		GENMASK(8, 0)
#define RZG3S_ADIVC_DIVADC_4		FIELD_PREP(RZG3S_ADIVC_DIVADC_MASK, 0x4)

#define RZG3S_ADCR_AD_MASK		GENMASK(11, 0)

/* Other constants */
#define RZG3S_ADSMP_DEFAULT_SAMPLING	0x7F
#define RZG3S_ADSMP_DEFAULT_SAMPLING_CH8	0xFF

#define RZG3S_ADC_RESOLUTION		12
/* 8 inputs ADC_CH0 to ADC_CH7 and one Thermal Sensor Unit (TSU) */
#define RZG3S_ADC_CHANNELS_NUM		9

/*
 * **************************** MACROS *******************************
 */
LOG_MODULE_REGISTER(adc_rzg3s, CONFIG_ADC_LOG_LEVEL);

/*
 * *********************** TYPE DECLARATIONS *************************
 */

/**
 * @brief RZ/G3S ADC config
 *
 * This structure contains constant data for given instance of RZ/G3S ADC.
 */
struct adc_rzg3s_config {
	/** Must be first */
	DEVICE_MMIO_ROM;
	/** pointer to clock device for ADC */
	const struct device *clock_dev;
	/** ADC clocks */
	struct renesas_cpg_clk adclk;
	struct renesas_cpg_clk pclk;
	/* reset controllers */
	struct reset_dt_spec adrstn;
	struct reset_dt_spec presetn;
	/** function pointer to irq setup */
	void (*irq_configure)(void);
};

/**
 * @brief RZ/G3S ADC data
 *
 * This structure contains data structures used by a RZ/G3S ADC.
 */
struct adc_rzg3s_data {
	/** Must be first */
	DEVICE_MMIO_RAM;
	/** Structure that handle state of ongoing read operation */
	struct adc_context ctx;
	/** Pointer to RZ/G3S ADC own device structure */
	const struct device *dev;
	/** Pointer to memory where next sample will be written */
	uint16_t *buf;
	/** Pointer to where will be data stored in case of repeated sampling */
	uint16_t *repeat_buf;
	/** Mask with channels that will be sampled */
	uint32_t channels;
};


/*
 * ************************* PRIVATE FUNCTIONS ***************************
 */

/**
 * @brief Function performs write value to RZ/G3S ADC register
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param offs ADC register offset
 * @param value Value to write
 *
 * @return none
 */
static void adc_rzg3s_write(const struct device *dev, uint32_t offs, uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_GET(dev) + offs);
}

/**
 * @brief Function performs read value from RZ/G3S ADC register
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param offs ADC register offset
 *
 * @return ADC register value
 */
static uint32_t adc_rzg3s_read(const struct device *dev, uint32_t offs)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + offs);
}

/**
 * @brief Function for reading ADC conversion result
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param channel ADC channel to read
 *
 * @return ADC conversion result value
 */
static inline uint16_t adc_rzg3s_get_result(const struct device *dev, uint8_t channel)
{
	return (uint16_t)adc_rzg3s_read(dev, RZG3S_ADCR(channel)) & RZG3S_ADCR_AD_MASK;
}

/**
 * @brief Function for reading ADC status
 *
 * @param dev Pointer to RZ/G3S ADC device
 *
 * @return ADC status
 */
static inline uint32_t adc_rzg3s_get_status(const struct device *dev)
{
	return adc_rzg3s_read(dev, RZG3S_ADSTS);
}

/**
 * @brief Function for clearing ADC status
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param mask Clearing bits mask
 *
 * @return none
 */
static inline void adc_rzg3s_clear_status(const struct device *dev, uint32_t mask)
{
	adc_rzg3s_write(dev, RZG3S_ADSTS, mask);
}

/**
 * @brief Function performs ADC software reset
 *
 * @param dev Pointer to RZ/G3S ADC device
 *
 * @return 0 on success, negative errno code on failures
 */
static int adc_rzg3s_sw_reset(const struct device *dev)
{
	uint32_t reg;
	int timeout = 5;

	reg = adc_rzg3s_read(dev, RZG3S_ADM(0));
	reg |= RZG3S_ADM0_SRESB;
	adc_rzg3s_write(dev, RZG3S_ADM(0), reg);

	/* Reset timeout = timeout * 100 us */
	while (!(adc_rzg3s_read(dev, RZG3S_ADM(0)) & RZG3S_ADM0_SRESB)) {
		if (!timeout) {
			return -EBUSY;
		}
		timeout--;
		k_busy_wait(100);
	}

	return 0;
}

/**
 * @brief Function starts/stops ADC conversion
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param on If true, ADC conversion starts and stops otherwise
 *
 * @return none
 */
static void adc_rzg3s_start_stop(const struct device *dev, bool start)
{
	int timeout = 5;
	uint32_t reg;

	reg = adc_rzg3s_read(dev, RZG3S_ADM(0));
	if (start) {
		reg |= RZG3S_ADM0_ADCE;
	} else {
		reg &= ~RZG3S_ADM0_ADCE;
	}
	adc_rzg3s_write(dev, RZG3S_ADM(0), reg);

	if (start) {
		return;
	}

	/* Stop timeout = timeout * 100 us */
	do {
		k_busy_wait(100);
		reg = adc_rzg3s_read(dev, RZG3S_ADM(0));
		timeout--;
		if (!timeout) {
			LOG_ERR("ADC stop timed out\n");
			break;
		}
	} while (((reg & RZG3S_ADM0_ADBSY) || (reg & RZG3S_ADM0_ADCE)));
}

/**
 * @brief Function for changing ADC power mode
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param on If true, release from power-saving mode, if false - enter PS mode
 *
 * @return none
 */
static void adc_rzg3s_set_power(const struct device *dev, bool on)
{
	uint32_t reg;

	if (!on) {
		/* Before entering power-saving mode, check that A/D conversion stopped */
		reg = adc_rzg3s_read(dev, RZG3S_ADM(0));
		if ((reg & RZG3S_ADM0_ADBSY) || (reg & RZG3S_ADM0_ADCE)) {
			adc_rzg3s_start_stop(dev, false);
		}
	}

	reg = adc_rzg3s_read(dev, RZG3S_ADM(0));
	if (on) {
		/* Release from power-saving mode */
		reg |= RZG3S_ADM0_PWDWNB;
	} else {
		/* Enter power-saving mode */
		reg &= ~RZG3S_ADM0_PWDWNB;
	}
	adc_rzg3s_write(dev, RZG3S_ADM(0), reg);
	/* Stabilization wait time */
	k_busy_wait(2);
}

/**
 * @brief Check if buffer in @p sequence is big enough to hold all ADC samples
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param sequence Pointer to ADC sequence description
 *
 * @return 0 on success
 * @return -ENOMEM if buffer is not big enough
 */
static int adc_rzg3s_check_buffer_size(const struct device *dev,
				     const struct adc_sequence *sequence)
{
	uint8_t channels = 0;
	size_t needed_size;
	uint32_t mask;

	/* Calculate number of channels in sequence */
	for (mask = BIT(RZG3S_ADC_CHANNELS_NUM - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed_size = channels * sizeof(uint16_t);
	if (sequence->options) {
		needed_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_size) {
		return -ENOMEM;
	}

	return 0;
}

/**
 * @brief Start A/D conversion request
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param sequence Pointer to ADC sequence description
 *
 * @return 0 on success
 * @return -ENOTSUP if requested resolution or channel is outside of supported range
 * @return -ENOMEM if buffer is not big enough (see @ref adc_rpi_check_buffer_size)
 * @return other error code returned by adc_context_wait_for_completion
 */
static int adc_rzg3s_start_conversion(const struct device *dev,
			      const struct adc_sequence *sequence)
{
	struct adc_rzg3s_data *data = dev->data;
	int err;

	if (sequence->resolution != RZG3S_ADC_RESOLUTION) {
		LOG_ERR("%s: supported only 12-bit resolution", dev->name);
		return -ENOTSUP;
	}

	if (!sequence->channels ||
	    find_msb_set(sequence->channels) > RZG3S_ADC_CHANNELS_NUM) {
		LOG_ERR("unsupported channels in mask: 0x%08x", sequence->channels);
		return -ENOTSUP;
	}

	err = adc_rzg3s_check_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buf = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

/**
 * @brief API function for pre-configure ADC channel
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param channel_cfg Pointer to single channel configuration
 *
 * @return 0 on success
 * @return -ENOTSUP if requested channel ID or mode is outside of supported range
 * @return -EINVAL if acquisition time or gain is not valid
 */
static int adc_rzg3s_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	if (channel_cfg->channel_id >= RZG3S_ADC_CHANNELS_NUM) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Acquisition time is not valid");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("unsupported differential mode");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -EINVAL;
	}

	return 0;
}

static int adc_rzg3s_conversion_setup(const struct device *dev, uint32_t chan_mask)
{
	uint32_t reg;
	uint8_t last_channel = find_msb_set(chan_mask) - 1;

	if (adc_rzg3s_read(dev, RZG3S_ADM(0)) & RZG3S_ADM0_ADBSY)
		return -EBUSY;

	/*
	 * Setup ADM1 for SW trigger
	 * TRGEN[21:16] - Unused, set 00
	 * EGA[13:12] - Set 00 to indicate hardware trigger is invalid
	 * BS[4] - Enable 1-buffer mode
	 * RPS[3] - Select single conversion number
	 * MS[2] - Enable Scan mode
	 * TRGIN[1] - Trigger input mode unused
	 * TRG[0] - Enable software trigger mode
	 */
	adc_rzg3s_write(dev, RZG3S_ADM(1), 0);

	/* Select analog input channels subjected to conversion. */
	adc_rzg3s_write(dev, RZG3S_ADM(2), RZG3S_ADM2_CHSEL_MASK & chan_mask);

	/*
	 * Setup AMD3
	 * ADIL[31:24] - Should be always set to 0
	 * ADCMP[23:16] - Should be always set to 0x1D
	 * ADSMP[15:0] - Set default sampling period for channels
	 */
	reg = RZG3S_ADM3_ADCMP_1D;
	reg |= ((last_channel == 8) ? RZG3S_ADSMP_DEFAULT_SAMPLING_CH8 :
				      RZG3S_ADSMP_DEFAULT_SAMPLING);
	adc_rzg3s_write(dev, RZG3S_ADM(3), reg);

	/*
	 * Setup ADINT
	 * INTS[31] - Select pulse signal
	 * CSEEN[16] - Enable channel select error interrupt
	 * INTEN[11:0] - Select channel interrupt for last channel in sequence
	 */

	reg = (RZG3S_ADINT_CSEEN | BIT(last_channel));
	adc_rzg3s_write(dev, RZG3S_ADINT, reg);

	return 0;
}

/**
 * @brief API function for performing A/D conversion in asynchronous mode
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param sequence Pointer to ADC sequence description
 * @param async Pointer to conversion complete callback,
 *	  if NULL, conversion will performed in synchronous mode
 *
 * @return 0 on success, negative errno code on failures
 */
static int adc_rzg3s_conversion_async(const struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct adc_rzg3s_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = adc_rzg3s_start_conversion(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

/**
 * @brief API function for performing A/D conversion in synchronous mode
 *
 * @param dev Pointer to RZ/G3S ADC device
 * @param sequence Pointer to ADC sequence description
 *
 * @return 0 on success, negative errno code on failures
 */
static int adc_rzg3s_conversion(const struct device *dev,
			const struct adc_sequence *sequence)
{
	return adc_rzg3s_conversion_async(dev, sequence, NULL);
}

/**
 * @brief Function called on init for each ADC device. It setups all
 *        channels to return constant 0 mV and create acquisition thread.
 *
 * @param dev Pointer to RZ/G3S ADC device
 *
 * @return 0 on success
 */
static int adc_rzg3s_init(const struct device *dev)
{
	const struct adc_rzg3s_config *config = dev->config;
	struct adc_rzg3s_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Device %s is not ready\n", dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->pclk);
	if (ret != 0) {
		LOG_ERR("Can't turn on pclk clock for %s\n", dev->name);
		return ret;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->adclk);
	if (ret != 0) {
		LOG_ERR("Can't turn on adclk clock for %s\n", dev->name);
		goto err;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* HW and software reset */
	(void)reset_line_toggle_dt(&config->adrstn);
	(void)reset_line_toggle_dt(&config->presetn);
	ret = adc_rzg3s_sw_reset(dev);
	if (ret != 0) {
		LOG_ERR("Device %s reset timeout\n", dev->name);
		goto err;
	}

	config->irq_configure();

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
err:
	/* On error */
	clock_control_off(config->clock_dev, (clock_control_subsys_t)&config->adclk);
	clock_control_off(config->clock_dev, (clock_control_subsys_t)&config->pclk);
	return ret;
}

/**
 * @brief Interrupt handler
 *
 * @param dev Pointer to RZ/G3S ADC device
 *
 * @return none
 */
static void adc_rzg3s_isr(const struct device *dev)
{
	struct adc_rzg3s_data *data = dev->data;
	uint32_t status = adc_rzg3s_get_status(dev);
	uint8_t first_chan = find_lsb_set(data->channels) - 1;
	uint8_t last_chan = find_msb_set(data->channels) - 1;
	uint8_t ch;

	/* Abort converting if error detected or no conversion end flag for current channel */
	if ((status & RZG3S_ADSTS_CSEST) || !(status && data->channels)) {
		adc_context_complete(&data->ctx, -EIO);
		/* Clear status and enter power-saving mode */
		adc_rzg3s_clear_status(dev, status);
		adc_rzg3s_set_power(data->dev, false);
		return;
	}

	/* Get conversion results from all read channels, copy to buffer */
	for (ch = first_chan; ch <= last_chan; ++ch) {
		if (data->channels & BIT(ch)) {
			*data->buf++ = adc_rzg3s_get_result(dev, ch);
		}
	}

	adc_rzg3s_clear_status(dev, status);

	/* Enter power-saving mode */
	adc_rzg3s_set_power(data->dev, false);
	/* Notify result if all data gathered. */
	adc_context_on_sampling_done(&data->ctx, dev);
}

/*
 * ************************* PUBLIC FUNCTIONS ***************************
 */
void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_rzg3s_data *data = CONTAINER_OF(ctx, struct adc_rzg3s_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buf = data->buf;

	adc_rzg3s_set_power(data->dev, true);

	/* Setup channels and start conversion */
	adc_rzg3s_conversion_setup(data->dev, data->channels);
	adc_rzg3s_start_stop(data->dev, true);
}

void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_rzg3s_data *data = CONTAINER_OF(ctx, struct adc_rzg3s_data, ctx);

	if (repeat_sampling) {
		data->buf = data->repeat_buf;
	}
}

/*
 * ************************* DRIVER REGISTER SECTION ***************************
 */
#define ADC_RZG3S_INIT(n)							   \
	static void adc_rzg3s_irq_configure_func_##n(void)			   \
	{									   \
		IRQ_CONNECT(DT_INST_IRQN(n),					   \
			    DT_INST_IRQ(n, priority),				   \
			    adc_rzg3s_isr,					   \
			    DEVICE_DT_INST_GET(n), 0);				   \
		irq_enable(DT_INST_IRQN(n));					   \
	}									   \
	static struct adc_driver_api adc_rzg3s_api_##n = {			   \
		.channel_setup = adc_rzg3s_channel_setup,			   \
		.read = adc_rzg3s_conversion,					   \
		.ref_internal = DT_INST_PROP(n, vref_mv),			   \
		IF_ENABLED(CONFIG_ADC_ASYNC,					   \
			   (.read_async = adc_rzg3s_conversion_async,))		   \
	};									   \
	static const struct adc_rzg3s_config adc_rzg3s_config_##n = {		   \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),				   \
		.irq_configure = adc_rzg3s_irq_configure_func_##n,		   \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		   \
		.adclk.module = DT_INST_CLOCKS_CELL_BY_NAME(n, adclk, module),     \
		.adclk.domain = DT_INST_CLOCKS_CELL_BY_NAME(n, adclk, domain),     \
		.pclk.module = DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, module),	   \
		.pclk.domain = DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, domain),	   \
		.presetn = RESET_DT_SPEC_INST_GET_BY_IDX(n, 0),			   \
		.adrstn = RESET_DT_SPEC_INST_GET_BY_IDX(n, 1),			   \
	};									   \
	static struct adc_rzg3s_data adc_rzg3s_data_##n = {			   \
		ADC_CONTEXT_INIT_TIMER(adc_rzg3s_data_##n, ctx),		   \
		ADC_CONTEXT_INIT_LOCK(adc_rzg3s_data_##n, ctx),			   \
		ADC_CONTEXT_INIT_SYNC(adc_rzg3s_data_##n, ctx),			   \
		.dev = DEVICE_DT_INST_GET(n),					   \
	};									   \
										   \
	DEVICE_DT_INST_DEFINE(n, adc_rzg3s_init, NULL,				   \
			      &adc_rzg3s_data_##n,				   \
			      &adc_rzg3s_config_##n, POST_KERNEL,		   \
			      CONFIG_ADC_INIT_PRIORITY,				   \
			      &adc_rzg3s_api_##n)

DT_INST_FOREACH_STATUS_OKAY(ADC_RZG3S_INIT);
