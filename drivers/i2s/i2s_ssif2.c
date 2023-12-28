/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#define DT_DRV_COMPAT renesas_ssif2

LOG_MODULE_REGISTER(i2s_ssif);

struct stream {
	int32_t state;
	struct k_spinlock lock;
	struct i2s_config cfg;
	const struct device *dev_dma;
	uint32_t dma_channel;
	uint32_t dma_conf;
	uint8_t dma_slot;
	struct dma_config dma_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	int (*stream_start)(struct stream *stream, const struct device *dev);
	void (*stream_disable)(struct stream *stream, const struct device *dev);
	void (*queue_drop)(struct stream *stream);
	struct k_msgq queue;
	void *mem_block;
	int master;
	const char *name;
};

/* Device const configuration */
struct ssif_config {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
	int fifo_depth;
	uint32_t audio_clk_rate;
	bool clock_stop_enable;
};

/* Device run time data */
struct ssif_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct stream rx;
	struct stream tx;
	void *rx_queue[CONFIG_I2S_RENESAS_SSIF_RX_BLOCK_COUNT];
	void *tx_queue[CONFIG_I2S_RENESAS_SSIF_TX_BLOCK_COUNT];
};

#define FIFO_DEPTH 32

#define SSICR			0x00		/* Control register */
#define SSISR			0x04		/* Status register */
#define SSIFCR			0x10		/* FIFO control register */
#define SSIFSR			0x14		/* FIFO status register */
#define SSIFTDR			0x18		/* Transmit FIFO data register */
#define SSIFRDR			0x1c		/* Receive FIFO data register */
#define SSIOFR			0x20		/* Audio format register */
#define SSISCR			0x24		/* Status control register */

#define SSICR_CKS		BIT(30)		/* Selects an audio clock for master-mode
						 * communication
						 */
#define SSICR_TUEIN		BIT(29)		/* Transmit underflow interrupt output enable */
#define SSICR_TOIEN		BIT(28)		/* Transmit overflow interrupt output enable */
#define SSICR_RUIEN		BIT(27)		/* Receive underflow interrupt output enable */
#define SSICR_ROIEN		BIT(26)		/* Receive overflow interrupt output enable */
#define SSICR_IIEN		BIT(25)		/* Idle mode interrupt output enable */
#define SSICR_MST		BIT(14)		/* Master enable */
#define SSICR_BCKP		BIT(13)		/* Selects bit clock polarity */
#define SSICR_LRCKP		BIT(12)		/* Selects the initial value and polarity
						 * of LRCK/FS
						 */
#define SSICR_SPDP		BIT(11)		/* Selects serial padding polarity */
#define SSICR_SDTA		BIT(10)		/* Selects serial data alignment */
#define SSICR_PDTA		BIT(9)		/* Selects placement data alignment */
#define SSICR_DEL		BIT(8)		/* Selects serial data delay */
#define SSICR_MUEN		BIT(3)		/* Mute (silent) enable */
#define SSICR_TEN		BIT(1)		/* Transmission enable */
#define SSICR_REN		BIT(0)		/* Reception enable */
#define SSICR_FRM_MASK		GENMASK(23, 22)
#define SSICR_FRM_SHIFT		22
#define SSICR_DWL_MASK		GENMASK(21, 19)
#define SSICR_DWL_SHIFT		19
#define SSICR_SWL_MASK		GENAMSK(18, 16)
#define SSICR_SWL_SHIFT		16
#define SSICR_CKDV_MASK		GENAMSK(7, 4)
#define SSICR_CKDV_SHIFT	4

#define SSISR_TUIRQ		BIT(29)		/* Transmit underflow error status flag */
#define SSISR_TOIRQ		BIT(28)		/* Transmit overflow error status flag */
#define SSISR_RUIRQ		BIT(27)		/* Receive underflow error status flag */
#define SSISR_ROIRQ		BIT(26)		/* Receive overflow error status flag */
#define SSISR_IIRQ		BIT(25)		/* Idle mode status flag */

#define SSIFSR_TDE		BIT(16)		/* Transmit Data Empty Flag */
#define SSIFSR_TDC_MASK		GENMASK(29, 24)
#define SSIFSR_TDC_SHIFT	24
#define SSIFSR_RDF		BIT(0)		/* Receive Data Full Flag */
#define SSIFSR_RDC_MASK		GENMASK(13, 8)
#define SSIFSR_RDC_SHIFT	8

#define SSISCR_RDFS_MASK	GENMASK(4, 0)
#define SSISCR_RDFS_SHIFT	0
#define SSISCR_TDES_MASK	GENMASK(12, 8)
#define SSISCR_TDES_SHIFT	8

#define SSIF_CFG_CKDV_BITS_1    (0x00)
#define SSIF_CFG_CKDV_BITS_2    (0x01)
#define SSIF_CFG_CKDV_BITS_4    (0x02)
#define SSIF_CFG_CKDV_BITS_8    (0x03)
#define SSIF_CFG_CKDV_BITS_16   (0x04)
#define SSIF_CFG_CKDV_BITS_32   (0x05)
#define SSIF_CFG_CKDV_BITS_64   (0x06)
#define SSIF_CFG_CKDV_BITS_128  (0x07)
#define SSIF_CFG_CKDV_BITS_6    (0x08)
#define SSIF_CFG_CKDV_BITS_12   (0x09)
#define SSIF_CFG_CKDV_BITS_24   (0x0a)
#define SSIF_CFG_CKDV_BITS_48   (0x0b)
#define SSIF_CFG_CKDV_BITS_96   (0x0c)

#define SSIF_AUDIO_CLK_DIV_1    (1u)
#define SSIF_AUDIO_CLK_DIV_2    (2u)
#define SSIF_AUDIO_CLK_DIV_4    (4u)
#define SSIF_AUDIO_CLK_DIV_8    (8u)
#define SSIF_AUDIO_CLK_DIV_16   (16u)
#define SSIF_AUDIO_CLK_DIV_32   (32u)
#define SSIF_AUDIO_CLK_DIV_64   (64u)
#define SSIF_AUDIO_CLK_DIV_128  (128u)
#define SSIF_AUDIO_CLK_DIV_6    (6u)
#define SSIF_AUDIO_CLK_DIV_12   (12u)
#define SSIF_AUDIO_CLK_DIV_24   (24u)
#define SSIF_AUDIO_CLK_DIV_48   (48u)
#define SSIF_AUDIO_CLK_DIV_96   (96u)

#define SSISR_TUIRQ		BIT(29)		/* Transmit underflow error status flag */
#define SSISR_TOIRQ		BIT(28)		/* Transmit overflow error status flag */
#define SSISR_RUIRQ		BIT(27)		/* Receive underflow error status flag*/
#define SSISR_ROIRQ		BIT(26)		/* Receive overflow error status flag */
#define SSISR_IIRRQ		BIT(25)		/* Idle mode status flag */
#define SSIFCR_AUCKE		BIT(31)		/* AUDIO_MCK Enable in Master-mode Communication */
#define SSIFCR_SSIRST		BIT(16)		/* Software Reset */
#define SSIFCR_BSW		BIT(11)		/* Byte Swap Enable */
#define SSIFCR_BCKNCE		BIT(10)		/* Noise Canceller Enable in Slave-mode
						 * Communication (SSIBCK)
						 */

#define SSIFCR_LRCKNCE		BIT(9)		/* Noise Canceller Enable in Slave-mode
						 * Communication (SSILRCK/SSIFS)
						 */

#define SSIFCR_RXDNCE		BIT(8)		/* Received Data Input Noise Canceller Enable
						 * in Slave-mode Communication (SSIRxD)
						 */
#define SSIFCR_TIE		BIT(3)		/* Transmit Data Empty Interrupt Output Enable */
#define SSIFCR_RIE		BIT(2)		/* Receive Data Full Interrupt Output Enable */
#define SSIFCR_TFRST		BIT(1)		/* Transmit FIFO data register reset */
#define SSIFCR_RFRST		BIT(1)		/* Receive FIFO data register reset */

#define SSIFSR_TDE		BIT(16)		/* Transmit Data Empty Flag */
#define SSIFSR_RDF		BIT(0)		/* Receive Data Full Flag */

#define SSIOFR_BCKASTP		BIT(9)		/* BCK Output Stop Enable */
#define SSIOFR_LRCONT		BIT(8)		/* LRCK/FS Continuation Enable */
#define SSIOFR_OMOD		(BIT(1) | BIT(0)) /* Audio Format Select */
#define SSIOFR_I2S		0		/* I2S format */
#define SSIOFR_TDM		1		/* TDM format */
#define SSIOFR_MONAURAL		2		/* Monaural format */

#define RX_SRC_INCREMENT	0
#define RX_DST_INCREMENT	1
#define TX_SRC_INCREMENT	1
#define TX_DST_INCREMENT	0

#define DMA_NUM_CHANNELS		8

static uint32_t ssif_read_reg(const struct device *dev, uint32_t offs)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + offs);
}

static void ssif_write_reg(const struct device *dev, uint32_t offs, uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_GET(dev) + offs);
}

static inline void ssif_clear_set_bit(const struct device *dev, uint32_t offs,
				      uint32_t clear, uint32_t set)
{
	ssif_write_reg(dev, offs, (ssif_read_reg(dev, offs) & ~clear) | set);
}

static int ssif_configure(const struct device *dev, enum i2s_dir dir,
			  const struct i2s_config *i2s_cfg)
{
	struct ssif_data *const data = dev->data;
	const struct ssif_config *const cfg = dev->config;
	uint32_t system_word = i2s_cfg->word_size >= 16U ? 32U : 16U;
	uint32_t syswd_per_smp = 2;
	int channel_div;
	struct stream *p_stream;
	uint32_t dot_clk, dwl, swl, division;
	uint16_t ckdv;
	uint32_t reset_mask;

	switch (dir) {
	case I2S_DIR_RX:
		reset_mask = SSIFCR_RFRST;
		p_stream = &data->rx;
		break;
	case I2S_DIR_TX:
		reset_mask = SSIFCR_TFRST;
		p_stream = &data->tx;
		break;
	case I2S_DIR_BOTH:
		LOG_ERR("I2S_DIR_BOTH is not supported for now");
		return -EINVAL;
	default:
		LOG_ERR("Invalid direction value");
		return -EINVAL;
	}

	p_stream->master = true;

	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE ||
	    i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE) {
		p_stream->master = false;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		memset(&p_stream->cfg, 0, sizeof(struct i2s_config));
		p_stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	if (i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) {
		LOG_ERR("invalid operating mode");
		return -EINVAL;
	}

	if (i2s_cfg->channels == 1) {
		channel_div = 2;
	} else if (i2s_cfg->channels == 2) {
		channel_div = 1;
	} else {
		LOG_ERR("invalid channels number");
		return -EINVAL;
	}
	switch (i2s_cfg->word_size) {
	case 8:
		dwl = 0;
		break;
	case 16:
		dwl = 1;
		break;
	case 18:
		dwl = 2;
		break;
	case 20:
		dwl = 3;
		break;
	case 22:
		dwl = 4;
		break;
	case 24:
		dwl = 5;
		break;
	case 32:
		dwl = 6;
		break;
	default:
		LOG_ERR("Unsupported word size %d", i2s_cfg->word_size);
		return -EINVAL;
	}

	swl = (system_word - 8) / 8;

	int req_buf_s = (cfg->fifo_depth * (i2s_cfg->word_size / 8)) / channel_div;

	if (i2s_cfg->block_size < req_buf_s) {
		LOG_ERR("not enough space to allocate single buffer");
		LOG_ERR("fifo requires at least %i bytes", req_buf_s);
		return -EINVAL;
	}

	dot_clk = system_word * syswd_per_smp * i2s_cfg->frame_clk_freq;
	division = cfg->audio_clk_rate / dot_clk;

	switch (division) {
	case SSIF_AUDIO_CLK_DIV_1:
		ckdv = SSIF_CFG_CKDV_BITS_1;
		break;
	case SSIF_AUDIO_CLK_DIV_2:
		ckdv = SSIF_CFG_CKDV_BITS_2;
		break;
	case SSIF_AUDIO_CLK_DIV_4:
		ckdv = SSIF_CFG_CKDV_BITS_4;
		break;
	case SSIF_AUDIO_CLK_DIV_8:
		ckdv = SSIF_CFG_CKDV_BITS_8;
		break;
	case SSIF_AUDIO_CLK_DIV_16:
		ckdv = SSIF_CFG_CKDV_BITS_16;
		break;
	case SSIF_AUDIO_CLK_DIV_32:
		ckdv = SSIF_CFG_CKDV_BITS_32;
		break;
	case SSIF_AUDIO_CLK_DIV_64:
		ckdv = SSIF_CFG_CKDV_BITS_64;
		break;
	case SSIF_AUDIO_CLK_DIV_128:
		ckdv = SSIF_CFG_CKDV_BITS_128;
		break;
	case SSIF_AUDIO_CLK_DIV_6:
		ckdv = SSIF_CFG_CKDV_BITS_6;
		break;
	case SSIF_AUDIO_CLK_DIV_12:
		ckdv = SSIF_CFG_CKDV_BITS_12;
		break;
	case SSIF_AUDIO_CLK_DIV_24:
		ckdv = SSIF_CFG_CKDV_BITS_24;
		break;
	case SSIF_AUDIO_CLK_DIV_48:
		ckdv = SSIF_CFG_CKDV_BITS_48;
		break;
	case SSIF_AUDIO_CLK_DIV_96:
		ckdv = SSIF_CFG_CKDV_BITS_96;
		break;
	default:
		{
			static uint8_t ckdvs[] = { 1,  2,  4,  8, 16, 32, 64, 128,
						   6, 12, 24, 48, 96 };
			char msg[128];
			int index = sprintf(msg, "Supported (stereo) rates are: ");
			int i;

			for (i = 0; i < ARRAY_SIZE(ckdvs); i++) {
				index += sprintf(msg + index, "%d ",
						cfg->audio_clk_rate / ckdvs[i] / system_word / 2);
			}
			LOG_ERR("%s", msg);
			return -EINVAL;
		}
	}

	p_stream->stream_disable(p_stream, dev);
	p_stream->queue_drop(p_stream);

	ssif_clear_set_bit(dev, SSIFCR, 0, reset_mask);

	ssif_write_reg(dev, SSIOFR, 0);
	ssif_write_reg(dev, SSICR, (ckdv << SSICR_CKDV_SHIFT) | (swl << SSICR_SWL_SHIFT) |
		       (dwl << SSICR_DWL_SHIFT));
	if (p_stream->master) {
		uint32_t ofr_reg;

		ofr_reg = SSIOFR_LRCONT;
		if (cfg->clock_stop_enable) {
			ofr_reg |= SSIOFR_BCKASTP;
		}
		ssif_write_reg(dev, SSIOFR, ofr_reg);
		ssif_clear_set_bit(dev, SSICR, 0, SSICR_MST);
		ssif_clear_set_bit(dev, SSIFCR, 0, SSIFCR_AUCKE);
	}
	ssif_clear_set_bit(dev, SSIFCR, reset_mask, 0);

	memcpy(&p_stream->cfg, i2s_cfg, sizeof(struct i2s_config));
	p_stream->state = I2S_STATE_READY;
	return 0;
}

static const struct i2s_config *ssif_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct ssif_data *const data = dev->data;
	struct stream *p_stream;

	switch (dir) {
	case I2S_DIR_TX:
		p_stream = &data->tx;
		break;
	case I2S_DIR_RX:
		p_stream = &data->rx;
		break;
	default:
		return NULL;
	}

	if (p_stream->state == I2S_STATE_NOT_READY) {
		return NULL;
	}

	return &p_stream->cfg;
}

static void ssif_set_enable(const struct device *dev, enum i2s_dir dir)
{
	uint32_t mask = 0;
	uint32_t primary_bit, secondary_bit, data_bit;

	switch (dir) {
	case I2S_DIR_TX:
		primary_bit = SSICR_TEN;
		secondary_bit = SSICR_REN;
		data_bit = SSIFSR_TDE;
		break;
	case I2S_DIR_RX:
		primary_bit = SSICR_REN;
		secondary_bit = SSICR_TEN;
		data_bit = SSIFSR_RDF;
		break;
	default:
		return;
	};
	mask = ssif_read_reg(dev, SSICR);
	if (mask & primary_bit) {
		return;
	}
	mask = 0;
	if (!(ssif_read_reg(dev, SSISR) & SSISR_IIRQ)) {
		ssif_clear_set_bit(dev, SSICR, secondary_bit, 0);
		k_busy_wait(100);
		mask = secondary_bit;
	}
	mask |= primary_bit;
	ssif_clear_set_bit(dev, SSICR, 0, mask);
	ssif_clear_set_bit(dev, SSIFSR, data_bit, 0);
}

static int ssif_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct ssif_data *const data = dev->data;
	void *buffer;
	int ret;

	if (data->rx.state == I2S_STATE_NOT_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	/* Get data from the beginning of RX queue */
	ret = k_msgq_get(&data->rx.queue, &buffer, SYS_TIMEOUT_MS(data->rx.cfg.timeout));
	if (ret) {
		if (data->rx.state == I2S_STATE_ERROR) {
			ret = -EIO;
		} else {
			ret = -EAGAIN;
		}
	}
	*mem_block = buffer;
	*size = data->rx.cfg.block_size;
	return ret;
}

static int ssif_write(const struct device *dev, void *mem_block, size_t size)
{
	struct ssif_data *const data = dev->data;

	if (data->tx.state != I2S_STATE_RUNNING && data->tx.state != I2S_STATE_READY) {
		LOG_DBG("invalid state");
		return -EIO;
	}

	return k_msgq_put(&data->tx.queue, &mem_block, SYS_TIMEOUT_MS(data->tx.cfg.timeout));
}

typedef int (*trigger_operation_t)(const struct device *dev, struct stream *stream);

static int ssif_trigger_start(const struct device *dev, struct stream *p_stream)
{
	int ret;

	if (p_stream->state != I2S_STATE_READY) {
		LOG_ERR("START trigger %s: invalid state %d", p_stream->name, p_stream->state);
		return -EIO;
	}

	ret = p_stream->stream_start(p_stream, dev);
	if (ret < 0) {
		LOG_ERR("START trigger %s failed %d", p_stream->name, ret);
		return ret;
	}
	p_stream->state = I2S_STATE_RUNNING;
	return 0;
}

static int ssif_trigger_stop(const struct device *dev, struct stream *p_stream)
{
	if (p_stream->state != I2S_STATE_RUNNING) {
		LOG_ERR("STOP %s trigger invalid state %d", p_stream->name, p_stream->state);
		return -EIO;
	}
	p_stream->stream_disable(p_stream, dev);
	p_stream->state = I2S_STATE_READY;
	return 0;
}

static int ssif_trigger_drop(const struct device *dev, struct stream *p_stream)
{
	if (p_stream->state == I2S_STATE_NOT_READY) {
		LOG_ERR("DROP trigger %s: invalid state %d", p_stream->name, p_stream->state);
		return -EIO;
	}
	p_stream->stream_disable(p_stream, dev);
	p_stream->queue_drop(p_stream);
	p_stream->state = I2S_STATE_READY;
	return 0;
}

static int ssif_trigger_drain(const struct device *dev, struct stream *p_stream)
{
	if (p_stream->state != I2S_STATE_RUNNING) {
		LOG_ERR("DRAIN trigger %s: invalid state %d", p_stream->name, p_stream->state);
		return -EIO;
	}
	p_stream->stream_disable(p_stream, dev);
	p_stream->queue_drop(p_stream);
	p_stream->state = I2S_STATE_READY;
	return 0;
}

static int ssif_trigger_prepare(const struct device *dev, struct stream *p_stream)
{
	if (p_stream->state != I2S_STATE_ERROR) {
		LOG_ERR("PREPARE trigger %s: invalid state %d", p_stream->name, p_stream->state);
		return -EIO;
	}
	p_stream->state = I2S_STATE_READY;
	p_stream->queue_drop(p_stream);
	return 0;
}

static int ssif_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct ssif_data *const data = dev->data;
	int ret;
	trigger_operation_t trigger;
	struct stream *p_stream;
	k_spinlock_key_t key;

	switch (dir) {
	case I2S_DIR_BOTH:
		return -EINVAL;
	case I2S_DIR_RX:
		p_stream = &data->rx;
		break;
	case I2S_DIR_TX:
		p_stream = &data->tx;
		break;
	default:
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		trigger = ssif_trigger_start;
		break;
	case I2S_TRIGGER_STOP:
		trigger = ssif_trigger_stop;
		break;
	case I2S_TRIGGER_DRAIN:
		trigger = ssif_trigger_drain;
		break;
	case I2S_TRIGGER_DROP:
		trigger = ssif_trigger_drop;
		break;
	case I2S_TRIGGER_PREPARE:
		trigger = ssif_trigger_prepare;
		break;
	default:
		LOG_ERR("unsupported trigger command");
		return -EINVAL;
	}
	key = k_spin_lock(&p_stream->lock);
	ret = trigger(dev, p_stream);
	k_spin_unlock(&p_stream->lock, key);
	return ret;
}

/*
 * Communication error/idle mode error
 */
static void ssif_isr_int_req(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	uint32_t reg_val = ssif_read_reg(dev, SSISR);

	if (reg_val & ~(SSISR_IIRQ)) {
		LOG_ERR("%s: err=%x", __func__, reg_val);
		reg_val &= ~(SSISR_TUIRQ | SSISR_TOIRQ | SSISR_RUIRQ | SSISR_ROIRQ);
		ssif_write_reg(dev, SSISR, reg_val);
	}
}

/*
 * This driver works only with DMA. When DMA is active it reroutes interrupt requests
 * to itself and those interupts can't reach to interrupt controller and isrs can't be called.
 */
/*
 * Receive data full interrupt
 */
static void ssif_isr_dma_rx(void *arg)
{
}

/*
 * Transmit data empty interrupt
 */
static void ssif_isr_dma_tx(void *arg)
{
}

/*
 * Transmit/Receive interrupt
 */
__attribute__((unused)) static void ssif_isr_dma_rt(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	if (ssif_read_reg(dev, SSIFSR) & SSIFSR_TDE) {
		ssif_isr_dma_tx((void *)dev);
	}
	if (ssif_read_reg(dev, SSIFSR) & SSIFSR_RDF) {
		ssif_isr_dma_rx((void *)dev);
	}
}

static int rx_stream_start(struct stream *p_stream, const struct device *dev)
{
	int ret;

	ret = k_mem_slab_alloc(p_stream->cfg.mem_slab, &p_stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		goto err;
	}

	ssif_clear_set_bit(dev, SSICR, 0, SSICR_RUIEN);
	p_stream->blk_cfg.block_size = p_stream->cfg.block_size;
	p_stream->blk_cfg.dest_address = (uint32_t)(p_stream->mem_block);
	p_stream->blk_cfg.fifo_mode_control = 0;

	p_stream->dma_cfg.head_block = &p_stream->blk_cfg;
	ret = dma_config(p_stream->dev_dma, p_stream->dma_channel, &p_stream->dma_cfg);
	if (ret < 0) {
		goto err;
	}

	sys_cache_data_invd_range(p_stream->mem_block, p_stream->cfg.block_size);

	ret = dma_start(p_stream->dev_dma, p_stream->dma_channel);
	if (ret < 0) {
		goto err;
	}

	ssif_set_enable(dev, I2S_DIR_RX);
	ssif_clear_set_bit(dev, SSIFCR, 0, SSIFCR_RIE);

	return 0;
err:
	ssif_clear_set_bit(dev, SSICR, SSICR_RUIEN, 0);
	if (p_stream->mem_block) {
		k_mem_slab_free(p_stream->cfg.mem_slab, p_stream->mem_block);
		p_stream->mem_block = NULL;
	}
	return ret;
}

static void rx_stream_disable(struct stream *p_stream, const struct device *dev)
{
	ssif_clear_set_bit(dev, SSICR, SSICR_RUIEN | SSICR_ROIEN, 0);
	ssif_clear_set_bit(dev, SSIFCR, SSIFCR_RIE, 0);
	ssif_clear_set_bit(dev, SSICR, SSICR_REN, 0);
	dma_stop(p_stream->dev_dma, p_stream->dma_channel);
	if (p_stream->mem_block) {
		k_mem_slab_free(p_stream->cfg.mem_slab, p_stream->mem_block);
		p_stream->mem_block = NULL;
	}
}

static void rx_queue_drop(struct stream *p_stream)
{
	void *mem_block;

	while (k_msgq_get(&p_stream->queue, &mem_block, K_NO_WAIT) == 0) {
		k_mem_slab_free(p_stream->cfg.mem_slab, mem_block);
	}

}

static int tx_stream_start(struct stream *p_stream, const struct device *dev)
{
	int ret;

	ret = k_msgq_get(&p_stream->queue, &p_stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}
	ssif_clear_set_bit(dev, SSICR, 0, SSICR_TUEIN | SSICR_TOIEN);
	p_stream->blk_cfg.block_size = p_stream->cfg.block_size;
	p_stream->blk_cfg.source_address = (uint32_t)(p_stream->mem_block);
	p_stream->blk_cfg.fifo_mode_control = 0;

	p_stream->dma_cfg.head_block = &p_stream->blk_cfg;
	ret = dma_config(p_stream->dev_dma, p_stream->dma_channel, &p_stream->dma_cfg);
	if (ret < 0) {
		goto err;
	}

	sys_cache_data_flush_range((void *)p_stream->mem_block, p_stream->cfg.block_size);
	ret = dma_start(p_stream->dev_dma, p_stream->dma_channel);
	if (ret) {
		goto err;
	}
	ssif_set_enable(dev, I2S_DIR_TX);
	ssif_clear_set_bit(dev, SSIFCR, 0, SSIFCR_TIE);
	return 0;
err:
	ssif_clear_set_bit(dev, SSICR, SSICR_TUEIN | SSICR_TOIEN, 0);
	if (p_stream->mem_block) {
		k_mem_slab_free(p_stream->cfg.mem_slab, p_stream->mem_block);
		p_stream->mem_block = NULL;
	}
	return ret;
}

static void tx_stream_disable(struct stream *p_stream, const struct device *dev)
{
	ssif_clear_set_bit(dev, SSIFCR, SSIFCR_TIE, 0);
	ssif_clear_set_bit(dev, SSICR, SSICR_TEN, 0);
	ssif_clear_set_bit(dev, SSICR, SSICR_TUEIN | SSICR_TOIEN, 0);

	dma_stop(p_stream->dev_dma, p_stream->dma_channel);
	if (p_stream->mem_block) {
		k_mem_slab_free(p_stream->cfg.mem_slab, p_stream->mem_block);
		p_stream->mem_block = NULL;
	}
}

static void tx_queue_drop(struct stream *p_stream)
{
	void *mem_block;

	while (k_msgq_get(&p_stream->queue, &mem_block, K_NO_WAIT) == 0) {
		k_mem_slab_free(p_stream->cfg.mem_slab, mem_block);
	}
}

static void dma_rx_callback(const struct device *dma_dev, void *user_data,
			    uint32_t channel, int status)
{
	struct device *dev = (struct device *)user_data;
	struct ssif_data *const data = dev->data;
	struct stream *stream = &data->rx;
	void *mblk_tmp = NULL;
	int ret;

	if (status < 0) {
		ret = -EIO;
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block);

	/* Stop reception if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		goto rx_disable;
	}

	mblk_tmp = stream->mem_block;

	/* Prepare to receive the next data block */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		stream->mem_block = NULL;
		goto rx_disable;
	}

	stream->blk_cfg.dest_address = (uint32_t)(stream->mem_block);
	stream->dma_cfg.head_block = &stream->blk_cfg;

	ret = dma_config(stream->dev_dma, stream->dma_channel, &stream->dma_cfg);
	if (ret < 0) {
		goto rx_disable;
	}

	sys_cache_data_invd_range(stream->mem_block, stream->cfg.block_size);

	ssif_clear_set_bit(dev, SSIFSR, SSIFSR_RDF, 0);
	ssif_clear_set_bit(dev, SSIFCR, SSIFCR_RIE, 0);

	ret = dma_start(stream->dev_dma, stream->dma_channel);
	if (ret < 0) {
		LOG_DBG("Failed to start RX DMA transfer: %d", ret);
		goto rx_disable;
	}

	ssif_clear_set_bit(dev, SSIFCR, 0, SSIFCR_RIE);

	/* All block data received */
	ret = k_msgq_put(&stream->queue, &mblk_tmp, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	return;
rx_disable:
	if (mblk_tmp) {
		k_mem_slab_free(stream->cfg.mem_slab, mblk_tmp);
	}
	if (stream->mem_block) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
	rx_stream_disable(stream, dev);
}

static void dma_tx_callback(const struct device *dev_dev, void *user_data,
			    uint32_t channel, int status)
{
	struct device *dev = (struct device *)user_data;
	struct ssif_data *const data = dev->data;
	struct stream *stream = &data->tx;
	int ret;

	if (status < 0) {
		stream->state = I2S_STATE_ERROR;
		goto tx_disable;
	}

	__ASSERT_NO_MSG(stream->mem_block);

	/* All block data sent */
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;

	/* Stop transmission if there was an error */
	if (stream->state == I2S_STATE_ERROR) {
		LOG_ERR("TX error detected");
		goto tx_disable;
	}

	/* Prepare to send the next data block */
	ret = k_msgq_get(&stream->queue, &stream->mem_block, K_NO_WAIT);
	if (ret < 0) {
		stream->state = I2S_STATE_READY;
		goto tx_disable;
	}

	stream->blk_cfg.block_size = stream->cfg.block_size;
	stream->blk_cfg.source_address = (uint32_t)(stream->mem_block);
	stream->dma_cfg.head_block = &stream->blk_cfg;
	ret = dma_config(stream->dev_dma, stream->dma_channel, &stream->dma_cfg);
	if (ret < 0) {
		goto tx_disable;
	}

	sys_cache_data_flush_range((void *)stream->mem_block, stream->cfg.block_size);

	ssif_clear_set_bit(dev, SSIFCR, SSIFCR_TIE, 0);
	ssif_clear_set_bit(dev, SSIFSR, SSIFSR_TDE, 0);

	ret = dma_start(stream->dev_dma, stream->dma_channel);
	if (ret < 0) {
		LOG_DBG("Failed to start TX DMA transfer: %d", ret);
		goto tx_disable;
	}
	ssif_clear_set_bit(dev, SSIFCR, 0, SSIFCR_TIE);
	return;
tx_disable:
	if (stream->state == I2S_STATE_READY || stream->state == I2S_STATE_RUNNING) {
		/* Wait until all data will be sent */
		while (ssif_read_reg(dev, SSIFSR) & SSIFSR_TDC_MASK) {
		}
	}
	tx_stream_disable(stream, dev);
}

static const struct i2s_driver_api ssif_driver_api = {
	.configure = ssif_configure,
	.config_get = ssif_config_get,
	.read = ssif_read,
	.write = ssif_write,
	.trigger = ssif_trigger,
};

static int ssif_initialize(const struct device *dev)
{
	const struct ssif_config *cfg = dev->config;
	struct ssif_data *const data = dev->data;
	int ret = 0;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)&cfg->mod_clk);
	if (ret < 0) {
		LOG_ERR("Can't get clock rate\n");
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* RX stream configure */
	k_msgq_init(&data->rx.queue, (char *)data->rx_queue, sizeof(void *),
		    CONFIG_I2S_RENESAS_SSIF_RX_BLOCK_COUNT);
	/* Configure dma rx config */
	memset(&data->rx.blk_cfg, 0, sizeof(data->rx.blk_cfg));
	data->rx.blk_cfg.source_address = (uint32_t)DEVICE_MMIO_ROM_PTR(dev)->phys_addr + SSIFRDR;
	data->rx.blk_cfg.dest_address = 0; /* dest not ready */

	if (data->rx.src_addr_increment) {
		data->rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (data->rx.dst_addr_increment) {
		data->rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* RX disable circular buffer */
	data->rx.blk_cfg.source_reload_en = 0;
	data->rx.blk_cfg.dest_reload_en = 0;
	data->rx.blk_cfg.fifo_mode_control = data->rx.fifo_threshold;

	data->rx.dma_cfg.head_block = &data->rx.blk_cfg;
	data->rx.dma_cfg.user_data = (void *)dev;
	data->rx.dma_cfg.dma_slot = data->rx.dma_slot;

	/* TX stream configure */
	k_msgq_init(&data->tx.queue, (char *)data->tx_queue, sizeof(void *),
		    CONFIG_I2S_RENESAS_SSIF_TX_BLOCK_COUNT);
	/* Configure dma tx config */
	memset(&data->tx.blk_cfg, 0, sizeof(data->tx.blk_cfg));

	data->tx.blk_cfg.dest_address = (uint32_t)DEVICE_MMIO_ROM_PTR(dev)->phys_addr + SSIFTDR;

	data->tx.blk_cfg.source_address = 0; /* not ready */

	if (data->tx.src_addr_increment) {
		data->tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (data->tx.dst_addr_increment) {
		data->tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	data->tx.blk_cfg.fifo_mode_control = data->tx.fifo_threshold;

	data->tx.dma_cfg.head_block = &data->tx.blk_cfg;
	data->tx.dma_cfg.user_data = (void *)dev;
	data->tx.dma_cfg.dma_slot = data->tx.dma_slot;

	cfg->irq_config(dev);
	return 0;
}

/* src_dev and dest_dev should be 'MEMORY' or 'PERIPHERAL'. */
#define I2S_DMA_CHANNEL_INIT(inst, dir, dir_cap, src_dev, dest_dev)	\
.dir = {								\
	.dev_dma = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, dir)),	\
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(inst, dir, channel),	\
	.dma_conf = DT_INST_DMAS_CELL_BY_NAME(inst, dir, config),	\
	.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, dir, slot),		\
	.dma_cfg = {							\
		.block_count = 1,					\
		.dma_slot = 0,						\
		.channel_direction = src_dev##_TO_##dest_dev,		\
		.source_data_size = 2,  /* 16bit default */		\
		.dest_data_size = 2,    /* 16bit default */		\
		.source_burst_length = 1, /* SINGLE transfer */		\
		.dest_burst_length = 1,					\
		.channel_priority = 0,					\
		.dma_callback = dma_##dir##_callback,			\
	},								\
	.stream_start = dir##_stream_start,				\
	.stream_disable = dir##_stream_disable,				\
	.src_addr_increment = dir_cap##_SRC_INCREMENT,			\
	.dst_addr_increment = dir_cap##_DST_INCREMENT,			\
	.queue_drop = dir##_queue_drop,					\
	.name = #dir_cap						\
}

#define INST_IRQ(inst, label)						\
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, label, irq),		\
		    DT_INST_IRQ_BY_NAME(inst, label, priority),		\
		    ssif_isr_##label,					\
		    DEVICE_DT_INST_GET(inst),				\
		    DT_INST_IRQ_BY_NAME(inst, label, flags));		\
	irq_enable(DT_INST_IRQ_BY_NAME(inst, label, irq));		\

#define IRQ_DEFINE(inst, label) COND_CODE_1(DT_IRQ_HAS_NAME(DT_DRV_INST(inst), label),\
					   (INST_IRQ(inst, label);), (EMPTY))

#define SSIF_INIT(inst)										\
	static struct ssif_data ssif_data_##inst = {						\
		UTIL_AND(DT_INST_DMAS_HAS_NAME(inst, rx),					\
			I2S_DMA_CHANNEL_INIT(inst, rx, RX, PERIPHERAL, MEMORY)),		\
		UTIL_AND(DT_INST_DMAS_HAS_NAME(inst, tx),					\
			I2S_DMA_CHANNEL_INIT(inst, tx, TX, MEMORY, PERIPHERAL)),		\
	};											\
	static void ssif_irq_config_func_##inst(const struct device *dev);			\
	PINCTRL_DT_INST_DEFINE(inst);								\
	static struct ssif_config ssif_cfg_##inst = {						\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),					\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),				\
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_NAME(inst, fck, module),		\
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_NAME(inst, fck, domain),		\
		.bus_clk.module = DT_INST_CLOCKS_CELL_BY_NAME(inst, bus, module),		\
		.bus_clk.domain = DT_INST_CLOCKS_CELL_BY_NAME(inst, bus, domain),		\
		.audio_clk_rate = DT_INST_PROP_BY_PHANDLE_IDX(inst, clocks, 2, clock_frequency),\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),					\
		.irq_config = ssif_irq_config_func_##inst,					\
		.fifo_depth = FIFO_DEPTH,							\
		.clock_stop_enable = DT_INST_PROP(inst, clock_stop_enable),			\
	};											\
	DEVICE_DT_INST_DEFINE(inst, ssif_initialize,						\
				NULL, &ssif_data_##inst,					\
				&ssif_cfg_##inst, POST_KERNEL,					\
				CONFIG_I2S_INIT_PRIORITY,					\
				&ssif_driver_api);						\
	static void ssif_irq_config_func_##inst(const struct device *dev)			\
	{											\
		IRQ_DEFINE(inst, int_req);							\
		IRQ_DEFINE(inst, dma_tx);							\
		IRQ_DEFINE(inst, dma_rx);							\
		IRQ_DEFINE(inst, dma_rt);							\
	}

DT_INST_FOREACH_STATUS_OKAY(SSIF_INIT)
