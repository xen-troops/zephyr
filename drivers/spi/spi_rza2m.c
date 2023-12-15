/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rspi_rz

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_rza2m);

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/cache.h>
#ifdef CONFIG_SPI_ASYNC
#include <zephyr/drivers/dma.h>
#endif /* CONFIG_SPI_ASYNC */

#include "spi_context.h"

#define LOG_DEV_ERR(dev, format, ...)	LOG_ERR("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...)	LOG_WRN("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...)	LOG_INF("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...)	LOG_DBG("%s:"#format, (dev)->name, ##__VA_ARGS__)

#define RSPI_SPCR		0x00	/* Control Register */
#define RSPI_SSLP		0x01	/* Slave Select Polarity Register */
#define RSPI_SPPCR		0x02	/* Pin Control Register */
#define RSPI_SPSR		0x03	/* Status Register */
#define RSPI_SPDR		0x04	/* Data Register */
#define RSPI_SPSCR		0x08	/* Sequence Control Register */
#define RSPI_SPSSR		0x09	/* Sequence Status Register */
#define RSPI_SPBR		0x0a	/* Bit Rate Register */
#define RSPI_SPDCR		0x0b	/* Data Control Register */
#define RSPI_SPCKD		0x0c	/* Clock Delay Register */
#define RSPI_SSLND		0x0d	/* Slave Select Negation Delay Register */
#define RSPI_SPND		0x0e	/* Next-Access Delay Register */
#define RSPI_SPCMD0		0x10	/* Command Register 0 */
#define RSPI_SPCMD1		0x12	/* Command Register 1 */
#define RSPI_SPCMD2		0x14	/* Command Register 2 */
#define RSPI_SPCMD3		0x16	/* Command Register 3 */
#define RSPI_SPBFCR		0x20	/* Buffer Control Register */
#define RSPI_SPBFDR		0x22	/* Buffer Data Count Setting Register */

/* SPCR - Control Register */
#define SPCR_SPRIE		BIT(7)	/* Receive Interrupt Enable */
#define SPCR_SPE		BIT(6)	/* Function Enable */
#define SPCR_SPTIE		BIT(5)	/* Transmit Interrupt Enable */
#define SPCR_SPEIE		BIT(4)	/* Error Interrupt Enable */
#define SPCR_MSTR		BIT(3)	/* Master/Slave Mode Select */
#define SPCR_MODFEN		BIT(2)	/* Mode Fault Error Detection Enable */

/* SSLP - Slave Select Polarity Register */
#define SSLP_SSLP		BIT(0)	/* SSL Signal Polarity Setting */

/* SPPCR - Pin Control Register */
#define SPPCR_MOIFE		BIT(5)	/* MOSI Idle Value Fixing Enable */
#define SPPCR_MOIFV		BIT(4)	/* MOSI Idle Fixed Value */
#define SPPCR_SPLP		BIT(0)	/* Loopback Mode (inverting) */

/* SPSR - Status Register */
#define SPSR_SPRF		BIT(7)	/* Receive Buffer Full Flag */
#define SPSR_TEND		BIT(6)	/* Transmit End */
#define SPSR_SPTEF		BIT(5)	/* Transmit Buffer Empty Flag */
#define SPSR_MODF		BIT(2)	/* Mode Fault Error Flag */
#define SPSR_OVRF		BIT(0)	/* Overrun Error Flag (RSPI only) */

/* SPSCR - Sequence Control Register */
#define SPSCR_SPSLN_MASK	0x03	/* Sequence Length Specification */

/* SPSSR - Sequence Status Register */
#define SPSSR_SPCP_MASK		0x03	/* Command Pointer Mask */

/* SPDCR - Data Control Register */
#define SPDCR_TXDMY		BIT(7)	/* Dummy Data Transmission Enable */
#define SPDCR_SPLW1		BIT(6)	/* Access Width Specification (RZ) */
#define SPDCR_SPLW0		BIT(5)	/* Access Width Specification (RZ) */
#define SPDCR_SPLLWORD		(SPDCR_SPLW1 | SPDCR_SPLW0)
#define SPDCR_SPLWORD		SPDCR_SPLW1
#define SPDCR_SPLBYTE		SPDCR_SPLW0

/* SPCKD - Clock Delay Register */
#define SPCKD_SCKDL_MASK	0x07	/* Clock Delay Setting (1-8) */

/* SSLND - Slave Select Negation Delay Register */
#define SSLND_SLNDL_MASK	0x07	/* SSL Negation Delay Setting (1-8) */

/* SPND - Next-Access Delay Register */
#define SPND_SPNDL_MASK		0x07	/* Next-Access Delay Setting (1-8) */

/* SPCMDn - Command Registers */
#define SPCMD_SCKDEN		BIT(15)	/* Clock Delay Setting Enable */
#define SPCMD_SLNDEN		BIT(14)	/* SSL Negation Delay Setting Enable */
#define SPCMD_SPNDEN		BIT(13)	/* Next-Access Delay Enable */
#define SPCMD_LSBF		BIT(12)	/* LSB First */
#define SPCMD_SPB_MASK		GENMASK(11, 8)	/* Data Length Setting */
#define SPCMD_SPB_SHIFT		8
#define SPCMD_SSLKP		BIT(7)	/* SSL Signal Level Keeping */
#define SPCMD_BRDV_MASK		GENMASK(3, 2)	/* Bit Rate Division Setting */
#define SPCMD_BRDV_SHIFT	2
#define SPCMD_CPOL		BIT(1)	/* Clock Polarity Setting */
#define SPCMD_CPHA		BIT(0)	/* Clock Phase Setting */

/* SPBFCR - Buffer Control Register */
#define SPBFCR_TXRST		BIT(7)	/* Transmit Buffer Data Reset */
#define SPBFCR_RXRST		BIT(6)	/* Receive Buffer Data Reset */
#define SPBFCR_TXTRG_MASK	GENMASK(5, 4)	/* Transmit Buffer Data Triggering Number */
#define SPBFCR_RXTRG_MASK	GENMASK(2, 0)	/* Receive Buffer Data Triggering Number */

/* SPBFDR - Buffer Data Count Setting Register */
#define SPBFDR_TXTRG_MASK	GENMASK(11, 8)	/* number of bytes of data to be transmitted */
#define SPBFDR_RXTRG_MASK	GENMASK(5, 0)	/* number of bytes of received data */

struct rza2m_spi_cfg {
	DEVICE_MMIO_ROM;
	const uint16_t			*reg_offset;
	const struct device		*clk_dev;
	struct renesas_cpg_clk		clk_mod;
	const struct pinctrl_dev_config *pcfg;
	uint32_t			num_cs;
	void				(*irq_config)(void);
#ifdef CONFIG_SPI_ASYNC
	const struct device *dma_dev;
	uint8_t tx_dma_slot;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_slot;
	uint8_t rx_dma_channel;
#endif /* CONFIG_SPI_ASYNC */
};

struct rza2m_spi_ctx {
	DEVICE_MMIO_RAM;
	struct spi_context	spi_ctx;
	uint32_t		clk_rate;
	uint32_t		speed_hz;
	uint8_t			dfs;
	uint8_t			r_spcr;
	uint8_t			r_sslp;
	uint8_t			r_sppcr;
	uint8_t			r_spbr;
	uint8_t			r_spdcr;
	uint16_t		r_spcmd;

	struct k_sem		sem_rx;
	struct k_sem		sem_tx;

#ifdef CONFIG_SPI_ASYNC
	uint32_t dma_segment_len;
#endif
};

static void rza2m_spi_write8(const struct device *dev, uint8_t data, uint16_t reg_ofs)
{
	sys_write8(data, DEVICE_MMIO_GET(dev) + reg_ofs);
}

static uint8_t rza2m_spi_read8(const struct device *dev, uint16_t reg_ofs)
{
	return sys_read8(DEVICE_MMIO_GET(dev) + reg_ofs);
}

static void rza2m_spi_write16(const struct device *dev, uint16_t data, uint16_t reg_ofs)
{
	sys_write16(data, DEVICE_MMIO_GET(dev) + reg_ofs);
}

static void rza2m_spi_enable_irq(const struct device *dev, uint8_t en_msk)
{
	rza2m_spi_write8(dev, rza2m_spi_read8(dev, RSPI_SPCR) | en_msk, RSPI_SPCR);
}

static void rza2m_spi_disable_irq(const struct device *dev, uint8_t dis_msk)
{
	rza2m_spi_write8(dev, rza2m_spi_read8(dev, RSPI_SPCR) & ~dis_msk, RSPI_SPCR);
}

static void rza2m_spi_irq_rx(const struct device *dev)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	uint8_t spsr;

	spsr = rza2m_spi_read8(dev, RSPI_SPSR);
	if (spsr & SPSR_SPRF) {
		rza2m_spi_disable_irq(dev, SPCR_SPRIE);
		k_sem_give(&ctx->sem_rx);
	}
}

static void rza2m_spi_irq_tx(const struct device *dev)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	uint8_t spsr;

	spsr = rza2m_spi_read8(dev, RSPI_SPSR);
	if (spsr & SPSR_SPTEF) {
		rza2m_spi_disable_irq(dev, SPCR_SPTIE);
		k_sem_give(&ctx->sem_tx);
	}
}

static void rza2m_spi_set_rate(struct rza2m_spi_ctx *ctx)
{
	int brdv = 0, spbr;

	spbr = DIV_ROUND_UP(ctx->clk_rate, 2 * ctx->speed_hz) - 1;
	while (spbr > 255 && brdv < 3) {
		brdv++;
		spbr = ((spbr + 1) >> 1) - 1;
	}

	ctx->r_spbr = (spbr > 255) ? 255 : spbr;
	ctx->r_spcmd |= (brdv << SPCMD_BRDV_SHIFT) & SPCMD_BRDV_MASK;
	ctx->speed_hz = DIV_ROUND_UP(ctx->clk_rate, (2U << brdv) * (spbr + 1));
}

static int rza2m_spi_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;

	if (spi_context_configured(spi_ctx, spi_cfg)) {
		return 0;
	}

	if (spi_cfg->operation & SPI_OP_MODE_SLAVE) {
		LOG_DEV_ERR(dev, "Slave mode not yet supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		LOG_DEV_ERR(dev, "Half-duplex not supported");
		return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_DEV_ERR(dev, "Only single line mode is supported");
		return -ENOTSUP;
	}

	ctx->r_sppcr = 0;
	if (spi_cfg->operation & SPI_MODE_LOOP) {
		ctx->r_sppcr |= SPPCR_SPLP;
	}

	ctx->r_sslp = (spi_cfg->operation & SPI_CS_ACTIVE_HIGH) ? SSLP_SSLP : 0;

	ctx->r_spcmd = 0;

	switch (SPI_WORD_SIZE_GET(spi_cfg->operation)) {
	case 8:
		ctx->dfs = 1;
		ctx->r_spcmd |= (0x7 << SPCMD_SPB_SHIFT);
		ctx->r_spdcr = SPDCR_SPLBYTE;
		break;
	case 16:
		ctx->dfs = 2;
		ctx->r_spcmd |= (0xf << SPCMD_SPB_SHIFT);
		ctx->r_spdcr = SPDCR_SPLWORD;
		break;
	case 32:
		ctx->dfs = 4;
		ctx->r_spcmd |= (0x3 << SPCMD_SPB_SHIFT);
		ctx->r_spdcr = SPDCR_SPLLWORD;
		break;
	default:
		LOG_DEV_ERR(dev, "Word size not supported");
		return -ENOTSUP;
	}

	ctx->r_spcmd |= (spi_cfg->operation & SPI_MODE_CPOL) ? SPCMD_CPOL : 0;
	ctx->r_spcmd |= (spi_cfg->operation & SPI_MODE_CPHA) ? SPCMD_CPHA : 0;
	ctx->r_spcmd |= (spi_cfg->operation & SPI_TRANSFER_LSB) ? SPCMD_LSBF : 0;
	ctx->r_spcmd |= (spi_cfg->operation & SPI_HOLD_ON_CS) ? SPCMD_SSLKP : 0;

	ctx->r_spcr = SPCR_MSTR | SPCR_SPE;

	ctx->speed_hz = spi_cfg->frequency;

	rza2m_spi_set_rate(ctx);

	LOG_DEV_DBG(dev, "trns: fr:%u fa:%u cmd:%04x",
		    spi_cfg->frequency,
		    ctx->speed_hz,
		    ctx->r_spcmd);

	/* SSL polarity */
	rza2m_spi_write8(dev, ctx->r_sslp, RSPI_SSLP);

	/* Sets output mode, MOSI signal, and (optionally) loopback */
	rza2m_spi_write8(dev, ctx->r_sppcr, RSPI_SPPCR);

	/* Sets transfer bit rate */
	rza2m_spi_write8(dev, ctx->r_spbr, RSPI_SPBR);

	/* Disable dummy transmission, set byte access */
	rza2m_spi_write8(dev, ctx->r_spdcr, RSPI_SPDCR);

	/* Sets RSPCK, SSL, next-access delay value */
	rza2m_spi_write8(dev, 0x00, RSPI_SPCKD);
	rza2m_spi_write8(dev, 0x00, RSPI_SSLND);
	rza2m_spi_write8(dev, 0x00, RSPI_SPND);

	/* Resets sequencer */
	rza2m_spi_write8(dev, 0, RSPI_SPSCR);
	rza2m_spi_write16(dev, ctx->r_spcmd, RSPI_SPCMD0);

	/* Sets RSPI mode */
	rza2m_spi_write8(dev, ctx->r_spcr, RSPI_SPCR);

	ctx->spi_ctx.config = spi_cfg;

	return 0;
}

static void rza2m_spi_write_data(const struct device *dev, const uint8_t *tx_buf)
{
	struct rza2m_spi_ctx *ctx = dev->data;

	if (ctx->dfs == 1) {
		sys_write8(*tx_buf, DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	} else if (ctx->dfs == 2) { /* 16 bit */
		sys_write16((*(uint16_t *)tx_buf), DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	} else { /* 32 bit */
		sys_write32((*(uint32_t *)tx_buf), DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	}
}

static void rza2m_spi_read_data(const struct device *dev, uint8_t *rx_buf)
{
	struct rza2m_spi_ctx *ctx = dev->data;

	if (ctx->dfs == 1) {
		*rx_buf = sys_read8(DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	} else if (ctx->dfs == 2) { /* 16 bit */
		(*(uint16_t *)rx_buf) = sys_read16(DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	} else { /* 32 bit */
		(*(uint32_t *)rx_buf) = sys_read32(DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	}
}

static void rza2m_spi_read_data_lb(const struct device *dev, uint8_t *rx_buf)
{
	struct rza2m_spi_ctx *ctx = dev->data;

	/* In loopback mode data is inverted in loopback path */
	if (ctx->dfs == 1) {
		*rx_buf = ~sys_read8(DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	} else if (ctx->dfs == 2) { /* 16 bit */
		(*(uint16_t *)rx_buf) = ~sys_read16(DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	} else { /* 32 bit */
		(*(uint32_t *)rx_buf) = ~sys_read32(DEVICE_MMIO_GET(dev) + RSPI_SPDR);
	}
}

static void rza2m_spi_receive_init(const struct device *dev)
{
	uint32_t data;
	uint8_t spsr;

	spsr = rza2m_spi_read8(dev, RSPI_SPSR);
	if (spsr & SPSR_SPRF) {
		rza2m_spi_read_data(dev, (uint8_t *)&data);
	}
	/* dummy read */
	if (spsr & SPSR_OVRF) {
		rza2m_spi_write8(dev, rza2m_spi_read8(dev, RSPI_SPSR) & ~SPSR_OVRF, RSPI_SPSR);
	}
}

static int rza2m_spi_wait_for_irq(const struct device *dev, uint8_t wait_mask, uint8_t en_bit,
				  struct k_sem *sem)
{
	uint8_t spsr;
	int ret;

	spsr = rza2m_spi_read8(dev, RSPI_SPSR);
	if (spsr & wait_mask) {
		return 0;
	}

	LOG_DEV_DBG(dev, "wait %X %X %X\n", spsr, wait_mask, en_bit);

	rza2m_spi_enable_irq(dev, en_bit);
	ret = k_sem_take(sem, K_MSEC(1));

	return ret;
}

static inline int rza2m_spi_wait_for_tx_empty(const struct device *dev)
{
	struct rza2m_spi_ctx *ctx = dev->data;

	return rza2m_spi_wait_for_irq(dev, SPSR_SPTEF, SPCR_SPTIE, &ctx->sem_tx);
}

static inline int rza2m_spi_wait_for_rx_full(const struct device *dev)
{
	struct rza2m_spi_ctx *ctx = dev->data;

	return rza2m_spi_wait_for_irq(dev, SPSR_SPRF, SPCR_SPRIE, &ctx->sem_rx);
}

static int rza2m_spi_data_out(const struct device *dev, const uint8_t *tx_buf)
{
	int error = rza2m_spi_wait_for_tx_empty(dev);

	if (error < 0) {
		LOG_DEV_ERR(dev, "transmit timeout");
		return error;
	}

	rza2m_spi_write_data(dev, tx_buf);
	return 0;
}

static int rza2m_spi_data_in(const struct device *dev, uint8_t *rx_buf)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	int error;

	error = rza2m_spi_wait_for_rx_full(dev);
	if (error) {
		LOG_DEV_ERR(dev, "receive timeout %d", error);
		return error;
	}

	if (!(ctx->spi_ctx.config->operation & SPI_MODE_LOOP)) {
		rza2m_spi_read_data(dev, rx_buf);
	} else {
		rza2m_spi_read_data_lb(dev, rx_buf);
	}
	return 0;
}

static int rza2m_spi_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;
	uint32_t data;
	int ret = 0;

	spi_context_lock(spi_ctx, false, NULL, NULL, spi_cfg);
	ret = rza2m_spi_configure(dev, spi_cfg);
	if (ret) {
		goto exit_err_cfg;
	}

	spi_context_buffers_setup(spi_ctx, tx_bufs, rx_bufs, ctx->dfs);
	spi_context_cs_control(spi_ctx, true);

	rza2m_spi_receive_init(dev);
	rza2m_spi_write8(dev, SPBFCR_TXRST | SPBFCR_RXRST, RSPI_SPBFCR);
	rza2m_spi_write8(dev, 0, RSPI_SPBFCR);

	while (spi_context_tx_buf_on(spi_ctx) || spi_context_rx_buf_on(spi_ctx)) {
		if (spi_context_tx_buf_on(spi_ctx)) {
			ret = rza2m_spi_data_out(dev, spi_ctx->tx_buf);
			if (ret) {
				goto exit_err_trx;
			}
		} else {
			data = 0;
			rza2m_spi_data_out(dev, (uint8_t *)&data);
		}
		spi_context_update_tx(spi_ctx, ctx->dfs, 1);

		if (spi_context_rx_buf_on(spi_ctx)) {
			ret = rza2m_spi_data_in(dev, spi_ctx->rx_buf);
			if (ret) {
				goto exit_err_trx;
			}
		} else {
			rza2m_spi_data_in(dev, (uint8_t *)&data);
		}
		spi_context_update_rx(spi_ctx, ctx->dfs, 1);
	}

	/* Wait for the last transmission */
	ret = rza2m_spi_wait_for_tx_empty(dev);
	if (ret) {
		LOG_DEV_ERR(dev, "transmit timeout\n");
	}

exit_err_trx:
	spi_context_cs_control(spi_ctx, false);

exit_err_cfg:
	spi_context_release(spi_ctx, ret);
	return ret;
}

static int rza2m_spi_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;

	if (!spi_context_configured(spi_ctx, spi_cfg)) {
		LOG_DEV_ERR(dev, "SPI configuration was not the last one to be used");
		return -EINVAL;
	}

	rza2m_spi_write8(dev, 0x0, RSPI_SPCR);

	spi_context_unlock_unconditionally(spi_ctx);

	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static void rza2m_spi_dma_rx_done(const struct device *dma_dev, void *arg,
				 uint32_t id, int error_code);

/**
 * @brief Function configure RX DMA channel for receive and start it
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param buf Pointer to the RX buffer
 * @param len Length of receive buffer
 *
 * @retval 0 if operation successful, negative errno code on failures
 */
static int rza2m_spi_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct rza2m_spi_cfg *cfg = dev->config;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_blk = { 0 };
	static uint8_t dummy;
	int retval;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = rza2m_spi_dma_rx_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->rx_dma_slot;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.dest_address = (uint32_t) buf;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		/* If RX buffer is absent, use static dummy variable as destination */
		dma_blk.dest_address = (uint32_t) &dummy;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.source_address = (uint32_t)DEVICE_MMIO_ROM_PTR(dev)->phys_addr + RSPI_SPDR;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
	if (retval != 0) {
		return retval;
	}

	sys_cache_data_invd_range((void *)buf, len);

	/* Enable RX interrupt */
	rza2m_spi_write8(dev, rza2m_spi_read8(dev, RSPI_SPCR) | SPCR_SPRIE, RSPI_SPCR);

	return dma_start(cfg->dma_dev, cfg->rx_dma_channel);
}

/**
 * @brief Function configure TX DMA channel for transmitting and start it
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param buf Pointer to the TX buffer
 * @param len Length of transmit buffer
 *
 * @retval 0 if operation successful, negative errno code on failures
 */
static int rza2m_spi_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct rza2m_spi_cfg *cfg = dev->config;
	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_blk = { 0 };
	static const uint8_t dummy;
	int retval;

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->tx_dma_slot;

	dma_blk.block_size = len;

	if (buf != NULL) {
		dma_blk.source_address = (uint32_t) buf;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		/* If TX buffer is absent, use static dummy variable as source */
		dma_blk.source_address = (uint32_t) &dummy;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_blk.dest_address = (uint32_t)DEVICE_MMIO_ROM_PTR(dev)->phys_addr + RSPI_SPDR;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel, &dma_cfg);
	if (retval != 0) {
		return retval;
	}

	sys_cache_data_flush_range((void *)buf, len);

	/* Enable TX interrupt */
	rza2m_spi_write8(dev, rza2m_spi_read8(dev, RSPI_SPCR) | SPCR_SPTIE, RSPI_SPCR);

	return dma_start(cfg->dma_dev, cfg->tx_dma_channel);
}

/**
 * @brief Function set dma_segment_len as minimum from the RX and TX
 *	  buffers length
 *
 * @param dev Pointer to the device structure for the driver instance
 *
 * @retval true if operation successful, false on error.
 */
static bool rza2m_spi_dma_advance_segment(const struct device *dev)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;
	uint32_t segment_len;

	/* Pick the shorter buffer of ones that have an actual length */
	if (spi_ctx->rx_len != 0) {
		segment_len = spi_ctx->rx_len;
		if (spi_ctx->tx_len != 0) {
			segment_len = MIN(segment_len, spi_ctx->tx_len);
		}
	} else {
		segment_len = spi_ctx->tx_len;
	}

	if (segment_len == 0) {
		return false;
	}

	segment_len = MIN(segment_len, 65535);

	ctx->dma_segment_len = segment_len;
	return true;
}

/**
 * @brief Function load RX and TX buffers in proper order
 *
 * @param dev Pointer to the device structure for the driver instance
 *
 * @retval 0 if operation successful, negative errno code on failure
 */
static int rza2m_spi_dma_advance_buffers(const struct device *dev)
{
	struct rza2m_spi_ctx *ctx = dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;
	uint32_t dma_segment_len = ctx->dma_segment_len;
	int retval;

	if (dma_segment_len == 0) {
		return -EINVAL;
	}

	/* Load receive buffer first, so it can accept transmit data */
	if (spi_ctx->rx_len) {
		retval = rza2m_spi_dma_rx_load(dev, spi_ctx->rx_buf, dma_segment_len);
	} else {
		retval = rza2m_spi_dma_rx_load(dev, NULL, dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	/* Now load the transmit buffer, which starts the actual bus clocking */
	if (spi_ctx->tx_len) {
		retval = rza2m_spi_dma_tx_load(dev, spi_ctx->tx_buf, dma_segment_len);
	} else {
		retval = rza2m_spi_dma_tx_load(dev, NULL, dma_segment_len);
	}

	if (retval != 0) {
		return retval;
	}

	return 0;
}

/**
 * @brief Callback function for DMA RX operation
 *
 * @param dma_dev Pointer to the device structure for the DMA driver instance
 * @param user_data Pointer to the dma_cfg.user_data
 * @param channel The channel number
 * @param status DMA operation status
 *
 * @retval void
 */
static void rza2m_spi_dma_rx_done(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status)
{
	const struct device *spi_dev = user_data;
	const struct rza2m_spi_cfg *cfg = spi_dev->config;
	struct rza2m_spi_ctx *ctx = spi_dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;
	int retval;

	ARG_UNUSED(channel);
	ARG_UNUSED(status);

	/* In loopback mode data is inverted in loopback path */
	if (ctx->spi_ctx.config->operation & SPI_MODE_LOOP) {
		for (size_t i = 0; i < spi_ctx->rx_len; ++i) {
			spi_ctx->rx_buf[i] = ~spi_ctx->rx_buf[i];
		}
	}

	spi_context_update_tx(spi_ctx, 1, ctx->dma_segment_len);
	spi_context_update_rx(spi_ctx, 1, ctx->dma_segment_len);

	if (!rza2m_spi_dma_advance_segment(spi_dev)) {
		/* Done, restore interrupts */
		rza2m_spi_write8(spi_dev, ctx->r_spcr, RSPI_SPCR);
		spi_context_cs_control(spi_ctx, false);
		spi_context_complete(spi_ctx, spi_dev, 0);
		return;
	}

	retval = rza2m_spi_dma_advance_buffers(spi_dev);
	if (retval != 0) {
		/* Error, stop DMA, restore interrupts, call callback */
		dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
		dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
		rza2m_spi_write8(spi_dev, ctx->r_spcr, RSPI_SPCR);
		spi_context_cs_control(spi_ctx, false);
		spi_context_complete(spi_ctx, spi_dev, retval);
		return;
	}
}

static int rza2m_spi_transceive_async(const struct device *dev,
				      const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs,
				      spi_callback_t cb,
				      void *userdata)
{
	const struct rza2m_spi_cfg *cfg = dev->config;
	struct rza2m_spi_ctx *ctx = dev->data;
	struct spi_context *spi_ctx = &ctx->spi_ctx;
	int retval;

	spi_context_lock(spi_ctx, true, cb, userdata, config);

	retval = rza2m_spi_configure(dev, config);
	if (retval != 0) {
		goto err_unlock;
	}

	/* Assert SSL signal */
	spi_context_cs_control(spi_ctx, true);

	/* TODO: Convert DMA to Link Mode */
	/* Set current TX and RX buffer from SPI buffers array */
	spi_context_buffers_setup(spi_ctx, tx_bufs, rx_bufs, 1);

	rza2m_spi_dma_advance_segment(dev);
	retval = rza2m_spi_dma_advance_buffers(dev);
	if (retval == 0) {
		return 0;
	}

	/* Error, stop DMA, restore interrupts */
	dma_stop(cfg->dma_dev, cfg->tx_dma_channel);
	dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
	rza2m_spi_write8(dev, ctx->r_spcr, RSPI_SPCR);

	/* Deassert SSL signal */
	spi_context_cs_control(spi_ctx, false);

err_unlock:
	spi_context_release(spi_ctx, retval);
	return retval;
}
#endif /* CONFIG_SPI_ASYNC */

static const struct spi_driver_api rza2m_spi_driver_api = {
	.transceive = rza2m_spi_transceive,
	.release = rza2m_spi_release,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = rza2m_spi_transceive_async,
#endif
};

static int rza2m_spi_init(const struct device *dev)
{
	const struct rza2m_spi_cfg *cfg = dev->config;
	struct rza2m_spi_ctx *ctx = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "Failed to configure SPI pins");
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_mod);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "Failed to configure clk");
		return ret;
	}

	ret = clock_control_get_rate(cfg->clk_dev,
				     (clock_control_subsys_t)&cfg->clk_mod,
				     &ctx->clk_rate);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "Failed to get clk rate");
		return ret;
	}

	ret = spi_context_cs_configure_all(&ctx->spi_ctx);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "Failed to configure CS pins: %d", ret);
		return ret;
	}

	rza2m_spi_write8(dev, 0x0, RSPI_SPCR);

	k_sem_init(&ctx->sem_tx, 0, 1);
	k_sem_init(&ctx->sem_rx, 0, 1);
	cfg->irq_config();

	spi_context_unlock_unconditionally(&ctx->spi_ctx);

	LOG_DEV_INF(dev, "Init done fck:%uHz", ctx->clk_rate);

	return 0;
}

#define RZA2M_SPI_IRQ_CONFIG_FUNC(n)							\
	static void rza2m_irq_config_##n(void)						\
	{										\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, rx, irq),				\
			    DT_INST_IRQ_BY_NAME(n, rx, priority),			\
			    rza2m_spi_irq_rx,						\
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ_BY_NAME(n, rx, flags));	\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, tx, irq),				\
			    DT_INST_IRQ_BY_NAME(n, tx, priority),			\
			    rza2m_spi_irq_tx,						\
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ_BY_NAME(n, tx, flags));	\
		irq_enable(DT_INST_IRQ_BY_NAME(n, rx, irq));				\
		irq_enable(DT_INST_IRQ_BY_NAME(n, tx, irq));				\
	}

#ifdef CONFIG_SPI_ASYNC
#define SPI_RZA2M_DMA_CHANNELS(n)							\
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),			\
	.tx_dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, slot),				\
	.tx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),			\
	.rx_dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, rx, slot),				\
	.rx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),
#else
#define SPI_RZA2M_DMA_CHANNELS(n)
#endif

#define RZA2M_SPI_DEVICE(n)								\
	PINCTRL_DT_INST_DEFINE(n);							\
	RZA2M_SPI_IRQ_CONFIG_FUNC(n)							\
	static const struct rza2m_spi_cfg rza2m_spi_##id##_cfg = {			\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),					\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(n, 0)),		\
		.clk_mod.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),		\
		.clk_mod.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.num_cs = DT_INST_PROP_OR(n, num_cs, 1),				\
		.irq_config = rza2m_irq_config_##n,					\
		SPI_RZA2M_DMA_CHANNELS(n)						\
	};										\
	static struct rza2m_spi_ctx rza2m_spi_##id##_data = {				\
			SPI_CONTEXT_INIT_LOCK(rza2m_spi_##id##_data, spi_ctx),		\
			SPI_CONTEXT_INIT_SYNC(rza2m_spi_##id##_data, spi_ctx),		\
			SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), spi_ctx)	\
	};										\
	DEVICE_DT_INST_DEFINE(n, rza2m_spi_init, NULL, &rza2m_spi_##id##_data,		\
			      &rza2m_spi_##id##_cfg, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	\
			      &rza2m_spi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RZA2M_SPI_DEVICE)
