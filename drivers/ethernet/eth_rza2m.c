/*
 * Driver for Renesas RZ/A2M ethernet controller.
 *
 * Copyright (c) 2023 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_rza2m, CONFIG_ETHERNET_LOG_LEVEL /*LOG_LEVEL_DBG*/);

#define DT_DRV_COMPAT renesas_ether_r7s9210

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <ethernet/eth_stats.h>

#include "eth.h"
#include "eth_rza2m_priv.h"

#define LOG_DEV_ERR(dev, format, ...)	LOG_ERR("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...)	LOG_WRN("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...)	LOG_INF("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...)	LOG_DBG("%s:"#format, (dev)->name, ##__VA_ARGS__)

/*
 * Grace period to wait for TX descriptor/fragment availability.
 * Worst case estimate is 1514*8 bits at 10 mbps for an existing packet
 * to be sent and freed, therefore 1ms is far more than enough.
 * Beyond that we'll drop the packet.
 */
#define TX_AVAIL_WAIT K_MSEC(1)

/* stack size for RX refill thread */
#define RX_REFILL_STACK_SIZE	1024

#define RZA2M_ETH_OFFSET_INVALID	((uint16_t)~0)

#define RZA2M_ETH_OFFSET_DEFAULTS			\
	[0 ... RZA2M_ETH_MAX_REGS_OFFSET - 1] = RZA2M_ETH_OFFSET_INVALID

static const uint16_t rza2m_eth_regs_offset[RZA2M_ETH_MAX_REGS_OFFSET] = {
	RZA2M_ETH_OFFSET_DEFAULTS,

	[ECMR]		= 0x0100,
	[RFLR]		= 0x0108,
	[ECSR]		= 0x0110,
	[ECSIPR]	= 0x0118,
	[PIR]		= 0x0120,
	[PSR]		= 0x0128,
	[RDMLR]		= 0x0140,
	[IPGR]		= 0x0150,
	[APR]		= 0x0154,
	[MPR]		= 0x0158,
	[RFCF]		= 0x0160,
	[TPAUSER]	= 0x0164,
	[TPAUSECR]	= 0x0168,
	[BCFRR]		= 0x016c,
	[MAHR]		= 0x01c0,
	[MALR]		= 0x01c8,
	[TROCR]		= 0x01d0,
	[CDCR]		= 0x01d4,
	[LCCR]		= 0x01d8,
	[CNDCR]		= 0x01dc,
	[CEFCR]		= 0x01e4,
	[FRECR]		= 0x01e8,
	[TSFRCR]	= 0x01ec,
	[TLFRCR]	= 0x01f0,
	[RFCR]		= 0x01f4,
	[MAFCR]		= 0x01f8,

	[EDMR]		= 0x0000,
	[EDTRR]		= 0x0008,
	[EDRRR]		= 0x0010,
	[TDLAR]		= 0x0018,
	[RDLAR]		= 0x0020,
	[EESR]		= 0x0028,
	[EESIPR]	= 0x0030,
	[TRSCER]	= 0x0038,
	[RMFCR]		= 0x0040,
	[TFTR]		= 0x0048,
	[FDR]		= 0x0050,
	[RMCR]		= 0x0058,
	[TFUCR]		= 0x0064,
	[RFOCR]		= 0x0068,
	[IOSR]		= 0x006c,
	[FCFTR]		= 0x0070,
	[RPADIR]	= 0x0078,
	[TRIMD]		= 0x007c,
	[RBWAR]		= 0x00c8,
	[RDFAR]		= 0x00cc,
	[TBRAR]		= 0x00d4,
	[TDFAR]		= 0x00d8,
};

struct rza2m_eth_reg_init {
	uint8_t		reg_id;
	uint32_t	reg_val;
};

static const struct rza2m_eth_reg_init rza2m_eth_reg_init[] = {
	/* no status irq */
	{ ECSIPR,	0 }, /* no irq */
	{ EESIPR,	0 },

	/* EDMAC */
	{ EDMR,		RZA2M_ETH_CFG_EDMR_VAL },
	{ TRSCER,	RZA2M_ETH_CFG_TRSCER_VAL },
	{ TFTR,		RZA2M_ETH_CFG_TFTR_VAL },
	{ FDR,			RZA2M_ETH_CFG_FDR_VAL },
	{ RMCR,		RZA2M_ETH_CFG_RMCR_VAL },
	{ RPADIR,	RZA2M_ETH_CFG_RPADIR_VAL },
	{ FCFTR,	RZA2M_ETH_CFG_TC_FCFTR_VAL },
	{ BCFRR,	CONFIG_ETH_RZA2M_NB_BC_RX_STORM_FILTER },

	/* ETHERC */
	{ RFLR,		RZA2M_ETH_CFG_RFLRL_VAL }, /* max rx frame size */
	{ IPGR,		RZA2M_ETH_CFG_IPGRL_VAL }, /* intergap */
	{ APR,		RZA2M_ETH_CFG_TC_APR_VAL }, /* Automatic PAUSE Frame */
	{ MPR,		RZA2M_ETH_CFG_TC_MPR_VAL }, /* Manual PAUSE Frame */
	{ TPAUSER,	RZA2M_ETH_CFG_TC_TPAUSER_VAL }, /* PAUSE Frame Retransmit Count */

	{ ECSR,		ECSR_ALL}, /* clear status ETHERC */
	{ EESR,		EESR_INIT}, /* clear status EDMAC */
};

struct rza2m_eth_cfg {
	DEVICE_MMIO_ROM;
	const uint16_t			*reg_offset;
	char				*phy_mode;
	const struct device		*clk_dev;
	struct renesas_cpg_clk		clk_mod;
	const struct device		*clk_ctrl;
	struct renesas_cpg_clk		clk_ctrl_mod;
	const struct pinctrl_dev_config *pcfg;
	const struct device		*phy_dev;
	uint8_t				mac_addr[6];
	uint32_t			irq_n;
	void				(*irq_config)(void);
	uint32_t			speed;
	bool				f_no_linksta:1;
	bool				f_linksta_active_low:1;
	bool				f_fixed_link:1;
	bool				f_full_duplex:1;
};

#define DEV_DATA(_dev) ((struct rza2m_eth_ctx *)((_dev)->data))

/* descriptor index iterators */
#define INC_WRAP(idx, size) ({ idx = ((idx) + 1) % (size); })
#define DEC_WRAP(idx, size) ({ idx = ((idx) + (size) - 1) % (size); })

struct rza2m_eth_ctx {
	DEVICE_MMIO_RAM;
	const struct device	*dev;
	struct net_if		*iface;
	uint8_t			mac_addr[6];

	uint32_t		tx_desc_head;
	uint32_t		tx_desc_tail;
	struct k_sem		sem_free_tx_descs;
	struct tx_desc_s	*tx_desc;
	struct net_buf		*tx_frags[CONFIG_ETH_RZA2M_NB_TX_DESCS];

	uint32_t		rx_desc_head;
	uint32_t		rx_desc_tail;
	struct k_sem		sem_free_rx_descs;
	struct net_pkt		*rx_pkt;
	uint32_t		rx_bytes;
	struct rx_desc_s	*rx_desc;
	struct net_buf		*rx_frags[CONFIG_ETH_RZA2M_NB_RX_DESCS];

	K_KERNEL_STACK_MEMBER(rx_refill_thread_stack, RX_REFILL_STACK_SIZE);
	struct k_thread		rx_refill_thread;

	struct k_work		linksta_work;
	struct k_mutex		mutex_link;
	uint32_t		ecmr_val;
	uint32_t		eesipr_val;
	uint32_t		ecsipr_val;
	uint8_t			f_running:1;	/* Running state flag */
	uint8_t			f_promisc:1;	/* Promisc mode state flag */
	uint8_t			f_link:1;	/* Link is up state flag */

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth	stats;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
#define	RZA2M_STATS_ERR_TX(ctx)		(ctx)->stats.errors.tx++
#define	RZA2M_STATS_ERR_TX_FIFO(ctx)	(ctx)->stats.error_details.tx_fifo_errors++;
#define	RZA2M_STATS_ERR_TX_ABORT(ctx)	(ctx)->stats.error_details.tx_aborted_errors++;
#define	RZA2M_STATS_ERR_TX_DROP(ctx)	(ctx)->stats.tx_dropped++;

#define	RZA2M_STATS_ERR_RX(ctx)		(ctx)->stats.errors.rx++;
#define	RZA2M_STATS_ERR_RX_DMA(ctx)	(ctx)->stats.error_details.rx_dma_failed++;
#define	RZA2M_STATS_ERR_RX_BUF_ALLOC(ctx)	(ctx)->stats.error_details.rx_buf_alloc_failed++;

void rza2m_stats_err_rx_rfs(struct rza2m_eth_ctx *ctx, uint32_t flags)
{
	if (flags & RD_RFS4_RRF) {
		ctx->stats.error_details.rx_align_errors++;
	}
	if (flags & RD_RFS3_RTLF) {
		ctx->stats.error_details.rx_long_length_errors++;
	}
	if (flags & RD_RFS2_RTSF) {
		ctx->stats.error_details.rx_short_length_errors++;
	}
	if (flags & RD_RFS1_PRE) {
		ctx->stats.error_details.rx_frame_errors++;
	}
	if (flags & RD_RFS0_CERF) {
		ctx->stats.error_details.rx_crc_errors++;
	}
}
#else
#define	RZA2M_STATS_ERR_TX(ctx)
#define	RZA2M_STATS_ERR_TX_FIFO(ctx)
#define	RZA2M_STATS_ERR_TX_ABORT(ctx)
#define	RZA2M_STATS_ERR_TX_DROP(ctx)

#define	RZA2M_STATS_ERR_RX(ctx)
#define	RZA2M_STATS_ERR_RX_DMA(ctx)
#define	RZA2M_STATS_ERR_RX_BUF_ALLOC(ctx)
#define rza2m_stats_err_rx_rfs(ctx, flags)
#endif

static void rza2m_eth_write(const struct device *dev, uint32_t data, int enum_index)
{
	const struct rza2m_eth_cfg *cfg = dev->config;
	uint16_t offset = cfg->reg_offset[enum_index];

	__ASSERT(offset == RZA2M_ETH_OFFSET_INVALID,
		 "%s called with enum_index %d", __func__, enum_index);

	sys_write32(data, DEVICE_MMIO_GET(dev) + offset);
}

static uint32_t rza2m_eth_read(const struct device *dev, int enum_index)
{
	const struct rza2m_eth_cfg *cfg = dev->config;
	uint16_t offset = cfg->reg_offset[enum_index];

	__ASSERT(offset == RZA2M_ETH_OFFSET_INVALID,
		 "%s called with enum_index %d", __func__, enum_index);

	return sys_read32(DEVICE_MMIO_GET(dev) + offset);
}

static void rza2m_eth_modify(const struct device *dev, int enum_index, uint32_t clear, uint32_t set)
{
	rza2m_eth_write(dev, (rza2m_eth_read(dev, enum_index) & ~clear) | set,
			enum_index);
}

static void rza2m_eth_regs_init(const struct device *dev)
{
	const struct rza2m_eth_reg_init *reg_init = rza2m_eth_reg_init;
	int i;

	for (i = 0; i < ARRAY_SIZE(rza2m_eth_reg_init); i++) {
		rza2m_eth_write(dev, reg_init->reg_val, reg_init->reg_id);
		reg_init++;
	}
}

static int rza2m_eth_soft_reset(const struct device *dev)
{
	rza2m_eth_modify(dev, EDMR, 0, EDMR_SRST_ETHER);
	k_msleep(3);
	rza2m_eth_modify(dev, EDMR, EDMR_SRST_ETHER, 0);

	return 0;
}

/* Program the hardware MAC address from dev->dev_addr. */
static void rza2m_eth_update_mac_addr(const struct device *dev)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	rza2m_eth_write(dev,
			(ctx->mac_addr[0] << 24) | (ctx->mac_addr[1] << 16) |
			(ctx->mac_addr[2] << 8) | (ctx->mac_addr[3]),
			MAHR);
	rza2m_eth_write(dev,
			(ctx->mac_addr[4] << 8) | (ctx->mac_addr[5]),
			MALR);
}

static void rza2m_eth_regs_dump(const struct device *dev);

/* for debug logs */
__maybe_unused static int net_pkt_get_nbfrags(struct net_pkt *pkt)
{
	struct net_buf *frag;
	int nbfrags = 0;

	for (frag = pkt->buffer; frag; frag = frag->frags) {
		nbfrags++;
	}
	return nbfrags;
}

static void rza2m_eth_tx_release(const struct device *dev, bool sent_only)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	struct tx_desc_s *desc;
	struct net_buf *frag;
	uint32_t td_ds_flags;
	uint32_t d_idx;
	bool sent;

	LOG_DEV_DBG(dev, "TXF:desc sem/head/tail=%d/%d/%d",
		    k_sem_count_get(&ctx->sem_free_tx_descs),
		    ctx->tx_desc_head, ctx->tx_desc_tail);

	for (d_idx = ctx->tx_desc_tail;
	     d_idx != ctx->tx_desc_head;
	     INC_WRAP(d_idx, CONFIG_ETH_RZA2M_NB_TX_DESCS), k_sem_give(&ctx->sem_free_tx_descs)) {

		desc = &ctx->tx_desc[d_idx];
		td_ds_flags = desc->td0;
		LOG_DEV_DBG(dev, "TXF:DESC[%d] TD0: 0x%08x sem:%d", d_idx, td_ds_flags,
			    k_sem_count_get(&ctx->sem_free_tx_descs));

		sent = !(td_ds_flags & TD_TACT);
		/* stop here if hardware still owns it and not clean up all */
		if (sent_only && !sent) {
			break;
		}

		/* release corresponding fragments */
		frag = ctx->tx_frags[d_idx];
		LOG_DEV_DBG(dev, "TXF:unref frag %p", (void *)frag->data);
		net_pkt_frag_unref(frag);

		/* last packet descriptor: */
		if (td_ds_flags & TD_FEND) {
			/* log any errors */
			if (td_ds_flags & TD_TFE) {
				LOG_DEV_INF(dev, "TXF:error TD0:0x%08x)", td_ds_flags);
				RZA2M_STATS_ERR_TX(ctx);
				RZA2M_STATS_ERR_TX_ABORT(ctx);
			}
		}

		if (d_idx == CONFIG_ETH_RZA2M_NB_TX_DESCS - 1) {
			desc->td0 = TD_TDLE;
		} else {
			desc->td0 = 0;
		}
		desc->td1 = 0;
		desc->td2 = 0;
	}

	ctx->tx_desc_tail = d_idx;
}

static int rza2m_eth_send(const struct device *dev, struct net_pkt *pkt)
{
	unsigned int pkt_len = net_pkt_get_len(pkt);
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	struct net_buf *frag, *pinned;
	struct tx_desc_s *desc;
	uint32_t td_ds_flags;
	uint32_t n_frags;
	uint32_t d_idx;

	LOG_DEV_DBG(dev, "TX:pkt len/frags=%d/%d", pkt_len, net_pkt_get_nbfrags(pkt));
	LOG_DEV_DBG(dev, "TX:desc sem/head/tail=%d/%d/%d",
		    k_sem_count_get(&ctx->sem_free_tx_descs),
		    ctx->tx_desc_head, ctx->tx_desc_tail);

	n_frags = net_pkt_get_nbfrags(pkt);
	if (n_frags > k_sem_count_get(&ctx->sem_free_tx_descs) + 1) {
		/* not enough descs for packet - try to recover once */
		k_sleep(K_MSEC(1));
		if (n_frags > k_sem_count_get(&ctx->sem_free_tx_descs)) {
			LOG_DEV_DBG(dev, "TX: low desc for pkt");
			RZA2M_STATS_ERR_TX_DROP(ctx);
			return -ENOMEM;
		}
	}

	/* initial flag values */
	td_ds_flags = TD_F1ST | TD_TACT;

	/* map packet fragments */
	d_idx = ctx->tx_desc_head;
	frag = pkt->buffer;
	do {
		/* reserve a free descriptor for this fragment */
		if (k_sem_take(&ctx->sem_free_tx_descs, TX_AVAIL_WAIT) != 0) {
			LOG_DEV_INF(dev, "TX:no more free tx descriptors");
			goto abort;
		}

		/* pin this fragment */
		pinned = net_buf_clone(frag, TX_AVAIL_WAIT);
		if (!pinned) {
			LOG_DEV_INF(dev, "TX:net_buf_clone() returned NULL");
			k_sem_give(&ctx->sem_free_tx_descs);
			goto abort;
		}

		sys_cache_data_flush_range(pinned->data, pinned->len);
		ctx->tx_frags[d_idx] = pinned;
		LOG_DEV_DBG(dev, "TX:desc[%d]: frag %p pinned %p len %d", d_idx,
			    (void *)frag->data, (void *)pinned->data, pinned->len);

		/* if no more fragments after this one: */
		if (!frag->frags) {
			/* set those flags on the last descriptor */
			td_ds_flags |= TD_FEND;
		}

		if (d_idx >= (CONFIG_ETH_RZA2M_NB_TX_DESCS - 1)) {
			td_ds_flags |= TD_TDLE;
		}

		/* fill the descriptor */
		desc = &ctx->tx_desc[d_idx];
		desc->td2 = z_mem_phys_addr(pinned->data);
		desc->td1 = (pinned->len << 16);
		desc->td0 = td_ds_flags;

		/* make sure all the above made it to memory */
		barrier_dmem_fence_full();
		LOG_DEV_DBG(dev, "TX:desc[%d]: desc %p flag %08X len:%08X", d_idx,
			    (void *)desc, td_ds_flags, desc->td1);

		/* clear the FD flag on subsequent descriptors */
		td_ds_flags = TD_TACT;

		INC_WRAP(d_idx, CONFIG_ETH_RZA2M_NB_TX_DESCS);
		frag = frag->frags;
	} while (frag);

	/* update the descriptor index head */
	ctx->tx_desc_head = d_idx;

	/* Restart the transmitter if disabled */
	if (!(rza2m_eth_read(dev, EDTRR) & EDTRR_TR))
		rza2m_eth_write(dev, EDTRR_TR, EDTRR);

	return 0;

abort:
	while (d_idx != ctx->tx_desc_head) {
		/* release already pinned fragments */
		DEC_WRAP(d_idx, CONFIG_ETH_RZA2M_NB_TX_DESCS);
		frag = ctx->tx_frags[d_idx];
		LOG_DEV_INF(dev, "TX:err[%d]: frag %p len %d", d_idx,
			    (void *)frag->data, frag->len);
		net_pkt_frag_unref(frag);
		k_sem_give(&ctx->sem_free_tx_descs);
	}
	RZA2M_STATS_ERR_TX_DROP(ctx);

	return -ENOMEM;
}

static void rza2m_eth_receive(const struct device *dev)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	struct rx_desc_s *desc;
	struct net_buf *frag;
	uint32_t rx_flags;
	uint32_t pkt_len;
	uint32_t d_idx;

	LOG_DEV_DBG(dev, "RX:desc sem/tail/head=%d/%d/%d",
		    k_sem_count_get(&ctx->sem_free_rx_descs),
		    ctx->rx_desc_tail, ctx->rx_desc_head);

	for (d_idx = ctx->rx_desc_tail;
	     d_idx != ctx->rx_desc_head;
	     INC_WRAP(d_idx, CONFIG_ETH_RZA2M_NB_RX_DESCS), k_sem_give(&ctx->sem_free_rx_descs)) {

		desc = &ctx->rx_desc[d_idx];
		rx_flags = desc->rd0;
		LOG_DEV_DBG(dev, "RX:desc[%d] RD0:0x%08x RD1:%08x", d_idx, rx_flags, desc->rd1);

		/* stop here if hardware still owns it */
		if (rx_flags & RD_RACT) {
			break;
		}

		/* a packet's first descriptor: */
		if (rx_flags & RD_F1ST) {
			ctx->rx_bytes = 0;
			if (ctx->rx_pkt) {
				LOG_DEV_ERR(dev, "RX:desc[%d] first desc but pkt exists", d_idx);
				net_pkt_unref(ctx->rx_pkt);
				RZA2M_STATS_ERR_RX(ctx);
				RZA2M_STATS_ERR_RX_DMA(ctx);
			}
			ctx->rx_pkt = net_pkt_rx_alloc_on_iface(ctx->iface, K_NO_WAIT);
			if (!ctx->rx_pkt) {
				LOG_DEV_ERR(dev, "RX:net_pkt_rx_alloc_on_iface() failed");
				RZA2M_STATS_ERR_RX(ctx);
				RZA2M_STATS_ERR_RX_BUF_ALLOC(ctx);
			}
		}

		if (!ctx->rx_pkt) {
			LOG_DEV_ERR(dev, "RX:desc[%d] no rx_pkt skipping", d_idx);
			RZA2M_STATS_ERR_RX(ctx);
			RZA2M_STATS_ERR_RX_BUF_ALLOC(ctx);
			continue;
		}

		/* retrieve current fragment */
		frag = ctx->rx_frags[d_idx];
		ctx->rx_frags[d_idx] = NULL;
		frag->len = (desc->rd1 & RD_RBL) >> 16;
		sys_cache_data_invd_range(frag->data, frag->size);

		/* last descriptor: */
		if (rx_flags & RD_FEND) {
			pkt_len = desc->rd1 & RD_RFL;
			frag->len = pkt_len - ctx->rx_bytes;
			net_pkt_frag_add(ctx->rx_pkt, frag);

			/* submit packet if no errors */
			if ((rx_flags & RD_RFE) && !(rx_flags & RD_RFS7_RMAF)) {
				LOG_DEV_ERR(dev, "RX:desc[%d] error RD0:0x%08x", d_idx, rx_flags);
				net_pkt_unref(ctx->rx_pkt);
				RZA2M_STATS_ERR_RX(ctx);
				rza2m_stats_err_rx_rfs(ctx, rx_flags);
			} else {
				LOG_DEV_DBG(dev, "RX:desc[%d] pkt len/frags=%zd:%d/%d",
					    d_idx, net_pkt_get_len(ctx->rx_pkt), pkt_len,
					    net_pkt_get_nbfrags(ctx->rx_pkt));
				net_recv_data(ctx->iface, ctx->rx_pkt);
			}
			ctx->rx_pkt = NULL;
		} else {
			ctx->rx_bytes += frag->len;
			net_pkt_frag_add(ctx->rx_pkt, frag);
		}
	}
	ctx->rx_desc_tail = d_idx;
}

static int rza2m_eth_rx_refill_desc(const struct device *dev, uint32_t d_idx)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	struct rx_desc_s *desc;
	struct net_buf *frag;

	desc = &ctx->rx_desc[d_idx];
	frag = ctx->rx_frags[d_idx];

	/* get a new fragment if the previous one was consumed */
	if (!frag) {
		frag = net_pkt_get_reserve_rx_data(RZA2M_ETH_RX_FRAG_SIZE, K_FOREVER);
		if (!frag) {
			LOG_DEV_ERR(dev, "RXF:net_pkt_get_reserve_rx_data() returned NULL");
			RZA2M_STATS_ERR_RX(ctx);
			RZA2M_STATS_ERR_RX_BUF_ALLOC(ctx)
			return -ENOMEM;
		}
		LOG_DEV_DBG(dev, "RXF:desc[%d] frag at %p", d_idx, frag->data);

		if (frag->size != RZA2M_ETH_RX_FRAG_SIZE) {
			LOG_DEV_ERR(dev, "RXF: wrong frag size");
			RZA2M_STATS_ERR_RX(ctx);
			RZA2M_STATS_ERR_RX_BUF_ALLOC(ctx);
			return -ENOMEM;
		}
		ctx->rx_frags[d_idx] = frag;
	} else {
		LOG_DEV_DBG(dev, "RXF:desc[%d] reusing frag at %p", d_idx, frag->data);
	}

	/* all is good: initialize the descriptor */
	desc->rd2 = z_mem_phys_addr(frag->data);
	desc->rd1 = (RZA2M_ETH_RX_FRAG_SIZE << 16);
	if (d_idx == CONFIG_ETH_RZA2M_NB_RX_DESCS - 1) {
		desc->rd0 = (RD_RDLE | RD_RACT);
	} else {
		desc->rd0 = RD_RACT;
	}

	return 0;
}

static void rza2m_eth_rx_refill_thread(void *arg1, void *unused1, void *unused2)
{
	const struct device *dev = arg1;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	struct rx_desc_s *desc;
	uint32_t d_idx;
	int ret = 0;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	d_idx = ctx->rx_desc_head;
	for (;;) {
		LOG_DEV_DBG(dev, "RXF:desc sem/head/tail=%d/%d/%d",
			    k_sem_count_get(&ctx->sem_free_rx_descs),
			    ctx->rx_desc_head, ctx->rx_desc_tail);

		/* wait for an empty descriptor */
		ret = k_sem_take(&ctx->sem_free_rx_descs, K_FOREVER);
		if (ret) {
			LOG_DEV_DBG(dev, "RXF:can't get free RX sem to refill");
			break;
		}

		desc = &ctx->rx_desc[d_idx];

		if (desc->rd0 & RD_RACT) {
			LOG_DEV_DBG(dev,
				    "RXF:desc[%d] RD0:%x: still hw owned! sem/head/tail=%d/%d/%d",
				    d_idx, desc->rd0, k_sem_count_get(&ctx->sem_free_rx_descs),
				    ctx->rx_desc_head, ctx->rx_desc_tail);
			RZA2M_STATS_ERR_RX(ctx);
			RZA2M_STATS_ERR_RX_DMA(ctx);
			break;
		}

		ret = rza2m_eth_rx_refill_desc(dev, d_idx);
		if (ret) {
			break;
		}

		/* commit the above to memory */
		barrier_dmem_fence_full();

		/* Restart the receiver if disabled */
		if (!(rza2m_eth_read(dev, EDRRR) & EDRRR_RR))
			rza2m_eth_write(dev, EDRRR_RR, EDRRR);

		/* advance to the next descriptor */
		ctx->rx_desc_head = INC_WRAP(d_idx, CONFIG_ETH_RZA2M_NB_RX_DESCS);
	}

	for (d_idx = 0; d_idx < CONFIG_ETH_RZA2M_NB_RX_DESCS; d_idx++) {
		ctx->rx_desc[d_idx].rd0 = 0;
		if (ctx->rx_frags[d_idx]) {
			net_pkt_frag_unref(ctx->rx_frags[d_idx]);
			ctx->rx_frags[d_idx] = NULL;
		}
	}

	if (ret == -EAGAIN) {
		LOG_DEV_INF(dev, "RXF: rx_refill_thread stopped");
	} else {
		LOG_DEV_ERR(dev, "RXF: critical rx_refill_thread stopped");
	}
}

static int rza2m_eth_desc_init(const struct device *dev)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	uint32_t i;

	memset(ctx->tx_desc, 0, CONFIG_ETH_RZA2M_NB_TX_DESCS * sizeof(struct tx_desc_s));
	/* Mark the end of the descriptors */
	ctx->tx_desc[CONFIG_ETH_RZA2M_NB_TX_DESCS - 1].td0 = TD_TDLE;

	ctx->tx_desc_head = 0;
	ctx->tx_desc_tail = 0;

	/* Point the controller to the tx descriptor list. */
	rza2m_eth_write(dev, z_mem_phys_addr(ctx->tx_desc), TDLAR);

	for (i = 0; i < CONFIG_ETH_RZA2M_NB_RX_DESCS - 1; i++) {
		rza2m_eth_rx_refill_desc(dev, i);
	}

	ctx->rx_desc[CONFIG_ETH_RZA2M_NB_RX_DESCS - 1].rd0 = RD_RDLE;

	ctx->rx_desc_head = CONFIG_ETH_RZA2M_NB_RX_DESCS - 1;
	ctx->rx_desc_tail = 0;

	/* set up RX buffer refill thread */
	k_thread_create(&ctx->rx_refill_thread, ctx->rx_refill_thread_stack,
			K_KERNEL_STACK_SIZEOF(ctx->rx_refill_thread_stack),
			rza2m_eth_rx_refill_thread, (void *)dev, NULL, NULL,
			0, K_PRIO_PREEMPT(0), K_NO_WAIT);
	k_thread_name_set(&ctx->rx_refill_thread, "rza2m_rx_refill");

	/* Point the controller to the rx descriptor list. */
	rza2m_eth_write(dev, z_mem_phys_addr(ctx->rx_desc), RDLAR);

	return 0;
}

void rza2m_eth_isr_tx(const struct device *dev, uint32_t eesr_val)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	(void)ctx;

	if (eesr_val & EESR_TDE) {
		LOG_DEV_INF(dev, "IRQ:EESR:%08X Transmit Descriptor Empty", eesr_val);
		RZA2M_STATS_ERR_TX(ctx);
		RZA2M_STATS_ERR_TX_FIFO(ctx);
	}

	if (eesr_val & EESR_TFE) {
		LOG_DEV_INF(dev, "IRQ:EESR:%08X Transmit FIFO Underflow", eesr_val);
		RZA2M_STATS_ERR_TX(ctx);
		RZA2M_STATS_ERR_TX_FIFO(ctx);
	}

	 /* EESR_TABT | EESR_CND | EESR_DLC | EESR_CD | EESR_TRO are reflected in tx desc */
	rza2m_eth_tx_release(dev, true);
}

void rza2m_eth_isr_rx(const struct device *dev, uint32_t eesr_val)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	static uint8_t eesr_rde = 8;
	static uint8_t eesr_rfcof = 8;

	(void)ctx;

	if (eesr_val & EESR_RDE) {
		/* RX is not working as there is no RX descs
		 * once RX descs added the RX will be restarted.
		 * Means RX DMA descs underflow. just limit notification and mask
		 */
		if (eesr_rde) {
			LOG_DEV_ERR(dev, "IRQ:EESR:%08X Receive Descriptor Empty", eesr_val);
			eesr_rde--;
		} else {
			rza2m_eth_modify(dev, EESIPR, EESR_RDE, 0);
		}
		RZA2M_STATS_ERR_RX(ctx);
	}

	if (eesr_val & EESR_RFCOF) {
		/* RX is not working and MAC dropping data due to FIFO full
		 * once RX descs added the RX will be restarted.
		 * Means RX DMA underflow. just limit notification and mask
		 */
		if (eesr_rfcof) {
			LOG_DEV_ERR(dev, "IRQ:EESR:%08X Receive Frame Counter Overflow", eesr_val);
			eesr_rfcof--;
		} else {
			rza2m_eth_modify(dev, EESIPR, EESR_RFCOF, 0);
		}
		RZA2M_STATS_ERR_RX(ctx);
	}

	 /* EESR_RABT | EESR_RFE | EESR_RRF | EESR_RTLF | EESR_RTSF | EESR_PRE | EESR_CERF
	  * are reflected in rx desc
	  */
	rza2m_eth_receive(dev);
}

void rza2m_eth_isr(const struct device *dev)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	uint32_t eesr_val;
	uint32_t ecsr_val;

	eesr_val = rza2m_eth_read(dev, EESR) & ctx->eesipr_val;
	ecsr_val = rza2m_eth_read(dev, ECSR) & ctx->ecsipr_val;

	LOG_DEV_DBG(dev, "IRQ EESR:%08X ECSR:%08X", eesr_val, ecsr_val);

	/* clean up irq before processing */
	rza2m_eth_write(dev, ecsr_val, ECSR);
	rza2m_eth_write(dev, eesr_val, EESR);

	/* LINKSTA event */
	if (ecsr_val & ECSR_LCHNG) {
		/* mask it */
		ctx->ecsipr_val &= ~ECSR_LCHNG;
		rza2m_eth_write(dev, ctx->ecsipr_val, ECSIPR);
		k_work_submit(&ctx->linksta_work);
	}

	if (eesr_val & EESR_TX) {
		rza2m_eth_isr_tx(dev, eesr_val);
	}

	if (eesr_val & EESR_RX) {
		rza2m_eth_isr_rx(dev, eesr_val);
	}
}

static int rza2m_eth_set_cfg(const struct device *dev,
			     enum ethernet_config_type type,
			     const struct ethernet_config *config)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	int ret = 0;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(ctx->mac_addr, config->mac_address.addr, sizeof(ctx->mac_addr));
		rza2m_eth_update_mac_addr(dev);
		net_if_set_link_addr(ctx->iface, ctx->mac_addr,
				     sizeof(ctx->mac_addr), NET_LINK_ETHERNET);
		break;

#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		if (config->promisc_mode == ctx->f_promisc) {
			ret = -EALREADY;
			break;
		}

		k_mutex_lock(&ctx->mutex_link, K_FOREVER);
		ctx->f_promisc = config->promisc_mode;
		if (!ctx->f_link) {
			/* it will be set from link_cb */
			k_mutex_unlock(&ctx->mutex_link);
			break;
		}

		ctx->ecmr_val &= ~ECMR_PRM;
		if (ctx->f_promisc) {
			ctx->ecmr_val |= ECMR_PRM;
		}
		if (!ctx->f_running) {
			/* it will be set at next start */
			k_mutex_unlock(&ctx->mutex_link);
			break;
		}
		rza2m_eth_write(dev, ctx->ecmr_val, ECMR);

		k_mutex_unlock(&ctx->mutex_link);
		break;
#endif

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *rza2m_eth_stats(const struct device *dev)
{
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	return &ctx->stats;
}
#endif

static enum ethernet_hw_caps rza2m_eth_get_caps(const struct device *dev)
{
	enum ethernet_hw_caps caps = 0;

	caps |= ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T;
	caps |= ETHERNET_PROMISC_MODE;

	return caps;
}

static int rza2m_eth_start(const struct device *dev)
{
	const struct rza2m_eth_cfg *cfg = dev->config;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	if (ctx->f_running) {
		LOG_DEV_INF(dev, "Device already running!");
		return 0;
	}

	rza2m_eth_soft_reset(dev);

	rza2m_eth_desc_init(dev);

	rza2m_eth_regs_init(dev);

	rza2m_eth_update_mac_addr(dev);

	irq_enable(cfg->irq_n);

	/* enable irq */
	rza2m_eth_write(dev, ctx->eesipr_val, EESIPR);
	rza2m_eth_write(dev, ctx->ecsipr_val, ECSIPR);

	/* handle link state */
	k_mutex_lock(&ctx->mutex_link, K_FOREVER);
	rza2m_eth_write(dev, ctx->ecmr_val, ECMR);
	if (ctx->f_link) {
		net_eth_carrier_on(ctx->iface);
		LOG_DEV_INF(dev, "PHY net_eth_carrier_on");
	} else {
		net_eth_carrier_off(ctx->iface);
		LOG_DEV_INF(dev, "PHY net_eth_carrier_off");
	}
	ctx->f_running = 1;
	k_mutex_unlock(&ctx->mutex_link);

	/* enable edmac rx */
	rza2m_eth_write(dev, EDRRR_RR, EDRRR);

	if (!cfg->f_no_linksta) {
		k_work_submit(&ctx->linksta_work);
	}

	LOG_DEV_INF(dev, "Starting Device...");
	return 0;
}

static int rza2m_eth_stop(const struct device *dev)
{
	const struct rza2m_eth_cfg *cfg = dev->config;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	if (!ctx->f_running) {
		LOG_DEV_INF(dev, "Device is not running!");
		return 0;
	}

	ctx->f_running = 0;
	/* Stop ETHERC RX/TX */
	rza2m_eth_write(dev, 0, ECMR);

	/* stoop edmac rx */
	rza2m_eth_write(dev, 0, EDRRR);

	/* disable irq */
	rza2m_eth_write(dev, 0, ECSIPR);
	rza2m_eth_write(dev, 0, EESIPR);

	irq_disable(cfg->irq_n);
	k_sleep(TX_AVAIL_WAIT);

	rza2m_eth_tx_release(dev, false);

	k_sem_reset(&ctx->sem_free_rx_descs);
	k_thread_join(&ctx->rx_refill_thread, K_FOREVER);

	if (!cfg->f_no_linksta) {
		struct k_work_sync work_sync;

		k_work_cancel_sync(&ctx->linksta_work, &work_sync);
	}

	LOG_DEV_INF(dev, "Stopping Device...");
	return 0;
}

static void phy_link_state_changed(const struct device *phy_dev,
				   struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	ARG_UNUSED(phy_dev);

	k_mutex_lock(&ctx->mutex_link, K_FOREVER);

	ctx->ecmr_val = 0;
	ctx->f_link = 0;
	if (state->is_up) {
		if (PHY_LINK_IS_FULL_DUPLEX(state->speed)) {
			ctx->ecmr_val |= ECMR_DM;
		}

		if (PHY_LINK_IS_SPEED_100M(state->speed)) {
			ctx->ecmr_val |= ECMR_RTM;
		}

		ctx->ecmr_val |= (ECMR_TE | ECMR_RE);
		if (ctx->f_promisc) {
			ctx->ecmr_val |= ECMR_PRM;
		}
		ctx->f_link = 1;
	}

	if (ctx->f_running) {
		rza2m_eth_write(dev, ctx->ecmr_val, ECMR);

		if (ctx->f_link) {
			net_eth_carrier_on(ctx->iface);
			LOG_DEV_INF(dev, "PHY net_eth_carrier_on");
		} else {
			net_eth_carrier_off(ctx->iface);
			LOG_DEV_INF(dev, "PHY net_eth_carrier_off");
		}
	}

	k_mutex_unlock(&ctx->mutex_link);
}

static void rza2m_eth_linksta_work(struct k_work *item)
{
	const struct rza2m_eth_cfg *cfg;
	struct phy_link_state state;
	struct rza2m_eth_ctx *ctx;
	const struct device *dev;
	bool link;

	ctx = CONTAINER_OF(item, struct rza2m_eth_ctx, linksta_work);
	dev = ctx->dev;
	cfg = dev->config;

	k_mutex_lock(&ctx->mutex_link, K_FOREVER);

	link = !!(rza2m_eth_read(dev, PSR) & PSR_LMON);
	if (cfg->f_linksta_active_low)
		link = !link;

	if (ctx->f_link == link) {
		goto exit_unlock;
	} else if (cfg->f_fixed_link) {
		ctx->f_link = link;
		goto update_link;
	}

	if (cfg->phy_dev) {
		/* process it through phy */
		phy_get_link_state(cfg->phy_dev, &state);
		k_mutex_unlock(&ctx->mutex_link);

		phy_link_state_changed(cfg->phy_dev, &state, (void *)dev);
		goto exit_unmask;
	}

update_link:
	ctx->ecmr_val &= ~(ECMR_TE | ECMR_RE);
	if (ctx->f_link) {
		ctx->ecmr_val |= (ECMR_TE | ECMR_RE);
		ctx->ecmr_val |= ctx->f_promisc ? ECMR_PRM : 0;
	}

	if (ctx->f_running) {
		rza2m_eth_write(dev, ctx->ecmr_val, ECMR);

		if (ctx->f_link) {
			net_eth_carrier_on(ctx->iface);
			LOG_DEV_INF(dev, "LINKSTA net_eth_carrier_on");
		} else {
			net_eth_carrier_off(ctx->iface);
			LOG_DEV_INF(dev, "LINKSTA net_eth_carrier_off");
		}
	}

exit_unlock:
	k_mutex_unlock(&ctx->mutex_link);

exit_unmask:
	/* unmask it */
	ctx->ecsipr_val |= ECSR_LCHNG;
	rza2m_eth_write(dev, ctx->ecsipr_val, ECSIPR);
}

static void rza2m_eth_fixedl_init(const struct device *dev)
{
	const struct rza2m_eth_cfg *cfg = dev->config;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	if (cfg->f_fixed_link) {
		ctx->ecmr_val |= cfg->f_full_duplex ? ECMR_DM : 0;
		ctx->ecmr_val |= (cfg->speed == 100) ? ECMR_RTM : 0;

		if (cfg->f_no_linksta) {
			ctx->f_link = 1;
			ctx->ecmr_val |= (ECMR_TE | ECMR_RE);
		}
	}

	if (!cfg->f_no_linksta) {
		ctx->eesipr_val |= EESR_ECI;
	}
}

static void rza2m_eth_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct rza2m_eth_cfg *cfg = dev->config;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);

	ctx->iface = iface;

	k_sem_init(&ctx->sem_free_tx_descs, CONFIG_ETH_RZA2M_NB_TX_DESCS - 1,
		   CONFIG_ETH_RZA2M_NB_TX_DESCS - 1);
	k_sem_init(&ctx->sem_free_rx_descs, 0,
		   CONFIG_ETH_RZA2M_NB_RX_DESCS - 1);

	ethernet_init(iface);

	/* skip if fixed-link or LINKSTA pin enabled */
	if (cfg->f_fixed_link || !cfg->f_no_linksta) {
		goto skip_phy;
	}

	if (cfg->phy_dev && device_is_ready(cfg->phy_dev)) {
		phy_link_callback_set(cfg->phy_dev, phy_link_state_changed,
				      (void *)dev);
	} else {
		LOG_DEV_ERR(dev, "PHY device not ready");
	}

skip_phy:
	net_if_set_link_addr(iface, ctx->mac_addr, sizeof(ctx->mac_addr),
			     NET_LINK_ETHERNET);
	net_if_carrier_off(ctx->iface);

	ctx->eesipr_val = (EESR_TX | EESR_RX);
	ctx->ecsipr_val = 0;

	rza2m_eth_fixedl_init(dev);

	LOG_DEV_INF(dev, "iface init done");
}

static const struct ethernet_api api_funcs = {
	.iface_api.init		= rza2m_eth_iface_init,
	.get_capabilities	= rza2m_eth_get_caps,
	.set_config		= rza2m_eth_set_cfg,
	.start			= rza2m_eth_start,
	.stop			= rza2m_eth_stop,
	.send			= rza2m_eth_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats		= rza2m_eth_stats,
#endif
};

int eth_init(const struct device *dev)
{
	const uint8_t z_mac_addr[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	const struct rza2m_eth_cfg *cfg = dev->config;
	struct rza2m_eth_ctx *ctx = DEV_DATA(dev);
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	ctx->dev = dev;

	/* fixed-link is not supported with PHY */
	if (cfg->f_fixed_link && cfg->phy_dev) {
		LOG_DEV_ERR(dev, "invalif_cfg: fixed-link used with phy");
		return -EINVAL;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}

	/* must enable ctrl shared clock first */
	ret = clock_control_on(cfg->clk_ctrl, (clock_control_subsys_t)&cfg->clk_ctrl_mod);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_mod);
	if (ret < 0) {
		return ret;
	}

	if (!memcmp(cfg->mac_addr, z_mac_addr, sizeof(cfg->mac_addr))) {
		gen_random_mac((uint8_t *)cfg->mac_addr,
			       RENESAS_OUI_B0, RENESAS_OUI_B1, RENESAS_OUI_B2);
		LOG_DEV_INF(dev, "Use Random MAC!!");
	}
	memcpy(ctx->mac_addr, cfg->mac_addr, sizeof(cfg->mac_addr));

	memset(ctx->rx_desc, 0, CONFIG_ETH_RZA2M_NB_RX_DESCS * sizeof(struct rx_desc_s));

	cfg->irq_config();

	k_work_init(&ctx->linksta_work, rza2m_eth_linksta_work);

	LOG_DEV_INF(dev, "hw_base:%lX virt:%p\n",
		    DEVICE_MMIO_ROM_PTR(dev)->phys_addr, (void *)DEVICE_MMIO_GET(dev));
	LOG_DEV_INF(dev, "MAC %02x:%02x:%02x:%02x:%02x:%02x",
		    ctx->mac_addr[0], ctx->mac_addr[1],
		    ctx->mac_addr[2], ctx->mac_addr[3],
		    ctx->mac_addr[4], ctx->mac_addr[5]);

	return 0;
}

#define RZA2M_ETH_FIXED_LINK_NODE(n)							\
	DT_INST_CHILD(n, fixed_link)

#define RZA2M_ETH_IS_FIXED_LINK(n)							\
	DT_NODE_EXISTS(RZA2M_ETH_FIXED_LINK_NODE(n))

#define RZA2M_ETH_FIXED_LINK_SPEED(n)							\
	DT_PROP_OR(RZA2M_ETH_FIXED_LINK_NODE(n), speed, (0))

#define RZA2M_ETH_FIXED_LINK_FULL_DUPLEX(n)						\
	DT_PROP_OR(RZA2M_ETH_FIXED_LINK_NODE(n), full_duplex, false)

#define RZA2M_ETH_IRQ_CONFIG_FUNC(n)							\
	static void irq_config_func_##n(void)						\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(n),						\
			    DT_INST_IRQ(n, priority),					\
			    rza2m_eth_isr,						\
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ(n, flags));		\
	}

#define ETH_RZA2M_DEVICE_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);							\
	RZA2M_ETH_IRQ_CONFIG_FUNC(n)							\
	static struct rza2m_eth_cfg eth_cfg_##n = {					\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),					\
		.reg_offset = rza2m_eth_regs_offset,					\
		.phy_mode = DT_INST_PROP(n, phy_mode),					\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(n, 0)),		\
		.clk_mod.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),		\
		.clk_mod.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),		\
		.clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(n, 1)),		\
		.clk_ctrl_mod.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),	\
		.clk_ctrl_mod.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.phy_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(n, phy_handle)),	\
		.mac_addr = DT_INST_PROP_OR(n, local_mac_address, {0U}),		\
		.irq_n = DT_INST_IRQN(n),						\
		.irq_config = irq_config_func_##n,					\
		.f_linksta_active_low = DT_INST_PROP_OR(n, renesas_ether_link_active_low, false), \
		.f_no_linksta = DT_INST_PROP_OR(n, renesas_no_ether_link, true),	\
		.f_fixed_link = RZA2M_ETH_IS_FIXED_LINK(n),				\
		.f_full_duplex = RZA2M_ETH_FIXED_LINK_FULL_DUPLEX(n),			\
		.speed = RZA2M_ETH_FIXED_LINK_SPEED(n),					\
	};										\
	static struct tx_desc_s tx_desc_q_##n[CONFIG_ETH_RZA2M_NB_TX_DESCS] __nocache;	\
	static struct rx_desc_s rx_desc_q_##n[CONFIG_ETH_RZA2M_NB_RX_DESCS] __nocache;	\
	static struct rza2m_eth_ctx eth_data_##n = {				\
		.tx_desc = tx_desc_q_##n,						\
		.rx_desc = rx_desc_q_##n,						\
	};										\
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_init, NULL, &eth_data_##n,			\
			      &eth_cfg_##n, CONFIG_ETH_INIT_PRIORITY,			\
			      &api_funcs, NET_ETH_MTU);					\

DT_INST_FOREACH_STATUS_OKAY(ETH_RZA2M_DEVICE_INIT)

__maybe_unused static void rza2m_eth_regs_dump(const struct device *dev)
{
	const struct rza2m_eth_cfg *cfg = dev->config;
	int i;

	for (i = BCFRR; i < TROCR; i++) {
		uint16_t offset = cfg->reg_offset[i];

		printk("%i %04X %08X\n", i, offset, rza2m_eth_read(dev, i));
		k_usleep(100);
	}
}
