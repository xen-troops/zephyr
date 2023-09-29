/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/dma/rza2_dma.h>
#include <zephyr/drivers/dma.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>

#define DT_DRV_COMPAT renesas_rza2_dma

#define RZA2_DMA_CH_OFFT        0x40
#define RZA2_DMA_CH_8_15_OFFSET 0x400

BUILD_ASSERT(CONFIG_DMA_RZA2_INIT_PRIORITY > CONFIG_INTC_INIT_PRIORITY,
	     "DMA_RZA2_INIT_PRIORITY must be higher than CONFIG_INTC_INIT_PRIORITY");

#define GET_SECTION_OFF(n) ((n > 7) ? RZA2_DMA_CH_8_15_OFFSET : 0)

#define GET_CNUM_OFF(n) (RZA2_DMA_CH_OFFT * (n & 7))

#define GET_CH_OFF(dev, n)                                                                         \
	(DEVICE_MMIO_NAMED_GET(dev, reg_main) + GET_SECTION_OFF(n) + GET_CNUM_OFF(n))

#define GET_DC_OFF(dev, n) (DEVICE_MMIO_NAMED_GET(dev, reg_main) + GET_SECTION_OFF(n))

#define N0SA(dev, n)   (GET_CH_OFF(dev, n) + 0x0)
#define N0DA(dev, n)   (GET_CH_OFF(dev, n) + 0x4)
#define N0TB(dev, n)   (GET_CH_OFF(dev, n) + 0x8)
#define N1SA(dev, n)   (GET_CH_OFF(dev, n) + 0xc)
#define N1DA(dev, n)   (GET_CH_OFF(dev, n) + 0x10)
#define N1TB(dev, n)   (GET_CH_OFF(dev, n) + 0x14)
#define CRSA(dev, n)   (GET_CH_OFF(dev, n) + 0x18)
#define CRDA(dev, n)   (GET_CH_OFF(dev, n) + 0x1c)
#define CRTB(dev, n)   (GET_CH_OFF(dev, n) + 0x20)
#define CHSTAT(dev, n) (GET_CH_OFF(dev, n) + 0x24)
#define CHCTRL(dev, n) (GET_CH_OFF(dev, n) + 0x28)
#define CHCFG(dev, n)  (GET_CH_OFF(dev, n) + 0x2c)
#define CHITVL(dev, n) (GET_CH_OFF(dev, n) + 0x30)
#define CHEXT(dev, n)  (GET_CH_OFF(dev, n) + 0x34)
#define NXLA(dev, n)   (GET_CH_OFF(dev, n) + 0x38)
#define CRLA(dev, n)   (GET_CH_OFF(dev, n) + 0x3c)

/* Common registers */
#define DCTRL(dev, n)     (GET_DC_OFF(dev, n) + 0x300)
#define DSTAT_EN(dev, n)  (GET_DC_OFF(dev, n) + 0x310)
#define DSTAT_ER(dev, n)  (GET_DC_OFF(dev, n) + 0x314)
#define DSTAT_END(dev, n) (GET_DC_OFF(dev, n) + 0x318)
#define DSTAT_TC(dev, n)  (GET_DC_OFF(dev, n) + 0x31c)
#define DSTAT_SUS(dev, n) (GET_DC_OFF(dev, n) + 0x320)

/*
 * Extended in range of 0..7 from "ext" region.
 * Reg is uint32_t with CHn in 0 -> 9 bits and CHn+1 in 16->25 bits
 */
#define DMARS(dev, n) (DEVICE_MMIO_NAMED_GET(dev, ext) + ((n / 0x2) * 4))

LOG_MODULE_REGISTER(dma_rza2, CONFIG_DMA_LOG_LEVEL);

/*
 * Link mode descriptor definitions
 */
#define HDR_LV  BIT(0) /* Indicates whether descriptor is valid */
#define HDR_LE  BIT(1) /* Indicated whether link ends */
#define HDR_WBD BIT(2) /* Write Back Disable */
#define HDR_DIM BIT(3) /* Interrupt Mask */

#define DMAC_PRV_CHCFG_SET_DMS          (0x80000000U)
#define DMAC_PRV_CHCFG_SET_REN          (0x40000000U)
#define DMAC_PRV_CHCFG_MASK_REN         (0x40000000U)
#define DMAC_PRV_CHCFG_SET_RSW          (0x20000000U)
#define DMAC_PRV_CHCFG_MASK_RSW         (0x20000000U)
#define DMAC_PRV_CHCFG_SET_RSEL         (0x10000000U)
#define DMAC_PRV_CHCFG_MASK_RSEL        (0x10000000U)
#define DMAC_PRV_CHCFG_MASK_SBE         (0x08000000U)
#define DMAC_PRV_CHCFG_SET_DEM          (0x01000000U)
#define DMAC_PRV_CHCFG_MASK_DEM         (0x01000000U)
#define DMAC_PRV_CHCFG_SET_TM           (0x00400000U)
#define DMAC_PRV_CHCFG_MASK_DAD         (0x00200000U)
#define DMAC_PRV_CHCFG_MASK_SAD         (0x00100000U)
#define DMAC_PRV_CHCFG_MASK_DDS         (0x000f0000U)
#define DMAC_PRV_CHCFG_MASK_SDS         (0x0000f000U)
#define DMAC_PRV_CHCFG_SET_AM_LEVEL     (0x00000100U)
#define DMAC_PRV_CHCFG_SET_AM_BUS_CYCLE (0x00000200U)
#define DMAC_PRV_CHCFG_MASK_AM          (0x00000700U)
#define DMAC_PRV_CHCFG_SET_LVL_EDGE     (0x00000000U)
#define DMAC_PRV_CHCFG_SET_LVL_LEVEL    (0x00000040U)
#define DMAC_PRV_CHCFG_MASK_LVL         (0x00000040U)
#define DMAC_PRV_CHCFG_SET_REQD_SRC     (0x00000000U)
#define DMAC_PRV_CHCFG_SET_REQD_DST     (0x00000008U)
#define DMAC_PRV_CHCFG_MASK_REQD        (0x00000008U)
#define DMAC_PRV_CHCFG_SHIFT_SBE        (27U)
#define DMAC_PRV_CHCFG_SHIFT_DAD        (21U)
#define DMAC_PRV_CHCFG_SHIFT_SAD        (20U)
#define DMAC_PRV_CHCFG_SHIFT_DDS        (16U)
#define DMAC_PRV_CHCFG_SHIFT_SDS        (12U)
#define DMAC_PRV_CHCFG_SHIFT_AM         (8U)
#define DMAC_PRV_CHCFG_SHIFT_REQD       (3U)
#define DMAC_PRV_CHCFG_SHIFT_LOEN       (4U)
#define DMAC_PRV_CHCFG_SHIFT_HIEN       (5U)
#define DMAC_PRV_CHCFG_SHIFT_LVL        (6U)

/* REQD value in CHCFG is undefined on configuration table */
#define DMAC_PRV_CHCFG_REQD_UNDEFINED (2)

/* CHEXT */
#define DMAC_PRV_CHEXT_SET_DCA_NORMAL        (0x00003000U)
#define DMAC_PRV_CHEXT_SET_DCA_STRONG        (0x00000000U)
#define DMAC_PRV_CHEXT_SET_DPR_NON_SECURE    (0x00000200U)
#define DMAC_PRV_CHEXT_SET_SCA_NORMAL        (0x00000030U)
#define DMAC_PRV_CHEXT_SET_SCA_STRONG        (0x00000000U)
#define DMAC_PRV_CHEXT_SET_SPR_NON_SECURE    (0x00000002U)

/* Address of area which is the target of setting change */
#define DMAC_PRV_DMA_EXTERNAL_BUS_START         (0x00000000U)
#define DMAC_PRV_DMA_EXTERNAL_BUS_END           (0x1FFFFFFFU)
#define DMAC_PRV_DMA_EXTERNAL_BUS_MIRROR_START  (0x40000000U)
#define DMAC_PRV_DMA_EXTERNAL_BUS_MIRROR_END    (0x5FFFFFFFU)

#define CLRINTMSK    BIT(17)
#define SETINTMSK    BIT(16)
#define CLRSUS       BIT(9)
#define SETSUS       BIT(8)
#define CLRTC        BIT(6)
#define CLREND       BIT(5)
#define CLRRQ        BIT(4)
#define SWRST        BIT(3)
#define STG          BIT(2)
#define CLREN        BIT(1)
#define SETEN        BIT(0)
#define CHCTRL_CLEAR (CLRINTMSK | CLRSUS | CLRTC | CLREND | CLRRQ | SWRST | CLREN)

#define DMAC_PRV_CHSTAT_MASK_DER  (BIT(10))
#define DMAC_PRV_CHSTAT_MASK_SR   (BIT(7))
#define DMAC_PRV_CHSTAT_MASK_END  (BIT(5))
#define DMAC_PRV_CHSTAT_MASK_ER   (BIT(4))
#define DMAC_PRV_CHSTAT_MASK_TACT (BIT(2))
#define DMAC_PRV_CHSTAT_MASK_EN   (BIT(0))

#define GET_RID(x)  ((x) & 0x3)
#define GET_MID(x)  (((x) & 0x3fc) >> 2)
#define GET_TM(x)   (((x) & 0xc00) >> 10)
#define GET_AM(x)   (((x) & 0x7000) >> 12)
#define GET_LVL(x)  (!!((x) & LVL1))
#define GET_HIEN(x) (!!((x) & HIEN1))
#define GET_LOEN(x) (!!((x) & LOEN1))
#define GET_REQD(x) (((x) & 0xC0000) >> 18)

#define IS_SET(value, mask) (!!((value) & (mask)))

struct rza2_dma_link_descriptor {
	uint32_t header;
	uint32_t src_addr;
	uint32_t dest_addr;
	uint32_t trans_byte;
	uint32_t config;
	uint32_t interval;
	uint32_t extension;
	uint32_t next_link_address;
};

struct dma_rza2_config {
	DEVICE_MMIO_NAMED_ROM(reg_main);
	DEVICE_MMIO_NAMED_ROM(ext);

	uint8_t num_channels;
	void (*irq_configure)(void);
	const struct pinctrl_dev_config *pcfg;
	uint32_t addr_alignment;
};

enum channel_mode {
	REGISTER_MODE = 0,
	LINK_MODE
};

struct dma_rza2_channel {
	int sw_trigger;
	bool busy;
	dma_callback_t dma_callback;
	void *user_data;
	int err_callback_en;
	int complete_callback_en;
	enum channel_mode mode;
	struct rza2_dma_link_descriptor *descrs;
	uint32_t total_bytes;
	uint32_t direction;
	int chunk;
};

BUILD_ASSERT(CONFIG_DMA_RZA2_DESCRS_CHUNKS <= 32,
	     "CONFIG_DMA_RZA2_DESCRS_CHUNKS > 32 is not supported\n");

struct dma_rza2_data {
	/* Dma context should be the first in data structure */
	struct dma_context ctx;

	DEVICE_MMIO_NAMED_RAM(reg_main);
	DEVICE_MMIO_NAMED_RAM(ext);

	struct dma_rza2_channel *channels;

	struct rza2_dma_link_descriptor *descr_pool;
	uint32_t descrs_busy;
	struct k_spinlock lock;
};

#define DEV_DATA(dev) ((struct dma_rza2_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct dma_rza2_config *)((dev)->config))

static void rza2_set_n0sa(const struct device *dev, int ch, uint32_t saddr)
{
	sys_write32(saddr, N0SA(dev, ch));
}

static void rza2_set_n0da(const struct device *dev, int ch, uint32_t daddr)
{
	sys_write32(daddr, N0DA(dev, ch));
}

static void rza2_set_n0tb(const struct device *dev, int ch, uint32_t byte_count)
{
	sys_write32(byte_count, N0TB(dev, ch));
}

static uint32_t rza2_get_crtb(const struct device *dev, int ch)
{
	return sys_read32(CRTB(dev, ch));
}

static uint32_t rza2_get_chstat(const struct device *dev, int ch)
{
	return sys_read32(CHSTAT(dev, ch));
}

static void rza2_set_chctrl(const struct device *dev, int ch, uint32_t flag)
{
	sys_write32(flag, CHCTRL(dev, ch));
}

static uint32_t rza2_get_chctrl(const struct device *dev, int ch)
{
	return sys_read32(CHCTRL(dev, ch));
}

static void rza2_set_chcfg(const struct device *dev, int ch, uint32_t value)
{
	sys_write32(value, CHCFG(dev, ch));
}

static uint32_t rza2_get_chcfg(const struct device *dev, int ch)
{
	return sys_read32(CHCFG(dev, ch));
}

static void rza2_set_chitv(const struct device *dev, int ch, uint32_t value)
{
	sys_write32(value, CHITVL(dev, ch));
}

static void rza2_set_chext(const struct device *dev, int ch, uint32_t value)
{
	sys_write32(value, CHEXT(dev, ch));
}

static void rza2_set_nxla(const struct device *dev, int ch, uint32_t value)
{
	sys_write32(value, NXLA(dev, ch));
}

static uint32_t rza2_get_crla(const struct device *dev, int ch)
{
	return sys_read32(CRLA(dev, ch));
}

static void rza2_set_dctrl(const struct device *dev, int ch, uint32_t value)
{
	sys_write32(value, DCTRL(dev, ch));
}

static int rza2_get_ch_state(mem_addr_t addr, int ch)
{
	uint32_t value = sys_read32(addr);
	uint8_t ch_bit = BIT(ch & 7);

	return value & ch_bit;
}

static int rza2_get_ch_err(const struct device *dev, int ch)
{
	return sys_read32(DSTAT_ER(dev, ch));
}

static int rza2_get_ch_sus(const struct device *dev, int ch)
{
	return rza2_get_ch_state(DSTAT_SUS(dev, ch), ch);
}

static void rza2_set_dmars(const struct device *dev, int ch, uint8_t mid, uint8_t rid)
{
	uint32_t dmars32 = sys_read32(DMARS(dev, ch));
	uint16_t value = FIELD_PREP(GENMASK(9, 2), mid) | FIELD_PREP(GENMASK(1, 0), rid);

	if (ch & 1) {
		dmars32 &= 0x0000ffff;
		dmars32 |= value << 16;
	} else {
		dmars32 &= 0xffff0000;
		dmars32 |= value;
	}

	sys_write32(dmars32, DMARS(dev, ch));
}

static int dma_get_data_size(uint32_t data_size)
{
	switch (data_size) {
	case 1:
		return 0;
	case 2:
		return 1;
	case 4:
		return 2;
	case 8:
		return 3;
	case 16:
		return 4;
	case 32:
		return 5;
	case 64:
		return 6;
	case 128:
		return 7;
	default:
		return -EINVAL;
	}
}

static int get_free_and_set_unlocked(uint32_t *descrs_busy)
{
	int chunk;

	chunk = find_lsb_set(~(*descrs_busy));
	if (chunk && chunk <= CONFIG_DMA_RZA2_DESCRS_CHUNKS) {
		*descrs_busy |= BIT(chunk - 1);
		return chunk - 1;
	}

	return -1;
}

static int free_chunk(const struct device *dev, int chunk)
{
	struct dma_rza2_data *data = dev->data;
	k_spinlock_key_t key;

	if (chunk >= CONFIG_DMA_RZA2_DESCRS_CHUNKS || chunk < 0) {
		return -EINVAL;
	}
	key = k_spin_lock(&data->lock);
	data->descrs_busy &= ~BIT(chunk);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int get_descrs(const struct device *dev, struct rza2_dma_link_descriptor **descr)
{
	k_spinlock_key_t key;
	struct dma_rza2_data *data = dev->data;
	int chunk;

	if (!descr) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	chunk = get_free_and_set_unlocked(&data->descrs_busy);
	if (chunk < 0) {
		*descr = NULL;
	} else {
		*descr = &data->descr_pool[chunk * CONFIG_DMA_RZA2_MAX_DESCRS];
	}

	k_spin_unlock(&data->lock, key);

	return chunk;
}

static bool not_aligned(const struct device *dev, uint32_t addr)
{
	const struct dma_rza2_config *cfg = dev->config;

	if (cfg->addr_alignment == 0) {
		return false;
	}

	return !!(addr & (cfg->addr_alignment - 1));
}

static int rza2_construct_link_chain(const struct device *dev, struct dma_config *dma_cfg,
				     struct dma_rza2_channel *chan, uint32_t ch_cfg)
{
	int i;
	struct dma_block_config *block = dma_cfg->head_block;
	uint32_t total_bytes = 0;

	__ASSERT(chan, "channel should be provided");

	if (!block) {
		LOG_ERR("no dma blocks were provided");
		return -EINVAL;
	}

	chan->chunk = get_descrs(dev, &chan->descrs);
	if (chan->chunk < 0) {
		LOG_ERR("%s: unable to allocate descriptor list for dma transfer", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < dma_cfg->block_count; i++) {
		/* Check source_address and dest_address alignment */
		if ((not_aligned(dev, block->dest_address)) ||
		    (not_aligned(dev, block->source_address))) {
			LOG_ERR("%s: buffers are not properly aligned", __func__);
			free_chunk(dev, chan->chunk);
			return -EINVAL;
		}

		chan->descrs[i].src_addr = Z_MEM_PHYS_ADDR(block->source_address);
		chan->descrs[i].dest_addr = Z_MEM_PHYS_ADDR(block->dest_address);
		chan->descrs[i].trans_byte = block->block_size;
		chan->descrs[i].interval = 0;
		chan->descrs[i].config = ch_cfg;
		chan->descrs[i].header = HDR_LV;

		/* DMA controller supports only 1 interval so apply the bigger one which enabled*/
		if (block->source_gather_en) {
			chan->descrs[i].interval = block->source_gather_interval;
		}

		if (block->dest_scatter_en &&
		    chan->descrs[i].interval < block->dest_scatter_interval) {
			chan->descrs[i].interval = block->dest_scatter_interval;
		}

		/* If this is the last block */
		if (i == dma_cfg->block_count - 1) {
			chan->descrs[i].header = HDR_LV | HDR_LE;
			chan->descrs[i].next_link_address = 0;
		} else {
			chan->descrs[i].next_link_address =
				Z_MEM_PHYS_ADDR((uint32_t)&chan->descrs[i + 1]);
		}

		total_bytes += block->block_size;
		block = block->next_block;
	}

	chan->total_bytes = total_bytes;
	return 0;
}

static void dma_rza2_channel_free(const struct device *dev, struct dma_rza2_channel *ch)
{
	ch->busy = false;
	free_chunk(dev, ch->chunk);
}

static inline int check_ch(const struct device *dev, uint32_t ch)
{
	const struct dma_rza2_config *cfg = dev->config;

	if (ch >= cfg->num_channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->num_channels, ch);
		return -EINVAL;
	}

	return 0;
}

static int dma_rza2_config(const struct device *dev, uint32_t channel, struct dma_config *dma_cfg)
{
	int ret;
	struct dma_rza2_data *data = dev->data;
	uint32_t channel_cfg = 0;
	uint32_t channel_ext;
	uint32_t dctrl_cfg = 0;
	uint32_t slot;
	uint32_t interval = 0;
	uint8_t tm;
	uint32_t phys_addr;

	slot = dma_cfg->dma_slot;

	ret = check_ch(dev, channel);
	if (ret < 0) {
		return ret;
	}

	if (data->channels[channel].busy) {
		return -EBUSY;
	}

	if (dma_cfg->source_chaining_en || dma_cfg->dest_chaining_en) {
		LOG_ERR("Channel chaining is not supported");
		return -EINVAL;
	}

	if (dma_cfg->channel_direction > PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, MEMORY_TO_PERIPHERAL or "
			"PERIPHERAL_TO_MEMORY (%" PRIu32 ")",
			dma_cfg->channel_direction);
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid source_addr_adj %" PRIu16, dma_cfg->head_block->source_addr_adj);
		return -ENOTSUP;
	}
	if (dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid dest_addr_adj %" PRIu16, dma_cfg->head_block->dest_addr_adj);
		return -ENOTSUP;
	}

	/* Set sel */
	channel_cfg |= (channel & 0x7u);

	if (dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_INCREMENT) {
		channel_cfg |= DMAC_PRV_CHCFG_MASK_SAD;
	}

	if (dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT) {
		channel_cfg |= DMAC_PRV_CHCFG_MASK_DAD;
	}

	ret = dma_get_data_size(dma_cfg->dest_data_size);
	if (ret < 0) {
		LOG_ERR("invalid dest_data_size %d\n", dma_cfg->dest_data_size);
		return ret;
	}

	channel_cfg |= ((ret << DMAC_PRV_CHCFG_SHIFT_DDS) & DMAC_PRV_CHCFG_MASK_DDS);

	ret = dma_get_data_size(dma_cfg->source_data_size);
	if (ret < 0) {
		LOG_ERR("invalid source_data_size %d\n", dma_cfg->source_data_size);
		return ret;
	}

	channel_cfg |= ((ret << DMAC_PRV_CHCFG_SHIFT_SDS) & DMAC_PRV_CHCFG_MASK_SDS);
	data->channels[channel].direction = dma_cfg->channel_direction;

	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY) {
		channel_cfg |= DMAC_PRV_CHCFG_SET_REQD_DST;

		/* Set sw_trigger so transfer will start on Channel Enable */
		data->channels[channel].sw_trigger = 1;
		channel_cfg |= DMAC_PRV_CHCFG_SET_TM;
		channel_cfg |= DMAC_PRV_CHCFG_SET_LVL_EDGE;
	} else {
		if (GET_REQD(slot) == REQ_BOTH) {
			if (dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
				channel_cfg |= DMAC_PRV_CHCFG_SET_REQD_DST;
			} else {
				channel_cfg |= DMAC_PRV_CHCFG_SET_REQD_SRC;
			}
		} else {
			channel_cfg |= ((GET_REQD(slot) << DMAC_PRV_CHCFG_SHIFT_REQD) &
					DMAC_PRV_CHCFG_MASK_REQD);
		}

		channel_cfg |= ((GET_AM(slot) << DMAC_PRV_CHCFG_SHIFT_AM) & DMAC_PRV_CHCFG_MASK_AM);

		tm = GET_TM(slot);
		if (tm != TM_BOTH && tm != 0) {
			channel_cfg |= DMAC_PRV_CHCFG_SET_TM;
		}


		channel_cfg |= GET_LOEN(slot) << DMAC_PRV_CHCFG_SHIFT_LOEN;
		channel_cfg |= GET_HIEN(slot) << DMAC_PRV_CHCFG_SHIFT_HIEN;
		channel_cfg |= GET_LVL(slot) << DMAC_PRV_CHCFG_SHIFT_LVL;
	}

	if ((channel_cfg & DMAC_PRV_CHCFG_MASK_REQD) == DMAC_PRV_CHCFG_SET_REQD_SRC) {
		channel_cfg |= 1 << DMAC_PRV_CHCFG_SHIFT_SBE; /* Takes effect only when reqd = 0 */
	}
	/* 0 for Fixed Priority, Round Robin otherwise */
	dctrl_cfg = (dma_cfg->channel_priority == 0);

	if (dma_cfg->block_count > 1) {
		/* Link Mode configuration */
		channel_cfg |= DMAC_PRV_CHCFG_SET_DMS;
		data->channels[channel].mode = LINK_MODE;

		/*
		 * IMPORTANT: channel_cfg value should be set for each descriptor in the linked
		 * list. So current channel_configuration has effect only on start stage. After
		 * loading the descriptor descr->config value will be loaded to CHCFG register.
		 */
		ret = rza2_construct_link_chain(dev, dma_cfg, &data->channels[channel],
						channel_cfg);
		if (ret) {
			return ret;
		};

		rza2_set_nxla(dev, channel,
			      Z_MEM_PHYS_ADDR((uint32_t)&data->channels[channel].descrs[0]));
	} else {
		data->channels[channel].mode = REGISTER_MODE;

		/* Check source_address and dest_address alignment */
		if ((not_aligned(dev, dma_cfg->head_block->dest_address)) ||
		    (not_aligned(dev, dma_cfg->head_block->source_address))) {
			LOG_ERR("%s: buffers are not properly aligned", __func__);
			return -EINVAL;
		}

		rza2_set_n0sa(dev, channel, Z_MEM_PHYS_ADDR(dma_cfg->head_block->source_address));
		rza2_set_n0da(dev, channel, Z_MEM_PHYS_ADDR(dma_cfg->head_block->dest_address));
		rza2_set_n0tb(dev, channel, dma_cfg->head_block->block_size);

		channel_cfg &= ~DMAC_PRV_CHCFG_SET_DMS;  /* Set register mode */
		channel_cfg &= ~DMAC_PRV_CHCFG_SET_REN;  /* Do not switch register set */
		channel_cfg &= ~DMAC_PRV_CHCFG_SET_RSEL; /* Select 0 register set */
		channel_cfg &= ~DMAC_PRV_CHCFG_SET_DEM;  /* Unmask DMA end interrupt */
		channel_cfg &= ~DMAC_PRV_CHCFG_SET_RSW;  /* No automatic register change */
		data->channels[channel].total_bytes = dma_cfg->head_block->block_size;
	}

	if (dma_cfg->head_block->source_gather_en) {
		interval = dma_cfg->head_block->source_gather_interval;
	} else if (dma_cfg->head_block->dest_scatter_en) {
		interval = dma_cfg->head_block->dest_scatter_interval;
	}

	channel_ext = (DMAC_PRV_CHEXT_SET_DPR_NON_SECURE | DMAC_PRV_CHEXT_SET_SPR_NON_SECURE);
	phys_addr = Z_MEM_PHYS_ADDR(dma_cfg->head_block->source_address);
	/* set bus parameter for source */
	if ((phys_addr <= DMAC_PRV_DMA_EXTERNAL_BUS_END) ||
	    ((phys_addr >= DMAC_PRV_DMA_EXTERNAL_BUS_MIRROR_START) &&
	    (phys_addr <= DMAC_PRV_DMA_EXTERNAL_BUS_MIRROR_END))) {
		channel_ext |= DMAC_PRV_CHEXT_SET_SCA_NORMAL;
	} else {
		channel_ext |= DMAC_PRV_CHEXT_SET_SCA_STRONG;
	}
	phys_addr = Z_MEM_PHYS_ADDR(dma_cfg->head_block->dest_address);
	/* set bus parameter for destination */
	if ((phys_addr <= DMAC_PRV_DMA_EXTERNAL_BUS_END) ||
	   ((phys_addr >= DMAC_PRV_DMA_EXTERNAL_BUS_MIRROR_START) &&
	    (phys_addr <= DMAC_PRV_DMA_EXTERNAL_BUS_MIRROR_END))) {
		channel_ext |= DMAC_PRV_CHEXT_SET_DCA_NORMAL;
	} else {
		channel_ext |= DMAC_PRV_CHEXT_SET_DCA_STRONG;
	}

	rza2_set_chcfg(dev, channel, channel_cfg);
	rza2_set_dctrl(dev, channel, dctrl_cfg);
	rza2_set_chitv(dev, channel, interval);
	rza2_set_chext(dev, channel, channel_ext);

	rza2_set_dmars(dev, channel, GET_MID(slot), GET_RID(slot));
	data->channels[channel].dma_callback = dma_cfg->dma_callback;
	data->channels[channel].user_data = dma_cfg->user_data;
	data->channels[channel].err_callback_en = dma_cfg->error_callback_en;
	data->channels[channel].complete_callback_en = dma_cfg->complete_callback_en;

	/* Clear status */
	rza2_set_chctrl(dev, channel, SWRST);

	return 0;
}

static int dma_rza2_start(const struct device *dev, uint32_t ch)
{
	struct dma_rza2_data *data = dev->data;
	int ret;
	uint32_t stat;

	ret = check_ch(dev, ch);
	if (ret < 0) {
		return ret;
	}

	if (data->channels[ch].busy) {
		return -EBUSY;
	}

	stat = rza2_get_chstat(dev, ch);
	if (IS_SET(stat, DMAC_PRV_CHSTAT_MASK_EN) || IS_SET(stat, DMAC_PRV_CHSTAT_MASK_TACT)) {
		return -EBUSY;
	}

	/* Clear status */
	rza2_set_chctrl(dev, ch, SWRST);

	data->channels[ch].busy = true;

	/* Enable Channel */
	if (data->channels[ch].sw_trigger) {
		rza2_set_chctrl(dev, ch, STG | SETEN);
	} else {
		rza2_set_chctrl(dev, ch, SETEN);
	}

	return 0;
}

static int dma_rza2_stop(const struct device *dev, uint32_t ch)
{
	struct dma_rza2_data *data = dev->data;
	int ret;
	uint32_t ch_cfg;

	ret = check_ch(dev, ch);
	if (ret < 0) {
		return ret;
	}

	if (!data->channels[ch].busy) {
		return 0;
	}

	rza2_set_chctrl(dev, ch, CLREN);

	ch_cfg = rza2_get_chcfg(dev, ch);
	if (IS_SET(ch_cfg, DMAC_PRV_CHCFG_MASK_SBE)) {
		ch_cfg &= ~DMAC_PRV_CHCFG_MASK_REQD;
		rza2_set_chcfg(dev, ch, ch_cfg);
	}

	rza2_set_chctrl(dev, ch, SWRST);

	dma_rza2_channel_free(dev, &data->channels[ch]);

	return 0;
}

static int dma_rza2_suspend(const struct device *dev, uint32_t ch)
{
	int ret;
	struct dma_rza2_data *data = dev->data;

	ret = check_ch(dev, ch);
	if (ret < 0) {
		return ret;
	}

	if (!data->channels[ch].busy) {
		return -EINVAL;
	}

	rza2_set_chctrl(dev, ch, SETSUS);

	return 0;
}

static int dma_rza2_resume(const struct device *dev, uint32_t ch)
{
	int ret;
	struct dma_rza2_data *data = dev->data;

	ret = check_ch(dev, ch);
	if (ret < 0) {
		return ret;
	}

	if (!data->channels[ch].busy) {
		return -EINVAL;
	}

	if (!rza2_get_ch_sus(dev, ch)) {
		return -EINVAL;
	}

	rza2_set_chctrl(dev, ch, CLRSUS);

	return 0;
}

static uint32_t get_copied_bytes(struct dma_rza2_channel *chan,
				 struct rza2_dma_link_descriptor *descr)
{
	uint32_t copied_bytes;
	bool found = false;
	struct rza2_dma_link_descriptor *cur = chan->descrs;

	while (cur) {
		if (cur == descr) {
			found = true;
			break;
		}
		copied_bytes += cur->trans_byte;
		cur = (struct rza2_dma_link_descriptor *)Z_MEM_VIRT_ADDR(cur->next_link_address);
	}

	if (!found) {
		LOG_ERR("Current link descriptor lays outside of the channel descriptor list");
		copied_bytes = 0;
	}

	return copied_bytes;
}

static int dma_rza2_get_status(const struct device *dev, uint32_t ch, struct dma_status *stat)
{
	int ret;
	struct dma_rza2_data *data = dev->data;
	struct rza2_dma_link_descriptor *link;
	uint32_t channel_cfg;

	ret = check_ch(dev, ch);
	if (ret < 0) {
		return ret;
	}

	channel_cfg = rza2_get_chcfg(dev, ch);

	stat->busy = data->channels[ch].busy;

	stat->dir = data->channels[ch].direction;

	if (IS_SET(channel_cfg, DMAC_PRV_CHCFG_SET_DMS)) {
		/* For link mode calculate already processed blocks */
		link = (struct rza2_dma_link_descriptor *)Z_MEM_VIRT_ADDR(rza2_get_crla(dev, ch));
		if (link) {
			stat->total_copied = get_copied_bytes(&data->channels[ch], link);
		} else {
			stat->total_copied = 0;
		}

		stat->pending_length = data->channels[ch].total_bytes - stat->total_copied;
	} else {
		/* For register mode get data from CRTB register */
		stat->pending_length = rza2_get_crtb(dev, ch);
		stat->total_copied = data->channels[ch].total_bytes - stat->pending_length;
	}

	return 0;
}

/* TODO get_attributes callback should be implemened */
static const struct dma_driver_api dma_rza2_driver_api = {
	.config = dma_rza2_config,
	.start = dma_rza2_start,
	.stop = dma_rza2_stop,
	.get_status = dma_rza2_get_status,
	.suspend = dma_rza2_suspend,
	.resume = dma_rza2_resume
};

#define DMA_LINK_TR_END        1
#define DMA_LINK_TR_INPROGRESS 0

static int dma_rza2_process_link(const struct device *dev, int ch, uint32_t stat)
{
	struct dma_rza2_data *data = dev->data;
	struct rza2_dma_link_descriptor *link;

	if (data->channels[ch].mode != LINK_MODE) {
		return -EINVAL;
	}

	link = (struct rza2_dma_link_descriptor *)Z_MEM_VIRT_ADDR(rza2_get_crla(dev, ch));
	if (!link) {
		return -EIO;
	}

	if (IS_SET(stat, DMAC_PRV_CHSTAT_MASK_TACT) && !IS_SET(stat, DMAC_PRV_CHSTAT_MASK_DER)) {
		if (data->channels[ch].sw_trigger) {
			rza2_set_chctrl(dev, ch, STG);
		}
	}

	if (!link->header & HDR_LV) {
		return -EIO;
	}

	return (link->header & HDR_LE) ? DMA_LINK_TR_END : DMA_LINK_TR_INPROGRESS;
}

static void dma_rza2_isr_common(const struct device *dev, int ch)
{
	struct dma_rza2_data *data = dev->data;
	uint32_t chctrl;
	uint32_t stat;
	int ret;

	if (check_ch(dev, ch) < 0) {
		LOG_ERR("invalid channel in isr handler");
		return;
	}

	if (!data->channels[ch].busy) {
		LOG_ERR("invalid interrupt, DMA Transfer should be started");
		return;
	}

	stat = rza2_get_chstat(dev, ch);

	if (data->channels[ch].mode == REGISTER_MODE) {
		if (IS_SET(stat, DMAC_PRV_CHSTAT_MASK_ER)) {
			goto err;
		}

		if (!IS_SET(stat, DMAC_PRV_CHSTAT_MASK_END) ||
		    IS_SET(stat, DMAC_PRV_CHSTAT_MASK_EN)) {
			rza2_set_chctrl(dev, ch, CHCTRL_CLEAR);
			dma_rza2_channel_free(dev, &data->channels[ch]);
			if (data->channels[ch].complete_callback_en == 0 &&
			    data->channels[ch].dma_callback) {
				data->channels[ch].dma_callback(dev, data->channels[ch].user_data,
								ch, DMA_STATUS_BLOCK);
			}

			return;
		}

		dma_rza2_channel_free(dev, &data->channels[ch]);
		if (data->channels[ch].complete_callback_en == 0 &&
		    data->channels[ch].dma_callback) {
			data->channels[ch].dma_callback(dev, data->channels[ch].user_data, ch,
							DMA_STATUS_COMPLETE);
		}
	} else if (data->channels[ch].mode == LINK_MODE) {
		if (!IS_SET(stat, DMAC_PRV_CHSTAT_MASK_END) ||
		    IS_SET(stat, DMAC_PRV_CHSTAT_MASK_DER)) {
			goto err;
		}

		ret = dma_rza2_process_link(dev, ch, stat);
		if (ret < 0) {
			goto err;
		}

		/* Send callback for block if needed*/
		if (data->channels[ch].complete_callback_en == 1 &&
		    data->channels[ch].dma_callback) {
			data->channels[ch].dma_callback(dev, data->channels[ch].user_data, ch,
							DMA_STATUS_BLOCK);
		}

		if (ret == DMA_LINK_TR_END && IS_SET(stat, DMAC_PRV_CHSTAT_MASK_EN) == 0) {
			chctrl = rza2_get_chctrl(dev, ch);
			rza2_set_chctrl(dev, ch, chctrl | CLREND);
			dma_rza2_channel_free(dev, &data->channels[ch]);
			/* CALL FINAL CALLBACK*/
			if (data->channels[ch].complete_callback_en == 0 &&
			    data->channels[ch].dma_callback) {
				data->channels[ch].dma_callback(dev, data->channels[ch].user_data,
								ch, DMA_STATUS_COMPLETE);
			}
		}
	}
	return;

err:
	rza2_set_chctrl(dev, ch, CHCTRL_CLEAR);
	dma_rza2_channel_free(dev, &data->channels[ch]);
	if (data->channels[ch].err_callback_en == 0 && data->channels[ch].dma_callback) {
		data->channels[ch].dma_callback(dev, data->channels[ch].user_data, ch, -EIO);
	}
}

static void dma_rza2_err_isr(const struct device *dev)
{
	int err_0_7, err_8_15;
	int ch;
	uint16_t channel_mask;
	const struct dma_rza2_config *config = dev->config;
	struct dma_rza2_data *data = dev->data;

	err_0_7 = rza2_get_ch_err(dev, 0);  /* Same value for all 0..7 channels */
	err_8_15 = rza2_get_ch_err(dev, 8); /* Same value for all 8..15 channels */

	channel_mask = err_0_7 | (err_8_15 << 8);

	for (ch = 0; ch < config->num_channels; ch++) {
		if ((channel_mask & BIT(ch)) && (data->channels[ch].err_callback_en == 0) &&
		    data->channels[ch].dma_callback) {
			data->channels[ch].dma_callback(dev, data->channels[ch].user_data, ch,
							DMA_STATUS_BLOCK);
			dma_rza2_channel_free(dev, &data->channels[ch]);
		}
	}
}

static int dma_rza2_init(const struct device *dev)
{
	const struct dma_rza2_config *cfg = dev->config;
	int ret;

#ifdef CONFIG_PINCTRL
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("unable to configure DMA pins");
		return -EINVAL;
	}
#endif

	DEVICE_MMIO_NAMED_MAP(dev, reg_main, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, ext, K_MEM_CACHE_NONE);

	cfg->irq_configure();

	return 0;
}

#define IRQ_ERR_CONFIGURE(inst, name)                                                              \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, name, irq),                                          \
		    DT_INST_IRQ_BY_NAME(inst, name, priority), dma_rza2_err_isr,                   \
		    DEVICE_DT_INST_GET(inst), DT_INST_IRQ_BY_NAME(inst, name, flags));             \
	irq_enable(DT_INST_IRQ_BY_NAME(inst, name, irq));

#define IRQ_DECLARE_ISR(n, inst)                                                                   \
	static void dma_rza2_##n##_##inst##_isr(const struct device *dev)                          \
	{                                                                                          \
		dma_rza2_isr_common(dev, n);                                                       \
	}

#define IRQ_CONFIGURE(n, inst)                                                                     \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    dma_rza2_##n##_##inst##_isr, DEVICE_DT_INST_GET(inst),                         \
		    DT_INST_IRQ_BY_IDX(inst, n, flags));                                           \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

#define DECLARE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_DECLARE_ISR, (), inst)

#define DMA_RZA2_POOL_SIZE CONFIG_DMA_RZA2_MAX_DESCRS * CONFIG_DMA_RZA2_DESCRS_CHUNKS

#ifdef CONFIG_PINCTRL
#define RZ_PINCTRL_DT_INST_DEFINE(n)                                                               \
	COND_CODE_1(DT_INST_NUM_PINCTRL_STATES(n), (PINCTRL_DT_INST_DEFINE(n);), (EMPTY))
#define RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n)                                                       \
	COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))
#else
#define RZ_PINCTRL_DT_INST_DEFINE(n)
#define RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n) NULL
#endif

#define RZA2_DMA_INIT(inst)                                                                        \
	DECLARE_ALL_IRQS(inst, DT_INST_PROP(inst, dma_channels))                                   \
                                                                                                   \
	static void dma_rza2_##inst##_irq_configure(void)                                          \
	{                                                                                          \
		CONFIGURE_ALL_IRQS(inst, DT_INST_PROP(inst, dma_channels));                        \
                                                                                                   \
		COND_CODE_1(DT_INST_IRQ_HAS_NAME(inst, err0), (IRQ_ERR_CONFIGURE(inst, err0)), ()) \
		COND_CODE_1(DT_INST_IRQ_HAS_NAME(inst, err1), (IRQ_ERR_CONFIGURE(inst, err1)), ()) \
	}                                                                                          \
                                                                                                   \
	RZ_PINCTRL_DT_INST_DEFINE(inst);                                                           \
                                                                                                   \
	static const struct dma_rza2_config dma_rza2_##inst##_config = {                           \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(reg_main, DT_DRV_INST(inst)),                   \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(ext, DT_DRV_INST(inst)),                        \
		.num_channels = DT_INST_PROP(inst, dma_channels),                                  \
		.irq_configure = dma_rza2_##inst##_irq_configure,                                  \
		.pcfg = RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.addr_alignment = DMA_BUF_ADDR_ALIGNMENT(DT_DRV_INST(inst)),                       \
	};                                                                                         \
                                                                                                   \
	static __aligned(DMA_BUF_ADDR_ALIGNMENT(DT_DRV_INST(inst)))                                \
		struct rza2_dma_link_descriptor descr_##inst##_pool[DMA_RZA2_POOL_SIZE]            \
		__attribute__((__section__(".nocache.dma"))) = {0};                                \
                                                                                                   \
	static struct dma_rza2_channel                                                             \
		dma_rza2_##inst##_channels[DT_INST_PROP(inst, dma_channels)];                      \
	ATOMIC_DEFINE(dma_rza2_atomic##inst, DT_INST_PROP(inst, dma_channels));                    \
                                                                                                   \
	static struct dma_rza2_data dma_rza2_##inst##_data = {                                     \
		.ctx =                                                                             \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.atomic = dma_rza2_atomic##inst,                                   \
				.dma_channels = DT_INST_PROP(inst, dma_channels),                  \
			},                                                                         \
		.channels = dma_rza2_##inst##_channels,                                            \
		.descr_pool = descr_##inst##_pool,                                                 \
		.descrs_busy = 0,                                                                  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_rza2_init, NULL, &dma_rza2_##inst##_data,                  \
			      &dma_rza2_##inst##_config, PRE_KERNEL_1,                             \
			      CONFIG_DMA_RZA2_INIT_PRIORITY, &dma_rza2_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RZA2_DMA_INIT)
