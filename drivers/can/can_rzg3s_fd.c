/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rzg3s_canfd

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/stats/stats.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(can_rzg3s_fd, CONFIG_CAN_LOG_LEVEL);

#include "can_rzg3s_fd.h"

#define LOG_DEV_ERR(dev, format, ...) LOG_ERR("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...) LOG_WRN("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...) LOG_INF("%s:" #format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...) LOG_DBG("%s:" #format, (dev)->name, ##__VA_ARGS__)

BUILD_ASSERT((2 * CAN_RZG3S_RX_MAX_PKTS * (12 + CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE)) <= 9728,
	     "not enough CAN RAM for requested configuration");

enum {
	CAN_RZG3S_CHN0,
	CAN_RZG3S_CHN1,
};

STATS_SECT_START(can_rzg3s_ch)
STATS_SECT_ENTRY32(arbitration_lost)
STATS_SECT_ENTRY32(bus_lock)
STATS_SECT_ENTRY32(error_active_state)
STATS_SECT_ENTRY32(error_warning_state)
STATS_SECT_ENTRY32(error_passive_state)
STATS_SECT_ENTRY32(bus_off_state)
STATS_SECT_ENTRY32(bus_off_recovery)
STATS_SECT_ENTRY32(bus_off_recovery_manual)
STATS_SECT_ENTRY32(tdc_violation)
STATS_SECT_END;

STATS_NAME_START(can_rzg3s_ch)
STATS_NAME(can_rzg3s_ch, arbitration_lost)
STATS_NAME(can_rzg3s_ch, bus_lock)
STATS_NAME(can_rzg3s_ch, error_active_state)
STATS_NAME(can_rzg3s_ch, error_warning_state)
STATS_NAME(can_rzg3s_ch, error_passive_state)
STATS_NAME(can_rzg3s_ch, bus_off_state)
STATS_NAME(can_rzg3s_ch, bus_off_recovery)
STATS_NAME(can_rzg3s_ch, bus_off_recovery_manual)
STATS_NAME(can_rzg3s_ch, tdc_violation)
STATS_NAME_END(can_rzg3s_ch);

STATS_SECT_START(can_rzg3s_global)
STATS_SECT_ENTRY32(ecc_error_ch0)
STATS_SECT_ENTRY32(ecc_error_ch1)
STATS_SECT_ENTRY32(canfd_msg_payload_overflow)
STATS_SECT_ENTRY32(rx_msg_lost_rfifo0)
STATS_SECT_ENTRY32(rx_msg_lost_rfifo1)
STATS_SECT_END;

STATS_NAME_START(can_rzg3s_global)
STATS_NAME(can_rzg3s_global, ecc_error_ch0)
STATS_NAME(can_rzg3s_global, ecc_error_ch1)
STATS_NAME(can_rzg3s_global, canfd_msg_payload_overflow)
STATS_NAME(can_rzg3s_global, rx_msg_lost_rfifo0)
STATS_NAME(can_rzg3s_global, rx_msg_lost_rfifo1)
STATS_NAME_END(can_rzg3s_global);

struct can_rzg3s_chn_config {
	const struct device *ctrl_dev;
	void (*irq_config)(void);
#ifdef CONFIG_CAN_STATS
	const char *stat_gr_name;
#endif /* CONFIG_CAN_STATS */
	uint32_t irq_ch_err;
	uint32_t irq_ch_trx;
	uint8_t id;

	uint32_t bitrate;
	uint32_t sample_point;
	uint32_t bitrate_data;
	uint32_t sample_point_data;

	const struct device *phy;
	uint32_t max_bitrate;
	uint8_t tx_delay_comp_offset;
	bool tx_delay_comp_offset_only;
};

struct can_rzg3s_chn_rx_afl_entry {
	can_rx_callback_t rx_cb;
	void *cb_arg;
	struct can_filter filter;
	bool fdf;
};

struct can_rzg3s_chn_tx_entry {
	can_tx_callback_t tx_cb;
	void *cb_arg;
};

struct can_rzg3s_chn_data {
	struct can_timing timing;
	struct can_timing timing_data;

	struct k_sem tx_sem;
	struct k_spinlock tx_tmb_lock;
	uint32_t tx_tmb_free;
	struct can_rzg3s_chn_tx_entry tx_entries[CAN_RZG3S_TX_MAX_PKTS];

	ATOMIC_DEFINE(rx_allocs, CAN_RZGS_FD_MAX_FILTER);
	struct can_rzg3s_chn_rx_afl_entry rx_afl[CAN_RZGS_FD_MAX_FILTER];

	struct k_mutex ch_mutex;
	enum can_state state;
	can_mode_t can_mode;
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	bool f_started;
#ifdef CONFIG_CAN_STATS
	STATS_SECT_DECL(can_rzg3s_ch) stats;
#endif /* CONFIG_CAN_STATS */
};

struct can_rzg3s_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev_pclk;
	struct renesas_cpg_clk clk_mod_pclk;
	const struct device *clk_dev_ram;
	struct renesas_cpg_clk clk_mod_ram;
	const struct device *clk_dev_ext;
	uint32_t clk_ext_rate;
	struct reset_dt_spec rstc_n;
	struct reset_dt_spec rstp_n;
	void (*irq_config)(void);
};

struct can_rzg3s_data {
	DEVICE_MMIO_RAM;
	/* channels using rx fifo map */
	const struct device *rx_fifo_ch_devs[CAN_RZG3S_RX_FIFO_NUM];

	uint32_t clk_pclk_rate;
	uint32_t clk_fcan_rate;
#ifdef CONFIG_CAN_STATS
	STATS_SECT_DECL(can_rzg3s_global) stats;
#endif /* CONFIG_CAN_STATS */
};

/* RX fifo configuration */
static const uint32_t can_rzg3s_rx_fifo_cfg[CAN_RZG3S_RX_FIFO_NUM] = {
	[0] = (R_CFD_RFCC_RFIM | R_CFD_RFCC_RFDC(CAN_RZGS_RX_RFDC) |
	       R_CFD_RFCC_RFPLS(CAN_RZGS_RX_RFPLS) | R_CFD_RFCC_RFIE),
	[1] = (R_CFD_RFCC_RFIM | R_CFD_RFCC_RFDC(CAN_RZGS_RX_RFDC) |
	       R_CFD_RFCC_RFPLS(CAN_RZGS_RX_RFPLS) | R_CFD_RFCC_RFIE),
	[2] = 0,
	[3] = 0,
	[4] = 0,
	[5] = 0,
	[6] = 0,
	[7] = 0,
};

static int can_rzg3s_g_get_core_clock(const struct device *ctrl_dev);
static int can_rzg3s_g_rfifo_register(const struct device *dev_ctrl, uint8_t rf_idx,
				      const struct device *ch_dev);

static void can_rzg3s_write32(const struct device *dev, uint32_t data, uint32_t reg_ofs)
{
	sys_write32(data, DEVICE_MMIO_GET(dev) + reg_ofs);
}

static uint32_t can_rzg3s_read32(const struct device *dev, uint32_t reg_ofs)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg_ofs);
}

static void can_rzg3s_modify32(const struct device *dev, uint32_t reg_ofs, uint32_t clear,
			       uint32_t set)
{
	can_rzg3s_write32(dev, (can_rzg3s_read32(dev, reg_ofs) & ~clear) | set, reg_ofs);
}

static void can_rzg3s_write8(const struct device *dev, uint8_t data, uint32_t reg_ofs)
{
	sys_write8(data, DEVICE_MMIO_GET(dev) + reg_ofs);
}

static uint8_t can_rzg3s_read8(const struct device *dev, uint32_t reg_ofs)
{
	return sys_read8(DEVICE_MMIO_GET(dev) + reg_ofs);
}

static int can_rzg3s_wait_bits1(const struct device *dev, uint32_t reg_ofs, uint32_t bits)
{
	uint32_t reg_val;
	int i = 100;

	do {
		k_busy_wait(10);
		reg_val = can_rzg3s_read32(dev, reg_ofs);
	} while ((reg_val & bits) != bits && i--);

	if (!i) {
		return -EIO;
	}

	return 0;
}

static int can_rzg3s_wait_bits0(const struct device *dev, uint32_t reg_ofs, uint32_t bits)
{
	uint32_t reg_val;
	int i = 100;

	do {
		k_busy_wait(10);
		reg_val = can_rzg3s_read32(dev, reg_ofs);
	} while ((reg_val & bits) && i--);

	if (!i) {
		return -EIO;
	}

	return 0;
}

static void can_rzg3s_afl_lock(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;

	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDG_AFL_ECTR, R_CFDG_AFL_ECTR_AFLDAE, 0);
}

static void can_rzg3s_afl_unlock(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;

	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDG_AFL_ECTR, 0, R_CFDG_AFL_ECTR_AFLDAE);
}

static void can_rzg3s_afl_set_page(const struct device *dev, uint32_t afl_num)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	uint32_t afl_pn;

	afl_pn = afl_num / R_CFDG_AFL_ENTRIES_PER_PAGE;
	LOG_DEV_DBG(dev, "AFL: set page:%u", afl_pn);

	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDG_AFL_ECTR, R_CFDG_AFL_ECTR_AFLPN, afl_pn);
}

static void can_rzg3s_afl_add(const struct device *dev, uint32_t afl_num,
			      const struct can_filter *filter)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	uint32_t afl_pn_ofs;
	uint32_t reg_val;
	uint32_t afl_idx;

	afl_idx = afl_num;
	if (cfg->id == CAN_RZG3S_CHN1) {
		afl_idx += CAN_RZGS_FD_MAX_FILTER;
	}
	afl_pn_ofs = afl_idx % R_CFDG_AFL_ENTRIES_PER_PAGE;

	LOG_DEV_DBG(dev, "AFLADD: num:%d id:%08x mask:%08x flags:%02x idx:%u pn_ofs:%u",
		    afl_num, filter->id, filter->mask, filter->flags, afl_idx, afl_pn_ofs);

	can_rzg3s_afl_unlock(dev);
	can_rzg3s_afl_set_page(dev, afl_idx);

	if (filter->flags & CAN_FILTER_IDE) {
		reg_val = filter->id & CAN_EXT_ID_MASK;
		reg_val |= R_CFDG_AFL_ID_GAFLIDE;
	} else {
		reg_val = filter->id & CAN_STD_ID_MASK;
	}
	if (filter->flags & CAN_FILTER_RTR) {
		reg_val |= R_CFDG_AFL_ID_GAFLRTR;
	}
	can_rzg3s_write32(cfg->ctrl_dev, reg_val, R_CFDG_AFL_ID(afl_pn_ofs));

	if (filter->flags & CAN_FILTER_IDE) {
		reg_val = filter->mask & CAN_EXT_ID_MASK;
	} else {
		reg_val = filter->mask & CAN_STD_ID_MASK;
	}
	reg_val |= R_CFDG_AFL_M_GAFLIDEM;
	if ((filter->flags & (CAN_FILTER_RTR | CAN_FILTER_DATA)) ==
	      (CAN_FILTER_RTR | CAN_FILTER_DATA)) {
		reg_val &= ~R_CFDG_AFL_M_GAFLRTRM;
	} else {
		reg_val |= R_CFDG_AFL_M_GAFLRTRM;
	}
	can_rzg3s_write32(cfg->ctrl_dev, reg_val, R_CFDG_AFL_M(afl_pn_ofs));

	/* save afl_num as GAFLPTR */
	reg_val = (afl_num << R_CFDG_AFL_LP0_GAFLPTR_OFS) & R_CFDG_AFL_LP0_GAFLPTR_MASK;
	can_rzg3s_write32(cfg->ctrl_dev, reg_val, R_CFDG_AFL_LP0(afl_pn_ofs));
	/* set RX fifo for channel */
	can_rzg3s_write32(cfg->ctrl_dev, BIT(cfg->id), R_CFDG_AFL_LP1(afl_pn_ofs));

	can_rzg3s_afl_lock(dev);
}

static void can_rzg3s_afl_del(const struct device *dev, uint32_t afl_num)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	uint32_t afl_pn_ofs;
	uint32_t afl_idx;

	afl_idx = afl_num;
	if (cfg->id == CAN_RZG3S_CHN1) {
		afl_idx += CAN_RZGS_FD_MAX_FILTER;
	}
	afl_pn_ofs = afl_idx % R_CFDG_AFL_ENTRIES_PER_PAGE;

	LOG_DEV_DBG(dev, "AFLDEL: num:%d idx:%u pn_ofs:%u", afl_num, afl_idx, afl_pn_ofs);

	can_rzg3s_afl_unlock(dev);
	can_rzg3s_afl_set_page(dev, afl_idx);

	can_rzg3s_write32(cfg->ctrl_dev, 0, R_CFDG_AFL_ID(afl_pn_ofs));
	can_rzg3s_write32(cfg->ctrl_dev, 0, R_CFDG_AFL_M(afl_pn_ofs));
	can_rzg3s_write32(cfg->ctrl_dev, 0, R_CFDG_AFL_LP0(afl_pn_ofs));
	can_rzg3s_write32(cfg->ctrl_dev, 0, R_CFDG_AFL_LP1(afl_pn_ofs));

	can_rzg3s_afl_lock(dev);
}

static void can_rzg3s_rfifo_get_data(const struct device *dev_ctrl, struct can_frame *frame,
				     uint32_t rf_idx)
{
	uint32_t i, lwords;

	lwords = DIV_ROUND_UP(can_dlc_to_bytes(frame->dlc), sizeof(uint32_t));

	for (i = 0; i < lwords; i++) {
		frame->data_32[i] = can_rzg3s_read32(dev_ctrl, R_CFD_RF_RFDF(rf_idx, i));
	}
}

static void can_rzg3s_rfifo_get_frame(const struct device *dev_ctrl, struct can_frame *frame,
				      uint32_t rf_idx, uint32_t *afl_num)
{
	uint32_t reg_val;

	reg_val = can_rzg3s_read32(dev_ctrl, R_CFD_RF_RFID(rf_idx));

	if (reg_val & R_CFD_RF_RFID_RFIDE) {
		frame->flags = CAN_FRAME_IDE;
		frame->id = reg_val & CAN_EXT_ID_MASK;
	} else {
		frame->id = reg_val & CAN_STD_ID_MASK;
	}
	if (reg_val & R_CFD_RF_RFID_RFRTR) {
		frame->flags |= CAN_FRAME_RTR;
	}

	reg_val = can_rzg3s_read32(dev_ctrl, R_CFD_RF_RFPTR(rf_idx));
	frame->dlc = R_CFD_RF_RFPTR_RFDLC(reg_val);
#ifdef CONFIG_CAN_RX_TIMESTAMP
	frame->timestamp = R_CFD_RF_RFPTR_RFTS(reg_val) >> 3;
#endif /* CAN_RX_TIMESTAMP */

	reg_val = can_rzg3s_read32(dev_ctrl, R_CFD_RF_RFFDSTS(rf_idx));
	if (reg_val & R_CFD_RF_RFFDSTS_RFFDF) {
		frame->flags |= CAN_FRAME_FDF;
	}
	if (reg_val & R_CFD_RF_RFFDSTS_RFBRS) {
		frame->flags |= CAN_FRAME_BRS;
	}
	if (reg_val & R_CFD_RF_RFFDSTS_RFESI) {
		frame->flags |= CAN_FRAME_ESI;
	}
	/* get RX FIFO Buffer Pointer Field RFPTR to get afl num*/
	*afl_num = R_CFD_RF_RFFDSTS_RFPTR(reg_val);

	can_rzg3s_rfifo_get_data(dev_ctrl, frame, rf_idx);

	/* Write 0xff to RFPCTR to increment the CPU-side
	 * pointer for the RX fifo
	 */
	can_rzg3s_write32(dev_ctrl, 0xff, R_CFD_RFPCTR(rf_idx));
}

static uint32_t can_rzg3s_tmb_to_tmbidx(uint32_t tmb_num, uint32_t ch_id)
{
	uint32_t ch_offset = ch_id ? CAN_RZG3S_TM_CH1_OFFSET : 0;
	uint32_t tmb_idx = tmb_num / 16;

	/* up to 32 TMB can be used per channels in below ranges
	 * CH0: TXMB0 - 15 and TXMB32 - 47 are available
	 * CH1: TXMB64 - 79 and TXMB96 - 111 are available
	 * so tmp_num need to be converted from [0..31] to the ranges acceptable by HW.
	 */
	return tmb_idx ? ch_offset + 32 + (tmb_num % 16) : ch_offset + (tmb_num % 16);
}

static void can_rzg3s_tmb_put_data(const struct device *dev_ctrl, const struct can_frame *frame,
				   uint32_t tmb_idx)
{
	uint32_t i, lwords;

	lwords = DIV_ROUND_UP(can_dlc_to_bytes(frame->dlc), sizeof(uint32_t));

	for (i = 0; i < lwords; i++) {
		can_rzg3s_write32(dev_ctrl, frame->data_32[i], R_CFD_TM_DF(tmb_idx, i));
	}
}

static void can_rzg3s_tmb_put_frame(const struct device *dev_ctrl, const struct can_frame *frame,
				    uint32_t tmb_idx, bool one_shot)
{
	uint32_t reg_val;
	uint8_t reg_val8;

	if (frame->flags & CAN_FRAME_IDE) {
		reg_val = frame->id & CAN_EXT_ID_MASK;
		reg_val |= R_CFD_TM_ID_TMIDE;
	} else {
		reg_val = frame->id & CAN_STD_ID_MASK;
	}

	if (frame->flags & CAN_FRAME_RTR) {
		reg_val |= R_CFD_TM_ID_TMRTR;
	}

	can_rzg3s_write32(dev_ctrl, reg_val, R_CFD_TM_ID(tmb_idx));

	reg_val = R_CFD_TM_PTR_DLC(frame->dlc);
	can_rzg3s_write32(dev_ctrl, reg_val, R_CFD_TM_PTR(tmb_idx));

	reg_val = 0;
	if (frame->flags & CAN_FRAME_FDF) {
		reg_val |= R_CFD_TM_FDCTR_TMFDF;
	}
	if (frame->flags & CAN_FRAME_BRS) {
		reg_val |= R_CFD_TM_FDCTR_TMBRS;
	}
	/* ESI bit depends on CFDCnFDCFG.ESIC cfg, but default represent node state.
	 * if CFDCnFDCFG.ESIC = 1 will used from frame unless err passive state.
	 * So, just set it here and let hw to decide.
	 */
	if (frame->flags & CAN_FRAME_ESI) {
		reg_val |= R_CFD_TM_FDCTR_TMESI;
	}
	can_rzg3s_write32(dev_ctrl, reg_val, R_CFD_TM_FDCTR(tmb_idx));

	can_rzg3s_tmb_put_data(dev_ctrl, frame, tmb_idx);

	/* request tx */
	reg_val8 = R_CFDC_TM_C_TMTR;
	if (one_shot) {
		reg_val8 |= R_CFDC_TM_C_TMOM;
	}
	can_rzg3s_write8(dev_ctrl, reg_val8, R_CFDC_TM_C(tmb_idx));
}

static void can_rzg3s_tmb_irq_en(const struct device *dev, uint32_t tmb_num)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;

	uint32_t regs = tmb_num / 16;
	uint32_t bits = tmb_num % 16;
	int i;

	for (i = 0; i < regs; i++) {
		can_rzg3s_write32(cfg->ctrl_dev, 0xffff, R_CFDC_TM_IEC(cfg->id, i));
	}

	can_rzg3s_write32(cfg->ctrl_dev, BIT_MASK(bits), R_CFDC_TM_IEC(cfg->id, i));
}

static int can_rzg3s_tmb_get_free(const struct device *dev)
{
	struct can_rzg3s_chn_data *ctx = dev->data;
	k_spinlock_key_t key;
	int tmb;

	key = k_spin_lock(&ctx->tx_tmb_lock);

	tmb = find_lsb_set(ctx->tx_tmb_free);
	if (tmb) {
		tmb--;
		ctx->tx_tmb_free &= ~BIT(tmb);
	} else {
		tmb = -EBUSY;
	}

	k_spin_unlock(&ctx->tx_tmb_lock, key);
	return tmb;
}

static void can_rzg3s_tmb_handle_compl_one(const struct device *dev, uint32_t tmb_num)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	uint8_t tx_result, reg_val8;
	can_tx_callback_t tx_cb;
	k_spinlock_key_t key;
	uint32_t tmb_idx;
	void *cb_arg;
	int status = 0;

	tmb_idx = can_rzg3s_tmb_to_tmbidx(tmb_num, cfg->id);

	/* get and clear tmb status */
	reg_val8 = can_rzg3s_read8(cfg->ctrl_dev, R_CFDC_TM_STS(tmb_idx));
	tx_result = R_CFDC_TM_STS_TMTRF(reg_val8);
	reg_val8 &= ~R_CFDC_TM_STS_TMTRF_MASK;
	can_rzg3s_write8(cfg->ctrl_dev, reg_val8, R_CFDC_TM_STS(tmb_idx));

	if (tx_result == R_CFDC_TM_STS_TMTRF_NORES) {
		LOG_DEV_WRN(dev, "TXC: no result. tmb:%u sts:%u", tmb_num, tx_result);
		return;
	} else if (tx_result == R_CFDC_TM_STS_TMTRF_ABORT) {
		if (ctx->state == CAN_STATE_BUS_OFF) {
			status = -ENETUNREACH;
		} else if (ctx->state == CAN_STATE_STOPPED) {
			status = -ENETDOWN;
		} else {
			status = -EIO;
		}
	}

	key = k_spin_lock(&ctx->tx_tmb_lock);

	tx_cb = ctx->tx_entries[tmb_num].tx_cb;
	cb_arg = ctx->tx_entries[tmb_num].cb_arg;
	ctx->tx_entries[tmb_num].tx_cb = NULL;
	ctx->tx_entries[tmb_num].cb_arg = NULL;
	ctx->tx_tmb_free |= BIT(tmb_num);

	k_spin_unlock(&ctx->tx_tmb_lock, key);

	if (tx_cb) {
		LOG_DEV_DBG(dev, "TXC: tmb:%d tmb_idx:%u tx_result:%u", tmb_num, tmb_idx,
			    tx_result);
		k_sem_give(&ctx->tx_sem);
		tx_cb(dev, status, cb_arg);
	} else {
		LOG_DEV_DBG(dev, "TXC: no tx_cb. tmb:%u sts:%u", tmb_num, tx_result);
	}
}

static void can_rzg3s_tmb_handle_compl(const struct device *dev, bool aborted)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	uint32_t reg_ofs;
	uint32_t tx_sts;
	uint32_t tmb;

	if (aborted) {
		reg_ofs = R_CFDC_TM_TASTS(cfg->id, 0);
	} else {
		reg_ofs = R_CFDC_TM_TCSTS(cfg->id, 0);
	}

	tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	for (tmb = 0; (tmb < 16) && tx_sts; tmb++) {
		/* process one tmb */
		if (tx_sts & BIT(tmb)) {
			can_rzg3s_tmb_handle_compl_one(dev, tmb);
		}
		tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	}

	reg_ofs += 0x4;
	tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	for (tmb = 16; (tmb < CAN_RZG3S_TX_MAX_PKTS) && tx_sts; tmb++) {
		/* process one tmb */
		if (tx_sts & BIT(tmb - 16)) {
			can_rzg3s_tmb_handle_compl_one(dev, tmb);
		}
		tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	}
}

static void can_rzg3s_tmb_abort_all(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	uint32_t reg_ofs;
	uint8_t reg_val8;
	uint32_t tmb_idx;
	uint32_t tx_sts;
	uint32_t tmb;

	reg_ofs = R_CFDC_TM_TRSTS(cfg->id, 0);

	tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	for (tmb = 0; (tmb < 16) && tx_sts; tmb++) {
		if (tx_sts & BIT(tmb)) {
			tmb_idx = can_rzg3s_tmb_to_tmbidx(tmb, cfg->id);

			reg_val8 = can_rzg3s_read8(cfg->ctrl_dev, R_CFDC_TM_C(tmb_idx));
			reg_val8 |= R_CFDC_TM_C_TMTAR;
			can_rzg3s_write8(cfg->ctrl_dev, reg_val8, R_CFDC_TM_C(tmb_idx));
		}
		tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	}

	reg_ofs += 0x4;
	tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	for (tmb = 16; (tmb < CAN_RZG3S_TX_MAX_PKTS) && tx_sts; tmb++) {
		if (tx_sts & BIT(tmb - 16)) {
			tmb_idx = can_rzg3s_tmb_to_tmbidx(tmb, cfg->id);

			reg_val8 = can_rzg3s_read8(cfg->ctrl_dev, R_CFDC_TM_C(tmb_idx));
			reg_val8 |= R_CFDC_TM_C_TMTAR;
			can_rzg3s_write8(cfg->ctrl_dev, reg_val8, R_CFDC_TM_C(tmb_idx));
		}
		tx_sts = can_rzg3s_read32(cfg->ctrl_dev, reg_ofs);
	}
}

static int can_rzg3s_chn_set_hw_mode(const struct device *dev, uint8_t chn, uint8_t mode)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	const struct device *dev_ctrl = cfg->ctrl_dev;
	int ret = -EINVAL;
	uint32_t reg_val;

	reg_val = can_rzg3s_read32(dev_ctrl, R_CFDC_CTR(chn)) & R_CFDC_CTR_CHDC_MASK;

	if (reg_val == mode) {
		return 0;
	}

	/* no SLEEP support */
	can_rzg3s_modify32(dev_ctrl, R_CFDC_CTR(chn), R_CFDC_CTR_CHDC_MASK, mode);

	if (mode == R_CFDC_CTR_CHDC_OPM) {
		ret = can_rzg3s_wait_bits0(dev_ctrl, R_CFDC_STS(chn), R_CFDC_STS_CNOPM);
	} else if (mode == R_CFDC_CTR_CHDC_RESET) {
		ret = can_rzg3s_wait_bits1(dev_ctrl, R_CFDC_STS(chn), R_CFDC_STS_RSTSTS);
	} else if (mode == R_CFDC_CTR_CHDC_HALT) {
		ret = can_rzg3s_wait_bits1(dev_ctrl, R_CFDC_STS(chn), R_CFDC_STS_HLTSTS);
	}

	return ret;
}

static void can_rzg3s_chn_set_nrate(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	const struct device *dev_ctrl = cfg->ctrl_dev;
	uint32_t reg_ncfg;

	reg_ncfg = (ctx->timing.prescaler - 1) & GENMASK(9, 0);
	reg_ncfg |= ((ctx->timing.sjw - 1) << 10) & GENMASK(16, 10);
	reg_ncfg |= ((ctx->timing.prop_seg + ctx->timing.phase_seg1 - 1) << 17) & GENMASK(24, 17);
	reg_ncfg |= ((ctx->timing.phase_seg2 - 1) << 25) & GENMASK(31, 25);

	can_rzg3s_write32(dev_ctrl, reg_ncfg, R_CFDC_NCFG(cfg->id));
}

static void can_rzg3s_chn_set_drate(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	const struct device *dev_ctrl = cfg->ctrl_dev;
	uint32_t reg_ncfg;

	reg_ncfg = (ctx->timing_data.prescaler - 1) & GENMASK(7, 0);
	reg_ncfg |= ((ctx->timing_data.prop_seg + ctx->timing_data.phase_seg1 - 1) << 8) &
		    GENMASK(12, 8);
	reg_ncfg |= ((ctx->timing_data.phase_seg2 - 1) << 16) & GENMASK(19, 16);
	reg_ncfg |= ((ctx->timing_data.sjw - 1) << 24) & GENMASK(27, 24);

	can_rzg3s_write32(dev_ctrl, reg_ncfg, R_CFDC_DCFG(cfg->id));
}

static void can_rzg3s_chn_set_tdco(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	const struct device *dev_ctrl = cfg->ctrl_dev;
	uint32_t reg_val;

	if (!cfg->tx_delay_comp_offset) {
		return;
	}
	reg_val = can_rzg3s_read32(dev_ctrl, R_CFDC_FDCFG(cfg->id));
	reg_val &= ~R_CFDC_FDCFG_TDCO_MSK;
	reg_val |= (cfg->tx_delay_comp_offset << R_CFDC_FDCFG_TDCO_OFS) & R_CFDC_FDCFG_TDCO_MSK;
	if (cfg->tx_delay_comp_offset_only) {
		reg_val |= R_CFDC_FDCFG_TDCO_TDCOC;
	}
	reg_val |= R_CFDC_FDCFG_TDCO_TDCE;

	can_rzg3s_write32(dev_ctrl, reg_val, R_CFDC_FDCFG(cfg->id));
}

static int can_rzg3s_chn_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;

	if (rate) {
		*rate = can_rzg3s_g_get_core_clock(cfg->ctrl_dev);
	}

	return 0;
}

static int can_rzg3s_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	return CAN_RZGS_FD_MAX_FILTER;
}

static int can_rzg3s_chn_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;

	*max_bitrate = cfg->max_bitrate;
	return 0;
}

static int can_rzg3s_chn_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_rzg3s_chn_data *ctx = dev->data;
	int ret = 0;

	if (!timing) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);
	if (ctx->f_started) {
		ret = -EBUSY;
		goto unlock;

	}

	ctx->timing = *timing;
	LOG_DEV_DBG(dev, "set nominal presc %d, seg1:%d, seg2:%d sjw:%d", ctx->timing.prescaler,
		    ctx->timing.phase_seg1, ctx->timing.phase_seg2, ctx->timing.sjw);

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}

static int can_rzg3s_chn_set_timing_data(const struct device *dev,
					 const struct can_timing *timing_data)
{
	struct can_rzg3s_chn_data *ctx = dev->data;
	int ret = 0;

	if (!timing_data) {
		return -EINVAL;
	}

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);
	if (ctx->f_started) {
		ret = -EBUSY;
		goto unlock;
	}

	ctx->timing_data = *timing_data;
	LOG_DEV_DBG(dev, "set data presc:%d, seg1:%d, seg2:%d sjw:%d", ctx->timing_data.prescaler,
		    ctx->timing_data.phase_seg1, ctx->timing_data.phase_seg2, ctx->timing_data.sjw);

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}

static int can_rzg3s_chn_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	if (!cap) {
		return -EIO;
	}

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_FD |
	       CAN_MODE_ONE_SHOT;

	return 0;
}

static int can_rzg3s_chn_set_mode(const struct device *dev, can_mode_t mode)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	uint32_t ctm = 0;
	int ret = 0;

	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_FD | CAN_MODE_ONE_SHOT))) {
		LOG_DEV_ERR(dev, "mode: unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if ((mode & (CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY)) ==
	    (CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY)) {
		LOG_DEV_ERR(dev, "mode: both loopback and listen-only modes not supported");
		return -ENOTSUP;
	}

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (ctx->f_started) {
		ret = -EBUSY;
		goto unlock;
	}

	ret = can_rzg3s_chn_set_hw_mode(dev, cfg->id, R_CFDC_CTR_CHDC_HALT);
	if (ret) {
		LOG_DEV_ERR(dev, "mode: failed to set halt mode %d", ret);
		goto unlock;
	}

	ctx->can_mode = mode;
	LOG_DEV_DBG(dev, "set mode %d", mode);

	if (!(mode & (CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY))) {
		can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id),
				   R_CFDC_CTR_CTME | R_CFDC_CTR_CTMS_MASK, 0);
		goto unlock;
	}

	if (mode & CAN_MODE_LOOPBACK) {
		ctm = R_CFDC_CTR_CTME | R_CFDC_CTR_CTMS(R_CFDC_CTR_CTMS_INT_LOOPBACK);
	} else if (mode & CAN_MODE_LISTENONLY) {
		ctm = R_CFDC_CTR_CTME | R_CFDC_CTR_CTMS(R_CFDC_CTR_CTMS_LISTENONLY);
	}

	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id),
			   R_CFDC_CTR_CTME | R_CFDC_CTR_CTMS_MASK, ctm);

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}

static int can_rzg3s_chn_start(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	int ret = 0;

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (ctx->f_started) {
		LOG_DEV_INF(dev, "start: already started");
		ret = -EALREADY;
		goto unlock;
	}

	LOG_DEV_DBG(dev, "start enter");

	if (cfg->phy) {
		ret = can_transceiver_enable(cfg->phy);
		if (ret) {
			LOG_DEV_ERR(dev, "start: failed to enable can-transceiver %d", ret);
			goto unlock;
		}
	}

	can_rzg3s_chn_set_nrate(dev);
	can_rzg3s_chn_set_drate(dev);
	can_rzg3s_chn_set_tdco(dev);

	/* restore can (test) mode cfg */
	ret = can_rzg3s_chn_set_mode(dev, ctx->can_mode);
	if (ret) {
		goto unlock;
	}

	can_rzg3s_write32(cfg->ctrl_dev, 0, R_CFDC_ERFL(cfg->id));

	ret = can_rzg3s_chn_set_hw_mode(dev, cfg->id, R_CFDC_CTR_CHDC_OPM);
	if (ret) {
		can_rzg3s_chn_set_hw_mode(dev, cfg->id, R_CFDC_CTR_CHDC_RESET);
		goto unlock;
	}

	irq_enable(cfg->irq_ch_err);
	irq_enable(cfg->irq_ch_trx);
	/* Enable Rx FIFO */
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFD_RFCC(cfg->id), 0, R_CFD_RFCC_RFE);

	ctx->state = CAN_STATE_ERROR_ACTIVE;
	ctx->f_started = true;
	STATS_INC(ctx->stats, error_active_state);

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}

static int can_rzg3s_chn_stop(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	int ret = 0;

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (!ctx->f_started) {
		LOG_DEV_DBG(dev, "stop: already stopped");
		ret = -EALREADY;
		goto unlock;
	}

	LOG_DEV_DBG(dev, "stop enter");

	ret = can_rzg3s_chn_set_hw_mode(dev, cfg->id, R_CFDC_CTR_CHDC_HALT);
	if (ret) {
		LOG_DEV_ERR(dev, "stop: failed to set halt mode %d", ret);
	}

	ctx->state = CAN_STATE_STOPPED;
	ctx->f_started = false;

	if (cfg->phy) {
		ret = can_transceiver_disable(cfg->phy);
		if (ret) {
			LOG_DEV_ERR(dev, "stop: failed to disable can-stransceiver %d", ret);
		}
	}

	/* Disable Rx FIFO */
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFD_RFCC(cfg->id), R_CFD_RFCC_RFE, 0);

	/* clean up TX */
	can_rzg3s_tmb_abort_all(dev);

	k_sleep(K_MSEC(100));
	if (k_sem_count_get(&ctx->tx_sem) != CAN_RZG3S_TX_MAX_PKTS) {
		LOG_DEV_ERR(dev, "stop: not all tmb are freed (%d < %d)",
			    k_sem_count_get(&ctx->tx_sem), CAN_RZG3S_TX_MAX_PKTS);
	}

	irq_disable(cfg->irq_ch_err);
	irq_disable(cfg->irq_ch_trx);

	ret = can_rzg3s_chn_set_hw_mode(dev, cfg->id, R_CFDC_CTR_CHDC_RESET);
	if (ret) {
		LOG_DEV_ERR(dev, "stop: failed to set reset mode %d", ret);
	}

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}

static int can_rzg3s_chn_get_state(const struct device *dev, enum can_state *state,
				   struct can_bus_err_cnt *err_cnt)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	uint32_t reg_val;

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (state) {
		*state = ctx->state;
	}

	if (err_cnt) {
		reg_val = can_rzg3s_read32(cfg->ctrl_dev, R_CFDC_STS(cfg->id));
		err_cnt->tx_err_cnt = R_CFDC_STS_TEC(reg_val);
		err_cnt->rx_err_cnt = R_CFDC_STS_REC(reg_val);
	}

	k_mutex_unlock(&ctx->ch_mutex);

	return 0;
}

static int can_rzg3s_chn_send(const struct device *dev, const struct can_frame *frame,
			      k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	int ret = 0;
	int tmb;

	__ASSERT(frame, "frame is null");

	if (ctx->can_mode & CAN_MODE_FD) {
		if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_FDF | CAN_FRAME_BRS |
				      CAN_FRAME_RTR | CAN_FRAME_ESI))) {
			LOG_DEV_ERR(dev, "TX: unsupported framefd flags 0x%02x", frame->flags);
			return -ENOTSUP;
		}
		if (frame->dlc > CANFD_MAX_DLC) {
			LOG_DEV_ERR(dev, "TX: framefd dlc %u exceeds maximum (%d)", frame->dlc,
				    CANFD_MAX_DLC);
			return -EINVAL;
		}
	} else {
		if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR))) {
			LOG_DEV_ERR(dev, "TX: unsupported frame flags 0x%02x", frame->flags);
			return -ENOTSUP;
		}
		if (frame->dlc > CAN_MAX_DLC) {
			LOG_DEV_ERR(dev, "TX: frame dlc %u exceeds maximum (%d)", frame->dlc,
				    CAN_MAX_DLC);
			return -EINVAL;
		}
	}

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (!ctx->f_started) {
		LOG_DEV_DBG(dev, "TX: in stopped state");
		ret = -ENETDOWN;
		goto unlock;
	}

	if (ctx->state == CAN_STATE_BUS_OFF) {
		LOG_DEV_ERR(dev, "TX: in bus-off state");
		ret = -ENETUNREACH;
		goto unlock;
	}

	if (ctx->can_mode & CAN_MODE_LISTENONLY) {
		LOG_DEV_ERR(dev, "TX: in listen-only mode");
		ret = -ENETUNREACH;
		goto unlock;
	}

	ret = k_sem_take(&ctx->tx_sem, timeout);
	if (ret) {
		LOG_DEV_DBG(dev, "TX: no more tx buffers");
		ret = -EAGAIN;
		goto unlock;
	}

	tmb = can_rzg3s_tmb_get_free(dev);
	if (ret < 0) {
		k_sem_give(&ctx->tx_sem);
		ret = tmb;
		goto unlock;
	}

	LOG_DEV_DBG(dev, "TX: frame id:%08x dlc:%u flags:%02x  tmb:%d tmb_idx:%u", frame->id,
		    frame->dlc, frame->flags, tmb, can_rzg3s_tmb_to_tmbidx(tmb, cfg->id));

	ctx->tx_entries[tmb].tx_cb = callback;
	ctx->tx_entries[tmb].cb_arg = user_data;
	can_rzg3s_tmb_put_frame(cfg->ctrl_dev, frame, can_rzg3s_tmb_to_tmbidx(tmb, cfg->id),
				!!(ctx->can_mode & CAN_MODE_ONE_SHOT));

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}

void can_rzg3s_chn_handle_rx(const struct device *dev, uint32_t rf_idx)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	uint32_t rf_sts, rf_empty, afl_num;

	rf_sts = can_rzg3s_read32(cfg->ctrl_dev, R_CFD_RFSTS(rf_idx));
	LOG_DEV_DBG(dev, "RX: rfifo stst %08x", rf_sts);

	rf_empty = can_rzg3s_read32(cfg->ctrl_dev, R_CFD_RF_ESTS);

	while (!(rf_empty & BIT(rf_idx))) {
		struct can_frame frame = {0};

		can_rzg3s_rfifo_get_frame(cfg->ctrl_dev, &frame, rf_idx, &afl_num);

		LOG_DEV_DBG(dev, "RX: frame id:%08x dlc:%u flags:%02x afl:%u", frame.id,
			    frame.dlc, frame.flags, afl_num);
#ifdef CONFIG_CAN_RX_TIMESTAMP
		LOG_DEV_DBG(dev, "RX: frame timestamp:%04x", frame.timestamp);
#endif /* CAN_RX_TIMESTAMP */

		if (atomic_test_bit(ctx->rx_allocs, afl_num)) {
			/* FDF filter - no hw support */
			if (!(ctx->rx_afl[afl_num].fdf && !(frame.flags | CAN_FRAME_FDF))) {
				ctx->rx_afl[afl_num].rx_cb(dev, &frame,
							   ctx->rx_afl[afl_num].cb_arg);
			} else {
				LOG_DEV_ERR(dev, "RX: fdf filter drop afl:%u", afl_num);
			}
		} else {
			LOG_DEV_DBG(dev, "RX: frame for removed afl:%u", afl_num);
		}

		rf_empty = can_rzg3s_read32(cfg->ctrl_dev, R_CFD_RF_ESTS);
	}

	can_rzg3s_modify32(cfg->ctrl_dev, R_CFD_RFSTS(cfg->id), R_CFD_RFSTS_RFIF, 0);
}

static int can_rzg3s_chn_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				       void *user_data, const struct can_filter *filter)
{
	uint8_t supported = CAN_FILTER_IDE | CAN_FILTER_DATA | CAN_FILTER_RTR | CAN_FILTER_FDF;
	struct can_rzg3s_chn_data *ctx = dev->data;
	int afl_alloc = -ENOSPC;
	int i;

	__ASSERT(filter, "rx_filter can not be null");
	__ASSERT(callback, "rx_callback can not be null");

	if ((filter->flags & ~(supported)) != 0) {
		LOG_DEV_ERR(dev, "AFLADD: unsupported filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	/* Find and allocate RX message buffer */
	for (i = 0; i < CAN_RZGS_FD_MAX_FILTER; i++) {
		if (!atomic_test_and_set_bit(ctx->rx_allocs, i)) {
			afl_alloc = i;
			break;
		}
	}

	if (afl_alloc == -ENOSPC) {
		LOG_DEV_ERR(dev, "AFLADD: no free entries");
		goto unlock;
	}

	ctx->rx_afl[afl_alloc].rx_cb = callback;
	ctx->rx_afl[afl_alloc].cb_arg = user_data;
	ctx->rx_afl[afl_alloc].fdf = !!(filter->flags & CAN_FILTER_FDF);
	can_rzg3s_afl_add(dev, afl_alloc, filter);

unlock:
	k_mutex_unlock(&ctx->ch_mutex);

	return afl_alloc;
}

static void can_rzg3s_chn_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_rzg3s_chn_data *ctx = dev->data;

	if (filter_id >= CAN_RZGS_FD_MAX_FILTER) {
		LOG_DEV_ERR(dev, "AFLDEL: filter id >= max (%d >= %d)", filter_id,
			    CAN_RZGS_FD_MAX_FILTER);
		return;
	}

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (atomic_test_and_clear_bit(ctx->rx_allocs, filter_id)) {
		ctx->rx_afl[filter_id].rx_cb = NULL;
		ctx->rx_afl[filter_id].cb_arg = NULL;
		can_rzg3s_afl_del(dev, filter_id);
	} else {
		LOG_DEV_WRN(dev, "AFLDEL: filter id %d already detached", filter_id);
	}

	k_mutex_unlock(&ctx->ch_mutex);
}

static void can_rzg3s_chn_state_change(const struct device *dev, uint32_t newstate)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	struct can_bus_err_cnt err_cnt;
	uint32_t reg_val;

	if (ctx->state == newstate) {
		return;
	}

	LOG_DEV_DBG(dev, "can state change new: %u old:%u", newstate, ctx->state);

	ctx->state = newstate;

	if (!ctx->state_change_cb) {
		return;
	}

	reg_val = can_rzg3s_read32(cfg->ctrl_dev, R_CFDC_STS(cfg->id));
	err_cnt.tx_err_cnt = R_CFDC_STS_TEC(reg_val);
	err_cnt.rx_err_cnt = R_CFDC_STS_REC(reg_val);

	ctx->state_change_cb(dev, newstate, err_cnt, ctx->state_change_cb_data);
}

static void can_rzg3s_chn_set_state_change_callback(const struct device *dev,
						    can_state_change_callback_t callback,
						    void *user_data)
{
	struct can_rzg3s_chn_data *ctx = dev->data;

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);
	ctx->state_change_cb = callback;
	ctx->state_change_cb_data = user_data;
	k_mutex_unlock(&ctx->ch_mutex);
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static int can_rzg3s_chn_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	struct can_bus_err_cnt err_cnt;
	int64_t start_time;
	uint32_t reg_val;
	int ret = 0;

	k_mutex_lock(&ctx->ch_mutex, K_FOREVER);

	if (!ctx->f_started) {
		ret = -ENETDOWN;
		goto unlock;
	}

	if (ctx->state != CAN_STATE_BUS_OFF) {
		goto unlock;
	}

	start_time = k_uptime_ticks();

	ret = can_rzg3s_chn_set_hw_mode(dev, cfg->id, R_CFDC_CTR_CHDC_OPM);
	if (ret) {
		LOG_DEV_ERR(dev, "recover: failed to set OPM mode %d", ret);
		goto unlock;
	}

	if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		while (ctx->state != CAN_STATE_BUS_OFF) {
			if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
			    k_uptime_ticks() - start_time >= timeout.ticks) {
				ret = -EAGAIN;
				LOG_DEV_INF(dev, "recover: timeout");
				goto unlock;
			}
		}
	}

	STATS_INC(ctx->stats, bus_off_recovery_manual);

	reg_val = can_rzg3s_read32(cfg->ctrl_dev, R_CFDC_STS(cfg->id));
	err_cnt.tx_err_cnt = R_CFDC_STS_TEC(reg_val);
	err_cnt.rx_err_cnt = R_CFDC_STS_REC(reg_val);
	if (err_cnt.tx_err_cnt < 96 && err_cnt.tx_err_cnt < 96) {
		STATS_INC(ctx->stats, error_active_state);
		can_rzg3s_chn_state_change(dev, CAN_STATE_ERROR_ACTIVE);
	}

unlock:
	k_mutex_unlock(&ctx->ch_mutex);
	return ret;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

static void can_rzg3s_isr_trx(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	uint8_t tx_gsts;

	tx_gsts = can_rzg3s_read8(cfg->ctrl_dev, R_CFDG_TINTSTS0(cfg->id));
	LOG_DEV_DBG(dev, "ch_trx: %02x", tx_gsts);

	while (tx_gsts) {
		if (tx_gsts & R_CFDG_TINTSTS0_TSIF) {
			/* handle TX Successful*/
			can_rzg3s_tmb_handle_compl(dev, false);
		}

		if (tx_gsts & R_CFDG_TINTSTS0_TAIF) {
			/* handle TX Abort */
			can_rzg3s_tmb_handle_compl(dev, true);
		}

		tx_gsts = can_rzg3s_read8(cfg->ctrl_dev, R_CFDG_TINTSTS0(cfg->id));
		LOG_DEV_DBG(dev, "ch_trx: 1 %02x", tx_gsts);
	}
}

static void can_rzg3s_isr_err(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	uint32_t erfl, fdsts;

	ARG_UNUSED(ctx);

	erfl = can_rzg3s_read32(cfg->ctrl_dev, R_CFDC_ERFL(cfg->id));
	LOG_DEV_DBG(dev, "ch_err: %08x", erfl);
	fdsts = can_rzg3s_read32(cfg->ctrl_dev, R_CFDC_FDSTS(cfg->id));

	if (erfl & R_CFDC_ERFL_ADERR) {
		CAN_STATS_ACK_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_B0ERR) {
		CAN_STATS_BIT0_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_B1ERR) {
		CAN_STATS_BIT1_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_CERR) {
		CAN_STATS_CRC_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_AERR) {
		CAN_STATS_ACK_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_FERR) {
		CAN_STATS_FORM_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_SERR) {
		CAN_STATS_STUFF_ERROR_INC(dev);
	}
	if (erfl & R_CFDC_ERFL_OVLF) {
		CAN_STATS_RX_OVERRUN_INC(dev);
	}

	if (erfl & R_CFDC_ERFL_ALF) {
		STATS_INC(ctx->stats, arbitration_lost);
		LOG_DEV_DBG(dev, "arbitration lost flag");
	}

	if (erfl & R_CFDC_ERFL_BLF) {
		STATS_INC(ctx->stats, bus_lock);
		LOG_DEV_ERR(dev, "bus lock detected");
		can_rzg3s_chn_state_change(dev, CAN_STATE_BUS_OFF);
	}

	if (erfl & R_CFDC_ERFL_BORF) {
		STATS_INC(ctx->stats, bus_off_recovery);
		STATS_INC(ctx->stats, error_active_state);
		can_rzg3s_chn_state_change(dev, CAN_STATE_ERROR_ACTIVE);
	}

	if (erfl & R_CFDC_ERFL_EWF) {
		STATS_INC(ctx->stats, error_warning_state);
		can_rzg3s_chn_state_change(dev, CAN_STATE_ERROR_WARNING);
	}

	if (erfl & R_CFDC_ERFL_EPF) {
		STATS_INC(ctx->stats, error_passive_state);
		can_rzg3s_chn_state_change(dev, CAN_STATE_ERROR_PASSIVE);
	}

	if (erfl & R_CFDC_ERFL_BOEF) {
		STATS_INC(ctx->stats, bus_off_state);
		can_rzg3s_chn_state_change(dev, CAN_STATE_BUS_OFF);
	}

	can_rzg3s_write32(cfg->ctrl_dev, ~erfl, R_CFDC_ERFL(cfg->id));

	if (fdsts & R_CFDC_FDSTS_TDCO_TDCVF) {
		STATS_INC(ctx->stats, tdc_violation);
		LOG_DEV_DBG(dev, "transceiver delay compensation violation %02x",
			    (uint32_t)(fdsts & R_CFDC_FDSTS_TDCO_TDCR));
		can_rzg3s_write32(cfg->ctrl_dev, ~R_CFDC_FDSTS_TDCO_TDCVF, R_CFDC_FDSTS(cfg->id));
	}
}

static const struct can_driver_api can_rzg3s_can_api = {
	.get_capabilities = can_rzg3s_chn_get_capabilities,
	.start = can_rzg3s_chn_start,
	.stop = can_rzg3s_chn_stop,
	.set_mode = can_rzg3s_chn_set_mode,
	.set_timing = can_rzg3s_chn_set_timing,
	.set_timing_data = can_rzg3s_chn_set_timing_data,
	.send = can_rzg3s_chn_send,
	.add_rx_filter = can_rzg3s_chn_add_rx_filter,
	.remove_rx_filter = can_rzg3s_chn_remove_rx_filter,
	.get_state = can_rzg3s_chn_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_rzg3s_chn_recover,
#endif
	.set_state_change_callback = can_rzg3s_chn_set_state_change_callback,
	.get_core_clock = can_rzg3s_chn_get_core_clock,
	.get_max_filters = can_rzg3s_get_max_filters,
	.get_max_bitrate = can_rzg3s_chn_get_max_bitrate,
	/*
	 * Note that the values here are the "physical" timing limits, whereas
	 * the register field limits are physical values minus 1
	 */
	.timing_min = {.sjw = 2, .prop_seg = 0, .phase_seg1 = 2, .phase_seg2 = 2, .prescaler = 1},
	.timing_max = {.sjw = 128,
		       .prop_seg = 0,
		       .phase_seg1 = 256,
		       .phase_seg2 = 128,
		       .prescaler = 1024},
	.timing_data_min = {
		.sjw = 1, .prop_seg = 0, .phase_seg1 = 2, .phase_seg2 = 2, .prescaler = 1},
	.timing_data_max = {
		.sjw = 16, .prop_seg = 0, .phase_seg1 = 32, .phase_seg2 = 16, .prescaler = 256},
};

static int can_rzg3s_chn_init(const struct device *dev)
{
	const struct can_rzg3s_chn_config *cfg = dev->config;
	struct can_rzg3s_chn_data *ctx = dev->data;
	uint32_t reg_val;
	int ret;

	ret = can_calc_timing(dev, &ctx->timing, cfg->bitrate, cfg->sample_point);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "can't find nom timing for given param %d", ret);
		return ret;
	}
	if (ret) {
		LOG_DEV_INF(dev, "nominal sample-point err : %d", ret);
	}

	ret = can_calc_timing_data(dev, &ctx->timing_data, cfg->bitrate_data,
				   cfg->sample_point_data);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "can't find data timing for given param %d", ret);
		return ret;
	}
	if (ret) {
		LOG_DEV_INF(dev, "data sample-point err : %d", ret);
	}

	/* Validate initial timing parameters */
	ret = can_set_timing(dev, &ctx->timing);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to set nom timing %d", ret);
		return ret;
	}
	/* Validate initial data phase timing parameters */
	ret = can_set_timing_data(dev, &ctx->timing_data);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to set data phase timing %d", ret);
		return ret;
	}

	k_sem_init(&ctx->tx_sem, CAN_RZG3S_TX_MAX_PKTS, CAN_RZG3S_TX_MAX_PKTS);
	k_mutex_init(&ctx->ch_mutex);
	ctx->tx_tmb_free = UINT32_MAX;

	cfg->irq_config();

	/* clear channels sleep */
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id), R_CFDC_CTR_CSLPR, 0);

	/* Error Display Mode - all */
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id), 0, R_CFDC_CTR_ERRD);

#ifdef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	/* Bus-Off Recovery Mode - 00: ISO11898-1 compliant */
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id), R_CFDC_CTR_BOM_MASK,
			   R_CFDC_CTR_BOM(R_CFDC_CTR_BOM_ISO));
#else
	/* Bus-Off Recovery Mode - 01: Entry to Channel Halt mode automatically at bus-off entry */
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id), R_CFDC_CTR_BOM_MASK,
			   R_CFDC_CTR_BOM(R_CFDC_CTR_BOM_BENTRY));
#endif

	/* Channel interrupts setup */
	reg_val = (R_CFDC_CTR_TAIE | R_CFDC_CTR_ALIE | R_CFDC_CTR_BLIE | R_CFDC_CTR_OLIE |
		   R_CFDC_CTR_BORIE | R_CFDC_CTR_BOEIE | R_CFDC_CTR_EPIE | R_CFDC_CTR_EWIE |
		   R_CFDC_CTR_BEIE);
	if (cfg->tx_delay_comp_offset) {
		reg_val |= R_CFDC_CTR_TDCVFIE;
	}
	can_rzg3s_modify32(cfg->ctrl_dev, R_CFDC_CTR(cfg->id), 0, reg_val);

	can_rzg3s_tmb_irq_en(dev, CAN_RZG3S_TX_MAX_PKTS);

#ifdef CONFIG_CAN_STATS
	stats_init_and_reg(&ctx->stats.s_hdr, sizeof(uint32_t),
			   (sizeof(ctx->stats) - sizeof(struct stats_hdr)) / sizeof(uint32_t),
			   STATS_NAME_INIT_PARMS(can_rzg3s_ch), cfg->stat_gr_name);
#endif /* CONFIG_CAN_STATS */

	can_rzg3s_g_rfifo_register(cfg->ctrl_dev, cfg->id, dev);

	LOG_DEV_INF(dev, "init done");

	return 0;
}

static void can_rzg3s_g_err_isr(const struct device *dev)
{
	struct can_rzg3s_data *ctx = dev->data;
	uint32_t reg_gerfl;
	uint32_t rx_gsts;
	uint8_t rf_idx;

	ARG_UNUSED(ctx);

	/* Handle global error interrupts */
	reg_gerfl = can_rzg3s_read32(dev, R_CFDG_ERFL);
	LOG_DEV_DBG(dev, "gerr %08x", reg_gerfl);

	if (reg_gerfl & R_CFDG_ERFL_EEF1) {
		STATS_INC(ctx->stats, ecc_error_ch1);
		LOG_DEV_ERR(dev, "ecc error has occurred on chn1");
	}
	if (reg_gerfl & R_CFDG_ERFL_EEF0) {
		STATS_INC(ctx->stats, ecc_error_ch0);
		LOG_DEV_ERR(dev, "ecc error has occurred on chn0");
	}
	if (reg_gerfl & R_CFDG_ERFL_CMPOF) {
		STATS_INC(ctx->stats, canfd_msg_payload_overflow);
		LOG_DEV_DBG(dev, "can-fd message payload overflow");
	}

	if (reg_gerfl & R_CFDG_ERFL_MES) {
		LOG_DEV_DBG(dev, "message lost error");

		rx_gsts = can_rzg3s_read32(dev, R_CFD_RF_MSTS) & 0xff;
		LOG_DEV_DBG(dev, "gerr rfifo msts %08x", rx_gsts);

		while (rx_gsts) {
			rf_idx = find_lsb_set(rx_gsts) - 1;

			can_rzg3s_modify32(dev, R_CFD_RFSTS(rf_idx), R_CFD_RFSTS_RFMLT, 0);
			if (!rf_idx) {
				STATS_INC(ctx->stats, rx_msg_lost_rfifo0);
			} else if (rf_idx == 1) {
				STATS_INC(ctx->stats, rx_msg_lost_rfifo1);
			}

			rx_gsts = can_rzg3s_read32(dev, R_CFD_RF_MSTS) & 0xff;
			LOG_DEV_DBG(dev, "gerr rfifo msts 1 %08x", rx_gsts);
		}
	}

	can_rzg3s_write32(dev, ~reg_gerfl, R_CFDG_ERFL);
}

static void can_rzg3s_g_recc_isr(const struct device *dev)
{
	struct can_rzg3s_data *ctx = dev->data;
	uint32_t rx_gsts;
	uint8_t rf_idx;

	rx_gsts = can_rzg3s_read32(dev, R_CFD_RF_ISTS) & 0xff;
	LOG_DEV_DBG(dev, "grecc rfifo ists %08x", rx_gsts);

	while (rx_gsts) {
		rf_idx = find_lsb_set(rx_gsts) - 1;

		can_rzg3s_chn_handle_rx(ctx->rx_fifo_ch_devs[rf_idx], rf_idx);

		rx_gsts = can_rzg3s_read32(dev, R_CFD_RF_ISTS) & 0xff;
		LOG_DEV_DBG(dev, "grecc rfifo ists 1 %08x", rx_gsts);
	}
}

static int can_rzg3s_g_get_core_clock(const struct device *ctrl_dev)
{
	struct can_rzg3s_data *ctx = ctrl_dev->data;

	return ctx->clk_fcan_rate;
}

static int can_rzg3s_g_rfifo_register(const struct device *dev_ctrl, uint8_t rf_idx,
				      const struct device *ch_dev)
{
	struct can_rzg3s_data *ctx = dev_ctrl->data;

	ctx->rx_fifo_ch_devs[rf_idx] = ch_dev;
	return 0;
}

static int can_rzg3s_g_init_hw(const struct device *dev)
{
	const struct can_rzg3s_config *cfg = dev->config;
	uint32_t reg_val;
	int ret, i;

	/* Check RAMINIT flag as CAN RAM initialization takes place after the MCU reset */
	ret = can_rzg3s_wait_bits0(dev, R_CFDG_STS, R_CFDG_STS_GRAMINIT);
	if (ret) {
		return ret;
	}

	/* Transition to Global Reset mode */
	can_rzg3s_modify32(dev, R_CFDG_CTR, R_CFDG_CTR_GSLPR, 0);
	can_rzg3s_modify32(dev, R_CFDG_CTR, R_CFDG_CTR_GMDC_MASK, R_CFDG_CTR_GMDC_GRESET);

	/* Ensure Global reset mode */
	ret = can_rzg3s_wait_bits1(dev, R_CFDG_STS, R_CFDG_STS_GRSTSTS);
	if (ret) {
		return ret;
	}

	/* Reset Global error flags */
	can_rzg3s_write32(dev, 0, R_CFDG_ERFL);

	/* Global configuration settings */
	reg_val = R_CFDG_CFG_CMPOC;

	/* Set External Clock if selected */
	if (cfg->clk_ext_rate)
		reg_val |= R_CFDG_CFG_DCS;

	can_rzg3s_write32(dev, reg_val, R_CFDG_CFG);

	/* RES Bit Protocol Exception Disable
	 * The RPED bit configures the protocol exception event handling according to ISO 11898-1.
	 */
	can_rzg3s_modify32(dev, R_CFDG_FDCFG, 0, R_CFDG_FDCFG_RPED);

	/* set num afl entries */
	can_rzg3s_write32(dev,
			  (CAN_RZGS_FD_MAX_FILTER << 16) | CAN_RZGS_FD_MAX_FILTER,
			  R_CFDG_AFL_CFG);

	/*  Channel's generic Rx fifo cfg */
	can_rzg3s_write32(dev, R_CFD_RMNB_CFG, R_CFD_RMNB);
	for (i = 0; i < CAN_RZG3S_RX_FIFO_NUM; i++) {
		can_rzg3s_write32(dev, can_rzg3s_rx_fifo_cfg[i], R_CFD_RFCC(i));
	}

	/* enable global irqs */
	reg_val = R_CFDG_CTR_MEIE | R_CFDG_CTR_CFMPOFIE;
	can_rzg3s_modify32(dev, R_CFDG_CTR, R_CFDG_CTR_GSLPR, reg_val);

	/* Start Global operation mode */
	can_rzg3s_modify32(dev, R_CFDG_CTR, R_CFDG_CTR_GMDC_MASK, R_CFDG_CTR_GMDC_GOPM);

	/* Verify mode change */
	ret = can_rzg3s_wait_bits0(dev, R_CFDG_STS, R_CFDG_STS_GNOPM);
	if (ret) {
		LOG_DEV_ERR(dev, "global op mode failed");
		return ret;
	}

	return 0;
}

static int can_rzg3s_g_init(const struct device *dev)
{
	const struct can_rzg3s_config *cfg = dev->config;
	struct can_rzg3s_data *ctx = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "Failed to configure CAN pins");
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->clk_dev_ram) || !device_is_ready(cfg->clk_dev_pclk)) {
		LOG_DEV_ERR(dev, "clks dev not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->rstp_n.dev) || !device_is_ready(cfg->rstc_n.dev)) {
		LOG_DEV_ERR(dev, "resets dev not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clk_dev_pclk, (clock_control_subsys_t)&cfg->clk_mod_pclk);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to configure clk_pclk");
		return ret;
	}

	ret = clock_control_on(cfg->clk_dev_ram, (clock_control_subsys_t)&cfg->clk_mod_ram);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to configure clk_ram");
		return ret;
	}

	ret = clock_control_get_rate(cfg->clk_dev_pclk,
				     (clock_control_subsys_t)&cfg->clk_mod_pclk,
				     &ctx->clk_pclk_rate);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to get clk rate");
		return ret;
	}

	ctx->clk_fcan_rate = cfg->clk_ext_rate;
	if (!ctx->clk_fcan_rate) {
		/* DLL clock is pclk div 2 internaly */
		ctx->clk_fcan_rate = ctx->clk_pclk_rate / 2;
	}

	(void)reset_line_deassert_dt(&cfg->rstp_n);
	(void)reset_line_deassert_dt(&cfg->rstc_n);

#ifdef CONFIG_CAN_STATS
	stats_init_and_reg(&ctx->stats.s_hdr, sizeof(uint32_t),
			   (sizeof(ctx->stats) - sizeof(struct stats_hdr)) / sizeof(uint32_t),
			   STATS_NAME_INIT_PARMS(can_rzg3s_global), "can_rzg3s_global");
#endif /* CONFIG_CAN_STATS */
	ret = can_rzg3s_g_init_hw(dev);
	if (ret) {
		LOG_DEV_ERR(dev, "failed to init");
		return ret;
	}

	cfg->irq_config();

	LOG_DEV_INF(dev, "CANFD IP init done ver:%08x pclk:%uHz ext_clk:%uHz",
		    can_rzg3s_read32(dev, R_CFDG_IPV), ctx->clk_pclk_rate, cfg->clk_ext_rate);
	return 0;
}

#define CAN_RZG3S_CHN_DATA_NAME(node_id) _CONCAT(can_rzg3s_chn_data, DT_DEP_ORD(node_id))
#define CAN_RZG3S_CHN_CFG_NAME(node_id)  _CONCAT(can_rzg3s_chn_config, DT_DEP_ORD(node_id))
#define CAN_RZG3S_CHN_IRQC_NAME(node_id) _CONCAT(can_rzg3s_chn_irq_config, DT_DEP_ORD(node_id))

#define CAN_RZG3S_CH_IRQ_CONFIG_FUNC(p_node)                                                       \
	static void CAN_RZG3S_CHN_IRQC_NAME(p_node)(void)                                          \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQ_BY_NAME(p_node, ch_err, irq),                                   \
			    DT_IRQ_BY_NAME(p_node, ch_err, priority), can_rzg3s_isr_err,           \
			    DEVICE_DT_GET(p_node), 0);                                             \
		IRQ_CONNECT(DT_IRQ_BY_NAME(p_node, ch_trx, irq),                                   \
			    DT_IRQ_BY_NAME(p_node, ch_trx, priority), can_rzg3s_isr_trx,           \
			    DEVICE_DT_GET(p_node), 0);                                             \
	}

#ifdef CONFIG_CAN_STATS
#define CAN_RZG3S_CHN_STATS_NAME(p_node)                                                           \
	.stat_gr_name = "can_rzg3s_ch" STRINGIFY(DT_REG_ADDR(p_node)),
#else
#define CAN_RZG3S_CHN_STATS_NAME(p_node)
#endif /* CONFIG_CAN_STATS */

#define CAN_RZG3S_CHN_INIT(p_node)                                                                 \
	CAN_RZG3S_CH_IRQ_CONFIG_FUNC(p_node);                                                      \
	static const struct can_rzg3s_chn_config CAN_RZG3S_CHN_CFG_NAME(p_node) = {                \
		.ctrl_dev = DEVICE_DT_GET(DT_PARENT(p_node)),                                      \
		.bitrate = DT_PROP(p_node, bus_speed),                                             \
		.sample_point = DT_PROP(p_node, sample_point),                                     \
		.bitrate_data = DT_PROP(p_node, bus_speed_data),                                   \
		.sample_point_data = DT_PROP(p_node, sample_point_data),                           \
		.phy = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(p_node, phys)),                            \
		.max_bitrate = DT_CAN_TRANSCEIVER_MAX_BITRATE(p_node, CAN_RZG3S_MAXBITRATE),       \
		.tx_delay_comp_offset = DT_PROP_OR(p_node, tx_delay_comp_offset, 0),               \
		.tx_delay_comp_offset_only =                                                       \
			DT_PROP_OR(p_node, renesas_tx_delay_comp_offset_only, 0),                  \
		.id = DT_REG_ADDR(p_node),                                                         \
		.irq_config = CAN_RZG3S_CHN_IRQC_NAME(p_node),                                     \
		.irq_ch_err = DT_IRQ_BY_NAME(p_node, ch_err, irq),                                 \
		.irq_ch_trx = DT_IRQ_BY_NAME(p_node, ch_trx, irq),                                 \
		CAN_RZG3S_CHN_STATS_NAME(p_node)};                                                 \
	static struct can_rzg3s_chn_data CAN_RZG3S_CHN_DATA_NAME(p_node) = {};                     \
                                                                                                   \
	CAN_DEVICE_DT_DEFINE(p_node, can_rzg3s_chn_init, NULL, &CAN_RZG3S_CHN_DATA_NAME(p_node),   \
			     &CAN_RZG3S_CHN_CFG_NAME(p_node), POST_KERNEL,                         \
			     CONFIG_CAN_RZG3S_CHN_INIT_PRIORITY, &can_rzg3s_can_api);

#define CAN_RZG3S_IRQ_CONFIG_FUNC(inst)                                                            \
	static void can_rzg3s_irq_config##inst(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, g_err, irq),                                 \
			    DT_INST_IRQ_BY_NAME(inst, g_err, priority), can_rzg3s_g_err_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, g_recc, irq),                                \
			    DT_INST_IRQ_BY_NAME(inst, g_recc, priority), can_rzg3s_g_recc_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, g_err, irq));                                 \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, g_recc, irq));                                \
	}

#define CAN_RZG3S_DEVICE_INIT(inst)                                                                \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	CAN_RZG3S_IRQ_CONFIG_FUNC(inst)                                                            \
                                                                                                   \
	static const struct can_rzg3s_config can_rzg3s_cfg_##inst = {                              \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.clk_dev_pclk = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, canfd)),           \
		.clk_mod_pclk.module = DT_INST_CLOCKS_CELL_BY_NAME(inst, canfd, module),           \
		.clk_mod_pclk.domain = DT_INST_CLOCKS_CELL_BY_NAME(inst, canfd, domain),           \
		.clk_dev_ram = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, fck)),              \
		.clk_mod_ram.module = DT_INST_CLOCKS_CELL_BY_NAME(inst, fck, module),              \
		.clk_mod_ram.domain = DT_INST_CLOCKS_CELL_BY_NAME(inst, fck, domain),              \
		.clk_ext_rate =                                                                    \
			DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(inst, can_clk), clock_frequency),      \
		.rstp_n = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 0),                                  \
		.rstc_n = RESET_DT_SPEC_INST_GET_BY_IDX(inst, 1),                                  \
		.irq_config = can_rzg3s_irq_config##inst,                                          \
	};                                                                                         \
                                                                                                   \
	static struct can_rzg3s_data can_rzg3s_data_##inst;                                        \
	DEVICE_DT_INST_DEFINE(inst, can_rzg3s_g_init, NULL, &can_rzg3s_data_##inst,                \
			      &can_rzg3s_cfg_##inst, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, NULL); \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, CAN_RZG3S_CHN_INIT);

DT_INST_FOREACH_STATUS_OKAY(CAN_RZG3S_DEVICE_INIT)
