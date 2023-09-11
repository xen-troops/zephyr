/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rza2_bsc

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include "bsc_rza2_defines.h"

DEVICE_MMIO_TOPLEVEL_STATIC(bsc_reg, DT_DRV_INST(0));

#define BSC_REG_BASE DEVICE_MMIO_TOPLEVEL_GET(bsc_reg)

#define AREA_MAX 5

#define CMNCR_OFF      0x0
#define CSnBCR_OFF(n)  (0x4 + (0x4 * (n)))
#define CSnWCR_OFF(n)  (0x28 + (0x4 * (n)))
#define SDCR_OFF       0x4c
#define RTCSR_OFF      0x50
#define RTCNT_OFF      0x54
#define RTCOR_OFF      0x58
#define TOSCORn_OFF(n) (0x60 + (0x4 * (n)))
#define TOSTR_OFF      0x80
#define TOENR_OFF      0x84
#define ACADJ_OFF      0x90

#define SDRAM_MODE_AREA_2 0x1000
#define SDRAM_MODE_AREA_3 0x2000
#define SDRAM_MODE_CS2    SDRAM_MODE_AREA_2 + 0x60 /* CS2: CAS=3, burst write, 16bit bus */
#define SDRAM_MODE_CS3    SDRAM_MODE_AREA_3 + 0x60 /* CS3: CAS=3, burst write, 16bit bus */

#define SET_PAR(value, write_v, shift, mask) ((value) & (~(mask))) | ((write_v) << (shift))
#define GET_PAR(value, shift, mask)          (((value) & (mask)) >> (shift));

struct bsc_area_dt_config {
	uint8_t type;
	uint8_t area;
	uint8_t iwrrs;
	uint8_t iwrrd;
	uint8_t iwrws;
	uint8_t iwrwd;
	uint8_t iww;
	uint8_t auto_ref;
	uint8_t rtcor;
	uint8_t rtcnt;
	uint8_t cks;
	uint8_t rrc;
	uint8_t deep;
	uint8_t pwr_down;
	uint8_t bas;
	uint8_t bw;
	uint8_t a2row;
	uint8_t a2col;
	uint8_t a3col;
	uint8_t a3row;
	uint8_t bactv;
	uint8_t wr_wait_cycles;
	uint8_t ww_wait_cycles;
	uint8_t sw;
	uint8_t hw;
	uint8_t wtrcd;
	uint8_t a3cl;
	uint8_t wtrp;
	uint8_t trwl;
	uint8_t wtrc;
	uint8_t a2cl;
	uint8_t wm;
	uint8_t bsz;
	uint8_t szsel;
	uint8_t sdridly;
	uint8_t sdrodly;
	const struct pinctrl_dev_config *pcfg;
	uint16_t sdcs2;
	uint16_t sdcs3;
	uint32_t idle;
};

/* Sets WW to be equalt to WR in area */
#define WW_EQ_WR   0x7f
#define CONF_EMPTY 0x7f
#define MPX_SZSEL  BIT(0)

#define RTCSR_WP_OFF_MASK 0xa55a0000

#ifdef CONFIG_PINCTRL
#define BSC_PINCTRL_DT_DEFINE(n)                                                                   \
	COND_CODE_1(DT_NUM_PINCTRL_STATES(n), (PINCTRL_DT_DEFINE(n);), (EMPTY))

#define BSC_PINCTRL_DT_INST_DEV_CONFIG_GET(n)                                                      \
	COND_CODE_1(DT_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_DEV_CONFIG_GET(n)), NULL)
#else
#define BSC_PINCTRL_DT_DEFINE(n)
#define BSC_PINCTRL_DT_INST_DEV_CONFIG_GET(n) NULL
#endif

#define BSC_PROP(node_id, prop)                                                                    \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop), (DT_PROP(node_id, prop)), (CONF_EMPTY))

#define BSC_PROP_ZERO(node_id, prop)                                                               \
	DT_PROP_OR(node_id, prop, 0)

#define DT_BSC_ADDR(node_id)                                                                       \
	{                                                                                          \
		.type = BSC_PROP(node_id, type), .area = DT_REG_ADDR(node_id),                     \
		.pcfg = BSC_PINCTRL_DT_INST_DEV_CONFIG_GET(node_id),                               \
		.iwrrs = BSC_PROP(node_id, iwrrs), .iwrrd = BSC_PROP(node_id, iwrrd),              \
		.iwrws = BSC_PROP(node_id, iwrws), .iwrwd = BSC_PROP(node_id, iwrwd),              \
		.iww = BSC_PROP(node_id, iww), .auto_ref = BSC_PROP(node_id, auto_ref),            \
		.rtcor = BSC_PROP(node_id, rtcor), .rtcnt = BSC_PROP(node_id, rtcnt),              \
		.cks = BSC_PROP(node_id, cks), .rrc = BSC_PROP(node_id, rrc),                      \
		.deep = BSC_PROP(node_id, deep), .pwr_down = BSC_PROP(node_id, pwr_down),          \
		.bas = BSC_PROP(node_id, bas), .bw = BSC_PROP(node_id, bw),                        \
		.a2row = BSC_PROP(node_id, a2row), .a2col = BSC_PROP(node_id, a2col),              \
		.a3col = BSC_PROP(node_id, a3col), .a3row = BSC_PROP(node_id, a3row),              \
		.bactv = BSC_PROP(node_id, bactv),                                                 \
		.wr_wait_cycles = BSC_PROP(node_id, wr_wait_cycles),                               \
		.ww_wait_cycles = BSC_PROP(node_id, ww_wait_cycles), .sw = BSC_PROP(node_id, sw),  \
		.hw = BSC_PROP(node_id, hw), .wtrcd = BSC_PROP(node_id, wtrcd),                    \
		.a3cl = BSC_PROP(node_id, a3cl), .wtrp = BSC_PROP(node_id, wtrp),                  \
		.trwl = BSC_PROP(node_id, trwl), .wtrc = BSC_PROP(node_id, wtrc),                  \
		.a2cl = BSC_PROP(node_id, a2cl), .wm = BSC_PROP(node_id, wm),                      \
		.bsz = BSC_PROP(node_id, bsz), .szsel = BSC_PROP(node_id, szsel),                  \
		.sdridly = BSC_PROP(node_id, sdridly), .sdrodly = BSC_PROP(node_id, sdrodly),      \
		.idle = BSC_PROP_ZERO(node_id, idle), .sdcs2 = BSC_PROP_ZERO(node_id, sdcs2),      \
	}

DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_DRV_INST(0), BSC_PINCTRL_DT_DEFINE, (;))

static const struct bsc_area_dt_config bsc_dt_conf[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_DRV_INST(0), DT_BSC_ADDR, (,))
};

/* Set fields common to the all WCR regs*/
#define WM_BIT      BIT(6);
#define WR_VAL(val) FIELD_PREP(GENMASK(10, 7), val)

enum bsc_mem_type {
	MEM_NORMAL = 0b0,
	MEM_BURST_ASYNC = 0b1,
	MEM_MPX_IO = 0b10,
	MEM_SRAM_BS = 0b11,
	MEM_SDRAM = 0b100,
	MEM_BURST_SYNC = 0b111
};

enum bsc_bus_width {
	BUS_8BIT = 0b1,
	BUS_16BIT = 0b10,
	BUS_MPX = 0b11
};

enum bsc_mpx_wait_cycle {
	NO_WAIT_CYCLE = 0x0,
	ONE_WAIT_CYCLE = 0x1
};

struct bsc_area_config {
	uint8_t type;
	uint8_t wr_cycles;
	uint8_t ww_cycles;
	uint8_t sw;
	uint8_t hw;
	uint8_t bas;
};

static int get_masked_width(uint8_t area, uint8_t width, int type)
{
	if ((area == 2 || area == 3) && width != 16) {
		return -EINVAL;
	}

	if (area == 0 && type == MEM_BURST_SYNC && width != 16) {
		return -EINVAL;
	}

	if (area == 5 && type == MEM_MPX_IO) {
		return BUS_MPX;
	}

	switch (width) {
	case 16:
		return BUS_16BIT;
	case 8:
		return BUS_8BIT;
	default:
		return -EINVAL;
	}
}

static int set_bsz_config(uint8_t area, uint8_t width, int type, int flags)
{
	/*
	 * Initial state and pin settings depend from boot mode (sec 3)
	 * boot mode 0 -> state ared 0 is fixed bus width 16 bits
	 *	       areas 1-5 same as 0 but bus width can be changed by program.
	 * modes 1..7 = areas 0..5 can be changed by the program
	 * pinctrl should be set. do not access ext addr space before setting pins.
	 */
	uint32_t csnbcr;
	uint32_t cs5wcr0;
	int ret = 0;

	if (area > AREA_MAX) {
		return -EINVAL;
	}

	csnbcr = sys_read32(BSC_REG_BASE + CSnBCR_OFF(area));
	csnbcr |= SET_PAR(csnbcr, type, BSC_CS0BCR_TYPE_SHIFT, BSC_CS0BCR_TYPE);
	if (width != CONF_EMPTY || (area == 5 && type == MEM_MPX_IO)) {
		ret = get_masked_width(area, width, type);
		if (ret < 0) {
			return ret;
		}

		csnbcr |= SET_PAR(csnbcr, ret, BSC_CS0BCR_BSZ_SHIFT, BSC_CS0BCR_BSZ);
		if (area == 5 && ret == BUS_MPX) {
			/* set SZSEL based on flags for MPX mode*/
			cs5wcr0 = sys_read32(BSC_REG_BASE + CSnWCR_OFF(area));
			cs5wcr0 |= SET_PAR(cs5wcr0, flags & MPX_SZSEL, BSC_CS5WCR_0_SZSEL_SHIFT,
					   BSC_CS5WCR_0_SZSEL);
			sys_write32(cs5wcr0, BSC_REG_BASE + CSnWCR_OFF(area));
		}
	}

	sys_write32(csnbcr, BSC_REG_BASE + CSnBCR_OFF(area));

	return 0;
}

static int set_wm_bit(uint8_t area, uint8_t wm)
{
	/*
	 * If WM bit in CSnWCR is cleared to 0 - Tnop is inserted after CSn space access for
	 * external wait.
	 * If WM bit in CSnWCR is set to 1, external waits are ignored and no Tnop cycle
	 * is inserted.
	 */
	uint32_t value;

	if (wm == CONF_EMPTY) {
		return 0;
	}

	if (area > AREA_MAX) {
		return -EINVAL;
	}

	value = sys_read32(BSC_REG_BASE + CSnWCR_OFF(area));
	/* WM bit has common place for all WCRn registers, so just set corresponding bit */
	if (wm) {
		value |= WM_BIT;
	} else {
		value &= ~WM_BIT;
	}

	sys_write32(value, BSC_REG_BASE + CSnWCR_OFF(area));

	return 0;
}

static int config_wr_wait_cycles(uint8_t area, uint8_t wait_cycles)
{
	switch (wait_cycles) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case CONF_EMPTY:
		return wait_cycles;
	case 8:
		return 0b111;
	case 10:
		return 0b1000;
	case 12:
		return 0b1001;
	case 14:
		return 0b1010;
	case 18:
		return 0b1011;
	case 24:
		return 0b1100;
	default:
		return -EINVAL;
	}
}

static int config_ww_wait_cycles(uint8_t area, uint8_t wait_cycles)
{
	if (wait_cycles == WW_EQ_WR || wait_cycles == CONF_EMPTY) {
		return 0;
	}

	if (wait_cycles > 6) {
		return -EINVAL;
	}

	return wait_cycles + 1;
}

static int set_area(uint8_t area, struct bsc_area_config *bsc_conf, uint32_t value, uint32_t *res)
{
	if (!bsc_conf) {
		return -EINVAL;
	}

	switch (area) {
	case 0:
		/* Fill up BSC_CS0WCR_0 */
		uint32_t cs0 = value;

		if (bsc_conf->type != MEM_NORMAL && bsc_conf->type != MEM_SRAM_BS &&
		    bsc_conf->type != MEM_MPX_IO && bsc_conf->type != MEM_BURST_SYNC) {
			return -EINVAL;
		}

		/* Only read cycles supported for CS0 */
		if (bsc_conf->ww_cycles != 0) {
			return -EINVAL;
		}

		if (bsc_conf->wr_cycles != CONF_EMPTY) {
			cs0 |= SET_PAR(cs0, bsc_conf->wr_cycles, BSC_CS0WCR_0_WR_SHIFT,
				       BSC_CS0WCR_0_WR);
		}

		if (bsc_conf->sw != CONF_EMPTY) {
			cs0 |= SET_PAR(cs0, bsc_conf->sw, BSC_CS0WCR_0_SW_SHIFT, BSC_CS0WCR_0_SW);
		}

		if (bsc_conf->hw != CONF_EMPTY) {
			cs0 |= SET_PAR(cs0, bsc_conf->hw, BSC_CS0WCR_0_HW_SHIFT, BSC_CS0WCR_0_HW);
		}

		if (bsc_conf->type == MEM_SRAM_BS && bsc_conf->bas != CONF_EMPTY) {
			cs0 |= SET_PAR(cs0, bsc_conf->bas, BSC_CS0WCR_0_BAS_SHIFT,
				       BSC_CS0WCR_0_BAS);
		}

		*res = cs0;
		return 0;
	case 1:
		/* CS1WCR_0 */
		uint32_t cs1 = value;

		if (bsc_conf->wr_cycles != CONF_EMPTY) {
			cs1 |= SET_PAR(cs1, bsc_conf->wr_cycles, BSC_CS1WCR_0_WR_SHIFT,
				       BSC_CS1WCR_0_WR);
			cs1 |= SET_PAR(cs1, bsc_conf->ww_cycles, BSC_CS1WCR_0_WW_SHIFT,
				       BSC_CS1WCR_0_WW);
		}

		if (bsc_conf->type == MEM_NORMAL || bsc_conf->type == MEM_SRAM_BS ||
		    bsc_conf->type == MEM_MPX_IO) {
			if (bsc_conf->sw != CONF_EMPTY) {
				cs1 |= SET_PAR(cs1, bsc_conf->sw, BSC_CS1WCR_0_SW_SHIFT,
					       BSC_CS1WCR_0_SW);
			}

			if (bsc_conf->hw != CONF_EMPTY) {
				cs1 |= SET_PAR(cs1, bsc_conf->hw, BSC_CS1WCR_0_HW_SHIFT,
					       BSC_CS1WCR_0_HW);
			}
		}

		if (bsc_conf->type == MEM_SRAM_BS && bsc_conf->bas != CONF_EMPTY) {
			cs1 |= SET_PAR(cs1, bsc_conf->bas, BSC_CS1WCR_0_BAS_SHIFT,
				       BSC_CS1WCR_0_BAS);
		}

		*res = cs1;
		return 0;
	case 2:
		/* CS2WCR_0 */
		uint32_t cs2 = value;

		if (bsc_conf->type != MEM_NORMAL && bsc_conf->type != MEM_SRAM_BS &&
		    bsc_conf->type != MEM_MPX_IO) {
			return -EINVAL;
		}

		if (bsc_conf->sw != CONF_EMPTY || bsc_conf->hw != CONF_EMPTY) {
			return -EINVAL;
		}

		/* Only read cycles supported */
		if (bsc_conf->ww_cycles != 0) {
			return -EINVAL;
		}

		if (bsc_conf->wr_cycles != CONF_EMPTY) {
			cs2 |= SET_PAR(cs2, bsc_conf->wr_cycles, BSC_CS2WCR_0_WR_SHIFT,
				       BSC_CS2WCR_0_WR);
		}

		if (bsc_conf->type == MEM_SRAM_BS && bsc_conf->bas != CONF_EMPTY) {
			cs2 |= SET_PAR(cs2, bsc_conf->bas, BSC_CS2WCR_0_BAS_SHIFT,
				       BSC_CS2WCR_0_BAS);
		}

		*res = cs2;
		return 0;
	case 3:
		/* CS3WCR_0*/
		uint32_t cs3 = value;

		if (bsc_conf->type != MEM_NORMAL && bsc_conf->type != MEM_SRAM_BS &&
		    bsc_conf->type != MEM_MPX_IO) {
			return -EINVAL;
		}

		if (bsc_conf->sw != CONF_EMPTY || bsc_conf->hw != CONF_EMPTY) {
			return -EINVAL;
		}

		/* Only read cycles supported */
		if (bsc_conf->ww_cycles != 0) {
			return -EINVAL;
		}

		if (bsc_conf->wr_cycles != CONF_EMPTY) {
			cs3 |= SET_PAR(cs3, bsc_conf->wr_cycles, BSC_CS3WCR_0_WR_SHIFT,
				       BSC_CS3WCR_0_WR);
		}

		if (bsc_conf->type == MEM_SRAM_BS && bsc_conf->bas != CONF_EMPTY) {
			cs3 |= SET_PAR(cs3, bsc_conf->bas, BSC_CS3WCR_0_BAS_SHIFT,
				       BSC_CS3WCR_0_BAS);
		}

		*res = cs3;
		return 0;
	case 4:
		uint32_t cs4 = value;

		if (bsc_conf->type == MEM_NORMAL || bsc_conf->type == MEM_SRAM_BS ||
		    bsc_conf->type == MEM_MPX_IO) {
			/* CS4WCR_0 */
			if (bsc_conf->wr_cycles != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->wr_cycles, BSC_CS4WCR_0_WR_SHIFT,
					       BSC_CS4WCR_0_WR);
				cs4 |= SET_PAR(cs4, bsc_conf->ww_cycles, BSC_CS4WCR_0_WW_SHIFT,
					       BSC_CS4WCR_0_WW);
			}
			if (bsc_conf->sw != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->sw, BSC_CS4WCR_0_SW_SHIFT,
					       BSC_CS4WCR_0_SW);
			}

			if (bsc_conf->hw != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->hw, BSC_CS4WCR_0_HW_SHIFT,
					       BSC_CS4WCR_0_HW);
			}

			if (bsc_conf->type == MEM_SRAM_BS && bsc_conf->bas != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->bas, BSC_CS4WCR_0_BAS_SHIFT,
					       BSC_CS4WCR_0_BAS);
			}

			*res = cs4;
			return 0;

		} else if (bsc_conf->type == MEM_BURST_ASYNC) {
			/* CS4WCR_1 */
			/* Only read cycles supported */
			if (bsc_conf->ww_cycles != 0) {
				return -EINVAL;
			}
			/* Set access wait cycles as wr */
			if (bsc_conf->wr_cycles != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->wr_cycles, BSC_CS4WCR_1_W_SHIFT,
					       BSC_CS4WCR_1_W);
			}
			if (bsc_conf->sw != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->sw, BSC_CS4WCR_1_SW_SHIFT,
					       BSC_CS4WCR_1_SW);
			}

			if (bsc_conf->hw != CONF_EMPTY) {
				cs4 |= SET_PAR(cs4, bsc_conf->hw, BSC_CS4WCR_1_HW_SHIFT,
					       BSC_CS4WCR_1_HW);
			}
			*res = cs4;
			return 0;
		}

		return -EINVAL;
	case 5:
		/* CS5WCR_0 */
		uint32_t cs5 = value;

		if (bsc_conf->type != MEM_NORMAL && bsc_conf->type != MEM_SRAM_BS &&
		    bsc_conf->type != MEM_MPX_IO) {
			return -EINVAL;
		}

		if (bsc_conf->bas != CONF_EMPTY) {
			return -EINVAL;
		}

		if (bsc_conf->wr_cycles != CONF_EMPTY) {
			cs5 |= SET_PAR(cs5, bsc_conf->wr_cycles, BSC_CS5WCR_0_WR_SHIFT,
				       BSC_CS5WCR_0_WR);
			cs5 |= SET_PAR(cs5, bsc_conf->ww_cycles, BSC_CS5WCR_0_WW_SHIFT,
				       BSC_CS5WCR_0_WW);
		}
		if (bsc_conf->sw != CONF_EMPTY) {
			cs5 |= SET_PAR(cs5, bsc_conf->sw, BSC_CS5WCR_0_SW_SHIFT, BSC_CS5WCR_0_SW);
		}

		if (bsc_conf->hw != CONF_EMPTY) {
			cs5 |= SET_PAR(cs5, bsc_conf->hw, BSC_CS5WCR_0_HW_SHIFT, BSC_CS5WCR_0_HW);
		}
		*res = cs5;
		return 0;
	default:
		return -EINVAL;
	}
}

static int set_area_config(uint8_t area, uint8_t type, uint8_t wr_wait_cycles,
			   uint8_t ww_wait_cycles, uint8_t sw, uint8_t hw, uint8_t bas)
{
	int ret;
	uint32_t value, res_value;
	struct bsc_area_config bsc_conf = {0};

	if (area > AREA_MAX) {
		return -EINVAL;
	}

	ret = config_wr_wait_cycles(area, wr_wait_cycles);
	if (ret < 0) {
		return ret;
	}
	bsc_conf.wr_cycles = ret;

	ret = config_ww_wait_cycles(area, ww_wait_cycles);
	if (ret < 0) {
		return ret;
	}
	bsc_conf.ww_cycles = ret;

	bsc_conf.sw = sw;
	bsc_conf.hw = hw;
	bsc_conf.bas = bas;

	value = sys_read32(BSC_REG_BASE + CSnWCR_OFF(area));

	ret = set_area(area, &bsc_conf, value, &res_value);
	if (ret) {
		return ret;
	}

	sys_write32(res_value, BSC_REG_BASE + CSnWCR_OFF(area));
	return 0;
}

static int set_mpxw(uint8_t area, uint8_t type, uint8_t value)
{
	/* CS5WCR_0 */
	uint32_t cs5;
	/* This configuration is supported only for MPX_IO on CS5*/
	if (type != MEM_MPX_IO || area != 5) {
		return -EINVAL;
	}

	if (value == CONF_EMPTY) {
		return 0;
	}

	cs5 = sys_read32(BSC_REG_BASE + CSnWCR_OFF(area));
	if (value == ONE_WAIT_CYCLE) {
		cs5 |= SET_PAR(cs5, 1, BSC_CS5WCR_0_MPXWBAS_SHIFT, BSC_CS5WCR_0_MPXWBAS);
	}

	sys_write32(cs5, BSC_REG_BASE + CSnWCR_OFF(area));
	return 0;
}

/* Set address multiplexing */
static int set_sdcr_config(uint8_t area, uint8_t a2row, uint8_t a2col, uint8_t a3col, uint8_t a3row,
			   uint8_t bactv)
{
	/* SDCR */
	uint32_t sd;

	if (area != 3 && bactv != CONF_EMPTY) {
		return -EINVAL;
	}

	sd = sys_read32(BSC_REG_BASE + SDCR_OFF);
	if (a2col != CONF_EMPTY) {
		sd |= SET_PAR(sd, a2col, BSC_SDCR_A2COL_SHIFT, BSC_SDCR_A2COL);
	}

	if (a2row != CONF_EMPTY) {
		sd |= SET_PAR(sd, a2row, BSC_SDCR_A2ROW_SHIFT, BSC_SDCR_A2ROW);
	}

	if (a3col != CONF_EMPTY) {
		sd |= SET_PAR(sd, a3col, BSC_SDCR_A3COL_SHIFT, BSC_SDCR_A3COL);
	}

	if (a3row != CONF_EMPTY) {
		sd |= SET_PAR(sd, a3row, BSC_SDCR_A3ROW_SHIFT, BSC_SDCR_A3ROW);
	}

	if (bactv != CONF_EMPTY) {
		sd |= SET_PAR(sd, bactv, BSC_SDCR_BACTV_SHIFT, BSC_SDCR_BACTV);
	}

	sys_write32(sd, BSC_REG_BASE + SDCR_OFF);

	return 0;
}

static int wtrc_conv(uint8_t value)
{
	switch (value) {
	case 2:
		return 0;
	case 3:
		return 0b01;
	case 5:
		return 0b10;
	case 8:
		return 0b11;
	default:
		return -EINVAL;
	}
}

static int wtr_conv(uint8_t value)
{
	if (value > 3) {
		return -EINVAL;
	}

	return value;
}

static int acl_conv(uint8_t value)
{
	switch (value) {
	case 1:
		return 0;
	case 2:
		return 0b1;
	case 3:
		return 0b10;
	case 4:
		return 0b11;
	default:
		return -EINVAL;
	}
}

static int set_cs3_mux_config(uint8_t type, uint8_t wtrcd, uint8_t a3cl, uint8_t wtrp, uint8_t trwl,
			      uint8_t wtrc)
{
	/* CS3WCR_1 */
	uint32_t cs3;
	int ret;

	if (type != MEM_SDRAM) {
		return -EINVAL;
	}

	cs3 = sys_read32(BSC_REG_BASE + CSnWCR_OFF(3));
	if (wtrcd != CONF_EMPTY) {
		ret = wtr_conv(wtrcd);
		if (ret < 0) {
			return ret;
		}
		cs3 |= SET_PAR(cs3, ret, BSC_CS3WCR_1_WTRCD_SHIFT, BSC_CS3WCR_1_WTRCD);
	}

	if (wtrp != CONF_EMPTY) {
		ret = wtr_conv(wtrp);
		if (ret < 0) {
			return ret;
		}
		cs3 |= SET_PAR(cs3, ret, BSC_CS3WCR_1_WTRP_SHIFT, BSC_CS3WCR_1_WTRP);
	}

	if (wtrc != CONF_EMPTY) {
		ret = wtrc_conv(wtrc);
		if (ret < 0) {
			return ret;
		}
		cs3 |= SET_PAR(cs3, ret, BSC_CS3WCR_1_WTRC_SHIFT, BSC_CS3WCR_1_WTRC);
	}

	if (trwl != CONF_EMPTY) {
		ret = wtr_conv(trwl);
		if (ret < 0) {
			return ret;
		}
		cs3 |= SET_PAR(cs3, ret, BSC_CS3WCR_1_TRWL_SHIFT, BSC_CS3WCR_1_TRWL);
	}

	if (a3cl != CONF_EMPTY) {
		ret = acl_conv(a3cl);
		if (ret < 0) {
			return ret;
		}
		cs3 |= SET_PAR(cs3, ret, BSC_CS3WCR_1_A3CL_SHIFT, BSC_CS3WCR_1_A3CL);
	}
	sys_write32(cs3, BSC_REG_BASE + CSnWCR_OFF(3));

	return 0;
}

static int set_cs2_mux_config(uint8_t type, uint8_t a2cl)
{
	/* CS2WCR_1 */
	uint32_t cw2;
	int ret;

	if (type != MEM_SDRAM) {
		return -EINVAL;
	}

	if (a2cl == CONF_EMPTY) {
		return 0;
	}

	cw2 = sys_read32(BSC_REG_BASE + CSnWCR_OFF(2));

	ret = acl_conv(a2cl);
	if (ret < 0) {
		return ret;
	}
	cw2 |= SET_PAR(cw2, ret, BSC_CS2WCR_1_A2CL_SHIFT, BSC_CS2WCR_1_A2CL);
	sys_write32(cw2, BSC_REG_BASE + CSnWCR_OFF(2));

	return 0;
}

static int set_sdcr_refresh(uint8_t rmode, uint8_t rfsh)
{
	/* SDCR */
	uint32_t sd;

	sd = sys_read32(BSC_REG_BASE + SDCR_OFF);
	sd |= SET_PAR(sd, rfsh, BSC_SDCR_RFSH_SHIFT, BSC_SDCR_RFSH);
	sd |= SET_PAR(sd, rmode, BSC_SDCR_RMODE_SHIFT, BSC_SDCR_RMODE);

	sys_write32(sd, BSC_REG_BASE + SDCR_OFF);

	return 0;
}

static int get_rrc(uint8_t rrc)
{
	switch (rrc) {
	case 1:
		return 0;
	case 2:
		return 0b01;
	case 4:
		return 0b10;
	case 6:
		return 0b11;
	case 8:
		return 0b100;
	default:
		return -EINVAL;
	}
}

static int set_rcstr(uint8_t cks, uint8_t rrc)
{
	/* RTCSR */
	uint32_t rt;
	int ret;

	if (cks == CONF_EMPTY && rrc == CONF_EMPTY) {
		return 0;
	}

	if (cks == CONF_EMPTY || rrc == CONF_EMPTY) {
		return -EINVAL;
	}

	rt = sys_read32(BSC_REG_BASE + RTCSR_OFF);

	rt |= SET_PAR(rt, cks, BSC_RTCSR_CKS_SHIFT, BSC_RTCSR_CKS);

	ret = get_rrc(rrc);
	if (ret < 0) {
		return -EINVAL;
	}
	rt |= SET_PAR(rt, ret, BSC_RTCSR_RRC_SHIFT, BSC_RTCSR_RRC);

	/* Mask should be added to cancel write protection */
	sys_write32(rt | RTCSR_WP_OFF_MASK, BSC_REG_BASE + RTCSR_OFF);
	return 0;
}

/*
 * set_refresh
 * autorefresh: RMODE = 0 and RFSH = 1 bits should be set in SDCR.
 * no autorefresh: RMODE = 1 and RFSH = 1 bits in SDCR.
 * Refreshing is performed at intervals determined by the input clock selected by bits
 * CKS2 to CKS0 in RTCSR, and the value set by in RTCOR. The value of bits CKS2 to CKS0
 * in RTCOR should be set so as to satisfy the refresh interval stipulation for the
 * SDRAM used.
 * First make the settings for RTCOR, RTCNT, and the RMODE and RFSH bits in SDCR, and
 * then make the CKS2 to CKS0 and RRC2 to RRC0 settings.
 */
static int set_refresh(uint8_t area, uint8_t type, uint8_t auto_ref, uint8_t rtcor, uint8_t rtcnt,
		       uint8_t cks, uint8_t rrc)
{

	if (auto_ref == CONF_EMPTY) {
		return 0;
	}

	if (type != MEM_SDRAM) {
		return -EINVAL;
	}

	if (auto_ref == 1) {
		set_sdcr_refresh(0, 1);
		if (rtcor != CONF_EMPTY) {
			sys_write32(rtcor | RTCSR_WP_OFF_MASK, BSC_REG_BASE + RTCOR_OFF);
		}

		if (rtcnt != CONF_EMPTY) {
			sys_write32(rtcnt | RTCSR_WP_OFF_MASK, BSC_REG_BASE + RTCNT_OFF);
		}

		set_rcstr(cks, rrc);
	} else {
		/* CMNCR */
		uint32_t cm;

		set_sdcr_refresh(1, 1);

		if (rtcor != CONF_EMPTY) {
			sys_write32(rtcor | RTCSR_WP_OFF_MASK, BSC_REG_BASE + RTCOR_OFF);
		}

		if (rtcnt != CONF_EMPTY) {
			sys_write32(rtcnt | RTCSR_WP_OFF_MASK, BSC_REG_BASE + RTCNT_OFF);
		}
		/*
		 * Note that the necessary signals such as CKE must be driven even in
		 * standby state by setting the HIZCNT bit in CMNCR to 1.
		 */
		cm = sys_read32(BSC_REG_BASE + CMNCR_OFF);

		cm |= SET_PAR(cm, 1, BSC_CMNCR_HIZCNT_SHIFT, BSC_CMNCR_HIZCNT);
		sys_write32(cm, BSC_REG_BASE + CMNCR_OFF);
	}

	return 0;
}

static int set_pwdown(uint8_t type, uint8_t pwr_down)
{
	/* SDCR */
	uint32_t sd;

	if (pwr_down == CONF_EMPTY) {
		return 0;
	}

	if (type != MEM_SDRAM) {
		return -EINVAL;
	}

	sd = sys_read32(BSC_REG_BASE + SDCR_OFF);
	sd |= SET_PAR(sd, pwr_down, BSC_SDCR_PDOWN_SHIFT, BSC_SDCR_PDOWN);
	sys_write32(sd, BSC_REG_BASE + SDCR_OFF);
	return 0;
}

static int set_deep(uint8_t type, uint8_t deep)
{
	/* SDCR */
	uint32_t sd;

	if (deep == CONF_EMPTY) {
		return 0;
	}

	if (type != MEM_SDRAM) {
		return -EINVAL;
	}

	sd = sys_read32(BSC_REG_BASE + SDCR_OFF);
	sd |= SET_PAR(sd, deep, BSC_SDCR_DEEP_SHIFT, BSC_SDCR_DEEP);
	sys_write32(sd, BSC_REG_BASE + SDCR_OFF);
	return 0;
}

static int bw_conv(uint8_t value)
{
	if (value > 3) {
		return -EINVAL;
	}

	return value;
}

static int set_cs0_bw(uint8_t type, uint8_t bw)
{
	int ret;
	/* CS0WCR_1 */
	uint32_t cs0;

	/* BW can be set for both burst sync and burst async modes */
	if (type != MEM_BURST_ASYNC && type != MEM_BURST_SYNC) {
		return -EINVAL;
	}

	if (bw == CONF_EMPTY) {
		return 0;
	}

	ret = bw_conv(bw);
	if (ret < 0) {
		return ret;
	}

	cs0 = sys_read32(BSC_REG_BASE + CSnWCR_OFF(0));

	cs0 |= SET_PAR(cs0, ret, BSC_CS0WCR_1_BW_SHIFT, BSC_CS0WCR_1_BW);

	sys_write32(cs0, BSC_REG_BASE + CSnWCR_OFF(0));
	return 0;
}

static int set_cs4_bw(uint8_t type, uint8_t bw)
{
	int ret;
	/* CS4WCR_1 */
	uint32_t cs4;

	if (type != MEM_BURST_ASYNC) {
		return -EINVAL;
	}

	if (bw == CONF_EMPTY) {
		return 0;
	}

	ret = bw_conv(bw);
	if (ret < 0) {
		return ret;
	}

	cs4 = sys_read32(BSC_REG_BASE + CSnWCR_OFF(4));

	cs4 |= SET_PAR(cs4, ret, BSC_CS4WCR_1_BW_SHIFT, BSC_CS4WCR_1_BW);

	sys_write32(cs4, BSC_REG_BASE + CSnWCR_OFF(4));
	return 0;
}

static int set_byte_selection(uint8_t area, uint8_t iwrrs, uint8_t iwrrd, uint8_t iwrws,
			      uint8_t iwrwd, uint8_t iww)
{
	/* CS0BCR */
	uint32_t csnbcr;

	if (area > AREA_MAX) {
		return -EINVAL;
	}

	csnbcr = sys_read32(BSC_REG_BASE + CSnBCR_OFF(area));
	if (iwrrs != CONF_EMPTY) {
		csnbcr |= SET_PAR(csnbcr, iwrrs, BSC_CS0BCR_IWRRS_SHIFT, BSC_CS0BCR_IWRRS);
	}

	if (iwrrd != CONF_EMPTY) {
		csnbcr |= SET_PAR(csnbcr, iwrrd, BSC_CS0BCR_IWRRD_SHIFT, BSC_CS0BCR_IWRRD);
	}

	if (iwrws != CONF_EMPTY) {
		csnbcr |= SET_PAR(csnbcr, iwrws, BSC_CS0BCR_IWRWS_SHIFT, BSC_CS0BCR_IWRWS);
	}

	if (iwrwd != CONF_EMPTY) {
		csnbcr |= SET_PAR(csnbcr, iwrwd, BSC_CS0BCR_IWRWD_SHIFT, BSC_CS0BCR_IWRWD);
	}

	if (iww != CONF_EMPTY) {
		csnbcr |= SET_PAR(csnbcr, iww, BSC_CS0BCR_IWW_SHIFT, BSC_CS0BCR_IWW);
	}

	sys_write32(csnbcr, BSC_REG_BASE + CSnBCR_OFF(area));
	return 0;
}

static void set_acadj(uint8_t sdridly, uint8_t sdrodly)
{
	/* ACADJ*/
	uint32_t a;

	a = sys_read32(BSC_REG_BASE + ACADJ_OFF);

	if (sdridly && (sdridly != CONF_EMPTY)) {
		a |= SET_PAR(a, sdridly, BSC_ACADJ_SDRIDLY_SHIFT, BSC_ACADJ_SDRIDLY);
	}

	if (sdrodly != CONF_EMPTY) {
		a |= SET_PAR(a, sdrodly, BSC_ACADJ_SDRODLY_SHIFT, BSC_ACADJ_SDRODLY);
	}

	sys_write32(a, BSC_REG_BASE + ACADJ_OFF);
}

static int set_bsc_config(const struct bsc_area_dt_config *dt_conf)
{
	int ret;
	int i;
#ifdef CONFIG_PINCTRL
	if (dt_conf->pcfg != NULL &&
	    pinctrl_apply_state(dt_conf->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
		return -EINVAL;
	}
#endif
	ret = set_bsz_config(dt_conf->area, dt_conf->bsz, dt_conf->type, dt_conf->szsel);
	if (ret < 0) {
		return ret;
	}

	ret = set_byte_selection(dt_conf->area, dt_conf->iwrrs, dt_conf->iwrrd, dt_conf->iwrws,
				 dt_conf->iwrwd, dt_conf->iww);
	if (ret < 0) {
		return ret;
	}

	ret = set_area_config(dt_conf->area, dt_conf->type, dt_conf->wr_wait_cycles,
			      dt_conf->ww_wait_cycles, dt_conf->sw, dt_conf->hw, dt_conf->bas);
	if (ret < 0) {
		return ret;
	}

	ret = set_wm_bit(dt_conf->area, dt_conf->wm);
	if (ret < 0) {
		return ret;
	}

	if (dt_conf->type == MEM_MPX_IO) {
		ret = set_mpxw(dt_conf->area, dt_conf->type, dt_conf->bas);
		if (ret < 0) {
			return ret;
		}
	}

	if (dt_conf->type == MEM_SDRAM && dt_conf->area == 2) {
		ret = set_cs2_mux_config(dt_conf->type, dt_conf->a2cl);
		if (ret < 0) {
			return ret;
		}
	}

	if (dt_conf->type == MEM_SDRAM && dt_conf->area == 3) {
		ret = set_cs3_mux_config(dt_conf->type, dt_conf->wtrcd, dt_conf->a3cl,
					 dt_conf->wtrp, dt_conf->trwl, dt_conf->wtrc);
		if (ret < 0) {
			return ret;
		}
	}

	if ((dt_conf->type == MEM_BURST_ASYNC || dt_conf->type == MEM_BURST_ASYNC) &&
	    dt_conf->area == 0) {
		ret = set_cs0_bw(dt_conf->type, dt_conf->bw);
		if (ret < 0) {
			return ret;
		}
	}

	if (dt_conf->type == MEM_BURST_ASYNC && dt_conf->area == 4) {
		ret = set_cs4_bw(dt_conf->type, dt_conf->bw);
		if (ret < 0) {
			return ret;
		}
	}

	ret = set_sdcr_config(dt_conf->area, dt_conf->a2row, dt_conf->a2col, dt_conf->a3col,
			      dt_conf->a3row, dt_conf->bactv);
	if (ret < 0) {
		return ret;
	}

	if (dt_conf->type == MEM_SDRAM && dt_conf->deep != CONF_EMPTY) {
		ret = set_deep(dt_conf->type, dt_conf->deep);
		if (ret < 0) {
			return ret;
		}
	}

	if (dt_conf->type == MEM_SDRAM && dt_conf->pwr_down != CONF_EMPTY) {
		ret = set_pwdown(dt_conf->type, dt_conf->pwr_down);
		if (ret < 0) {
			return ret;
		}
	}

	if (dt_conf->type == MEM_SDRAM) {
		ret = set_refresh(dt_conf->area, dt_conf->type, dt_conf->auto_ref, dt_conf->rtcor,
				  dt_conf->rtcnt, dt_conf->cks, dt_conf->rrc);
		if (ret < 0) {
			return ret;
		}
	}

	if (dt_conf->type == MEM_SDRAM) {
		/* wait */
		for (i = 0; i < dt_conf->idle; i++) {
			__asm("nop");
		}

		sys_write16(dt_conf->sdcs2, BSC_REG_BASE + SDRAM_MODE_CS2);
		sys_write16(dt_conf->sdcs3, BSC_REG_BASE + SDRAM_MODE_CS3);
	}

	set_acadj(dt_conf->sdridly, dt_conf->sdrodly);

	return 0;
}

__boot_func static int bsc_rza2_driver_init(void)
{
	int ret;
	int i;

	DEVICE_MMIO_TOPLEVEL_MAP(bsc_reg, K_MEM_CACHE_NONE);

	for (i = 0; i < ARRAY_SIZE(bsc_dt_conf); i++) {
		ret = set_bsc_config(&bsc_dt_conf[i]);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

SYS_INIT(bsc_rza2_driver_init, PRE_KERNEL_1, CONFIG_BSC_RZA2_INIT_PRIORITY);
