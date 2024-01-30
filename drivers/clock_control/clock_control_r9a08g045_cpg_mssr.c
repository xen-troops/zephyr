/*
 * Copyright (c) 2024 EPAM Systems
 *
 * Renesas RZ/G3S r9a08g045 Clock Pulse Generator, Module Standby controller
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r9a08g045_cpg_mssr

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/clock/r9a08g045_cpg_mssr.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_rzg3s);

#define LOG_DEV_ERR(dev, format, ...)	LOG_ERR("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...)	LOG_WRN("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...)	LOG_INF("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...)	LOG_DBG("%s:"#format, (dev)->name, ##__VA_ARGS__)

#define MON_REG_WAIT_US 50
#define CPG_NUM_DOMAINS 2

/* Specific MSTOP register offsets of RZ/G3S */
#define ACPU_MSTOP		(0xB60)
#define MCPU1_MSTOP		(0xB64)
#define MCPU2_MSTOP		(0xB68)
#define PERI_COM_MSTOP		(0xB6C)
#define PERI_CPU_MSTOP		(0xB70)
#define PERI_DDR_MSTOP		(0xB74)
#define PERI_VIDEO_MSTOP	(0xB78)
#define REG0_MSTOP		(0xB7C)
#define REG1_MSTOP		(0xB80)
#define TZCDDR_MSTOP		(0xB84)
#define MHU_MSTOP		(0xB88)
#define PERI_STP_MSTOP		(0xB8C)
#define MCPU3_MSTOP		(0xB90)
#define PERI_CPU2_MSTOP		(0xB94)

#define CLK_MON_R(reg)		(0x180 + (reg))

enum rzg3s_clk_ids {
	/* Core Clock Outputs exported to DT */
	LAST_DT_CORE_CLK = R9A08G045_OSCCLK2,

	/* External Input Clocks */
	CLK_EXTAL,

	/* Internal Core Clocks */
	CLK_OSC_DIV1000,
	CLK_PLL1,
	CLK_PLL2,
	CLK_PLL2_DIV2,
	CLK_PLL2_DIV2_8,
	CLK_PLL2_DIV6,
	CLK_PLL3,
	CLK_PLL3_DIV2,
	CLK_PLL3_DIV2_2,
	CLK_PLL3_DIV2_4,
	CLK_PLL3_DIV2_8,
	CLK_PLL3_DIV6,
	CLK_PLL4,
	CLK_PLL6,
	CLK_PLL6_DIV2,
	CLK_SEL_SPI,
	CLK_SEL_OCTA,
	CLK_SEL_SDHI0,
	CLK_SEL_SDHI1,
	CLK_SEL_SDHI2,
	CLK_SEL_PLL4,
	CLK_P1_DIV2,
	CLK_P0_DIV2,
	CLK_P4_DIV2,
	CLK_P3_DIV2,
	CLK_SD0_DIV4,
	CLK_SD1_DIV4,
	CLK_SD2_DIV4,

	/* Module Clocks */
	MOD_CLK_BASE,
};

struct rzg3s_cpg_core_clk {
	uint16_t	parent;
	uint32_t	out_freq;
};

#define DEF_INPUT(_id, _freq) \
	[_id] = {.out_freq = _freq, .parent = (-1)}

#define DEF_FIXED(_id, _parent, _freq) \
	[_id] = {.out_freq = _freq, .parent = _parent}

static const struct rzg3s_cpg_core_clk rzg3s_core_clks[MOD_CLK_BASE] = {
	/* External Clock Inputs */
	DEF_INPUT(CLK_EXTAL, 24000000),

	DEF_FIXED(R9A08G045_OSCCLK, CLK_EXTAL, 24000000),
	DEF_FIXED(R9A08G045_OSCCLK2, CLK_EXTAL, 8000000),
	DEF_FIXED(R9A08G045_CLK_SD0, CLK_SEL_SDHI0, 133333000),
	DEF_FIXED(R9A08G045_CLK_SD1, CLK_SEL_SDHI1, 133333000),
	DEF_FIXED(R9A08G045_CLK_SD2, CLK_SEL_SDHI2, 133333000),

	DEF_FIXED(R9A08G045_CLK_I, CLK_PLL1, 1100000000),
	DEF_FIXED(R9A08G045_CLK_I2, CLK_PLL6_DIV2, 250000000),
	DEF_FIXED(R9A08G045_CLK_I3, CLK_PLL6_DIV2, 250000000),
	DEF_FIXED(R9A08G045_CLK_S0, CLK_SEL_PLL4, 400000000),
	DEF_FIXED(R9A08G045_CLK_P0, CLK_PLL2_DIV2_8, 100000000),
	DEF_FIXED(R9A08G045_CLK_P1, CLK_PLL3_DIV2_4, 200000000),
	DEF_FIXED(R9A08G045_CLK_P2, CLK_PLL3_DIV2_8, 100000000),
	DEF_FIXED(R9A08G045_CLK_P3, CLK_PLL3_DIV2_4, 200000000),
	DEF_FIXED(R9A08G045_CLK_P4, CLK_PLL2_DIV2, 160000000),
	DEF_FIXED(R9A08G045_CLK_P5, CLK_PLL2_DIV2, 200000000),
	DEF_FIXED(R9A08G045_CLK_M0, CLK_PLL3_DIV2_4, 200000000),
	DEF_FIXED(R9A08G045_CLK_ZT, CLK_PLL3_DIV2_8, 100000000),
	DEF_FIXED(R9A08G045_CLK_HP, CLK_PLL6, 250000000),
	DEF_FIXED(R9A08G045_CLK_SPI0, CLK_SEL_SPI, 33333000),
	DEF_FIXED(R9A08G045_CLK_SPI1, R9A08G045_CLK_SPI0, 16667000),
	DEF_FIXED(R9A08G045_CLK_OC0, CLK_SEL_OCTA, 33333000),
	DEF_FIXED(R9A08G045_CLK_OC1, CLK_SEL_OCTA, 16667000),
	DEF_FIXED(R9A08G045_CLK_TSU, CLK_PLL2_DIV2, 100000000),
	DEF_FIXED(R9A08G045_CLK_AT, CLK_PLL3_DIV2_2, 400000000),
	DEF_FIXED(CLK_P4_DIV2, R9A08G045_CLK_P4, 80000000),
};

struct rzg3s_mod_clk {
	uint16_t parent;
	uint16_t clkon_off;
	uint16_t clkon_bit;
	uint16_t mstop_off;
	uint16_t mstop_bits;
};

#define DEF_MOD(_id, _parent, _off, _bit, _mstop_off, _mstop_bit)	\
	[_id] = { \
		.parent = (_parent), \
		.clkon_off = (_off), \
		.clkon_bit = (_bit), \
		.mstop_off = (_mstop_off), \
		.mstop_bits = (_mstop_bit), \
	}

static const struct rzg3s_mod_clk rzg3s_mod_clks[R9A08G045_LAST_CLK] = {
	DEF_MOD(R9A08G045_GIC600_GICCLK, R9A08G045_CLK_P1, 0x514, 0, 0, 0),
	DEF_MOD(R9A08G045_IA55_PCLK, R9A08G045_CLK_P2, 0x518, 0, 0, 0),
	DEF_MOD(R9A08G045_IA55_CLK, R9A08G045_CLK_P1,
		0x518, 1, PERI_CPU_MSTOP, BIT(13)),
	DEF_MOD(R9A08G045_DMAC_ACLK, R9A08G045_CLK_P3,
		0x52c, 0, REG1_MSTOP, BIT(2) | BIT(3)),
	DEF_MOD(R9A08G045_DMAC_PCLK, CLK_P3_DIV2, 0x52c, 1, 0, 0),
	DEF_MOD(R9A08G045_OSTM0_PCLK, R9A08G045_CLK_P0,
		0x534, 0, REG0_MSTOP, BIT(4)),
	DEF_MOD(R9A08G045_OSTM1_PCLK, R9A08G045_CLK_P0,
		0x534, 1, REG0_MSTOP, BIT(5)),
	DEF_MOD(R9A08G045_OSTM2_PCLK, R9A08G045_CLK_P0,
		0x534, 2, REG0_MSTOP, BIT(6)),
	DEF_MOD(R9A08G045_OSTM3_PCLK, R9A08G045_CLK_P0,
		0x534, 3, REG0_MSTOP, BIT(7)),
	DEF_MOD(R9A08G045_OSTM4_PCLK, R9A08G045_CLK_P0,
		0x534, 4, REG0_MSTOP, BIT(8)),
	DEF_MOD(R9A08G045_OSTM5_PCLK, R9A08G045_CLK_P0,
		0x534, 5, REG0_MSTOP, BIT(9)),
	DEF_MOD(R9A08G045_OSTM6_PCLK, R9A08G045_CLK_P0,
		0x534, 6, REG0_MSTOP, BIT(10)),
	DEF_MOD(R9A08G045_OSTM7_PCLK, R9A08G045_CLK_P0,
		0x534, 7, REG0_MSTOP, BIT(11)),
	DEF_MOD(R9A08G045_MTU_X_MCK_MTU3, R9A08G045_CLK_P0,
		0x538, 0, MCPU1_MSTOP, BIT(2)),
	DEF_MOD(R9A08G045_POE3_CLKM_POE, R9A08G045_CLK_P0,
		0x53C, 0, MCPU1_MSTOP, BIT(9)),
	DEF_MOD(R9A08G045_GPT_PCLK, R9A08G045_CLK_P0,
		0x540, 0, MCPU3_MSTOP, BIT(5)),
	DEF_MOD(R9A08G045_POEG_A_CLKP, R9A08G045_CLK_P0,
		0x544, 0, 0, 0),
	DEF_MOD(R9A08G045_POEG_B_CLKP, R9A08G045_CLK_P0,
		0x544, 1, 0, 0),
	DEF_MOD(R9A08G045_POEG_C_CLKP, R9A08G045_CLK_P0,
		0x544, 2, 0, 0),
	DEF_MOD(R9A08G045_POEG_D_CLKP, R9A08G045_CLK_P0,
		0x544, 3, 0, 0),
	DEF_MOD(R9A08G045_WDT0_PCLK, R9A08G045_CLK_P0,
		0x548, 0, 0, 0),
	DEF_MOD(R9A08G045_WDT0_CLK, R9A08G045_OSCCLK,
		0x548, 1, REG0_MSTOP, BIT(0)),
	DEF_MOD(R9A08G045_SPI_HCLK, R9A08G045_CLK_P3,
		0x550, 0, 0, 0),
	DEF_MOD(R9A08G045_SPI_ACLK, R9A08G045_CLK_P3,
		0x550, 1, 0, 0),
	DEF_MOD(R9A08G045_SPI_CLK, R9A08G045_CLK_SPI1,
		0x550, 2, MCPU1_MSTOP, BIT(1)),
	DEF_MOD(R9A08G045_SPI_CLKX2, R9A08G045_CLK_SPI0,
		0x550, 3, 0, 0),
	DEF_MOD(R9A08G045_SDHI0_IMCLK, CLK_SD0_DIV4,
		0x554, 0, PERI_COM_MSTOP, BIT(0)),
	DEF_MOD(R9A08G045_SDHI0_IMCLK2, CLK_SD0_DIV4,
		0x554, 1, 0, 0),
	DEF_MOD(R9A08G045_SDHI0_CLK_HS, R9A08G045_CLK_SD0,
		0x554, 2, 0, 0),
	DEF_MOD(R9A08G045_SDHI0_ACLK, R9A08G045_CLK_P1,
		0x554, 3, 0, 0),
	DEF_MOD(R9A08G045_SDHI1_IMCLK, CLK_SD1_DIV4,
		0x554, 4, PERI_COM_MSTOP, BIT(1)),
	DEF_MOD(R9A08G045_SDHI1_IMCLK2, CLK_SD1_DIV4,
		0x554, 5, 0, 0),
	DEF_MOD(R9A08G045_SDHI1_CLK_HS, R9A08G045_CLK_SD1,
		0x554, 6, 0, 0),
	DEF_MOD(R9A08G045_SDHI1_ACLK, R9A08G045_CLK_P1,
		0x554, 7, 0, 0),
	DEF_MOD(R9A08G045_SDHI2_IMCLK, CLK_SD2_DIV4,
		0x554, 8, 0, 0),
	DEF_MOD(R9A08G045_SDHI2_IMCLK2, CLK_SD2_DIV4,
		0x554, 9, 0, 0),
	DEF_MOD(R9A08G045_SDHI2_CLK_HS, R9A08G045_CLK_SD2,
		0x554, 10, 0, 0),
	DEF_MOD(R9A08G045_SDHI2_ACLK, R9A08G045_CLK_P1,
		0x554, 11, 0, 0),
	DEF_MOD(R9A08G045_SSI0_PCLK2, R9A08G045_CLK_P0,
		0x570, 0, MCPU1_MSTOP, BIT(10)),
	DEF_MOD(R9A08G045_SSI0_PCLK_SFR, R9A08G045_CLK_P0,
		0x570, 1, 0, 0),
	DEF_MOD(R9A08G045_SSI1_PCLK2, R9A08G045_CLK_P0,
		0x570, 2, MCPU1_MSTOP, BIT(11)),
	DEF_MOD(R9A08G045_SSI1_PCLK_SFR, R9A08G045_CLK_P0,
		0x570, 3, 0, 0),
	DEF_MOD(R9A08G045_SSI2_PCLK2, R9A08G045_CLK_P0,
		0x570, 4, MCPU1_MSTOP, BIT(12)),
	DEF_MOD(R9A08G045_SSI2_PCLK_SFR, R9A08G045_CLK_P0,
		0x570, 5, 0, 0),
	DEF_MOD(R9A08G045_SSI3_PCLK2, R9A08G045_CLK_P0,
		0x570, 6, MCPU1_MSTOP, BIT(13)),
	DEF_MOD(R9A08G045_SSI3_PCLK_SFR, R9A08G045_CLK_P0,
		0x570, 7, 0, 0),
	DEF_MOD(R9A08G045_SRC_CLKP, R9A08G045_CLK_P0,
		0x574, 0, MCPU1_MSTOP, BIT(3)),
	DEF_MOD(R9A08G045_USB_U2H0_HCLK, R9A08G045_CLK_P1,
		0x578, 0, 0, 0),
	DEF_MOD(R9A08G045_USB_U2H1_HCLK, R9A08G045_CLK_P1,
		0x578, 1, 0, 0),
	DEF_MOD(R9A08G045_USB_U2P_EXR_CPUCLK, R9A08G045_CLK_P1,
		0x578, 2, 0, 0),
	DEF_MOD(R9A08G045_USB_PCLK, R9A08G045_CLK_P1,
		0x578, 3, PERI_COM_MSTOP,
			  BIT(4) | BIT(5) | BIT(6) | BIT(7)),
	DEF_MOD(R9A08G045_ETH0_CLK_AXI, R9A08G045_CLK_M0,
		0x57c, 0, PERI_COM_MSTOP, BIT(2)),
	DEF_MOD(R9A08G045_ETH0_REFCLK, R9A08G045_CLK_HP,
		0x57c, 8, 0, 0),
	DEF_MOD(R9A08G045_ETH1_CLK_AXI, R9A08G045_CLK_M0,
		0x57c, 1, PERI_COM_MSTOP, BIT(3)),
	DEF_MOD(R9A08G045_ETH1_REFCLK, R9A08G045_CLK_HP,
		0x57c, 9, 0, 0),
	DEF_MOD(R9A08G045_I2C0_PCLK, R9A08G045_CLK_P0,
		0x580, 0, MCPU2_MSTOP, BIT(10)),
	DEF_MOD(R9A08G045_I2C1_PCLK, R9A08G045_CLK_P0,
		0x580, 1, MCPU2_MSTOP, BIT(11)),
	DEF_MOD(R9A08G045_I2C2_PCLK, R9A08G045_CLK_P0,
		0x580, 2, MCPU2_MSTOP, BIT(12)),
	DEF_MOD(R9A08G045_I2C3_PCLK, R9A08G045_CLK_P0,
		0x580, 3, MCPU2_MSTOP, BIT(13)),
	DEF_MOD(R9A08G045_SCIF0_CLK_PCK, R9A08G045_CLK_P0,
		0x584, 0, MCPU2_MSTOP, BIT(1)),
	DEF_MOD(R9A08G045_SCIF1_CLK_PCK, R9A08G045_CLK_P0,
		0x584, 1, MCPU2_MSTOP, BIT(2)),
	DEF_MOD(R9A08G045_SCIF2_CLK_PCK, R9A08G045_CLK_P0,
		0x584, 2, MCPU2_MSTOP, BIT(3)),
	DEF_MOD(R9A08G045_SCIF3_CLK_PCK, R9A08G045_CLK_P0,
		0x584, 3, MCPU2_MSTOP, BIT(4)),
	DEF_MOD(R9A08G045_SCIF4_CLK_PCK, R9A08G045_CLK_P0,
		0x584, 4, MCPU2_MSTOP, BIT(5)),
	DEF_MOD(R9A08G045_SCIF5_CLK_PCK, R9A08G045_CLK_P0,
		0x584, 5, MCPU3_MSTOP, BIT(4)),
	DEF_MOD(R9A08G045_SCI0_CLKP, R9A08G045_CLK_P0,
		0x588, 0, MCPU2_MSTOP, BIT(7)),
	DEF_MOD(R9A08G045_SCI1_CLKP, R9A08G045_CLK_P0,
		0x588, 1, MCPU2_MSTOP, BIT(8)),
	DEF_MOD(R9A08G045_RSPI0_CLKB, R9A08G045_CLK_P0,
		0x590, 0, MCPU1_MSTOP, BIT(14)),
	DEF_MOD(R9A08G045_RSPI1_CLKB, R9A08G045_CLK_P0,
		0x590, 1, MCPU1_MSTOP, BIT(15)),
	DEF_MOD(R9A08G045_RSPI2_CLKB, R9A08G045_CLK_P0,
		0x590, 2, MCPU2_MSTOP, BIT(0)),
	DEF_MOD(R9A08G045_RSPI3_CLKB, R9A08G045_CLK_P0,
		0x590, 3, MCPU3_MSTOP, BIT(2)),
	DEF_MOD(R9A08G045_RSPI4_CLKB, R9A08G045_CLK_P0,
		0x590, 4, MCPU3_MSTOP, BIT(3)),
	DEF_MOD(R9A08G045_CANFD_PCLK, CLK_P4_DIV2,
		0x594, 0, MCPU2_MSTOP, BIT(9)),
	DEF_MOD(R9A08G045_CANFD_CLK_RAM, R9A08G045_CLK_P4,
		0x594, 1, 0, 0),
	DEF_MOD(R9A08G045_GPIO_HCLK, R9A08G045_OSCCLK,
		0x598, 0, 0, 0),
	DEF_MOD(R9A08G045_ADC_ADCLK, R9A08G045_CLK_TSU,
		0x5a8, 0, MCPU2_MSTOP, BIT(14)),
	DEF_MOD(R9A08G045_ADC_PCLK, R9A08G045_CLK_TSU,
		0x5a8, 1, 0, 0),
	DEF_MOD(R9A08G045_TSU_PCLK, R9A08G045_CLK_TSU,
		0x5ac, 0, MCPU2_MSTOP, BIT(15)),
	DEF_MOD(R9A08G045_OCTA_ACLK, R9A08G045_CLK_OC1,
		0x5f4, 0, 0, 0),
	DEF_MOD(R9A08G045_OCTA_MCLK, R9A08G045_CLK_OC0,
		0x5f4, 1, MCPU3_MSTOP, BIT(0)),
	DEF_MOD(R9A08G045_PDM_PCLK, R9A08G045_CLK_P0,
		0x604, 0, 0, 0),
	DEF_MOD(R9A08G045_PDM_CCLK, R9A08G045_OSCCLK2,
		0x604, 1, MCPU3_MSTOP, BIT(9)),
	DEF_MOD(R9A08G045_PCI_ACLK, R9A08G045_CLK_M0,
		0x608, 0, 0, 0),
	DEF_MOD(R9A08G045_PCI_CLKL1PM, R9A08G045_CLK_ZT,
		0x608, 1, 0, 0),
	DEF_MOD(R9A08G045_SPDIF_PCLK, R9A08G045_CLK_P0,
		0x60c, 0, MCPU3_MSTOP, BIT(6)),
	DEF_MOD(R9A08G045_I3C_TCLK, R9A08G045_CLK_P5,
		0x610, 1, MCPU3_MSTOP, BIT(10)),
	DEF_MOD(R9A08G045_I3C_PCLK, R9A08G045_CLK_TSU,
		0x610, 0, 0, 0),
	DEF_MOD(R9A08G045_VBAT_BCLK, R9A08G045_OSCCLK,
		0x614, 0, MCPU3_MSTOP, BIT(8)),
};

struct rzg3s_cpg_config {
	DEVICE_MMIO_ROM; /* Must be first */

	const struct rzg3s_cpg_core_clk *core_clks;
	const struct rzg3s_mod_clk *mod_clks;
};

struct rzg3s_cpg_data {
	DEVICE_MMIO_RAM; /* Must be first */

	struct k_spinlock lock;
};

static int rzg3s_cpg_module_checkpar(const struct device *dev, clock_control_subsys_t sys)
{
	struct renesas_cpg_clk *clk = (struct renesas_cpg_clk *)sys;

	if (!dev || !sys) {
		return -EINVAL;
	}

	if (clk->domain != CPG_MOD && clk->domain != CPG_CORE) {
		return -EINVAL;
	}

	if (clk->domain == CPG_MOD && clk->module >= R9A08G045_LAST_CLK) {
		LOG_DEV_ERR(dev, "wrong module id %u", clk->module);
		return -EINVAL;
	}

	if (clk->domain == CPG_CORE && clk->module >= MOD_CLK_BASE) {
		LOG_DEV_ERR(dev, "wrong core module id %u", clk->module);
		return -EINVAL;
	}

	return 0;
}

static int rzg3s_cpg_clk_get_rate(const struct device *dev,
				  clock_control_subsys_t sys, uint32_t *rate)
{
	struct renesas_cpg_clk *clk = (struct renesas_cpg_clk *)sys;
	const struct rzg3s_cpg_config *cfg = dev->config;
	int32_t ret = -EINVAL;
	uint32_t parent_id;

	if (!rate) {
		LOG_DEV_ERR(dev, "received null ptr rate");
		return ret;
	}

	ret = rzg3s_cpg_module_checkpar(dev, sys);
	if (ret) {
		return ret;
	}

	if (clk->domain == CPG_MOD) {
		parent_id = cfg->mod_clks[clk->module].parent;

		if (parent_id >= MOD_CLK_BASE) {
			LOG_DEV_ERR(dev, "wrong module:%u parent id %u", clk->module, parent_id);
			return -EINVAL;
		}

		ret = cfg->core_clks[parent_id].out_freq;
	} else if (clk->domain == CPG_CORE) {
		ret = cfg->core_clks[clk->module].out_freq;
	}

	*rate = ret;
	return 0;
}

static void rzg3s_cpg_change_reg_bits(mm_reg_t reg, uint32_t bitmask, bool set)
{
	uint32_t reg_val = (bitmask << 16);

	if (set) {
		reg_val |= bitmask;
	}

	sys_write32(reg_val, reg);
}

static int rzg3s_cpg_wait_bit_val(mm_reg_t reg_addr, uint32_t mask, uint32_t val,
				  int32_t us_wait)
{
	uint32_t reg_val;
	int32_t wait_cnt = (us_wait / 5);

	do {
		reg_val = sys_read32(reg_addr) & mask;

		if (reg_val == val) {
			break;
		}

		if (wait_cnt > 0) {
			int loops = 500;

			while (loops-- > 0) {
				arch_nop();
			}
		}
	} while (wait_cnt-- > 0);

	return wait_cnt;
}

static int rzg3s_cpg_mod_clock_on(const struct device *dev, uint32_t module)
{
	const struct rzg3s_cpg_config *cfg = dev->config;
	const struct rzg3s_mod_clk *mod_clk;
	mm_reg_t reg_addr;
	uint32_t val;
	int ret;

	mod_clk = &cfg->mod_clks[module];

	/* enable mod clock */
	reg_addr = DEVICE_MMIO_GET(dev) + mod_clk->clkon_off;
	val = BIT(mod_clk->clkon_bit);
	rzg3s_cpg_change_reg_bits(reg_addr, val, true);

	/* monitor mod clock */
	reg_addr = DEVICE_MMIO_GET(dev) + CLK_MON_R(mod_clk->clkon_off);
	ret = rzg3s_cpg_wait_bit_val(reg_addr, val, val, MON_REG_WAIT_US);
	if (!ret) {
		return -EIO;
	}

	/* clear mod mstop */
	reg_addr = DEVICE_MMIO_GET(dev) + mod_clk->mstop_off;
	val = mod_clk->mstop_bits;
	rzg3s_cpg_change_reg_bits(reg_addr, val, false);

	return 0;
}

static int rzg3s_cpg_mod_clock_off(const struct device *dev, uint32_t module)
{
	const struct rzg3s_cpg_config *cfg = dev->config;
	const struct rzg3s_mod_clk *mod_clk;
	mm_reg_t reg_addr;
	uint32_t val;
	int ret;

	mod_clk = &cfg->mod_clks[module];

	/* set mod mstop */
	reg_addr = DEVICE_MMIO_GET(dev) + mod_clk->mstop_off;
	val = mod_clk->mstop_bits;
	rzg3s_cpg_change_reg_bits(reg_addr, val, true);

	/* disable mod clock */
	reg_addr = DEVICE_MMIO_GET(dev) + mod_clk->clkon_off;
	val = BIT(mod_clk->clkon_bit);
	rzg3s_cpg_change_reg_bits(reg_addr, val, false);

	/* monitor mod clock */
	reg_addr = DEVICE_MMIO_GET(dev) + CLK_MON_R(mod_clk->clkon_off);
	ret = rzg3s_cpg_wait_bit_val(reg_addr, val, 0, MON_REG_WAIT_US);
	if (!ret) {
		LOG_DEV_DBG(dev, "module id %u enable tmo", module);
		return -EIO;
	}

	return 0;
}

static int rzg3s_cpg_module_start(const struct device *dev, clock_control_subsys_t sys)
{
	struct renesas_cpg_clk *clk = (struct renesas_cpg_clk *)sys;
	struct rzg3s_cpg_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	ret = rzg3s_cpg_module_checkpar(dev, sys);
	if (ret) {
		return ret;
	}

	if (clk->domain == CPG_CORE) {
		LOG_DEV_WRN(dev, "en/dis of core clocks aren't supported by this controller");
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	ret = rzg3s_cpg_mod_clock_on(dev, clk->module);
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int rzg3s_cpg_module_stop(const struct device *dev, clock_control_subsys_t sys)
{
	struct renesas_cpg_clk *clk = (struct renesas_cpg_clk *)sys;
	struct rzg3s_cpg_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	ret = rzg3s_cpg_module_checkpar(dev, sys);
	if (ret) {
		return ret;
	}

	if (clk->domain == CPG_CORE) {
		LOG_DEV_WRN(dev, "en/dis of core clocks aren't supported by this controller");
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	ret = rzg3s_cpg_mod_clock_off(dev, clk->module);
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int rzg3s_cpg_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static const struct clock_control_driver_api rzg3s_cpg_api = {
	.on = rzg3s_cpg_module_start,
	.off = rzg3s_cpg_module_stop,
	.get_rate = rzg3s_cpg_clk_get_rate,
};

#define RZG3S_CPG_INIT(inst)                                                             \
	static const struct rzg3s_cpg_config rzg3s_cpg_##inst##_config = {       \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                  \
		.core_clks = rzg3s_core_clks,                                         \
		.mod_clks = rzg3s_mod_clks,                                           \
	};                                                                                \
                                                                                          \
	static struct rzg3s_cpg_data rzg3s_cpg_##inst##_data = {                 \
	};                                                                                \
	                                                                                  \
	DEVICE_DT_INST_DEFINE(inst, rzg3s_cpg_init, NULL, &rzg3s_cpg_##inst##_data,   \
			      &rzg3s_cpg_##inst##_config, PRE_KERNEL_1,                    \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &rzg3s_cpg_api);

DT_INST_FOREACH_STATUS_OKAY(RZG3S_CPG_INIT)
