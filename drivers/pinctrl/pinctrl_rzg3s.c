/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r9a08g045_pinctrl

#include <errno.h>
#include <soc.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/renesas/pinctrl-r9a08g045.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pinctrl_rzg3s, CONFIG_PINCTRL_LOG_LEVEL);

/* TODO: remove */
#include "../../soc/arm/renesas_rz/rz_g/pinctrl_soc.h"

#define LOG_DEV_ERR(dev, format, ...)	LOG_ERR("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...)	LOG_WRN("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...)	LOG_INF("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...)	LOG_DBG("%s:"#format, (dev)->name, ##__VA_ARGS__)

#define RZG3S_PORT0	0x20
#define RZG3S_PORT5	0x21
#define RZG3S_PORT6	0x22
#define RZG3S_PORT11	0x23
#define RZG3S_PORT12	0x24
#define RZG3S_PORT13	0x25
#define RZG3S_PORT14	0x26
#define RZG3S_PORT15	0x27
#define RZG3S_PORT16	0x28
#define RZG3S_PORT17	0x29
#define RZG3S_PORT18	0x2A
#define RZG3S_PORT1	0x30
#define RZG3S_PORT2	0x31
#define RZG3S_PORT3	0x32
#define RZG3S_PORT4	0x33
#define RZG3S_PORT7	0x34
#define RZG3S_PORT8	0x35
#define RZG3S_PORT9	0x36
#define RZG3S_PORT10	0x37

#define RZG3S_PORT_NUM		19

/*
 * n indicates number of pins in the port, a is the register index
 * and f is pin configuration capabilities supported.
 */
#define RZG3S_GPIO_PORT_DEF(num_pins, reg_ofs)	(((num_pins) << 16) | (reg_ofs))
#define RZG3S_GPIO_PORT_GET_PINCNT(p)	(((p) & GENMASK(31, 16)) >> 16)
#define RZG3S_GPIO_PORT_GET_REG(x)	(((x) & GENMASK(15, 0)))

static const uint32_t rzg3s_pinctrl_ports[RZG3S_PORT_NUM] = {
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT0),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT1),
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT2),
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT3),
	RZG3S_GPIO_PORT_DEF(6, RZG3S_PORT4),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT5),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT6),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT7),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT8),
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT9),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT10),
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT11),
	RZG3S_GPIO_PORT_DEF(2, RZG3S_PORT12),
	RZG3S_GPIO_PORT_DEF(5, RZG3S_PORT13),
	RZG3S_GPIO_PORT_DEF(3, RZG3S_PORT14),
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT15),
	RZG3S_GPIO_PORT_DEF(2, RZG3S_PORT16),
	RZG3S_GPIO_PORT_DEF(4, RZG3S_PORT17),
	RZG3S_GPIO_PORT_DEF(6, RZG3S_PORT18),
};

#define R_PM(n)			(0x0100 + (n) * 2)
#define R_PMC(n)		(0x0200 + (n))
#define R_PFC(n)		(0x0400 + (n) * 4)
#define R_IOLH(n)		(0x1000 + (n) * 8)
#define R_IEN(n)		(0x1800 + (n) * 8)
#define R_PUPD(n)		(0x1C00 + (n) * 8)
#define R_PWPR			(0x3000)

/* R_PWPR */
#define R_PWPR_PFSWE	BIT(6) /* PFSWE Bit Enable */
#define R_PWPR_B0WI	BIT(7) /* PFSWE Bit Disable */

/* R_PM Port Mode Register */
#define R_PM_MODE_MASK			GENMASK(1, 0)
#define R_PM_MODE_VAL(pin, val)		(((val) & R_PM_MODE_MASK) << ((pin) << 1))

#define R_PM_MODE_HI_Z			0x0
#define R_PM_MODE_IN			BIT(0)
#define R_PM_MODE_OUT			BIT(1)
#define R_PM_MODE_OUT_IN		(R_PM_MODE_IN | R_PM_MODE_OUT)

/* R_PFC Port Function Control Register */
#define R_PFC_FUNC_MASK			GENMASK(2, 0)
#define R_PFC_FUNC_VAL(pin, val)	(((val) & R_PFC_FUNC_MASK) << ((pin) << 2))

/* R_PUPD Pull-Up/Pull-Down Switching Register */
#define R_PUPD_MASK			GENMASK(1, 0)
#define R_PUPD_VAL(pin, val)		(((val) & R_PUPD_MASK) << ((pin) << 3))
#define R_PUPD_NONE			0x0
#define R_PUPD_UP			BIT(0)
#define R_PUPD_DOWN			BIT(1)

/* R_IOLH Driving Ability Control Register */
#define R_IOLH_MASK			GENMASK(1, 0)
#define R_IOLH_VAL(pin, val)		(((val) & R_IOLH_MASK) << ((pin) << 3))

/* R_IEN Input Enable Control Register */
#define R_IEN_MASK			BIT(0)
#define R_IEN_VAL(pin, val)		(((val) & R_IEN_MASK) << ((pin) << 3))

#define R_SD_CH0_POC			(0x3004)
#define R_SD_CH1_POC			(0x3008)
#define R_XSPI_POC			(0x300C)
#define R_ETH0_POC			(0x3010)
#define R_ETH1_POC			(0x3014)
#define R_ETH_MODE			(0x3018)
#define R_I3C_SET			(0x301C)
#define R_XSPI_HI_Z			(0x3020)

typedef int (*rzg3s_pinctrl_grp_handler)(const struct device *dev,
					 const struct pinctrl_soc_rzg3s_grp *grp);

#define SPIN_CFG_IOLH			BIT(0)
#define SPIN_CFG_IEN			BIT(1)
#define SPIN_CFG_FILONOFF		BIT(2)
#define SPIN_CFG_ETH_OEN		BIT(3)

struct rzg3s_pinctrl_spin_cfg {
	uint16_t	reg_idx;	/* IEN/IOLH/FILxx registers index */
	uint8_t		reg_pin;	/* IEN/IOLH/FILxx registers pin (p * 8) */
	uint8_t		cfg;		/* pin configuration capabilities supported */
};

#define SPIN_DEF(_id, _reg_idx, _reg_pin, _cfg) \
		[_id] = {.reg_idx = _reg_idx, .reg_pin = _reg_pin, .cfg = _cfg}

static const struct rzg3s_pinctrl_spin_cfg rzg3s_pinctrl_spins[PINCTRL_RZG3S_SPIN_LAST] = {
	SPIN_DEF(PINCTRL_RZG3S_NMI_SPIN,	0x0, 0, (SPIN_CFG_FILONOFF)),
	SPIN_DEF(PINCTRL_RZG3S_TMS_SWDIO_SPIN,	0x1, 0, (SPIN_CFG_IEN | SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_TDO_SPIN,	0x1, 1, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_SPCLK_SPIN,	0x4, 0, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_RESET_SPIN,	0x4, 1, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_WP_SPIN,	0x4, 2, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_DS_SPIN,	0x4, 3, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_CS0_SPIN,	0x4, 4, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_CS1_SPIN,	0x4, 5, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO0_SPIN,	0x5, 0, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO1_SPIN,	0x5, 1, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO2_SPIN,	0x5, 2, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO3_SPIN,	0x5, 3, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO4_SPIN,	0x5, 4, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO5_SPIN,	0x5, 5, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO6_SPIN,	0x5, 6, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_XSPI_IO7_SPIN,	0x5, 7, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_WDTOVF_PERROUT_SPIN, 0x6, 0, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_CLK_SPIN,	0x10, 0, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_CMD_SPIN,	0x10, 1, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_RST_SPIN,	0x10, 2, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA0_SPIN,	0x11, 0, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA1_SPIN,	0x11, 1, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA2_SPIN,	0x11, 2, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA3_SPIN,	0x11, 3, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA4_SPIN,	0x11, 4, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA5_SPIN,	0x11, 5, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA6_SPIN,	0x11, 6, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD0_DATA7_SPIN,	0x11, 7, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD1_CLK_SPIN,	0x12, 0, (SPIN_CFG_IOLH)),
	SPIN_DEF(PINCTRL_RZG3S_SD1_CMD_SPIN,	0x12, 1, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD1_DATA0_SPIN,	0x13, 0, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD1_DATA1_SPIN,	0x13, 1, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD1_DATA2_SPIN,	0x13, 2, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD1_DATA3_SPIN,	0x13, 3, (SPIN_CFG_IOLH | SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_AUDIO_CLK1_SPIN,	0x2, 0, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_AUDIO_CLK2_SPIN,	0x2, 1, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_I3C_SDA_SPIN,	0x9, 0, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_I3C_SCL_SPIN,	0x9, 1, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD2_CMD_SPIN,	0x23, 1, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD2_DATA0_SPIN,	0x23, 2, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD2_DATA1_SPIN,	0x23, 3, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD2_DATA2_SPIN,	0x24, 0, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_SD2_DATA3_SPIN,	0x24, 1, (SPIN_CFG_IEN)),
	SPIN_DEF(PINCTRL_RZG3S_ET0_TXC_TX_CLK_SPIN, 0x30, 0, (SPIN_CFG_IEN | SPIN_CFG_ETH_OEN)),
	SPIN_DEF(PINCTRL_RZG3S_ET1_TXC_TX_CLK_SPIN, 0x34, 0, (SPIN_CFG_IEN | SPIN_CFG_ETH_OEN)),
	SPIN_DEF(PINCTRL_RZG3S_ET0_TX_CTL_TX_EN_SPIN,	0, 0, (SPIN_CFG_ETH_OEN)),
	SPIN_DEF(PINCTRL_RZG3S_ET1_TX_CTL_TX_EN_SPIN,	0, 0, (SPIN_CFG_ETH_OEN)),
};

struct rzg3s_pinctrl_cfg {
	DEVICE_MMIO_NAMED_ROM(pinctrl);
};

struct rzg3s_pinctrl_data {
	DEVICE_MMIO_NAMED_ROM(pinctrl);

	struct k_spinlock lock;
};

#define DEV_DATA(dev) ((struct rzg3s_pinctrl_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct rzg3s_pinctrl_cfg *)((dev)->config))

static uint16_t rzg3s_pinctrl_port_get_reg(uint8_t port)
{
	return RZG3S_GPIO_PORT_GET_REG(rzg3s_pinctrl_ports[port]);
}

static uint16_t rzg3s_pinctrl_port_get_pinnum(uint8_t port)
{
	return RZG3S_GPIO_PORT_GET_PINCNT(rzg3s_pinctrl_ports[port]);
}

static void rzg3s_pinctrl_pfc_we(const struct device *dev)
{
	/* PFS Register Write Enable B0WI=0 R_PFSWE=1*/
	sys_write8(0x00, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PWPR);
	sys_write8(R_PWPR_PFSWE, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PWPR);
}

static void rzg3s_pinctrl_pfc_wd(const struct device *dev)
{
	/* PFS Register Write Protect B0WI=1 R_PFSWE=0 */
	sys_write8(0x00, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PWPR);
	sys_write8(R_PWPR_B0WI, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PWPR);
}

static void rzg3s_pinctrl_pfc_set(const struct device *dev,
				  const struct pinctrl_soc_rzg3s_pinmux *pinmux)
{
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(pinmux->port);
	uint32_t reg32;
	uint16_t reg16;
	uint8_t reg8;

	/* Set pin to 'Non-use (Hi-z input protection)'  */
	reg16 = sys_read16(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PM(port_reg));
	reg16 &= ~R_PM_MODE_VAL(pinmux->pin, R_PM_MODE_MASK);
	sys_write16(reg16, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PM(port_reg));

	/* switch to GPIO (temp) */
	reg8 = sys_read8(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PMC(port_reg));
	reg8 &= ~BIT(pinmux->pin);
	sys_write8(reg8, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PMC(port_reg));

	if (pinmux->func == PINCTRL_RZG3S_FUNC_GPIO) {
		return;
	}

	rzg3s_pinctrl_pfc_we(dev);

	/* Set Pin function */
	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PFC(port_reg));
	reg32 &= ~R_PFC_FUNC_VAL(pinmux->pin, R_PFC_FUNC_MASK);
	reg32 |= R_PFC_FUNC_VAL(pinmux->pin, (pinmux->func - 1));
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PFC(port_reg));

	rzg3s_pinctrl_pfc_wd(dev);

	/* Port Mode  : Peripheral module pin functions */
	reg8 = sys_read8(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PMC(port_reg));
	reg8 |= BIT(pinmux->pin);
	sys_write8(reg8, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PMC(port_reg));
}

static void rzg3s_pinctrl_pupd_set(const struct device *dev,
				   const struct pinctrl_soc_rzg3s_pinmux *pinmux)
{
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(pinmux->port);
	uint8_t pin = pinmux->pin;
	uint8_t pull = R_PUPD_NONE;
	uint32_t reg32;

	if (pinmux->pull_pin_default) {
		return;
	}

	if (pinmux->pull_up) {
		pull = R_PUPD_UP;
	} else if (pinmux->pull_down) {
		pull = R_PUPD_DOWN;
	}

	/* handle _L/_H for 32-bit register read/write */
	if (pin >= 4) {
		pin -= 4;
		port_reg += 4;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PUPD(port_reg));
	reg32 &= ~R_PUPD_VAL(pin, R_PUPD_MASK);
	reg32 |= R_PUPD_VAL(pin, pull);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_PUPD(port_reg));
}

static void rzg3s_pinctrl_pinmux_iolh_set(const struct device *dev,
					  const struct pinctrl_soc_rzg3s_pinmux *pinmux)
{
	uint16_t port_reg = rzg3s_pinctrl_port_get_reg(pinmux->port);
	uint8_t pin = pinmux->pin;
	uint8_t iolh = pinmux->drive_strength_microamp;
	uint32_t reg32;

	if (!pinmux->drive_strength) {
		return;
	}

	/* handle _L/_H for 32-bit register read/write */
	if (pin >= 4) {
		pin -= 4;
		port_reg += 4;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_IOLH(port_reg));
	reg32 &= ~R_IOLH_VAL(pin, R_IOLH_MASK);
	reg32 |= R_IOLH_VAL(pin, iolh);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_IOLH(port_reg));
}

static int rzg3s_pinctrl_pinmux(const struct device *dev,
				const struct pinctrl_soc_rzg3s_pinmux *pinmux)
{
	int ret = 0;

	if (pinmux->port >= RZG3S_PORT_NUM) {
		LOG_DEV_ERR(dev, "port:%u:%u func:%u invalid port",
			    pinmux->port, pinmux->pin, pinmux->func);
		return -EINVAL;
	}
	if (pinmux->pin >= rzg3s_pinctrl_port_get_pinnum(pinmux->port)) {
		LOG_DEV_ERR(dev, "port:%u:%u func:%u invalid pin",
			    pinmux->port, pinmux->pin, pinmux->func);
		return -EINVAL;
	}
	if (pinmux->func > (R_PFC_FUNC_MASK + 1)) {
		LOG_DEV_ERR(dev, "port:%u:%u invalid func:%u",
			    pinmux->port, pinmux->pin, pinmux->func);
		return -EINVAL;
	}
	if (pinmux->pull_up && pinmux->pull_down) {
		LOG_DEV_ERR(dev, "port:%u:%u func:%u pull up/down err",
			    pinmux->port, pinmux->pin, pinmux->func);
		return -EINVAL;
	}
	if (pinmux->drive_strength && pinmux->drive_strength_microamp > R_IOLH_MASK) {
		LOG_DEV_ERR(dev, "port:%u:%u func:%u drive-strength invalid",
			    pinmux->port, pinmux->pin, pinmux->func);
		return -EINVAL;
	}

	rzg3s_pinctrl_pfc_set(dev, pinmux);
	rzg3s_pinctrl_pupd_set(dev, pinmux);
	rzg3s_pinctrl_pinmux_iolh_set(dev, pinmux);
	return ret;
}

static int rzg3s_pinctrl_grp_poc_raw(const struct device *dev,
				     const struct pinctrl_soc_rzg3s_grp *grp)
{
	uint32_t reg_shift = 0;
	uint32_t reg_ofs;
	uint32_t reg_msk;
	uint32_t reg32;

	if (!grp->f_power_source) {
		return 0;
	}

	switch (grp->grp) {
	case PINCTRL_RZG3S_ETH0_GRP:
		reg_ofs = R_ETH0_POC;
		reg_msk = 0x3;
		break;
	case PINCTRL_RZG3S_ETH1_GRP:
		reg_ofs = R_ETH1_POC;
		reg_msk = 0x3;
		break;
	case PINCTRL_RZG3S_SD_CH0_GRP:
		reg_ofs = R_SD_CH0_POC;
		reg_msk = 0x1;
		break;
	case PINCTRL_RZG3S_SD_CH1_GRP:
		reg_ofs = R_SD_CH1_POC;
		reg_msk = 0x1;
		break;
	case PINCTRL_RZG3S_XSPI_GRP:
		reg_ofs = R_XSPI_POC;
		reg_msk = 0x3;
		break;
	case PINCTRL_RZG3S_I3C_GRP:
		reg_ofs = R_I3C_SET;
		reg_msk = 0x1;
		reg_shift = 0x1;
		break;
	default:
		return -EINVAL;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + reg_ofs);
	reg32 &= ~reg_msk << reg_shift;
	reg32 |= grp->power_source << reg_shift;
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + reg_ofs);

	return 0;
}

static int rzg3s_pinctrl_grp_poc(const struct device *dev, const struct pinctrl_soc_rzg3s_grp *grp)
{
	if (grp->bias_disable ||
	    grp->bias_high_impedance ||
	    grp->low_power_disable ||
	    grp->low_power_enable) {
		LOG_DEV_ERR(dev, "grp:%u: invalid cfg", grp->grp);
		return -EINVAL;
	}

	return rzg3s_pinctrl_grp_poc_raw(dev, grp);
}

static int rzg3s_pinctrl_grp_xspi(const struct device *dev,
				  const struct pinctrl_soc_rzg3s_grp *grp)
{
	uint32_t reg_ofs = R_XSPI_HI_Z;
	uint32_t reg32;
	int ret;

	if (grp->low_power_disable || grp->low_power_enable) {
		LOG_DEV_ERR(dev, "grp:%u: invalid cfg", grp->grp);
		return -EINVAL;
	}

	if (grp->bias_disable && grp->bias_high_impedance) {
		LOG_DEV_ERR(dev, "grp:%u: invalid Hi-Z cfg", grp->grp);
		return -EINVAL;
	}

	ret = rzg3s_pinctrl_grp_poc_raw(dev, grp);
	if (ret) {
		return ret;
	}

	if (!grp->bias_disable && !grp->bias_high_impedance) {
		return 0;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + reg_ofs);
	reg32 &= ~BIT(0);
	if (grp->bias_disable) {
		reg32 |= BIT(0);
	}
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + reg_ofs);

	return 0;
}

static int rzg3s_pinctrl_grp_i3c(const struct device *dev, const struct pinctrl_soc_rzg3s_grp *grp)
{
	uint32_t reg_ofs = R_I3C_SET;
	uint32_t reg32;
	int ret;

	if (grp->bias_disable || grp->bias_high_impedance) {
		LOG_DEV_ERR(dev, "grp:%u: invalid cfg", grp->grp);
		return -EINVAL;
	}

	if (grp->low_power_disable && grp->low_power_enable) {
		LOG_DEV_ERR(dev, "grp:%u: invalid low_power cfg", grp->grp);
		return -EINVAL;
	}

	ret = rzg3s_pinctrl_grp_poc_raw(dev, grp);
	if (ret) {
		return ret;
	}

	if (!grp->low_power_disable && !grp->low_power_enable) {
		return 0;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + reg_ofs);
	reg32 &= ~BIT(0);
	if (grp->low_power_disable) {
		reg32 |= BIT(0);
	}
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + reg_ofs);

	return 0;
}

static const rzg3s_pinctrl_grp_handler rzg3s_pinctrl_grps[PINCTRL_RZG3S_GRP_LAST] = {
	[PINCTRL_RZG3S_ETH0_GRP] = rzg3s_pinctrl_grp_poc,
	[PINCTRL_RZG3S_ETH1_GRP] = rzg3s_pinctrl_grp_poc,
	[PINCTRL_RZG3S_SD_CH0_GRP] = rzg3s_pinctrl_grp_poc,
	[PINCTRL_RZG3S_SD_CH1_GRP] = rzg3s_pinctrl_grp_poc,
	[PINCTRL_RZG3S_SD_CH1_GRP] = rzg3s_pinctrl_grp_poc,
	[PINCTRL_RZG3S_XSPI_GRP] = rzg3s_pinctrl_grp_xspi,
	[PINCTRL_RZG3S_I3C_GRP] = rzg3s_pinctrl_grp_i3c,
};

static int rzg3s_pinctrl_pingrp(const struct device *dev, const struct pinctrl_soc_rzg3s_grp *grp)
{
	if (grp->grp >= PINCTRL_RZG3S_GRP_LAST) {
		LOG_DEV_ERR(dev, "grp:%u: invalid group", grp->grp);
		return -EINVAL;
	}

	return rzg3s_pinctrl_grps[grp->grp](dev, grp);
}

static int rzg3s_pinctrl_spin_iolh_set(const struct device *dev,
				       const struct pinctrl_soc_rzg3s_spin *spin)
{
	const struct rzg3s_pinctrl_spin_cfg *spin_cfg = &rzg3s_pinctrl_spins[spin->spin];
	uint8_t iolh = spin->drive_strength_microamp;
	uint16_t port_reg = spin_cfg->reg_idx;
	uint8_t pin = spin_cfg->reg_pin;
	uint32_t reg32;

	if (!spin->drive_strength) {
		return 0;
	}

	if (spin->drive_strength && !(spin_cfg->cfg & SPIN_CFG_IOLH)) {
		LOG_DEV_ERR(dev, "spin:%u: invalid drive-strength cfg", spin->spin);
		return -EINVAL;
	}

	/* handle _L/_H for 32-bit register read/write */
	if (pin >= 4) {
		pin -= 4;
		port_reg += 4;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_IOLH(port_reg));
	reg32 &= ~R_IOLH_VAL(pin, R_IOLH_MASK);
	reg32 |= R_IOLH_VAL(pin, iolh);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_IOLH(port_reg));
	return 0;
}

static int rzg3s_pinctrl_spin_ien_set(const struct device *dev,
				      const struct pinctrl_soc_rzg3s_spin *spin)
{
	const struct rzg3s_pinctrl_spin_cfg *spin_cfg = &rzg3s_pinctrl_spins[spin->spin];
	uint8_t iolh = spin->drive_strength_microamp;
	uint16_t port_reg = spin_cfg->reg_idx;
	uint8_t pin = spin_cfg->reg_pin;
	uint32_t reg32;

	if (!spin->input_disable && !spin->input_enable) {
		return 0;
	}

	if (spin->input_disable && spin->input_enable) {
		LOG_DEV_ERR(dev, "spin:%u: invalid input-disable|enable cfg", spin->spin);
		return -EINVAL;
	}

	if (!(spin_cfg->cfg & SPIN_CFG_IEN)) {
		LOG_DEV_ERR(dev, "spin:%u: input-disable|enable not supported", spin->spin);
		return -EINVAL;
	}

	/* handle _L/_H for 32-bit register read/write */
	if (pin >= 4) {
		pin -= 4;
		port_reg += 4;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_IEN(port_reg));
	reg32 &= ~R_IEN_VAL(pin, R_IEN_MASK);
	reg32 |= R_IEN_VAL(pin, iolh);
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_IEN(port_reg));
	return 0;
}

static int rzg3s_pinctrl_spin_oen_set(const struct device *dev,
				      const struct pinctrl_soc_rzg3s_spin *spin)
{
	const struct rzg3s_pinctrl_spin_cfg *spin_cfg = &rzg3s_pinctrl_spins[spin->spin];
	uint8_t pin;
	uint32_t reg32;

	if (!spin->input_enable && !spin->output_enable) {
		return 0;
	}

	if (spin->spin == PINCTRL_RZG3S_ET0_TXC_TX_CLK_SPIN) {
		pin = BIT(0);
	} else if (spin->spin == PINCTRL_RZG3S_ET1_TXC_TX_CLK_SPIN) {
		pin = BIT(1);
	} else if (spin->spin == PINCTRL_RZG3S_ET0_TX_CTL_TX_EN_SPIN) {
		pin = BIT(2);
	} else if (spin->spin == PINCTRL_RZG3S_ET1_TX_CTL_TX_EN_SPIN) {
		pin = BIT(3);
	} else {
		return 0;
	}

	if (spin->input_enable && spin->output_enable) {
		LOG_DEV_ERR(dev, "spin:%u:oen: invalid input-enable|output-enable cfg", spin->spin);
		return -EINVAL;
	}

	if (!(spin_cfg->cfg & SPIN_CFG_ETH_OEN)) {
		LOG_DEV_ERR(dev,
			    "spin:%u:oen input-enable|output-enable not supported", spin->spin);
		return -EINVAL;
	}

	reg32 = sys_read32(DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_ETH_MODE);
	reg32 &= ~pin;
	if (spin->input_enable) {
		reg32 |= pin;
	}
	sys_write32(reg32, DEVICE_MMIO_NAMED_GET(dev, pinctrl) + R_ETH_MODE);
	return 0;
}

static int rzg3s_pinctrl_spin(const struct device *dev, const struct pinctrl_soc_rzg3s_spin *spin)
{
	int ret;

	if (spin->spin >= PINCTRL_RZG3S_SPIN_LAST) {
		LOG_DEV_ERR(dev, "spin:%u: invalid group", spin->spin);
		return -EINVAL;
	}

	ret = rzg3s_pinctrl_spin_iolh_set(dev, spin);
	if (ret) {
		return ret;
	}

	ret = rzg3s_pinctrl_spin_ien_set(dev, spin);
	if (ret) {
		return ret;
	}

	return rzg3s_pinctrl_spin_oen_set(dev, spin);
}

static int rzg3s_pinctrl_configure_pin(const struct device *dev, const pinctrl_soc_pin_t *pin)
{
	struct rzg3s_pinctrl_data *data = DEV_DATA(dev);
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&data->lock);

	switch (pin->type) {
	case PINCTRL_RZG3S_TYPE_PINMUX:
		ret = rzg3s_pinctrl_pinmux(dev, &pin->pinmux);
		break;
	case PINCTRL_RZG3S_TYPE_SPIN:
		ret = rzg3s_pinctrl_pingrp(dev, &pin->grp);
		break;
	case PINCTRL_RZG3S_TYPE_GRP:
		ret = rzg3s_pinctrl_spin(dev, &pin->spin);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	k_spin_unlock(&data->lock, key);

	return ret;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	int ret = 0;

	ARG_UNUSED(reg);
	while (pin_cnt-- > 0U) {
		ret = rzg3s_pinctrl_configure_pin(dev, pins++);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

static int rzg3s_pinctrl_init(const struct device *dev)
{
	DEVICE_MMIO_NAMED_MAP(dev, pinctrl, K_MEM_CACHE_NONE);
	return 0;
}

#define RZG3S_PINCTRL_INIT(inst)                                                                   \
	static struct rzg3s_pinctrl_data rzg3s_pinctrl_##inst##_data = {};                         \
                                                                                                   \
	static const struct rzg3s_pinctrl_cfg rzg3s_pinctrl_##inst##_cfg = {                       \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(pinctrl, DT_DRV_INST(inst)),                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, rzg3s_pinctrl_init, NULL, &rzg3s_pinctrl_##inst##_data,        \
			      &rzg3s_pinctrl_##inst##_cfg, PRE_KERNEL_1,                           \
			      CONFIG_PINCTRL_RZG3S_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RZG3S_PINCTRL_INIT);
