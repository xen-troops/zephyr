/*
 * Copyright (c) 2024 EPAM Systems
 *
 * Renesas RZ/G3S r9a08g045 CPG Reset controller
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r9a08g045_cpg_reset

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/clock/r9a08g045_cpg_mssr.h>
#include <zephyr/drivers/reset.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(reset_rzg3s, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

#define LOG_DEV_ERR(dev, format, ...)	LOG_ERR("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...)	LOG_WRN("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...)	LOG_INF("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...)	LOG_DBG("%s:"#format, (dev)->name, ##__VA_ARGS__)

#define MON_REG_WAIT_US 50

#define RST_MON_R(reg)		(0x180 + (reg))

struct rzg3s_cpg_reset {
	uint16_t off;
	uint16_t bit;
};

#define DEF_RST(_id, _off, _bit)	\
	[_id] = { \
		.off = (_off), \
		.bit = (_bit) \
	}

static const struct rzg3s_cpg_reset rzg3s_cpg_resets[R9A08G045_LST_RESETN] = {
	DEF_RST(R9A08G045_GIC600_GICRESET_N, 0x814, 0),
	DEF_RST(R9A08G045_GIC600_DBG_GICRESET_N, 0x814, 1),
	DEF_RST(R9A08G045_IA55_RESETN, 0x818, 0),
	DEF_RST(R9A08G045_DMAC_ARESETN, 0x82c, 0),
	DEF_RST(R9A08G045_DMAC_RST_ASYNC, 0x82c, 1),
	DEF_RST(R9A08G045_OSTM0_PRESETZ, 0x834, 0),
	DEF_RST(R9A08G045_OSTM1_PRESETZ, 0x834, 1),
	DEF_RST(R9A08G045_OSTM2_PRESETZ, 0x834, 2),
	DEF_RST(R9A08G045_OSTM3_PRESETZ, 0x834, 3),
	DEF_RST(R9A08G045_OSTM4_PRESETZ, 0x834, 4),
	DEF_RST(R9A08G045_OSTM5_PRESETZ, 0x834, 5),
	DEF_RST(R9A08G045_OSTM6_PRESETZ, 0x834, 6),
	DEF_RST(R9A08G045_OSTM7_PRESETZ, 0x834, 7),
	DEF_RST(R9A08G045_MTU_X_PRESET_MTU3, 0x838, 0),
	DEF_RST(R9A08G045_POE3_RST_M_REG, 0x83c, 0),
	DEF_RST(R9A08G045_GPT_RST_C, 0x840, 0),
	DEF_RST(R9A08G045_POEG_A_RST, 0x844, 0),
	DEF_RST(R9A08G045_POEG_B_RST, 0x844, 1),
	DEF_RST(R9A08G045_POEG_C_RST, 0x844, 2),
	DEF_RST(R9A08G045_POEG_D_RST, 0x844, 3),
	DEF_RST(R9A08G045_WDT0_PRESETN, 0x848, 0),
	DEF_RST(R9A08G045_SPI_HRESETN, 0x850, 0),
	DEF_RST(R9A08G045_SPI_ARESETN, 0x850, 1),
	DEF_RST(R9A08G045_SDHI0_IXRST, 0x854, 0),
	DEF_RST(R9A08G045_SDHI1_IXRST, 0x854, 1),
	DEF_RST(R9A08G045_SDHI2_IXRST, 0x854, 2),
	DEF_RST(R9A08G045_SSI0_RST_M2_REG, 0x870, 0),
	DEF_RST(R9A08G045_SSI1_RST_M2_REG, 0x870, 1),
	DEF_RST(R9A08G045_SSI2_RST_M2_REG, 0x870, 2),
	DEF_RST(R9A08G045_SSI3_RST_M2_REG, 0x870, 3),
	DEF_RST(R9A08G045_SRC_RST, 0x874, 0),
	DEF_RST(R9A08G045_USB_U2H0_HRESETN, 0x878, 0),
	DEF_RST(R9A08G045_USB_U2H1_HRESETN, 0x878, 1),
	DEF_RST(R9A08G045_USB_U2P_EXL_SYSRST, 0x878, 2),
	DEF_RST(R9A08G045_USB_PRESETN, 0x878, 3),
	DEF_RST(R9A08G045_ETH0_RST_HW_N, 0x87c, 0),
	DEF_RST(R9A08G045_ETH1_RST_HW_N, 0x87c, 1),
	DEF_RST(R9A08G045_I2C0_MRST, 0x880, 0),
	DEF_RST(R9A08G045_I2C1_MRST, 0x880, 1),
	DEF_RST(R9A08G045_I2C2_MRST, 0x880, 2),
	DEF_RST(R9A08G045_I2C3_MRST, 0x880, 3),
	DEF_RST(R9A08G045_SCIF0_RST_SYSTEM_N, 0x884, 0),
	DEF_RST(R9A08G045_SCIF1_RST_SYSTEM_N, 0x884, 1),
	DEF_RST(R9A08G045_SCIF2_RST_SYSTEM_N, 0x884, 2),
	DEF_RST(R9A08G045_SCIF3_RST_SYSTEM_N, 0x884, 3),
	DEF_RST(R9A08G045_SCIF4_RST_SYSTEM_N, 0x884, 4),
	DEF_RST(R9A08G045_SCIF5_RST_SYSTEM_N, 0x884, 5),
	DEF_RST(R9A08G045_SCI0_RST, 0x888, 0),
	DEF_RST(R9A08G045_SCI1_RST, 0x888, 1),
	DEF_RST(R9A08G045_RSPI0_RST, 0x890, 0),
	DEF_RST(R9A08G045_RSPI1_RST, 0x890, 1),
	DEF_RST(R9A08G045_RSPI2_RST, 0x890, 2),
	DEF_RST(R9A08G045_RSPI3_RST, 0x890, 3),
	DEF_RST(R9A08G045_RSPI4_RST, 0x890, 4),
	DEF_RST(R9A08G045_CANFD_RSTP_N, 0x894, 0),
	DEF_RST(R9A08G045_CANFD_RSTC_N, 0x894, 1),
	DEF_RST(R9A08G045_GPIO_RSTN, 0x898, 0),
	DEF_RST(R9A08G045_GPIO_PORT_RESETN, 0x898, 1),
	DEF_RST(R9A08G045_GPIO_SPARE_RESETN, 0x898, 2),
	DEF_RST(R9A08G045_ADC_PRESETN, 0x8a8, 0),
	DEF_RST(R9A08G045_ADC_ADRST_N, 0x8a8, 1),
	DEF_RST(R9A08G045_TSU_PRESETN, 0x8ac, 0),
	DEF_RST(R9A08G045_OCTA_ARESETN, 0x8f4, 0),
	DEF_RST(R9A08G045_PDM0_PRESETNT, 0x904, 0),
	DEF_RST(R9A08G045_PCI_ARESETN, 0x908, 0),
	DEF_RST(R9A08G045_PCI_RST_B, 0x908, 1),
	DEF_RST(R9A08G045_PCI_RST_GP_B, 0x908, 2),
	DEF_RST(R9A08G045_PCI_RST_PS_B, 0x908, 3),
	DEF_RST(R9A08G045_PCI_RST_RSM_B, 0x908, 4),
	DEF_RST(R9A08G045_PCI_RST_CFG_B, 0x908, 5),
	DEF_RST(R9A08G045_PCI_RST_LOAD_B, 0x908, 6),
	DEF_RST(R9A08G045_SPDIF_RST, 0x90c, 0),
	DEF_RST(R9A08G045_I3C_TRESETN, 0x910, 0),
	DEF_RST(R9A08G045_I3C_PRESETN, 0x910, 1),
	DEF_RST(R9A08G045_VBAT_BRESETN, 0x914, 0),
};

struct rzg3s_reset_cpg_config {
	const struct rzg3s_cpg_reset *mod_rsts;
	const struct device	*cpg_dev;
};

struct rzg3s_reset_cpg_data {
	mm_reg_t base_addr;

	struct k_spinlock lock;
};

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

static int rzg3s_reset_cpg_init(const struct device *dev)
{
	const struct rzg3s_reset_cpg_config *cfg = dev->config;
	struct rzg3s_reset_cpg_data *data = dev->data;

	/* Get IO addr from parent */
	data->base_addr = DEVICE_MMIO_GET(cfg->cpg_dev);

	return 0;
}

static int rzg3s_reset_cpg_checkpar(const struct device *dev, uint32_t id)
{
	if (!dev) {
		return -EINVAL;
	}

	if (id >= R9A08G045_LST_RESETN) {
		LOG_DEV_ERR(dev, "wrong module reset id %u", id);
		return -EINVAL;
	}

	return 0;
}

static int rzg3s_reset_cpg_doreset(const struct device *dev, uint32_t id)
{
	const struct rzg3s_reset_cpg_config *cfg = dev->config;
	struct rzg3s_reset_cpg_data *data = dev->data;
	const struct rzg3s_cpg_reset *mod_rst;
	k_spinlock_key_t key;
	mm_reg_t reg_addr;
	uint32_t val;
	int ret;

	ret = rzg3s_reset_cpg_checkpar(dev, id);
	if (ret) {
		return ret;
	}

	mod_rst = &cfg->mod_rsts[id];

	key = k_spin_lock(&data->lock);
	/* reset assert */
	reg_addr = data->base_addr + cfg->mod_rsts[id].off;
	val = BIT(cfg->mod_rsts[id].bit);
	rzg3s_cpg_change_reg_bits(reg_addr, val, false);

	/* monitor mod reset */
	reg_addr = data->base_addr + RST_MON_R(cfg->mod_rsts[id].off);
	ret = rzg3s_cpg_wait_bit_val(reg_addr, val, val, MON_REG_WAIT_US);
	if (!ret) {
		LOG_DEV_DBG(dev, "module id %u resetA tmo", id);
	}

	/* Wait for at least one cycle of the RCLK clock (@ ca. 32 kHz) */
	k_busy_wait(35);

	/* reset de-assert */
	reg_addr = data->base_addr + cfg->mod_rsts[id].off;
	val = BIT(cfg->mod_rsts[id].bit);
	rzg3s_cpg_change_reg_bits(reg_addr, val, true);

	/* monitor mod reset */
	reg_addr = data->base_addr + RST_MON_R(cfg->mod_rsts[id].off);
	ret = rzg3s_cpg_wait_bit_val(reg_addr, val, 0, MON_REG_WAIT_US);
	if (!ret) {
		LOG_DEV_DBG(dev, "module id %u resetD tmo", id);
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rzg3s_reset_cpg_assert(const struct device *dev, uint32_t id)
{
	const struct rzg3s_reset_cpg_config *cfg = dev->config;
	struct rzg3s_reset_cpg_data *data = dev->data;
	const struct rzg3s_cpg_reset *mod_rst;
	k_spinlock_key_t key;
	mm_reg_t reg_addr;
	uint32_t val;
	int ret;

	ret = rzg3s_reset_cpg_checkpar(dev, id);
	if (ret) {
		return ret;
	}

	mod_rst = &cfg->mod_rsts[id];

	key = k_spin_lock(&data->lock);
	/* reset assert */
	reg_addr = data->base_addr + mod_rst->off;
	val = BIT(mod_rst->bit);
	rzg3s_cpg_change_reg_bits(reg_addr, val, false);

	/* monitor mod reset */
	reg_addr = data->base_addr + RST_MON_R(mod_rst->off);
	ret = rzg3s_cpg_wait_bit_val(reg_addr, val, val, MON_REG_WAIT_US);
	if (!ret) {
		LOG_DEV_DBG(dev, "module id %u resetA tmo", id);
		ret = -EIO;
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rzg3s_reset_cpg_deassert(const struct device *dev, uint32_t id)
{
	const struct rzg3s_reset_cpg_config *cfg = dev->config;
	struct rzg3s_reset_cpg_data *data = dev->data;
	const struct rzg3s_cpg_reset *mod_rst;
	k_spinlock_key_t key;
	mm_reg_t reg_addr;
	uint32_t val;
	int ret;

	ret = rzg3s_reset_cpg_checkpar(dev, id);
	if (ret) {
		return ret;
	}

	mod_rst = &cfg->mod_rsts[id];

	key = k_spin_lock(&data->lock);
	/* reset de-assert */
	reg_addr = data->base_addr + mod_rst->off;
	val = BIT(mod_rst->bit);
	rzg3s_cpg_change_reg_bits(reg_addr, val, true);

	/* monitor mod reset */
	reg_addr = data->base_addr + RST_MON_R(mod_rst->off);
	ret = rzg3s_cpg_wait_bit_val(reg_addr, val, 0, MON_REG_WAIT_US);
	if (!ret) {
		LOG_DEV_DBG(dev, "module id %u resetD tmo", id);
		ret = -EIO;
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rzg3s_reset_cpg_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct rzg3s_reset_cpg_config *cfg = dev->config;
	struct rzg3s_reset_cpg_data *data = dev->data;
	const struct rzg3s_cpg_reset *mod_rst;
	mm_reg_t reg_addr;
	int ret;

	ret = rzg3s_reset_cpg_checkpar(dev, id);
	if (ret) {
		return ret;
	}

	mod_rst = &cfg->mod_rsts[id];

	/* monitor mod reset */
	reg_addr = data->base_addr + RST_MON_R(mod_rst->off);
	*status = !!(sys_read32(reg_addr) & BIT(mod_rst->bit));

	return 0;
}

static const struct reset_driver_api rzg3s_cpg_reset_driver_api = {
	.status = rzg3s_reset_cpg_status,
	.line_assert = rzg3s_reset_cpg_assert,
	.line_deassert = rzg3s_reset_cpg_deassert,
	.line_toggle = rzg3s_reset_cpg_doreset,
};

#define RZG3S_RESET_CPG_INIT(inst)                                                                 \
	static const struct rzg3s_reset_cpg_config rzg3s_reset_cpg_##inst##_cfg = {                \
		.mod_rsts = rzg3s_cpg_resets,                                                      \
		.cpg_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                    \
	};                                                                                         \
                                                                                                   \
	static struct rzg3s_reset_cpg_data rzg3s_reset_cpg_##inst##_data = {};                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, rzg3s_reset_cpg_init, NULL, &rzg3s_reset_cpg_##inst##_data,    \
			      &rzg3s_reset_cpg_##inst##_cfg, PRE_KERNEL_1,                         \
			      CONFIG_RESET_INIT_PRIORITY, &rzg3s_cpg_reset_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RZG3S_RESET_CPG_INIT)
