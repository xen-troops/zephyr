/*
 * Copyright (c) 2023 EPAM Systems
 *
 * r8a779f Clock Pulse Generator / Module Standby and Software Reset
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r8a779f_cpg_mssr

#include <errno.h>
#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/r8a779f_cpg_mssr.h>
#include <zephyr/irq.h>
#include "clock_control_renesas_cpg_mssr.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(clock_control_rcar);

#define R8A779F_CLK_SD_STOP_BIT BIT(8)
#define R8A779F_CLK_SD_DIV_MASK 0x3
#define R8A779F_CLK_SD_DIV_SHIFT 0

#define R8A779F_CLK_SDH_STOP_BIT BIT(9)
#define R8A779F_CLK_SDH_DIV_MASK 0x7
#define R8A779F_CLK_SDH_DIV_SHIFT 2

#define R8A779F_CLK_SD0CKCR1_DIV_MASK 0x3
#define R8A779F_CLK_SD0CKCR1_DIV_SHIFT 29

struct r8a779f_cpg_mssr_cfg {
	mm_reg_t base_address;
};

struct r8a779f_cpg_mssr_data {
	struct rcar_cpg_mssr_data cmn;
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table core_props[] = {
	RCAR_CORE_CLK_INFO_ITEM(R8A779F_CLK_PLL5VCO, RCAR_CPG_NONE,
				RCAR_CPG_NONE, RCAR_CPG_MHZ(3200)),

	RCAR_CORE_CLK_INFO_ITEM(R8A779F_CLK_SD0CKCR1, 0x08A4, R8A779F_CLK_PLL5VCO, RCAR_CPG_NONE),
	RCAR_CORE_CLK_INFO_ITEM(R8A779F_CLK_SDH, 0x0870, R8A779F_CLK_SD0CKCR1, RCAR_CPG_NONE),
	RCAR_CORE_CLK_INFO_ITEM(R8A779F_CLK_SD, 0x0870, R8A779F_CLK_SDH, RCAR_CPG_NONE),
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table mod_props[] = {
	RCAR_MOD_CLK_INFO_ITEM(706, R8A779F_CLK_SD),
};


static int r8a779f_cpg_enable_disable_core(const struct r8a779f_cpg_mssr_cfg *cfg,
					   struct cpg_clk_info_table *clk_info,
					   uint32_t enable)
{
	int ret = 0;
	uint32_t reg;

	enable = !!enable;

	switch (clk_info->module) {
	case R8A779F_CLK_SD0CKCR1:
		return 0;
	case R8A779F_CLK_SD:
		reg = sys_read32(cfg->base_address + clk_info->offset);
		reg &= ~R8A779F_CLK_SD_STOP_BIT;
		reg |= !enable * R8A779F_CLK_SD_STOP_BIT;
		break;
	case R8A779F_CLK_SDH:
		reg = sys_read32(cfg->base_address + clk_info->offset);
		reg &= ~R8A779F_CLK_SDH_STOP_BIT;
		reg |= !enable * R8A779F_CLK_SDH_STOP_BIT;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	if (!ret) {
		rcar_cpg_write(cfg->base_address, clk_info->offset, reg);
	}
	return ret;
}

static int r8a779f_cpg_core_clock_endisable(const struct device *dev,
					    struct rcar_cpg_clk *clk,
					    bool enable)
{
	struct cpg_clk_info_table *clk_info;
	const struct r8a779f_cpg_mssr_cfg *cfg = dev->config;
	struct r8a779f_cpg_mssr_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	clk_info = rcar_cpg_find_clk_info_by_module_id(dev, clk->domain, clk->module);
	if (!clk_info) {
		return -EINVAL;
	}

	if (enable) {
		if ((int)clk->rate > 0) {
			uintptr_t rate = clk->rate;

			ret = rcar_cpg_set_rate(dev, (clock_control_subsys_t)clk,
						(clock_control_subsys_rate_t)rate);
			if (ret < 0) {
				return ret;
			}
		}
	}

	key = k_spin_lock(&data->cmn.lock);
	r8a779f_cpg_enable_disable_core(cfg, clk_info, enable);
	k_spin_unlock(&data->cmn.lock, key);

	return ret;
}

int r8a779f_cpg_mssr_start_stop(const struct device *dev,
	clock_control_subsys_t sys, bool enable)
{
	const struct r8a779f_cpg_mssr_cfg *config = dev->config;
	struct r8a779f_cpg_mssr_data *data = dev->data;
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *)sys;
	int ret = -EINVAL;
	k_spinlock_key_t key;

	if (clk->domain == CPG_CORE) {
		return r8a779f_cpg_core_clock_endisable(dev, clk, enable);
	}

	key = k_spin_lock(&data->cmn.lock);
	ret = rcar_cpg_mstp_clock_endisable(config->base_address, clk->module, enable);
	k_spin_unlock(&data->cmn.lock, key);
	return ret;
}

static int r8a779f_cpg_mssr_start(const struct device *dev,
				  clock_control_subsys_t sys)
{
	return r8a779f_cpg_mssr_start_stop(dev, sys, true);
}

static int r8a779f_cpg_mssr_stop(const struct device *dev,
				 clock_control_subsys_t sys)
{
	return r8a779f_cpg_mssr_start_stop(dev, sys, false);
}

static int r8a779f_cpg_mssr_init(const struct device *dev)
{
	rcar_cpg_build_clock_relationship(dev);
	rcar_cpg_update_all_in_out_freq(dev);
	return 0;
}

static uint32_t r8a779f_get_div_helper(uint32_t reg_val, uint32_t module)
{
	uint32_t divider = RCAR_CPG_NONE;

	switch (module) {
	case R8A779F_CLK_SD0CKCR1:
		reg_val >>= R8A779F_CLK_SD0CKCR1_DIV_SHIFT;
		reg_val &= R8A779F_CLK_SD0CKCR1_DIV_MASK;
		if (reg_val < 3) {
			divider = reg_val + 4;
		}
		break;
	case R8A779F_CLK_SDH:
		reg_val >>= R8A779F_CLK_SDH_DIV_SHIFT;
		if ((reg_val & R8A779F_CLK_SDH_DIV_MASK) < 5) {
			divider = 1 << (reg_val & R8A779F_CLK_SDH_DIV_MASK);
		}
		break;
	case R8A779F_CLK_SD:
		divider = 1 << ((reg_val & R8A779F_CLK_SD_DIV_MASK) + 1);
		break;
	default:
		break;
	}

	return divider;
}

static int r8a779f_set_rate_helper(uint32_t module, uint32_t *divider, uint32_t *div_mask)
{
	int ret = -ENOTSUP;

	switch (module) {
	case R8A779F_CLK_SD0CKCR1:
		if (*divider > 3 && *divider < 7) {
			*divider -= 4;
			*divider <<= R8A779F_CLK_SD0CKCR1_DIV_SHIFT;
			*div_mask = R8A779F_CLK_SD0CKCR1_DIV_MASK <<
				    R8A779F_CLK_SD0CKCR1_DIV_SHIFT;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	case R8A779F_CLK_SD:
		if (rcar_cpg_is_pwr_of_two(*divider) && *divider <= 4) {
			*divider >>= 2;
			*div_mask = R8A779F_CLK_SD_DIV_MASK << R8A779F_CLK_SD_DIV_SHIFT;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	case R8A779F_CLK_SDH:
		if (!rcar_cpg_is_pwr_of_two(*divider) || *divider > 16) {
			ret = -EINVAL;
			break;
		}
		ret = 0;
		*divider = (find_lsb_set(*divider) - 1) << R8A779F_CLK_SDH_DIV_SHIFT;
		*div_mask = R8A779F_CLK_SDH_DIV_MASK << R8A779F_CLK_SDH_DIV_SHIFT;
		break;
	default:
		break;
	}

	return ret;
}

/* TODO: add at least get/set rate */
static const struct clock_control_driver_api r8a779f_cpg_mssr_api = {
	.on = r8a779f_cpg_mssr_start,
	.off = r8a779f_cpg_mssr_stop,
	.get_rate = rcar_cpg_get_rate,
	.set_rate = rcar_cpg_set_rate,
};

#define R8A779F_MSSR_INIT(inst)							\
	static struct r8a779f_cpg_mssr_cfg cpg_mssr##inst##_cfg = {		\
		.base_address = DT_INST_REG_ADDR(inst)				\
	};									\
										\
	static struct r8a779f_cpg_mssr_data cpg_mssr##inst##_data = {		\
		.cmn.base_addr = DT_INST_REG_ADDR(inst),			\
		.cmn.clk_info_table[CPG_CORE] = core_props,			\
		.cmn.clk_info_table_size[CPG_CORE] = ARRAY_SIZE(core_props),	\
		.cmn.clk_info_table[CPG_MOD] = mod_props,			\
		.cmn.clk_info_table_size[CPG_MOD] = ARRAY_SIZE(mod_props),	\
		.cmn.get_div_helper = r8a779f_get_div_helper,			\
		.cmn.set_rate_helper = r8a779f_set_rate_helper			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			      &r8a779f_cpg_mssr_init,				\
			      NULL,						\
			      &cpg_mssr##inst##_data,				\
			      &cpg_mssr##inst##_cfg,				\
			      PRE_KERNEL_1,					\
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,		\
			      &r8a779f_cpg_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(R8A779F_MSSR_INIT)
