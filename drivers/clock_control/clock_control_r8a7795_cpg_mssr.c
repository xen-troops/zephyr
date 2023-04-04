/*
 * Copyright (c) 2022 IoT.bzh
 *
 * r8a7795 Clock Pulse Generator / Module Standby and Software Reset
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r8a7795_cpg_mssr

#include <errno.h>
#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/r8a7795_cpg_mssr.h>
#include <zephyr/irq.h>
#include "clock_control_renesas_cpg_mssr.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(clock_control_rcar);

#define R8A7795_CLK_SD_STOP_BIT BIT(8)
#define R8A7795_CLK_SD_DIV_MASK 0x3
#define R8A7795_CLK_SD_DIV_SHIFT 0

#define R8A7795_CLK_SDH_STOP_BIT BIT(9)
#define R8A7795_CLK_SDH_DIV_MASK 0x7
#define R8A7795_CLK_SDH_DIV_SHIFT 2

#define R8A7795_CLK_CANFD_STOP_BIT BIT(8)
#define R8A7795_CLK_CANFD_DIV_MASK 0x3f

struct r8a7795_cpg_mssr_config {
	mm_reg_t base_address;
};

struct r8a7795_cpg_mssr_data {
	struct rcar_cpg_mssr_data cmn;
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table core_props[] = {
	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD0H, 0x0074, RCAR_CPG_NONE, RCAR_CPG_MHZ(800)),
	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD0, 0x0074, R8A7795_CLK_SD0H, RCAR_CPG_MHZ(800)),

	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD1H, 0x0078, RCAR_CPG_NONE, RCAR_CPG_MHZ(800)),
	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD1, 0x0078, R8A7795_CLK_SD1H, RCAR_CPG_MHZ(800)),

	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD2H, 0x0268, RCAR_CPG_NONE, RCAR_CPG_MHZ(800)),
	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD2, 0x0268, R8A7795_CLK_SD2H, RCAR_CPG_MHZ(800)),

	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD3H, 0x026C, RCAR_CPG_NONE, RCAR_CPG_MHZ(800)),
	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_SD3, 0x026C, R8A7795_CLK_SD3H, RCAR_CPG_MHZ(800)),

	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_CANFD, 0x0244, RCAR_CPG_NONE, RCAR_CPG_MHZ(800)),

	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_S3D4, RCAR_CPG_NONE,
				RCAR_CPG_NONE, RCAR_CPG_KHZ(66600)),

	RCAR_CORE_CLK_INFO_ITEM(R8A7795_CLK_S0D12, RCAR_CPG_NONE,
				RCAR_CPG_NONE, RCAR_CPG_KHZ(66600)),
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table mod_props[] = {
	RCAR_MOD_CLK_INFO_ITEM(311, R8A7795_CLK_SD3),
	RCAR_MOD_CLK_INFO_ITEM(312, R8A7795_CLK_SD2),
	RCAR_MOD_CLK_INFO_ITEM(313, R8A7795_CLK_SD1),
	RCAR_MOD_CLK_INFO_ITEM(314, R8A7795_CLK_SD0),
};

static int r8a7795_cpg_enable_disable_core(const struct r8a7795_cpg_mssr_config *cfg,
					struct cpg_clk_info_table *clk_info,
					uint32_t enable)
{
	int ret = 0;
	uint32_t reg;

	enable = !!enable;

	switch (clk_info->module) {
	case R8A7795_CLK_SD0:
	case R8A7795_CLK_SD1:
	case R8A7795_CLK_SD2:
	case R8A7795_CLK_SD3:
		reg = sys_read32(cfg->base_address + clk_info->offset);
		reg &= ~R8A7795_CLK_SD_STOP_BIT;
		reg |= !enable * R8A7795_CLK_SD_STOP_BIT;
		break;
	case R8A7795_CLK_SD0H:
	case R8A7795_CLK_SD1H:
	case R8A7795_CLK_SD2H:
	case R8A7795_CLK_SD3H:
		reg = sys_read32(cfg->base_address + clk_info->offset);
		reg &= ~R8A7795_CLK_SDH_STOP_BIT;
		reg |= !enable * R8A7795_CLK_SDH_STOP_BIT;
		break;
	case R8A7795_CLK_CANFD:
		reg = sys_read32(cfg->base_address + clk_info->offset);
		reg &= ~R8A7795_CLK_CANFD_STOP_BIT;
		reg |= !enable * R8A7795_CLK_CANFD_STOP_BIT;
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

static int r8a7795_cpg_core_clock_endisable(const struct device *dev,
					    struct rcar_cpg_clk *clk,
					    bool enable)
{
	struct cpg_clk_info_table *clk_info;
	const struct r8a7795_cpg_mssr_config *cfg = dev->config;
	struct r8a7795_cpg_mssr_data *data = dev->data;
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
	r8a7795_cpg_enable_disable_core(cfg, clk_info, enable);
	k_spin_unlock(&data->cmn.lock, key);

	return ret;
}

static int r8a7795_cpg_mssr_start_stop(const struct device *dev,
				       clock_control_subsys_t sys,
				       bool enable)
{
	const struct r8a7795_cpg_mssr_config *config = dev->config;
	struct r8a7795_cpg_mssr_data *data = dev->data;
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *)sys;
	int ret = -EINVAL;

	if (clk->domain == CPG_MOD) {
		k_spinlock_key_t key;

		key = k_spin_lock(&data->cmn.lock);
		ret = rcar_cpg_mstp_clock_endisable(config->base_address, clk->module, enable);
		k_spin_unlock(&data->cmn.lock, key);
	} else if (clk->domain == CPG_CORE) {
		ret = r8a7795_cpg_core_clock_endisable(dev, clk, enable);
	}

	return ret;
}

static uint32_t r8a7795_get_div_helper(uint32_t reg_val, uint32_t module)
{
	uint32_t divider = RCAR_CPG_NONE;

	switch (module) {
	case R8A7795_CLK_SD0H:
	case R8A7795_CLK_SD1H:
	case R8A7795_CLK_SD2H:
	case R8A7795_CLK_SD3H:
		reg_val >>= R8A7795_CLK_SDH_DIV_SHIFT;
		if ((reg_val & R8A7795_CLK_SDH_DIV_MASK) < 5) {
			divider = 1 << (reg_val & R8A7795_CLK_SDH_DIV_MASK);
		}
		break;
	case R8A7795_CLK_SD0:
	case R8A7795_CLK_SD1:
	case R8A7795_CLK_SD2:
	case R8A7795_CLK_SD3:
		divider = 1 << ((reg_val & R8A7795_CLK_SD_DIV_MASK) + 1);
		break;
	case R8A7795_CLK_CANFD:
		divider = (reg_val & R8A7795_CLK_CANFD_DIV_MASK) + 1;
		break;
	case R8A7795_CLK_S3D4:
	case R8A7795_CLK_S0D12:
		divider = 1;
		break;
	default:
		break;
	}

	return divider;
}

static int r8a7795_set_rate_helper(uint32_t module, uint32_t *divider, uint32_t *div_mask)
{
	int ret = -ENOTSUP;

	switch (module) {
	case R8A7795_CLK_SD0:
	case R8A7795_CLK_SD1:
	case R8A7795_CLK_SD2:
	case R8A7795_CLK_SD3:
		if (rcar_cpg_is_pwr_of_two(*divider) && *divider <= 4) {
			*divider >>= 2;
			*div_mask = R8A7795_CLK_SD_DIV_MASK << R8A7795_CLK_SD_DIV_SHIFT;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	case R8A7795_CLK_SD0H:
	case R8A7795_CLK_SD1H:
	case R8A7795_CLK_SD2H:
	case R8A7795_CLK_SD3H:
		if (!rcar_cpg_is_pwr_of_two(*divider) || *divider > 16) {
			ret = -EINVAL;
			break;
		}
		ret = 0;
		*divider = (find_lsb_set(*divider) - 1) << R8A7795_CLK_SDH_DIV_SHIFT;
		*div_mask = R8A7795_CLK_SDH_DIV_MASK << R8A7795_CLK_SDH_DIV_SHIFT;
		break;
	case R8A7795_CLK_CANFD:
		*divider -= 1;
		if (*divider <= R8A7795_CLK_CANFD_DIV_MASK) {
			ret = 0;
			*div_mask = R8A7795_CLK_CANFD_DIV_MASK;
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		break;
	}

	return ret;
}

static int r8a7795_cpg_mssr_start(const struct device *dev,
				  clock_control_subsys_t sys)
{
	return r8a7795_cpg_mssr_start_stop(dev, sys, true);
}

static int r8a7795_cpg_mssr_stop(const struct device *dev,
				 clock_control_subsys_t sys)
{
	return r8a7795_cpg_mssr_start_stop(dev, sys, false);
}

static int r8a7795_cpg_mssr_init(const struct device *dev)
{
	rcar_cpg_build_clock_relationship(dev);
	rcar_cpg_update_all_in_out_freq(dev);
	return 0;
}

static const struct clock_control_driver_api r8a7795_cpg_mssr_api = {
	.on = r8a7795_cpg_mssr_start,
	.off = r8a7795_cpg_mssr_stop,
	.get_rate = rcar_cpg_get_rate,
	.set_rate = rcar_cpg_set_rate,
};

#define R8A7795_MSSR_INIT(inst)							  \
	static struct r8a7795_cpg_mssr_config r8a7795_cpg_mssr##inst##_config = { \
		.base_address = DT_INST_REG_ADDR(inst),				  \
	};									  \
										  \
	static struct r8a7795_cpg_mssr_data r8a7795_cpg_mssr##inst##_data = {	  \
		.cmn.base_addr = DT_INST_REG_ADDR(inst),			  \
		.cmn.clk_info_table[CPG_CORE] = core_props,			  \
		.cmn.clk_info_table_size[CPG_CORE] = ARRAY_SIZE(core_props),	  \
		.cmn.clk_info_table[CPG_MOD] = mod_props,			  \
		.cmn.clk_info_table_size[CPG_MOD] = ARRAY_SIZE(mod_props),	  \
		.cmn.get_div_helper = r8a7795_get_div_helper,			  \
		.cmn.set_rate_helper = r8a7795_set_rate_helper			  \
	};									  \
										  \
	DEVICE_DT_INST_DEFINE(inst,						  \
			      &r8a7795_cpg_mssr_init,				  \
			      NULL,						  \
			      &r8a7795_cpg_mssr##inst##_data,			  \
			      &r8a7795_cpg_mssr##inst##_config,			  \
			      PRE_KERNEL_1,					  \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,		  \
			      &r8a7795_cpg_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(R8A7795_MSSR_INIT)
