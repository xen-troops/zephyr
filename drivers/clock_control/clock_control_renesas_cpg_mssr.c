/*
 * Copyright (c) 2020-2022 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/renesas_cpg_mssr.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include "clock_control_renesas_cpg_mssr.h"

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_rcar);

static void rcar_cpg_reset(uint32_t base_address, uint32_t reg, uint32_t bit)
{
	rcar_cpg_write(base_address, srcr[reg], BIT(bit));
	rcar_cpg_write(base_address, SRSTCLR(reg), BIT(bit));
}

void rcar_cpg_write(uint32_t base_address, uint32_t reg, uint32_t val)
{
	sys_write32(~val, base_address + CPGWPR);
	sys_write32(val, base_address + reg);
	/* Wait for at least one cycle of the RCLK clock (@ ca. 32 kHz) */
	k_sleep(K_USEC(35));
}

int rcar_cpg_mstp_clock_endisable(uint32_t base_address, uint32_t module, bool enable)
{
	uint32_t reg = module / 100;
	uint32_t bit = module % 100;
	uint32_t bitmask = BIT(bit);
	uint32_t reg_val;
	unsigned int key;

	__ASSERT((bit < 32) && reg < ARRAY_SIZE(mstpcr), "Invalid module number for cpg clock: %d",
		 module);

	key = irq_lock();

	reg_val = sys_read32(base_address + mstpcr[reg]);
	if (enable) {
		reg_val &= ~bitmask;
	} else {
		reg_val |= bitmask;
	}

	sys_write32(reg_val, base_address + mstpcr[reg]);
	if (!enable) {
		rcar_cpg_reset(base_address, reg, bit);
	}

	irq_unlock(key);

	return 0;
}

struct cpg_clk_info_table *
rcar_cpg_find_clk_info_by_module_id(const struct device *dev, uint32_t domain, uint32_t id)
{
	struct rcar_cpg_mssr_data *data = dev->data;
	uint32_t left = 0, right = data->clk_info_table_size[domain] - 1;

	while (left <= right) {
		uint32_t middle = left + ((right - left) >> 1);
		struct cpg_clk_info_table *item = &data->clk_info_table[domain][middle];
		uint32_t module = item->module;

		if (module == id) {
			return item;
		}

		if (module < id) {
			left = middle + 1;
		} else {
			right = middle - 1;
		}
	}

	LOG_ERR("%s: can't find clk info (domain %u module %u)", dev->name, domain, id);
	return NULL;
}

static uint32_t rcar_cpg_get_divider(const struct device *dev, struct cpg_clk_info_table *clk_info)
{
	mem_addr_t reg_addr;
	mm_reg_t reg_val;
	uint32_t divider = RCAR_CPG_NONE;
	struct rcar_cpg_mssr_data *data = dev->data;

	if (clk_info->domain == CPG_MOD) {
		return 1;
	}

	reg_addr = clk_info->offset;
	if (reg_addr == RCAR_CPG_NONE) {
		return divider;
	}

	reg_addr += data->base_addr;
	reg_val = sys_read32(reg_addr);

	if (data->get_div_helper) {
		divider = data->get_div_helper(reg_val, clk_info->module);
	}

	if (!divider) {
		return RCAR_CPG_NONE;
	}

	return divider;
}

static int rcar_cpg_update_out_freq(const struct device *dev, struct cpg_clk_info_table *clk_info)
{
	uint32_t divider = rcar_cpg_get_divider(dev, clk_info);

	if (divider == RCAR_CPG_NONE) {
		return -EINVAL;
	}

	clk_info->out_freq = clk_info->in_freq / divider;
	return 0;
}

static int rcar_cpg_get_in_update_out_freq(const struct device *dev,
					   struct cpg_clk_info_table *clk_info)
{
	int freq = -ENOTSUP;
	struct cpg_clk_info_table *parent_clk;

	if (!clk_info) {
		return freq;
	}

	if (clk_info->in_freq != RCAR_CPG_NONE) {
		if (clk_info->out_freq == RCAR_CPG_NONE) {
			if (rcar_cpg_update_out_freq(dev, clk_info) < 0) {
				return freq;
			}
		}
		return clk_info->in_freq;
	}

	parent_clk = clk_info->parent;

	freq = rcar_cpg_get_in_update_out_freq(dev, parent_clk);
	if (freq < 0) {
		return freq;
	}

	clk_info->in_freq = parent_clk->out_freq;

	freq = rcar_cpg_update_out_freq(dev, clk_info);
	if (freq < 0) {
		return freq;
	}

	return clk_info->in_freq;
}

static int rcar_cpg_get_out_freq(const struct device *dev, struct cpg_clk_info_table *clk_info)
{
	int freq;

	if (clk_info->out_freq != RCAR_CPG_NONE) {
		return clk_info->out_freq;
	}

	freq = rcar_cpg_get_in_update_out_freq(dev, clk_info->parent);
	if (freq < 0) {
		return freq;
	}

	return clk_info->out_freq;
}

bool rcar_cpg_is_pwr_of_two(uint32_t val)
{
	return (val & (val - 1)) ? false : true;
}

static void rcar_cpg_change_children_in_out_freq(const struct device *dev,
						 struct cpg_clk_info_table *parent)
{
	struct cpg_clk_info_table *children_list = parent->children_list;

	while (children_list) {
		children_list->in_freq = parent->out_freq;

		if (rcar_cpg_update_out_freq(dev, children_list) < 0) {
			__ASSERT_NO_MSG(0);
			return;
		}

		/* child can have childrens */
		rcar_cpg_change_children_in_out_freq(dev, children_list);
		children_list = children_list->next_sibling;
	}
}

int rcar_cpg_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	int ret;
	struct rcar_cpg_mssr_data *data = dev->data;
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *)sys;
	k_spinlock_key_t key;

	struct cpg_clk_info_table *clk_info;

	if (!dev || !sys || !rate) {
		LOG_ERR("%s: received null ptr input arg(s) dev %p sys %p rate %p",
			__func__, dev, sys, rate);
		return -EINVAL;
	}

	clk_info = rcar_cpg_find_clk_info_by_module_id(dev, clk->domain, clk->module);
	if (clk_info == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	ret = rcar_cpg_get_out_freq(dev, clk_info);
	k_spin_unlock(&data->lock, key);

	if (ret < 0) {
		return ret;
	}

	*rate = ret;
	return 0;
}

int rcar_cpg_set_rate(const struct device *dev, clock_control_subsys_t sys,
		      clock_control_subsys_rate_t rate)
{
	int ret = -ENOTSUP;
	k_spinlock_key_t key;
	struct cpg_clk_info_table *clk_info;
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *)sys;
	struct rcar_cpg_mssr_data *data = dev->data;
	int in_freq;
	uint32_t divider;
	uint32_t div_mask;
	uint32_t module;
	uintptr_t u_rate = (uintptr_t)rate;

	if (!dev || !sys || !rate) {
		LOG_ERR("%s: received null ptr input arg(s) dev %p sys %p rate %p",
			__func__, dev, sys, rate);
		return -EINVAL;
	}

	clk_info = rcar_cpg_find_clk_info_by_module_id(dev, clk->domain, clk->module);
	if (clk_info == NULL) {
		return -EINVAL;
	}

	if (clk_info->domain == CPG_MOD) {
		__ASSERT_NO_MSG(clk_info->parent);
		clk_info = clk_info->parent;
	}

	module = clk_info->module;

	key = k_spin_lock(&data->lock);
	in_freq = rcar_cpg_get_in_update_out_freq(dev, clk_info);
	if (in_freq < 0) {
		ret =  in_freq;
		goto unlock;
	}

	if (u_rate == 0) {
		ret = -EINVAL;
		goto unlock;
	}

	divider = in_freq / u_rate;
	if (divider * u_rate != in_freq) {
		ret = -EINVAL;
		goto unlock;
	}

	if (!data->set_rate_helper) {
		ret = -ENOTSUP;
		goto unlock;
	}

	ret = data->set_rate_helper(module, &divider, &div_mask);

	if (ret < 0) {
		uint32_t reg = sys_read32(clk_info->offset + data->base_addr);

		reg &= ~div_mask;
		rcar_cpg_write(data->base_addr, clk_info->offset, reg | divider);
		clk_info->out_freq = u_rate;

		rcar_cpg_change_children_in_out_freq(dev, clk_info);
	}

unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}

void rcar_cpg_build_clock_relationship(const struct device *dev)
{
	uint32_t domain;
	k_spinlock_key_t key;
	struct rcar_cpg_mssr_data *data = dev->data;

	if (!data) {
		return;
	}

	key = k_spin_lock(&data->lock);
	for (domain = 0; domain < 2; domain++) {
		uint32_t idx;
		uint32_t prev_mod_id = 0;
		struct cpg_clk_info_table *item = data->clk_info_table[domain];

		for (idx = 0; idx < data->clk_info_table_size[domain]; idx++, item++) {
			struct cpg_clk_info_table *parent;

			/* check if an array is sorted by module id or not */
			__ASSERT_NO_MSG(prev_mod_id < item->module);
			prev_mod_id = item->module;

			if (item->parent_id == RCAR_CPG_NONE) {
				continue;
			}

			parent = rcar_cpg_find_clk_info_by_module_id(dev, CPG_CORE,
								     item->parent_id);
			__ASSERT_NO_MSG(parent);

			__ASSERT_NO_MSG(item->parent == NULL);
			item->parent = parent;

			/* insert in the head of the children list of the parent */
			item->next_sibling = parent->children_list;
			parent->children_list = item;
		}
	}
	k_spin_unlock(&data->lock, key);
}

void rcar_cpg_update_all_in_out_freq(const struct device *dev)
{
	uint32_t domain;
	k_spinlock_key_t key;
	struct rcar_cpg_mssr_data *data = dev->data;

	if (!data) {
		return;
	}

	key = k_spin_lock(&data->lock);
	for (domain = 0; domain < 2; domain++) {
		uint32_t idx;
		struct cpg_clk_info_table *item = data->clk_info_table[domain];

		for (idx = 0; idx < data->clk_info_table_size[domain]; idx++, item++) {
			rcar_cpg_get_in_update_out_freq(dev, item);
		}
	}
	k_spin_unlock(&data->lock, key);
}
