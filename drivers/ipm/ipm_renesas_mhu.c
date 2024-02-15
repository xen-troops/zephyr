/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ipm_renesas_mhu, CONFIG_IPM_LOG_LEVEL);

#define DT_DRV_COMPAT renesas_mhu

enum msg_type {
	MHU_SEND,
	MHU_RSP
};

#define MHU_MAX_ID_VAL 0

#define INT_STS_OFF (0x0)
#define INT_SET_OFF (0x4)
#define INT_CLR_OFF (0x8)

#define RSP_OFF (0x10)

#define MSG_REG(dev, off) (DEVICE_MMIO_GET(dev) + off)
#define RSP_REG(dev, off) (DEVICE_MMIO_GET(dev) + RSP_OFF + off)

struct mhu_config {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clk_dev;
	struct renesas_cpg_clk clk_mod;
	const struct reset_dt_spec reset;
	void (*irq_configure)(void);
	enum msg_type type;
	uint32_t shm_addr;
	uint32_t shm_size;
	uint16_t mhu_ch_size;
};

struct mhu_data {
	DEVICE_MMIO_RAM; /* Must be first */
	ipm_callback_t cb;
	void *user_data;
	void *shm_tx;
	void *shm_rx;
	uint32_t shm_ch_size;
	char *mhu_msg_buf;
};

#define DEV_CFG(_dev)  ((const struct mhu_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct mhu_data *const)(_dev)->data)

static void mhu_irq_handler(const struct device *dev)
{
	const struct mhu_config *cfg = DEV_CFG(dev);
	struct mhu_data *data = DEV_DATA(dev);
	mm_reg_t sts_off, clr_off;

	if (cfg->type == MHU_RSP) {
		sts_off = MSG_REG(dev, INT_STS_OFF);
		clr_off = MSG_REG(dev, INT_CLR_OFF);
	} else {
		sts_off = RSP_REG(dev, INT_STS_OFF);
		clr_off = RSP_REG(dev, INT_CLR_OFF);
	}

	if (sys_read32(sts_off) == 0) {
		LOG_ERR("ERROR: Unexpected status\n");
		return;
	};

	/* Copy buffer only if cb was provided */
	if (data->cb && cfg->shm_addr) {
		memcpy(data->mhu_msg_buf, data->shm_rx, cfg->mhu_ch_size);
	}

	sys_write32(1, clr_off);

	if (data->cb) {
		data->cb(dev, data->user_data, 0, data->mhu_msg_buf);
	}
}

static int mhu_send(const struct device *dev, int wait, uint32_t cpu_id, const void *buf, int size)
{
	const struct mhu_config *cfg = DEV_CFG(dev);
	struct mhu_data *data = DEV_DATA(dev);
	mm_reg_t sts_off, set_off;

	if (size > cfg->mhu_ch_size) {
		return -EINVAL;
	}

	if (cfg->type == MHU_SEND) {
		sts_off = MSG_REG(dev, INT_STS_OFF);
		set_off = MSG_REG(dev, INT_SET_OFF);
	} else {
		sts_off = RSP_REG(dev, INT_STS_OFF);
		set_off = RSP_REG(dev, INT_SET_OFF);
	}

	while (sys_read32(sts_off)) {
		k_busy_wait(1);
	};

	if (cfg->shm_addr) {
		if (buf && size) {
			/* Put message to the shared memory */
			memcpy(data->shm_tx, buf, size);
		} else {
			/* cleanup memory if buffer was not set */
			memset(data->shm_tx, 0, cfg->mhu_ch_size);
		}
	}

	sys_write32(1, set_off);

	return 0;
}

static void mhu_reg_callback(const struct device *dev, ipm_callback_t cb, void *user_data)
{
	struct mhu_data *data = DEV_DATA(dev);

	data->cb = cb;
	data->user_data = user_data;
}

static int mhu_init(const struct device *dev)
{
	int ret;
	struct mhu_data *data = DEV_DATA(dev);
	const struct mhu_config *cfg = DEV_CFG(dev);

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (device_is_ready(cfg->clk_dev)) {
		ret = clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->clk_mod);
		if (ret < 0) {
			LOG_ERR("Failed to configure clk");
			return ret;
		}
	}

	if (device_is_ready(cfg->reset.dev)) {
		(void)reset_line_deassert_dt(&cfg->reset);
	}

	if (cfg->shm_addr) {
		data->shm_tx = (void *)(cfg->shm_addr);
		data->shm_rx = (void *)(cfg->shm_addr + cfg->mhu_ch_size);
	}

	cfg->irq_configure();
	return 0;
}

static int mhu_set_enabled(const struct device *dev, int enable)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(enable);
	return 0;
}

static int mhu_max_data_size_get(const struct device *dev)
{
	const struct mhu_config *cfg = DEV_CFG(dev);

	ARG_UNUSED(dev);

	return cfg->mhu_ch_size;
}

static uint32_t mhu_max_id_val_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return MHU_MAX_ID_VAL;
}

static const struct ipm_driver_api mhu_driver_api = {
	.send = mhu_send,
	.register_callback = mhu_reg_callback,
	.max_data_size_get = mhu_max_data_size_get,
	.max_id_val_get = mhu_max_id_val_get,
	.set_enabled = mhu_set_enabled,
};

#define MHU_SHM_SIZE(n)                                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),                                       \
		    (DT_REG_SIZE(DT_INST_PHANDLE(n, memory_region))), (0))

#define MHU_SET_CLOCK(n)                                                                           \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, clocks),                                              \
		    (.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(n, 0)),                   \
		     .clk_mod.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),                   \
		     .clk_mod.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),),                 \
		    ())

#define MHU_SET_RESET(n)                                                                           \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n),), ())

#define RENESAS_MHU_INIT(n)                                                                        \
                                                                                                   \
	static char mhu_buf_##n[(MHU_SHM_SIZE(n) >> 1) + 1] = {};                                  \
                                                                                                   \
	static void mhu_irq_##n##_config(void)                                                     \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ_BY_IDX(n, 0, priority),     \
			    mhu_irq_handler, DEVICE_DT_INST_GET(n), 0);                            \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                         \
	}                                                                                          \
                                                                                                   \
	static const struct mhu_config mhu_##n##_conf = {                                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		MHU_SET_CLOCK(n) MHU_SET_RESET(n).irq_configure = mhu_irq_##n##_config,            \
		.type = DT_INST_PROP(n, renesas_mhu_type),                                         \
		.shm_addr = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),                   \
					(DT_REG_ADDR(DT_INST_PHANDLE(n, memory_region))), (0)),    \
		.shm_size = MHU_SHM_SIZE(n),                                                       \
		.mhu_ch_size = MHU_SHM_SIZE(n) >> 1,                                               \
	};                                                                                         \
                                                                                                   \
	static struct mhu_data mhu_##n##_data = {                                                  \
		.mhu_msg_buf = mhu_buf_##n,                                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &mhu_init, NULL, &mhu_##n##_data, &mhu_##n##_conf, PRE_KERNEL_1,  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mhu_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_MHU_INIT);
