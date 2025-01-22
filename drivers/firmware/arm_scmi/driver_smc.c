/*
 * Copyright 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/firmware/arm_scmi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(arm_scmi_smc, CONFIG_ARM_SCMI_LOG_LEVEL);

#include "shmem.h"
#include "base.h"

#if defined(CONFIG_DT_HAS_ARM_SCMI_SMC_ENABLED)
#define DT_DRV_COMPAT arm_scmi_smc
#elif defined(CONFIG_DT_HAS_ARM_SCMI_SMC_PARAM_ENABLED)
#define DT_DRV_COMPAT arm_scmi_smc_param
#else
BUILD_ASSERT(0, "unsupported scmi interface");
#endif

struct scmi_smc_config {
	uint32_t smc_func_id;
	uintptr_t shm_tx_phys;
	size_t shm_tx_size;
};

struct scmi_smc_data {
	mm_reg_t shm_tx_base;
	/* TODO: handle smc-param now just 0 */
	uint32_t param_page;
	uint32_t param_offset;

	struct scmi_xfer xfer;
	uint8_t tx_buf[128];
	uint16_t xfer_seq;
	struct k_spinlock xfer_lock;

	struct scmi_revision_info rev;
};

#define SHMEM_SIZE (4096)
#define SHMEM_SHIFT 12
#define SHMEM_PAGE(x) ((x) >> SHMEM_SHIFT)
#define SHMEM_OFFSET(x) ((x) & (SHMEM_SIZE - 1))

#if defined(CONFIG_ARM_SCMI_SMC_METHOD_SMC)
static void scmi_smccc_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			    unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			    struct arm_smccc_res *res)
{
	arm_smccc_smc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}
#else
static void scmi_smccc_call(unsigned long a0, unsigned long a1, unsigned long a2, unsigned long a3,
			    unsigned long a4, unsigned long a5, unsigned long a6, unsigned long a7,
			    struct arm_smccc_res *res)
{
	arm_smccc_hvc(a0, a1, a2, a3, a4, a5, a6, a7, res);
}
#endif /* CONFIG_ARM_SCMI_SMC_METHOD_SMC */

static int scmi_smc_do_xfer(const struct device *dev, struct scmi_xfer *xfer)
{
	const struct scmi_smc_config *cfg = dev->config;
	struct scmi_smc_data *data = dev->data;
	struct arm_smccc_res res;
	unsigned long page = data->param_page;
	unsigned long offset = data->param_offset;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&data->xfer_lock);

	xfer->hdr.seq = data->xfer_seq++;
	ret = scmi_shm_tx_put(data->shm_tx_base, xfer);
	if (ret) {
		goto err_exit;
	}

	LOG_DBG("xfer start seq:%u prot:%02x msg:%02x",
		xfer->hdr.seq,
		xfer->hdr.protocol_id,
		xfer->hdr.msg_id);

	scmi_smccc_call(cfg->smc_func_id, page, offset, 0, 0, 0, 0, 0, &res);

	/* Only SMCCC_RET_NOT_SUPPORTED is valid error code */
	if (res.a0) {
		ret = -EOPNOTSUPP;
		goto err_exit;
	}

	scmi_shm_fetch_response(data->shm_tx_base, xfer);

	if (xfer->hdr.status) {
		ret = scmi_to_errno(xfer->hdr.status);
	}

	LOG_DBG("xfer done seq:%u prot:%02x msg:%02x status:%d (%d)",
		xfer->hdr.seq,
		xfer->hdr.protocol_id,
		xfer->hdr.msg_id,
		xfer->hdr.status,
		ret);
err_exit:
	k_spin_unlock(&data->xfer_lock, key);
	return ret;
}

static int scmi_smc_init(const struct device *dev)
{
	const struct scmi_smc_config *cfg = dev->config;
	struct scmi_smc_data *data = dev->data;
	int ret;

	__ASSERT((cfg->shm_tx_phys % sizeof(uint32_t)) == 0U,
		 "SCMI shmem must be aligned to sizeof(uint32_t)");
	__ASSERT((cfg->shm_tx_size % sizeof(uint32_t)) == 0U,
		 "SCMI shmem size must be aligned to sizeof(uint32_t)");

	device_map(&data->shm_tx_base, cfg->shm_tx_phys, cfg->shm_tx_size, K_MEM_CACHE_NONE);
	LOG_DBG("shmem phys:%lx dev:%lx", data->shm_tx_base, cfg->shm_tx_phys);

	data->xfer_seq = 0x1;
#if defined(CONFIG_DT_HAS_ARM_SCMI_SMC_PARAM_ENABLED)
	data->param_page = SHMEM_PAGE(cfg->shm_tx_phys);
	data->param_offset = SHMEM_OFFSET(cfg->shm_tx_phys);
#endif /* CONFIG_DT_HAS_ARM_SCMI_SMC_PARAM_ENABLED */

	ret = scmi_base_get_revision_info(dev, &data->rev);
	if (ret) {
		LOG_ERR("base proto get revision failed (%d)", ret);
		return ret;
	}

	if (data->rev.major_ver != SCMI_PROTOCOL_BASE_REV_MAJOR) {
		LOG_ERR("unsupported base protocol v%04x.%04x", data->rev.major_ver,
			data->rev.minor_ver);
		return -ENOTSUP;
	}

	LOG_INF("scmi base protocol v%04x.%04x", data->rev.major_ver, data->rev.minor_ver);
	LOG_INF("scmi base revision info vendor '%s:%s' fw version 0x%x protocols:%d agents:%d",
		data->rev.vendor_id, data->rev.sub_vendor_id, data->rev.impl_ver,
		data->rev.num_protocols, data->rev.num_agents);
	LOG_INF("scmi calling method:%s",
		IS_ENABLED(CONFIG_ARM_SCMI_SMC_METHOD_SMC) ? "smc" : "hvc");

	return ret;
}

static const struct scmi_driver_api scmi_driver_api = {
	.do_xfer = scmi_smc_do_xfer,
};

#define SCMI_SMC_INIT(inst)                                                                        \
	static struct scmi_smc_config scmi_smc_cfg_##inst = {                                      \
		.smc_func_id = DT_INST_PROP(inst, arm_smc_id),                                     \
		.shm_tx_phys = DT_REG_ADDR(DT_INST_PHANDLE_BY_IDX(inst, shmem, 0)),                \
		.shm_tx_size = DT_REG_SIZE(DT_INST_PHANDLE_BY_IDX(inst, shmem, 0)),                \
	};                                                                                         \
                                                                                                   \
	static struct scmi_smc_data scmi_smc_data_##inst = {};                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, scmi_smc_init, NULL, &scmi_smc_data_##inst,                    \
			      &scmi_smc_cfg_##inst, PRE_KERNEL_2,                                  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &scmi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCMI_SMC_INIT)
