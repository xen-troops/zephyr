/*
 * Copyright 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/firmware/arm_scmi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(arm_scmi_reset, CONFIG_ARM_SCMI_LOG_LEVEL);

#define DT_DRV_COMPAT arm_scmi_reset

/*
 * SCMI Reset protocol
 */
#define SCMI_PROTOCOL_RESET_REV_MAJOR 0x1

enum scmi_reset_protocol_cmd {
	SCMI_RESET_DOMAIN_ATTRIBUTES = 0x3,
	SCMI_RESET = 0x4,
	SCMI_RESET_NOTIFY = 0x5,
	SCMI_RESET_DOMAIN_NAME_GET = 0x6,
};

/* Reset PROTOCOL_ATTRIBUTES cmd response */
struct scmi_msg_reset_attributes_resp {
	uint32_t attributes;
#define SCMI_RESET_ATTR_NUM_DOMAINS           GENMASK(15, 0)
#define SCMI_RESET_ATTR_GET_NUM_DOMAINS(attr) FIELD_GET(SCMI_RESET_ATTR_NUM_DOMAINS, (attr))
} __packed;

/* RESET_DOMAIN_ATTRIBUTES cmd request */
struct scmi_msg_reset_domain_attr_req {
	uint32_t domain_id;
} __packed;

/* RESET_DOMAIN_ATTRIBUTES cmd response */
struct scmi_msg_reset_domain_attr_resp {
	uint32_t attr;
#define SCMI_RESET_ATTR_SUPPORTS_ASYNC     BIT(31)
#define SCMI_RESET_ATTR_SUPPORTS_NOTIFY    BIT(30)
#define SCMI_RESET_ATTR_SUPPORTS_EXT_NAMES BIT(29)
	uint32_t latency;
#define SCMI_RESET_ATTR_LATENCY_UNK1 0x7fffffff
#define SCMI_RESET_ATTR_LATENCY_UNK2 0xffffffff
	char name[SCMI_SHORT_NAME_MAX_SIZE];
} __packed;

/* RESET cmd response */
struct scmi_msg_reset_domain_reset_req {
	uint32_t domain_id;
	uint32_t flags;
#define SCMI_RESET_AUTONOMOUS         BIT(0)
#define SCMI_RESET_EXPLICIT_ASSERT    BIT(1)
#define SCMI_RESET_ASYNCHRONOUS_RESET BIT(2)
	uint32_t reset_state;
#define SCMI_RESET_ARCH_COLD_RESET 0
} __packed;

/* scmi reset driver definitions */
struct scmi_reset_config {
	const struct device *scmi_dev;
};

struct scmi_reset_drv_data {
	struct scmi_msg_prot_version_p2a version;
	uint16_t num_domains;
};

static int scmi_reset_proto_attr_get(const struct device *scmi_dev, uint16_t *num_domains)
{
	struct scmi_msg_reset_attributes_resp attr = {0};
	int ret;

	ret = scmi_xfer_no_tx(scmi_dev, SCMI_PROTOCOL_RESET, SCMI_PROTOCOL_ATTRIBUTES,
			      (uint8_t *)&attr, sizeof(struct scmi_msg_reset_attributes_resp));
	if (ret) {
		LOG_ERR("reset proto get attributes failed (%d)", ret);
		return ret;
	}

	*num_domains = SCMI_RESET_ATTR_GET_NUM_DOMAINS(sys_le32_to_cpu(attr.attributes));

	return 0;
}

static int scmi_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct scmi_reset_config *cfg = dev->config;
	struct scmi_msg_reset_domain_reset_req reset_cmd;
	int ret;

	reset_cmd.domain_id = sys_cpu_to_le32(id);
	reset_cmd.flags = sys_cpu_to_le32(SCMI_RESET_EXPLICIT_ASSERT);
	reset_cmd.reset_state = sys_cpu_to_le32(SCMI_RESET_ARCH_COLD_RESET);

	ret = scmi_xfer_no_rx(cfg->scmi_dev, SCMI_PROTOCOL_RESET, SCMI_RESET, (uint8_t *)&reset_cmd,
			      sizeof(struct scmi_msg_reset_domain_reset_req));
	if (ret) {
		LOG_ERR("scmi reset:%u assert failed (%d)", id, ret);
	}

	LOG_DBG("scmi reset:%u assert", id);

	return ret;
}

static int scmi_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct scmi_reset_config *cfg = dev->config;
	struct scmi_msg_reset_domain_reset_req reset_cmd;
	int ret;

	reset_cmd.domain_id = sys_cpu_to_le32(id);
	reset_cmd.flags = 0;
	reset_cmd.reset_state = sys_cpu_to_le32(SCMI_RESET_ARCH_COLD_RESET);

	ret = scmi_xfer_no_rx(cfg->scmi_dev, SCMI_PROTOCOL_RESET, SCMI_RESET, (uint8_t *)&reset_cmd,
			      sizeof(struct scmi_msg_reset_domain_reset_req));
	if (ret) {
		LOG_ERR("scmi reset deassert failed (%d)", ret);
	}

	LOG_DBG("scmi reset:%u deassert", id);

	return ret;
}

static int scmi_reset_line_toggle(const struct device *dev, uint32_t id)
{
	const struct scmi_reset_config *cfg = dev->config;
	struct scmi_msg_reset_domain_reset_req reset_cmd;
	int ret;

	reset_cmd.domain_id = sys_cpu_to_le32(id);
	reset_cmd.flags = sys_cpu_to_le32(SCMI_RESET_AUTONOMOUS);
	reset_cmd.reset_state = sys_cpu_to_le32(SCMI_RESET_ARCH_COLD_RESET);

	ret = scmi_xfer_no_rx(cfg->scmi_dev, SCMI_PROTOCOL_RESET, SCMI_RESET, (uint8_t *)&reset_cmd,
			      sizeof(struct scmi_msg_reset_domain_reset_req));
	if (ret) {
		LOG_ERR("scmi reset toggle failed (%d)", ret);
	}

	LOG_DBG("scmi reset:%u toggle", id);

	return ret;
}

static int scmi_reset_init(const struct device *dev)
{
	const struct scmi_reset_config *cfg = dev->config;
	struct scmi_reset_drv_data *data = dev->data;
	int ret;

	ret = scmi_proto_get_version(cfg->scmi_dev, SCMI_PROTOCOL_RESET, &data->version);
	if (ret) {
		return ret;
	}

	if (data->version.ver.major != SCMI_PROTOCOL_RESET_REV_MAJOR) {
		LOG_ERR("unsupported reset protocol version 0x%08x", data->version.version);
		return -ENOTSUP;
	}

	ret = scmi_reset_proto_attr_get(cfg->scmi_dev, &data->num_domains);
	if (ret) {
		return ret;
	}

	LOG_INF("scmi reset rotocol version 0x%04x.%04x num_domains:%u", data->version.ver.major,
		data->version.ver.minor, data->num_domains);

	return 0;
}

static const struct reset_driver_api scmi_reset_driver_api = {
	.line_assert = scmi_reset_line_assert,
	.line_deassert = scmi_reset_line_deassert,
	.line_toggle = scmi_reset_line_toggle,
};

static struct scmi_reset_drv_data scmi_reset_data;

static const struct scmi_reset_config scmi_reset_config = {
	.scmi_dev = DEVICE_DT_GET(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, scmi_reset_init, NULL, &scmi_reset_data, &scmi_reset_config, PRE_KERNEL_2,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &scmi_reset_driver_api);
