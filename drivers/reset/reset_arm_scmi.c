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

#if defined(CONFIG_ARM_SCMI_RESET_SHELL)
static int scmi_reset_domain_attr_get(const struct device *scmi_dev, uint32_t id,
				      struct scmi_msg_reset_domain_attr_resp *dom_attr)
{
	struct scmi_msg_reset_domain_attr_req dom_attr_req = {0};
	int ret;

	dom_attr_req.domain_id = sys_cpu_to_le32(id);

	ret = scmi_xfer(scmi_dev, SCMI_PROTOCOL_RESET, SCMI_RESET_DOMAIN_ATTRIBUTES,
			(uint8_t *)&dom_attr_req, sizeof(struct scmi_msg_reset_domain_attr_req),
			(uint8_t *)dom_attr, sizeof(struct scmi_msg_reset_domain_attr_resp));
	if (ret) {
		LOG_ERR("reset domain get attributes failed (%d)", ret);
		return ret;
	}

	dom_attr->attr = sys_le32_to_cpu(dom_attr->attr);
	dom_attr->latency = sys_le32_to_cpu(dom_attr->latency);

	return 0;
}
#endif /* CONFIG_ARM_SCMI_RESET_SHELL */

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

#if defined(CONFIG_ARM_SCMI_RESET_SHELL)
#include <zephyr/shell/shell.h>

#if defined(CONFIG_DT_HAS_ARM_SCMI_RESET_ENABLED)
static const struct device *reset_dev = DEVICE_DT_GET_ANY(DT_DRV_COMPAT);
#else
BUILD_ASSERT(1, "unsupported scmi reset interface");
#endif

static int scmi_shell_reset_revision(const struct shell *sh, size_t argc, char **argv)
{
	struct scmi_reset_drv_data *data = reset_dev->data;

	shell_print(sh, "ARM SCMI Reset protocol version 0x%04x.%04x num_domains:%u",
		    data->version.ver.major, data->version.ver.minor, data->num_domains);

	return 0;
}

static int scmi_shell_reset_dom_list(const struct shell *sh, size_t argc, char **argv)
{
	const struct scmi_reset_config *cfg = reset_dev->config;
	struct scmi_msg_reset_domain_attr_resp dom_attr = {0};
	struct scmi_reset_drv_data *data = reset_dev->data;
	uint16_t i;
	int ret;

	shell_print(sh, "domain_id,name,latency,attributes");

	for (i = 0; i < data->num_domains; i++) {
		ret = scmi_reset_domain_attr_get(cfg->scmi_dev, i, &dom_attr);
		if (ret) {
			shell_error(sh, "reset domain:%u get attributes failed (%d)", ret, i);
			return ret;
		}

		shell_print(sh, "%u,%s,0x%08x,0x%08x", i, dom_attr.name, dom_attr.latency,
			    dom_attr.attr);
	}

	return 0;
}

static int scmi_shell_reset_info(const struct shell *sh, size_t argc, char **argv)
{
	const struct scmi_reset_config *cfg = reset_dev->config;
	struct scmi_msg_reset_domain_attr_resp dom_attr = {0};
	struct scmi_reset_drv_data *data = reset_dev->data;
	uint32_t domain_id;
	int ret;

	domain_id = atoi(argv[1]);
	if (domain_id >= data->num_domains) {
		shell_error(sh, "invalid reset domain index %s\n", argv[1]);
		return -ENOENT;
	}

	ret = scmi_reset_domain_attr_get(cfg->scmi_dev, domain_id, &dom_attr);
	if (ret) {
		shell_error(sh, "reset domain get attributes failed (%d)", ret);
		return ret;
	}

	shell_print(sh, "ARM SCMI reset domain: %u", domain_id);
	shell_print(sh, " name\t\t: %s", dom_attr.name);
	if (dom_attr.latency == SCMI_RESET_ATTR_LATENCY_UNK1 ||
	    dom_attr.latency == SCMI_RESET_ATTR_LATENCY_UNK1) {
		shell_print(sh, " latency\t: unk");
	} else {
		shell_print(sh, " latency\t: %u", dom_attr.latency);
	}
	shell_print(sh, " attr\t\t: 0x%08x", dom_attr.attr);

	return 0;
}

static int scmi_shell_reset_assert(const struct shell *sh, size_t argc, char **argv)
{
	struct scmi_reset_drv_data *data = reset_dev->data;
	uint32_t domain_id;

	domain_id = atoi(argv[1]);
	if (domain_id >= data->num_domains) {
		shell_error(sh, "invalid reset domain index %s\n", argv[1]);
		return -ENOENT;
	}

	return scmi_reset_line_assert(reset_dev, domain_id);
}

static int scmi_shell_reset_deassert(const struct shell *sh, size_t argc, char **argv)
{
	struct scmi_reset_drv_data *data = reset_dev->data;
	uint32_t domain_id;

	domain_id = atoi(argv[1]);
	if (domain_id >= data->num_domains) {
		shell_error(sh, "invalid reset domain index %s\n", argv[1]);
		return -ENOENT;
	}

	return scmi_reset_line_deassert(reset_dev, domain_id);
}

static int scmi_shell_reset_toggle(const struct shell *sh, size_t argc, char **argv)
{
	struct scmi_reset_drv_data *data = reset_dev->data;
	uint32_t domain_id;

	domain_id = atoi(argv[1]);
	if (domain_id >= data->num_domains) {
		shell_error(sh, "invalid reset domain index %s\n", argv[1]);
		return -ENOENT;
	}

	return scmi_reset_line_toggle(reset_dev, domain_id);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_scmi_reset_cmds,
	SHELL_CMD_ARG(revision, NULL,
		      "SCMI Reset proto show revision information\n"
		      "Usage: arm_scmi reset revision\n",
		      scmi_shell_reset_revision, 1, 0),
	SHELL_CMD_ARG(list, NULL,
		      "SCMI Reset domains list\n"
		      "Usage: arm_scmi reset list\n",
		      scmi_shell_reset_dom_list, 1, 0),
	SHELL_CMD_ARG(info, NULL,
		      "SCMI Reset domain show info\n"
		      "Usage: arm_scmi reset info <domain_id>\n",
		      scmi_shell_reset_info, 2, 0),
	SHELL_CMD_ARG(assert, NULL,
		      "SCMI Reset domain assert\n"
		      "Usage: arm_scmi reset assert <domain_id>\n",
		      scmi_shell_reset_assert, 2, 0),
	SHELL_CMD_ARG(deassert, NULL,
		      "SCMI Reset domain de-assert\n"
		      "Usage: arm_scmi reset deassert <domain_id>\n",
		      scmi_shell_reset_deassert, 2, 0),
	SHELL_CMD_ARG(autoreset, NULL,
		      "SCMI Reset domain Autonomous reset\n"
		      "Usage: arm_scmi reset autoreset <domain_id>\n",
		      scmi_shell_reset_toggle, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((arm_scmi), reset, &sub_scmi_reset_cmds,
		 "SCMI Reset proto commands.",
		 NULL, 1, 1);

#endif /* CONFIG_ARM_SCMI_RESET_SHELL */
