/*
 * System Control and Management Interface (SCMI) Base Protocol
 *
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/firmware/arm_scmi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(arm_scmi_proto_base, CONFIG_ARM_SCMI_LOG_LEVEL);

#include "base.h"

/*
 * SCMI Base protocol
 */
enum scmi_base_protocol_cmd {
	SCMI_BASE_DISCOVER_VENDOR = 0x3,
	SCMI_BASE_DISCOVER_SUB_VENDOR = 0x4,
	SCMI_BASE_DISCOVER_IMPLEMENT_VERSION = 0x5,
	SCMI_BASE_DISCOVER_LIST_PROTOCOLS = 0x6,
	SCMI_BASE_DISCOVER_AGENT = 0x7,
	SCMI_BASE_NOTIFY_ERRORS = 0x8,
	SCMI_BASE_SET_DEVICE_PERMISSIONS = 0x9,
	SCMI_BASE_SET_PROTOCOL_PERMISSIONS = 0xa,
	SCMI_BASE_RESET_AGENT_CONFIGURATION = 0xb,
};

/* BASE PROTOCOL_ATTRIBUTES */
struct scmi_msg_base_attributes_p2a {
	uint8_t num_protocols;
	uint8_t num_agents;
	uint16_t reserved;
} __packed;

/* BASE_DISCOVER_VENDOR */
struct scmi_msg_base_vendor_id_p2a {
	char vendor_id[SCMI_SHORT_NAME_MAX_SIZE];
} __packed;

/* BASE_DISCOVER_SUB_VENDOR */
struct scmi_msg_base_subvendor_id_p2a {
	char subvendor_id[SCMI_SHORT_NAME_MAX_SIZE];
} __packed;

/* BASE_DISCOVER_IMPLEMENTATION_VERSION */
struct scmi_msg_base_impl_ver_p2a {
	uint32_t impl_ver;
} __packed;

/*
 * BASE_DISCOVER_AGENT
 */
struct scmi_msg_base_discover_agent_a2p {
	uint32_t agent_id;
} __packed;

struct scmi_msg_base_discover_agent_p2a {
	uint32_t agent_id;
	char name[SCMI_SHORT_NAME_MAX_SIZE];
} __packed;

/*
 * BASE_SET_DEVICE_PERMISSIONS
 */
#define SCMI_BASE_DEVICE_ACCESS_ALLOW			BIT(0)

struct scmi_msg_base_set_device_permissions_a2p {
	uint32_t agent_id;
	uint32_t device_id;
	uint32_t flags;
} __packed;

/*
 * BASE_RESET_AGENT_CONFIGURATION
 */
#define SCMI_BASE_AGENT_PERMISSIONS_RESET		BIT(0)

struct scmi_msg_base_reset_agent_cfg_a2p {
	uint32_t agent_id;
	uint32_t flags;
} __packed;

#if defined(CONFIG_ARM_SCMI_BASE_EXT_REV)
static int scmi_base_attributes_get(const struct device *scmi_dev,
				    struct scmi_msg_base_attributes_p2a *attr)
{
	int ret;

	ret = scmi_xfer_no_tx(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_PROTOCOL_ATTRIBUTES,
			      (uint8_t *)attr, sizeof(*attr));
	if (ret) {
		LOG_ERR("base proto get attributes failed (%d)", ret);
		return ret;
	}

	LOG_DBG("base attr num_protocols:0x%02x num_agents:0x%02x", attr->num_protocols,
		attr->num_agents);

	return 0;
}

static int scmi_base_vendor_id_get(const struct device *scmi_dev,
				   struct scmi_msg_base_vendor_id_p2a *id)
{
	int ret;

	ret = scmi_xfer_no_tx(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_BASE_DISCOVER_VENDOR,
			      (uint8_t *)id, sizeof(*id));
	if (ret) {
		LOG_ERR("base proto get vendor id failed (%d)", ret);
		return ret;
	}

	LOG_DBG("base vendor id:%s", id->vendor_id);

	return 0;
}

static int scmi_base_subvendor_id_get(const struct device *scmi_dev,
				      struct scmi_msg_base_subvendor_id_p2a *id)
{
	int ret;

	ret = scmi_xfer_no_tx(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_BASE_DISCOVER_SUB_VENDOR,
			      (uint8_t *)id, sizeof(*id));
	if (ret) {
		LOG_ERR("base proto get subvendor id failed (%d)", ret);
		return ret;
	}

	LOG_DBG("base subvendor id:%s", id->subvendor_id);

	return 0;
}

static int scmi_base_implementation_version_get(const struct device *scmi_dev,
						struct scmi_msg_base_impl_ver_p2a *impl_ver)
{
	int ret;

	ret = scmi_xfer_no_tx(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_BASE_DISCOVER_IMPLEMENT_VERSION,
			      (uint8_t *)impl_ver, sizeof(*impl_ver));
	if (ret) {
		LOG_ERR("base proto get impl_ver failed (%d)", ret);
		return ret;
	}

	LOG_DBG("base impl_ver:0x%08x", impl_ver->impl_ver);

	return ret;
}
#endif /* CONFIG_ARM_SCMI_BASE_EXT_REV */

union scmi_base_msgs_t {
	struct scmi_msg_prot_version_p2a version;
	struct scmi_msg_base_attributes_p2a attr;
	struct scmi_msg_base_vendor_id_p2a vendor_id;
	struct scmi_msg_base_subvendor_id_p2a subvendor_id;
	struct scmi_msg_base_impl_ver_p2a impl_ver;
};

int scmi_base_get_revision_info(const struct device *scmi_dev, struct scmi_revision_info *rev)
{
	union scmi_base_msgs_t msgs;
	int ret;

	ret = scmi_proto_get_version(scmi_dev, SCMI_PROTOCOL_BASE, &msgs.version);
	if (ret) {
		return ret;
	}

	rev->major_ver = msgs.version.ver.major;
	rev->minor_ver = msgs.version.ver.minor;

	LOG_DBG("scmi base protocol v%04x.%04x", rev->major_ver, rev->minor_ver);

#if defined(CONFIG_ARM_SCMI_BASE_EXT_REV)
	ret = scmi_base_attributes_get(scmi_dev, &msgs.attr);
	if (ret) {
		return ret;
	}

	rev->num_agents = msgs.attr.num_agents;
	rev->num_protocols = msgs.attr.num_protocols;

	ret = scmi_base_vendor_id_get(scmi_dev, &msgs.vendor_id);
	if (ret) {
		return ret;
	}

	memcpy(rev->vendor_id, msgs.vendor_id.vendor_id, SCMI_SHORT_NAME_MAX_SIZE);

	ret = scmi_base_subvendor_id_get(scmi_dev, &msgs.subvendor_id);
	if (ret) {
		return ret;
	}

	memcpy(rev->sub_vendor_id, msgs.subvendor_id.subvendor_id, SCMI_SHORT_NAME_MAX_SIZE);

	ret = scmi_base_implementation_version_get(scmi_dev, &msgs.impl_ver);
	if (ret) {
		return ret;
	}

	rev->impl_ver = sys_le32_to_cpu(msgs.impl_ver.impl_ver);

	LOG_DBG("scmi base revision info vendor '%s:%s' fw version 0x%x protocols:%d agents:%d",
		rev->vendor_id, rev->sub_vendor_id, rev->impl_ver, rev->num_protocols,
		rev->num_agents);
#endif /* CONFIG_ARM_SCMI_BASE_EXT_REV */

	return 0;
}

int scmi_base_discover_agent(const struct device *scmi_dev, uint32_t agent_id,
			     struct scmi_agent_info *agent_inf)
{
	struct scmi_msg_base_discover_agent_a2p tx;
	struct scmi_msg_base_discover_agent_p2a rx;
	int ret;

	tx.agent_id = agent_id;

	ret = scmi_xfer(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_BASE_DISCOVER_AGENT, (uint8_t *)&tx,
			sizeof(struct scmi_msg_base_discover_agent_a2p), (uint8_t *)&rx,
			sizeof(struct scmi_msg_base_discover_agent_p2a));
	if (ret) {
		LOG_ERR("base proto discover agent failed (%d)", ret);
		return ret;
	}

	agent_inf->agent_id = sys_le32_to_cpu(rx.agent_id);
	strncpy(agent_inf->name, rx.name, SCMI_SHORT_NAME_MAX_SIZE);

	LOG_DBG("base discover_agent agent_id:%u name:%s", agent_inf->agent_id, agent_inf->name);

	return 0;
}

int scmi_base_device_permission(const struct device *scmi_dev, uint32_t agent_id,
				uint32_t device_id, bool allow)
{
	struct scmi_msg_base_set_device_permissions_a2p tx;
	int ret;

	LOG_DBG("base proto agent:%u device:%u permission set allow:%d", agent_id, device_id,
		allow);

	tx.agent_id = agent_id;
	tx.device_id = device_id;
	tx.flags = allow ? SCMI_BASE_DEVICE_ACCESS_ALLOW : 0;

	ret = scmi_xfer_no_rx(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_BASE_SET_DEVICE_PERMISSIONS,
			      (uint8_t *)&tx,
			      sizeof(struct scmi_msg_base_set_device_permissions_a2p));
	if (ret) {
		LOG_ERR("base proto agent:%u device:%u permission allow:%d failed (%d)", agent_id,
			device_id, allow, ret);
		return ret;
	}

	LOG_DBG("base proto agent:%u device:%u permission set allow:%d done", agent_id, device_id,
		allow);

	return 0;
}

int scmi_base_reset_agent_cfg(const struct device *scmi_dev, uint32_t agent_id, bool reset_perm)
{
	struct scmi_msg_base_reset_agent_cfg_a2p tx;
	int ret;

	LOG_DBG("base proto agent:%u reset cfg reset_perm:%d", agent_id, reset_perm);

	tx.agent_id = agent_id;
	tx.flags = reset_perm ? SCMI_BASE_AGENT_PERMISSIONS_RESET : 0;

	ret = scmi_xfer_no_rx(scmi_dev, SCMI_PROTOCOL_BASE, SCMI_BASE_RESET_AGENT_CONFIGURATION,
			      (uint8_t *)&tx, sizeof(struct scmi_msg_base_reset_agent_cfg_a2p));
	if (ret) {
		LOG_ERR("base proto agent:%u reset cfg failed (%d)", agent_id, ret);
		return ret;
	}

	LOG_DBG("base proto agent:%u reset cfg reset_perm:%d done", agent_id, reset_perm);

	return 0;
}
