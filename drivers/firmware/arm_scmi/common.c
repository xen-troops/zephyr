/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/firmware/arm_scmi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(arm_scmi_common, CONFIG_ARM_SCMI_LOG_LEVEL);

int scmi_xfer_no_tx(const struct device *scmi_dev, uint8_t protocol_id, uint8_t msg_id,
		    uint8_t *rx_buf, size_t rx_len)
{
	const struct scmi_driver_api *api = (const struct scmi_driver_api *)scmi_dev->api;
	struct scmi_xfer xfer = {0};

	if (!api || !api->do_xfer) {
		return -EOPNOTSUPP;
	}

	xfer.hdr.protocol_id = protocol_id;
	xfer.hdr.msg_id = msg_id;
	xfer.hdr.type = SCMI_MSG_TYPE_COMMAND;
	xfer.hdr.status = SCMI_SUCCESS;

	xfer.tx.buf = NULL;
	xfer.tx.len = 0;
	xfer.rx.buf = rx_buf;
	xfer.rx.len = rx_len;

	return api->do_xfer(scmi_dev, &xfer);
}

int scmi_xfer_no_rx(const struct device *scmi_dev, uint8_t protocol_id, uint8_t msg_id,
		    uint8_t *tx_buf, size_t tx_len)
{
	const struct scmi_driver_api *api = (const struct scmi_driver_api *)scmi_dev->api;
	struct scmi_xfer xfer = {0};

	if (!api || !api->do_xfer) {
		return -EOPNOTSUPP;
	}

	xfer.hdr.protocol_id = protocol_id;
	xfer.hdr.msg_id = msg_id;
	xfer.hdr.type = SCMI_MSG_TYPE_COMMAND;
	xfer.hdr.status = SCMI_SUCCESS;

	xfer.tx.buf = tx_buf;
	xfer.tx.len = tx_len;
	xfer.rx.buf = NULL;
	xfer.rx.len = 0;

	return api->do_xfer(scmi_dev, &xfer);
}

int scmi_xfer(const struct device *scmi_dev, uint8_t protocol_id, uint8_t msg_id, uint8_t *tx_buf,
	      size_t tx_len, uint8_t *rx_buf, size_t rx_len)
{
	const struct scmi_driver_api *api = (const struct scmi_driver_api *)scmi_dev->api;
	struct scmi_xfer xfer = {0};

	if (!api || !api->do_xfer) {
		return -EOPNOTSUPP;
	}

	xfer.hdr.protocol_id = protocol_id;
	xfer.hdr.msg_id = msg_id;
	xfer.hdr.type = SCMI_MSG_TYPE_COMMAND;
	xfer.hdr.status = SCMI_SUCCESS;

	xfer.tx.buf = tx_buf;
	xfer.tx.len = tx_len;
	xfer.rx.buf = rx_buf;
	xfer.rx.len = rx_len;

	return api->do_xfer(scmi_dev, &xfer);
}

int scmi_proto_get_version(const struct device *dev, uint8_t protocol_id,
			   struct scmi_msg_prot_version_p2a *version)
{
	int ret;

	ret = scmi_xfer_no_tx(dev, protocol_id, SCMI_PROTOCOL_VERSION, (uint8_t *)version,
			      sizeof(*version));
	if (ret) {
		LOG_ERR("get version failed (%d)", ret);
		return ret;
	}

	version->version = sys_le32_to_cpu(version->version);
	LOG_DBG("scmi protocol:0x%02x version 0x%08x", protocol_id, version->version);

	return 0;
}
