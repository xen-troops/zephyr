/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ARM_SCMI_H_
#define DRIVERS_ARM_SCMI_H_

#include <zephyr/drivers/firmware/arm_scmi_protocol.h>

static const int32_t scmi_errmap[] = {
	/* better than switch case as long as return value is continuous */
	0,           /* SCMI_SUCCESS */
	-EOPNOTSUPP, /* SCMI_ERR_SUPPORT */
	-EINVAL,     /* SCMI_ERR_PARAM */
	-EACCES,     /* SCMI_ERR_ACCESS */
	-ENOENT,     /* SCMI_ERR_ENTRY */
	-ERANGE,     /* SCMI_ERR_RANGE */
	-ENOMEM,     /* SCMI_ERR_BUSY */
	-ECOMM,      /* SCMI_ERR_COMMS */
	-EIO,        /* SCMI_ERR_GENERIC */
	-EREMOTE,    /* SCMI_ERR_HARDWARE */
	-EPROTO,     /* SCMI_ERR_PROTOCOL */
	-EBUSY,      /* SCMI_ERR_IN_USE */
};

static inline int scmi_to_errno(int32_t errno)
{
	int32_t err_idx = -errno;

	if (err_idx >= SCMI_SUCCESS && err_idx < ARRAY_SIZE(scmi_errmap)) {
		return scmi_errmap[err_idx];
	}

	return -EIO;
}

/**
 * struct scmi_msg_hdr - Message(Tx/Rx) header
 *
 * @msg_id: The identifier of the message being sent
 * @protocol_id: The identifier of the protocol used to send @id message
 * @type: The SCMI type for this message
 * @seq: The token to identify the message. When a message returns, the
 *	platform returns the whole message header unmodified including the
 *	token
 * @status: Status of the transfer once it's complete
 */
struct scmi_msg_hdr {
	uint8_t msg_id;
	uint8_t protocol_id;
	uint8_t type;
	uint16_t seq;
	int32_t status;
};

/**
 * pack_scmi_header() - packs and returns 32-bit header
 *
 * @hdr: pointer to header containing all the information on message id,
 *	protocol id, sequence id and type.
 *
 * Return: 32-bit packed message header to be sent to the platform.
 */
static inline uint32_t scmi_pack_msg_header(struct scmi_msg_hdr *hdr)
{
	uint32_t val;

	val = FIELD_PREP(SCMI_MSG_ID_MASK, hdr->msg_id) |
		FIELD_PREP(SCMI_MSG_TYPE_MASK, hdr->type) |
		FIELD_PREP(SCMI_MSG_TOKEN_ID_MASK, hdr->seq) |
		FIELD_PREP(SCMI_MSG_PROTOCOL_ID_MASK, hdr->protocol_id);
	return val;
}

/**
 * @brief Message(Tx/Rx) structure
 *
 * @buf: Buffer pointer
 * @len: Length of data in the Buffer
 */
struct scmi_msg_data {
	uint8_t *buf;
	size_t len;
};

/**
 * @brief Structure representing SCMI message xfer
 *
 * @transfer_id: Unique ID for debug & profiling purpose
 * @hdr: Transmit message header
 * @tx: Transmit message
 * @rx: Receive message, the buffer should be pre-allocated to store
 *	message. If request-ACK protocol is used, we can reuse the same
 *	buffer for the rx path as we use for the tx path.
 */
struct scmi_xfer {
	int transfer_id;
	struct scmi_msg_hdr hdr;
	struct scmi_msg_data tx;
	struct scmi_msg_data rx;
};

/**
 * @brief SCMI API to perform sync transfer.
 */
typedef int (*scmi_do_xfer_t)(const struct device *dev, struct scmi_xfer *xfer);

/**
 * @brief  API structure for scmi base transport driver.
 *
 */
struct scmi_driver_api {
	scmi_do_xfer_t do_xfer;
};

/**
 * @brief Perform SCMI xfer without TX data
 *
 * @param scmi_dev SCMI device reference.
 * @param protocol_id SCMI protocol id.
 * @param msg_id SCMI message id.
 * @param rx_buf SCMI RX data buffer
 * @param rx_len SCMI RX data buffer length
 *
 * @retval 0 If successful.
 * @retval -ETIMEDOUT if transport not ready.
 * @retval SCMI error from @scmi_errmap
 */
int scmi_xfer_no_tx(const struct device *scmi_dev, uint8_t protocol_id, uint8_t msg_id,
		    uint8_t *rx_buf, size_t rx_len);

/**
 * @brief Perform SCMI xfer without RX data
 *
 * @param scmi_dev SCMI device reference.
 * @param protocol_id SCMI protocol id.
 * @param msg_id SCMI message id.
 * @param tx_buf SCMI TX data buffer
 * @param tx_len SCMI TX data buffer length
 *
 * @retval 0 If successful.
 * @retval -ETIMEDOUT if transport not ready.
 * @retval SCMI error from @scmi_errmap
 */
int scmi_xfer_no_rx(const struct device *scmi_dev, uint8_t protocol_id, uint8_t msg_id,
		    uint8_t *tx_buf, size_t tx_len);

/**
 * @brief Perform SCMI xfer with RX/TX data
 *
 * @param scmi_dev SCMI device reference.
 * @param protocol_id SCMI protocol id.
 * @param msg_id SCMI message id.
 * @param tx_buf SCMI TX data buffer
 * @param tx_len SCMI TX data buffer length
 * @param rx_buf SCMI RX data buffer
 * @param rx_len SCMI RX data buffer length
 *
 * @retval 0 If successful.
 * @retval -ETIMEDOUT if transport not ready.
 * @retval SCMI error from @scmi_errmap
 */
int scmi_xfer(const struct device *scmi_dev, uint8_t protocol_id, uint8_t msg_id, uint8_t *tx_buf,
	      size_t tx_len, uint8_t *rx_buf, size_t rx_len);

/**
 * @brief Get SCMI protocol version (SCMI_PROTOCOL_VERSION msg)
 *
 * @param scmi_dev SCMI device reference.
 * @param protocol_id SCMI protocol id.
 * @param version pointer to SCMI protocol version struct scmi_msg_prot_version_resp
 *
 * @retval 0 If successful.
 */
int scmi_proto_get_version(const struct device *scmi_dev, uint8_t protocol_id,
			   struct scmi_msg_prot_version_p2a *version);

#endif /* DRIVERS_ARM_SCMI_H_ */
