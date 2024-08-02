/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ARM_SCMI_PROTOCOL_H_
#define DRIVERS_ARM_SCMI_PROTOCOL_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define SCMI_MAX_STR_SIZE        64
#define SCMI_SHORT_NAME_MAX_SIZE 16

enum scmi_error_codes {
	SCMI_SUCCESS = 0,        /* Success */
	SCMI_ERR_SUPPORT = -1,   /* Not supported */
	SCMI_ERR_PARAMS = -2,    /* Invalid Parameters */
	SCMI_ERR_ACCESS = -3,    /* Invalid access/permission denied */
	SCMI_ERR_ENTRY = -4,     /* Not found */
	SCMI_ERR_RANGE = -5,     /* Value out of range */
	SCMI_ERR_BUSY = -6,      /* Device busy */
	SCMI_ERR_COMMS = -7,     /* Communication Error */
	SCMI_ERR_GENERIC = -8,   /* Generic Error */
	SCMI_ERR_HARDWARE = -9,  /* Hardware Error */
	SCMI_ERR_PROTOCOL = -10, /* Protocol Error */
	SCMI_ERR_IN_USE = -11,   /* The resource is currently in use */
};

enum scmi_protocols {
	SCMI_PROTOCOL_BASE = 0x10,
	SCMI_PROTOCOL_POWER = 0x11,
	SCMI_PROTOCOL_SYSTEM = 0x12,
	SCMI_PROTOCOL_PERF = 0x13,
	SCMI_PROTOCOL_CLOCK = 0x14,
	SCMI_PROTOCOL_SENSOR = 0x15,
	SCMI_PROTOCOL_RESET = 0x16,
	SCMI_PROTOCOL_VOLTAGE = 0x17,
	SCMI_PROTOCOL_POWERCAP = 0x18,
	SCMI_PROTOCOL_PINCTRL = 0x19,
};

#define SCMI_PROTOCOL_REV_MINOR_MASK GENMASK(15, 0)
#define SCMI_PROTOCOL_REV_MAJOR_MASK GENMASK(31, 16)
#define SCMI_PROTOCOL_REV_MAJOR(x)   (FIELD_GET(SCMI_PROTOCOL_REV_MAJOR_MASK, (x)))
#define SCMI_PROTOCOL_REV_MINOR(x)   (FIELD_GET(SCMI_PROTOCOL_REV_MINOR_MASK, (x)))

/* Message header defines */
#define SCMI_MSG_ID_MASK             GENMASK(7, 0)
#define SCMI_MSG_XTRACT_ID(hdr)      FIELD_GET(SCMI_MSG_ID_MASK, (hdr))
#define SCMI_MSG_TYPE_MASK           GENMASK(9, 8)
#define SCMI_MSG_XTRACT_TYPE(hdr)    FIELD_GET(SCMI_MSG_TYPE_MASK, (hdr))
#define SCMI_MSG_TYPE_COMMAND        0
#define SCMI_MSG_TYPE_DELAYED_RESP   2
#define SCMI_MSG_TYPE_NOTIFICATION   3
#define SCMI_MSG_PROTOCOL_ID_MASK    GENMASK(17, 10)
#define SCMI_MSG_XTRACT_PROT_ID(hdr) FIELD_GET(SCMI_MSG_PROTOCOL_ID_MASK, (hdr))
#define SCMI_MSG_TOKEN_ID_MASK       GENMASK(27, 18)
#define SCMI_MSG_XTRACT_TOKEN(hdr)   FIELD_GET(SCMI_MSG_TOKEN_ID_MASK, (hdr))
#define SCMI_MSG_TOKEN_MAX           (SCMI_MSG_XTRACT_TOKEN(SCMI_MSG_TOKEN_ID_MASK) + 1)

enum scmi_common_cmds {
	SCMI_PROTOCOL_VERSION = 0x0,
	SCMI_PROTOCOL_ATTRIBUTES = 0x1,
	SCMI_PROTOCOL_MESSAGE_ATTRIBUTES = 0x2,
};

/**
 * struct scmi_msg_prot_version_resp - Response for a message SCMI_PROTOCOL_VERSION
 *
 * Protocol versioning uses a 32-bit unsigned integer, where
 * - the upper 16 bits are the major revision;
 * - the lower 16 bits are the minor revision.
 *
 */
struct scmi_msg_prot_version_p2a {
	union {
		uint32_t version;
		struct {
			uint16_t minor;
			uint16_t major;
		} ver;
	};
} __packed;

/*
 * SCMI Base protocol
 */
#define SCMI_PROTOCOL_BASE_REV_MAJOR 0x2

#endif /* DRIVERS_ARM_SCMI_PROTOCOL_H_ */
