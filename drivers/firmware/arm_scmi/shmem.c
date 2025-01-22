/*
 * Copyright 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/firmware/arm_scmi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
LOG_MODULE_REGISTER(arm_scmi_shm, CONFIG_ARM_SCMI_LOG_LEVEL);

#include "shmem.h"

#define SCMI_SHM_MAX_RX_TIMEOUT_MS 30

/*
 * Note. SCMI specification requires all parameters, message headers, return
 * arguments or any protocol data to be expressed in little endian.
 */
struct scmi_shm_hdr {
	uint32_t reserved;
	uint32_t channel_status;
#define SCMI_SHM_CHANNEL_STATUS_ERROR BIT(1)
#define SCMI_SHM_CHANNEL_STATUS_FREE  BIT(0)
	uint32_t reserved1[2];
	uint32_t flags;
#define SCMI_SHM_FLAG_INTR_ENABLED BIT(0)
	uint32_t length;
	uint32_t msg_header;
	uint8_t payload[];
};

#define SCMI_SHM_CHANNEL_STATUS_OFS offsetof(struct scmi_shm_hdr, channel_status)
#define SCMI_SHM_FLAGS_OFS          offsetof(struct scmi_shm_hdr, flags)
#define SCMI_SHM_MSG_LEN_OFS        offsetof(struct scmi_shm_hdr, length)
#define SCMI_SHM_MSG_HDR_OFS        offsetof(struct scmi_shm_hdr, msg_header)
#define SCMI_SHM_MSG_PAYLOAD_OFS    offsetof(struct scmi_shm_hdr, payload)
#define SCMI_SHM_MSG_RX_STATUS_OFS  SCMI_SHM_MSG_PAYLOAD_OFS
#define SCMI_SHM_MSG_TX_DATA_OFS    (SCMI_SHM_MSG_PAYLOAD_OFS)
#define SCMI_SHM_MSG_RX_DATA_OFS    (SCMI_SHM_MSG_PAYLOAD_OFS + 4)

int scmi_shm_tx_put(mm_reg_t shmem, struct scmi_xfer *xfer)
{
	volatile struct scmi_shm_hdr *hdr = (struct scmi_shm_hdr *)shmem;
	int32_t timeout = 2 * SCMI_SHM_MAX_RX_TIMEOUT_MS * USEC_PER_MSEC;
	uint32_t val;

	LOG_HEXDUMP_DBG((void *)shmem, 64, "shmem_before");

	if (!WAIT_FOR(scmi_shm_is_free(shmem), timeout, k_busy_wait(10))) {
		LOG_ERR("shmem tx channel free timeout");
		return -ETIMEDOUT;
	}
	/* Mark channel busy + clear error */
	hdr->channel_status = 0;
	hdr->flags = 0;
	val = sizeof(uint32_t) + xfer->tx.len;
	sys_put_le32(val, (uint8_t *)&hdr->length);
	sys_put_le32(scmi_pack_msg_header(&xfer->hdr), (uint8_t *)&hdr->msg_header);

	if (xfer->tx.buf) {
		memcpy((uint8_t *)hdr->payload, xfer->tx.buf, xfer->tx.len);
	}

	barrier_dmem_fence_full();

	LOG_HEXDUMP_DBG((void *)shmem, 64, "shmem after");
	return 0;
}

void scmi_shm_fetch_response(mm_reg_t shmem, struct scmi_xfer *xfer)
{
	volatile struct scmi_shm_hdr *hdr = (struct scmi_shm_hdr *)shmem;
	size_t len;

	LOG_HEXDUMP_DBG((void *)shmem, 64, "shmem fetch");

	len = sys_get_le32((uint8_t *)&hdr->length);
	xfer->hdr.status = sys_get_le32((uint8_t *)hdr->payload);
	/* Skip the length of header and status in scmi shmem area i.e 8 bytes */
	len = len > 8 ? len - 8 : 0;
	xfer->rx.len = MIN(xfer->rx.len, len);

	/* Take a copy to the rx buffer.. */
	memcpy(xfer->rx.buf, (uint8_t *)hdr->payload + sizeof(uint32_t), xfer->rx.len);

	barrier_dmem_fence_full();
}

bool scmi_shm_is_free(mm_reg_t shmem)
{
	uint32_t val = sys_read32(shmem + SCMI_SHM_CHANNEL_STATUS_OFS);

	return (sys_le32_to_cpu(val) & SCMI_SHM_CHANNEL_STATUS_FREE);
}

uint32_t shmem_read_header(mm_reg_t shmem)
{
	uint32_t val = sys_read32(shmem + SCMI_SHM_MSG_HDR_OFS);

	return sys_le32_to_cpu(val);
}

void shmem_clear_channel(mm_reg_t shmem)
{
	uint32_t val = sys_cpu_to_le32(SCMI_SHM_CHANNEL_STATUS_FREE);

	sys_write32(val, shmem + SCMI_SHM_CHANNEL_STATUS_OFS);
}

bool shmem_poll_done(mm_reg_t shmem, struct scmi_xfer *xfer)
{
	uint32_t val = sys_le32_to_cpu(sys_read32(shmem + SCMI_SHM_MSG_HDR_OFS));
	uint16_t xfer_id;

	xfer_id = SCMI_MSG_XTRACT_TOKEN(val);

	if (xfer->hdr.seq != xfer_id) {
		return false;
	}

	val = sys_le32_to_cpu(sys_read32(shmem + SCMI_SHM_CHANNEL_STATUS_OFS));

	return val & (SCMI_SHM_CHANNEL_STATUS_ERROR | SCMI_SHM_CHANNEL_STATUS_FREE);
}
