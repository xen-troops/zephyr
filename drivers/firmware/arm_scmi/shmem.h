/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ARM_SCMI_SHMEM_H_
#define DRIVERS_ARM_SCMI_SHMEM_H_

/**
 * @brief Prepare SCMI Shared memory (shmem) based transport for xfer.
 *
 * @param shmem SCMI shmem address.
 * @param xfer pointer on SCMI xfer data.
 * @retval 0 If successful.
 * @retval -ETIMEDOUT if shmem is not free.
 */
int scmi_shm_tx_put(mm_reg_t shmem, struct scmi_xfer *xfer);

/**
 * @brief Get SCMI xfer result from shmem based transport.
 *
 * @param shmem SCMI shmem address.
 * @param xfer pointer SCMI xfer data.
 */
void scmi_shm_fetch_response(mm_reg_t shmem, struct scmi_xfer *xfer);

/**
 * @brief Check if SCMI Shared memory (shmem) based transport is free.
 *
 * @param shmem SCMI shmem address.
 * @retval true If free.
 * @retval true If busy.
 */
bool scmi_shm_is_free(mm_reg_t shmem);

#endif /* DRIVERS_ARM_SCMI_SHMEM_H_ */
