/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _TEE_PRIV_H_
#define _TEE_PRIV_H_

#include <zephyr/device.h>

/**
 * @brief Registers shm object in the registry list
 *
 * Register shm object in external list. Expecting list to be properly initialized.
 * This function registers tee_shm which were allocated in terms of the session, such
 * as invoke_fn call, all allocation calls from TEE and doesn't cover shm_alloc or
 * shm_register syscalls because this memory is supposed to be requested by client and
 * do not have any information about session so system wouldn't know at which point this
 * memory should be freed. That's why those resources should be managed by client itself.
 *
 * @param list Pointer to the initialized dlist object, stored in client driver data
 * @param shm Shared memory object pointer to register
 * @param session_id Current session id
 */
int tee_shm_list_reg(sys_dlist_t *list, struct tee_shm *shm, uint32_t session_id);

/**
 * @brief Unregisters shm object from the registry list
 *
 * Should be called just before freeing shm object.
 *
 * @param list Pointer to the initialized dlist object, stored in client driver data
 * @param shm Shared memory object pointer to register
 */
void tee_shm_list_unreg(sys_dlist_t *list, struct tee_shm *shm);

/**
 * @brief Cleaning up shm objects that wasn't cleaned
 *
 * Should be called before closing session to clean all remaining shm objects.
 *
 * @param list Pointer to the initialized dlist object, stored in client driver data
 * @param session_id Current session id
 */
void tee_shm_session_clean(sys_dlist_t *list, uint32_t session_id);

#endif /* _TEE_PRIV_H_ */
