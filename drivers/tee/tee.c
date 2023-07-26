/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/tee.h>
#include <zephyr/logging/log.h>

#include "tee_priv.h"

LOG_MODULE_REGISTER(tee);
K_MUTEX_DEFINE(shm_reg_mutex);

int tee_add_shm(const struct device *dev, void *addr, size_t align, size_t size,
		uint32_t flags, struct tee_shm **shmp)
{
	int rc;
	void *p = addr;
	struct tee_shm *shm;

	if (!shmp) {
		return -EINVAL;
	}

	if (flags & TEE_SHM_ALLOC) {
		if (align) {
			p = k_aligned_alloc(align, size);
		} else {
			p = k_malloc(size);
		}
	}

	if (!p) {
		return -ENOMEM;
	}

	shm = k_malloc(sizeof(struct tee_shm));
	if (!shm) {
		rc = -ENOMEM;
		goto err;
	}

	shm->addr = p;
	shm->size = size;
	shm->flags = flags;
	shm->dev = dev;

	if (flags & TEE_SHM_REGISTER) {
		const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

		if (!api->shm_register) {
			rc = -ENOSYS;
			goto err;
		}

		rc = api->shm_register(dev, shm);
		if (rc) {
			goto err;
		}
	}

	*shmp = shm;

	return 0;
err:
	k_free(shm);
	if (flags & TEE_SHM_ALLOC) {
		k_free(p);
	}

	return rc;
}

int tee_rm_shm(const struct device *dev, struct tee_shm *shm)
{
	int rc = 0;

	if (!shm) {
		return -EINVAL;
	}

	if (shm->flags & TEE_SHM_REGISTER) {
		const struct tee_driver_api *api = (const struct tee_driver_api *)dev->api;

		if (api->shm_unregister) {
			/*
			 * We don't return immediately if callback returned error,
			 * just return this code after cleanup.
			 */
			rc = api->shm_unregister(dev, shm);
		} else {
			/*
			 * Set ENOSYS is SHM_REGISTER flag was set, but callback
			 * is not set.
			 */
			rc = -ENOSYS;
		}
	}

	if (shm->flags & TEE_SHM_ALLOC) {
		k_free(shm->addr);
	}

	k_free(shm);

	return rc;
}

static bool tee_shm_is_registered(sys_dlist_t *list, struct tee_shm *shm, uint32_t session_id)
{
	struct tee_shm *iter;

	if (!list || !shm || shm->session != session_id) {
		return false;
	}

	k_mutex_lock(&shm_reg_mutex, K_FOREVER);
	SYS_DLIST_FOR_EACH_CONTAINER(list, iter, node) {
		if (iter == shm) {
			k_mutex_unlock(&shm_reg_mutex);
			return true;
		}
	}
	k_mutex_unlock(&shm_reg_mutex);

	return false;
}

void tee_shm_list_unreg(sys_dlist_t *list, struct tee_shm *shm)
{
	if (!shm) {
		return;
	}

	if (!tee_shm_is_registered(list, shm, shm->session)) {
		return;
	}

	k_mutex_lock(&shm_reg_mutex, K_FOREVER);
	sys_dlist_remove(&shm->node);
	k_mutex_unlock(&shm_reg_mutex);
}

int tee_shm_list_reg(sys_dlist_t *list, struct tee_shm *shm, uint32_t session_id)
{
	if (!shm || !list) {
		return -EINVAL;
	}

	if (tee_shm_is_registered(list, shm, session_id)) {
		return 0;
	}

	k_mutex_lock(&shm_reg_mutex, K_FOREVER);
	shm->session = session_id;
	sys_dlist_append(list, &shm->node);
	k_mutex_unlock(&shm_reg_mutex);

	return 0;
}

void tee_shm_session_clean(sys_dlist_t *list, uint32_t session_id)
{
	struct tee_shm *iter, *next;

	k_mutex_lock(&shm_reg_mutex, K_FOREVER);
	SYS_DLIST_FOR_EACH_CONTAINER_SAFE(list, iter, next, node) {
		if (iter->session == session_id) {
			sys_dlist_remove(&iter->node);
			if (tee_rm_shm(iter->dev, iter)) {
				LOG_ERR("Unable to clean shared memory");
			}
		}
	}
	k_mutex_unlock(&shm_reg_mutex);
}
