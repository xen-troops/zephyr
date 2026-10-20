/*
 * Copyright (c) 2026 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/tee.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tee_client_api.h>
#include "tee_supplicant.h"

/* BINARY_PREFIX is expected by teec_trace.h */
#ifndef BINARY_PREFIX
#define BINARY_PREFIX           "teec_trace"
#endif
#include <teec_trace.h>

LOG_MODULE_REGISTER(tee_client_api);

static K_MUTEX_DEFINE(teec_mutex);
static K_MUTEX_DEFINE(ckteec_mutex);
static K_MUTEX_DEFINE(shm_mutex);

static void teec_open_session_arg_to_tee(const struct teec_open_session_arg * const teec,
					 struct tee_open_session_arg * const tee)
{
	memcpy(tee->uuid, teec->uuid, sizeof(tee->uuid));
	memcpy(tee->clnt_uuid, teec->clnt_uuid, sizeof(tee->clnt_uuid));
	tee->clnt_login = teec->clnt_login;
}


static void teec_open_session_arg_from_tee(struct teec_open_session_arg * const teec,
					   const struct tee_open_session_arg * const tee)
{
	teec->ret = tee->ret;
	teec->ret_origin = tee->ret_origin;
}

static void teec_invoke_func_arg_to_tee(const struct teec_invoke_func_arg * const teec,
					struct tee_invoke_func_arg * const tee)
{
	tee->session = teec->session;
        tee->func = teec->func;
}

static void teec_invoke_func_arg_from_tee(struct teec_invoke_func_arg * const teec,
					  const struct tee_invoke_func_arg * const tee)
{
	teec->ret = tee->ret;
        teec->ret_origin = tee->ret_origin;
}

static void teec_param_to_tee(const struct teec_param * const teec, struct tee_param * const tee,
                              unsigned int num_param)
{
        for (size_t i = 0; i < num_param; i++) {
                tee[i].attr = teec[i].attr;
                tee[i].a = teec[i].a;
                tee[i].b = teec[i].b;
                tee[i].c = teec[i].c;
        }
}

static void teec_param_from_tee(struct teec_param * const teec, const struct tee_param * const tee,
                                unsigned int num_param)
{
        for (size_t i = 0; i < num_param; i++) {
                teec[i].attr = tee[i].attr;
                teec[i].a = tee[i].a;
                teec[i].b = tee[i].b;
                teec[i].c = tee[i].c;
        }
}

int teec_mutex_lock(void)
{
	return k_mutex_lock(&teec_mutex, K_FOREVER);
}

int teec_mutex_unlock(void)
{
	return k_mutex_unlock(&teec_mutex);
}

int ckteec_mutex_lock(void)
{
	return k_mutex_lock(&ckteec_mutex, K_FOREVER);
}

int ckteec_mutex_unlock(void)
{
	return k_mutex_unlock(&ckteec_mutex);
}

int shmteec_mutex_lock(void)
{
	return k_mutex_lock(&shm_mutex, K_FOREVER);
}

int shmteec_mutex_unlock(void)
{
	return k_mutex_unlock(&shm_mutex);
}

void *teec_malloc(size_t size)
{
        return k_malloc(size);
}

void teec_free(void *ptr)
{
        k_free(ptr);
}

int teec_open_dev(struct device const **dev, const char *capabilities,
			 uint32_t *gen_caps)
{
	struct tee_version_info info;
	int ret;

	memset(&info, 0, sizeof(info));
	*dev = DEVICE_DT_GET_ONE(linaro_optee_tz);
	ret = tee_get_version(*dev, &info);
	if (ret) {
		return -1;
	}

	/* We can only handle GP TEEs */
	if (!(info.gen_caps & TEE_GEN_CAP_GP)) {
		return -1;
	}

	if (capabilities) {
		if (strcmp(capabilities, "optee-tz") == 0) {
			if (info.impl_id != TEE_IMPL_ID_OPTEE) {
				return -1;
			}
			if (!(info.impl_caps & TEE_OPTEE_CAP_TZ)) {
				return -1;
			}
		} else {
			/* Unrecognized capability requested */
			return -1;
		}
	}

	*gen_caps = info.gen_caps;
	return 0;
}

int teec_open_session(const struct device *dev,
					  struct teec_open_session_arg* teec_arg,
					  unsigned int num_param, struct teec_param *cparam,
					  uint32_t *session_id)
{
        struct tee_open_session_arg arg = {0};
	struct tee_param param[TEEC_CONFIG_PAYLOAD_REF_COUNT];
        const size_t arg_size = sizeof(struct tee_open_session_arg) +
		TEEC_CONFIG_PAYLOAD_REF_COUNT *
		sizeof(struct tee_param);
	union {
		struct tee_open_session_arg arg;
		uint8_t data[arg_size];
	} buf;

	teec_param_to_tee(cparam, param, num_param);
	teec_open_session_arg_to_tee(teec_arg, &arg);

        memset(&buf, 0, sizeof(buf));

        int rc = tee_open_session(dev, &arg, num_param, param, session_id);
        if (rc)
                LOG_ERR("tee_open_session failed");

	teec_open_session_arg_from_tee(teec_arg, &arg);
        return rc;
}

int teec_close_session(const struct device *dev, uint32_t session_id)
{
        int rc = tee_close_session(dev, session_id);
        if (rc)
		LOG_ERR("Failed to close session 0x%x", session_id);
        return rc;
}

int teec_invoke_func(const struct device *dev, struct teec_invoke_func_arg *teec_arg,
			      unsigned int num_param, struct teec_param *cparam)
{
        struct tee_invoke_func_arg arg = {0};
	struct tee_param param[TEEC_CONFIG_PAYLOAD_REF_COUNT];

	teec_param_to_tee(cparam, param, num_param);
	teec_invoke_func_arg_to_tee(teec_arg, &arg);

	int rc = tee_invoke_func(dev, &arg, num_param, param);
        if (rc)
		LOG_ERR("tee_invoke_func failed");

	teec_param_from_tee(cparam, param, num_param);
	teec_invoke_func_arg_from_tee(teec_arg, &arg);
        return rc;
}

int teec_cancel(const struct device *dev, uint32_t session_id, uint32_t cancel_id)
{
	int rc = tee_cancel(dev, session_id, cancel_id);
        if (rc)
                LOG_ERR("TEE_IOC_CANCEL: %s", strerror(errno));
        return rc;
}

#define TEE_CLIENT_PAGE_SIZE 4096

void *teec_paged_aligned_alloc(size_t sz)
{
	return k_aligned_alloc(TEE_CLIENT_PAGE_SIZE, sz);
}

struct tee_shm *teec_shm_alloc(const struct device *dev, size_t size, uint32_t flags)
{
	struct tee_shm *p;
	int ret;

	ret = tee_shm_alloc(dev, size, flags, &p);
	if (ret < 0)
		return NULL;
	return p;
}

int teec_shm_register(const struct device *dev, void *buf, size_t size, uint32_t flags,
			     struct tee_shm **shm)
{
	return tee_shm_register(dev, buf, size, flags, shm);
}
int teec_shm_unregister(const struct device *dev, struct tee_shm *shm)
{
        return tee_shm_unregister(dev, shm);
}
