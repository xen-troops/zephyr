/*
 * Copyright (c) 2024, EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public Port Output Enable for GPT (POEG) Driver APIs
 */

#ifndef _ZEPHYR_INCLUDE_DRIVERS_POEG_H_
#define _ZEPHYR_INCLUDE_DRIVERS_POEG_H_

typedef void (*poeg_callback_t)(void *args);

/**
 * @brief POEG reset callback handler function signature
 *
 * @note The callback handler may be called in interrupt context.
 *
 * @note Resets POEG flags to enable output for PWM driver
 *
 * @param[in] dev POEG device instance.
 */
typedef void (*poeg_reset_t)(const struct device *dev);

/**
 * @brief POEG callback handler function signature
 *
 * @note Sets callback to call when on Output disable request
 *
 * @param[in] dev POEG device instance.
 * @param[in] cb pointer to the callback function.
 * @param[in] args pointer to the callback argument structure.
 */
typedef void (*poeg_cb_set_t)(const struct device *dev, poeg_callback_t cb, void *args);

/**
 * @brief POEG status handler function signature
 *
 * @note Returns current POEG status flags
 *
 * @param[out] driver specific POEG status flags.
 */
typedef uint32_t (*poeg_status_t)(const struct device *dev);

/** @brief POEG driver API definition. */
__subsystem struct poeg_driver_api {
	poeg_reset_t reset;
	poeg_cb_set_t cb_set;
	poeg_status_t status;
};

/**
 * @brief Reset POEG status flags
 *
 * Resets POEG disable flags to enable PWM output.
 * This should be called after processing disable request to
 * clear POEG status
 *
 * @param[in] dev POEG device instance.
 */
__syscall void poeg_reset(const struct device *dev);

static inline void z_impl_poeg_reset(const struct device *dev)
{
	const struct poeg_driver_api *api = (const struct poeg_driver_api *)dev->api;

	if (!api || !api->reset) {
		return;
	}

	api->reset(dev);
}
/**
 * @brief Set callback to the POEG on disable request
 *
 * Sets the callback on the POEG device to be called on
 * disable request from external pin or from PWM
 *
 * @param[in] dev POEG device instance.
 * @param[in] cb pointer to the callback function.
 * @param[in] args pointer to the callback argument structure.
 */
__syscall void poeg_cb_set(const struct device *dev, poeg_callback_t cb, void *args);

static inline void z_impl_poeg_cb_set(const struct device *dev, void (*cb)(void *), void *args)
{
	const struct poeg_driver_api *api = (const struct poeg_driver_api *)dev->api;

	if (!api || !api->cb_set) {
		return;
	}

	api->cb_set(dev, cb, args);
}

/**
 * @brief POEG status
 *
 * @note Returns current POEG status flags
 *
 * @param[out] driver specific POEG status flags.
 */
__syscall uint32_t poeg_status(const struct device *dev);

static inline uint32_t z_impl_poeg_status(const struct device *dev)
{
	const struct poeg_driver_api *api = (const struct poeg_driver_api *)dev->api;

	if (!api || !api->status) {
		return -ENOSYS;
	}

	return api->status(dev);
}

/**
 * @}
 */

#include <syscalls/poeg.h>

#endif /* _ZEPHYR_INCLUDE_DRIVERS_POEG_H_ */
