/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZA2_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZA2_GPIO_H_

/**
 * @brief RZ/A2 -specific GPIO Flags
 *
 * The drive flags are encoded in the 8 upper bits of @ref gpio_dt_flags_t as
 * follows:
 *
 * - Bit 8: Drive strength (0=NORMAL, 1=HIGH)
 *
 * @ingroup gpio_interface
 * @{
 */

/**
 * @name RZ/A2 GPIO drive strength flags
 * @brief RZ/A2 GPIO drive strength flags
 * @{
 */

/** @cond INTERNAL_HIDDEN */
/** Drive mode field mask */
#define RZA2_GPIO_DRIVE_MSK 0x0100U
/** @endcond */

/** Normal drive */
#define RZA2_GPIO_DRIVE_NORMAL (0U << 8U)
/** High drive */
#define RZA2_GPIO_DRIVE_HIGH   (1U << 8U)

/** @} */

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZA2_GPIO_H_ */
