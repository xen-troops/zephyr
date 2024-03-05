/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZG3S_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZG3S_GPIO_H_

/**
 * @brief RZ G3S specific GPIO Flags
 *
 * The pin driving ability flags are encoded in the 8 upper bits of @ref gpio_dt_flags_t as
 * follows:
 *
 * - Bit 8: Pin driving ability present
 * - Bit 10..9: Pin driving ability value using PINCTRL_RZG3S_PIN_IOLH_xx macro as specified in
 *              dt-bindings/pinctrl/renesas/pinctrl-r9a08g045.h
 *
 *   Use RZG3S_GPIO_DRIVE_IOLH_SET(iolh_val) macro for the GPIO pin driving ability configuration.
 *
 * @ingroup gpio_interface
 * @{
 */

/**
 * @name RZ G3S pin driving ability flags
 * @brief RZ G3S pin driving ability flags
 * @{
 */

/** @cond INTERNAL_HIDDEN */
/**  Pin driving ability value shift */
#define RZG3S_GPIO_DRIVE_IOLH_SHIFT 9U
#define RZG3S_GPIO_DRIVE_IOLH_MASK  (0x3 << RZG3S_GPIO_DRIVE_IOLH_SHIFT)

/** @endcond */

/**  Pin driving ability present */
#define RZG3S_GPIO_DRIVE_IOLH_PRESENT BIT(8)

/**  Pin driving ability configuration macro */
#define RZG3S_GPIO_DRIVE_IOLH_SET(iolh_val)                                                        \
	(RZG3S_GPIO_DRIVE_IOLH_PRESENT | ((iolh_val) << RZG3S_GPIO_DRIVE_IOLH_SHIFT)))

#define RZG3S_GPIO_DRIVE_IOLH_GET(flags)                                                           \
	(((flags) & RZG3S_GPIO_DRIVE_IOLH_MASK) >> RZG3S_GPIO_DRIVE_IOLH_SHIFT)


/** @} */

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZG3S_GPIO_H_ */
