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
 * - Bit 11: Pin Digital Noise Filter ability present
 * - Bit 13..12: Digital Noise Filter Number value
 * - Bit 15..14: Digital Noise Filter Clock Selection value
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

/**  Pin Digital Noise Filter Number */
#define RZG3S_GPIO_FILTER_NUM_SHIFT 12U
#define RZG3S_GPIO_FILTER_NUM_MASK  (0x3 << RZG3S_GPIO_FILTER_NUM_SHIFT)

/**  Pin Digital Noise Filter Clock Selection */
#define RZG3S_GPIO_FILTER_CLK_SHIFT 14U
#define RZG3S_GPIO_FILTER_CLK_MASK  (0x3 << RZG3S_GPIO_FILTER_CLK_SHIFT)

/** @endcond */

/**  Pin driving ability present */
#define RZG3S_GPIO_DRIVE_IOLH_PRESENT BIT(8)

/**  Pin Digital Noise Filter present */
#define RZG3S_GPIO_FILTER_PRESENT BIT(11)

/**  Pin driving ability configuration macro */
#define RZG3S_GPIO_DRIVE_IOLH_SET(iolh_val)                                                        \
	(RZG3S_GPIO_DRIVE_IOLH_PRESENT | ((iolh_val) << RZG3S_GPIO_DRIVE_IOLH_SHIFT)))

#define RZG3S_GPIO_DRIVE_IOLH_GET(flags)                                                           \
	(((flags) & RZG3S_GPIO_DRIVE_IOLH_MASK) >> RZG3S_GPIO_DRIVE_IOLH_SHIFT)

/**  Pin Digital Noise Filter ability configuration macro */
#define RZG3S_GPIO_FILTER_SET(filnum, filclksel)                                                   \
	(RZG3S_GPIO_FILTER_PRESENT | ((filnum) << RZG3S_GPIO_FILTER_NUM_SHIFT) |                   \
	 ((filclksel) << RZG3S_GPIO_FILTER_CLK_SHIFT))

#define RZG3S_GPIO_FILTER_NUM_GET(flags)                                                           \
	(((flags) & RZG3S_GPIO_FILTER_NUM_MASK) >> RZG3S_GPIO_FILTER_NUM_SHIFT)

#define RZG3S_GPIO_FILTER_CLK_GET(flags)                                                           \
	(((flags) & RZG3S_GPIO_FILTER_CLK_MASK) >> RZG3S_GPIO_FILTER_CLK_SHIFT)

/** @} */

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_RZG3S_GPIO_H_ */
