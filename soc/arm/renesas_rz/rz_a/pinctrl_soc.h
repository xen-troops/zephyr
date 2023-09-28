/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_SOC_ARM_RENESAS_RZA2_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_RENESAS_RZA2_PINCTRL_SOC_H_

#include <zephyr/types.h>

typedef struct pinctrl_soc_pin {
	uint8_t port;
	uint8_t pin;
	uint8_t func;
	uint8_t drive_strength;
} pinctrl_soc_pin_t;

#define RZA2_PINS_PER_PORT 8

/*
 * According to the HW Manual - pinctrl mux function for each port is represented by 3bit field
 * so the maximum value is 7.
 */
#define FUNC_GPIO_INPUT   BIT(3) /* Pin as input */
#define FUNC_GPIO_OUTPUT  BIT(4) /* Pin as output */
#define FUNC_GPIO_INT_EN  BIT(5) /* Enable interrupt for gpio */
#define FUNC_GPIO_INT_DIS BIT(6) /* Disable interrupt for gpio */

/*
 * Use 16 lower bits [15:0] for pin identifier
 * Use 16 higher bits [31:16] for pin mux function
 */
#define MUX_PIN_ID_MASK   GENMASK(15, 0)
#define MUX_FUNC_MASK     GENMASK(31, 16)
#define MUX_FUNC_OFFS     16
#define RZA_FUNC(prop)    ((prop & MUX_FUNC_MASK) >> MUX_FUNC_OFFS)
#define RZA_PINCODE(prop) (prop & MUX_PIN_ID_MASK)
#define RZA_PORT(prop)    ((RZA_PINCODE(prop)) / RZA2_PINS_PER_PORT)
#define RZA_PIN(prop)     ((RZA_PINCODE(prop)) % RZA2_PINS_PER_PORT)

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, state_prop, idx)                                         \
	{                                                                                          \
		.port = RZA_PORT(DT_PROP_BY_IDX(node_id, state_prop, idx)),                        \
		.pin = RZA_PIN(DT_PROP_BY_IDX(node_id, state_prop, idx)),                          \
		.func = RZA_FUNC(DT_PROP_BY_IDX(node_id, state_prop, idx)),                        \
		.drive_strength = DT_PROP_OR(node_id, drive_strength, 0),                          \
	},

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM(DT_PHANDLE(node_id, prop), pinmux, Z_PINCTRL_STATE_PIN_INIT)};

#endif /* ZEPHYR_SOC_ARM_RENESAS_RZA2_PINCTRL_SOC_H_ */
