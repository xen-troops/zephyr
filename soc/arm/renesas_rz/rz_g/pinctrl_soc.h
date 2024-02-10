/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ARM_RENESAS_RZG_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_RENESAS_RZG_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	PINCTRL_RZG3S_TYPE_PINMUX = 0,
	PINCTRL_RZG3S_TYPE_SPIN,
	PINCTRL_RZG3S_TYPE_GRP,
};

enum {
	PINCTRL_RZG3S_ETH0_GRP = 0,
	PINCTRL_RZG3S_ETH1_GRP,
	PINCTRL_RZG3S_SD_CH0_GRP,
	PINCTRL_RZG3S_SD_CH1_GRP,
	PINCTRL_RZG3S_XSPI_GRP,
	PINCTRL_RZG3S_I3C_GRP,
	PINCTRL_RZG3S_GRP_LAST,
};

enum {
	PINCTRL_RZG3S_NMI_SPIN = 0,
	PINCTRL_RZG3S_TMS_SWDIO_SPIN,
	PINCTRL_RZG3S_TDO_SPIN,
	PINCTRL_RZG3S_XSPI_SPCLK_SPIN,
	PINCTRL_RZG3S_XSPI_RESET_SPIN,
	PINCTRL_RZG3S_XSPI_WP_SPIN,
	PINCTRL_RZG3S_XSPI_DS_SPIN,
	PINCTRL_RZG3S_XSPI_CS0_SPIN,
	PINCTRL_RZG3S_XSPI_CS1_SPIN,
	PINCTRL_RZG3S_XSPI_IO0_SPIN,
	PINCTRL_RZG3S_XSPI_IO1_SPIN,
	PINCTRL_RZG3S_XSPI_IO2_SPIN,
	PINCTRL_RZG3S_XSPI_IO3_SPIN,
	PINCTRL_RZG3S_XSPI_IO4_SPIN,
	PINCTRL_RZG3S_XSPI_IO5_SPIN,
	PINCTRL_RZG3S_XSPI_IO6_SPIN,
	PINCTRL_RZG3S_XSPI_IO7_SPIN,
	PINCTRL_RZG3S_WDTOVF_PERROUT_SPIN,
	PINCTRL_RZG3S_SD0_CLK_SPIN,
	PINCTRL_RZG3S_SD0_CMD_SPIN,
	PINCTRL_RZG3S_SD0_RST_SPIN,
	PINCTRL_RZG3S_SD0_DATA0_SPIN,
	PINCTRL_RZG3S_SD0_DATA1_SPIN,
	PINCTRL_RZG3S_SD0_DATA2_SPIN,
	PINCTRL_RZG3S_SD0_DATA3_SPIN,
	PINCTRL_RZG3S_SD0_DATA4_SPIN,
	PINCTRL_RZG3S_SD0_DATA5_SPIN,
	PINCTRL_RZG3S_SD0_DATA6_SPIN,
	PINCTRL_RZG3S_SD0_DATA7_SPIN,
	PINCTRL_RZG3S_SD1_CLK_SPIN,
	PINCTRL_RZG3S_SD1_CMD_SPIN,
	PINCTRL_RZG3S_SD1_DATA0_SPIN,
	PINCTRL_RZG3S_SD1_DATA1_SPIN,
	PINCTRL_RZG3S_SD1_DATA2_SPIN,
	PINCTRL_RZG3S_SD1_DATA3_SPIN,
	PINCTRL_RZG3S_AUDIO_CLK1_SPIN,
	PINCTRL_RZG3S_AUDIO_CLK2_SPIN,
	PINCTRL_RZG3S_I3C_SDA_SPIN,
	PINCTRL_RZG3S_I3C_SCL_SPIN,
	PINCTRL_RZG3S_SD2_CMD_SPIN,
	PINCTRL_RZG3S_SD2_DATA0_SPIN,
	PINCTRL_RZG3S_SD2_DATA1_SPIN,
	PINCTRL_RZG3S_SD2_DATA2_SPIN,
	PINCTRL_RZG3S_SD2_DATA3_SPIN,
	PINCTRL_RZG3S_ET0_TXC_TX_CLK_SPIN,
	PINCTRL_RZG3S_ET1_TXC_TX_CLK_SPIN,
	PINCTRL_RZG3S_ET0_TX_CTL_TX_EN_SPIN,
	PINCTRL_RZG3S_ET1_TX_CTL_TX_EN_SPIN,
	PINCTRL_RZG3S_SPIN_LAST,
};

#define PINCTRL_RZG3S_ETH0_GRP_PWR_3300		0
#define PINCTRL_RZG3S_ETH0_GRP_PWR_1800		1
#define PINCTRL_RZG3S_ETH0_GRP_PWR_2500		2
#define PINCTRL_RZG3S_ETH1_GRP_PWR_3300		0
#define PINCTRL_RZG3S_ETH1_GRP_PWR_1800		1
#define PINCTRL_RZG3S_ETH1_GRP_PWR_2500		2

#define PINCTRL_RZG3S_SD_CH0_GRP_PWR_3300	0
#define PINCTRL_RZG3S_SD_CH0_GRP_PWR_1800	1
#define PINCTRL_RZG3S_SD_CH1_GRP_PWR_3300	0
#define PINCTRL_RZG3S_SD_CH1_GRP_PWR_1800	1

#define PINCTRL_RZG3S_XSPI_GRP_PWR_3300		0
#define PINCTRL_RZG3S_XSPI_GRP_PWR_1800		1
#define PINCTRL_RZG3S_XSPI_GRP_PWR_2500		2

#define PINCTRL_RZG3S_I3C_GRP_PWR_1800		0
#define PINCTRL_RZG3S_I3C_GRP_PWR_1200		1

/* pinmux configuration */
struct pinctrl_soc_rzg3s_pinmux {
	uint8_t	port;
	uint8_t	pin;
	uint8_t	func;
	uint8_t	pull_down:1;
	uint8_t	pull_up:1;
	uint8_t	pull_pin_default:1;
	uint8_t	drive_strength:1;
	uint8_t drive_strength_microamp:2;
};

/* Special Purpose pins configuration */
struct pinctrl_soc_rzg3s_spin {
	uint8_t	spin;
	uint8_t	input_enable:1;
	uint8_t	input_disable:1;
	uint8_t output_enable:1;
	uint8_t	drive_strength:1;
	uint8_t drive_strength_microamp:2;
};

/* Pins group configuration */
struct pinctrl_soc_rzg3s_grp {
	uint8_t	grp;
	uint8_t	f_power_source;
	uint8_t	power_source:3;
	uint8_t	bias_high_impedance:1;
	uint8_t bias_disable:1;
	uint8_t low_power_enable:1;
	uint8_t low_power_disable:1;
};

typedef struct pinctrl_soc_pin {
	uint8_t type;
	union {
		struct pinctrl_soc_rzg3s_pinmux	pinmux;
		struct pinctrl_soc_rzg3s_spin spin;
		struct pinctrl_soc_rzg3s_grp grp;
	};
} pinctrl_soc_pin_t;

/* Iterate over each pinctrl-n phandle child */
#define Z_PINCTRL_STATE_PINS_INIT1(node_id, state_prop, idx)                                       \
	DT_FOREACH_CHILD(DT_PHANDLE_BY_IDX(node_id, state_prop, idx),                              \
			 Z_PINCTRL_STATE_PIN_CHILD_INIT)

/* Iterate over each pinctrl-n phandle child */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM_SEP(node_id, prop, Z_PINCTRL_STATE_PINS_INIT1, ())};

/*
 * If child has groups property:
 *   - Iterate over each pin in group and populate pinctrl_soc_pin_t
 * If child has pins property:
 *   - Iterate over each pin in pins and populate pinctrl_soc_pin_t
 */
#define Z_PINCTRL_STATE_PIN_CHILD_INIT(node_id)				\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pinmux),			\
		    (DT_FOREACH_PROP_ELEM(node_id, pinmux, Z_PINCTRL_STATE_PIN_CHILD_PINMUX_INIT)),\
		    ())		\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, groups),		\
		    (Z_PINCTRL_STATE_PIN_CHILD_GROUP_INIT(node_id, groups)), ())	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pins),			\
		(DT_FOREACH_PROP_ELEM(node_id, pins, Z_PINCTRL_STATE_PIN_CHILD_SPIN_INIT)), \
		())

#define RZG_PIN_NUM_IN_PORT 8

/*
 * Use 16 lower bits [15:0] for pin identifier
 * Use 16 higher bits [31:16] for pin mux function
 */
#define MUX_PIN_ID_MASK   GENMASK(15, 0)
#define MUX_FUNC_MASK     GENMASK(31, 16)
#define MUX_FUNC_OFFS     16
#define RZG_FUNC(prop)    (((prop) & MUX_FUNC_MASK) >> MUX_FUNC_OFFS)
#define RZG_PINCODE(prop) ((prop) & MUX_PIN_ID_MASK)
#define RZG_PORT(prop)    ((RZG_PINCODE(prop)) / RZG_PIN_NUM_IN_PORT)
#define RZG_PIN(prop)     ((RZG_PINCODE(prop)) % RZG_PIN_NUM_IN_PORT)

#define Z_PINCTRL_RZG3S_HAS_IOLH(node_id)	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, drive_strength_microamp),	(1), (0))

#define Z_PINCTRL_RZG3S_GET_IOLH(node_id)	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, drive_strength_microamp),	\
		    (DT_PROP(node_id, drive_strength_microamp)), (0))

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_CHILD_PINMUX_INIT(node_id, state_prop, idx)                 \
	{                                                                               \
		.type = PINCTRL_RZG3S_TYPE_PINMUX,	\
		.pinmux = {	\
			.port = RZG_PORT(DT_PROP_BY_IDX(node_id, state_prop, idx)),     \
			.pin = RZG_PIN(DT_PROP_BY_IDX(node_id, state_prop, idx)),       \
			.func = RZG_FUNC(DT_PROP_BY_IDX(node_id, state_prop, idx)),     \
			.pull_down = DT_PROP(node_id, bias_pull_down),			\
			.pull_up = DT_PROP(node_id, bias_pull_up),			\
			.pull_pin_default = DT_PROP(node_id, bias_pull_pin_default),	\
			.drive_strength = Z_PINCTRL_RZG3S_HAS_IOLH(node_id),		\
			.drive_strength_microamp = Z_PINCTRL_RZG3S_GET_IOLH(node_id),	\
		},	\
	},

#define Z_PINCTRL_GRP_HAS_PWR(node_id)	\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, power_source), (1), (0))

#define Z_PINCTRL_PWR(node_id, prop)					\
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, power_source),		\
		    (UTIL_CAT(UTIL_CAT(UTIL_CAT(PINCTRL_RZG3S_,		\
						DT_STRING_UPPER_TOKEN(node_id, prop)),	\
				      _GRP_PWR_),					\
			      DT_PROP(node_id, power_source))),			\
		    ())

/* Process each group using PINCTRL_RZG3S_x_GRP macros defined above */
#define Z_PINCTRL_STATE_PIN_CHILD_GROUP_INIT(node_id, prop)	\
	{                                                                                      \
		.type = PINCTRL_RZG3S_TYPE_GRP,	\
		.grp = {	\
			.grp = UTIL_CAT(PINCTRL_RZG3S_,		\
					UTIL_CAT(DT_STRING_UPPER_TOKEN(node_id, prop), _GRP)),\
			.f_power_source = Z_PINCTRL_GRP_HAS_PWR(node_id),	\
			.power_source = Z_PINCTRL_PWR(node_id, prop), \
			.bias_disable = DT_PROP(node_id, bias_disable),		\
			.bias_high_impedance = DT_PROP(node_id, bias_high_impedance),		\
			.low_power_enable = DT_PROP(node_id, low_power_enable),		\
			.low_power_disable = DT_PROP(node_id, low_power_disable),		\
		},	\
	},

/* Process each spesial pin in pins using PINCTRL_RZG3S_x_SPIN macros defined above */
#define Z_PINCTRL_STATE_PIN_CHILD_SPIN_INIT(node_id, prop, idx)	\
	{                                                                                      \
		.type = PINCTRL_RZG3S_TYPE_SPIN,	\
		.spin = {	\
			.spin = UTIL_CAT(PINCTRL_RZG3S_,	\
					 UTIL_CAT(DT_STRING_UPPER_TOKEN_BY_IDX(node_id, prop, idx),\
						  _SPIN)),\
			.input_enable = DT_PROP(node_id, input_enable),		\
			.input_disable = DT_PROP(node_id, input_disable),	\
			.output_enable = DT_PROP(node_id, output_enable),	\
			.drive_strength = Z_PINCTRL_RZG3S_HAS_IOLH(node_id),		\
			.drive_strength_microamp = Z_PINCTRL_RZG3S_GET_IOLH(node_id),	\
		},	\
	},

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_RENESAS_RZG_PINCTRL_SOC_H_ */
