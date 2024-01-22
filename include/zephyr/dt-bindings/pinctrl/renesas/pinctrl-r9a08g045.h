/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_R9A08G045_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_R9A08G045_H_

#define RZG_PIN_NUM_IN_PORT 8

/* Port names as labeled in the Hardware Manual */
#define PORT0  0
#define PORT1  1
#define PORT2  2
#define PORT3  3
#define PORT4  4
#define PORT5  5
#define PORT6  6
#define PORT7  7
#define PORT8  8
#define PORT9  9
#define PORT10 10
#define PORT11 11
#define PORT12 12
#define PORT13 13
#define PORT14 14
#define PORT15 15
#define PORT16 16
#define PORT17 17
#define PORT18 18

#define PINCTRL_RZG3S_FUNC_GPIO		0x0

/*
 * Create the pin index from its bank and position numbers and store in
 * the upper 16 bits the alternate function identifier
 *
 * b: port number 0..18
 * p: pin number within port 0..
 * f: pin function
 */
#define RZG3S_PINMUX(b, p, f) ((b) * RZG_PIN_NUM_IN_PORT + (p) | ((f) << 16))

/* Driving Ability Control Register (IOLH_m)
 * Group-A 3.3V
 */
#define PINCTRL_RZG3S_PIN_IOLH_A_3_3V_1900	0
#define PINCTRL_RZG3S_PIN_IOLH_A_3_3V_4000	1
#define PINCTRL_RZG3S_PIN_IOLH_A_3_3V_8000	2
#define PINCTRL_RZG3S_PIN_IOLH_A_3_3V_9000	3
/* Group-A 1.8V */
#define PINCTRL_RZG3S_PIN_IOLH_A_1_8V_2200	0
#define PINCTRL_RZG3S_PIN_IOLH_A_1_8V_4400	1
#define PINCTRL_RZG3S_PIN_IOLH_A_1_8V_9000	2
#define PINCTRL_RZG3S_PIN_IOLH_A_1_8V_10000	3

/* Group-B 3.3V */
#define PINCTRL_RZG3S_PIN_IOLH_B_3_3V_4000	0
#define PINCTRL_RZG3S_PIN_IOLH_B_3_3V_6000	1
#define PINCTRL_RZG3S_PIN_IOLH_B_3_3V_8000	2
#define PINCTRL_RZG3S_PIN_IOLH_B_3_3V_9000	3
/* Group-B 1.8V */
#define PINCTRL_RZG3S_PIN_IOLH_B_1_8V_7000	0
#define PINCTRL_RZG3S_PIN_IOLH_B_1_8V_8000	1
#define PINCTRL_RZG3S_PIN_IOLH_B_1_8V_9000	2
#define PINCTRL_RZG3S_PIN_IOLH_B_1_8V_10000	3

/* Group-C 3.3V */
#define PINCTRL_RZG3S_PIN_IOLH_C_3_3V_4500	0
#define PINCTRL_RZG3S_PIN_IOLH_C_3_3V_5200	1
#define PINCTRL_RZG3S_PIN_IOLH_C_3_3V_5700	2
#define PINCTRL_RZG3S_PIN_IOLH_C_3_3V_6050	3
/* Group-C 2.5V */
#define PINCTRL_RZG3S_PIN_IOLH_C_2_5V_4700	0
#define PINCTRL_RZG3S_PIN_IOLH_C_2_5V_5300	1
#define PINCTRL_RZG3S_PIN_IOLH_C_2_5V_5800	2
#define PINCTRL_RZG3S_PIN_IOLH_C_2_5V_6100	3
/* Group-C 1.8V */
#define PINCTRL_RZG3S_PIN_IOLH_C_1_8V_5200	0
#define PINCTRL_RZG3S_PIN_IOLH_C_1_8V_6000	1
#define PINCTRL_RZG3S_PIN_IOLH_C_1_8V_6550	2
#define PINCTRL_RZG3S_PIN_IOLH_C_1_8V_6800	3

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_R9A08G045_H_ */
