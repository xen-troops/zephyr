/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_R7S9210_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_R7S9210_H_

#define RZA2_PIN_NUM_IN_PORT 8

/* Port names as labeled in the Hardware Manual */
#define PORT0 0
#define PORT1 1
#define PORT2 2
#define PORT3 3
#define PORT4 4
#define PORT5 5
#define PORT6 6
#define PORT7 7
#define PORT8 8
#define PORT9 9
#define PORTA 10
#define PORTB 11
#define PORTC 12
#define PORTD 13
#define PORTE 14
#define PORTF 15
#define PORTG 16
#define PORTH 17
/* No I */
#define PORTJ 18
#define PORTK 19
#define PORTL 20
#define PORTM 21 /* Pins PM_0/1 are labeled JP_0/1 in HW manual */

#define PORTCKIO             22
#define PORTPPOC             23

#define PIN_POSEL 0 /* Sets function for POSEL0 bits. 00, 01, 10 - 1.8v, 11 - 3.3v */
#define PIN_POC2  1 /* Sets function for SSD host 0, 0 - 1.8v 1 - 3.3v */
#define PIN_POC3  2 /* Sets function for SSD host 1, 0 - 1.8v 1 - 3.3v */

/*
 * Create the pin index from its bank and position numbers and store in
 * the upper 16 bits the alternate function identifier
 */
#define RZA2_PINMUX(b, p, f) ((b) * RZA2_PIN_NUM_IN_PORT + (p) | (f << 16))

#define CKIO_DRV RZA2_PINMUX(PORTCKIO, 0, 0)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_R7S9210_H_ */
