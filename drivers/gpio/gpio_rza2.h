/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_RZA2_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_RZA2_H_

#define RZA2_GPIO_LEVEL_HIGH   0b11
#define RZA2_GPIO_LEVEL_NORMAL 0x01

/**
 * @brief Read GPIO port INPUT value
 *
 * @param port number
 *
 * @return port register value
 */
uint8_t rza2_in_get(uint8_t port);

/**
 * @brief Read GPIO port OUTPUT value
 *
 * @param port number
 *
 * @return port register value
 */
uint8_t rza2_out_get(uint8_t port);

/**
 * @brief Write GPIO port OUTPUT value
 *
 * @param port number
 * @param value to set to the port
 */
void rza2_out_set(uint8_t port, uint8_t value);

/**
 * @brief Set GPIO drive level value
 *
 * @param port number
 * @param pin number
 * @param flag to set level,c an be RZA2_GPIO_LEVEL_HIGH or RZA2_GPIO_LEVEL_NORMAL
 *
 * @return 0 on success, -ENOTSUP if setting is not allowed
 */
int rza2_drive_set(uint8_t port, uint8_t pin, uint8_t flag);

/**
 * @brief ISR handler for GPIO port
 * This handler is used by GPIO driver to call for specific gpio port when interrupt
 * raised. Should be implemented on gpio-port driver side
 *
 * @param port device
 * @param pin mask
 *
 * @return 0 on success, -ENOTSUP if setting is not allowed
 */
void rza2_isr_handler(const struct device *port, uint8_t mask);

/**
 * @brief Check if interrupt pin is allowed for GPIO port
 *
 * @param port number
 * @param pin id
 *
 * @return true on success, false otherwise
 */
bool rza2_pin_int_is_allowed(uint32_t port, uint8_t pin);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_RZA2_H_ */
