/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Verify POEG can work in cooperation with PWM.
 *        It controls PWM output using GPIO pin level to
 *        enable or disable PWM output.
 *
 * @details
 * - Test Steps
 *   -# Bind PWM_3 port 3.
 *   -# Use logic analyzer or other instruments to measure the output
 *	from PWM_OUT_0 and GPIO pin.
 * - Expected Results
 *     Logic analizer should show the following scheme:
 *        PWM_3:  _______|||||__|||||
 *                __   ______    ____
 *        GPIO:     |__|     |__|
 *
 */

#include <zephyr/device.h>
#include <inttypes.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/poeg.h>

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_0), okay)
#error "Define a PWM device"
#endif

#define POEG_NODE DT_INST(0, renesas_rzg3s_poeg)

#if !DT_NODE_HAS_STATUS(POEG_NODE, okay)
#error "Define a PWM device"
#endif

#define PWM_DEV_NODE DT_ALIAS(pwm_0)

#define DEFAULT_PERIOD_CYCLE 64000
#define DEFAULT_PULSE_CYCLE  32000
#define DEFAULT_PERIOD_NSEC  2000000
#define DEFAULT_PULSE_NSEC   1000000

#if defined CONFIG_BOARD_RZ_G3S
/* Default port should be adapted per board to fit the channel
 * associated to the PWM pin. For instance, for following device,
 *       aliases {
 *              pwm-0 = &gpt32e3;
 *       };
 * the following should be used:
 * #define DEFAULT_PWM_PORT 3
 */
#define DEFAULT_PWM_PORT 3
#else
#define DEFAULT_PWM_PORT 0
#endif

#define UNIT_NSECS 1

#define SW_NODE DT_NODELABEL(n)

static const struct gpio_dt_spec gpio = GPIO_DT_SPEC_GET(SW_NODE, gpios);

const struct device *get_pwm_device(void)
{
	return DEVICE_DT_GET(PWM_DEV_NODE);
}

const struct device *get_poeg_device(void)
{
	return DEVICE_DT_GET(POEG_NODE);
}

static int start_pwm(uint32_t port, uint32_t period, uint32_t pulse, uint8_t unit)
{
	TC_PRINT("[PWM]: %" PRIu8 ", [period]: %" PRIu32 ", [pulse]: %" PRIu32 "\n", port, period,
		 pulse);

	const struct device *pwm_dev = get_pwm_device();

	if (!device_is_ready(pwm_dev)) {
		TC_PRINT("PWM device is not ready\n");
		return TC_FAIL;
	}

	if (pwm_set(pwm_dev, port, period, pulse, 0)) {
		TC_PRINT("Fail to set the period and pulse width\n");
		return TC_FAIL;
	}

	return TC_PASS;
}

static void cb(void *par)
{
	const struct device *poeg_dev = get_poeg_device();
	uint32_t status = poeg_status(poeg_dev);

	TC_PRINT("Got POEG status %x\n", status);
	k_busy_wait(1000000);

	gpio_pin_toggle_dt(&gpio);

	poeg_reset(poeg_dev);
}

ZTEST_USER(poeg_basic, test_poeg_trigger)
{
	int ret;
	const struct device *poeg_dev = get_poeg_device();

	ret = gpio_pin_configure_dt(&gpio, GPIO_OUTPUT_ACTIVE);

	poeg_cb_set(poeg_dev, cb, NULL);
	zassert_equal(ret, 0, "Failed to configure the pin");
	k_sleep(K_MSEC(1000));

	zassert_true(start_pwm(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC, DEFAULT_PULSE_NSEC,
			       UNIT_NSECS) == TC_PASS,
		     NULL);
	k_sleep(K_MSEC(1000));
	ret = gpio_pin_toggle_dt(&gpio);
	zassert_equal(ret, 0, "Failed to toggle pin value");
}

ZTEST_SUITE(poeg_basic, NULL, NULL, NULL, NULL, NULL);
