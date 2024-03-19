/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Verify PWM channels A and B work.
 * Includes 3 test steps *
 * @details
 * - Test Steps
 *   -# Bind PWM ch A a and PWM ch B to port 0.
 *   -# Set PWM period and pulse using pwm_set_cycles() or pwm_set().
 *   -# Use multimeter or other instruments to measure the output
 *	from PWM_OUT_0_A and PWM_OUT_0_B.
 * - Expected Results
 *   -# The output of PWM_OUT_0_A and PWM_OUT_0_B will differ according to the value
 *	of period and pulse.
 *	1) OUT_0_A -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	   OUT_O_B -> Always off
 *	2) OUT_0_A -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	   OUT_O_B -> Always off
 *	1) OUT_0_A -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	   OUT_O_B -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	2) OUT_0_A -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	   OUT_O_B -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	3) OUT_0_B -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	   OUT_O_A -> Always off
 *	4) OUT_0_B -> Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	   OUT_O_A -> Always off
 *	5) Always on  ->  Period : Pulse (1 : 1)  ->  3.3V
 *	6) Half on  ->  Period : Pulse (2 : 1)  ->  1.65V
 *	7) Always off  ->  Period : Pulse (1 : 0)  ->  0V
 */

#include <zephyr/device.h>
#include <inttypes.h>
#include <zephyr/dt-bindings/pwm/rza2m_pwm.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm_0), okay)
#define PWM_DEV_NODE DT_ALIAS(pwm_0)
#else
#error "Define a PWM device"
#endif

#define DEFAULT_PERIOD_CYCLE 64000
#define DEFAULT_PULSE_CYCLE 32000
#define DEFAULT_PERIOD_NSEC 2000000
#define DEFAULT_PULSE_NSEC 1000000

#define DEFAULT_PWM_PORT 0
#define PWM_PORT_B PWM_PROTECTED(DEFAULT_PWM_PORT | PWM_CHANNEL_INPUT_B)

#define UNIT_CYCLES	0
#define UNIT_NSECS	1

const struct device *get_pwm_device(void)
{
	return DEVICE_DT_GET(PWM_DEV_NODE);
}

static int test_task(uint32_t port, uint32_t period, uint32_t pulse, uint8_t unit)
{
	TC_PRINT("[PWM]: %x, [period]: %" PRIu32 ", [pulse]: %" PRIu32 "\n",
		port, period, pulse);

	const struct device *pwm_dev = get_pwm_device();

	if (!device_is_ready(pwm_dev)) {
		TC_PRINT("PWM device is not ready\n");
		return TC_FAIL;
	}

	if (unit == UNIT_CYCLES) {
		/* Verify pwm_set_cycles() */
		if (pwm_set_cycles(pwm_dev, port, period, pulse, 0)) {
			TC_PRINT("Fail to set the period and pulse width\n");
			return TC_FAIL;
		}
	} else { /* unit == UNIT_NSECS */
		/* Verify pwm_set() */
		if (pwm_set(pwm_dev, port, period, pulse, 0)) {
			TC_PRINT("Fail to set the period and pulse width\n");
			return TC_FAIL;
		}
	}

	return TC_PASS;
}

ZTEST_USER(pwm_ab, test_pwm_nsec)
{
	/* Period : Pulse (2000000 : 1000000), unit (nsec). Voltage : 1.65V */
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
				DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);
	k_sleep(K_MSEC(1000));

	/* Period : Pulse (2000000 : 2000000), unit (nsec). Voltage : 3.3V */
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
				DEFAULT_PERIOD_NSEC, UNIT_NSECS) == TC_PASS, NULL);
	k_sleep(K_MSEC(1000));

	/* Period : Pulse (2000000 : 0), unit (nsec). Voltage : 0V */
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
				0, UNIT_NSECS) == TC_PASS, NULL);
	k_sleep(K_MSEC(1000));
}

ZTEST_USER(pwm_ab, test_pwm_a)
{
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
				DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));

	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
		DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);
}

ZTEST_USER(pwm_ab, test_pwm_b)
{
	/* Period : Pulse (2000000 : 1000000), unit (nsec). Voltage : 1.65V */
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
				DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));

	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
		DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);
}

ZTEST_USER(pwm_ab, test_pwm_ab)
{
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
				DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
				DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));

	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
		DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
		DEFAULT_PULSE_NSEC, UNIT_NSECS) == TC_PASS, NULL);

	k_sleep(K_MSEC(1000));
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);
	zassert_true(test_task(PWM_PORT_B, DEFAULT_PERIOD_NSEC,
					0, UNIT_NSECS) == TC_PASS, NULL);
}

ZTEST_USER(pwm_ab, test_pwm_cycle)
{
	/* Period : Pulse (64000 : 32000), unit (cycle). Voltage : 1.65V */
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_CYCLE,
				DEFAULT_PULSE_CYCLE, UNIT_CYCLES) == TC_PASS, NULL);
	k_sleep(K_MSEC(1000));

	/* Period : Pulse (64000 : 64000), unit (cycle). Voltage : 3.3V */
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_CYCLE,
				DEFAULT_PERIOD_CYCLE, UNIT_CYCLES) == TC_PASS, NULL);
	k_sleep(K_MSEC(1000));

	/* Period : Pulse (64000 : 0), unit (cycle). Voltage : 0V */
	zassert_true(test_task(DEFAULT_PWM_PORT, DEFAULT_PERIOD_CYCLE,
				0, UNIT_CYCLES) == TC_PASS, NULL);
	k_sleep(K_MSEC(1000));
}
