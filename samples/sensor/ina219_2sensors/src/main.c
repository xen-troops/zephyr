/*
 * Copyright (c) 2021 Leonard Pollak,
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * *************************** INCLUDES *****************************
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/drivers/sensor.h>

/*
 * *************************** CONSTANTS *****************************
 */
/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define INA219_AT40 DT_NODELABEL(ina219addr40)
#define INA219_AT41 DT_NODELABEL(ina219addr41)

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

const struct device *const ina40 = DEVICE_DT_GET(INA219_AT40);
const struct device *const ina41 = DEVICE_DT_GET(INA219_AT41);

/**
 **************************** GLOBAL VARIABLES ****************************
 **/
/* Thread stack defines */
K_THREAD_STACK_DEFINE(thread0_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(thread1_stack_area, STACKSIZE);

K_MUTEX_DEFINE(my_mutex);

struct k_thread thread0_data, thread1_data;

struct led {
	struct gpio_dt_spec spec;
	uint8_t num;
};

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

static const struct led led1 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
	.num = 1,
};

/*
 * ************************* PUBLIC FUNCTIONS ***************************
 */
void sensorFetch(uint8_t id)
{
	const struct device *const ina = (id == 0) ? (ina40) : (ina41);
	struct sensor_value v_bus, power, current;
	int rc;

	rc = sensor_sample_fetch(ina);
	if (rc) {
		printk("Could not fetch sensor data.\n");
		return;
	}

	sensor_channel_get(ina, SENSOR_CHAN_VOLTAGE, &v_bus);
	sensor_channel_get(ina, SENSOR_CHAN_POWER, &power);
	sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &current);

	/* We reduce the output as much as possible because the printk slows down the thread */
	printk("%d\t%d\t%d\t%d\n", id, v_bus.val1, power.val1, current.val1);
}

void threads(void *p1, void *p2, void *p3)
{
	uint8_t id = (uint32_t)p1;
	const struct led *led = (id == 0) ? (&led0) : (&led1);
	uint32_t sleep_ms = 100;
	const struct gpio_dt_spec *spec = &led->spec;
	int ret;

	if (!device_is_ready(spec->port)) {
		printk("Error: %s device is not ready\n", spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d (LED '%d')\n", ret, spec->pin,
		       led->num);
		return;
	}

	gpio_pin_set(spec->port, spec->pin, 0);
	while (1) {
		gpio_pin_toggle(spec->port, spec->pin);
		k_msleep(sleep_ms);
		sensorFetch(id);
	}
}

int main(void)
{
	if (!device_is_ready(ina40)) {
		printk("Device %s is not ready.\n", ina40->name);
		return 0;
	}

	if (!device_is_ready(ina41)) {
		printk("Device %s is not ready.\n", ina41->name);
		return 0;
	}

	/* Print table header */
	printf("ID\tBus[V]\tPow[W]\tCurr[A]\n");

	/* Create and start thread with id=0 */
	k_thread_create(&thread0_data, thread0_stack_area,
			K_KERNEL_STACK_SIZEOF(thread0_stack_area), threads, (void *)0, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);

	/* Create and start thread with id=1 */
	k_thread_create(&thread1_data, thread1_stack_area,
			K_KERNEL_STACK_SIZEOF(thread1_stack_area), threads, (void *)1, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);

	while (true) {
		k_msleep(100);
	}
}
