/*
 * Copyright 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/ft5336.h>

#define TOUCH_NODE DT_ALIAS(dut)

const struct device *touch_dev = DEVICE_DT_GET(TOUCH_NODE);

static int x, y;
static uint8_t weight, direction, area, speed;

static const char *get_gest_name(uint8_t gest)
{
	switch (gest) {
	case GEST_MOVE_UP:
		return "MOVE_UP";
	case GEST_MOVE_LEFT:
		return "MOVE_LEFT";
	case GEST_MOVE_DOWN:
		return "MOVE_DOWN";
	case GEST_MOVE_RIGHT:
		return "MOVE_RIGHT";
	case GEST_ZOOM_IN:
		return "ZOOM_IN";
	case GEST_ZOOM_OUT:
		return "ZOOM_OUT";
	}
	return "?";
}

static const char *get_dir_name(uint8_t direction)
{
	switch (direction) {
	case TOUCH_DIR_UP:
		return "UP";
	case TOUCH_DIR_DOWN:
		return "DOWN";
	case TOUCH_DIR_LEFT:
		return "LEFT";
	case TOUCH_DIR_RIGHT:
		return "RIGHT";
	}
	return "?";
}

static void input_cb(struct input_event *evt)
{
	switch (evt->type) {
	case INPUT_EV_KEY:
		if (evt->code != INPUT_BTN_TOUCH) {
			return;
		}
		printk("[%d:%s] x:%d y:%d weight:%d direction:%s area:%d speed:%d\n",
		       evt->value & 0xf, evt->value & 0x10 ? "DOWN" : "UP", x, y,
		       weight, get_dir_name(direction), area, speed);
		break;
	case INPUT_EV_ABS:
		if (evt->code) {
			y = evt->value;
		} else {
			x = evt->value;
		}
		break;
	case INPUT_EV_MSC:
		switch (evt->code) {
		case INPUT_MSC_GESTURE:
			printk("Gestute: [%s]\n", get_gest_name(evt->value));
			break;
		case INPUT_MSC_WEIGHT:
			weight = evt->value;
			break;
		case INPUT_MSC_DIRECTION:
			direction = evt->value;
			break;
		case INPUT_MSC_AREA:
			area = evt->value;
			break;
		case INPUT_MSC_SPEED:
			speed = evt->value;
			break;
		}
		break;
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);

int main(void)
{
	if (!device_is_ready(touch_dev)) {
		printk("Device not ready %s\n", touch_dev->name);
	}
	return 0;
}
