/*
 * Copyright 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_FT5336_H_
#define ZEPHYR_INCLUDE_FT5336_H_

/* Misc event codes */
#define INPUT_MSC_GESTURE	0x10
#define INPUT_MSC_WEIGHT	0x11
#define INPUT_MSC_DIRECTION	0x12
#define INPUT_MSC_AREA		0x13
#define INPUT_MSC_SPEED		0x14

/* Touch directions */
#define TOUCH_DIR_UP		0x00
#define TOUCH_DIR_DOWN		0x01
#define TOUCH_DIR_LEFT		0x02
#define TOUCH_DIR_RIGHT		0x03

/* Gestures */
#define GEST_NONE		0x00
#define GEST_MOVE_UP		0x10
#define GEST_MOVE_LEFT		0x14
#define GEST_MOVE_DOWN		0x08
#define GEST_MOVE_RIGHT		0x0c
#define GEST_ZOOM_IN		0x48
#define GEST_ZOOM_OUT		0x49

#endif /* ZEPHYR_INCLUDE_FT5336_H_ */
