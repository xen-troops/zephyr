/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RZA2M_PWM_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RZA2M_PWM_H_

#define PWM_CHANNEL_INPUT_A (0 << 31)
#define PWM_CHANNEL_INPUT_B (1 << 31)

#define PWM_CHANNEL_PROTECTED (1 << 30)

#define PWM_PROTECTED(x) (x | PWM_CHANNEL_PROTECTED)

#define PWM_HIGH BIT(0)
#define PWM_LOW BIT(1)
#define PWM_DT BIT(2)

#define MTU_PWM_CH_IO_A 0
#define MTU_PWM_CH_IO_B 1
#define MTU_PWM_CH_IO_C 2
#define MTU_PWM_CH_IO_D 3

#define MTU_PWM_CH_IO_U 0
#define MTU_PWM_CH_IO_V 1
#define MTU_PWM_CH_IO_W 2

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RZA2M_PWM_H_ */
