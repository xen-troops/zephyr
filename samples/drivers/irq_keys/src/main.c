/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/r7s9210-intc.h>

#define IRQ_KEYS_INST   DT_INST(0, irq_keys)

PINCTRL_DT_DEFINE(IRQ_KEYS_INST);
PINCTRL_DT_DEV_CONFIG_DECLARE(IRQ_KEYS_INST);

/*
 * if we performed the next number of presses on button during NUMBER_OF_MS_FOR_CLICK_IN_ROW
 * this samaple should change IRQ detection mode.
 */
#define NUMBER_OF_CLICK_IN_ROW 3
#define NUMBER_OF_MS_FOR_CLICK_IN_ROW 3000
BUILD_ASSERT(NUMBER_OF_CLICK_IN_ROW > 1, "Number clicks in a row should be greater than 1");

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

enum irq_edge_detection {
	IRQ_FALLING,
	IRQ_RISING,
};

static const char *const det_mode_str[] = {
	"FALLING EDGE",
	"RISING EDGE",
};

/* event from key */
struct irq_key_data {
	uint32_t irq_line;
	uint32_t irq_priority;
	bool change_detection_mode_notify;
	struct k_event key_event;
	unsigned int num_key_events;
	unsigned int num_key_events_in_row;
	unsigned int current_irq_det_mode;
	uint32_t last_trigger_ms;
};

#define DT_IRQ_BY_IDX_OR(node_id, idx, cell, otherwise)						\
	COND_CODE_1(DT_IRQ_HAS_CELL_AT_IDX(node_id, idx, cell),					\
		    (DT_IRQ_BY_IDX_OR(node_id, idx, cell)),					\
		    (otherwise))

#define IRQ_KEYS_CONNECT(node_id)								 \
	IRQ_CONNECT(DT_IRQ_BY_IDX(node_id, 0, irq), DT_IRQ_BY_IDX_OR(node_id, 0, priority, 0),	 \
		    key_isr, DT_IRQ_BY_IDX(node_id, 0, irq),  DT_IRQ_BY_IDX(node_id, 0, flags)); \
	irq_enable(DT_IRQ_BY_IDX(node_id, 0, irq))

#define IRQ_KEYS_INIT_STATE(node_id)							\
	{										\
		.irq_line = DT_IRQ_BY_IDX(node_id, 0, irq),				\
		.current_irq_det_mode = DT_IRQ_BY_IDX(node_id, 0, flags),		\
		.irq_priority = DT_IRQ_BY_IDX_OR(node_id, 0, priority, 0),              \
	}

struct irq_key_data keys_state[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(IRQ_KEYS_INST, IRQ_KEYS_INIT_STATE, (,)),
};

void key_isr(void *arg)
{
	uint32_t current_irq_line = POINTER_TO_UINT(arg);
	struct irq_key_data *key_data = NULL;
	uint32_t current_uptime_ms = 0;

	/* find key event */
	for (unsigned int i = 0; i < ARRAY_SIZE(keys_state); i++) {
		if (keys_state[i].irq_line == current_irq_line) {
			key_data = &keys_state[i];
			break;
		}
	}

	if (key_data == NULL) {
		k_panic();
	}

	current_uptime_ms = k_ticks_to_ms_ceil32(k_uptime_ticks());
	if (key_data->last_trigger_ms) {
		uint32_t diff_ms = current_uptime_ms - key_data->last_trigger_ms;

		if (diff_ms <= NUMBER_OF_MS_FOR_CLICK_IN_ROW) {
			key_data->num_key_events_in_row++;
			if (key_data->num_key_events_in_row == NUMBER_OF_CLICK_IN_ROW) {
				key_data->current_irq_det_mode++;
				key_data->current_irq_det_mode %= ARRAY_SIZE(det_mode_str);
				key_data->change_detection_mode_notify = true;
				key_data->num_key_events_in_row = 0;
			}
		} else {
			key_data->num_key_events_in_row = 1;
			key_data->last_trigger_ms = current_uptime_ms;
		}
	} else {
		key_data->num_key_events_in_row++;
		key_data->last_trigger_ms = current_uptime_ms;
	}

	key_data->num_key_events++;
	k_event_set(&key_data->key_event, key_data->num_key_events);
}

void key_thread_handler(void *key_data_state, void *unused1, void *unused2)
{
	struct irq_key_data *key_data = key_data_state;

	while (true) {
		uint32_t events = k_event_wait(&key_data->key_event, ~0, true, K_FOREVER);

		LOG_INF("Button (irq line %u) pressed %u times", key_data->irq_line, events);

		irq_disable(key_data->irq_line);
		if (key_data->change_detection_mode_notify) {
			key_data->change_detection_mode_notify = false;
			z_arm_irq_priority_set(key_data->irq_line, key_data->irq_priority,
					       (1 << (key_data->current_irq_det_mode + 2)));
			LOG_INF("Changing of IRQ line (%d) detection mode to %s",
				key_data->irq_line, det_mode_str[key_data->current_irq_det_mode]);
		}
		irq_enable(key_data->irq_line);
	}
}

int main(void)
{
	int ret = 0;
	const unsigned int num_keys = ARRAY_SIZE(keys_state);

	LOG_INF("Starting IRQ keys sample...\nNumber of IRQ Keys detected %u", num_keys);

	/* configure pins as IRQ */
	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(IRQ_KEYS_INST), PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Error can't apply pinctrl state");
		return -EINVAL;
	}

	/* connect and enabled irqs for all keys */
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(IRQ_KEYS_INST, IRQ_KEYS_CONNECT, (;));

	for (unsigned int i = 0; i < num_keys; i++) {
		switch (keys_state[i].current_irq_det_mode) {
		case IRQ_TYPE_EDGE_FALLING:
			keys_state[i].current_irq_det_mode = IRQ_FALLING;
			break;
		case IRQ_TYPE_EDGE_RISING:
			keys_state[i].current_irq_det_mode = IRQ_RISING;
			break;
		default:
			break;
		}

		k_event_init(&keys_state[i].key_event);

		/*
		 * Create a thread for each key, except for the last one,
		 * which will be handled by the main thread
		 */
		if (i != num_keys - 1) {
			struct z_thread_stack_element *stack;
			struct k_thread *thread_data;

			stack = k_aligned_alloc(Z_KERNEL_STACK_OBJ_ALIGN,
						Z_KERNEL_STACK_SIZE_ADJUST(STACKSIZE));
			if (stack == NULL) {
				LOG_ERR("Can't allocate memory for thread stack");
				return -ENOMEM;
			}

			thread_data = k_malloc(sizeof(*thread_data));
			if (thread_data == NULL) {
				LOG_ERR("Can't allocate memory for thread data");
				return -ENOMEM;
			}

			k_thread_create(thread_data, stack, STACKSIZE, key_thread_handler,
					&keys_state[i], NULL, NULL, PRIORITY, 0, K_FOREVER);
			k_thread_start(thread_data);
		}
	}

	key_thread_handler(&keys_state[num_keys - 1], NULL, NULL);
	return ret;
}
