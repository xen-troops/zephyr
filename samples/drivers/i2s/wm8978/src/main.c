/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wm8978_sample);

#include "LR_44_1K16B_S.dat"

#define BLOCK_SIZE		4096
#define NUM_TX_BLOCKS		8
#define SAMPLE_RATE		44100
#define SAMPLE_BIT_WIDTH	16
#define BYTES_PER_SAMPLE	sizeof(int16_t)
#define TIMEOUT			2000

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

uint8_t rx_buffer[BLOCK_SIZE];

const struct device *const dev_codec = DEVICE_DT_GET(DT_NODELABEL(codec));
const struct device *const dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2s_node0));

K_MEM_SLAB_DEFINE(tx_mem_slab, BLOCK_SIZE, NUM_TX_BLOCKS, 32);
K_MEM_SLAB_DEFINE(rx_mem_slab, BLOCK_SIZE, NUM_TX_BLOCKS, 32);

static volatile int stop_play;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	stop_play = 1;
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);
	i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_STOP);
}

int configure_stream(const struct device *dev_i2s, enum i2s_dir dir, int master)
{
	int ret;
	struct i2s_config i2s_cfg;

	i2s_cfg.word_size = SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = SAMPLE_RATE;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = TIMEOUT;

	if (master) {
		/* Configure Master */
		i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	}

	i2s_cfg.mem_slab = (dir == I2S_DIR_RX) ? &rx_mem_slab : &tx_mem_slab;
	ret = i2s_configure(dev_i2s, dir, &i2s_cfg);
	if (ret < 0) {
		printk("Failed to configure I2S %s stream (%d)\n",
		       (dir == I2S_DIR_RX) ? "RX" : (dir == I2S_DIR_RX) ? "TX" : "BOTH", ret);
		return ret;
	}
	return 0;
}

int main(void)
{
	int ret;

	printk("sizeof %d\n", (int)sizeof(LR_44_1K16B_S));
	printk("codec device %s\n", dev_codec->name);
	printk("i2s device %s\n", dev_i2s->name);

	stop_play = 0;
	audio_codec_configure(dev_codec, NULL);
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_MODE_ENABLE_ONLY);
#else
	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
#endif
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);
	return 0;
}

static int shell_play(const struct shell *p_shell, size_t argc, char **argv)
{
	uint8_t *buf = (uint8_t *)LR_44_1K16B_S;
	int i, ret;
	int buf_num;

	if (configure_stream(dev_i2s, I2S_DIR_TX, 1)) {
		printk("i2S device can't be configured\n");
		return 0;
	}

	shell_print(p_shell, "Press key (SW4SW) to terminate demo");

	i = 0;
	ret = i2s_buf_write(dev_i2s, buf, BLOCK_SIZE);
	if (ret) {
		printk("Write buffer error %d\n", ret);
		return 0;
	}
	buf += BLOCK_SIZE;
	i += BLOCK_SIZE;
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret) {
		printk("Start buffer error %d\n", ret);
		return 0;
	}
	buf_num = 1;
	stop_play = 0;
	while (i < sizeof(LR_44_1K16B_S)) {
		int count = 0;

		if (stop_play) {
			break;
		}
		ret = i2s_buf_write(dev_i2s, buf, BLOCK_SIZE);
		if (ret) {
			if (ret == -ENOMEM) {
				if (count++ < 100) {
					continue;
				}
			}
			printk("Write buffer %d error %d\n", buf_num, ret);
			break;
		}
		buf += BLOCK_SIZE;
		i += BLOCK_SIZE;
		buf_num++;
	}
	shell_print(p_shell, "Play finished\n");
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
	return 0;
}

static int shell_record(const struct shell *p_shell, size_t argc, char **argv)
{
	int ret;
	uint32_t rx_size;

	if (configure_stream(dev_i2s, I2S_DIR_RX, 0)) {
		printk("i2S device can't be configured (RX)\n");
		return 0;
	}

	if (configure_stream(dev_i2s, I2S_DIR_TX, 1)) {
		printk("i2S device can't be configured (TX)\n");
		return 0;
	}

	shell_print(p_shell, "Press key (SW4) to terminate demo");

	stop_play = 0;

	ret = i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret) {
		shell_print(p_shell, "RX start failed %d\n", ret);
		stop_play = 1;
	}

	for (int i = 0; i < 3; i++) {
		ret = i2s_buf_read(dev_i2s, rx_buffer, &rx_size);
		if (ret < 0) {
			if (ret != -EAGAIN) {
				shell_print(p_shell, "Read error %d\n", ret);
				goto out;
			}
			continue;
		}

		ret = i2s_buf_write(dev_i2s, rx_buffer, rx_size);
		if (ret < 0) {
			shell_print(p_shell, "Write error %d\n", ret);
			goto out;
		}
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret) {
		shell_print(p_shell, "TX start failed %d\n", ret);
		goto out;
	}

	while (!stop_play) {
		ret = i2s_buf_read(dev_i2s, rx_buffer, &rx_size);
		if (ret < 0) {
			if (ret != -EAGAIN) {
				shell_print(p_shell, "Read error %d\n", ret);
				goto out;
			}
			continue;
		}

		ret = i2s_buf_write(dev_i2s, rx_buffer, BLOCK_SIZE);
		if (ret < 0) {
			shell_print(p_shell, "Write error %d\n", ret);
			break;
		}
	}
out:
	shell_print(p_shell, "Record/Playback finished\n");
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
	i2s_trigger(dev_i2s, I2S_DIR_RX, I2S_TRIGGER_DROP);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(subcmd_codec,
	SHELL_CMD_ARG(play, NULL, " Play sample\n", shell_play, 1, 0),
	SHELL_CMD_ARG(record, NULL, " Record\n", shell_record, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(codec, &subcmd_codec, "CODEC test commands", NULL, 3, 0);
