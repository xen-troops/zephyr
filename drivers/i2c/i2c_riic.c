/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_riic

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

LOG_MODULE_REGISTER(riic);

#include "i2c-priv.h"

#define NUM_SLAVES		3
#define NO_ACTIVE_SLAVE		(-1)

struct riic_config {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;
	const struct pinctrl_dev_config *pcfg;
	void (*init_irq_func)(const struct device *dev);
	uint32_t bitrate;
};

#ifdef CONFIG_I2C_TARGET
struct riic_target_config {
	struct i2c_target_config *slave_cfg;
	bool slave_attached;
	bool first_read;
	bool first_write;
};
#endif

struct riic_data {
	DEVICE_MMIO_RAM; /* Must be first */
	uint32_t clk_rate; /* Peripheral clock frequency */
	uint32_t interrupt_mask;
	uint32_t status_bits;
	struct k_sem int_sem;
	struct k_mutex i2c_lock_mtx; /* For I2C transfer locking mechanism */
	int master_active;
	uint32_t dev_config;
#ifdef CONFIG_I2C_TARGET
	int active_slave_num; /* -1, if no active slave */
	struct riic_target_config slave[NUM_SLAVES];
#endif
};

/* Registers */
#define RIIC_CR1		0x00	/* I²C Bus Control Register 1 */
#define RIIC_CR2		0x04	/* I²C Bus Control Register 2 */
#define RIIC_MR1		0x08	/* I²C Bus Mode Register 1 */
#define RIIC_MR2		0x0c	/* I²C Bus Mode Register 2 */
#define RIIC_MR3		0x10	/* I²C Bus Mode Register 3 */
#define RIIC_FER		0x14	/* I²C Bus Function Enable Register */
#define RIIC_SER		0x18	/* I²C Bus Status Enable Register */
#define RIIC_IER		0x1c	/* I²C Bus Interrupt Enable Register */
#define RIIC_SR1		0x20	/* I²C Bus Status Register 1 */
#define RIIC_SR2		0x24	/* I²C Bus Status Register 2 */
#define RIIC_SAR0		0x28	/* I²C Slave Address Register 0 */
#define RIIC_SAR1		0x2c	/* I²C Slave Address Register 1 */
#define RIIC_SAR2		0x30	/* I²C Slave Address Register 2 */
#define RIIC_BRL		0x34	/* I²C Bus Bit Rate Low-Level Register */
#define RIIC_BRH		0x38	/* I²C Bus Bit Rate High-Level Register */
#define RIIC_DRT		0x3c	/* I²C Bus Transmit Data Register */
#define RIIC_DRR		0x40	/* I²C Bus Receive Data Register */

#define RIIC_CR1_ICE		BIT(7)	/* Bus Interface Enable */
#define RIIC_CR1_IICRST		BIT(6)	/* Bus Interface Internal Reset */
#define RIIC_CR1_CLO		BIT(5)	/* Extra SCL Clock Cycle Output */
#define RIIC_CR1_SOWP		BIT(4)	/* SCLO/SDAO Write Protect */
#define RIIC_CR1_SCLO		BIT(3)	/* SCL Output Control */
#define RIIC_CR1_SDAO		BIT(2)	/* SDA Output Control */
#define RIIC_CR1_SCLI		BIT(1)	/* SCL Bus Input Monitor */
#define RIIC_CR1_SDAI		BIT(0)	/* SDA Bus Input Monitor */

#define RIIC_CR2_BBSY		BIT(7)	/* Bus Busy Detection Flag */
#define RIIC_CR2_MST		BIT(6)	/* Master/Slave Mode */
#define RIIC_CR2_TRS		BIT(5)	/* Transmit/Receive Mode */
#define RIIC_CR2_SP		BIT(3)	/* Stop Condition Issuance Request */
#define RIIC_CR2_RS		BIT(2)	/* Restart Condition Issuance Request */
#define RIIC_CR2_ST		BIT(1)	/* Start Condition Issuance Request */

#define RIIC_MR1_BCWP		BIT(3)	/* BC Write Protect */
#define RIIC_MR1_CKS_MASK	0x70
#define RIIC_MR1_CKS(x)		((((x) << 4) & RIIC_MR1_CKS_MASK) | RIIC_MR1_BCWP)

#define RIIC_MR2_DLCS		BIT(7)	/* SDA Output Delay Clock Source Selection */
#define RIIC_MR2_TMOH		BIT(2)	/* Timeout H Count Control */
#define RIIC_MR2_TMOL		BIT(1)	/* Timeout L Count Control */
#define RIIC_MR2_TMOS		BIT(0)	/* Timeout Detection Time Selection */

#define RIIC_MR3_DMBE		BIT(7)	/* SMBus/I2C Bus Selection */
#define RIIC_MR3_WAIT		BIT(6)	/* WAIT */
#define RIIC_MR3_RDRFS		BIT(5)	/* RDRF Flag Set Timing Selection */
#define RIIC_MR3_ACKWP		BIT(4)	/* ACKBT Write Protect */
#define RIIC_MR3_ACKBT		BIT(3)	/* Transmit Acknowledge */
#define RIIC_MR3_ACKBR		BIT(2)	/* Receive Acknowledge */

#define RIIC_FER_FMPE		BIT(7)	/* Fast-mode Plus Enable */
#define RIIC_FER_SCLE		BIT(6)	/* SCL Synchronous Circuit Enable */
#define RIIC_FER_NFE		BIT(5)	/* Digital Noise Filter Circuit Enable */
#define RIIC_FER_NACKE		BIT(4)	/* NACK Reception Transfer Suspension Enable */
#define RIIC_FER_SALE		BIT(3)	/* Slave Arbitration-Lost Detection Enable */
#define RIIC_FER_NALE		BIT(2)	/* NACK Transmission Arbitration-Lost Detection Enable */
#define RIIC_FER_MALE		BIT(1)	/* Master Arbitration-Lost Detection Enable */
#define RIIC_FER_TMOE		BIT(0)	/* Timeout Function Enable */

#define RIIC_SER_HOAE		BIT(7)	/* Host Address Enable */
#define RIIC_SER_DIE		BIT(5)	/* Device-ID Address Detection Enable */
#define RIIC_SER_GCE		BIT(3)	/* General Call Address Enable */
#define RIIC_SER_SAR2		BIT(2)	/* Slave Address Register 2 Enable */
#define RIIC_SER_SAR1		BIT(1)	/* Slave Address Register 1 Enable */
#define RIIC_SER_SAR0		BIT(0)	/* Slave Address Register 0 Enable */
#define RIIC_SER_SLAVE_MASK	(RIIC_SER_SAR0 | RIIC_SER_SAR1 | RIIC_SER_SAR2)

#define RIIC_IER_TIE		BIT(7)	/* Transmit Data Empty Interrupt Enable */
#define RIIC_IER_TEIE		BIT(6)	/* Transmit End Interrupt Enable */
#define RIIC_IER_RIE		BIT(5)	/* Receive Data Full Interrupt Enable */
#define RIIC_IER_NAKIE		BIT(4)	/* NACK Reception Interrupt Enable */
#define RIIC_IER_SPIE		BIT(3)	/* Stop Condition Detection Interrupt Enable */
#define RIIC_IER_STIE		BIT(2)	/* Start Condition Detection Interrupt Enable */
#define RIIC_IER_ALIE		BIT(1)	/* Arbitration-Lost Interrupt Enable */
#define RIIC_IER_TMOIE		BIT(0)	/* Timeout Interrupt Enable */

#define RIIC_SR1_HOA		BIT(7)	/* Host Address Detection Flag */
#define RIIC_SR1_DID		BIT(5)	/* Device-ID Address Detection Flag */
#define RIIC_SR1_GCA		BIT(3)	/* General Call Address Detection Flag */
#define RIIC_SR1_AAS2		BIT(2)	/* Slave Address 2 Detection Flag */
#define RIIC_SR1_AAS1		BIT(1)	/* Slave Address 1 Detection Flag */
#define RIIC_SR1_AAS0		BIT(0)	/* Slave Address 0 Detection Flag */
#define RIIC_SR1_AAS_MASK	(RIIC_SR1_AAS0 | RIIC_SR1_AAS1 | RIIC_SR1_AAS2)

#define RIIC_SR2_TDRE		BIT(7)	/* Transmit Data Empty Flag */
#define RIIC_SR2_TEND		BIT(6)	/* Transmit End Flag */
#define RIIC_SR2_RDRF		BIT(5)	/* Receive Data Full Flag */
#define RIIC_SR2_NACKF		BIT(4)	/* NACK Reception Flag */
#define RIIC_SR2_STOP		BIT(3)	/* Stop Condition Detection Flag */
#define RIIC_SR2_START		BIT(2)	/* Start Condition Detection Flag */
#define RIIC_SR2_AL		BIT(1)	/* Arbitration-Lost Flag */
#define RIIC_SR2_TMOF		BIT(0)	/* Timeout Flag */

#define RIIC_SAR_FS		BIT(15)	/* 7-Bit/10-Bit Address Format Selection */
/* 10-Bit Address LSB The least significant bit (LSB) of a 10-bit slave address is set.*/
#define RIIC_SAR_SVA0		BIT(0)
#define RIIC_SAR_SVA_MASK	0xfe
#define RIIC_SAR_SVA10_MASK	0x1ff
#define RIIC_SAR_SVA_SHIFT	1

#define RIIC_BR_RESERVED	0xe0 /* Should be 1 on writes */

#define MAX_WAIT_US		500

#define TRANSFER_TIMEOUT_MS	10		/* Timeout for @riic_data::transfer_mtx */

static void riic_write(const struct device *dev, uint32_t offs, uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_GET(dev) + offs);
}

static uint32_t riic_read(const struct device *dev, uint32_t offs)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + offs);
}

static int riic_wait_for_clear(const struct device *dev, uint32_t offs, uint16_t mask)
{
	uint32_t timeout = 0;

	while ((riic_read(dev, offs) & mask) && (timeout < 10)) {
		k_busy_wait(USEC_PER_MSEC);
		timeout++;
	}
	if (timeout == 10) {
		return -EIO;
	}
	return 0;
}

static inline void riic_clear_set_bit(const struct device *dev, uint32_t offs,
				      uint16_t clear, uint16_t set)
{
	riic_write(dev, offs, (riic_read(dev, offs) & ~clear) | set);
}

static int riic_wait_for_state(const struct device *dev, uint8_t mask)
{
	struct riic_data *data = dev->data;
	int ret;
	uint32_t int_backup;
	uint16_t timeout = 0;

	data->interrupt_mask = mask;
	data->status_bits = riic_read(dev, RIIC_SR2);
	if (data->status_bits & mask) {
		data->interrupt_mask = 0;
		data->status_bits &= mask;
		return 0;
	}

	/* Reset interrupts semaphore */
	k_sem_reset(&data->int_sem);

	/* Save previous interrupts before modifying */
	int_backup = riic_read(dev, RIIC_IER);

	/* Enable interrupts */
	riic_write(dev, RIIC_IER, mask);

	/* Wait for the interrupts */
	ret = k_sem_take(&data->int_sem, K_USEC(MAX_WAIT_US));

	/* Restore previous interrupts and wait for the changes to take effect */
	riic_write(dev, RIIC_IER, int_backup);
	while ((riic_read(dev, RIIC_IER) != int_backup) && (timeout < 10)) {
		k_msleep(1);
		timeout++;
	}

	if (!ret) {
		return 0;
	}
	data->status_bits = riic_read(dev, RIIC_SR2) & mask;
	if (data->status_bits) {
		data->interrupt_mask = 0;
		return 0;
	}
	return ret;
}

static inline void riic_transmit_ack(const struct device *dev)
{
	riic_clear_set_bit(dev, RIIC_MR3, 0, RIIC_MR3_ACKWP);
	riic_clear_set_bit(dev, RIIC_MR3, RIIC_MR3_ACKBT, 0);
	riic_clear_set_bit(dev, RIIC_MR3, RIIC_MR3_ACKWP, 0);
}

static inline void riic_transmit_nack(const struct device *dev)
{
	riic_clear_set_bit(dev, RIIC_MR3, 0, RIIC_MR3_ACKWP);
	riic_clear_set_bit(dev, RIIC_MR3, 0, RIIC_MR3_ACKBT);
	riic_clear_set_bit(dev, RIIC_MR3, RIIC_MR3_ACKWP, 0);
}

static int riic_finish(const struct device *dev)
{
	riic_clear_set_bit(dev, RIIC_CR2, 0, RIIC_CR2_SP);
	riic_wait_for_state(dev, RIIC_IER_SPIE);
	if (riic_read(dev, RIIC_CR2) & RIIC_SR2_START) {
		riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_START, 0);
	}
	riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_NACKF | RIIC_SR2_STOP, 0);
	return 0;
}

static int riic_set_addr(const struct device *dev, uint16_t chip, uint16_t flags)
{
	uint8_t read = !!(flags & I2C_MSG_READ);
	struct riic_data *data = dev->data;

	k_busy_wait(MAX_WAIT_US);
	if (riic_wait_for_state(dev, RIIC_IER_TIE)) {
		riic_finish(dev);
		return 1;
	}
	/* Set slave address & transfer mode */
	if (flags & I2C_MSG_ADDR_10_BITS) {
		riic_write(dev, RIIC_DRT, 0xf0 | ((chip >> 5) & 0x6) | read);
		riic_wait_for_state(dev, RIIC_IER_TIE);
		riic_write(dev, RIIC_DRT, chip & 0xff);
	} else {
		riic_write(dev, RIIC_DRT, ((chip & 0x7f) << 1) | read);
	}
	riic_wait_for_state(dev, RIIC_IER_NAKIE);
	if (data->status_bits & RIIC_SR2_NACKF) {
		return 1;
	}
	if ((riic_read(dev, RIIC_MR3) & RIIC_MR3_ACKBR) == 0) {
		return 0;
	}
	return 1;
}

static int riic_transfer_msg(const struct device *dev, struct i2c_msg *msg)
{
	uint32_t i;
	int ret = 0;
	uint32_t status;

	if ((msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		if (riic_wait_for_state(dev, RIIC_IER_RIE)) {
			return -EIO;
		}
		if (msg->len == 1) {
			riic_clear_set_bit(dev, RIIC_MR3, 0, RIIC_MR3_WAIT);
		}
		/* Dummy read */
		riic_read(dev, RIIC_DRR);
		for (i = 0; i < msg->len; i++) {
			if (riic_wait_for_state(dev, RIIC_IER_RIE)) {
				status = riic_read(dev, RIIC_SR2);
				return -EIO;
			}
			if (msg->len == i + 2) {
				riic_clear_set_bit(dev, RIIC_MR3, 0, RIIC_MR3_WAIT);
			}
			if (msg->len == i + 1) {
				riic_clear_set_bit(dev, RIIC_CR2, 0, RIIC_CR2_SP);
				riic_transmit_nack(dev);
			} else {
				riic_transmit_ack(dev);
			}

			/* Receive next byte */
			msg->buf[i] = riic_read(dev, RIIC_DRR) & 0xff;
		}
	} else {
		/* Writing as master */
		for (i = 0; i < msg->len; i++) {
			if (riic_wait_for_state(dev, RIIC_IER_TIE)) {
				return -EIO;
			}
			riic_write(dev, RIIC_DRT, msg->buf[i]);
		}
		riic_wait_for_state(dev, RIIC_IER_TEIE);
	}
	return ret;
}

#define OPERATION(msg) (((struct i2c_msg *)msg)->flags & I2C_MSG_RW_MASK)
static int riic_transfer(const struct device *dev,
			 struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	struct riic_data *data = dev->data;
	uint16_t timeout = 0;
	int i;
	int ret = 0;
	struct i2c_msg *current, *next;

	if (!num_msgs) {
		return 0;
	}

	/* Prohibiting simultaneous transfer over the same I2C bus from different threads */
	if (k_mutex_lock(&data->i2c_lock_mtx, K_MSEC(TRANSFER_TIMEOUT_MS))) {
		LOG_ERR("Bus is busy\n");
		return -EIO;
	}

	/* Wait for the bus to be available */
	while ((riic_read(dev, RIIC_CR2) & RIIC_CR2_BBSY) && (timeout < 10)) {
		k_busy_wait(USEC_PER_MSEC);
		timeout++;
	}
	if (timeout == 10) {
		LOG_ERR("Bus is busy\n");
		ret = -EIO;
		goto exit;
	}
	current = msgs;
	current->flags |= I2C_MSG_RESTART;
	for (i = 1; i <= num_msgs; i++) {
		if (i < num_msgs) {
			next = current + 1;
			if (OPERATION(current) != OPERATION(next)) {
				if (!(next->flags & I2C_MSG_RESTART)) {
					ret = -EIO;
					goto exit;
				}
			}
			if (current->flags & I2C_MSG_STOP) {
				ret = -EIO;
				goto exit;
			}
		} else {
			current->flags |= I2C_MSG_STOP;
		}
		current++;
	}
	current = msgs;

	data->master_active = 1;
	do {
		if (msgs->flags & I2C_MSG_RESTART) {
			if (riic_read(dev, RIIC_SR2) & RIIC_SR2_START) {
				/* Generate RESTART condition */
				riic_clear_set_bit(dev, RIIC_CR2, 0, RIIC_CR2_RS);
			} else {
				/* Generate START condition */
				riic_clear_set_bit(dev, RIIC_CR2, 0, RIIC_CR2_ST);
			}

			/* Send slave address */
			if (riic_set_addr(dev, addr, msgs->flags)) {
				riic_finish(dev);
				ret = -EIO; /* No ACK received */
				break;
			}
		}

		/* Transfer data */
		if (msgs->len) {
			ret = riic_transfer_msg(dev, msgs);
			if (ret != 0) {
				break;
			}
		}

		/* Finish the transfer */
		if ((msgs->flags & I2C_MSG_STOP) == I2C_MSG_STOP) {
			ret = riic_finish(dev);
			if (ret != 0) {
				break;
			}
		}

		/* Next message */
		msgs++;
		num_msgs--;
	} while (num_msgs);
	data->master_active = 0;

	/* Complete without error */
exit:
	k_mutex_unlock(&data->i2c_lock_mtx);
	return ret;
}

static int riic_configure(const struct device *dev, uint32_t dev_config)
{
	const struct riic_config *config = dev->config;
	struct riic_data *data = dev->data;
	uint32_t int_ref_clock = data->clk_rate;
	int total_ticks, cks, brl, brh;
	uint32_t scl_rise_ns, scl_fall_ns;
	uint8_t rise_edge_ticks, fall_edge_ticks;
	bool fast_plus = false;

	/* TODO: This will be removed after confirming 10-bit addressing works */
	/* We are not supporting 10-bit addressing */
	if ((dev_config & I2C_ADDR_10_BITS) == I2C_ADDR_10_BITS) {
		return -EIO;
	}

	/* Values taken from I2C-bus specification and user manual (UM10204) */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		scl_rise_ns = 1000;
		scl_fall_ns = 300;
		break;
	case I2C_SPEED_FAST:
		scl_rise_ns = 300;
		scl_fall_ns = 300;
		break;
	case I2C_SPEED_FAST_PLUS:
		scl_rise_ns = 120;
		scl_fall_ns = 120;
		fast_plus = true;
		break;
	default:
		LOG_ERR("%s: supported only I2C_SPEED_STANDARD, I2C_SPEED_FAST and "
				"I2C_SPEED_FAST_PLUS\n", dev->name);
		return -EIO;
	}

	/*
	 * Determine reference clock rate. We must be able to get the desired
	 * frequency with only 62 clock ticks max (31 high, 31 low).
	 * Aim for a duty of 60% LOW, 40% HIGH.
	 */
	total_ticks = DIV_ROUND_UP(data->clk_rate, config->bitrate);

	for (cks = 0; cks < 7; cks++) {
		/*
		 * 60% low time must be less than BRL + 2 + 1
		 * BRL max register value is 0x1F.
		 */
		brl = ((total_ticks * 6) / 10);
		if (brl <= (0x1F + 3))
			break;

		total_ticks /= 2;
		/* Calculate internal reference clock value (IICf) */
		int_ref_clock /= 2;
	}

	if (brl > (0x1F + 3)) {
		LOG_ERR("invalid speed (%lu). Too slow.\n", (unsigned long)config->bitrate);
		return -EIO;
	}

	brh = total_ticks - brl;

	/* Remove automatic clock ticks for sync circuit and NF */
	if (cks == 0) {
		brl -= 4;
		brh -= 4;
	} else {
		brl -= 3;
		brh -= 3;
	}
	/*
	 * Remove clock ticks for rise and fall times. Convert ns to clock
	 * ticks.
	 */
	fall_edge_ticks = DIV_ROUND_CLOSEST((uint64_t) scl_fall_ns * int_ref_clock, 1000000000);
	rise_edge_ticks = DIV_ROUND_CLOSEST((uint64_t) scl_rise_ns * int_ref_clock, 1000000000);
	brl -= fall_edge_ticks;
	brh -= rise_edge_ticks;

	/* Adjust for min register values for when SCLE=1 and NFE=1 */
	if (brl < 1)
		brl = 1;
	if (brh < 1)
		brh = 1;

	/* Prohibiting the bus configuration during transfer. */
	if (k_mutex_lock(&data->i2c_lock_mtx, K_MSEC(TRANSFER_TIMEOUT_MS))) {
		LOG_ERR("Bus is busy\n");
		return -EIO;
	}

	LOG_DBG("i2c-riic: freq=%u, duty=%d, fall=%u, rise=%u, cks=%d, brl=%d, brh=%d\n",
		int_ref_clock / total_ticks, ((brl + 3) * 100) / (brl + brh + 6),
		fall_edge_ticks, rise_edge_ticks, cks, brl, brh);

	riic_clear_set_bit(dev, RIIC_CR1, RIIC_CR1_ICE, 0);
	riic_clear_set_bit(dev, RIIC_CR1, 0, RIIC_CR1_IICRST);
	riic_clear_set_bit(dev, RIIC_CR1, 0, RIIC_CR1_ICE);

	riic_write(dev, RIIC_MR1, RIIC_MR1_CKS(cks));
	riic_write(dev, RIIC_BRH, brh | RIIC_BR_RESERVED);
	riic_write(dev, RIIC_BRL, brl | RIIC_BR_RESERVED);

	riic_write(dev, RIIC_SER, 0);
	riic_write(dev, RIIC_MR3, RIIC_MR3_RDRFS);

	if (fast_plus) {
		riic_clear_set_bit(dev, RIIC_FER, 0, RIIC_FER_FMPE);
	}

	riic_clear_set_bit(dev, RIIC_CR1, RIIC_CR1_IICRST, 0);
	data->master_active = 0;
	data->dev_config = dev_config;
#ifdef CONFIG_I2C_TARGET
	data->active_slave_num = NO_ACTIVE_SLAVE;
#endif

	k_mutex_unlock(&data->i2c_lock_mtx);

	return 0;
}

static int riic_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct riic_data *data = dev->data;

	*dev_config = data->dev_config;
	return 0;
}

static int riic_init(const struct device *dev)
{
	const struct riic_config *config = dev->config;
	struct riic_data *data = dev->data;
	uint32_t bitrate_cfg;
	int ret;

	k_sem_init(&data->int_sem, 0, 1);

	k_mutex_init(&data->i2c_lock_mtx);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Device %s is not ready\n", dev->name);
		return -ENODEV;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Can't apply pinctrl state for %s\n", dev->name);
		return ret;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
	if (ret != 0) {
		LOG_ERR("Can't turn on clock for %s\n", dev->name);
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev, (clock_control_subsys_t)&config->bus_clk,
				     &data->clk_rate);
	if (ret != 0) {
		LOG_ERR("Can't get clock rate for %s\n", dev->name);
		goto err;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);
	ret = riic_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret != 0) {
		LOG_ERR("Can't configure device %s\n", dev->name);
		goto err;
	}

	config->init_irq_func(dev);

	return 0;
err:
	clock_control_off(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
	return ret;
}

#if defined(CONFIG_I2C_TARGET)
static void riic_slave_event(const struct device *dev)
{
	struct riic_data *data = dev->data;
	int slave_num = data->active_slave_num;
	const struct i2c_target_callbacks *slave_cb;
	struct riic_target_config *slave;
	struct i2c_target_config *slave_cfg;
	uint8_t val;
	uint32_t status;

	if (slave_num < 0) {
		return;
	}

	slave = &data->slave[slave_num];
	slave_cfg = slave->slave_cfg;

	if (!slave_cfg) {
		LOG_WRN("No config for slave %d", slave_num);
		return;
	}
	slave_cb = slave_cfg->callbacks;

	status = riic_read(dev, RIIC_SR2);

	if (status & RIIC_SR2_RDRF) {
		/* RX data is available, read it and issue write callback */
		val = riic_read(dev, RIIC_DRR);
		if (slave->first_write) {
			/* First byte received (device address + W/R) */
			slave->first_write = false;
			if (slave_cb->write_requested &&
			    slave_cb->write_requested(slave_cfg)) {
				/* NAK further bytes */
				riic_transmit_nack(dev);
				return;
			}
		} else {
			if (slave_cb->write_received &&
			    slave_cb->write_received(slave_cfg, val)) {
				/* NAK further bytes */
				riic_transmit_nack(dev);
				return;
			}
		}
		riic_transmit_ack(dev);
		return;
	}

	if (status & RIIC_SR2_TDRE) {
		/* TX data requested, issue read callback and write out */
		if (slave->first_read) {
			/* First byte will send */
			slave->first_read = false;
			if (slave_cb->read_requested &&
			    !slave_cb->read_requested(slave_cfg, &val)) {
				/* No error, send byte */
				riic_write(dev, RIIC_DRT, val);
			}
		} else {
			if (slave_cb->read_processed &&
			    !slave_cb->read_processed(slave_cfg, &val)) {
				/* No error, send byte */
				riic_write(dev, RIIC_DRT, val);
			}
		}
		return;
	}

	/* STOP event handler must be located before the NACK event handler */
	if (status & RIIC_SR2_STOP) {
		/* Transaction stopped */
		slave_cb->stop(slave_cfg);
		slave->first_read = true;
		slave->first_write = true;
		data->active_slave_num = NO_ACTIVE_SLAVE;
		riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_NACKF | RIIC_SR2_STOP, 0);
		return;
	}

	if (status & RIIC_SR2_NACKF) {
		/* NACK from master. Dummy read DRR to release SCL line */
		riic_read(dev, RIIC_DRR);
		return;
	}
}

/* Attach and start I2C as slave */
int riic_target_register(const struct device *dev, struct i2c_target_config *config)
{
	struct riic_data *data = dev->data;
	int slave_num = -1;
	uint32_t val;
	int ret = 0;

	if (!config) {
		return -EINVAL;
	}

	/* Prohibiting the target configuration during transfer. */
	if (k_mutex_lock(&data->i2c_lock_mtx, K_MSEC(TRANSFER_TIMEOUT_MS))) {
		LOG_ERR("Bus is busy\n");
		return -EBUSY;
	}

	val = (~riic_read(dev, RIIC_SER)) & RIIC_SER_SLAVE_MASK;
	slave_num = __builtin_ffs(val) - 1;
	if (slave_num < 0) {
		ret = -EBUSY;
		goto unlock;
	}

	if (config->flags == I2C_TARGET_FLAGS_ADDR_10_BITS) {
		val = (config->address & RIIC_SAR_SVA10_MASK) | RIIC_SAR_FS;
	} else {
		val = (config->address << RIIC_SAR_SVA_SHIFT) & RIIC_SAR_SVA_MASK;
	}
	riic_write(dev, RIIC_SAR0 + slave_num * 4, val);
	riic_clear_set_bit(dev, RIIC_SER, 0, BIT(slave_num));
	data->slave[slave_num].slave_cfg = config;
	data->slave[slave_num].slave_attached = true;
	data->slave[slave_num].first_read = true;
	data->slave[slave_num].first_write = true;

	/* Enable interrupts  */
	riic_write(dev, RIIC_IER, RIIC_IER_RIE | RIIC_IER_TIE | RIIC_IER_NAKIE | RIIC_IER_SPIE);

	LOG_DBG("i2c: target registered. Address %x", config->address);

unlock:
	k_mutex_unlock(&data->i2c_lock_mtx);
	return ret;
}

int riic_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	struct riic_data *data = dev->data;
	int i;
	int slave_num = -1;
	int ret = 0;

	/* Prohibiting the target configuration during transfer. */
	if (k_mutex_lock(&data->i2c_lock_mtx, K_MSEC(TRANSFER_TIMEOUT_MS))) {
		LOG_ERR("Bus is busy\n");
		return -EBUSY;
	}

	for (i = 0; i < NUM_SLAVES; i++) {
		if (data->slave[i].slave_attached && data->slave[i].slave_cfg == config) {
			slave_num = i;
			break;
		}
	}
	if (slave_num < 0) {
		ret = -EINVAL;
		goto unlock;
	}

	if (data->active_slave_num == slave_num) {
		ret = -EBUSY;
		goto unlock;
	}

	riic_clear_set_bit(dev, RIIC_SER, BIT(slave_num), 0);
	data->slave[i].slave_attached = false;
	data->slave[i].slave_cfg = NULL;

	/* Disable interrupts */
	riic_write(dev, RIIC_IER, 0);

	LOG_DBG("i2c: slave unregistered");

unlock:
	k_mutex_unlock(&data->i2c_lock_mtx);
	return ret;
}
#endif /* defined(CONFIG_I2C_TARGET) */

static void riic_isr(const struct device *dev)
{
	struct riic_data *data = dev->data;
	uint32_t value;

#if defined(CONFIG_I2C_TARGET)
	int slave_num;

	if (!data->master_active) {
		/* Only for slave mode */
		value = riic_read(dev, RIIC_SR1) & RIIC_SR1_AAS_MASK;
		slave_num = __builtin_ffs(value) - 1;
		if (slave_num >= 0) {
			/* Save active slave number because we cannot read it after stop bit */
			data->active_slave_num = slave_num;
		}
		if (data->active_slave_num >= 0) {
			if (data->slave[data->active_slave_num].slave_attached &&
					!data->master_active) {
				riic_slave_event(dev);
			}
		}
		return;
	}
#endif
	/* Only for master mode */
	value = riic_read(dev, RIIC_SR2);
	if (value & data->interrupt_mask) {
		data->status_bits = value & data->interrupt_mask;
		k_sem_give(&data->int_sem);
		data->interrupt_mask = 0;
	}
}

static void riic_isr_tei(const struct device *dev)
{
	riic_isr(dev);
	riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_TEND, 0);
	riic_wait_for_clear(dev, RIIC_SR2, RIIC_SR2_TEND);
}

static void riic_isr_spi(const struct device *dev)
{
	riic_isr(dev);
	riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_STOP, 0);
	riic_wait_for_clear(dev, RIIC_SR2, RIIC_SR2_STOP);
}

static void riic_isr_sti(const struct device *dev)
{
	riic_isr(dev);
	riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_START, 0);
	riic_wait_for_clear(dev, RIIC_SR2, RIIC_SR2_START);
}

static void riic_isr_nacki(const struct device *dev)
{
	riic_isr(dev);

#ifdef CONFIG_I2C_TARGET
	/* In slave mode don't clear the NACKF flag to 0 before the STOP flag is set to 1
	 * because this will cause the transfer start operation.
	 */
	if (((struct riic_data *)dev->data)->active_slave_num == NO_ACTIVE_SLAVE)
#endif
	{
		riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_NACKF, 0);
		riic_wait_for_clear(dev, RIIC_SR2, RIIC_SR2_NACKF);
	}
}

static void riic_isr_ali(const struct device *dev)
{
	riic_isr(dev);
	riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_AL, 0);
	riic_wait_for_clear(dev, RIIC_SR2, RIIC_SR2_AL);
}

static void riic_isr_tmoi(const struct device *dev)
{
	riic_isr(dev);
	riic_clear_set_bit(dev, RIIC_SR2, RIIC_SR2_TMOF, 0);
	riic_wait_for_clear(dev, RIIC_SR2, RIIC_SR2_TMOF);
}

/* TODO: Add asynchronous callback api */
static const struct i2c_driver_api riic_driver_api = {
	.configure = riic_configure,
	.get_config = riic_get_config,
	.transfer = riic_transfer,
#if defined(CONFIG_I2C_TARGET)
	.target_register = riic_target_register,
	.target_unregister = riic_target_unregister,
#endif
};

/* Device Instantiation */
#define RIIC_SET_IRQ(n, name, isr)					\
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq),			\
		    DT_INST_IRQ_BY_NAME(n, name, priority),		\
		    isr,						\
		    DEVICE_DT_INST_GET(n),				\
		    DT_INST_IRQ_BY_NAME(n, name, flags));		\
		irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));

#define RIIC_INIT(n)							\
	static void riic_##n##_init_irq(const struct device *dev)	\
	{								\
		RIIC_SET_IRQ(n, tei, riic_isr_tei);			\
		RIIC_SET_IRQ(n, ri, riic_isr);				\
		RIIC_SET_IRQ(n, ti, riic_isr);				\
		RIIC_SET_IRQ(n, spi, riic_isr_spi);			\
		RIIC_SET_IRQ(n, sti, riic_isr_sti);			\
		RIIC_SET_IRQ(n, nacki, riic_isr_nacki);			\
		RIIC_SET_IRQ(n, ali, riic_isr_ali);			\
		RIIC_SET_IRQ(n, tmoi, riic_isr_tmoi);			\
	}								\
	PINCTRL_DT_INST_DEFINE(n);					\
	static const struct riic_config riic_cfg_##n = {		\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),			\
		.init_irq_func = riic_##n##_init_irq,			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.bitrate = DT_INST_PROP(n, clock_frequency),		\
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),\
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),\
		.bus_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),\
		.bus_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
	};								\
									\
	static struct riic_data riic_data_##n;				\
									\
	I2C_DEVICE_DT_INST_DEFINE(n,					\
			      riic_init,				\
			      NULL,					\
			      &riic_data_##n,				\
			      &riic_cfg_##n,				\
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	\
			      &riic_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RIIC_INIT)
