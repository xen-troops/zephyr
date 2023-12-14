/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rza2_usbd

#include <soc.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/init.h>
#include <zephyr/spinlock.h>
#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_rza2);

#include "usb_dc_rza2.h"

#define USB_DEV_NODE DT_CHOSEN(zephyr_usb_device)
BUILD_ASSERT(DT_NODE_HAS_STATUS(USB_DEV_NODE, okay), "No chosen usb device");

#define LOG_DEV_ERR(dev, format, ...)	LOG_ERR("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_WRN(dev, format, ...)	LOG_WRN("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_INF(dev, format, ...)	LOG_INF("%s:"#format, (dev)->name, ##__VA_ARGS__)
#define LOG_DEV_DBG(dev, format, ...)	LOG_DBG("%s:"#format, (dev)->name, ##__VA_ARGS__)

#define INTSTS0_MAGIC 0xF800
#define IRQ_INIT_STATE (BEMPE | BRDYE | DVSE | CTRE | VBSE | NRDYE)

#define USB_EP_IS_CTRL(ep) (USB_EP_GET_IDX(ep) == USB_EP_TYPE_CONTROL)

#define USB_EP_MXPS_CTRL	(64)
#define USB_EP_MXPS_BULK	(1024)
#define USB_EP_MXPS_ISO		(1024)
#define USB_EP_MXPS_INT		(64)

enum usbd_periph_state {
	USBD_DETACH,
	USBD_ATTACH,
};

/*
 * pipe config
 * PIPE0 Control ― ― Fixed to 64 bytes/
 * 256 bytes (CNTMD = 1)
 * PIPE1 Iso/Bulk √ √ (Bulk only) Up to 2 Kbytes
 * PIPE2 Iso/Bulk √ √ (Bulk only) Up to 2 Kbytes
 * PIPE3 Bulk √ √ Up to 2 Kbytes
 * PIPE4 Bulk √ √ Up to 2 Kbytes
 * PIPE5 Bulk √ √ Up to 2 Kbytes
 * PIPE6 Int ― ― Fixed to 64 bytes
 * PIPE7 Int ― ― Fixed to 64 bytes
 * PIPE8 Int ― ― Fixed to 64 bytes
 * PIPE9 Int/Bulk √ (Bulk only) √ (Bulk only) Up to 2 Kbytes
 * PIPE10 Int/Bulk √ (Bulk only) √ (Bulk only) Up to 2 Kbytes
 * PIPE11 Bulk √ √ Up to 2 Kbytes
 * PIPE12 Bulk √ √ Up to 2 Kbytes
 * PIPE13 Bulk √ √ Up to 2 Kbytes
 * PIPE14 Bulk √ √ Up to 2 Kbytes
 * PIPE15 Bulk √ √ Up to 2 Kbytes
 */

enum pipe_num {
	PIPE0 = 0,
	PIPE1,
	PIPE2,
	PIPE3,
	PIPE4,
	PIPE5,
	PIPE6,
	PIPE7,
	PIPE8,
	PIPE9,
	PIPE10,
	PIPE11,
	PIPE12,
	PIPE13,
	PIPE14,
	PIPE15,
	PIPE_MAX
};

enum e_buffer_type {
	B_NONE = 0,
	B_DOUBLE,
	B_DOUBLE_BO,
};

enum e_transfer_mode {
	T_NONE = 0,
	T_CONT,
	T_CONT_BO,
};

static const uint16_t pipe_type_array[] = {
	[USB_DC_EP_BULK] = TYPE_BULK,
	[USB_DC_EP_INTERRUPT] = TYPE_INT,
	[USB_DC_EP_ISOCHRONOUS] = TYPE_ISO,
};

struct usbd_rza2_pipe_config {
	int t_type_mask;
	enum e_buffer_type b_type;
	enum e_transfer_mode t_mode;
	uint32_t buffer_size;
	bool buffer_fixed;
	uint32_t def_buffer_size;
	uint8_t bufnum;
	bool double_buf;
};

static const struct usbd_rza2_pipe_config g_pipe_config[] = {
	[PIPE0] = {BIT(USB_DC_EP_CONTROL), B_NONE, T_NONE, 256, 1, 64, 0x00, false},
	[PIPE1] = {BIT(USB_DC_EP_ISOCHRONOUS) | BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT_BO,
		   KB(2), 0, 1024, 0x08, true},
	[PIPE2] = {BIT(USB_DC_EP_ISOCHRONOUS) | BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT_BO,
		   KB(2), 0, 1024, 0x28, true},
	[PIPE3] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0x48, true},
	[PIPE4] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0x58, true},
	[PIPE5] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0x68, true},
	[PIPE6] = {BIT(USB_DC_EP_INTERRUPT), B_NONE, T_NONE, 64, 1, 64, 0x04, false},
	[PIPE7] = {BIT(USB_DC_EP_INTERRUPT), B_NONE, T_NONE, 64, 1, 64, 0x05, false},
	[PIPE8] = {BIT(USB_DC_EP_INTERRUPT), B_NONE, T_NONE, 64, 1, 64, 0x06, false},
	[PIPE9] = {BIT(USB_DC_EP_INTERRUPT) | BIT(USB_DC_EP_BULK), B_DOUBLE_BO, T_CONT_BO,
		   KB(2), 0, 512, 0x7, true},
	[PIPE10] = {BIT(USB_DC_EP_INTERRUPT) | BIT(USB_DC_EP_BULK), B_DOUBLE_BO, T_CONT_BO,
		    KB(2), 0, 512, 0x88, true},
	[PIPE11] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0x98, true},
	[PIPE12] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0xa8, true},
	[PIPE13] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0xb8, true},
	[PIPE14] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0xc8, true},
	[PIPE15] = {BIT(USB_DC_EP_BULK), B_DOUBLE, T_CONT, KB(2), 0, 512, 0xd8, true},
};

/* PIPEnTRN/PIPEnTRE */
struct pipe_trx_config {
	int trn;
	int tre;
};

static const struct pipe_trx_config g_pipe_trx[] = {
	[PIPE0] = {-1, -1},
	[PIPE1] = {D_PIPE1TRN, D_PIPE1TRE},
	[PIPE2] = {D_PIPE2TRN, D_PIPE2TRE},
	[PIPE3] = {D_PIPE3TRN, D_PIPE3TRE},
	[PIPE4] = {D_PIPE4TRN, D_PIPE4TRE},
	[PIPE5] = {D_PIPE5TRN, D_PIPE5TRE},
	[PIPE6] = {-1, -1},
	[PIPE7] = {-1, -1},
	[PIPE8] = {-1, -1},
	[PIPE9] = {D_PIPE9TRN, D_PIPE9TRE},
	[PIPE10] = {D_PIPEATRN, D_PIPEATRE},
	[PIPE11] = {D_PIPEBTRN, D_PIPEBTRE},
	[PIPE12] = {D_PIPECTRN, D_PIPECTRE},
	[PIPE13] = {D_PIPEDTRN, D_PIPEDTRE},
	[PIPE14] = {D_PIPEETRN, D_PIPEETRE},
	[PIPE15] = {D_PIPEFTRN, D_PIPEFTRE},
};

struct usb_rza2_setup_packet {
	uint8_t requestType;
	uint8_t request;
	uint16_t value;
	uint16_t index;
	uint16_t length;
} __packed;

#define SETUP_PACKET_SIZE (sizeof(struct usb_rza2_setup_packet))

struct usbd_rza2_irq_state {
	uint16_t intsts0;
	uint16_t brdysts;
	uint16_t nrdysts;
	uint16_t bempsts;
};

#define EP_STATE_INVALID ((uint8_t)-1)

struct usbd_rza2_ep_data {
	uint8_t pipe;
	uint8_t state;

	struct k_spinlock lock;
	usb_dc_ep_callback cb;

	bool is_used: 1;
	bool is_enabled: 1;
	bool is_stall: 1;
};

struct usbd_rza2_config {
	DEVICE_MMIO_NAMED_ROM(dev_reg);
	DEVICE_MMIO_NAMED_ROM(host_reg);
	const struct device *clock_dev;
	struct renesas_cpg_clk usb_clk;
	uint32_t clk_freq;
	bool is_usb_x1_src;

	size_t num_of_eps;
	int speed_idx;
	const struct pinctrl_dev_config *pcfg;
	void (*init_irq_func)(const struct device *dev);
};

struct usbd_rza2_pipe_cfg {
	struct usb_dc_ep_cfg_data ep_cfg;
	bool is_used: 1;
	bool is_enabled: 1;
	bool is_zlp_in: 1;
};

struct usbd_rza2_fifo {
	uint32_t port_off;
	uint32_t sel_off;
	uint32_t ctr_off;
	struct k_spinlock lock;
};

struct usbd_rza2_irq_en {
	uint16_t intenb0;
	uint16_t brdyenb;
	uint16_t nrdyenb;
	uint16_t bempenb;
};

enum usb_msg_callback_type {
	USBD_RZA2_CB_EP = 0,
	USBD_RZA2_CB_STATUS
};

struct usbd_rza2_msg {
	enum usb_msg_callback_type type;
	uint16_t ep_addr;
	uint16_t cmd;
	uint16_t data1;
};

struct usb_device_data {
	DEVICE_MMIO_NAMED_RAM(dev_reg);
	DEVICE_MMIO_NAMED_RAM(host_reg);
	struct usbd_rza2_pipe_cfg pipe_cfg[PIPE_MAX];
	struct usbd_rza2_fifo cfifo;
	struct usbd_rza2_ep_data *ep_data_in;
	struct usbd_rza2_ep_data *ep_data_out;
	usb_dc_status_callback status_cb;
	struct usbd_rza2_irq_en irq_en;

	struct k_msgq queue;
	struct usbd_rza2_msg usb_msgs[CONFIG_USBD_RZA2_MESSAGE_COUNT];
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_USB_DC_RZA2_STACK_SIZE);
	bool is_attached: 1;
	bool is_susp: 1;
	bool can_susp: 1;

};

#define DEV_DATA(dev) ((struct usb_device_data *)((dev)->data))
#define DEV_CFG(dev)  ((struct usbd_rza2_config *)((dev)->config))

#define RZA2_DEV_REG(dev)  DEVICE_MMIO_NAMED_GET(dev, dev_reg)
#define RZA2_HOST_REG(dev) DEVICE_MMIO_NAMED_GET(dev, host_reg)

static uint16_t usbf_read(const struct device *dev, uint32_t reg)
{
	mm_reg_t func = RZA2_DEV_REG(dev);

	return sys_read16(func + reg);
}

static void usbf_write(const struct device *dev, uint32_t reg, uint16_t data)
{
	mm_reg_t func = RZA2_DEV_REG(dev);

	sys_write16(data, func + reg);
}

uint32_t usbf_read32(const struct device *dev, uint32_t off)
{
	mm_reg_t func = RZA2_DEV_REG(dev);

	return sys_read32(func + off);
}

static void usbf_write32(const struct device *dev, uint32_t off, uint32_t data)
{
	mm_reg_t func = RZA2_DEV_REG(dev);

	sys_write32(data, func + off);
}

uint8_t usbf_read8(const struct device *dev, uint32_t off)
{
	mm_reg_t func = RZA2_DEV_REG(dev);

	return sys_read8(func + off);
}

static void usbf_write8(const struct device *dev, uint32_t off, uint32_t data)
{
	mm_reg_t func = RZA2_DEV_REG(dev);

	sys_write8(data, func + off);
}

uint32_t usbh_read32(const struct device *dev, uint32_t off)
{
	return sys_read32(RZA2_HOST_REG(dev) + off);
}

static void usbh_write32(const struct device *dev, uint32_t off, uint32_t data)
{
	sys_write32(data, RZA2_HOST_REG(dev) + off);
}

static void usbf_bset(const struct device *dev, uint32_t reg, uint16_t mask, uint16_t data)
{
	uint16_t val = usbf_read(dev, reg);

	val &= ~mask;
	val |= data & mask;

	usbf_write(dev, reg, val);
}

static void usbh_bset32(const struct device *dev, uint32_t reg, uint32_t mask, uint32_t data)
{
	uint32_t val = usbh_read32(dev, reg);

	val &= ~mask;
	val |= data & mask;

	usbh_write32(dev, reg, val);
}

static void usbd_rza2_tx_irq_ctrl(const struct device *dev, uint8_t pipe, int enable);
static void usbd_rza2_rx_irq_ctrl(const struct device *dev, uint8_t pipe, int enable);

static k_spinlock_key_t usbd_rza2_fifo_lock(struct usbd_rza2_fifo *fifo)
{
	return k_spin_lock(&fifo->lock);
}

static void usbd_rza2_fifo_unlock(struct usbd_rza2_fifo *fifo,  k_spinlock_key_t key)
{
	k_spin_unlock(&fifo->lock, key);
}

static int usbd_rza2_fifo_is_ready(const struct device *dev, struct usbd_rza2_fifo *fifo)
{
	uint16_t tmp_reg16;
	int i;

	for (i = 0; i < 4; i++) {
		if (usbf_read(dev, fifo->ctr_off) & CFIFOCTR_FRDY) {
			return 0;
		}
		/* up 100ns delay */
		tmp_reg16 = usbf_read(dev, D_SYSCFG0);
		tmp_reg16 = usbf_read(dev, D_SYSSTS0);
	}

	return -EBUSY;
}

static void usbd_rza2_fifo_wr(const struct device *dev, struct usbd_rza2_fifo *fifo,
				  const uint8_t *buf, uint32_t len)
{
	uint32_t pos = 0;
	uint32_t i;

	if (len >= 4) {
		for (i = 0; i < (len >> 2); i++) {
			uint32_t val = *(uint32_t *)(buf + pos);

			usbf_write32(dev, fifo->port_off, val);
			pos += sizeof(uint32_t);
		}
	}

	if (len - pos >= 2) {
		uint16_t val = *(uint16_t *)(buf + pos);

		usbf_bset(dev, fifo->sel_off, CFIFOSEL_MBW, CFIFOSEL_USB_MBW_16);
		usbf_write(dev, fifo->port_off, val);
		pos += sizeof(uint16_t);
	}

	if (pos < len) {
		usbf_bset(dev, fifo->sel_off, CFIFOSEL_MBW, CFIFOSEL_USB_MBW_8);
		usbf_write8(dev, fifo->port_off, buf[pos]);
		pos++;
	}

	usbf_bset(dev, fifo->sel_off, CFIFOSEL_MBW, CFIFOSEL_USB_MBW_32);
}

static int usbd_rza2_fifo_rd(const struct device *dev, struct usbd_rza2_fifo *fifo,
			     uint8_t *buf, uint32_t len)
{
	uint32_t pos = 0;
	int i, j;

	/* For RX MBW cam't be changed until data is all read, so stick with 32bits */

	if (len >= 4) {
		for (i = 0U; i < (len >> 2); i++) {
			*(uint32_t *)(buf + pos) = usbf_read32(dev, fifo->port_off);
			pos += sizeof(uint32_t);
		}
	}

	if (len & 0x3) {
		/* Not multiple of 4 */
		uint32_t last_dw = usbf_read32(dev, fifo->port_off);

		for (j = 0U; j < (len & 0x3); j++) {
			*(buf + pos + j) = (last_dw >> (j * 8U)) & 0xFF;
		}
	}

	return 0;
}

static int usbd_rza2_fifo_get_rd_len(const struct device *dev, struct usbd_rza2_fifo *fifo)
{
	return usbf_read(dev, fifo->ctr_off) & CFIFOCTR_DTLN;
}

static void usbd_rza2_fifo_clear0(const struct device *dev, struct usbd_rza2_fifo *fifo)
{
	usbf_write(dev, fifo->ctr_off, CFIFOCTR_BCLR);
}

static int usbd_rza2_fifo_sel0(const struct device *dev, struct usbd_rza2_fifo *fifo, int write)
{
	uint16_t mask = (CFIFOSEL_MBW | CFIFOSEL_ISEL | CFIFOSEL_CURPIPE);
	uint16_t r_val = PIPE0 | CFIFOSEL_USB_MBW_32;
	int timeout = 1024;

	r_val |= (write == 1) ? CFIFOSEL_ISEL : 0;

	/* Set CURPIPE and MBW, little endian */
	usbf_write(dev, fifo->sel_off, r_val);

	/* check ISEL and CURPIPE value */
	while (timeout--) {
		if (r_val == (mask & usbf_read(dev, fifo->sel_off))) {
			return 0;
		}
		k_busy_wait(10);
	}

	LOG_DEV_ERR(dev, "fifo0 select error");
	return -EIO;
}

static int usbd_rza2_fifo_sel(const struct device *dev, uint8_t pipe, struct usbd_rza2_fifo *fifo)
{
	uint16_t mask = (CFIFOSEL_MBW | CFIFOSEL_CURPIPE | CFIFOSEL_RCNT);
	uint16_t r_val = pipe | CFIFOSEL_USB_MBW_32 | CFIFOSEL_RCNT;
	int timeout = 1024;

	/* Set CURPIPE and MBW, little endian */
	usbf_write(dev, fifo->sel_off, r_val);

	/* check ISEL and CURPIPE value */
	while (timeout--) {
		if (r_val == (mask & usbf_read(dev, fifo->sel_off))) {
			return 0;
		}
		k_busy_wait(10);
	}

	LOG_DEV_ERR(dev, "fifo select error pipe:%d", pipe);
	return -EIO;
}

static void usbd_rza2_fifo_clear(const struct device *dev, struct usbd_rza2_fifo *fifo)
{
	usbf_write(dev, fifo->ctr_off, CFIFOCTR_BCLR);
}

static void usbd_rza2_cpipe_pid_set(const struct device *dev, uint16_t val)
{
	usbf_bset(dev, D_DCPCTR, PID, val);
}

static uint16_t usbd_rza2_cpipe_pid_get(const struct device *dev)
{
	return usbf_read(dev, D_DCPCTR) & PID;
}

static void usbd_rza2_cpipe_stall2nak(const struct device *dev)
{
	uint16_t pid = usbd_rza2_cpipe_pid_get(dev);

	switch (pid) {
	case PID_STALL11:
		usbd_rza2_cpipe_pid_set(dev, PID_STALL10);
		/* fall-through*/
	case PID_STALL10:
	case PID_BUF:
		usbd_rza2_cpipe_pid_set(dev, PID_NAK);
		break;
	}
}

static int usbd_rza2_cpipe_is_busy(const struct device *dev)
{
	int timeout = 1024;
	uint16_t val;

	do {
		val = usbf_read(dev, D_DCPCTR);
		if (!(val & PBUSY)) {
			return 0;
		}

		k_busy_wait(10);
	} while (timeout--);

	LOG_DEV_ERR(dev, "pipe0 busy wait tmo");
	return -EIO;
}

static void usbd_rza2_cpipe_fifo_clear(const struct device *dev)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_fifo *fifo = &data->cfifo;
	k_spinlock_key_t key;

	key = usbd_rza2_fifo_lock(fifo);

	/* clear DCP FIFO of transmission */
	if (usbd_rza2_fifo_sel0(dev, fifo, 1) < 0) {
		return;
	}
	usbd_rza2_fifo_clear(dev, fifo);

	/* clear DCP FIFO of reception */
	if (usbd_rza2_fifo_sel0(dev, fifo, 0) < 0) {
		return;
	}
	usbd_rza2_fifo_clear(dev, fifo);

	usbd_rza2_fifo_unlock(fifo, key);
}

static int usbd_rza2_cpipe_is_stall(const struct device *dev)
{
	uint16_t pid = usbd_rza2_cpipe_pid_get(dev);

	return (int)(pid == PID_STALL10 || pid == PID_STALL11);
}

static void usbd_rza2_cpipe_stall(const struct device *dev)
{
	uint16_t pid = usbd_rza2_cpipe_pid_get(dev);

	pid &= PID;

	switch (pid) {
	case PID_NAK:
		usbd_rza2_cpipe_pid_set(dev, PID_STALL10);
		break;
	case PID_BUF:
		usbd_rza2_cpipe_pid_set(dev, PID_STALL11);
		break;
	}
}

static int usbd_rza2_cpipe_can_access(const struct device *dev)
{
	return (usbf_read(dev, D_DCPCTR) & BSTS) ? 0 : -EBUSY;
}

static void usbd_rza2_cpipe_set_ccpl(const struct device *dev, uint16_t val)
{
	usbf_bset(dev, D_DCPCTR, CCPL, val);
}

static int usbd_rza2_cpipe_cfg(const struct device *dev,
			       const struct usb_dc_ep_cfg_data * const ep_cfg)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[PIPE0];
	int ret;

	if (pipe_cfg->is_used) {
		/* PIPE0 referenced by 2 IN/OUT eps, but configured only once */
		return 0;
	}

	usbd_rza2_cpipe_stall2nak(dev);
	usbd_rza2_cpipe_pid_set(dev, PID_NAK);
	ret = usbd_rza2_cpipe_is_busy(dev);
	if (ret) {
		return ret;
	}

	pipe_cfg->is_used = true;
	memcpy(&pipe_cfg->ep_cfg, ep_cfg, sizeof(*ep_cfg));

	usbf_write(dev, D_DCPCFG, 0);
	/*  clear data sequence */
	usbf_bset(dev, D_DCPCTR, SQCLR | SQSET, SQCLR);
	/* set max pkt size */
	usbf_bset(dev, D_DCPMAXP, MXPSC, ep_cfg->ep_mps);
	usbd_rza2_cpipe_fifo_clear(dev);

	LOG_DEV_DBG(dev, "CFG: pipe0 cfg D_DCPCTR:%X D_DCPMAXP:%X",
		    usbf_read(dev, D_DCPCTR),
		    usbf_read(dev, D_DCPMAXP));
	return 0;
}

static int usbd_rza2_cpipe_en(const struct device *dev)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[PIPE0];
	int ret;

	if (pipe_cfg->is_enabled) {
		/* PIPE0 referenced by 2 IN/OUT eps, but enabled only once */
		return 0;
	}

	usbd_rza2_cpipe_stall2nak(dev);
	usbd_rza2_cpipe_pid_set(dev, PID_NAK);
	ret = usbd_rza2_cpipe_is_busy(dev);
	if (ret) {
		return ret;
	}

	/*  clear data sequence */
	usbf_bset(dev, D_DCPCTR, SQCLR | SQSET, SQCLR);
	usbd_rza2_cpipe_fifo_clear(dev);

	usbf_bset(dev, D_BRDYSTS, BIT(PIPE0), 0);
	usbd_rza2_rx_irq_ctrl(dev, PIPE0, 1);

	pipe_cfg->is_enabled = true;

	return 0;
}

static void usbd_rza2_cpipe_dis(const struct device *dev)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[PIPE0];
	int ret;

	if (!pipe_cfg->is_enabled) {
		/* PIPE0 referenced by 2 IN/OUT eps, but dis only once */
		return;
	}

	usbd_rza2_cpipe_stall2nak(dev);
	usbd_rza2_cpipe_pid_set(dev, PID_NAK);
	ret = usbd_rza2_cpipe_is_busy(dev);
	if (ret) {
		return;
	}

	memset(pipe_cfg, 0, sizeof(*pipe_cfg));
}

static int usbd_rza2_dpipe_find(const struct device *dev, enum usb_dc_ep_transfer_type type)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = NULL;
	bool found = false;
	int i;

	for (i = 0; i < PIPE_MAX; i++) {
		pipe_cfg = &data->pipe_cfg[i];

		if (pipe_cfg->is_used) {
			continue;
		}

		if (!(g_pipe_config[i].t_type_mask & BIT(type))) {
			continue;
		}

		found = true;
		break;
	}

	if (!found) {
		LOG_DEV_ERR(dev, "unable to find pipe type:%d", type);
		return -EINVAL;
	}

	return i;
}

static void usbd_rza2_dpipe_select(const struct device *dev, uint8_t pipe)
{
	usbf_write(dev, D_PIPESEL, (pipe & PIPESEL));
}

static int usbd_rza2_pipe_get_maxpacket(const struct device *dev, uint8_t pipe)
{
	struct usb_device_data *data = dev->data;

	return data->pipe_cfg[pipe].ep_cfg.ep_mps;
}

static void usbd_rza2_pipe_init(const struct device *dev)
{
	int i;

	usbd_rza2_cpipe_fifo_clear(dev);

	for (i = 1; i < PIPE_MAX; i++) {
		usbf_bset(dev, D_PIPENCTR1(i), SQCLR | SQSET, SQCLR);
		usbf_bset(dev, D_PIPENCTR1(i), ACLRM, ACLRM);
		usbf_bset(dev, D_PIPENCTR1(i), ACLRM, 0);
	}
}

static void usbd_rza2_dpipe_pid_set(const struct device *dev, uint8_t pipe, uint16_t val)
{
	usbf_bset(dev, D_PIPENCTR1(pipe), PID, val);
}

static uint16_t usbd_rza2_dpipe_pid_get(const struct device *dev, uint8_t pipe)
{
	return usbf_read(dev, D_PIPENCTR1(pipe)) & PID;
}

static void usbd_rza2_dpipe_stall2nak(const struct device *dev, uint8_t pipe)
{
	uint16_t pid = usbd_rza2_dpipe_pid_get(dev, pipe);

	switch (pid) {
	case PID_STALL11:
		usbd_rza2_dpipe_pid_set(dev, pipe, PID_STALL10);
		/* fall-through*/
	case PID_STALL10:
	case PID_BUF:
		usbd_rza2_dpipe_pid_set(dev, pipe, PID_NAK);
		break;
	}
}

static int usbd_rza2_dpipe_is_busy(const struct device *dev, uint8_t pipe)
{
	int timeout = 1024;
	uint16_t val;

	do {
		val = usbf_read(dev, D_PIPENCTR1(pipe));
		if (!(val & PBUSY)) {
			return 0;
		}

		k_busy_wait(10);
	} while (timeout--);

	LOG_DEV_ERR(dev, "pipe:%d busy wait tmo", pipe);
	return -EIO;
}

static uint16_t usbd_rza2_dpipe_setup_buf(const struct device *dev, uint8_t pipe)
{
	uint16_t buff_size;
	uint16_t bufnmb;
	uint16_t bufnmb_cnt;

	buff_size = g_pipe_config[pipe].def_buffer_size;
	bufnmb = g_pipe_config[pipe].bufnum;

	/* change buff_size to register value */
	bufnmb_cnt = (buff_size / 64) - 1;

	LOG_DEV_DBG(dev, "pipe:%d : buff_size 0x%x: bufnmb 0x%x\n", pipe, buff_size, bufnmb);

	return ((bufnmb_cnt << BUFSIZE_SHIFT) & BUFSIZE) | (bufnmb & BUFNMB);
}

static void usbd_rza2_dpipe_trn_set(const struct device *dev, uint8_t pipe, uint16_t val)
{
	int reg;

	__ASSERT(pipe < PIPE_MAX, "invalid pipe%d", pipe);

	reg = g_pipe_trx[pipe].trn;

	if (reg < 0) {
		LOG_DEV_DBG(dev, "unable to set trn for pipe %d\n", pipe);
		return;
	}

	usbf_bset(dev, reg, 0xffff, val);
}

static void usbhd_rza2_dpipe_tre_set(const struct device *dev, uint8_t pipe, uint16_t val)
{
	int reg;

	__ASSERT(pipe < PIPE_MAX, "invalid pipe%d", pipe);

	reg = g_pipe_trx[pipe].tre;

	if (reg < 0) {
		LOG_DEV_DBG(dev, "unable to set tre for pipe %d\n", pipe);
		return;
	}

	usbf_bset(dev, reg, TRCLR | TRENB, val);
}

static int usbd_rza2_dpipe_is_stall(const struct device *dev, uint8_t pipe)
{
	uint16_t pid = usbd_rza2_dpipe_pid_get(dev, pipe);

	return (int)(pid == PID_STALL10 || pid == PID_STALL11);
}

static void usbd_rza2_dpipe_stall(const struct device *dev, uint8_t pipe)
{
	uint16_t pid = usbd_rza2_dpipe_pid_get(dev, pipe);

	pid &= PID;

	switch (pid) {
	case PID_NAK:
		usbd_rza2_dpipe_pid_set(dev, pipe, PID_STALL10);
		break;
	case PID_BUF:
		usbd_rza2_dpipe_pid_set(dev, pipe, PID_STALL11);
		break;
	}
}

static int usbd_rza2_dpipe_can_access(const struct device *dev, uint8_t pipe)
{
	return (usbf_read(dev, D_PIPENCTR1(pipe)) & BSTS) ? 0 : -EBUSY;
}

static bool usbd_rza2_dpipe_has_data(const struct device *dev, uint8_t pipe)
{
	return !!(usbf_read(dev, D_PIPENCTR1(pipe)) & INBUFM);
}

static int usbd_rza2_dpipe_cfg(const struct device *dev, uint8_t pipe,
			       const struct usb_dc_ep_cfg_data * const ep_cfg)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[pipe];
	uint16_t r_pipe_conf = 0;
	uint16_t r_pipebuf;
	int ret;

	if (pipe_cfg->is_used) {
		return -EBUSY;
	}

	usbd_rza2_dpipe_stall2nak(dev, pipe);
	usbd_rza2_dpipe_pid_set(dev, pipe, PID_NAK);
	ret = usbd_rza2_dpipe_is_busy(dev, pipe);
	if (ret) {
		return ret;
	}

	r_pipe_conf |= pipe_type_array[ep_cfg->ep_type];
	r_pipe_conf |= USB_EP_GET_IDX(ep_cfg->ep_addr) & EPNUM;
	r_pipe_conf |= g_pipe_config[pipe].double_buf ? DBLB : 0;

	if (USB_EP_DIR_IS_IN(ep_cfg->ep_addr)) {
		r_pipe_conf |= DIR_OUT;
	} else {
		if (ep_cfg->ep_type == USB_DC_EP_BULK || ep_cfg->ep_type == USB_DC_EP_INTERRUPT) {
			r_pipe_conf |= SHTNAK;
		}
	}
	r_pipebuf = usbd_rza2_dpipe_setup_buf(dev, pipe);

	pipe_cfg->is_used = true;
	memcpy(&pipe_cfg->ep_cfg, ep_cfg, sizeof(*ep_cfg));

	usbd_rza2_dpipe_select(dev, pipe);
	usbf_write(dev, D_PIPECFG, r_pipe_conf);
	usbf_write(dev, D_PIPEBUF, r_pipebuf);
	usbf_write(dev, D_PIPEMAXP, ep_cfg->ep_mps & MXPSD);
	/*  clear data sequence */
	usbf_bset(dev, D_PIPENCTR1(pipe), SQCLR | SQSET, SQCLR);
	usbf_bset(dev, D_PIPENCTR1(pipe), ACLRM, ACLRM);
	usbf_bset(dev, D_PIPENCTR1(pipe), ACLRM, 0);

	LOG_DEV_DBG(dev,
		    "CFG: pipe%d  D_PIPENCTR:%04X D_PIPECFG:%04X D_PIPEBUF:%04X D_PIPEMAXP:%04X",
		    pipe,
		    usbf_read(dev, D_PIPENCTR1(pipe)),
		    usbf_read(dev, D_PIPECFG),
		    usbf_read(dev, D_PIPEBUF),
		    usbf_read(dev, D_PIPEMAXP));
	return ret;
}

static int usbd_rza2_dpipe_en(const struct device *dev, uint8_t pipe)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[pipe];
	int ret;

	if (pipe_cfg->is_enabled) {
		LOG_DEV_ERR(dev, "EN: pipe %d is already enabled", pipe);
		return -EINVAL;
	}

	usbd_rza2_dpipe_stall2nak(dev, pipe);
	usbd_rza2_dpipe_pid_set(dev, pipe, PID_NAK);
	ret = usbd_rza2_dpipe_is_busy(dev, pipe);
	if (ret) {
		return ret;
	}

	/*  clear data sequence */
	usbf_bset(dev, D_PIPENCTR1(pipe), SQCLR | SQSET, SQCLR);
	/* Clear pipe state */
	usbf_bset(dev, D_PIPENCTR1(pipe), ACLRM, ACLRM);
	usbf_bset(dev, D_PIPENCTR1(pipe), ACLRM, 0);
	usbhd_rza2_dpipe_tre_set(dev, pipe, TRCLR);

	if (USB_EP_DIR_IS_OUT(pipe_cfg->ep_cfg.ep_addr)) {
		if (pipe_cfg->ep_cfg.ep_type == USB_DC_EP_BULK) {
			usbd_rza2_dpipe_trn_set(dev, pipe, CONFIG_USBD_RZA2_BULK_TRN_COUNT);
			usbhd_rza2_dpipe_tre_set(dev, pipe, TRENB);
		}

		usbf_bset(dev, D_BRDYSTS, BIT(pipe), 0);
		usbd_rza2_rx_irq_ctrl(dev, pipe, 1);
		/* en pipe */
		usbd_rza2_dpipe_pid_set(dev, pipe, PID_BUF);
	}
	pipe_cfg->is_enabled = true;

	return 0;
}

static void usbd_rza2_dpipe_dis(const struct device *dev, uint8_t pipe)
{
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[pipe];
	int ret;

	if (!pipe_cfg->is_enabled) {
		LOG_DEV_ERR(dev, "DIS: pipe %d is not enabled", pipe);
		return;
	}

	if (USB_EP_DIR_IS_OUT(pipe_cfg->ep_cfg.ep_addr)) {
		usbd_rza2_rx_irq_ctrl(dev, pipe, 0);
	}
	usbd_rza2_dpipe_stall2nak(dev, pipe);
	usbd_rza2_dpipe_pid_set(dev, pipe, PID_NAK);
	ret = usbd_rza2_dpipe_is_busy(dev, pipe);
	if (ret) {
		return;
	}

	memset(pipe_cfg, 0, sizeof(*pipe_cfg));
}

static struct usbd_rza2_ep_data *usbd_rza2_ep_get(const struct device *dev, uint8_t ep_addr)
{
	const struct usbd_rza2_config *config = dev->config;
	struct usb_device_data *data = dev->data;
	int ep = USB_EP_GET_IDX(ep_addr);

	if (ep >= config->num_of_eps) {
		return NULL;
	}

	if (USB_EP_DIR_IS_IN(ep_addr)) {
		return &data->ep_data_in[ep];
	}

	return &data->ep_data_out[ep];
}

static void usbd_rza2_tx_irq_ctrl(const struct device *dev, uint8_t pipe, int enable)
{
	struct usb_device_data *data = dev->data;

	/*
	 * And DCP pipe can NOT use "ready interrupt" for "send"
	 * it should use "empty" interrupt.
	 * on the other hand, normal pipe can use "ready interrupt" for "send"
	 * even though it is single/double buffer
	 */
	if (pipe == PIPE0) {
		if (enable) {
			data->irq_en.bempenb |= BIT(pipe);
		} else {
			data->irq_en.bempenb &= ~BIT(pipe);
		}
		usbf_write(dev, D_BEMPENB, data->irq_en.bempenb);
	} else {
		if (enable) {
			data->irq_en.brdyenb |= BIT(pipe);
		} else {
			data->irq_en.brdyenb &= ~BIT(pipe);
		}
		usbf_write(dev, D_BRDYENB, data->irq_en.brdyenb);
	}
}

static void usbd_rza2_rx_irq_ctrl(const struct device *dev, uint8_t pipe, int enable)
{
	struct usb_device_data *data = dev->data;

	data->irq_en.brdyenb &= ~BIT(pipe);
	if (enable) {
		data->irq_en.brdyenb |= BIT(pipe);
	}
	usbf_write(dev, D_BRDYENB, data->irq_en.brdyenb);
}

static int usbd_rza2_tx0(const struct device *dev, const uint8_t *buf, uint32_t buf_len)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[PIPE0];
	struct usbd_rza2_fifo *fifo = &data->cfifo;
	k_spinlock_key_t key;
	bool finished;
	uint32_t len;
	int ret;

	key = usbd_rza2_fifo_lock(fifo);

	ret = usbd_rza2_fifo_sel0(dev, fifo, 1);
	if (ret < 0) {
		ret = -EAGAIN;
		goto unlock;
	}

	if (pipe_cfg->is_zlp_in) {
		usbd_rza2_fifo_clear0(dev, fifo);
		k_busy_wait(1);
		pipe_cfg->is_zlp_in = false;
		usbf_write(dev, D_BEMPSTS, ~(1 << PIPE0));
		LOG_DEV_DBG(dev, "tx0: fifo reset");
	}

	pipe_cfg->is_zlp_in = !buf_len ? true : false;

	ret = usbd_rza2_cpipe_can_access(dev);
	if (ret < 0) {
		/* inaccessible pipe is not an error */
		ret = -EAGAIN;
		LOG_DEV_DBG(dev, "tx0: pipe is not accessible");
		goto unlock;
	}

	ret = usbd_rza2_fifo_is_ready(dev, fifo);
	if (ret < 0) {
		ret = -EAGAIN;
		LOG_DEV_DBG(dev, "tx0: fifo is not ready");
		goto unlock;
	}

	len = MIN(buf_len, pipe_cfg->ep_cfg.ep_mps);

	usbd_rza2_fifo_wr(dev, fifo, buf, len);

	finished = !(len < buf_len);
	if (finished) {
		/* Short Packet */
		usbf_bset(dev, fifo->ctr_off, CFIFOCTR_BVAL, CFIFOCTR_BVAL);
	}

	LOG_DEV_DBG(dev, "tx0: write %u bytes", len);
	usbd_rza2_tx_irq_ctrl(dev, PIPE0, !finished);
	usbd_rza2_cpipe_pid_set(dev, PID_BUF);

	ret = len;

unlock:
	usbd_rza2_fifo_unlock(fifo, key);
	return ret;
}

static int usbd_rza2_tx(const struct device *dev, uint8_t pipe, const uint8_t *buf,
			uint32_t buf_len)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[pipe];
	struct usbd_rza2_fifo *fifo = &data->cfifo;
	k_spinlock_key_t key;
	bool finished;
	uint32_t len;
	int ret;

	key = usbd_rza2_fifo_lock(fifo);

	ret = usbd_rza2_fifo_sel(dev, pipe, fifo);
	if (ret < 0) {
		ret = -EAGAIN;
		goto unlock;
	}

	if (!buf_len) {
		LOG_DEV_WRN(dev, "tx: ZLP on non dcp pipe:%d", pipe);
	}

	ret = usbd_rza2_dpipe_can_access(dev, pipe);
	if (ret < 0) {
		/* inaccessible pipe is not an error */
		ret = -EAGAIN;
		LOG_DEV_WRN(dev, "tx: pipe%d is not accessible", pipe);
		goto unlock;
	}

	ret = usbd_rza2_fifo_is_ready(dev, fifo);
	if (ret < 0) {
		ret = -EAGAIN;
		goto unlock;
	}

	len = MIN(buf_len, pipe_cfg->ep_cfg.ep_mps);

	usbd_rza2_fifo_wr(dev, fifo, buf, len);

	finished = !(len < buf_len);
	if (finished) {
		/* Short Packet */
		usbf_bset(dev, fifo->ctr_off, CFIFOCTR_BVAL, CFIFOCTR_BVAL);
	}

	LOG_DEV_DBG(dev, "tx: pipe %d ep%x write %u bytes", pipe, pipe_cfg->ep_cfg.ep_addr, len);
	usbd_rza2_tx_irq_ctrl(dev, pipe, true);
	usbd_rza2_dpipe_pid_set(dev, pipe, PID_BUF);

	ret = len;

unlock:
	usbd_rza2_fifo_unlock(fifo, key);
	return ret;
}

static const struct device *usbd_rza2_get_device(void)
{
	return DEVICE_DT_GET(USB_DEV_NODE);
}

int usb_dc_reset(void)
{
	const struct device *dev = usbd_rza2_get_device();

	LOG_DEV_ERR(dev, "Not implemented");
	return 0;
}

int usb_dc_detach(void)
{
	const struct device *dev = usbd_rza2_get_device();

	LOG_DEV_ERR(dev, "Not implemented");
	return 0;
}

int usb_dc_set_address(const uint8_t addr)
{
	const struct device *dev = usbd_rza2_get_device();

	/* Nothing to do here. The HW already takes care of setting addr */
	__ASSERT(addr == (uint8_t)(usbf_read(dev, D_USBADDR) & 0x7f),
		 "USB Address %X incorrect!", addr);

	LOG_DEV_INF(dev, "Address set to: %d", addr);
	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data * const cfg)
{
	const struct device *dev = usbd_rza2_get_device();
	const struct usbd_rza2_config *dcfg = dev->config;
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DEV_DBG(dev, "check_cap: ep %x, mps %d, type %d",
		    cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

	if (ep_idx > dcfg->num_of_eps - 1) {
		LOG_DEV_ERR(dev, "endpoint index/address:%x out of range", cfg->ep_addr);
		return -1;
	}

	if (cfg->ep_type == USB_DC_EP_CONTROL) {
		if (ep_idx) {
			LOG_DEV_ERR(dev, "invalid ctrl endpoint configuration");
			return -EINVAL;
		}

		if (cfg->ep_mps > USB_EP_MXPS_CTRL) {
			LOG_DEV_ERR(dev, "invalid ctrl endpoint mxps:%d", cfg->ep_mps);
			return -EINVAL;
		}
	}

	if (cfg->ep_type == USB_DC_EP_BULK && cfg->ep_mps > USB_EP_MXPS_BULK) {
		LOG_DEV_ERR(dev, "invalid bulk endpoint mxps:%d", cfg->ep_mps);
		return -EINVAL;
	}

	if (cfg->ep_type == USB_DC_EP_ISOCHRONOUS && cfg->ep_mps > USB_EP_MXPS_BULK) {
		LOG_DEV_ERR(dev, "invalid iso endpoint mxps:%d", cfg->ep_mps);
		return -EINVAL;
	}

	if (cfg->ep_type == USB_DC_EP_INTERRUPT && cfg->ep_mps > USB_EP_MXPS_INT) {
		LOG_DEV_ERR(dev, "invalid int endpoint mxps:%d", cfg->ep_mps);
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data * const cfg)
{
	const struct device *dev = usbd_rza2_get_device();
	const struct usbd_rza2_config *config = dev->config;
	struct usb_device_data *data = dev->data;
	struct usbd_rza2_ep_data *ep_data;
	int ep = USB_EP_GET_IDX(cfg->ep_addr);
	k_spinlock_key_t key;
	int pipe;
	int ret = 0;

	LOG_DEV_DBG(dev, "CFG: enter ep %x", cfg->ep_addr);

	if (ep >= config->num_of_eps) {
		return -EINVAL;
	}

	if (USB_EP_DIR_IS_IN(cfg->ep_addr)) {
		ep_data = &data->ep_data_in[ep];
	} else {
		ep_data = &data->ep_data_out[ep];
	}

	key = k_spin_lock(&ep_data->lock);
	if (ep_data->is_used) {
		LOG_DEV_ERR(dev, "CFG: can't request ep %x", cfg->ep_addr);
		ret = -EBUSY;
		goto unlock_err;
	}

	pipe = 0;
	if (ep != USB_EP_TYPE_CONTROL) {
		pipe = usbd_rza2_dpipe_find(dev, cfg->ep_type);
	}

	if (pipe < 0) {
		LOG_DEV_ERR(dev, "CFG: can't request pipe %x type %d", cfg->ep_addr, cfg->ep_type);
		ret = -EBUSY;
		goto unlock_err;
	}

	if (ep == USB_EP_TYPE_CONTROL) {
		ret = usbd_rza2_cpipe_cfg(dev, cfg);
	} else {
		ret = usbd_rza2_dpipe_cfg(dev, pipe, cfg);
	}
	if (ret < 0) {
		goto unlock_err;
	}

	/* Save ep_cfg_data to ep structure */
	ep_data->pipe = pipe;
	ep_data->is_used = true;

	LOG_DEV_DBG(dev, "CFG: ep %x:%d, mps %d, type %d",
		    cfg->ep_addr, pipe, cfg->ep_mps, cfg->ep_type);

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_set_stall(const uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret = 0;

	LOG_DEV_DBG(dev, "set_stall: enter ep %x", ep_addr);

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "set_stall: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "set_stall: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	if (USB_EP_IS_CTRL(ep_addr)) {
		usbd_rza2_cpipe_stall(dev);
	} else {
		usbd_rza2_dpipe_stall(dev, ep_data->pipe);
	}
	ep_data->is_stall = true;

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_clear_stall(const uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret = 0;

	LOG_DEV_DBG(dev, "clr_stall: enter ep %x", ep_addr);

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "clr_stall: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "clr_stall: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	if (USB_EP_IS_CTRL(ep_addr)) {
		usbd_rza2_cpipe_stall2nak(dev);
		usbd_rza2_cpipe_pid_set(dev, PID_BUF);
	} else {
		usbd_rza2_dpipe_stall2nak(dev, ep_data->pipe);
		usbd_rza2_dpipe_pid_set(dev, ep_data->pipe, PID_BUF);
	}
	ep_data->is_stall = false;

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_is_stalled(const uint8_t ep_addr, uint8_t *const stalled)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret = 0;

	LOG_DEV_DBG(dev, "is_stall: enter ep %x", ep_addr);

	if (!stalled) {
		return -EINVAL;
	}

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "is_stall: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "is_stall: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	if (USB_EP_IS_CTRL(ep_addr)) {
		*stalled = usbd_rza2_cpipe_is_stall(dev);
	} else {
		*stalled = usbd_rza2_dpipe_is_stall(dev, ep_data->pipe);
	}

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	const struct device *dev = usbd_rza2_get_device();

	LOG_DEV_DBG(dev, "halt");
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_enable(const uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret;

	LOG_DEV_DBG(dev, "EN: enter ep %x", ep_addr);

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "EN: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used) {
		LOG_DEV_ERR(dev, "EN: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	ep_data->state = EP_STATE_INVALID;
	if (USB_EP_IS_CTRL(ep_addr)) {
		ret = usbd_rza2_cpipe_en(dev);
	} else {
		ret = usbd_rza2_dpipe_en(dev, ep_data->pipe);
	}
	ep_data->is_enabled = true;

	LOG_DEV_DBG(dev, "EN: enabled ep %x", ep_addr);

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_disable(const uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret = 0;

	LOG_DEV_DBG(dev, "DIS: enter ep %x", ep_addr);

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "DIS: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "DIS: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	usbd_rza2_tx_irq_ctrl(dev, ep_data->pipe, 0);
	usbd_rza2_rx_irq_ctrl(dev, ep_data->pipe, 0);

	if (USB_EP_IS_CTRL(ep_addr)) {
		usbd_rza2_cpipe_dis(dev);
	} else {
		usbd_rza2_dpipe_dis(dev, ep_data->pipe);
	}

	ep_data->is_used = false;
	ep_data->is_enabled = false;
	ep_data->pipe = 0;

	LOG_DEV_DBG(dev, "DIS: disabled ep %x", ep_addr);

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_flush(const uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();

	LOG_DEV_DBG(dev, "flush: not implemented ep %x", ep_addr);
	return 0;
}

int usb_dc_ep_write(const uint8_t ep_addr, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret;

	if (!USB_EP_DIR_IS_IN(ep_addr)) {
		LOG_DEV_ERR(dev, "tx: wrong endpoint direction");
		return -EINVAL;
	}

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "tx: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "tx: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	if (USB_EP_IS_CTRL(ep_addr)) {
		ret = usbd_rza2_tx0(dev, data, data_len);
	} else {
		ret = usbd_rza2_tx(dev, ep_data->pipe, data, data_len);
	}

	if (ret < 0) {
		goto unlock_err;
	}

	if (ret_bytes) {
		*ret_bytes = ret;
	}
	ret = 0;

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_read_wait0(const struct device *dev,
			 uint8_t *buf_data, uint32_t max_data_len, uint32_t *read_bytes)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[PIPE0];
	struct usbd_rza2_fifo *fifo = &data->cfifo;
	k_spinlock_key_t key;
	uint32_t len;
	int recv_len;
	int ret;

	key = usbd_rza2_fifo_lock(fifo);

	ret = usbd_rza2_fifo_sel0(dev, fifo, 0);
	if (ret < 0) {
		ret = -EAGAIN;
		goto busy;
	}

	ret = usbd_rza2_cpipe_can_access(dev);
	if (ret < 0) {
		ret = -EAGAIN;
		goto busy;
	}

	ret = usbd_rza2_fifo_is_ready(dev, fifo);
	if (ret < 0) {
		ret = -EAGAIN;
		goto busy;
	}

	recv_len = usbd_rza2_fifo_get_rd_len(dev, fifo);

	len = MIN(max_data_len, recv_len);

	if (!data && !max_data_len) {
		*read_bytes = len;
		goto end;
	}

	if (recv_len == 0) {
		usbd_rza2_fifo_clear(dev, fifo);
		goto end;
	}

	ret = usbd_rza2_fifo_rd(dev, fifo, buf_data, len);

	if (read_bytes) {
		*read_bytes = len;
	}

end:
	LOG_DEV_DBG(dev, "rx0: ep%x, req %d, read %d bytes",
		    pipe_cfg->ep_cfg.ep_addr, max_data_len, len);

busy:
	usbd_rza2_fifo_unlock(fifo, key);
	return ret;
}

int usb_dc_ep_read_waitn(const struct device *dev, uint8_t pipe,
			 uint8_t *buf_data, uint32_t max_data_len, uint32_t *read_bytes)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_pipe_cfg *pipe_cfg = &data->pipe_cfg[pipe];
	struct usbd_rza2_fifo *fifo = &data->cfifo;
	k_spinlock_key_t key;
	uint32_t len;
	int recv_len;
	int ret;

	key = usbd_rza2_fifo_lock(fifo);

	ret = usbd_rza2_fifo_sel(dev, pipe, fifo);
	if (ret < 0) {
		ret = -EAGAIN;
		goto busy;
	}

	ret = usbd_rza2_dpipe_can_access(dev, pipe);
	if (ret < 0) {
		ret = -EAGAIN;
		goto busy;
	}

	if (!usbd_rza2_dpipe_has_data(dev, pipe)) {
		LOG_DEV_INF(dev, "rx: pipe %x:%d is empty", pipe_cfg->ep_cfg.ep_addr, pipe);
		ret = -ENODATA;
		goto busy;
	}

	ret = usbd_rza2_fifo_is_ready(dev, fifo);
	if (ret < 0) {
		ret = -EAGAIN;
		goto busy;
	}

	recv_len = usbd_rza2_fifo_get_rd_len(dev, fifo);

	if (!buf_data && !max_data_len) {
		*read_bytes = recv_len;
		goto busy;
	}

	len = MIN(max_data_len, recv_len);

	if (recv_len == 0) {
		usbd_rza2_fifo_clear(dev, fifo);
		goto end;
	}

	ret = usbd_rza2_fifo_rd(dev, fifo, buf_data, len);

	if (read_bytes) {
		*read_bytes = len;
	}

end:
	LOG_DEV_DBG(dev, "rx: ep%x:%d, req %d, read %d bytes",
		    pipe_cfg->ep_cfg.ep_addr, pipe, max_data_len, len);
busy:
	usbd_rza2_fifo_unlock(fifo, key);
	return ret;
}

int usb_dc_ep_read_wait(uint8_t ep_addr, uint8_t *buf_data, uint32_t max_data_len,
			uint32_t *read_bytes)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret;

	if (!buf_data && max_data_len) {
		LOG_DEV_ERR(dev, "rx: wrong arguments");
		return -EINVAL;
	}

	if (!USB_EP_DIR_IS_OUT(ep_addr)) {
		LOG_DEV_ERR(dev, "rx: Wrong endpoint direction %x", ep_addr);
		return -EINVAL;
	}

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "rx: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "rx: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	/* usbhs_pipe_set_trans_count_if_bulk(dev, pipe, 1522); */

	if (USB_EP_IS_CTRL(ep_addr)) {
		ret = usb_dc_ep_read_wait0(dev, buf_data, max_data_len, read_bytes);
	} else {
		ret = usb_dc_ep_read_waitn(dev, ep_data->pipe, buf_data, max_data_len, read_bytes);
	}

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

int usb_dc_ep_read_continue0(const struct device *dev)
{
	usbd_rza2_cpipe_pid_set(dev, PID_BUF);
	return 0;
}

int usb_dc_ep_read_continuen(const struct device *dev, uint8_t pipe)
{
	usbd_rza2_dpipe_pid_set(dev, pipe, PID_BUF);
	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret;

	if (!USB_EP_DIR_IS_OUT(ep_addr)) {
		LOG_DEV_ERR(dev, "rxc: Wrong endpoint direction %x", ep_addr);
		return -EINVAL;
	}

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "rxc: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "rxc: ep %x is not configured", ep_addr);
		ret = -EINVAL;
		goto unlock_err;
	}

	if (USB_EP_IS_CTRL(ep_addr)) {
		ret = usb_dc_ep_read_continue0(dev);
	} else {
		ret = usb_dc_ep_read_continuen(dev, ep_data->pipe);
	}

unlock_err:
	k_spin_unlock(&ep_data->lock, key);
	return ret;
}

static int usbd_rza2_handle_ctrl_req(const struct device *dev, uint8_t *const data,
				     const uint32_t max_data_len)
{
	struct usb_rza2_setup_packet *setup_raw = (struct usb_rza2_setup_packet *)data;
	uint16_t reg16;

	if (!setup_raw) {
		return -EINVAL;
	}

	if (max_data_len != SETUP_PACKET_SIZE) {
		LOG_DEV_ERR(dev, "rd: wrong setup packet size");
		return -EINVAL;
	}

	reg16 = usbf_read(dev, D_USBREQ);
	setup_raw->requestType = reg16 & BMREQUESTTYPE;
	setup_raw->request = (reg16 & BREQUEST) >> BREQUEST_SHIFT;

	setup_raw->value = usbf_read(dev, D_USBVAL);
	setup_raw->index = usbf_read(dev, D_USBINDX);
	setup_raw->length = usbf_read(dev, D_USBLENG);

	LOG_DEV_DBG(dev, "SP: T:%x %x %x %x L:%x",
		    setup_raw->requestType, setup_raw->request, setup_raw->value,
		    setup_raw->index, setup_raw->length);

	return 0;
}

int usb_dc_ep_read(const uint8_t ep_addr, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;
	k_spinlock_key_t key;
	int ret;

	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "rd: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	key = k_spin_lock(&ep_data->lock);

	if (!ep_data->is_used || !ep_data->is_enabled) {
		LOG_DEV_ERR(dev, "rd: ep %x is not configured", ep_addr);
		k_spin_unlock(&ep_data->lock, key);
		return -EINVAL;
	}

	if (ep_addr == USB_CONTROL_EP_OUT && ep_data->state == USB_DC_EP_SETUP) {
		ep_data->state = EP_STATE_INVALID;
		ret = usbd_rza2_handle_ctrl_req(dev, data, max_data_len);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "rd: unable to handle setup packet");
		}
		k_spin_unlock(&ep_data->lock, key);
		return ret;
	}

	k_spin_unlock(&ep_data->lock, key);

	ret = usb_dc_ep_read_wait(ep_addr, data, max_data_len, read_bytes);
	if (ret) {
		return ret;
	}

	if (!max_data_len) {
		return 0;
	}

	return usb_dc_ep_read_continue(ep_addr);
}

int usb_dc_ep_set_callback(const uint8_t ep_addr, const usb_dc_ep_callback cb)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;

	LOG_DEV_DBG(dev, "start ep_addr = %x", ep_addr);
	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "set_callback: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	ep_data->cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	const struct device *dev = usbd_rza2_get_device();

	LOG_DEV_DBG(dev, "enter");
	DEV_DATA(dev)->status_cb = cb;
}

int usb_dc_ep_mps(const uint8_t ep_addr)
{
	const struct device *dev = usbd_rza2_get_device();
	struct usbd_rza2_ep_data *ep_data;

	LOG_DEV_DBG(dev, "enter");
	ep_data = usbd_rza2_ep_get(dev, ep_addr);
	if (!ep_data) {
		LOG_DEV_ERR(dev, "rd: can't get ep %x", ep_addr);
		return -EINVAL;
	}

	if (!ep_data->is_used || !ep_data->is_enabled) {
		return 0;
	}

	return usbd_rza2_pipe_get_maxpacket(dev, ep_data->pipe);
}

static void usbd_rza2_ctrl_transfer_done(const struct device *dev)
{
	struct usb_device_data *data = DEV_DATA(dev);

	__ASSERT(pipe != PIPE0, "Invalid pipe for DCP transfer");

	data->irq_en.bempenb &= ~BIT(PIPE0);
	usbf_write(dev, D_BEMPENB, data->irq_en.bempenb);
	data->irq_en.nrdyenb &= ~BIT(PIPE0);
	usbf_write(dev, D_NRDYENB, data->irq_en.nrdyenb);

	usbd_rza2_cpipe_pid_set(dev, PID_BUF);

	usbd_rza2_cpipe_set_ccpl(dev, CCPL);
}

/* IRQ RESM */
static int usbd_rza2_irq_dev_resm(const struct device *dev, struct usbd_rza2_irq_state *irq_state)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_msg msg;

	LOG_DEV_DBG(dev, "irq:RESM:");

	/* RESM interrupt disable */
	usbf_write(dev, D_INTSTS0, ~RESM & INTSTS0_MAGIC);
	data->irq_en.intenb0 &= ~RSME;
	usbf_write(dev, D_INTENB0, data->irq_en.intenb0);

	msg.type = USBD_RZA2_CB_STATUS;
	msg.cmd = USB_DC_RESUME;
	k_msgq_put(&DEV_DATA(dev)->queue, &msg, K_NO_WAIT);

	return 0;
}

/* DVST (DVSQ) */
static int usbd_rza2_irq_dev_state(const struct device *dev, struct usbd_rza2_irq_state *irq_state)
{
	struct usb_device_data *data = DEV_DATA(dev);
	uint16_t dvsq = (irq_state->intsts0 & DVSQ);
	struct usbd_rza2_msg msg = { 0 };
	uint16_t r_syssts0;

	LOG_DEV_DBG(dev, "irq:dvsq: %04X", dvsq);

	r_syssts0 = usbf_read(dev, D_SYSSTS0);
	if (data->can_susp && (irq_state->intsts0 & DVSQ_SUSP) &&
	    ((r_syssts0 & LNST) == 0x0001u /* USB_FS_JSTS */)) {

		/* Resume interrupt enable */
		usbf_write(dev, D_INTSTS0, ~RESM & INTSTS0_MAGIC);
		data->irq_en.intenb0 |= RSME;
		usbf_write(dev, D_INTENB0, data->irq_en.intenb0);

		/* Suspend */
		usbf_write(dev, D_LPSTS, 0);
		data->is_susp = true;
		LOG_DEV_DBG(dev, "irq:dvsq: suspen");
	}

	data->can_susp = true;
	msg.type = USBD_RZA2_CB_STATUS;

	switch (dvsq) {
	case DVSQ_ADDS:
	case DVSQ_POWR:
		break;
	case DVSQ_DFLT:
		msg.cmd = USB_DC_RESET;
		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
		break;
	case DVSQ_CNFG:
		msg.cmd = USB_DC_CONFIGURED;
		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
		break;
	case DVSQ_SPD_CNFG:
	case DVSQ_SPD_ADDR:
	case DVSQ_SPD_DFLT:
	case DVSQ_SPD_POWR:
		data->can_susp = false;
		msg.cmd = USB_DC_SUSPEND;
		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
	break;
	}

	return 0;
}

/* CTRT (CTSQ) */
static int usbd_rza2_irq_ctrt(const struct device *dev, struct usbd_rza2_irq_state *irq_state)
{
	struct usb_device_data *data = DEV_DATA(dev);
	uint16_t ctsq = (irq_state->intsts0 & CTSQ);
	struct usbd_rza2_msg msg;

	LOG_DEV_DBG(dev, "irq:ctsq: %04X", ctsq);

	msg.type = USBD_RZA2_CB_EP;
	msg.data1 = ctsq;

	/* Clear usb VALID */
	usbf_write(dev, D_INTSTS0, usbf_read(dev, D_INTSTS0) & ~VALID);

	switch (ctsq) {
	case CS_IDST:
		break;
	case CS_RDDS:
		/* fall through */
	case CS_WRDS:
		/* fall through */
	case CS_WRND:
		data->ep_data_out[USB_EP_TYPE_CONTROL].state = USB_DC_EP_SETUP;
		msg.ep_addr = USB_CONTROL_EP_OUT;
		msg.cmd = USB_DC_EP_SETUP;
		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
		break;
	case CS_RDSS:
	case CS_WRSS:
		/* status stage - finalize */
		usbd_rza2_ctrl_transfer_done(dev);
		break;
	default:
		/* fall through */
	};

	return 0;
}

static int usbd_rza2_irq_emptyn(const struct device *dev, struct usbd_rza2_irq_state *irq_state)
{
	LOG_DEV_DBG(dev, "enter");
	return 0;
}

/* BEMP / BEMPSTS */
static int usbd_rza2_irq_empty(const struct device *dev, struct usbd_rza2_irq_state *irq_state)
{
	struct usbd_rza2_msg msg;
	int ep;

	LOG_DEV_DBG(dev, "irq:BEMPS: %04X", irq_state->bempsts);

	ep = irq_state->bempsts & BIT(0);
	if (!ep) {
		/* non control pipe0 */
		return usbd_rza2_irq_emptyn(dev, irq_state);
	}

	/* BEMP is used for pipe0 to notify usbd that it can continue sending data IN(tx). */
	msg.type = USBD_RZA2_CB_EP;
	msg.ep_addr = USB_CONTROL_EP_IN;
	msg.cmd = USB_DC_EP_SETUP;
	msg.data1 = 0;
	k_msgq_put(&DEV_DATA(dev)->queue, &msg, K_NO_WAIT);

	return 0;
}

/* BRDY / BRDYSTS */
static int usbd_rza2_irq_ready(const struct device *dev, struct usbd_rza2_irq_state *irq_state)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_msg msg;
	int pipe;

	LOG_DEV_DBG(dev, "irq:BRDY: %04X", irq_state->brdysts);

	for (pipe = 0; pipe < 16 && irq_state->brdysts; pipe++) {

		if (!(irq_state->brdysts & (1 << pipe))) {
			continue;
		}

		irq_state->brdysts &= ~(1 << pipe);

		msg.type = USBD_RZA2_CB_EP;
		msg.data1 = 0;
		if (pipe) {
			msg.ep_addr = data->pipe_cfg[pipe].ep_cfg.ep_addr;
			if (USB_EP_DIR_IS_IN(msg.ep_addr)) {
				msg.cmd = USB_DC_EP_DATA_IN;
			} else {
				msg.cmd = USB_DC_EP_DATA_OUT;
			}
		} else {
			/* for pipe0 BRDY is generated only for OUT (rx) direction */
			msg.ep_addr = USB_CONTROL_EP_OUT;
			msg.cmd = USB_DC_EP_DATA_OUT;
		}

		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
	}

	return 0;
}

static enum usbd_periph_state usbd_rza2_chk_vbsts(const struct device *dev)
{
	uint16_t connect_info;
	uint16_t buf1;
	uint16_t buf2;
	uint16_t buf3;

	/* VBUS chattering cut */
	do {
		buf1 = usbf_read(dev, D_INTSTS0);
		k_busy_wait(10);
		buf2 = usbf_read(dev, D_INTSTS0);
		k_busy_wait(10);
		buf3 = usbf_read(dev, D_INTSTS0);
	} while (((buf1 & VBSTS) != (buf2 & VBSTS)) || ((buf2 & VBSTS) != (buf3 & VBSTS)));

	if ((buf1 & VBSTS) != 0) {
		connect_info = USBD_ATTACH;
	} else {
		connect_info = USBD_DETACH;
	}

	return connect_info;
}

static void usbd_rza2_do_attach_proc(const struct device *dev)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_msg msg = { 0 };

	if (!data->is_attached) {
		usbf_bset(dev, D_SYSCFG0, DPRPU | CNEN, DPRPU | CNEN);
		k_busy_wait(10);
		data->is_attached = true;

		msg.type = USBD_RZA2_CB_STATUS;
		msg.cmd = USB_DC_CONNECTED;
		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
		LOG_DEV_INF(dev, "vbus attach");
	}
}

static void usbd_rza2_do_detach_proc(const struct device *dev)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_msg msg = { 0 };

	if (data->is_attached) {
		usbf_bset(dev, D_SYSCFG0, DPRPU | CNEN, 0);
		usbf_bset(dev, D_SYSCFG0, USBE, 0);
		usbf_read(dev, D_SYSCFG0);
		usbf_bset(dev, D_SYSCFG0, USBE, USBE);
		data->is_attached = false;

		/* RESM interrupt disable */
		usbf_write(dev, D_INTSTS0, ~RESM & INTSTS0_MAGIC);
		data->irq_en.intenb0 &= ~RSME;
		usbf_write(dev, D_INTENB0, data->irq_en.intenb0);

		/* Suspend */
		usbf_write(dev, D_LPSTS, 0);
		data->is_susp = true;

		msg.type = USBD_RZA2_CB_STATUS;
		msg.cmd = USB_DC_DISCONNECTED;
		k_msgq_put(&data->queue, &msg, K_NO_WAIT);
		LOG_DEV_INF(dev, "vbus detach");
	}
}

static void usbd_rza2_thread_main(void *p1, void *unused1, void *unused2)
{
	const struct device *dev = p1;
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_msg msg;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	while (true) {
		k_msgq_get(&data->queue, &msg, K_FOREVER);
		LOG_DEV_DBG(dev, "MSG:%d cmd:%x ep:%x d1:%x ",
			    msg.type, msg.cmd, msg.ep_addr, msg.data1);

		switch (msg.type) {
		case USBD_RZA2_CB_EP:
			struct usbd_rza2_ep_data *ep_data;
			k_spinlock_key_t key;

			ep_data = usbd_rza2_ep_get(dev, msg.ep_addr);
			if (!ep_data) {
				LOG_DEV_ERR(dev, "unable to get EP:%x", msg.ep_addr);
				break;
			}

			key = k_spin_lock(&ep_data->lock);

			if (!ep_data->is_used || !ep_data->is_enabled) {
				k_spin_unlock(&ep_data->lock, key);
				break;
			}
			k_spin_unlock(&ep_data->lock, key);

			if (ep_data->cb) {
				ep_data->cb(msg.ep_addr, msg.cmd);
			}

			/* event from pipe0 can have data1 set to CTSQ value */
			if (msg.data1 == CS_WRND) {
				/* no-data, finish transaction after notifying usbd core */
				usbd_rza2_ctrl_transfer_done(dev);
			} else if (msg.data1 == CS_WRDS) {
				/* enable OUT (rx) pipe */
				usbd_rza2_cpipe_pid_set(dev, PID_BUF);
			}

			break;
		case USBD_RZA2_CB_STATUS:
			if (data->status_cb) {
				data->status_cb(msg.cmd, NULL);
			}
			break;
		default:
			LOG_DEV_DBG(dev, "invalid message type: %d", msg.type);
		};
	}
}

static void usbd_rza2_isr(const struct device *dev)
{
	struct usb_device_data *data = DEV_DATA(dev);
	struct usbd_rza2_irq_state state;
	int ret;

	if (data->is_susp) {
		/* --- SUSPEND -> RESUME --- */
		usbf_write(dev, D_LPSTS, SUSPM);
		k_busy_wait(100);
		data->is_susp = false;
	}

	state.intsts0 = usbf_read(dev, D_INTSTS0);
	state.brdysts = usbf_read(dev, D_BRDYSTS);
	state.nrdysts = usbf_read(dev, D_NRDYSTS);
	state.bempsts = usbf_read(dev, D_BEMPSTS);
	LOG_DEV_DBG(dev, "irq: %04X", state.intsts0);

	/* clear interrupt status */
	usbf_write(dev, D_INTSTS0, ~state.intsts0 & INTSTS0_MAGIC);
	usbf_read(dev, D_INTSTS0);

	if (state.intsts0 & BRDY) {
		usbf_write(dev, D_BRDYSTS, ~state.brdysts);
	}

	if (state.intsts0 & NRDY) {
		usbf_write(dev, D_NRDYSTS, ~state.nrdysts);
	}

	if (state.intsts0 & BEMP) {
		usbf_write(dev, D_BEMPSTS, ~state.bempsts);
	}

	if (state.intsts0 & RESM & data->irq_en.intenb0) {
		ret = usbd_rza2_irq_dev_resm(dev, &state);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "error processig resm %d", ret);
		}
	}

	if (state.intsts0 & VBINT) {
		if (usbd_rza2_chk_vbsts(dev) == USBD_ATTACH) {
			usbd_rza2_do_attach_proc(dev);
		} else {
			usbd_rza2_do_detach_proc(dev);
		}
	}

	if (state.intsts0 & DVST) {
		ret = usbd_rza2_irq_dev_state(dev, &state);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "error processig irq_dev_state %d", ret);
		}
	}

	if (state.intsts0 & CTRT) {
		ret = usbd_rza2_irq_ctrt(dev, &state);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "error processig irq_ctrt %d", ret);
		}
	}

	if (state.intsts0 & BEMP) {
		ret = usbd_rza2_irq_empty(dev, &state);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "error processig irq_empty %d", ret);
		}
	}

	if (state.intsts0 & BRDY) {
		ret = usbd_rza2_irq_ready(dev, &state);
		if (ret < 0) {
			LOG_DEV_ERR(dev, "error processig irq_ready %d", ret);
		}
	}

	if (state.intsts0 & NRDY) {
		LOG_DEV_DBG(dev, "irq:NRDY: %04X", state.nrdysts);
	}
}

static int usbd_rza2_module_config(const struct device *dev, int speed_idx)
{
	uint16_t mask = DRPD | DPRPU | HSE | USBE | CNEN;
	uint16_t val = USBE;

	usbf_write(dev, D_LPSTS, SUSPM);
	k_busy_wait(1);

	/* High-speed*/
	if (speed_idx == 2) {
		val |= HSE;
	}

	usbf_bset(dev, D_SYSCFG0, mask, val);
	LOG_DEV_DBG(dev, "D_SYSCFG0:%X", usbf_read(dev, D_SYSCFG0));

	usbf_bset(dev, D_SYSCFG1, BWAIT, CONFIG_USBD_RZA2_BUSWAIT << BWAIT_SHIFT);

	return 0;
}

static int usbd_rza2_module_init(const struct device *dev)
{
	uint32_t reg32;

	LOG_DEV_DBG(dev, "start");

	/* Disable overcurrent */
	usbh_bset32(dev, H_HCRHDESCRIPTORA, HCRHDESCRIPTORA_NOCP, HCRHDESCRIPTORA_NOCP);

	/* Set clock to usb_x1 if it was provided */
	if (DEV_CFG(dev)->is_usb_x1_src) {
		usbh_write32(dev, H_PHYCLK_CTRL, PHYCLK_CTRL_UCLKSEL);
	}

	k_busy_wait(500); /* 500us wait */

	/* Switch usb device to peripheral mode */
	usbh_write32(dev, H_COMMCTRL, OTG_PERI);

	/* Release from reset state */
	reg32 = usbh_read32(dev, H_USBCTR);
	usbh_write32(dev, H_USBCTR, reg32 & ~PLL_RST);

	 /* 100us wait */
	k_busy_wait(100);

	return 0;
}

static int usbd_rza2_usb_enable(const struct device *dev)
{
	const struct usbd_rza2_config *cfg = dev->config;
	struct usb_device_data *data = dev->data;
	int ret;

	LOG_DEV_DBG(dev, "start");

	/* Init pipe flags */
	memset(data->pipe_cfg, 0, sizeof(data->pipe_cfg[0]) * PIPE_MAX);
	/* Init ep structs */
	memset(data->ep_data_in, 0, sizeof(data->ep_data_in[0]) * cfg->num_of_eps);
	memset(data->ep_data_in, 0, sizeof(data->ep_data_in[0]) * cfg->num_of_eps);

	/* Init CFIFO */
	data->cfifo.port_off = D_CFIFO;
	data->cfifo.sel_off = D_CFIFOSEL;
	data->cfifo.ctr_off = D_CFIFOCTR;

	/* Start callback thread */
	k_msgq_init(&data->queue, (char *)data->usb_msgs, sizeof(struct usbd_rza2_msg),
		    CONFIG_USBD_RZA2_MESSAGE_COUNT);

	k_thread_create(&data->thread, data->thread_stack,
			K_KERNEL_STACK_SIZEOF(data->thread_stack),
			usbd_rza2_thread_main, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_USB_DC_RZA2_THREAD_PRIORITY), K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(&data->thread, dev->name);

	ret = usbd_rza2_module_init(dev);
	if (ret) {
		return ret;
	}

	ret = usbd_rza2_module_config(dev, cfg->speed_idx);
	if (ret) {
		return ret;
	}

	usbd_rza2_pipe_init(dev);

	/* Enable interrupts */
	cfg->init_irq_func(dev);

	data->irq_en.intenb0 = IRQ_INIT_STATE;
	data->irq_en.brdyenb = 0;
	data->irq_en.nrdyenb = 0;
	data->irq_en.bempenb = 0;
	usbf_write(dev, D_BEMPENB, data->irq_en.bempenb);
	usbf_write(dev, D_BRDYENB, data->irq_en.brdyenb);
	usbf_write(dev, D_NRDYENB, data->irq_en.nrdyenb);
	usbf_write(dev, D_INTENB0, data->irq_en.intenb0);

	LOG_DEV_DBG(dev, "enabled");
	return 0;
}

int usb_dc_attach(void)
{
	const struct device *dev = usbd_rza2_get_device();

	LOG_DEV_DBG(dev, "start");
	return usbd_rza2_usb_enable(dev);
}

static int usbd_rza2_driver_init(const struct device *dev)
{
	const struct usbd_rza2_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_DEV_ERR(dev, "Device %s is not ready\n", dev->name);
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_DEV_ERR(dev, "unable to configure USB pins");
		return -EINVAL;
	}

	ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)&cfg->usb_clk);
	if (ret != 0) {
		LOG_DEV_ERR(dev, "Can't turn on clock for %s\n", dev->name);
		return ret;
	}

	DEVICE_MMIO_NAMED_MAP(dev, dev_reg, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, host_reg, K_MEM_CACHE_NONE);

	usbf_write(dev, D_INTENB0, 0);

	LOG_DEV_INF(dev, "Init done. virt:%p num_of_eps:%d speed_idx: %d",
		    (void *)DEVICE_MMIO_GET(dev), cfg->num_of_eps, cfg->speed_idx);

	return 0;
}

#define USBD_RZA2_SET_IRQ(n, name, isr)                                                            \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq), DT_INST_IRQ_BY_NAME(n, name, priority),     \
		    isr, DEVICE_DT_INST_GET(n), 0);                                                \
	irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));

#define USBD_RZA2_GET_RATE(inst) DT_INST_PROP_BY_PHANDLE_IDX(inst, clocks, 1, clock_frequency)

#define USBD_RZA2_DEVICE_DEFINE(n)                                                                 \
	static void usbd_rza2_init_irq_##n(const struct device *dev)                               \
	{                                                                                          \
		USBD_RZA2_SET_IRQ(n, usbf, usbd_rza2_isr);                                         \
	}                                                                                          \
	static struct usbd_rza2_ep_data ep_data_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];     \
	static struct usbd_rza2_ep_data ep_data_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];      \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct usbd_rza2_config usbd_rza2_config_##n = {                              \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(dev_reg, DT_DRV_INST(n)),                       \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(host_reg, DT_DRV_INST(n)),                      \
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),                                \
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.usb_clk.domain = DT_INST_CLOCKS_CELL(n, domain),                                  \
		.usb_clk.module = DT_INST_CLOCKS_CELL(n, module),                                  \
		.clk_freq = USBD_RZA2_GET_RATE(n),                                                 \
		.is_usb_x1_src = (DT_INST_CLOCKS_HAS_NAME(n, usb_x1) ? true : false),              \
		.init_irq_func = usbd_rza2_init_irq_##n,                                           \
	};                                                                                         \
                                                                                                   \
	static struct usb_device_data usbd_rza2_data_##n = {                                       \
		.ep_data_in = ep_data_in_##n,                                                      \
		.ep_data_out = ep_data_out_##n,                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, usbd_rza2_driver_init, NULL, &usbd_rza2_data_##n,                 \
			      &usbd_rza2_config_##n, POST_KERNEL,                                  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(USBD_RZA2_DEVICE_DEFINE)
