/*
 * Copyright (c) 2021 IoT.bzh
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

/* TODO: Add error handling */

struct scif_reg {
	uint8_t offset, size;
};

/*
 * SCI register subset common for all port types.
 * Not all registers will exist on all parts.
 */
enum {
	SCSMR,				/* Serial Mode Register */
	SCBRR,				/* Bit Rate Register */
	SCSCR,				/* Serial Control Register */
	SCFSR,				/* Serial Status Register */
	SCFCR,				/* FIFO Control Register */
	SCFDR,				/* FIFO Data Count Register */
	SCFTDR,				/* Transmit (FIFO) Data Register */
	SCFRDR,				/* Receive (FIFO) Data Register */
	SCLSR,				/* Line Status Register */
	SCTFDR,				/* Transmit FIFO Data Count Register */
	SCRFDR,				/* Receive FIFO Data Count Register */
	SCSPTR,				/* Serial Port Register */
	SCDL,				/* BRG Frequency Division Register */
	SEMR,				/* Serial extended mode register */
	FTCR,				/* FIFO Trigger Control Register */

	SCIF_NR_REGS,
};

enum {
	SCIF_SCIFA_TYPE,
	SCIF_RZ_SCIFA_TYPE,

	NR_REGTYPES,
};

struct scif_params {
	const struct scif_reg regs[SCIF_NR_REGS];
	uint16_t init_lsr_mask;
	uint16_t init_interrupt_mask;
};

struct uart_scif_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;
	const struct pinctrl_dev_config *pcfg;
	const struct scif_params *params;
	uint32_t type;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

struct uart_scif_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct uart_config current_config;
	uint32_t clk_rate;
	struct k_spinlock lock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

/* SCSMR (Serial Mode Register) */
#define SCSMR_C_A       BIT(7)  /* Communication Mode */
#define SCSMR_CHR       BIT(6)  /* 7-bit Character Length */
#define SCSMR_PE        BIT(5)  /* Parity Enable */
#define SCSMR_O_E       BIT(4)  /* Odd Parity */
#define SCSMR_STOP      BIT(3)  /* Stop Bit Length */
#define SCSMR_CKS1      BIT(1)  /* Clock Select 1 */
#define SCSMR_CKS0      BIT(0)  /* Clock Select 0 */

/* SCSCR (Serial Control Register) */
#define SCSCR_TEIE_SCIFA  BIT(11) /* Transmit End Interrupt Enable on SCIFA*/
#define SCSCR_TEIE_RZ   BIT(2) /* Transmit End Interrupt Enable RZ */
#define SCSCR_TIE       BIT(7)  /* Transmit Interrupt Enable */
#define SCSCR_RIE       BIT(6)  /* Receive Interrupt Enable */
#define SCSCR_TE        BIT(5)  /* Transmit Enable */
#define SCSCR_RE        BIT(4)  /* Receive Enable */
#define SCSCR_REIE      BIT(3)  /* Receive Error Interrupt Enable */
#define SCSCR_TOIE_SCIFA  BIT(2)  /* Timeout Interrupt Enable on SCIFA */
#define SCSCR_CKE1      BIT(1)  /* Clock Enable 1 */
#define SCSCR_CKE0      BIT(0)  /* Clock Enable 0 */

/* SCFCR (FIFO Control Register) */
#define SCFCR_RTRG1     BIT(7)  /* Receive FIFO Data Count Trigger 1 */
#define SCFCR_RTRG0     BIT(6)  /* Receive FIFO Data Count Trigger 0 */
#define SCFCR_TTRG1     BIT(5)  /* Transmit FIFO Data Count Trigger 1 */
#define SCFCR_TTRG0     BIT(4)  /* Transmit FIFO Data Count Trigger 0 */
#define SCFCR_MCE       BIT(3)  /* Modem Control Enable */
#define SCFCR_TFRST     BIT(2)  /* Transmit FIFO Data Register Reset */
#define SCFCR_RFRST     BIT(1)  /* Receive FIFO Data Register Reset */
#define SCFCR_LOOP      BIT(0)  /* Loopback Test */

/* SCFSR (Serial Status Register) */
#define SCFSR_PER3      BIT(15) /* Parity Error Count 3 */
#define SCFSR_PER2      BIT(14) /* Parity Error Count 2 */
#define SCFSR_PER1      BIT(13) /* Parity Error Count 1 */
#define SCFSR_PER0      BIT(12) /* Parity Error Count 0 */
#define SCFSR_FER3      BIT(11) /* Framing Error Count 3 */
#define SCFSR_FER2      BIT(10) /* Framing Error Count 2 */
#define SCFSR_FER1      BIT(9)  /* Framing Error Count 1 */
#define SCFSR_FER0      BIT(8)  /* Framing Error Count 0 */
#define SCFSR_ER        BIT(7)  /* Receive Error */
#define SCFSR_TEND      BIT(6)  /* Transmission ended */
#define SCFSR_TDFE      BIT(5)  /* Transmit FIFO Data Empty */
#define SCFSR_BRK       BIT(4)  /* Break Detect */
#define SCFSR_FER       BIT(3)  /* Framing Error */
#define SCFSR_PER       BIT(2)  /* Parity Error */
#define SCFSR_RDF       BIT(1)  /* Receive FIFO Data Full */
#define SCFSR_DR        BIT(0)  /* Receive Data Ready */

/* SCLSR (Line Status Register) on (H)SCIF */
#define SCLSR_TO_SCIFA  BIT(2)  /* Timeout on SCIFA */
#define SCLSR_ORER      BIT(0)  /* Overrun Error */

/* Serial Extended Mode Register */
#define SEMR_ABCS0      BIT(0)  /* Asynchronous Base Clock Select */
#define SEMR_NFEN       BIT(2)  /* Noise Cancellation Enable */
#define SEMR_DIR        BIT(3)  /* Data Transfer Direction Select */
#define SEMR_MDDRS      BIT(4)  /* Modulation Duty Register Selec */
#define SEMR_BRME       BIT(5)  /* Bit Rate Modulation Enable */
/* Baud Rate Generator Double-Speed Mode Select */
#define SEMR_BGDM       BIT(7)

/* Registers */
static const struct scif_params port_params[NR_REGTYPES] = {
	[SCIF_SCIFA_TYPE] = {
		.regs = {
			[SCSMR]		= { 0x00, 16 },
			[SCBRR]		= { 0x04,  8 },
			[SCSCR]		= { 0x08, 16 },
			[SCFTDR]	= { 0x0c,  8 },
			[SCFSR]		= { 0x10, 16 },
			[SCFRDR]	= { 0x14,  8 },
			[SCFCR]		= { 0x18, 16 },
			[SCFDR]		= { 0x1c, 16 },
			[SCSPTR]	= { 0x20, 16 },
			[SCLSR]		= { 0x24, 16 },
		},
		.init_lsr_mask = SCLSR_ORER | SCLSR_TO_SCIFA,
		.init_interrupt_mask = SCSCR_TIE | SCSCR_RIE | SCSCR_REIE |
				       SCSCR_TEIE_SCIFA | SCSCR_TOIE_SCIFA,
	},
	[SCIF_RZ_SCIFA_TYPE] = {
		.regs = {
			[SCSMR]		= { 0x00, 16 },
			[SCBRR]		= { 0x02,  8 },
			[SCSCR]		= { 0x04, 16 },
			[SCFTDR]	= { 0x06,  8 },
			[SCFSR]		= { 0x08, 16 },
			[SCFRDR]	= { 0x0A,  8 },
			[SCFCR]		= { 0x0C, 16 },
			[SCFDR]		= { 0x0E, 16 },
			[SCSPTR]	= { 0x10, 16 },
			[SCLSR]		= { 0x12, 16 },
			[SEMR]		= { 0x14, 8 },
			[FTCR]		= { 0x16, 16 },
		},
		.init_lsr_mask = SCLSR_ORER,
		.init_interrupt_mask = SCSCR_TIE | SCSCR_RIE | SCSCR_REIE | SCSCR_TEIE_RZ,
	},
};

#define scif_getreg(dev, offset)	\
	&(((const struct uart_scif_cfg *)((dev)->config))->params->regs[offset])

/* TODO: Make unify uart_scif_read/uart_scif_write functions */

static uint8_t uart_scif_read_8(const struct device *dev,
				uint32_t offs)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	return sys_read8(DEVICE_MMIO_GET(dev) + offset);
}

static void uart_scif_write_8(const struct device *dev,
			      uint32_t offs, uint8_t value)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	sys_write8(value, DEVICE_MMIO_GET(dev) + offset);
}

static uint16_t uart_scif_read_16(const struct device *dev,
				  uint32_t offs)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	return sys_read16(DEVICE_MMIO_GET(dev) + offset);
}

static void uart_scif_write_16(const struct device *dev,
			       uint32_t offs, uint16_t value)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	sys_write16(value, DEVICE_MMIO_GET(dev) + offset);
}

static void uart_scif_set_baudrate(const struct device *dev,
				   uint32_t baud_rate)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;
	uint8_t reg_val;

	/* Those formulas for hardcoded parameters */
	if (config->type == SCIF_RZ_SCIFA_TYPE) {
		reg_val = ((data->clk_rate + 4 * baud_rate) / (8 * baud_rate) - 1);
	} else {
		reg_val = ((data->clk_rate + 16 * baud_rate) / (32 * baud_rate) - 1);
	}
	uart_scif_write_8(dev, SCBRR, reg_val);
}

static int uart_scif_poll_in(const struct device *dev, unsigned char *p_char)
{
	struct uart_scif_data *data = dev->data;
	uint16_t reg_val;
	int ret = 0;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Receive FIFO empty */
	if (!((uart_scif_read_16(dev, SCFSR)) & SCFSR_RDF)) {
		ret = -1;
		goto unlock;
	}

	*p_char = uart_scif_read_8(dev, SCFRDR);

	reg_val = uart_scif_read_16(dev, SCFSR);
	reg_val &= ~SCFSR_RDF;
	uart_scif_write_16(dev, SCFSR, reg_val);

unlock:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static void uart_scif_poll_out(const struct device *dev, unsigned char out_char)
{
	struct uart_scif_data *data = dev->data;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for empty space in transmit FIFO */
	while (!(uart_scif_read_16(dev, SCFSR) & SCFSR_TDFE)) {
	}

	uart_scif_write_8(dev, SCFTDR, out_char);

	reg_val = uart_scif_read_16(dev, SCFSR);
	reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
	uart_scif_write_16(dev, SCFSR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_configure(const struct device *dev,
			       const struct uart_config *cfg)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	uint8_t reg_val8;
	k_spinlock_key_t key;

	if (cfg->parity != UART_CFG_PARITY_NONE ||
	    cfg->stop_bits != UART_CFG_STOP_BITS_1 ||
	    cfg->data_bits != UART_CFG_DATA_BITS_8 ||
	    cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	/* Disable Transmit and Receive */
	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_TE | SCSCR_RE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	/* Emptying Transmit and Receive FIFO */
	reg_val = uart_scif_read_16(dev, SCFCR);
	reg_val |= (SCFCR_TFRST | SCFCR_RFRST);
	uart_scif_write_16(dev, SCFCR, reg_val);

	/* Resetting Errors Registers */
	reg_val = uart_scif_read_16(dev, SCFSR);
	reg_val &= ~(SCFSR_ER | SCFSR_DR | SCFSR_BRK | SCFSR_RDF);
	uart_scif_write_16(dev, SCFSR, reg_val);

	reg_val = uart_scif_read_16(dev, SCLSR);
	reg_val &= ~(config->params->init_lsr_mask);
	uart_scif_write_16(dev, SCLSR, reg_val);

	/* Select internal clock */
	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_CKE1 | SCSCR_CKE0);
	uart_scif_write_16(dev, SCSCR, reg_val);

	/* Serial Configuration (8N1) & Clock divider selection */
	reg_val = uart_scif_read_16(dev, SCSMR);
	reg_val &= ~(SCSMR_C_A | SCSMR_CHR | SCSMR_PE | SCSMR_O_E | SCSMR_STOP |
		     SCSMR_CKS1 | SCSMR_CKS0);
	uart_scif_write_16(dev, SCSMR, reg_val);

	/* Set baudrate */
	uart_scif_set_baudrate(dev, cfg->baudrate);

	/* FIFOs data count trigger configuration */
	reg_val = uart_scif_read_16(dev, SCFCR);
	reg_val &= ~(SCFCR_RTRG1 | SCFCR_RTRG0 | SCFCR_TTRG1 | SCFCR_TTRG0 |
		     SCFCR_MCE | SCFCR_TFRST | SCFCR_RFRST);
	uart_scif_write_16(dev, SCFCR, reg_val);

	/* Enable Transmit & Receive + disable Interrupts */
	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_TE | SCSCR_RE);
	reg_val &= ~(config->params->init_interrupt_mask);
	uart_scif_write_16(dev, SCSCR, reg_val);

	if (config->type == SCIF_RZ_SCIFA_TYPE) {
		reg_val8 = uart_scif_read_8(dev, SEMR);
		reg_val8 |= (SEMR_ABCS0 | SEMR_BGDM);
		uart_scif_write_8(dev, SEMR, reg_val8);
	}

	data->current_config = *cfg;

	k_spin_unlock(&data->lock, key);

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_scif_config_get(const struct device *dev,
				struct uart_config *cfg)
{
	struct uart_scif_data *data = dev->data;

	*cfg = data->current_config;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_scif_init(const struct device *dev)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev,
			       (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev,
				     (clock_control_subsys_t)&config->bus_clk,
				     &data->clk_rate);
	if (ret < 0) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = uart_scif_configure(dev, &data->current_config);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static bool uart_scif_irq_is_enabled(const struct device *dev,
				     uint32_t irq)
{
	return !!(uart_scif_read_16(dev, SCSCR) & irq);
}

static int uart_scif_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int len)
{
	struct uart_scif_data *data = dev->data;
	int num_tx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((len - num_tx) > 0) &&
	       (uart_scif_read_16(dev, SCFSR) & SCFSR_TDFE)) {
		/* Send current byte */
		uart_scif_write_8(dev, SCFTDR, tx_data[num_tx]);

		reg_val = uart_scif_read_16(dev, SCFSR);
		reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
		uart_scif_write_16(dev, SCFSR, reg_val);

		num_tx++;
	}

	k_spin_unlock(&data->lock, key);

	return num_tx;
}

static int uart_scif_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int size)
{
	struct uart_scif_data *data = dev->data;
	int num_rx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((size - num_rx) > 0) &&
	       (uart_scif_read_16(dev, SCFSR) & SCFSR_RDF)) {
		/* Receive current byte */
		rx_data[num_rx++] = uart_scif_read_8(dev, SCFRDR);

		reg_val = uart_scif_read_16(dev, SCFSR);
		reg_val &= ~(SCFSR_RDF);
		uart_scif_write_16(dev, SCFSR, reg_val);

	}

	k_spin_unlock(&data->lock, key);

	return num_rx;
}

static void uart_scif_irq_tx_enable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_TIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_scif_irq_tx_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_TIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_irq_tx_ready(const struct device *dev)
{
	return !!(uart_scif_read_16(dev, SCFSR) & SCFSR_TDFE);
}

static void uart_scif_irq_rx_enable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_RIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_scif_irq_rx_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_RIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_irq_rx_ready(const struct device *dev)
{
	return !!(uart_scif_read_16(dev, SCFSR) & SCFSR_RDF);
}

static void uart_scif_irq_err_enable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_REIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_scif_irq_err_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_REIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_irq_is_pending(const struct device *dev)
{
	return (uart_scif_irq_rx_ready(dev) && uart_scif_irq_is_enabled(dev, SCSCR_RIE)) ||
	       (uart_scif_irq_tx_ready(dev) && uart_scif_irq_is_enabled(dev, SCSCR_TIE));
}

static int uart_scif_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_scif_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_scif_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
void uart_scif_isr(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_scif_driver_api = {
	.poll_in = uart_scif_poll_in,
	.poll_out = uart_scif_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_scif_configure,
	.config_get = uart_scif_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_scif_fifo_fill,
	.fifo_read = uart_scif_fifo_read,
	.irq_tx_enable = uart_scif_irq_tx_enable,
	.irq_tx_disable = uart_scif_irq_tx_disable,
	.irq_tx_ready = uart_scif_irq_tx_ready,
	.irq_rx_enable = uart_scif_irq_rx_enable,
	.irq_rx_disable = uart_scif_irq_rx_disable,
	.irq_rx_ready = uart_scif_irq_rx_ready,
	.irq_err_enable = uart_scif_irq_err_enable,
	.irq_err_disable = uart_scif_irq_err_disable,
	.irq_is_pending = uart_scif_irq_is_pending,
	.irq_update = uart_scif_irq_update,
	.irq_callback_set = uart_scif_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
};

/* Device Instantiation */
#define UART_SCIF_DECLARE_CFG(n, soc_type, IRQ_FUNC_INIT)		\
	PINCTRL_DT_INST_DEFINE(n);					\
	static const struct uart_scif_cfg uart_scif_cfg_##n = {		\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),			\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.mod_clk.module =					\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),	\
		.mod_clk.domain =					\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),	\
		.bus_clk.module =					\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),	\
		.bus_clk.domain =					\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.params = &port_params[soc_type],			\
		.type = soc_type,					\
		IRQ_FUNC_INIT						\
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SCIF_IRQ_CONFIG_FUNC_SCIFA(n)				\
	static void irq_config_func_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    uart_scif_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

#define UART_SET_IRQ(n, name)						\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq),		\
			    DT_INST_IRQ_BY_NAME(n, name, priority),	\
			    uart_scif_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));

#define UART_SCIF_IRQ_CONFIG_FUNC_RZ(n)					\
	static void irq_config_func_##n(const struct device *dev)	\
	{								\
		UART_SET_IRQ(n, eri);					\
		UART_SET_IRQ(n, rxi);					\
		UART_SET_IRQ(n, txi);					\
		UART_SET_IRQ(n, tei);					\
	}
#define UART_SCIF_IRQ_CFG_FUNC_INIT(n) \
	.irq_config_func = irq_config_func_##n
#define UART_SCIF_INIT_CFG(n, soc_type) \
	UART_SCIF_DECLARE_CFG(n, soc_type, UART_SCIF_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_SCIF_IRQ_CONFIG_FUNC(n)
#define UART_SCIF_IRQ_CFG_FUNC_INIT
#define UART_SCIF_INIT_CFG(n, soc_type) \
	UART_SCIF_DECLARE_CFG(n, soc_type, UART_SCIF_IRQ_CFG_FUNC_INIT)
#endif

#define UART_SCIF_XXX_INIT(n, soc_type)					\
	static struct uart_scif_data uart_scif_data_##n = {		\
		.current_config = {					\
			.baudrate = DT_INST_PROP(n, current_speed),	\
			.parity = UART_CFG_PARITY_NONE,			\
			.stop_bits = UART_CFG_STOP_BITS_1,		\
			.data_bits = UART_CFG_DATA_BITS_8,		\
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,		\
		},							\
	};								\
	static const struct uart_scif_cfg uart_scif_cfg_##n;		\
	DEVICE_DT_INST_DEFINE(n,					\
			      uart_scif_init,				\
			      NULL,					\
			      &uart_scif_data_##n,			\
			      &uart_scif_cfg_##n,			\
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,\
			      &uart_scif_driver_api);			\
									\
	UART_SCIF_IRQ_CONFIG_FUNC(n)					\
	UART_SCIF_INIT_CFG(n, soc_type);

#define DT_DRV_COMPAT renesas_rcar_scif
#define UART_SCIF_SCIFA_INIT(n) UART_SCIF_XXX_INIT(n, SCIF_SCIFA_TYPE)
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SCIF_IRQ_CONFIG_FUNC UART_SCIF_IRQ_CONFIG_FUNC_SCIFA
#endif
DT_INST_FOREACH_STATUS_OKAY(UART_SCIF_SCIFA_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT renesas_rza2m_scif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#undef UART_SCIF_IRQ_CONFIG_FUNC
#define UART_SCIF_IRQ_CONFIG_FUNC UART_SCIF_IRQ_CONFIG_FUNC_RZ
#endif
#define UART_SCIF_RZ_INIT(n) UART_SCIF_XXX_INIT(n, SCIF_RZ_SCIFA_TYPE)
DT_INST_FOREACH_STATUS_OKAY(UART_SCIF_RZ_INIT)
