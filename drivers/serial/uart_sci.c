/*
 * Copyright (c) 2021 IoT.bzh
 * Copyright (c) 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rza2m_sci

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

struct uart_sci_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

#define INTERRUPT_READ	BIT(0)
#define INTERRUPT_ERR	BIT(1)

struct uart_sci_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct uart_config current_config;
	uint32_t clk_rate;
	struct k_spinlock lock;
	uint8_t read_int;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

#define SCI_DEFAULT_SPEED	115200
#define SCI_DEFAULT_PARITY	UART_CFG_PARITY_NONE
#define SCI_DEFAULT_STOP_BITS	UART_CFG_STOP_BITS_1
#define SCI_DEFAULT_DATA_BITS	UART_CFG_DATA_BITS_8

/* Registers */
#define SCSMR		0x00	/* Serial Mode Register */
#define SCBRR		0x01	/* Bit Rate Register */
#define SCSCR		0x02	/* Serial Control Register */
#define SCTDR		0x03	/* Transmit Data Register */
#define SCSSR		0x04	/* Serial Status Register */
#define SCRDR		0x05	/* Receive Data Register */
#define SCSCMR		0x06	/* Smart Card Mode Register */
#define SCSEMR		0x07	/* Serial Extended Mode Register */
#define SCSNFR		0x08	/* Noise Filter Setting Register */
#define SCSECR		0x0d	/* Extended Function Control Register*/
#define SCTDRH		0x0e	/* Transmit Data Register H */
#define SCTDRL		0x0f	/* Transmit Data Register L */
#define SCTDRHL		0x0e	/* Transmit Data Register HL */
#define SCRDRH		0x10	/* Receive Data Register H */
#define SCRDRL		0x11	/* Receive Data Register L */
#define SCRDRHL		0x10	/* Receive Data Register HL */
#define SCMDRR		0x12	/* Modulation Duty Register */

/* SCSMR (Serial Mode Register) */
#define SCSMR_C_A	BIT(7)  /* Communication Mode */
#define SCSMR_CHR	BIT(6)  /* 7-bit Character Length */
#define SCSMR_PE	BIT(5)  /* Parity Enable */
#define SCSMR_O_E	BIT(4)  /* Odd Parity */
#define SCSMR_STOP	BIT(3)  /* Stop Bit Length */
#define SCSMR_CKS1	BIT(1)  /* Clock Select 1 */
#define SCSMR_CKS0	BIT(0)  /* Clock Select 0 */

/* SCSCR (Serial Control Register) */
#define SCSCR_TIE	BIT(7)  /* Transmit Interrupt Enable */
#define SCSCR_RIE	BIT(6)  /* Receive Interrupt Enable */
#define SCSCR_TE	BIT(5)  /* Transmit Enable */
#define SCSCR_RE	BIT(4)  /* Receive Enable */
#define SCSCR_MPIE	BIT(3)  /* Multi-Processor Interrupt Enable */
#define SCSCR_TEIE	BIT(2)  /* Transmit End Interrupt Enable */
#define SCSCR_CKE1	BIT(1)  /* Clock Enable 1 */
#define SCSCR_CKE0	BIT(0)  /* Clock Enable 0 */

/* SCSSR (Serial Status Register) */
#define SCSSR_TDRE	BIT(7) /* Transmit Data Empty Flag */
#define SCSSR_RDRF	BIT(6) /* Receive Data Full Flag */
#define SCSSR_ORER	BIT(5) /* Overrun Error Flag */
#define SCSSR_FER	BIT(4) /* Framing Error Flag (No Smart Card) */
#define SCSSR_ERS	BIT(4) /* Error Signal Status Flag (Smart Card) */
#define SCSSR_PER	BIT(3) /* Parity Error Flag */
#define SCSSR_TEND	BIT(2) /* Transmit End Flag */
#define SCSSR_MPB	BIT(1) /* Multi-Processor */
#define SCSSR_MPBT	BIT(0) /* Multi-Processor Bit Transfer */

/* SCSCMR (Smart Card Mode Register) */
#define SCSCMR_BCP2	BIT(7) /* Base Clock Pulse 2 */
#define SCSCMR_CHR1	BIT(4) /* Character Length 1 */
#define SCSCMR_SDIR	BIT(3) /* Transmitted/Received Datai Transfer Direction */
#define SCSCMR_SINV	BIT(2) /* Transmitted/Received Data Invert */
#define SCSCMR_SMIF	BIT(0) /* Smart Card Interface Mode Select */

/* SCSEMR (Serial Extended Mode Register) */
#define SCSEMR_RXDESEL	BIT(7) /* Asynchronous Start Bit Edge Detection Select*/
#define SCSEMR_BGDM	BIT(6) /* Baud Rate Generator Double-Speed Mode Select*/
#define SCSEMR_NFEN	BIT(5) /* Digital Noise Filter Function Enable*/
#define SCSEMR_ABCS	BIT(4) /* Asynchronous Mode Base Clock Select*/
#define SCSEMR_BRME	BIT(2) /* Bit Rate Modulation Enable*/
#define SCSEMR_ACS0	BIT(0) /* Asynchronous Mode Clock Source Select*/

static void uart_sci_write_8(const struct device *dev, uint32_t offs, uint8_t value)
{
	sys_write8(value, DEVICE_MMIO_GET(dev) + offs);
}

static uint8_t uart_sci_read_8(const struct device *dev, uint32_t offs)
{
	return sys_read8(DEVICE_MMIO_GET(dev) + offs);
}

static void uart_sci_set_baudrate(const struct device *dev, uint32_t baud_rate)
{
	struct uart_sci_data *data = dev->data;
	uint8_t reg_val;

	reg_val = (data->clk_rate + 16 * baud_rate) / (32 * baud_rate) - 1;
	uart_sci_write_8(dev, SCBRR, reg_val);
}

static int uart_sci_poll_in(const struct device *dev, unsigned char *p_char)
{
	struct uart_sci_data *data = dev->data;
	uint8_t reg_val;
	int ret = 0;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	*p_char = uart_sci_read_8(dev, SCSSR);

	reg_val = uart_sci_read_8(dev, SCSSR);
	if (!(reg_val & SCSSR_RDRF)) {
		if (reg_val & (SCSSR_ORER | SCSSR_FER | SCSSR_PER)) {
			reg_val &= ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER);
			uart_sci_write_8(dev, SCSSR, reg_val);
		}
		ret = -1;
		goto unlock;
	}

	*p_char = uart_sci_read_8(dev, SCRDR);

	reg_val = uart_sci_read_8(dev, SCSSR);
	reg_val &= ~SCSSR_RDRF;
	uart_sci_write_8(dev, SCSSR, reg_val);

unlock:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static void uart_sci_poll_out(const struct device *dev, unsigned char out_char)
{
	struct uart_sci_data *data = dev->data;
	uint8_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for empty space in transmit FIFO */
	while (!(uart_sci_read_8(dev, SCSSR) & SCSSR_TDRE)) {
	}

	uart_sci_write_8(dev, SCTDR, out_char);

	reg_val = uart_sci_read_8(dev, SCSSR);
	reg_val &= ~(SCSSR_TDRE | SCSSR_TEND);
	uart_sci_write_8(dev, SCSSR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_sci_configure(const struct device *dev, const struct uart_config *cfg)
{
	struct uart_sci_data *data = dev->data;
	uint8_t reg_val;

	k_spinlock_key_t key;

	if (cfg->data_bits < UART_CFG_DATA_BITS_7 ||
	    cfg->data_bits > UART_CFG_DATA_BITS_8 ||
	    cfg->stop_bits == UART_CFG_STOP_BITS_0_5 ||
	    cfg->stop_bits == UART_CFG_STOP_BITS_1_5 ||
	    cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	/* Disable Transmit and Receive */
	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val &= ~(SCSCR_TE | SCSCR_RE | SCSCR_RIE | SCSCR_TIE | SCSCR_TEIE);
	uart_sci_write_8(dev, SCSCR, reg_val);

	/* Resetting Errors Registers */
	reg_val = uart_sci_read_8(dev, SCSSR);
	reg_val &= ~(SCSSR_PER | SCSSR_FER | SCSSR_ORER | SCSSR_RDRF);
	uart_sci_write_8(dev, SCSSR, reg_val);

	/* Select internal clock */
	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val &= ~(SCSCR_CKE1 | SCSCR_CKE0);
	uart_sci_write_8(dev, SCSCR, reg_val);

	/* Serial Configuration (8N1) & Clock divider selection */
	reg_val = uart_sci_read_8(dev, SCSMR);
	reg_val &= ~(SCSMR_C_A | SCSMR_CHR | SCSMR_PE | SCSMR_O_E | SCSMR_STOP |
		     SCSMR_CKS1 | SCSMR_CKS0);
	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		break;
	case UART_CFG_PARITY_ODD:
		reg_val |= SCSMR_PE | SCSMR_O_E;
		break;
	case UART_CFG_PARITY_EVEN:
		reg_val |= SCSMR_PE;
		break;
	default:
		return -ENOTSUP;
	}
	if (cfg->stop_bits == UART_CFG_STOP_BITS_2) {
		reg_val |= SCSMR_STOP;
	}
	if (cfg->data_bits == UART_CFG_DATA_BITS_7) {
		reg_val |= SCSMR_CHR;
	}
	uart_sci_write_8(dev, SCSMR, reg_val);
	reg_val = uart_sci_read_8(dev, SCSCMR);
	reg_val |= SCSCMR_CHR1;
	uart_sci_write_8(dev, SCSCMR, reg_val);

	/* Set baudrate */
	uart_sci_set_baudrate(dev, cfg->baudrate);

	/* Enable Transmit & Receive + disable Interrupts */
	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val |= (SCSCR_TE);
	uart_sci_write_8(dev, SCSCR, reg_val);
	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val |= (SCSCR_RE);
	uart_sci_write_8(dev, SCSCR, reg_val);

	data->current_config = *cfg;

	k_spin_unlock(&data->lock, key);

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_sci_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_sci_data *data = dev->data;

	*cfg = data->current_config;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_sci_init(const struct device *dev)
{
	const struct uart_sci_cfg *config = dev->config;
	struct uart_sci_data *data = dev->data;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
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

	ret = uart_sci_configure(dev, &data->current_config);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static bool uart_sci_irq_is_enabled(const struct device *dev, uint32_t irq)
{
	return !!(uart_sci_read_8(dev, SCSCR) & irq);
}

static int uart_sci_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	struct uart_sci_data *data = dev->data;
	int num_tx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((len - num_tx) > 0) && (uart_sci_read_8(dev, SCSSR) & SCSSR_TDRE)) {
		/* Send current byte */
		uart_sci_write_8(dev, SCTDR, tx_data[num_tx]);

		reg_val = uart_sci_read_8(dev, SCSSR);
		reg_val &= ~(SCSSR_TDRE | SCSSR_TEND);
		uart_sci_write_8(dev, SCSSR, reg_val);

		num_tx++;
	}

	k_spin_unlock(&data->lock, key);

	return num_tx;
}

static int uart_sci_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	struct uart_sci_data *data = dev->data;
	int num_rx = 0;
	uint8_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((size - num_rx) > 0) && (uart_sci_read_8(dev, SCSSR) & SCSSR_RDRF)) {
		/* Receive current byte */
		rx_data[num_rx++] = uart_sci_read_8(dev, SCRDR);

		reg_val = uart_sci_read_8(dev, SCSSR);
		reg_val &= ~(SCSSR_RDRF);
		uart_sci_write_8(dev, SCSSR, reg_val);

	}

	k_spin_unlock(&data->lock, key);

	return num_rx;
}

static void uart_sci_irq_tx_enable(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;
	uint8_t reg_val;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val |= (SCSCR_TIE | SCSCR_TEIE);
	uart_sci_write_8(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_sci_irq_tx_disable(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val &= ~(SCSCR_TIE | SCSCR_TEIE);
	uart_sci_write_8(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_sci_irq_tx_ready(const struct device *dev)
{
	return !!(uart_sci_read_8(dev, SCSSR) & SCSSR_TDRE);
}

static void uart_sci_irq_rx_enable(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;
	uint8_t reg_val;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->read_int |= INTERRUPT_READ;

	reg_val = uart_sci_read_8(dev, SCSCR);

	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val |= (SCSCR_RIE);
	uart_sci_write_8(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_sci_irq_rx_disable(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->read_int &= ~(INTERRUPT_READ);
	if (data->read_int) {
		reg_val = uart_sci_read_8(dev, SCSCR);
		reg_val &= ~(SCSCR_RIE);
		uart_sci_write_8(dev, SCSCR, reg_val);
	}

	k_spin_unlock(&data->lock, key);
}

static int uart_sci_irq_rx_ready(const struct device *dev)
{
	return !!(uart_sci_read_8(dev, SCSSR) & SCSSR_RDRF);
}

static void uart_sci_irq_err_enable(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->read_int |= INTERRUPT_ERR;

	reg_val = uart_sci_read_8(dev, SCSCR);
	reg_val |= (SCSCR_RIE);
	uart_sci_write_8(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_sci_irq_err_disable(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->read_int &= ~(INTERRUPT_ERR);
	if (data->read_int) {
		reg_val = uart_sci_read_8(dev, SCSCR);
		reg_val &= ~(SCSCR_RIE);
		uart_sci_write_8(dev, SCSCR, reg_val);
	}

	k_spin_unlock(&data->lock, key);
}

static int uart_sci_irq_is_pending(const struct device *dev)
{
	return (uart_sci_irq_rx_ready(dev) && uart_sci_irq_is_enabled(dev, SCSCR_RIE)) ||
	       (uart_sci_irq_tx_ready(dev) && uart_sci_irq_is_enabled(dev, SCSCR_TIE));
}

static int uart_sci_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_sci_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *cb_data)
{
	struct uart_sci_data *data = dev->data;

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

void uart_sci_isr_common(const struct device *dev)
{
	struct uart_sci_data *data = dev->data;

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
}

void uart_sci_isr_eri(const struct device *dev)
{
	uint8_t reg_val;

	uart_sci_isr_common(dev);
	reg_val = uart_sci_read_8(dev, SCSSR);
	reg_val &= ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER);
	uart_sci_write_8(dev, SCSSR, reg_val);
}

void uart_sci_isr_rxi(const struct device *dev)
{
	uart_sci_isr_common(dev);
}

void uart_sci_isr_txi(const struct device *dev)
{
	uart_sci_isr_common(dev);
}

void uart_sci_isr_tei(const struct device *dev)
{
	uart_sci_isr_common(dev);
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_sci_driver_api = {
	.poll_in = uart_sci_poll_in,
	.poll_out = uart_sci_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_sci_configure,
	.config_get = uart_sci_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_sci_fifo_fill,
	.fifo_read = uart_sci_fifo_read,
	.irq_tx_enable = uart_sci_irq_tx_enable,
	.irq_tx_disable = uart_sci_irq_tx_disable,
	.irq_tx_ready = uart_sci_irq_tx_ready,
	.irq_rx_enable = uart_sci_irq_rx_enable,
	.irq_rx_disable = uart_sci_irq_rx_disable,
	.irq_rx_ready = uart_sci_irq_rx_ready,
	.irq_err_enable = uart_sci_irq_err_enable,
	.irq_err_disable = uart_sci_irq_err_disable,
	.irq_is_pending = uart_sci_irq_is_pending,
	.irq_update = uart_sci_irq_update,
	.irq_callback_set = uart_sci_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
};

/* Device Instantiation */
#define UART_SCI_DECLARE_CFG(n, IRQ_FUNC_INIT)					\
	PINCTRL_DT_INST_DEFINE(n);						\
	static const struct uart_sci_cfg uart_sci_cfg_##n = {			\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),				\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
		.mod_clk.module =						\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),		\
		.mod_clk.domain =						\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),		\
		.bus_clk.module =						\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),		\
		.bus_clk.domain =						\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		IRQ_FUNC_INIT							\
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SET_IRQ(n, name)							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq),			\
			DT_INST_IRQ_BY_NAME(n, name, priority),			\
			uart_sci_isr_##name,					\
			DEVICE_DT_INST_GET(n),					\
			DT_INST_IRQ_BY_NAME(n, name, flags));			\
										\
		irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));

#define UART_SCI_CONFIG_FUNC(n)							\
	static void irq_config_func_##n(const struct device *dev)		\
	{									\
		UART_SET_IRQ(n, eri);						\
		UART_SET_IRQ(n, rxi);						\
		UART_SET_IRQ(n, txi);						\
		UART_SET_IRQ(n, tei);						\
	}

#define UART_SCI_IRQ_CFG_FUNC_INIT(n) \
	.irq_config_func = irq_config_func_##n
#define UART_SCI_INIT_CFG(n) \
	UART_SCI_DECLARE_CFG(n, UART_SCI_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_SCI_CONFIG_FUNC(n)
#define UART_SCI_IRQ_CFG_FUNC_INIT
#define UART_SCI_INIT_CFG(n) \
	UART_SCI_DECLARE_CFG(n, UART_SCI_IRQ_CFG_FUNC_INIT)
#endif

#define UART_SCI_INIT(n)							\
	static struct uart_sci_data uart_sci_data_##n = {			\
		.current_config = {						\
			.baudrate = DT_INST_PROP_OR(n, current_speed,		\
						 SCI_DEFAULT_SPEED),		\
			.parity = DT_INST_ENUM_IDX_OR(n, parity,		\
						SCI_DEFAULT_PARITY),		\
			.stop_bits = DT_INST_ENUM_IDX_OR(n, stop_bits,		\
						SCI_DEFAULT_STOP_BITS),		\
			.data_bits = DT_INST_ENUM_IDX_OR(n, data_bits,		\
						SCI_DEFAULT_DATA_BITS),		\
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,			\
		},								\
	};									\
										\
	static const struct uart_sci_cfg uart_sci_cfg_##n;			\
										\
	DEVICE_DT_INST_DEFINE(n, uart_sci_init,					\
			NULL,							\
			&uart_sci_data_##n,					\
			&uart_sci_cfg_##n,					\
			PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,		\
			&uart_sci_driver_api);					\
										\
	UART_SCI_CONFIG_FUNC(n)							\
										\
	UART_SCI_INIT_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(UART_SCI_INIT)
