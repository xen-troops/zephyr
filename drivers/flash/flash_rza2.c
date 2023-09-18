/*
 * Copyright (c) 2023 Epam Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rza2_flash_controller

#include <stddef.h>
#include <zephyr/arch/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/mem_manage.h>
#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#include "spi_rza2.h"

LOG_MODULE_REGISTER(flash_rza2, CONFIG_FLASH_LOG_LEVEL);

#define ERASE_VALUE 0xff

#define FLASH_SIZE KB(CONFIG_FLASH_SIZE)
#define FLASH_BASE CONFIG_FLASH_BASE_ADDRESS

#define FLASH_PAGE_SIZE   DT_PROP(DT_CHOSEN(zephyr_flash), write_block_size)
/* Maximum size per one writing is 256 byte and minimum size per one writing is 1 byte */
#define FLASH_SECTOR_SIZE DT_PROP(DT_CHOSEN(zephyr_flash), erase_block_size)

static const struct flash_parameters flash_rza2_parameters = {
	.write_block_size = FLASH_PAGE_SIZE,
	.erase_value = ERASE_VALUE,
};

enum flash_rza2_types {
	SERIAL_FLASH,
	HYPER_FLASH,
	OCTA_FLASH,
};

struct flash_rza2_config {
	enum flash_rza2_types type;
	const struct pinctrl_dev_config *pcfg;
};

K_MUTEX_DEFINE(lock);

/*
 * TODO: This commands are flash-specific. Another Vendor may not support those
 * commands so they should be configurable.
 */
/* Serial Flash commands*/
#define SPI_OUTPUT_ADDR           SPI_OUTPUT_ADDR_32
#define SFLASHCMD_SECTOR_ERASE    (0x21u) /* SE4B 4-byte address(1bit) */
#define SFLASHCMD_PAGE_PROGRAM    (0x12u) /* PP4B 4-byte address(1bit), data(1bit) */
#define SFLASHCMD_PAGE_READ       (0x13u) /* READ4B 4-byte address(1bit), data(1bit) */
#define SFLASHCMD_READ_NORMAL     (0x3u)  /* Normal read operation */
#define SFLASHCMD_READ_STATUS_REG (0x05u) /* RDSR data(1bit) */
#define SFLASHCMD_WRITE_ENABLE    (0x06u) /* WREN */

/* serial flash register definitions */
#define STREG_BUSY_BIT (0x01u) /* SR.[0]BUSY Erase/Write In Progress (RO) */

static uint8_t write_tmp_buf[FLASH_PAGE_SIZE];

struct spibsc_reg {
	uint32_t cdb;   /* bit-width : command */
	uint32_t ocdb;  /* bit-width : optional command */
	uint32_t adb;   /* bit-width : address */
	uint32_t opdb;  /* bit-width : option data */
	uint32_t spidb; /* bit-width : data */

	uint32_t cde;   /* Enable : command */
	uint32_t ocde;  /* Enable : optional command */
	uint32_t ade;   /* Enable : address */
	uint32_t opde;  /* Enable : option data */
	uint32_t spide; /* Enable : data */

	uint32_t sslkp; /* SPBSSL level */
	uint32_t spire; /* Enable data read */
	uint32_t spiwe; /* Enable data write */

	uint32_t dme; /* Enable : dummy cycle */

	uint32_t addre;  /* DDR enable : address  */
	uint32_t opdre;  /* DDR enable : option data */
	uint32_t spidre; /* DDR enable : data */

	uint8_t dmdb;  /* bit-width : dummy cycle */
	uint8_t dmcyc; /* number of dummy cycles */

	uint8_t cmd;       /* command */
	uint8_t ocmd;      /* optional command */
	uint32_t addr;     /* address */
	uint8_t opd[4];    /* option data 3/2/1/0 */
	uint32_t smrdr[2]; /* read data */
	uint32_t smwdr[2]; /* write data */
};

DEVICE_MMIO_TOPLEVEL_STATIC(spi_reg, DT_DRV_INST(0));
#define RZA2_SPI_REG_BASE DEVICE_MMIO_TOPLEVEL_GET(spi_reg)

static uint32_t spi_reg_read(uint32_t off, uint32_t shift, uint32_t mask)
{
	uint32_t val;

	val = sys_read32(RZA2_SPI_REG_BASE + off);
	return (val & mask) >> shift;
}

static void spi_reg_write(uint32_t off, uint32_t write_value, uint32_t shift, uint32_t mask)
{
	uint32_t val;

	val = sys_read32(RZA2_SPI_REG_BASE + off);
	val = (val & (~mask)) | (write_value << shift);
	sys_write32(val, RZA2_SPI_REG_BASE + off);
}

static void spi_smrdr(uint32_t off, uint32_t spide, uint32_t *smrdr)
{
	switch (spide) {
	case SPI_OUTPUT_SPID_8:
		*smrdr = sys_read8(RZA2_SPI_REG_BASE + off);
		break;
	case SPI_OUTPUT_SPID_16:
		*smrdr = sys_read16(RZA2_SPI_REG_BASE + off);
		break;
	case SPI_OUTPUT_SPID_32:
		*smrdr = sys_read32(RZA2_SPI_REG_BASE + off);
		break;
	}
}

static void spi_smwdr(uint32_t off, uint32_t spide, uint32_t smwdr)
{
	switch (spide) {
	case SPI_OUTPUT_SPID_8:
		sys_write8((uint8_t)smwdr, RZA2_SPI_REG_BASE + off);
		break;
	case SPI_OUTPUT_SPID_16:
		sys_write16((uint16_t)smwdr, RZA2_SPI_REG_BASE + off);
		break;
	case SPI_OUTPUT_SPID_32:
		sys_write32(smwdr, RZA2_SPI_REG_BASE + off);
		break;
	}
}

static int32_t spi_xfer(struct spibsc_reg *regset)
{
	if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_MD_SHIFT, SPI_CMNCR_MD) != SPI_CMNCR_MD_SPI) {
		if (spi_reg_read(CMNSR_OFF, SPI_CMNSR_SSLF_SHIFT, SPI_CMNSR_SSLF) !=
		    SPI_SSL_NEGATE) {
			LOG_ERR("%s: SSL is in the high state", __func__);
			return -EBUSY;
		}
		/* SPI Mode */
		spi_reg_write(CMNCR_OFF, SPI_CMNCR_MD_SPI, SPI_CMNCR_MD_SHIFT, SPI_CMNCR_MD);
	}

	if (spi_reg_read(CMNSR_OFF, SPI_CMNSR_TEND_SHIFT, SPI_CMNSR_TEND) != SPI_TRANS_END) {
		LOG_ERR("%s: transaction is still in progress", __func__);
		return -EBUSY;
	}

	/* Command Sectiom */
	/* Enable/Disable */
	spi_reg_write(SMENR_OFF, regset->cde, SPI_SMENR_CDE_SHIFT, SPI_SMENR_CDE);
	if (regset->cde != SPI_OUTPUT_DISABLE) {
		/* Command */
		spi_reg_write(SMCMR_OFF, regset->cmd, SPI_SMCMR_CMD_SHIFT, SPI_SMCMR_CMD);
		/* Single/Dual/Quad */
		spi_reg_write(SMENR_OFF, regset->cdb, SPI_SMENR_CDB_SHIFT, SPI_SMENR_CDB);
	}

	/* Option Command Section */
	/* Enable/Disable */
	spi_reg_write(SMENR_OFF, regset->ocde, SPI_SMENR_OCDE_SHIFT, SPI_SMENR_OCDE);
	if (regset->ocde != SPI_OUTPUT_DISABLE) {
		/* Option Command */
		spi_reg_write(SMCMR_OFF, regset->ocmd, SPI_SMCMR_OCMD_SHIFT, SPI_SMCMR_OCMD);
		/* Single/Dual/Quad */
		spi_reg_write(SMENR_OFF, regset->ocdb, SPI_SMENR_OCDB_SHIFT, SPI_SMENR_OCDB);
	}

	/* Address Section */
	/* Enable/Disable */
	spi_reg_write(SMENR_OFF, regset->ade, SPI_SMENR_ADE_SHIFT, SPI_SMENR_ADE);
	if (regset->ade != SPI_OUTPUT_DISABLE) {
		/* Address */
		spi_reg_write(SMADR_OFF, regset->addr, SPI_SMADR_ADR_SHIFT, SPI_SMADR_ADR);
		/* Single/Dual/Quad */
		spi_reg_write(SMENR_OFF, regset->adb, SPI_SMENR_ADB_SHIFT, SPI_SMENR_ADB);
	}

	/* Option Data Section */
	/* Enable/Disable */
	spi_reg_write(SMENR_OFF, regset->opde, SPI_SMENR_OPDE_SHIFT, SPI_SMENR_OPDE);
	if (regset->opde != SPI_OUTPUT_DISABLE) {
		/* Option Data */
		spi_reg_write(SMOPR_OFF, regset->opd[0], SPI_SMOPR_OPD3_SHIFT, SPI_SMOPR_OPD3);
		spi_reg_write(SMOPR_OFF, regset->opd[1], SPI_SMOPR_OPD2_SHIFT, SPI_SMOPR_OPD2);
		spi_reg_write(SMOPR_OFF, regset->opd[2], SPI_SMOPR_OPD1_SHIFT, SPI_SMOPR_OPD1);
		spi_reg_write(SMOPR_OFF, regset->opd[3], SPI_SMOPR_OPD0_SHIFT, SPI_SMOPR_OPD0);
		/* Single/Dual/Quad */
		spi_reg_write(SMENR_OFF, regset->opdb, SPI_SMENR_OPDB_SHIFT, SPI_SMENR_OPDB);
	}

	/* Dummy Cycles */
	/* Enable/Disable */
	spi_reg_write(SMENR_OFF, regset->dme, SPI_SMENR_DME_SHIFT, SPI_SMENR_DME);
	if (regset->dme != SPI_DUMMY_CYC_DISABLE) {
		/* Dummy Cycle */
		spi_reg_write(SMDMCR_OFF, regset->dmcyc, SPI_SMDMCR_DMCYC_SHIFT, SPI_SMDMCR_DMCYC);
	}

	/* Data Section */
	/* Enable/Disable */
	spi_reg_write(SMENR_OFF, regset->spide, SPI_SMENR_SPIDE_SHIFT, SPI_SMENR_SPIDE);
	if (regset->spide != SPI_OUTPUT_DISABLE) {
		if (regset->spide == SPI_OUTPUT_SPID_8) {
			if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_BSZ_SHIFT, SPI_CMNCR_BSZ) ==
			    SPI_CMNCR_BSZ_SINGLE) {
				spi_smwdr(SMWDR0_OFF, regset->spide, regset->smwdr[0]);
			} else {
				spi_smwdr(SMWDR0_OFF, SPI_OUTPUT_SPID_16, regset->smwdr[0]);
			}
		} else if (regset->spide == SPI_OUTPUT_SPID_16) {
			if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_BSZ_SHIFT, SPI_CMNCR_BSZ) ==
			    SPI_CMNCR_BSZ_SINGLE) {
				spi_smwdr(SMWDR0_OFF, regset->spide, regset->smwdr[0]);
			} else {
				spi_smwdr(SMWDR0_OFF, SPI_OUTPUT_SPID_32, regset->smwdr[0]);
			}
		} else if (regset->spide == SPI_OUTPUT_SPID_32) {
			if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_BSZ_SHIFT, SPI_CMNCR_BSZ) ==
			    SPI_CMNCR_BSZ_SINGLE) {
				spi_smwdr(SMWDR0_OFF, regset->spide, regset->smwdr[0]);
			} else {
				spi_smwdr(SMWDR0_OFF, regset->spide, regset->smwdr[0]);
				spi_smwdr(SMWDR1_OFF, regset->spide, regset->smwdr[1]);
			}
		}

		/* Single/Dual/Quad */
		spi_reg_write(SMENR_OFF, regset->spidb, SPI_SMENR_SPIDB_SHIFT, SPI_SMENR_SPIDB);
	}

	spi_reg_write(SMCR_OFF, regset->sslkp, SPI_SMCR_SSLKP_SHIFT, SPI_SMCR_SSLKP);

	if ((regset->spidb != SPI_1BIT) && (regset->spide != SPI_OUTPUT_DISABLE)) {
		if ((regset->spire == SPI_SPIDATA_ENABLE) &&
		    (regset->spiwe == SPI_SPIDATA_ENABLE)) {
			LOG_ERR("Read/Write mode is not supported for data width != 1 bit");
			return -EINVAL;
		}
	}

	spi_reg_write(SMCR_OFF, regset->spire, SPI_SMCR_SPIRE_SHIFT, SPI_SMCR_SPIRE);
	spi_reg_write(SMCR_OFF, regset->spiwe, SPI_SMCR_SPIWE_SHIFT, SPI_SMCR_SPIWE);

	/* SDR Transmission/DDR Transmission Setting */
	spi_reg_write(SMDRENR_OFF, regset->addre, SPI_SMDRENR_ADDRE_SHIFT, SPI_SMDRENR_ADDRE);
	spi_reg_write(SMDRENR_OFF, regset->opdre, SPI_SMDRENR_OPDRE_SHIFT, SPI_SMDRENR_OPDRE);
	spi_reg_write(SMDRENR_OFF, regset->spidre, SPI_SMDRENR_SPIDRE_SHIFT, SPI_SMDRENR_SPIDRE);

	/* execute after setting SPNDL bit */
	spi_reg_write(SMCR_OFF, SPI_SPI_ENABLE, SPI_SMCR_SPIE_SHIFT, SPI_SMCR_SPIE);

	/* wait for transfer-start */
	while (spi_reg_read(CMNSR_OFF, SPI_CMNSR_TEND_SHIFT, SPI_CMNSR_TEND) != SPI_TRANS_END) {
		/* wait for transfer-end */
	}

	if (regset->spide == SPI_OUTPUT_SPID_8) {
		if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_BSZ_SHIFT, SPI_CMNCR_BSZ) ==
		    SPI_CMNCR_BSZ_SINGLE) {
			spi_smrdr(SMRDR0_OFF, regset->spide, &regset->smrdr[0]);
		} else {
			spi_smrdr(SMRDR0_OFF, SPI_OUTPUT_SPID_16, &regset->smrdr[0]);
		}
	} else if (regset->spide == SPI_OUTPUT_SPID_16) {
		if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_BSZ_SHIFT, SPI_CMNCR_BSZ) ==
		    SPI_CMNCR_BSZ_SINGLE) {
			spi_smrdr(SMRDR0_OFF, regset->spide, &regset->smrdr[0]);
		} else {
			spi_smrdr(SMRDR0_OFF, SPI_OUTPUT_SPID_32, &regset->smrdr[0]);
		}
	} else if (regset->spide == SPI_OUTPUT_SPID_32) {
		if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_BSZ_SHIFT, SPI_CMNCR_BSZ) ==
		    SPI_CMNCR_BSZ_SINGLE) {
			spi_smrdr(SMRDR0_OFF, regset->spide, &regset->smrdr[0]);
		} else {
			spi_smrdr(SMRDR0_OFF, regset->spide, &regset->smrdr[0]);
			spi_smrdr(SMRDR1_OFF, regset->spide, &regset->smrdr[1]);
		}
	}

	return 0;
}

static void spi_stop(void)
{
	uint32_t value = sys_read32(RZA2_SPI_REG_BASE + DRCR_OFF);

	if (((value & SPI_DRCR_RBE) != 0) && ((value & SPI_DRCR_SSLE) != 0)) {
		spi_reg_write(DRCR_OFF, 1, SPI_DRCR_SSLN_SHIFT, SPI_DRCR_SSLN);
	}

	while (spi_reg_read(CMNSR_OFF, SPI_CMNSR_SSLF_SHIFT, SPI_CMNSR_SSLF) != SPI_SSL_NEGATE) {
		;
	}

	while (spi_reg_read(CMNSR_OFF, SPI_CMNSR_TEND_SHIFT, SPI_CMNSR_TEND) != SPI_TRANS_END) {
		;
	}
}
static void clear_spi_reg(struct spibsc_reg *regset)
{
	regset->cde = SPI_OUTPUT_DISABLE;
	regset->cdb = SPI_1BIT;
	regset->cmd = 0x00;

	regset->ocde = SPI_OUTPUT_DISABLE;
	regset->ocdb = SPI_1BIT;
	regset->ocmd = 0x00;

	regset->ade = SPI_OUTPUT_DISABLE;
	regset->addre = SPI_SDR_TRANS; /* SDR */
	regset->adb = SPI_1BIT;
	regset->addr = 0x00000000;

	regset->opde = SPI_OUTPUT_DISABLE;
	regset->opdre = SPI_SDR_TRANS; /* SDR */
	regset->opdb = SPI_1BIT;
	regset->opd[0] = 0x00; /* OPD3 */
	regset->opd[1] = 0x00; /* OPD2 */
	regset->opd[2] = 0x00; /* OPD1 */
	regset->opd[3] = 0x00; /* OPD0 */

	regset->dme = SPI_DUMMY_CYC_DISABLE;
	regset->dmdb = SPI_1BIT;
	regset->dmcyc = SPI_DUMMY_1CYC;

	regset->spide = SPI_OUTPUT_DISABLE;
	regset->spidre = SPI_SDR_TRANS; /* SDR */
	regset->spidb = SPI_1BIT;

	regset->sslkp = SPI_SPISSL_NEGATE;   /* SPBSSL level */
	regset->spire = SPI_SPIDATA_DISABLE; /* read enable/disable */
	regset->spiwe = SPI_SPIDATA_DISABLE; /* write enable/disable */
}

static int32_t read_register(uint8_t cmd, uint8_t *status)
{
	int32_t ret;
	struct spibsc_reg spimd_reg;

	clear_spi_reg(&spimd_reg);

	spimd_reg.cde = SPI_OUTPUT_ENABLE;
	spimd_reg.cdb = SPI_1BIT;
	spimd_reg.cmd = cmd;

	spimd_reg.sslkp = SPI_SPISSL_NEGATE;  /* SPBSSL level */
	spimd_reg.spire = SPI_SPIDATA_ENABLE; /* read enable/disable */
	spimd_reg.spiwe = SPI_SPIDATA_ENABLE; /* write enable/disable */

	spimd_reg.spide = SPI_OUTPUT_SPID_8; /* Enable(8bit) */
	spimd_reg.spidre = SPI_SDR_TRANS;    /* SDR */
	spimd_reg.spidb = SPI_1BIT;
	spimd_reg.smwdr[0] = 0x00; /* Output 0 in read status */
	spimd_reg.smwdr[1] = 0x00; /* Output 0 in read status */

	ret = spi_xfer(&spimd_reg);
	if (ret == 0) {
		*status = (uint8_t)(spimd_reg.smrdr[0]); /* Data[7:0]  */
	}

	return ret;
}

static int32_t wait_status_from_flash(void)
{
	int32_t ret = 0;
	uint8_t st_reg;

	while (1) {
		ret = read_register(SFLASHCMD_READ_STATUS_REG, &st_reg);
		if (ret != 0) {
			break;
		}
		if ((st_reg & STREG_BUSY_BIT) == 0) {
			break;
		}
	}

	return ret;
}

static void ex_mode(void)
{
	uint32_t cmncr;
	uint32_t value;

	if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_MD_SHIFT, SPI_CMNCR_MD) == SPI_CMNCR_MD_EXTRD) {
		return;
	}

	spi_stop();

	/* Flush SPIBSC's read cache */
	spi_reg_write(DRCR_OFF, SPI_DRCR_RCF_EXE, SPI_DRCR_RCF_SHIFT, SPI_DRCR_RCF);
	/* Dummy read */
	sys_read32(RZA2_SPI_REG_BASE + DRCR_OFF);

	cmncr = 0;
	/* External address mode */
	cmncr = (cmncr & (~SPI_CMNCR_MD)) | (SPI_CMNCR_MD_EXTRD << SPI_CMNCR_MD_SHIFT);

	/* Only one connected serial flash is supported */
	cmncr = (cmncr & (~SPI_CMNCR_BSZ)) | (0 << SPI_CMNCR_BSZ_SHIFT);
	cmncr = (cmncr & (~SPI_CMNCR_IO0FV)) | (2 << SPI_CMNCR_IO0FV_SHIFT);
	cmncr = (cmncr & (~SPI_CMNCR_IO2FV)) | (2 << SPI_CMNCR_IO2FV_SHIFT);
	cmncr = (cmncr & (~SPI_CMNCR_IO3FV)) | (2 << SPI_CMNCR_IO3FV_SHIFT);
	/*MOIIO3*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO3)) | (2 << SPI_CMNCR_MOIIO3_SHIFT);
	/*MOIIO2*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO2)) | (2 << SPI_CMNCR_MOIIO2_SHIFT);
	/*MOIIO1*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO1)) | (2 << SPI_CMNCR_MOIIO1_SHIFT);
	/*MOIIO0*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO0)) | (2 << SPI_CMNCR_MOIIO0_SHIFT);
	/* Set reserved bit to 1 */
	cmncr |= (1 << SPI_CMNCR_RESERV_SHIFT);
	/* Dummy read */
	sys_read32(RZA2_SPI_REG_BASE + CMNCR_OFF);

	sys_write32(cmncr, RZA2_SPI_REG_BASE + CMNCR_OFF); /* Write to register */

	/* Dummy read */
	sys_read32(RZA2_SPI_REG_BASE + CMNCR_OFF);

	/* Set DRCR */
	value = 0;

	/* Enable Read Burst */
	value = (value & (~SPI_DRCR_RBE)) | (1 << SPI_DRCR_RBE_SHIFT);

	/* Set 3 continious data units in read burst */
	value = (value & (~SPI_DRCR_RBURST)) | (3 << SPI_DRCR_RBURST_SHIFT);
	sys_write32(value, RZA2_SPI_REG_BASE + DRCR_OFF); /* Write to register */

	/* Set DRCMR */
	spi_reg_write(DRCMR_OFF, SFLASHCMD_READ_NORMAL, SPI_DRCMR_CMD_SHIFT, SPI_DRCMR_CMD);

	/* Clean DREAR */
	sys_write32(0, RZA2_SPI_REG_BASE + DREAR_OFF);
	/* Clear DROPR */
	sys_write32(0, RZA2_SPI_REG_BASE + DROPR_OFF);

	/* Set DRENR */
	value = 0;
	value = (value & (~SPI_DRENR_ADE)) | (0b0111 << SPI_DRENR_ADE_SHIFT);
	value = (value & (~SPI_DRENR_DME)) | (0 << SPI_DRENR_DME_SHIFT);
	value = (value & (~SPI_DRENR_CDE)) | (1 << SPI_DRENR_CDE_SHIFT);
	sys_write32(value, RZA2_SPI_REG_BASE + DRENR_OFF); /* Write to register */

	/* Clear DRDMCR */
	sys_write32(0, RZA2_SPI_REG_BASE + DRDMCR_OFF);

	/* Clear DRDRENR */
	sys_write32(0, RZA2_SPI_REG_BASE + DRDRENR_OFF);

	/* Clean PHYCNT. Bits 5, 6 and 9 should be 1 according to HW manual */
	spi_reg_write(PHYCNT_OFF, 0x00000260, 0, 0xffffffff);

	/* Set register initial value */
	value = 0x1511144;
	/* Set DDRTMG to PHYOFFSET1 */
	value |= (3 << SPI_PHYOFFSET1_DDRTMG_SHIFT);
	sys_write32(value, RZA2_SPI_REG_BASE + PHYOFFSET1_OFF);

	value = 0x31;
	value |= (4 << SPI_PHYOFFSET2_OCTTMG_SHIFT);
	sys_write32(value, RZA2_SPI_REG_BASE + PHYOFFSET2_OFF);

	value = 0x70000;
	value |= SPI_PHYINT_WPVAL | SPI_PHYINT_INTEN | SPI_PHYINT_WPEN | SPI_PHYINT_RSTEN;
	sys_write32(value, RZA2_SPI_REG_BASE + PHYINT_OFF);

	sys_write32(0, RZA2_SPI_REG_BASE + PHYADJ1_OFF);
	sys_write32(0, RZA2_SPI_REG_BASE + PHYADJ2_OFF);
}

static int32_t write_enable(void)
{
	int32_t ret;
	struct spibsc_reg spimd_reg;

	clear_spi_reg(&spimd_reg);

	spimd_reg.cde = SPI_OUTPUT_ENABLE;
	spimd_reg.cdb = SPI_1BIT;
	spimd_reg.cmd = SFLASHCMD_WRITE_ENABLE;

	ret = spi_xfer(&spimd_reg);

	return ret;
}

static void flash_rza2_manual_mode(void)
{
	uint32_t cmncr;

	/* Select SDR mode */
	spi_reg_write(PHYCNT_OFF, 0, SPI_PHYCNT_PHYMEM_SHIFT, SPI_PHYCNT_PHYMEM);
	/* Set phyoffset SDR mode*/
	spi_reg_write(PHYOFFSET1_OFF, 3, SPI_PHYOFFSET1_DDRTMG_SHIFT, SPI_PHYOFFSET1_DDRTMG);

	spi_reg_write(SMDMCR_OFF, 0, SPI_SMDMCR_DMCYC_SHIFT, SPI_SMDMCR_DMCYC);

	/* Set CMNCR */
	cmncr = sys_read32(RZA2_SPI_REG_BASE + CMNCR_OFF);
	cmncr = (cmncr & (~SPI_CMNCR_MD)) | (SPI_CMNCR_MD_SPI << SPI_CMNCR_MD_SHIFT);
	/*MOIIO3*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO3)) | (1 << SPI_CMNCR_MOIIO3_SHIFT);
	/*MOIIO2*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO2)) | (1 << SPI_CMNCR_MOIIO2_SHIFT);
	/*MOIIO1*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO1)) | (1 << SPI_CMNCR_MOIIO1_SHIFT);
	/*MOIIO0*/
	cmncr = (cmncr & (~SPI_CMNCR_MOIIO0)) | (1 << SPI_CMNCR_MOIIO0_SHIFT);
	/* SET BSZ to single*/
	cmncr = (cmncr & (~SPI_CMNCR_BSZ)) | (SPI_CMNCR_BSZ_SINGLE << SPI_CMNCR_BSZ_SHIFT);

	/* Dummy read */
	sys_read32(RZA2_SPI_REG_BASE + CMNCR_OFF);

	/* SPI Mode */
	sys_write32(cmncr, RZA2_SPI_REG_BASE + CMNCR_OFF); /* Write to register */

	sys_write32(0, RZA2_SPI_REG_BASE + SSLDR_OFF);

	sys_read32(RZA2_SPI_REG_BASE + CMNCR_OFF);
}

static void spi_mode(void)
{
	if (spi_reg_read(CMNCR_OFF, SPI_CMNCR_MD_SHIFT, SPI_CMNCR_MD) == SPI_CMNCR_MD_SPI) {
		return;
	}

	spi_stop();

	/* Dummy read */
	sys_read32(RZA2_SPI_REG_BASE + CMNCR_OFF);

	/* Set Manual mode */
	flash_rza2_manual_mode();
}

static int32_t data_send(uint32_t bit_width, uint32_t spbssl_level, const uint8_t *buf,
			 int32_t size)
{
	int32_t ret = 0;
	int32_t unit;
	uint8_t *buf_b;
	uint16_t *buf_s;
	uint32_t *buf_l;
	struct spibsc_reg spimd_reg;

	clear_spi_reg(&spimd_reg);

	spimd_reg.sslkp = SPI_SPISSL_KEEP;    /* SPBSSL level */
	spimd_reg.spiwe = SPI_SPIDATA_ENABLE; /* write enable/disable */

	spimd_reg.spidb = bit_width;
	spimd_reg.spidre = SPI_SDR_TRANS; /* SDR */

	if (((uint32_t)size & 0x3) == 0) {
		spimd_reg.spide = SPI_OUTPUT_SPID_32; /* Enable(32bit) */
		unit = 4;
	} else if (((uint32_t)size & 0x1) == 0) {
		spimd_reg.spide = SPI_OUTPUT_SPID_16; /* Enable(16bit) */
		unit = 2;
	} else {
		spimd_reg.spide = SPI_OUTPUT_SPID_8; /* Enable(8bit) */
		unit = 1;
	}

	while (size > 0) {
		if (unit == 1) {
			buf_b = (uint8_t *)buf;
			spimd_reg.smwdr[0] = *buf_b;
		} else if (unit == 2) {
			buf_s = (uint16_t *)buf;
			spimd_reg.smwdr[0] = *buf_s;
		} else if (unit == 4) {
			buf_l = (uint32_t *)buf;
			spimd_reg.smwdr[0] = *buf_l;
		}

		buf += unit;
		size -= unit;

		if (size <= 0) {
			spimd_reg.sslkp = spbssl_level;
		}

		ret = spi_xfer(&spimd_reg); /* Data */
		if (ret != 0) {
			return ret;
		}
	}

	return ret;
}

int32_t _page_program(uint32_t offset, const uint8_t *buf, uint32_t size)
{
	int32_t ret;
	int32_t program_size;
	int32_t remainder;
	int32_t idx = 0;
	struct spibsc_reg spimd_reg;

	while (size > 0) {
		if (size > FLASH_PAGE_SIZE) {
			program_size = FLASH_PAGE_SIZE;
		} else {
			program_size = size;
		}
		remainder = FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE);
		if ((remainder != 0) && (program_size > remainder)) {
			program_size = remainder;
		}

		memcpy(write_tmp_buf, &buf[idx], program_size);

		cache_data_flush_range((void *)write_tmp_buf, program_size);

		ret = write_enable(); /* WREN Command */
		if (ret != 0) {
			return ret;
		}

		clear_spi_reg(&spimd_reg);

		spimd_reg.cde = SPI_OUTPUT_ENABLE;
		spimd_reg.cdb = SPI_1BIT;
		spimd_reg.cmd = SFLASHCMD_PAGE_PROGRAM;

		spimd_reg.ade = SPI_OUTPUT_ADDR;
		spimd_reg.addre = SPI_SDR_TRANS; /* SDR */
		spimd_reg.adb = SPI_1BIT;
		spimd_reg.addr = offset;

		spimd_reg.sslkp = SPI_SPISSL_KEEP; /* SPBSSL level */

		ret = spi_xfer(&spimd_reg); /* Command,Address */
		if (ret != 0) {
			return ret;
		}

		ret = data_send(SPI_1BIT, SPI_SPISSL_NEGATE, write_tmp_buf, program_size);
		if (ret != 0) {
			return ret;
		}

		ret = wait_status_from_flash();
		if (ret != 0) {
			return ret;
		}

		offset += program_size;
		idx += program_size;
		size -= program_size;
	}

	return ret;
}

static bool is_valid_range(off_t offset, uint32_t size)
{
	return (offset >= 0) && ((offset + size) <= FLASH_SIZE);
}

static int flash_rza2_read(const struct device *dev, off_t offset, void *data, size_t size)
{
	uint8_t *virt;
	size_t aligned_size = (size < FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : size;
	const struct flash_rza2_config *config = dev->config;

	if (config->type != SERIAL_FLASH) {
		LOG_ERR("%s: Only Serial Flash is supported", __func__);
		return -ENOTSUP;
	}

	if (size == 0) {
		return 0;
	}

	if (!is_valid_range(offset, size)) {
		LOG_ERR("Range exceeds the flash boundaries. Offset=%#lx, Size=%u", offset, size);
		return -EINVAL;
	}

	if (data == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);

	ex_mode();

	z_phys_map(&virt, FLASH_BASE + offset, aligned_size, K_MEM_CACHE_NONE);

	memcpy(data, virt, size);

	z_phys_unmap(virt, aligned_size);
	spi_mode();

	k_mutex_unlock(&lock);

	return 0;
}

static int flash_rza2_write(const struct device *dev, off_t offset, const void *data, size_t size)
{
	int ret;
	uint32_t key;
	const struct flash_rza2_config *config = dev->config;

	if (config->type != SERIAL_FLASH) {
		LOG_ERR("%s: Only Serial Flash is supported", __func__);
		return -ENOTSUP;
	}

	if (size == 0) {
		LOG_DBG("%s: Skip writing 0 length buffer", __func__);
		return 0;
	}

	if (!data) {
		return -EINVAL;
	}

	if (!is_valid_range(offset, size)) {
		LOG_ERR("Range exceeds the flash boundaries. Offset=%#lx, Size=%u", offset, size);
		return -EINVAL;
	}

	key = irq_lock();

	ret = _page_program(offset, data, size);

	irq_unlock(key);

	return ret;
}

static int sector_erase_serial(uint32_t offset)
{
	int32_t ret;
	struct spibsc_reg spimd_reg;

	ret = write_enable(); /* WREN Command */
	if (ret != 0) {
		return ret;
	}

	clear_spi_reg(&spimd_reg);

	spimd_reg.cde = SPI_OUTPUT_ENABLE;
	spimd_reg.cdb = SPI_1BIT;
	spimd_reg.cmd = SFLASHCMD_SECTOR_ERASE;

	spimd_reg.ade = SPI_OUTPUT_ADDR;
	spimd_reg.addre = SPI_SDR_TRANS; /* SDR */
	spimd_reg.adb = SPI_1BIT;
	spimd_reg.addr = offset;

	ret = spi_xfer(&spimd_reg);
	if (ret != 0) {
		return ret;
	}

	return wait_status_from_flash();
}

static int range_erase_serial(const struct device *dev, off_t offset, size_t size)
{
	int ret;
	uint32_t offt = offset;

	/* This function expects valid offset and size values */
	do {
		ret = sector_erase_serial(offt);
		if (ret) {
			LOG_ERR("%s: Unable to clear sector on addr: %x", __func__, offt);
			return ret;
		}

		offt += FLASH_SECTOR_SIZE;
	} while (offt < offset + size);

	return 0;
}

static int flash_rza2_erase(const struct device *dev, off_t offset, size_t size)
{
	int ret;
	uint32_t key;
	const struct flash_rza2_config *config = dev->config;

	if (config->type != SERIAL_FLASH) {
		LOG_ERR("%s: Only Serial Flash is supported", __func__);
		return -ENOTSUP;
	}

	if (size % FLASH_SECTOR_SIZE) {
		LOG_ERR("%s: erase size isn't aligned to the sector size", __func__);
		return -EINVAL;
	}

	if (offset % FLASH_SECTOR_SIZE) {
		LOG_ERR("%s: offset isn't aligned to the sector size", __func__);
		return -EINVAL;
	}

	if (size == 0) {
		LOG_DBG("%s: Skip writing 0 length buffer", __func__);
		return 0;
	}

	if (!is_valid_range(offset, size)) {
		LOG_ERR("Erase range exceeds the flash boundaries. Offset=%#lx, Size=%u", offset,
			size);
		return -EINVAL;
	}

	key = irq_lock();

	ret = range_erase_serial(dev, offset, size);

	irq_unlock(key);

	return ret;
}

static const struct flash_parameters *flash_rza2_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_rza2_parameters;
}

#if CONFIG_FLASH_PAGE_LAYOUT

static const struct flash_pages_layout flash_rza2_pages_layout = {
	.pages_count = FLASH_SIZE / FLASH_PAGE_SIZE,
	.pages_size = FLASH_PAGE_SIZE
};

void flash_rza2_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
			    size_t *layout_size)
{
	*layout = &flash_rza2_pages_layout;
	*layout_size = 1;
}

#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flash_rza2_driver_api = {
	.read = flash_rza2_read,
	.write = flash_rza2_write,
	.erase = flash_rza2_erase,
	.get_parameters = flash_rza2_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_rza2_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
};

/*
 * Currently the following configuration is supported:
 *   - one connected Serial Flash to the controller;
 *   - access to this Serial Flash is performed in SDR mode;
 *   - Read command is performed from External Address Space Mode;
 *   - Write and Erace commands are performed in Manual Mode;
 *   - for Serial Flash only Master mode is supported according to the HW manual;
 * TODO:
 *   - serial flash configurations, such as 2 Serial Flash, Octa-Flash,
 *	HyperFlash;
 *   - DMA support;
 *   - DDR access support;
 *   - perform Read and Write/ERASE operation from Manual Mode to avoid extra
 *     map/unmap operations and mode switching.
 */
static int flash_rza2_init(const struct device *dev)
{
	const struct flash_rza2_config *cfg = dev->config;

	DEVICE_MMIO_TOPLEVEL_MAP(spi_reg, K_MEM_CACHE_NONE);

#ifdef CONFIG_PINCTRL
	if (cfg->pcfg != NULL && pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
		LOG_ERR("%s: unable to apply pinctrl configuration", __func__);
		return -EINVAL;
	}
#endif

	spi_mode();

	return 0;
}

#ifdef CONFIG_PINCTRL
#define RZ_PINCTRL_DT_DEFINE(n)                                                                    \
	COND_CODE_1(DT_NUM_PINCTRL_STATES(n), (PINCTRL_DT_DEFINE(n);), (EMPTY))

#define RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n)                                                       \
	COND_CODE_1(DT_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_DEV_CONFIG_GET(n)), NULL)
#else
#define RZ_PINCTRL_DT_DEFINE(n)
#define RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(n) NULL
#endif

RZ_PINCTRL_DT_DEFINE(DT_DRV_INST(0));

const struct flash_rza2_config flash_rza2_cfg = {
	.type = SERIAL_FLASH,
	.pcfg = RZ_PINCTRL_DT_INST_DEV_CONFIG_GET(DT_DRV_INST(0))
};

DEVICE_DT_INST_DEFINE(0, flash_rza2_init, NULL, NULL, &flash_rza2_cfg, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_rza2_driver_api);
