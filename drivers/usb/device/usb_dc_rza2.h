/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USB_DC_RZA2_H
#define USB_DC_RZA2_H

#define H_HCRHDESCRIPTORA    (0x048)

#define H_USBCTR             (0x20C)

#define H_COMMCTRL	(0x800) /* OTG/BC Module Control Register */
#define H_PHYCLK_CTRL	(0x844) /* USBPHY Supply Clock Control Register */
#define H_PHYIF_CTRL	(0x848) /* PHYIF CTRL Register */

/* Function registers */
#define D_SYSCFG0   (0x000) /* System Configuration Control Register 0 16-bit */
#define D_SYSCFG1   (0x002) /* System Configuration Control Register 1 16-bit */
#define D_SYSSTS0   (0x004) /* System Configuration Status Register 16-bit */
#define D_DVSTCTR0  (0x008) /* Device Control Register 0 16-bit */
#define D_TESTMODE  (0x00C) /* Test Mode Register 16-bit */
#define D_CFIFO     (0x014) /* CFIFO Port Register 8-/16-/32-bit */
#define D_CFIFOSEL  (0x020) /* CFIFO Port Select Register 16-bit */
#define D_CFIFOCTR  (0x022) /* CFIFO Port Control Register 16-bit */
#define D_D0FIFOSEL (0x028) /* D0FIFO Port Select Register 16-bit */
#define D_D0FIFOCTR (0x02A) /* D0FIFO Port Control Register 16-bit */
#define D_D1FIFOSEL (0x02C) /* D1FIFO Port Select Register 16-bit */
#define D_D1FIFOCTR (0x02E) /* D1FIFO Port Control Register 16-bit */
#define D_INTENB0   (0x030) /* Interrupt Enable Register 0 16-bit */
#define D_BRDYENB   (0x036) /* BRDY Interrupt Enable Register 16-bit */
#define D_NRDYENB   (0x038) /* NRDY Interrupt Enable Register 16-bit */
#define D_BEMPENB   (0x03A) /* BEMP Interrupt Enable Register 16-bit */
#define D_SOFCFG    (0x03C) /* SOF Output Configuration Register 16-bit */
#define D_INTSTS0   (0x040) /* Interrupt Status Register 0 16-bit */
#define D_BRDYSTS   (0x046) /* BRDY Interrupt Status Register 16-bit */
#define D_NRDYSTS   (0x048) /* NRDY Interrupt Status Register 16-bit */
#define D_BEMPSTS   (0x04A) /* BEMP Interrupt Status Register 16-bit */
#define D_FRMNUM    (0x04C) /* Frame Number Register 16-bit */
#define D_UFRMNUM   (0x04E) /* Micro Frame Number Register 16-bit */
#define D_USBADDR   (0x050) /* USB Address Register 16-bit */
#define D_USBREQ    (0x054) /* USB Request Type Register 16-bit */
#define D_USBVAL    (0x056) /* USB Request Value Register 16-bit */
#define D_USBINDX   (0x058) /* USB Request Index Register 16-bit */
#define D_USBLENG   (0x05A) /* USB Request Length Register 16-bit */
#define D_DCPCFG    (0x05C) /* DCP Configuration Register 16-bit */
#define D_DCPMAXP   (0x05E) /* DCP Max. Packet Size Register 16-bit */
#define D_DCPCTR    (0x060) /* DCP Control Register 16-bit */
#define D_PIPESEL   (0x064) /* Pipe Window Select Register 16-bit */
#define D_PIPECFG   (0x068) /* Pipe Configuration Register 16-bit */
#define D_PIPEBUF   (0x06A) /* Pipe Buffer Setting Register 16-bit */
#define D_PIPEMAXP  (0x06C) /* Pipe Max. Packet Size Register 16-bit */
#define D_PIPEPERI  (0x06E) /* Pipe Cycle Control Register 16-bit */

#define D_PIPE1CTR  (0x070) /* PIPE1 Control Register 16-bit */
#define D_PIPENCTR(n) (D_PIPE1CTR + (n)) /* PIPEn Control Register 16-bit */
#define D_PIPENCTR1(n) (D_PIPE1CTR + (((n) - 1) * sizeof(uint16_t))) /* PIPEn Control Register */

#define D_PIPE1TRE  (0x090) /* PIPE1 Transaction Counter Enable Register 16-bit */
#define D_PIPE1TRN  (0x092) /* PIPE1 Transaction Counter Register 16-bit */
#define D_PIPE2TRE  (0x094) /* PIPE2 Transaction Counter Enable Register 16-bit */
#define D_PIPE2TRN  (0x096) /* PIPE2 Transaction Counter Register 16-bit */
#define D_PIPE3TRE  (0x098) /* PIPE3 Transaction Counter Enable Register 16-bit */
#define D_PIPE3TRN  (0x09A) /* PIPE3 Transaction Counter Register 16-bit */
#define D_PIPE4TRE  (0x09C) /* PIPE4 Transaction Counter Enable Register 16-bit */
#define D_PIPE4TRN  (0x09E) /* PIPE4 Transaction Counter Register 16-bit */
#define D_PIPE5TRE  (0x0A0) /* PIPE5 Transaction Counter Enable Register 16-bit */
#define D_PIPE5TRN  (0x0A2) /* PIPE5 Transaction Counter Register 16-bit */
#define D_PIPEBTRE  (0x0A4) /* PIPEB Transaction Counter Enable Register 16-bit */
#define D_PIPEBTRN  (0x0A6) /* PIPEB Transaction Counter Register 16-bit */
#define D_PIPECTRE  (0x0A8) /* PIPEC Transaction Counter Enable Register 16-bit */
#define D_PIPECTRN  (0x0AA) /* PIPEC Transaction Counter Register 16-bit */
#define D_PIPEDTRE  (0x0AC) /* PIPED Transaction Counter Enable Register 16-bit */
#define D_PIPEDTRN  (0x0AE) /* PIPED Transaction Counter Register 16-bit */
#define D_PIPEETRE  (0x0B0) /* PIPEE Transaction Counter Enable Register 16-bit */
#define D_PIPEETRN  (0x0B2) /* PIPEE Transaction Counter Register 16-bit */
#define D_PIPEFTRE  (0x0B4) /* PIPEF Transaction Counter Enable Register 16-bit */
#define D_PIPEFTRN  (0x0B6) /* PIPEF Transaction Counter Register 16-bit */
#define D_PIPE9TRE  (0x0B8) /* PIPE9 Transaction Counter Enable Register 16-bit */
#define D_PIPE9TRN  (0x0BA) /* PIPE9 Transaction Counter Register 16-bit */
#define D_PIPEATRE  (0x0BC) /* PIPEA Transaction Counter Enable Register 16-bit */
#define D_PIPEATRN  (0x0BE) /* PIPEA Transaction Counter Register 16-bit */
#define D_DEVADD0   (0x0D0) /* Device Address 0 Configuration Register 16-bit */
#define D_DEVADD1   (0x0D2) /* Device Address 1 Configuration Register 16-bit */
#define D_DEVADD2   (0x0D4) /* Device Address 2 Configuration Register 16-bit */
#define D_DEVADD3   (0x0D6) /* Device Address 3 Configuration Register 16-bit */
#define D_DEVADD4   (0x0D8) /* Device Address 4 Configuration Register 16-bit */
#define D_DEVADD5   (0x0DA) /* Device Address 5 Configuration Register 16-bit */
#define D_DEVADD6   (0x0DC) /* Device Address 6 Configuration Register 16-bit */
#define D_DEVADD7   (0x0DE) /* Device Address 7 Configuration Register 16-bit */
#define D_DEVADD8   (0x0E0) /* Device Address 8 Configuration Register 16-bit */
#define D_DEVADD9   (0x0E2) /* Device Address 9 Configuration Register 16-bit */
#define D_DEVADDA   (0x0E4) /* Device Address A Configuration Register 16-bit */
#define D_LPCTRL    (0x100) /* Low Power Control Register 16-bit */
#define D_LPSTS     (0x102) /* Low Power Status Register 16-bit */
#define D_PHYFUNCTR (0x104) /* PHY Function Control Register 16-bit */
#define D_PHYOTGCTR (0x10A) /* PHY OTG Control Register 16-bit */
#define D_PL1CTRL1  (0x144) /* Peripheral L1 Control Register 1 16-bit */
#define D_PL1CTRL2  (0x146) /* Peripheral L1 Control Register 2 16-bit */
#define D_N0SA_0    (0x400) /* Next0 Source Address Register 0 32-bit */
#define D_N0DA_0    (0x404) /* Next0 Destination Address Register 0 32-bit */
#define D_N0TB_0    (0x408) /* Next0 Transaction Byte Register 0 32-bit */
#define D_N1SA_0    (0x40C) /* Next1 Source Address Register 0 32-bit */
#define D_N1DA_0    (0x410) /* Next1 Destination Address Register 0 32-bit */
#define D_N1TB_0    (0x414) /* Next1 Transaction Byte Register 0 32-bit */
#define D_CRSA_0    (0x418) /* Current Source Address Register 0 32-bit */
#define D_CRDA_0    (0x41C) /* Current Destination Address Register 0 32-bit */
#define D_CRTB_0    (0x420) /* Current Transaction Byte Register 0 32-bit */
#define D_CHSTAT_0  (0x424) /* Channel Status Register 0 32-bit */
#define D_CHCTRL_0  (0x428) /* Channel Control Register 0 32-bit */
#define D_CHCFG_0   (0x42C) /* Channel Configuration Register 0 32-bit */
#define D_CHITVL_0  (0x430) /* Channel Interval Register 0 32-bit */
#define D_CHEXT_0   (0x434) /* Channel Extension Register 0 32-bit */
#define D_NXLA_0    (0x438) /* Next Link Address Register 0 32-bit */
#define D_CRLA_0    (0x43C) /* Current Link Address Register 0 32-bit */
#define D_N0SA_1    (0x440) /* Next0 Source Address Register 1 32-bit */
#define D_N0DA_1    (0x444) /* Next0 Destination Address Register 1 32-bit */
#define D_N0TB_1    (0x448) /* Next0 Transaction Byte Register 1 32-bit */
#define D_N1SA_1    (0x44C) /* Next1 Source Address Register 1 32-bit */
#define D_N1DA_1    (0x450) /* Next1 Destination Address Register 1 32-bit */
#define D_N1TB_1    (0x454) /* Next1 Transaction Byte Register 1 32-bit */
#define D_CRSA_1    (0x458) /* Current Source Address Register 1 32-bit */
#define D_CRDA_1    (0x45C) /* Current Destination Address Register 1 32-bit */
#define D_CRTB_1    (0x460) /* Current Transaction Byte Register 1 32-bit */
#define D_CHSTAT_1  (0x464) /* Channel Status Register 1 32-bit */
#define D_CHCTRL_1  (0x468) /* Channel Control Register 1 32-bit */
#define D_CHCFG_1   (0x46C) /* Channel Configuration Register 1 32-bit */
#define D_CHITVL_1  (0x470) /* Channel Interval Register 1 32-bit */
#define D_CHEXT_1   (0x474) /* Channel Extension Register 1 32-bit */
#define D_NXLA_1    (0x478) /* Next Link Address Register 1 32-bit */
#define D_CRLA_1    (0x47C) /* Current Link Address Register 1 32-bit */
#define D_SCNT_0    (0x600) /* Source Continuous Register 0 32-bit */
#define D_SSKP_0    (0x604) /* Source Skip Register 0 32-bit */
#define D_DCNT_0    (0x608) /* Destination Continuous Register 0 32-bit */
#define D_DSKP_0    (0x60C) /* Destination Skip Register 0 32-bit */
#define D_SCNT_1    (0x620) /* Source Continuous Register 1 32-bit */
#define D_SSKP_1    (0x624) /* Source Skip Register 1 32-bit */
#define D_DCNT_1    (0x628) /* Destination Continuous Register 1 32-bit */
#define D_DSKP_1    (0x62C) /* Destination Skip Register 1 32-bit */
#define D_DCTRL     (0x700) /* DMA Control Register 32-bit */
#define D_DSCITVL   (0x704) /* Descriptor Interval 32-bit */
#define D_DST_EN    (0x710) /* DMA Status EN Register 32-bit */
#define D_DST_ER    (0x714) /* DMA Status ER Register 32-bit */
#define D_DST_END   (0x718) /* DMA Status END Register 32-bit */
#define D_DST_TC    (0x71C) /* DMA Status TC Register 32-bit */
#define D_DST_SUS   (0x720) /* DMA Status SUS Register 32-bit */

/* H_HCRHDESCRIPTORA */
#define HCRHDESCRIPTORA_NDP                                (0x000000FFu)
#define HCRHDESCRIPTORA_NDP_SHIFT                          (0u)
#define HCRHDESCRIPTORA_PSM                                BIT(8)
#define HCRHDESCRIPTORA_NPS                                BIT(9)
#define HCRHDESCRIPTORA_DT                                 BIT(10)
#define HCRHDESCRIPTORA_OCPM                               BIT(11)
#define HCRHDESCRIPTORA_NOCP                               BIT(12)
#define HCRHDESCRIPTORA_POTPGT                             (0xFF000000u)
#define HCRHDESCRIPTORA_POTPGT_SHIFT                       (24u)

/* H_USBCTR */
#define USBH_RST       BIT(0)
#define PLL_RST        BIT(1)
#define DIRPD          BIT(2)

/* H_COMMCTRL */
#define OTG_PERI	BIT(31)

/* H_PHYCLK_CTRL */
#define PHYCLK_CTRL_UCLKSEL                       BIT(0)
/* H_PHYIF_CTRL */
#define PHYIF_CTRL_FIXPHY                         BIT(0)

/* SYSCFG0 */
#define USBE        BIT(0)
#define DPRPU       BIT(4)
#define DRPD        BIT(5)
#define HSE         BIT(7)
#define CNEN        BIT(8)

/* SYSCFG1 */
#define BWAIT       (0x003Fu)
#define BWAIT_SHIFT (0u)

/* SYSSTS0 */
#define LNST       (0x0003u)
#define LNST_SHIFT (0u)

/* DVSTCTR0 */
#define RHST        (0x0007u)
#define RHST_SHIFT  (0u)
#define WKUP        BIT(8)
#define BRST1       BIT(15)

/* CFIFO */
#define FIFOPORT       (0xFFFFFFFFu)
#define FIFOPORT_SHIFT (0u)

/* CFIFOSEL */
#define CFIFOSEL_CURPIPE	(0x000Fu)
#define CFIFOSEL_CURPIPE_SHIFT	(0u)
#define CFIFOSEL_ISEL		BIT(5)
#define CFIFOSEL_BIGEND		BIT(8)
#define CFIFOSEL_MBW		(0x0C00u)
#define CFIFOSEL_MBW_SHIFT	(10u)
#define CFIFOSEL_USB_MBW_8	(0x0u)    /* FIFO access : 8bit */
#define CFIFOSEL_USB_MBW_16	(0x0400u) /* FIFO access : 16bit */
#define CFIFOSEL_USB_MBW_32	(0x0800u) /* FIFO access : 32bit */
#define CFIFOSEL_REW		BIT(14)
#define CFIFOSEL_RCNT		BIT(15)

/* CFIFOCTR */
#define CFIFOCTR_DTLN          (0x0FFFu)
#define CFIFOCTR_FRDY          BIT(13)
#define CFIFOCTR_BCLR          BIT(14)
#define CFIFOCTR_BVAL          BIT(15)

#define DnFIFOSEL_CURPIPE       (0x000Fu)
#define DnFIFOSEL_CURPIPE_SHIFT (0u)
#define DnFIFOSEL_MBW           (0x0C00u)
#define DnFIFOSEL_MBW_SHIFT     (10u)
#define DnFIFOSEL_DREQE         (0x1000u)
#define DnFIFOSEL_DREQE_SHIFT   (12u)
#define DnFIFOSEL_DCLRM         (0x2000u)
#define DnFIFOSEL_DCLRM_SHIFT   (13u)
#define DnFIFOSEL_REW           (0x4000u)
#define DnFIFOSEL_REW_SHIFT     (14u)
#define DnFIFOSEL_RCNT          (0x8000u)
#define DnFIFOSEL_RCNT_SHIFT    (15u)
#define DnFIFOCTR_DTLN          (0x0FFFu)
#define DnFIFOCTR_DTLN_SHIFT    (0u)
#define DnFIFOCTR_FRDY          (0x2000u)
#define DnFIFOCTR_FRDY_SHIFT    (13u)
#define DnFIFOCTR_BCLR          (0x4000u)
#define DnFIFOCTR_BCLR_SHIFT    (14u)
#define DnFIFOCTR_BVAL          (0x8000u)
#define DnFIFOCTR_BVAL_SHIFT    (15u)

/* INTENB0 */
#define BRDYE		BIT(8)
#define NRDYE		BIT(9)
#define BEMPE		BIT(10)
#define CTRE		BIT(11)
#define DVSE		BIT(12)
#define SOFE		BIT(13)
#define RSME		BIT(14)
#define VBSE		BIT(15)

/* SOFCFG */
#define SOFM             GENMASK(3, 2)
#define BRDYM            BIT(6)

/* INTSTS0 */
#define CTSQ		GENMASK(2, 0)
#define CS_SQER		(0x0006u) /* Sequence error */
#define CS_WRND		(0x0005u) /* Ctrl write nodata status stage */
#define CS_WRSS		(0x0004u) /* Ctrl write status stage */
#define CS_WRDS		(0x0003u) /* Ctrl write data stage */
#define CS_RDSS		(0x0002u) /* Ctrl read status stage */
#define CS_RDDS		(0x0001u) /* Ctrl read data stage */
#define CS_IDST		(0x0000u) /* Idle or setup stage */

#define VALID		BIT(3)

#define DVSQ		GENMASK(6, 4)
#define DVSQ_SPD_CNFG	(0x0070u) /* Suspend Configured */
#define DVSQ_SPD_ADDR	(0x0060u) /* Suspend Address */
#define DVSQ_SPD_DFLT	(0x0050u) /* Suspend Default */
#define DVSQ_SPD_POWR	(0x0040u) /* Suspend Powered */
#define DVSQ_SUSP	(0x0040u) /* Suspend mask */
#define DVSQ_CNFG	(0x0030u) /* Configured */
#define DVSQ_ADDS	(0x0020u) /* Address */
#define DVSQ_DFLT	(0x0010u) /* Default */
#define DVSQ_POWR	(0x0000u) /* Powered */

#define VBSTS		BIT(7)
#define BRDY		BIT(8)
#define NRDY		BIT(9)
#define BEMP		BIT(10)
#define CTRT		BIT(11)
#define DVST		BIT(12)
#define SOFR		BIT(13)
#define RESM		BIT(14)
#define VBINT		BIT(15)

/* FRMNUM/UFRMNUM */
#define FRNM           (0x07FFu)
#define CRCE           BIT(14)
#define OVRN           BIT(15)
#define UFRNM          (0x0007u)
#define DVCHG          BIT(15)

/* USBADDR */
#define USBADDR         (0x007Fu)
#define USBADDR_SHIFT   (0u)
#define STSRECOV0       (0x0700u)
#define STSRECOV0_SHIFT (8u)

/* USBREQ */
#define BMREQUESTTYPE       (0x00FFu)
#define BMREQUESTTYPE_SHIFT (0u)
#define BREQUEST            (0xFF00u)
#define BREQUEST_SHIFT      (8u)

/* USBVAL */
#define wValue       (0xFFFFu)
#define wValue_SHIFT (0u)

/* USBIDX */
#define WINDEX        (0xFFFFu)
#define WINDEX_SHIFT  (0u)
#define WLENGTH       (0xFFFFu)
#define WLENGTH_SHIFT (0u)

/* DCPCFG */
#define SHTNAK       BIT(7)
#define CNTMD        BIT(8)

/* DCPMAXP */
#define MXPSC      (0x007F)
/* PIPEMAXP */
#define MXPSD       (0x07FF)

/* DCPCTR */
#define PID         (0x0003u)
#define PID_NAK     0
#define PID_BUF     1
#define PID_STALL10 2
#define PID_STALL11 3
#define CCPL        BIT(2)
#define PBUSY       BIT(5)
#define SQMON       BIT(6)
#define SQSET       BIT(7)
#define SQCLR       BIT(8)
#define BSTS        BIT(15)

/* PIPESEL */
#define PIPESEL       (0x000Fu)

/* PIPECFG */
#define EPNUM        (0x000Fu)
#define DIR_OUT      BIT(4)
#define SHTNAK       BIT(7)
#define CNTMD        BIT(8)
#define DBLB         BIT(9)
#define BFRE         BIT(10)
#define TYPE         (0xC000u)
#define TYPE_SHIFT   (14u)
#define TYPE_NONE (0 << 14) /* Transfer Type */
#define TYPE_BULK (1 << 14)
#define TYPE_INT  (2 << 14)
#define TYPE_ISO  (3 << 14)

/* PIPEBUF */
#define BUFNMB        (0x00FFu)
#define BUFSIZE       (0x7C00u)
#define BUFSIZE_SHIFT (10u)

/* PIPEPERI */
#define IITV         (0x0007u)
#define IITV_SHIFT   (0u)
#define IFIS         BIT(12)

/* PIPEnCTR */
#define ACLRM        BIT(9)
#define ATREPM       BIT(10)
#define INBUFM       BIT(14)

/* PIPEnTRE/PIPEnTRN/LPCTRL */
#define TRCLR        BIT(8)
#define TRENB        BIT(9)

/* LPCTRL */
#define HWUPM        BIT(7)

/* LPSTS */
#define SUSPM       BIT(14)

/* PHYFUNCTR */
#define SUSMON        BIT(14)

/* PHY_OTG */
#define DPPUDWN       BIT(9)
#define DMPUDWN       BIT(10)

#define PL1CTRL1_L1RESPEN       BIT(0)
#define PL1CTRL1_L1RESPMD       (0x0006u)
#define PL1CTRL1_L1RESPMD_SHIFT (1u)
#define PL1CTRL1_L1NEGOMD       BIT(3)
#define PL1CTRL1_DVSQ           (0x00F0u)
#define PL1CTRL1_DVSQ_SHIFT     (4u)
#define PL1CTRL1_HIRDTHR        (0x0F00u)
#define PL1CTRL1_HIRDTHR_SHIFT  (8u)
#define PL1CTRL1_L1EXTMD        BIT(14)
#define PL1CTRL2_HIRDMON        (0x0F00u)
#define PL1CTRL2_HIRDMON_SHIFT  (8u)
#define PL1CTRL2_RWEMON         BIT(12)

/* DMA */
#define N0SA_0_SA_WD             (0xFFFFFFFFu)
#define N0SA_0_SA_WD_SHIFT       (0u)
#define N0DA_0_DA                (0xFFFFFFFFu)
#define N0DA_0_DA_SHIFT          (0u)
#define N0TB_0_TB                (0xFFFFFFFFu)
#define N0TB_0_TB_SHIFT          (0u)
#define N1SA_0_SA_WD             (0xFFFFFFFFu)
#define N1SA_0_SA_WD_SHIFT       (0u)
#define N1DA_0_DA                (0xFFFFFFFFu)
#define N1DA_0_DA_SHIFT          (0u)
#define N1TB_0_TB                (0xFFFFFFFFu)
#define N1TB_0_TB_SHIFT          (0u)
#define CRSA_0_CRSA              (0xFFFFFFFFu)
#define CRSA_0_CRSA_SHIFT        (0u)
#define CRDA_0_CRDA              (0xFFFFFFFFu)
#define CRDA_0_CRDA_SHIFT        (0u)
#define CRTB_0_CRTB              (0xFFFFFFFFu)
#define CRTB_0_CRTB_SHIFT        (0u)
#define CHSTAT_0_EN              (0x00000001u)
#define CHSTAT_0_EN_SHIFT        (0u)
#define CHSTAT_0_RQST            (0x00000002u)
#define CHSTAT_0_RQST_SHIFT      (1u)
#define CHSTAT_0_TACT            (0x00000004u)
#define CHSTAT_0_TACT_SHIFT      (2u)
#define CHSTAT_0_SUS             (0x00000008u)
#define CHSTAT_0_SUS_SHIFT       (3u)
#define CHSTAT_0_ER              (0x00000010u)
#define CHSTAT_0_ER_SHIFT        (4u)
#define CHSTAT_0_END             (0x00000020u)
#define CHSTAT_0_END_SHIFT       (5u)
#define CHSTAT_0_TC              (0x00000040u)
#define CHSTAT_0_TC_SHIFT        (6u)
#define CHSTAT_0_SR              (0x00000080u)
#define CHSTAT_0_SR_SHIFT        (7u)
#define CHSTAT_0_DL              (0x00000100u)
#define CHSTAT_0_DL_SHIFT        (8u)
#define CHSTAT_0_DW              (0x00000200u)
#define CHSTAT_0_DW_SHIFT        (9u)
#define CHSTAT_0_DER             (0x00000400u)
#define CHSTAT_0_DER_SHIFT       (10u)
#define CHSTAT_0_MODE            (0x00000800u)
#define CHSTAT_0_MODE_SHIFT      (11u)
#define CHSTAT_0_INTM            (0x00010000u)
#define CHSTAT_0_INTM_SHIFT      (16u)
#define CHSTAT_0_DMARQM          (0x00020000u)
#define CHSTAT_0_DMARQM_SHIFT    (17u)
#define CHSTAT_0_SWPRQ           (0x00040000u)
#define CHSTAT_0_SWPRQ_SHIFT     (18u)
#define CHSTAT_0_DNUM            (0xFF000000u)
#define CHSTAT_0_DNUM_SHIFT      (24u)
#define CHCTRL_0_SETEN           (0x00000001u)
#define CHCTRL_0_SETEN_SHIFT     (0u)
#define CHCTRL_0_CLREN           (0x00000002u)
#define CHCTRL_0_CLREN_SHIFT     (1u)
#define CHCTRL_0_STG             (0x00000004u)
#define CHCTRL_0_STG_SHIFT       (2u)
#define CHCTRL_0_SWRST           (0x00000008u)
#define CHCTRL_0_SWRST_SHIFT     (3u)
#define CHCTRL_0_CLRRQ           (0x00000010u)
#define CHCTRL_0_CLRRQ_SHIFT     (4u)
#define CHCTRL_0_CLREND          (0x00000020u)
#define CHCTRL_0_CLREND_SHIFT    (5u)
#define CHCTRL_0_CLRTC           (0x00000040u)
#define CHCTRL_0_CLRTC_SHIFT     (6u)
#define CHCTRL_0_CLRDER          (0x00000080u)
#define CHCTRL_0_CLRDER_SHIFT    (7u)
#define CHCTRL_0_SETSUS          (0x00000100u)
#define CHCTRL_0_SETSUS_SHIFT    (8u)
#define CHCTRL_0_CLRSUS          (0x00000200u)
#define CHCTRL_0_CLRSUS_SHIFT    (9u)
#define CHCTRL_0_SETREN          (0x00001000u)
#define CHCTRL_0_SETREN_SHIFT    (12u)
#define CHCTRL_0_SETSSWPRQ       (0x00004000u)
#define CHCTRL_0_SETSSWPRQ_SHIFT (14u)
#define CHCTRL_0_SETINTM         (0x00010000u)
#define CHCTRL_0_SETINTM_SHIFT   (16u)
#define CHCTRL_0_CLRINTM         (0x00020000u)
#define CHCTRL_0_CLRINTM_SHIFT   (17u)
#define CHCTRL_0_SETDMARQM       (0x00040000u)
#define CHCTRL_0_SETDMARQM_SHIFT (18u)
#define CHCTRL_0_CLRDMARQM       (0x00080000u)
#define CHCTRL_0_CLRDMARQM_SHIFT (19u)
#define CHCFG_0_SEL              (0x00000001u)
#define CHCFG_0_SEL_SHIFT        (0u)
#define CHCFG_0_REQD             (0x00000008u)
#define CHCFG_0_REQD_SHIFT       (3u)
#define CHCFG_0_DRRP             (0x00000800u)
#define CHCFG_0_DRRP_SHIFT       (11u)
#define CHCFG_0_SDS              (0x0000F000u)
#define CHCFG_0_SDS_SHIFT        (12u)
#define CHCFG_0_DDS              (0x000F0000u)
#define CHCFG_0_DDS_SHIFT        (16u)
#define CHCFG_0_SAD              (0x00100000u)
#define CHCFG_0_SAD_SHIFT        (20u)
#define CHCFG_0_DAD              (0x00200000u)
#define CHCFG_0_DAD_SHIFT        (21u)
#define CHCFG_0_WONLY            (0x00800000u)
#define CHCFG_0_WONLY_SHIFT      (23u)
#define CHCFG_0_DEM              (0x01000000u)
#define CHCFG_0_DEM_SHIFT        (24u)
#define CHCFG_0_TCM              (0x02000000u)
#define CHCFG_0_TCM_SHIFT        (25u)
#define CHCFG_0_DIM              (0x04000000u)
#define CHCFG_0_DIM_SHIFT        (26u)
#define CHCFG_0_SBE              (0x08000000u)
#define CHCFG_0_SBE_SHIFT        (27u)
#define CHCFG_0_RSEL             (0x10000000u)
#define CHCFG_0_RSEL_SHIFT       (28u)
#define CHCFG_0_RSW              (0x20000000u)
#define CHCFG_0_RSW_SHIFT        (29u)
#define CHCFG_0_REN              (0x40000000u)
#define CHCFG_0_REN_SHIFT        (30u)
#define CHCFG_0_DMS              (0x80000000u)
#define CHCFG_0_DMS_SHIFT        (31u)
#define CHITVL_0_ITVL            (0x0000FFFFu)
#define CHITVL_0_ITVL_SHIFT      (0u)
#define CHEXT_0_SPR              (0x0000000Fu)
#define CHEXT_0_SPR_SHIFT        (0u)
#define CHEXT_0_DPR              (0x00000F00u)
#define CHEXT_0_DPR_SHIFT        (8u)
#define NXLA_0_NXLA              (0xFFFFFFFFu)
#define NXLA_0_NXLA_SHIFT        (0u)
#define CRLA_0_CRLA              (0xFFFFFFFFu)
#define CRLA_0_CRLA_SHIFT        (0u)
#define N0SA_1_SA_WD             (0xFFFFFFFFu)
#define N0SA_1_SA_WD_SHIFT       (0u)
#define N0DA_1_DA                (0xFFFFFFFFu)
#define N0DA_1_DA_SHIFT          (0u)
#define N0TB_1_TB                (0xFFFFFFFFu)
#define N0TB_1_TB_SHIFT          (0u)
#define N1SA_1_SA_WD             (0xFFFFFFFFu)
#define N1SA_1_SA_WD_SHIFT       (0u)
#define N1DA_1_DA                (0xFFFFFFFFu)
#define N1DA_1_DA_SHIFT          (0u)
#define N1TB_1_TB                (0xFFFFFFFFu)
#define N1TB_1_TB_SHIFT          (0u)
#define CRSA_1_CRSA              (0xFFFFFFFFu)
#define CRSA_1_CRSA_SHIFT        (0u)
#define CRDA_1_CRDA              (0xFFFFFFFFu)
#define CRDA_1_CRDA_SHIFT        (0u)
#define CRTB_1_CRTB              (0xFFFFFFFFu)
#define CRTB_1_CRTB_SHIFT        (0u)
#define CHSTAT_1_EN              (0x00000001u)
#define CHSTAT_1_EN_SHIFT        (0u)
#define CHSTAT_1_RQST            (0x00000002u)
#define CHSTAT_1_RQST_SHIFT      (1u)
#define CHSTAT_1_TACT            (0x00000004u)
#define CHSTAT_1_TACT_SHIFT      (2u)
#define CHSTAT_1_SUS             (0x00000008u)
#define CHSTAT_1_SUS_SHIFT       (3u)
#define CHSTAT_1_ER              (0x00000010u)
#define CHSTAT_1_ER_SHIFT        (4u)
#define CHSTAT_1_END             (0x00000020u)
#define CHSTAT_1_END_SHIFT       (5u)
#define CHSTAT_1_TC              (0x00000040u)
#define CHSTAT_1_TC_SHIFT        (6u)
#define CHSTAT_1_SR              (0x00000080u)
#define CHSTAT_1_SR_SHIFT        (7u)
#define CHSTAT_1_DL              (0x00000100u)
#define CHSTAT_1_DL_SHIFT        (8u)
#define CHSTAT_1_DW              (0x00000200u)
#define CHSTAT_1_DW_SHIFT        (9u)
#define CHSTAT_1_DER             (0x00000400u)
#define CHSTAT_1_DER_SHIFT       (10u)
#define CHSTAT_1_MODE            (0x00000800u)
#define CHSTAT_1_MODE_SHIFT      (11u)
#define CHSTAT_1_INTM            (0x00010000u)
#define CHSTAT_1_INTM_SHIFT      (16u)
#define CHSTAT_1_DMARQM          (0x00020000u)
#define CHSTAT_1_DMARQM_SHIFT    (17u)
#define CHSTAT_1_SWPRQ           (0x00040000u)
#define CHSTAT_1_SWPRQ_SHIFT     (18u)
#define CHSTAT_1_DNUM            (0xFF000000u)
#define CHSTAT_1_DNUM_SHIFT      (24u)
#define CHCTRL_1_SETEN           (0x00000001u)
#define CHCTRL_1_SETEN_SHIFT     (0u)
#define CHCTRL_1_CLREN           (0x00000002u)
#define CHCTRL_1_CLREN_SHIFT     (1u)
#define CHCTRL_1_STG             (0x00000004u)
#define CHCTRL_1_STG_SHIFT       (2u)
#define CHCTRL_1_SWRST           (0x00000008u)
#define CHCTRL_1_SWRST_SHIFT     (3u)
#define CHCTRL_1_CLRRQ           (0x00000010u)
#define CHCTRL_1_CLRRQ_SHIFT     (4u)
#define CHCTRL_1_CLREND          (0x00000020u)
#define CHCTRL_1_CLREND_SHIFT    (5u)
#define CHCTRL_1_CLRTC           (0x00000040u)
#define CHCTRL_1_CLRTC_SHIFT     (6u)
#define CHCTRL_1_CLRDER          (0x00000080u)
#define CHCTRL_1_CLRDER_SHIFT    (7u)
#define CHCTRL_1_SETSUS          (0x00000100u)
#define CHCTRL_1_SETSUS_SHIFT    (8u)
#define CHCTRL_1_CLRSUS          (0x00000200u)
#define CHCTRL_1_CLRSUS_SHIFT    (9u)
#define CHCTRL_1_SETREN          (0x00001000u)
#define CHCTRL_1_SETREN_SHIFT    (12u)
#define CHCTRL_1_SETSSWPRQ       (0x00004000u)
#define CHCTRL_1_SETSSWPRQ_SHIFT (14u)
#define CHCTRL_1_SETINTM         (0x00010000u)
#define CHCTRL_1_SETINTM_SHIFT   (16u)
#define CHCTRL_1_CLRINTM         (0x00020000u)
#define CHCTRL_1_CLRINTM_SHIFT   (17u)
#define CHCTRL_1_SETDMARQM       (0x00040000u)
#define CHCTRL_1_SETDMARQM_SHIFT (18u)
#define CHCTRL_1_CLRDMARQM       (0x00080000u)
#define CHCTRL_1_CLRDMARQM_SHIFT (19u)
#define CHCFG_1_SEL              (0x00000001u)
#define CHCFG_1_SEL_SHIFT        (0u)
#define CHCFG_1_REQD             (0x00000008u)
#define CHCFG_1_REQD_SHIFT       (3u)
#define CHCFG_1_DRRP             (0x00000800u)
#define CHCFG_1_DRRP_SHIFT       (11u)
#define CHCFG_1_SDS              (0x0000F000u)
#define CHCFG_1_SDS_SHIFT        (12u)
#define CHCFG_1_DDS              (0x000F0000u)
#define CHCFG_1_DDS_SHIFT        (16u)
#define CHCFG_1_SAD              (0x00100000u)
#define CHCFG_1_SAD_SHIFT        (20u)
#define CHCFG_1_DAD              (0x00200000u)
#define CHCFG_1_DAD_SHIFT        (21u)
#define CHCFG_1_WONLY            (0x00800000u)
#define CHCFG_1_WONLY_SHIFT      (23u)
#define CHCFG_1_DEM              (0x01000000u)
#define CHCFG_1_DEM_SHIFT        (24u)
#define CHCFG_1_TCM              (0x02000000u)
#define CHCFG_1_TCM_SHIFT        (25u)
#define CHCFG_1_DIM              (0x04000000u)
#define CHCFG_1_DIM_SHIFT        (26u)
#define CHCFG_1_SBE              (0x08000000u)
#define CHCFG_1_SBE_SHIFT        (27u)
#define CHCFG_1_RSEL             (0x10000000u)
#define CHCFG_1_RSEL_SHIFT       (28u)
#define CHCFG_1_RSW              (0x20000000u)
#define CHCFG_1_RSW_SHIFT        (29u)
#define CHCFG_1_REN              (0x40000000u)
#define CHCFG_1_REN_SHIFT        (30u)
#define CHCFG_1_DMS              (0x80000000u)
#define CHCFG_1_DMS_SHIFT        (31u)
#define CHITVL_1_ITVL            (0x0000FFFFu)
#define CHITVL_1_ITVL_SHIFT      (0u)
#define CHEXT_1_SPR              (0x0000000Fu)
#define CHEXT_1_SPR_SHIFT        (0u)
#define CHEXT_1_DPR              (0x00000F00u)
#define CHEXT_1_DPR_SHIFT        (8u)
#define NXLA_1_NXLA              (0xFFFFFFFFu)
#define NXLA_1_NXLA_SHIFT        (0u)
#define CRLA_1_CRLA              (0xFFFFFFFFu)
#define CRLA_1_CRLA_SHIFT        (0u)
#define SCNT_0_SCNT              (0xFFFFFFFFu)
#define SCNT_0_SCNT_SHIFT        (0u)
#define SSKP_0_SSKP              (0xFFFFFFFFu)
#define SSKP_0_SSKP_SHIFT        (0u)
#define DCNT_0_DCNT              (0xFFFFFFFFu)
#define DCNT_0_DCNT_SHIFT        (0u)
#define DSKP_0_DSKP              (0xFFFFFFFFu)
#define DSKP_0_DSKP_SHIFT        (0u)
#define SCNT_1_SCNT              (0xFFFFFFFFu)
#define SCNT_1_SCNT_SHIFT        (0u)
#define SSKP_1_SSKP              (0xFFFFFFFFu)
#define SSKP_1_SSKP_SHIFT        (0u)
#define DCNT_1_DCNT              (0xFFFFFFFFu)
#define DCNT_1_DCNT_SHIFT        (0u)
#define DSKP_1_DSKP              (0xFFFFFFFFu)
#define DSKP_1_DSKP_SHIFT        (0u)
#define DCTRL_PR                 (0x00000001u)
#define DCTRL_PR_SHIFT           (0u)
#define DCTRL_LVINT              (0x00000002u)
#define DCTRL_LVINT_SHIFT        (1u)
#define DCTRL_LDPR               (0x000F0000u)
#define DCTRL_LDPR_SHIFT         (16u)
#define DCTRL_LWPR               (0x0F000000u)
#define DCTRL_LWPR_SHIFT         (24u)
#define DSCITVL_DITVL            (0x0000FF00u)
#define DSCITVL_DITVL_SHIFT      (8u)
#define DST_EN_EN0               (0x00000001u)
#define DST_EN_EN0_SHIFT         (0u)
#define DST_EN_EN1               (0x00000002u)
#define DST_EN_EN1_SHIFT         (1u)
#define DST_ER_ER0               (0x00000001u)
#define DST_ER_ER0_SHIFT         (0u)
#define DST_ER_ER1               (0x00000002u)
#define DST_ER_ER1_SHIFT         (1u)
#define DST_END_END0             (0x00000001u)
#define DST_END_END0_SHIFT       (0u)
#define DST_END_END1             (0x00000002u)
#define DST_END_END1_SHIFT       (1u)
#define DST_TC_TC0               (0x00000001u)
#define DST_TC_TC0_SHIFT         (0u)
#define DST_TC_TC1               (0x00000002u)
#define DST_TC_TC1_SHIFT         (1u)
#define DST_SUS_SUS0             (0x00000001u)
#define DST_SUS_SUS0_SHIFT       (0u)
#define DST_SUS_SUS1             (0x00000002u)
#define DST_SUS_SUS1_SHIFT       (1u)

#endif /* USB_DC_RZA2 */
