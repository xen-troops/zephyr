/*
 * Driver for Renesas RZ/A2M ethernet controller.
 *
 * Copyright (c) 2023 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ETHERNET_ETH_RZA2M_PRIV_H_
#define DRIVERS_ETHERNET_ETH_RZA2M_PRIV_H_

#define RENESAS_OUI_B0 0x74
#define RENESAS_OUI_B1 0x90
#define RENESAS_OUI_B2 0x50

enum regs {
	/* E-DMAC registers */
	EDMR = 0,
	EDTRR,
	EDRRR,
	TDLAR,
	RDLAR,
	EESR,
	EESIPR,
	TRSCER,
	RMFCR,
	TFTR,
	FDR,
	RMCR,
	TFUCR,
	RFOCR,
	IOSR,
	FCFTR,
	RPADIR,
	TRIMD,
	RBWAR,
	RDFAR,
	TBRAR,
	TDFAR,

	/* Ether registers */
	ECMR,
	RFLR,
	ECSR,
	ECSIPR,
	PIR,
	PSR,
	RDMLR,
	IPGR,
	APR,
	MPR,
	RFCF,
	TPAUSER,
	TPAUSECR,
	BCFRR,
	MAHR,
	MALR,
	TROCR,
	CDCR,
	LCCR,
	CNDCR,
	CEFCR,
	FRECR,
	TSFRCR,
	TLFRCR,
	RFCR,
	MAFCR,

	/* This value must be written at last. */
	RZA2M_ETH_MAX_REGS_OFFSET,
};

/* ECMR */
enum ECMR_BIT {
	ECMR_TPC = BIT(20),
	ECMR_ZPF = BIT(19),
	ECMR_PFR = BIT(18),
	ECMR_RXF = BIT(17),
	ECMR_TXF = BIT(16),
	ECMR_PRCEF = BIT(12),
	ECMR_MPDE = BIT(9),
	ECMR_RE = BIT(6),
	ECMR_TE = BIT(5),
	ECMR_ILB = BIT(3),
	ECMR_RTM = BIT(2),
	ECMR_DM = BIT(1),
	ECMR_PRM = BIT(0),
};

#define ECMR_PAUSE_EN	(ECMR_ZPF | ECMR_PFR | ECMR_RXF | ECMR_TXF)

/* ECSR */
enum ECSR_BIT {
	ECSR_BFR = BIT(5),
	ECSR_PSRTO = BIT(4),
	ECSR_LCHNG = BIT(2),
	ECSR_MPD = BIT(1),
	ECSR_ICD = BIT(0),
};

#define ECSR_ALL	(ECSR_BFR | ECSR_PSRTO | ECSR_LCHNG | ECSR_MPD | ECSR_ICD)

/* PSR */
enum PSR_BIT {
	PSR_LMON = BIT(0),
};

enum EDMR_BIT {
	EDMR_EL = BIT(6), /* Litte endian */
	EDMR_DL1 = BIT(5),
	EDMR_DL0 = BIT(4),
	EDMR_SRST_ETHER = BIT(0),
};

#if CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE == 64
	#define EMDR_DESC EDMR_DL1
#elif CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE == 32
	#define EMDR_DESC EDMR_DL0
#elif CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE == 16 /* Default */
	#define EMDR_DESC 0
#elif defined(CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE)
	#error "invalid CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE"
#endif

/* EDTRR */
enum EDTRR_BIT {
	EDTRR_TR = BIT(0),
};

/* EDRRR */
enum EDRRR_BIT {
	EDRRR_RR = BIT(0),
};

/* EESR */
enum EESR_BIT {
	EESR_TWB	= BIT(30),	/* same as TWB0 */
	EESR_TABT	= BIT(26),	/* Transmit Abort Detect */
	EESR_RABT	= BIT(25),	/* Receive Abort Detect */
	EESR_RFCOF	= BIT(24),	/* Receive Frame Counter Overflow */
	EESR_ECI	= BIT(22),	/* ETHERC Status Register Source */
	EESR_FTC	= BIT(21),	/* Frame Transfer Complete same as TC or TC0 */
	EESR_TDE	= BIT(20),	/* Transmit Descriptor Empty */
	EESR_TFE	= BIT(19),	/* Transmit FIFO Underflow same as TFUF */
	EESR_FRC	= BIT(18),	/* Frame Receive same as FR */
	EESR_RDE	= BIT(17),	/* Receive Descriptor Empty */
	EESR_RFE	= BIT(16),	/* Receive FIFO Overflow Same as RFOF */
	EESR_CND	= BIT(11),	/* Carrier Not Detect */
	EESR_DLC	= BIT(10),	/* Loss of Carrier Detect */
	EESR_CD		= BIT(9),	/* Late Collision Detect */
	EESR_TRO	= BIT(8),	/* Transmit Retry Over */
	EESR_RMAF	= BIT(7),	/* Multicast Address Frame Receive  */
	EESR_RRF	= BIT(4),	/* Alignment Error */
	EESR_RTLF	= BIT(3),	/* Frame-Too-Long Error */
	EESR_RTSF	= BIT(2),	/* Frame-Too-Short Error */
	EESR_PRE	= BIT(1),	/* PHY-LSI Receive Error */
	EESR_CERF	= BIT(0),	/* CRC Error */
};

#define EESR_INIT ~(BIT(31) | BIT(29) | BIT(28) | BIT(27) | BIT(23) | BIT(15) | BIT(14) | \
		    BIT(13) | BIT(12) | BIT(6) | BIT(5))

#define EESR_TX (EESR_FTC | \
		 EESR_TDE | EESR_TFE | \
		 EESR_TABT | EESR_CND | EESR_DLC | EESR_CD | EESR_TRO)

#define EESR_RX (EESR_FRC | \
		 EESR_RDE | EESR_RFE | \
		 EESR_RABT | EESR_RFCOF | EESR_RRF | EESR_RTLF | EESR_RTSF | EESR_PRE | EESR_CERF)

/* RMCR */
enum RMCR_BIT {
	RMCR_RST = BIT(0),
};

/* TRSCER */
enum TRSCER_BIT {
	TRSCER_RMAFCE	= BIT(7),
	TRSCER_RRFCE	= BIT(4),
};

/* Tx descriptor */
struct tx_desc_s {
	volatile uint32_t	td0;
	uint32_t		td1;
	uint32_t		td2;
} __aligned(CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE) __packed;

/* Transmit descriptor 0 bits */
enum TD_STS_BIT {
	TD_TACT	= BIT(31),
	TD_TDLE	= BIT(30),
	TD_TFP1	= BIT(29),
	TD_TFP0	= BIT(28),
	TD_TFE	= BIT(27),
	TD_TWBI	= BIT(26),
};

#define TD_F1ST	TD_TFP1
#define TD_FEND	TD_TFP0
#define TD_TFP	(TD_TFP1 | TD_TFP0)

#define TD_TFS8_TAD	BIT(8) /* Transmit abort is detected eq EESR.TABT flag) */
#define TD_TFS3_CND	BIT(3) /* No carrier is detected eq EESR.CND flag */
#define TD_TFS2_DLC	BIT(2) /* Loss of carrier is detected eq EESR.DLC flag */
#define TD_TFS1_CD	BIT(1) /* Late collision is detected eq EESR.CD flag */
#define TD_TFS0_TRO	BIT(0) /* Transmit retry over eq EESR.TRO flag */

#define TD_TFS_MASK	(TD_TFS0_TRO | TD_TFS1_CD | TD_TFS2_DLC | TD_TFS3_CND)

/* Rx buffer descriptor */
struct rx_desc_s {
	volatile uint32_t	rd0;
	uint32_t		rd1;
	uint32_t		rd2;
} __aligned(CONFIG_ETH_RZA2M_DESC_ALIGN_SIZE) __packed;

/* Receive descriptor 0 bits */
#define	RD_RACT		BIT(31)
#define RD_RDLE		BIT(30)
#define RD_RFP1		BIT(29)
#define RD_RFP0		BIT(28)
#define RD_RFE		BIT(27)

#define RD_RFS9_RFOF		BIT(9)
#define RD_RFS8_RABT		BIT(8)
#define RD_RFS7_RMAF		BIT(7)
#define RD_RFS4_RRF		BIT(4)
#define RD_RFS3_RTLF		BIT(3)
#define RD_RFS2_RTSF		BIT(2)
#define RD_RFS1_PRE		BIT(1)
#define RD_RFS0_CERF		BIT(0)

#define RD_F1ST		RD_RFP1
#define RD_FEND		RD_RFP0
#define RD_RFP		(RD_RFP1 | RD_RFP0)

/* Receive descriptor 1 bits */
enum RD_LEN_BIT {
	RD_RFL	= 0x0000ffff,	/* receive frame  length */
	RD_RBL	= 0xffff0000,	/* receive buffer length */
};

/*
 *
 * RZA2M default regs configuration which does not support dynamic settings
 *
 */

#define RZA2M_ETH_CFG_EDMR_VAL		(EDMR_EL | EMDR_DESC)
#define RZA2M_ETH_CFG_TRSCER_VAL	0x0
/* Transmit FIFO Threshold */
#define RZA2M_ETH_CFG_TFTR_VAL		0x0
/* FIFO Depth - default */
#define RZA2M_ETH_CFG_FDR_VAL		0x0000070f
/* don't clear EDRRR.RR after each packet */
#define RZA2M_ETH_CFG_RMCR_VAL		(RMCR_RST)
/* Receive Data Padding Insert - none */
#define RZA2M_ETH_CFG_RPADIR_VAL	0x0

/*Ethernet length 1514bytes + CRC */
#define RZA2M_ETH_CFG_RFLRL_VAL		1518

/*Ethernet intergap is 96-bit time */
#define RZA2M_ETH_CFG_IPGRL_VAL		0x14

/*
 * Flow control (FC)
 */
#define RZA2M_ETH_CFG_TC_APR_VAL	0xffff
#define RZA2M_ETH_CFG_TC_MPR_VAL	0x0
#define RZA2M_ETH_CFG_TC_TPAUSER_VAL	0x0
/* FC FIFO Threshold */
#define RZA2M_ETH_CFG_TC_FCFTR_VAL	0x0

/*
 * (ECMR_ZPF | ECMR_RXF | ECMR_TXF )
 */
#define RZA2M_ETH_CFG_TC_ECMR_VAL	0x0

/* size of pre-allocated packet fragments */
#define RZA2M_ETH_RX_FRAG_SIZE CONFIG_NET_BUF_DATA_SIZE

#endif /* DRIVERS_ETHERNET_ETH_RZA2M_PRIV_H_ */
