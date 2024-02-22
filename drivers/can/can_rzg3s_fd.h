/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CAN_CAN_RZG3S_FD_H_
#define ZEPHYR_DRIVERS_CAN_CAN_RZG3S_FD_H_

#define CAN_RZG3S_MAXBITRATE   8000000
/* per-channel: number of AFL entries */
#define CAN_RZGS_FD_MAX_FILTER CONFIG_CAN_MAX_FILTER

/* RX Max number of buffers per channel */
#define CAN_RZG3S_RX_MAX_PKTS CONFIG_CAN_RZGS_FD_MAX_RX
/* RX FIFO RFDC value */
#if (CONFIG_CAN_RZGS_FD_MAX_RX == 4)
#define CAN_RZGS_RX_RFDC 0x1
#elif (CONFIG_CAN_RZGS_FD_MAX_RX == 8)
#define CAN_RZGS_RX_RFDC 0x2
#elif (CONFIG_CAN_RZGS_FD_MAX_RX == 16)
#define CAN_RZGS_RX_RFDC 0x3
#elif (CONFIG_CAN_RZGS_FD_MAX_RX == 32)
#define CAN_RZGS_RX_RFDC 0x4
#elif (CONFIG_CAN_RZGS_FD_MAX_RX == 48)
#define CAN_RZGS_RX_RFDC 0x5
#elif (CONFIG_CAN_RZGS_FD_MAX_RX == 64)
#define CAN_RZGS_RX_RFDC 0x6
#elif (CONFIG_CAN_RZGS_FD_MAX_RX == 128)
#define CAN_RZGS_RX_RFDC 0x7
#else
BUILD_ASSERT(0, "incorrect value for CONFIG_CAN_RZGS_FD_MAX_RX");
#endif /* CONFIG_CAN_RZGS_FD_MAX_RX */

/* RX Max CAN Payload Data Size, RX FIFO RFPLS value */
#if (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 8)
#define CAN_RZGS_RX_RFPLS 0x0
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 12)
#define CAN_RZGS_RX_RFPLS 0x1
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 16)
#define CAN_RZGS_RX_RFPLS 0x2
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 20)
#define CAN_RZGS_RX_RFPLS 0x3
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 24)
#define CAN_RZGS_RX_RFPLS 0x4
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 32)
#define CAN_RZGS_RX_RFPLS 0x5
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 48)
#define CAN_RZGS_RX_RFPLS 0x6
#elif (CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE == 64)
#define CAN_RZGS_RX_RFPLS 0x7
#else
BUILD_ASSERT(0, "incorrect value for CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE");
#endif /* CONFIG_CAN_RZGS_FD_MAX_RX_DATA_SIZE */

/* TX Max number of TX Message Buffer per channel to use */
#define CAN_RZG3S_TX_MAX_PKTS CONFIG_CAN_RZGS_FD_MAX_TX
#if (CONFIG_CAN_RZGS_FD_MAX_TX > 32)
BUILD_ASSERT(0, "incorrect value for CONFIG_CAN_RZGS_FD_MAX_TX");
#endif

/* Rx FIFO is a global resource of the controller. There are 8 such FIFOs
 * available. Each channel gets a dedicated one RX FIFO
 */
#define CAN_RZG3S_RX_FIFO_NUM 8

/* TX Message Buffers a per channel resource.
 * Each channel has up to 64 TX Message Buffers dedicated to them, but can use only 32 of them.
 * CH0: TXMB0 - 15 and TXMB32 - 47 are available
 * CH1: TXMB64 - 79 and TXMB96 - 111 are available
 */
#define CAN_RZG3S_TM_CH1_OFFSET 64

/* RX Message buffers are not used */
#define R_CFD_RMNB_CFG	0x0

/*
 * Global registers
 */
/* CFDGIPV */
#define R_CFDG_IPV             0x0080
/* CFDGCFG */
#define R_CFDG_CFG             0x0084
/* CFDGCTR */
#define R_CFDG_CTR             0x0088
/* CFDGSTS */
#define R_CFDG_STS             0x008c
/* CFDGERFL */
#define R_CFDG_ERFL            0x0090
/* CFDGTSC */
#define R_CFDG_TSC             0x0094
/* (CFDRMNB) */
#define R_CFD_RMNB             0x00a4
/* ((CFDGTINTSTS0) 8bit access */
#define R_CFDG_TINTSTS0(ch)    (0x1300 + (ch))
/* (CFDGFDCFG) */
#define R_CFDG_FDCFG           0x1314
/*
 * Global registers fields
 */
/* CFDGCFG */
#define R_CFDG_CFG_TSSS        BIT(12)
#define R_CFDG_CFG_CMPOC       BIT(5)
#define R_CFDG_CFG_DCS         BIT(4)
#define R_CFDG_CFG_MME         BIT(3)
#define R_CFDG_CFG_DRE         BIT(2)
#define R_CFDG_CFG_DCE         BIT(1)
#define R_CFDG_CFG_TPRI        BIT(0)
/* CFDGCTR */
#define R_CFDG_CTR_TSRST       BIT(16)
#define R_CFDG_CTR_MOWEIE      BIT(15)
#define R_CFDG_CTR_QMEIE       BIT(14)
#define R_CFDG_CTR_CFMPOFIE    BIT(11)
#define R_CFDG_CTR_THLEIE      BIT(10)
#define R_CFDG_CTR_MEIE        BIT(9)
#define R_CFDG_CTR_DEIE        BIT(8)
#define R_CFDG_CTR_GSLPR       BIT(2)
#define R_CFDG_CTR_GMDC_MASK   (0x3)
#define R_CFDG_CTR_GMDC_GOPM   (0x0)
#define R_CFDG_CTR_GMDC_GRESET (0x1)
#define R_CFDG_CTR_GMDC_GHALT  (0x2)
#define R_CFDG_CTR_GMDC_KEEP   (0x3)
/* CFDGSTS */
#define R_CFDG_STS_GRAMINIT    BIT(3)
#define R_CFDG_STS_GSLPSTS     BIT(2)
#define R_CFDG_STS_GHLTSTS     BIT(1)
#define R_CFDG_STS_GRSTSTS     BIT(0)
/* CFDGERFL */
#define R_CFDG_ERFL_EEF1       BIT(17)
#define R_CFDG_ERFL_EEF0       BIT(16)
#define R_CFDG_ERFL_CMPOF      BIT(3)
#define R_CFDG_ERFL_MES        BIT(1)

/* Non-operational status */
#define R_CFDG_STS_GNOPM       (BIT(0) | BIT(1) | BIT(2) | BIT(3))
/* ((CFDGTINTSTS0) */
#define R_CFDG_TINTSTS0_CFOTIF BIT(6)
#define R_CFDG_TINTSTS0_TQOFIF BIT(5)
#define R_CFDG_TINTSTS0_THIF   BIT(4)
#define R_CFDG_TINTSTS0_CFTIF  BIT(3)
#define R_CFDG_TINTSTS0_TQIF   BIT(2)
#define R_CFDG_TINTSTS0_TAIF   BIT(1)
#define R_CFDG_TINTSTS0_TSIF   BIT(0)
/* (CFDGFDCFG) */
#define R_CFDG_FDCFG_RPED      BIT(0)

/*
 * Channel Registers
 */
/* CFDCnNCFG */
#define R_CFDC_NCFG(idx)             (0x0000 + (0x10 * (idx)))
/* CFDCnCTR */
#define R_CFDC_CTR(idx)              (0x0004 + (0x10 * (idx)))
/* CFDCnSTS */
#define R_CFDC_STS(idx)              (0x0008 + (0x10 * (idx)))
/* CFDCnERFL */
#define R_CFDC_ERFL(idx)             (0x000C + (0x10 * (idx)))
/* CAN FD mode specific registers */
/* CFDCnDCFG */
#define R_CFDC_DCFG(idx)             (0x1400 + (0x20 * (idx)))
/* CFDCnFDCFG */
#define R_CFDC_FDCFG(idx)            (0x1400 + 0x04 + (0x20 * (idx)))
/* CFDCnFDCTR */
#define R_CFDC_FDCTR(idx)            (0x1400 + 0x08 + (0x20 * (idx)))
/* CFDCnFDSTS */
#define R_CFDC_FDSTS(idx)            (0x1400 + 0x0c + (0x20 * (idx)))
/* CFDCnFDCRC */
#define R_CFDC_FDCRC(idx)            (0x1400 + 0x10 + (0x20 * (idx)))
/*
 * Channel Registers fields
 */
/* CFDCnCTR */
#define R_CFDC_CTR_CTMS_MASK         GENMASK(26, 25)
#define R_CFDC_CTR_CTMS(val)         (((val) << 25) & R_CFDC_CTR_CTMS_MASK)
#define R_CFDC_CTR_CTMS_STANDARD     (0x0)
#define R_CFDC_CTR_CTMS_LISTENONLY   (0x1)
#define R_CFDC_CTR_CTMS_EXT_LOOPBACK (0x2)
#define R_CFDC_CTR_CTMS_INT_LOOPBACK (0x3)
#define R_CFDC_CTR_CTME              BIT(24)
#define R_CFDC_CTR_ERRD              BIT(23)
#define R_CFDC_CTR_BOM_MASK          GENMASK(22, 21)
#define R_CFDC_CTR_BOM(val)          (((val) << 21) & GENMASK(22, 21))
#define R_CFDC_CTR_BOM_ISO           (0x0)
#define R_CFDC_CTR_BOM_BENTRY        (0x1)
#define R_CFDC_CTR_BOM_BEND          (0x2)
#define R_CFDC_CTR_TDCVFIE           BIT(19)
#define R_CFDC_CTR_TAIE              BIT(16)
#define R_CFDC_CTR_ALIE              BIT(15)
#define R_CFDC_CTR_BLIE              BIT(14)
#define R_CFDC_CTR_OLIE              BIT(13)
#define R_CFDC_CTR_BORIE             BIT(12)
#define R_CFDC_CTR_BOEIE             BIT(11)
#define R_CFDC_CTR_EPIE              BIT(10)
#define R_CFDC_CTR_EWIE              BIT(9)
#define R_CFDC_CTR_BEIE              BIT(8)
#define R_CFDC_CTR_CSLPR             BIT(2)
#define R_CFDC_CTR_CHDC_MASK         GENMASK(1, 0)
#define R_CFDC_CTR_CHDC_OPM          (0x0)
#define R_CFDC_CTR_CHDC_RESET        (0x1)
#define R_CFDC_CTR_CHDC_HALT         (0x2)
#define R_CFDC_CTR_CHDC_KEEP         (0x3)
/* CFDCnSTS */
#define R_CFDC_STS_TEC(reg)          (((reg) & GENMASK(31, 24)) >> 24)
#define R_CFDC_STS_REC(reg)          (((reg) & GENMASK(23, 16)) >> 16)
#define R_CFDC_STS_ESIF              BIT(8)
#define R_CFDC_STS_COMSTS            BIT(7)
#define R_CFDC_STS_RECSTS            BIT(6)
#define R_CFDC_STS_TRMSTS            BIT(5)
#define R_CFDC_STS_BOSTS             BIT(4)
#define R_CFDC_STS_EPSTS             BIT(3)
#define R_CFDC_STS_SLPSTS            BIT(2)
#define R_CFDC_STS_HLTSTS            BIT(1)
#define R_CFDC_STS_RSTSTS            BIT(0)
/* Channel Non-operational status */
#define R_CFDC_STS_CNOPM             (BIT(0) | BIT(1) | BIT(2) | BIT(3))
/* CFDCnERFL */
#define R_CFDC_ERFL_ADERR            BIT(14)
#define R_CFDC_ERFL_B0ERR            BIT(13)
#define R_CFDC_ERFL_B1ERR            BIT(12)
#define R_CFDC_ERFL_CERR             BIT(11)
#define R_CFDC_ERFL_AERR             BIT(10)
#define R_CFDC_ERFL_FERR             BIT(9)
#define R_CFDC_ERFL_SERR             BIT(8)
#define R_CFDC_ERFL_ALF              BIT(7)
#define R_CFDC_ERFL_BLF              BIT(6)
#define R_CFDC_ERFL_OVLF             BIT(5)
#define R_CFDC_ERFL_BORF             BIT(4)
#define R_CFDC_ERFL_BOEF             BIT(3)
#define R_CFDC_ERFL_EPF              BIT(2)
#define R_CFDC_ERFL_EWF              BIT(1)
#define R_CFDC_ERFL_BEF              BIT(0)
/* CFDCnFDCFG */
#define R_CFDC_FDCFG_TDCO_MSK        GENMASK(23, 16)
#define R_CFDC_FDCFG_TDCO_OFS        16
#define R_CFDC_FDCFG_TDCO_TDCE       BIT(9)
#define R_CFDC_FDCFG_TDCO_TDCOC      BIT(8)
/* CFDCnFDSTS */
#define R_CFDC_FDSTS_TDCO_TDCVF      BIT(15)
#define R_CFDC_FDSTS_TDCO_TDCR       GENMASK(7, 0)

/*
 * RX FIFO Registers
 */
/* CFDRFCCn */
#define R_CFD_RFCC(x)             (0x00c0 + (0x04 * (x)))
/* (CFDRFSTSn) */
#define R_CFD_RFSTS(x)            (0x00e0 + (0x04 * (x)))
/* (CFDRFPCTRn) */
#define R_CFD_RFPCTR(x)           (0x0100 + (0x04 * (x)))
/*(CFDFESTS)*/
#define R_CFD_RF_ESTS             0x02a0
/* (CFDFMSTS) */
#define R_CFD_RF_MSTS             0x02a8
/* (CFDRFISTS) */
#define R_CFD_RF_ISTS             0x02ac
/*
 * RX FIFO Registers fields
 */
/* CFDRFCCn */
#define R_CFD_RFCC_RFFIE          BIT(16)
#define R_CFD_RFCC_RFIGCV(x)      (((x) & 0x7) << 13)
#define R_CFD_RFCC_RFIM           BIT(12)
#define R_CFD_RFCC_RFDC(x)        (((x) & 0x7) << 8)
#define R_CFD_RFCC_RFPLS(x)       (((x) & 0x7) << 4)
#define R_CFD_RFCC_RFIE           BIT(1)
#define R_CFD_RFCC_RFE            BIT(0)
/* (CFDRFSTSn) */
#define R_CFD_RFSTS_RFIF          BIT(3)
#define R_CFD_RFSTS_RFMLT         BIT(2)
/*
 * RX FIFO Access Message Buffer Component (CFDRFMBCPb[i])
 */
#define R_CFD_RF_OFFSET           0x6000
#define R_CFD_RF_RFID(x)          (R_CFD_RF_OFFSET + (0x80 * (x)))
#define R_CFD_RF_RFPTR(x)         (R_CFD_RF_OFFSET + (0x80 * (x)) + 0x04)
#define R_CFD_RF_RFFDSTS(x)       (R_CFD_RF_OFFSET + (0x80 * (x)) + 0x08)
#define R_CFD_RF_RFDF(x, dw)      (R_CFD_RF_OFFSET + (0x80 * (x)) + 0x0c + (0x04 * (dw)))
/*
 * RX FIFO Access Message Buffer Component fields
 */
/* (CFDRFIDn) */
#define R_CFD_RF_RFID_RFIDE       BIT(31)
#define R_CFD_RF_RFID_RFRTR       BIT(30)
/* (CFDRFPTRn) */
#define R_CFD_RF_RFPTR_RFDLC(x)   (((x) >> 28) & 0xf)
#define R_CFD_RF_RFPTR_RFTS(x)    (((x) >> 0) & 0xffff)
/* (CFDRFFDSTSn) */
#define R_CFD_RF_RFFDSTS_RFPTR(x) (((x) & GENMASK(31, 16)) >> 16)
#define R_CFD_RF_RFFDSTS_RFFDF    BIT(2)
#define R_CFD_RF_RFFDSTS_RFBRS    BIT(1)
#define R_CFD_RF_RFFDSTS_RFESI    BIT(0)

/*
 * TX Message Buffer
 */
/* (CFDTMCn) */
#define R_CFDC_TM_C(idx)              (0x02d0 + (0x01 * (idx)))
/* (CFDTMSTSn) */
#define R_CFDC_TM_STS(idx)            (0x07D0 + (0x01 * (idx)))
/* (CFDTMTRSTSn) */
#define R_CFDC_TM_TRSTS(ch, y)        (0x0cd0 + (0x04 * (((ch) * 2) + (y))))
/* (CFDTMTARSTSn) */
#define R_CFDC_TM_TARSTS(ch, y)       (0x0d70 + (0x04 * (((ch) * 2) + (y))))
/* (CFDTMTCSTSn) */
#define R_CFDC_TM_TCSTS(ch, y)        (0x0e10 + (0x04 * (((ch) * 2) + (y))))
/* (CFDTMTASTSn) */
#define R_CFDC_TM_TASTS(ch, y)        (0x0eb0 + (0x04 * (((ch) * 2) + (y))))
/* (CFDTMIECn) */
#define R_CFDC_TM_IEC(ch, y)          (0x0f50 + (0x04 * (((ch) * 2) + (y))))
/*
 * TX Message Buffer fields
 */
/* (CFDTMCn) */
#define R_CFDC_TM_C_TMOM              BIT(2)
#define R_CFDC_TM_C_TMTAR             BIT(1)
#define R_CFDC_TM_C_TMTR              BIT(0)
/* (CFDTMSTSn) */
#define R_CFDC_TM_STS_TMTRF_MASK      GENMASK(2, 1)
#define R_CFDC_TM_STS_TMTRF(reg)      (((reg) & R_CFDC_TM_STS_TMTRF_MASK) >> 1)
#define R_CFDC_TM_STS_TMTRF_TOK_ABORT 0x3
#define R_CFDC_TM_STS_TMTRF_TOK       0x2
#define R_CFDC_TM_STS_TMTRF_ABORT     0x1
#define R_CFDC_TM_STS_TMTRF_NORES     0x0
/*
 * TX Message Buffer Access Message Buffer Component (CFDTMBCPb[i])
 */
#define R_CFD_TM_OFFSET               0x10000
#define R_CFD_TM_ID(idx)              (R_CFD_TM_OFFSET + (0x80 * (idx)))
#define R_CFD_TM_PTR(idx)             (R_CFD_TM_OFFSET + (0x80 * (idx)) + 0x04)
#define R_CFD_TM_FDCTR(idx)           (R_CFD_TM_OFFSET + (0x80 * (idx)) + 0x08)
#define R_CFD_TM_DF(idx, dw)          (R_CFD_TM_OFFSET + (0x80 * (idx)) + 0x0c + (0x04 * (dw)))
/*
 * TX Message Buffer Access Message Buffer Component fields
 */
/* (CFDTMIDn */
#define R_CFD_TM_ID_TMIDE             BIT(31)
#define R_CFD_TM_ID_TMRTR             BIT(30)
#define R_CFD_TM_ID_THLEN             BIT(29)
/* (CFDTMPTRn) */
#define R_CFD_TM_PTR_DLC(x)           (((x) & 0xf) << 28)
/* (CFDTMFDCTRn) */
#define R_CFD_TM_FDCTR_TMFDF          BIT(2)
#define R_CFD_TM_FDCTR_TMBRS          BIT(1)
#define R_CFD_TM_FDCTR_TMESI          BIT(0)

/*
 * Acceptance Filter List Entry Control Register
 */
/* (CFDGAFLECTR) */
#define R_CFDG_AFL_ECTR             0x0098
/* (CFDGAFLCFG0) */
#define R_CFDG_AFL_CFG              0x009C
#define R_CFDG_AFL_OFFSET           0x1800
/* (CFDGAFLIDn) */
#define R_CFDG_AFL_ID(j)            (R_CFDG_AFL_OFFSET + (0x10 * (j)))
/* (CFDGAFLMn) */
#define R_CFDG_AFL_M(j)             (R_CFDG_AFL_OFFSET + 0x04 + (0x10 * (j)))
/* (CFDGAFLP0n) */
#define R_CFDG_AFL_LP0(j)           (R_CFDG_AFL_OFFSET + 0x08 + (0x10 * (j)))
/* (CFDGAFLP1n) */
#define R_CFDG_AFL_LP1(j)           (R_CFDG_AFL_OFFSET + 0x0c + (0x10 * (j)))
/*
 * Acceptance Filter List Entry Control fields
 */
#define R_CFDG_AFL_ENTRIES_PER_PAGE 16
/* (CFDGAFLECTR) */
#define R_CFDG_AFL_ECTR_AFLDAE      BIT(8)
#define R_CFDG_AFL_ECTR_AFLPN       GENMASK(6, 0)
/* (CFDGAFLIDn) */
#define R_CFDG_AFL_ID_GAFLIDE       BIT(31)
#define R_CFDG_AFL_ID_GAFLRTR       BIT(30)
#define R_CFDG_AFL_ID_GAFLLB        BIT(29)
/* (CFDGAFLMn) */
#define R_CFDG_AFL_M_GAFLIDEM       BIT(31)
#define R_CFDG_AFL_M_GAFLRTRM       BIT(30)
/* (CFDGAFLP0n) */
#define R_CFDG_AFL_LP0_GAFLPTR_MASK GENMASK(31, 16)
#define R_CFDG_AFL_LP0_GAFLPTR_OFS  16

#endif /* ZEPHYR_DRIVERS_CAN_CAN_RZG3S_FD_H_ */
