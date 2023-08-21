/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RZA2_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_RZA2_DMA_H_

/*
 * The following format represents RZ/A2 Soc series device specific
 * dma channel configurations. Expample from Table 9.4 of HW Manual:
 * NAME     MID       RID TM AM[2:0] LVL HIEN LOEN REQD
 * RXF_DMA0 1010_1011 11  0  001     0   1    0    0/1
 *
 * DMA Slot configuration is uin32_t bitmask and has the following format:
 * RID  [1..0]
 * MID  [9..2]
 * TM   [11..10] - values: 0, 1, TM_BOTH
 * AM   [14..12]
 * LVL  [15]
 * HIEN [16]
 * LOEN [17]
 * REQD [19..18] - values: 0, 1, REQ_BOTH
 */

#define RID(x)   ((x & 0x3))
#define MID(x)   ((x & 0xff) << 2)
#define TM_BOTH  3
#define TM(x)    ((x & 0x3) << 10)
#define AM(x)    ((x & 0x7) << 12)
#define LVL1     (1 << 15)
#define HIEN1    (1 << 16)
#define LOEN1    (1 << 17)
#define REQ_BOTH 3
#define REQD(x)  ((x & 0x3) << 18)

#define DMA_OSTM0TINT (MID(8) | RID(3) | TM(TM_BOTH) | AM(2) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OSTM1TINT (MID(9) | RID(3) | TM(TM_BOTH) | AM(2) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OSTM2TINT (MID(10) | RID(3) | TM(TM_BOTH) | AM(2) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA0 (MID(16) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB0 (MID(17) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIC0 (MID(18) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGID0 (MID(19) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA1 (MID(20) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB1 (MID(21) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA2 (MID(22) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB2 (MID(23) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA3 (MID(24) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB3 (MID(25) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIC3 (MID(26) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGID3 (MID(27) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA4 (MID(28) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB4 (MID(29) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIC4 (MID(30) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGID4 (MID(31) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TCIV4 (MID(32) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGUI5 (MID(33) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIV5 (MID(34) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIW5 (MID(35) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA6 (MID(36) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB6 (MID(37) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIC6 (MID(38) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGID6 (MID(39) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA7 (MID(40) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB7 (MID(41) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIC7 (MID(42) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGID7 (MID(43) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIV7 (MID(44) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_TGIA8 (MID(45) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIB8 (MID(46) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGIC8 (MID(47) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_TGID8 (MID(48) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA0  (MID(49) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB0  (MID(50) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC0   (MID(51) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD0   (MID(52) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE0   (MID(56) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF0   (MID(57) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA0 (MID(58) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB0 (MID(59) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF0    (MID(60) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF0    (MID(61) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA1  (MID(62) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB1  (MID(63) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC1   (MID(64) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD1   (MID(65) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE1   (MID(69) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF1   (MID(70) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA1 (MID(71) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB1 (MID(72) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF1    (MID(73) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF1    (MID(74) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA2  (MID(75) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB2  (MID(76) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC2   (MID(77) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD2   (MID(78) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE2   (MID(82) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF2   (MID(83) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA2 (MID(84) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB2 (MID(85) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF2    (MID(86) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF2    (MID(87) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA3  (MID(88) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB3  (MID(89) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC3   (MID(90) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD3   (MID(91) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE3   (MID(95) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF3   (MID(96) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA3 (MID(97) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB3 (MID(98) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF3    (MID(99) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF3    (MID(100) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA4  (MID(101) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB4  (MID(102) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC4   (MID(103) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD4   (MID(104) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE4   (MID(108) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF4   (MID(109) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA4 (MID(110) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB4 (MID(111) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF4    (MID(112) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF4    (MID(113) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA5  (MID(114) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB5  (MID(115) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC5   (MID(116) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD5   (MID(117) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE5   (MID(121) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF5   (MID(122) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA5 (MID(123) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB5 (MID(124) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF5    (MID(125) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF5    (MID(126) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA6  (MID(127) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB6  (MID(128) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC6   (MID(129) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD6   (MID(130) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE6   (MID(134) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF6   (MID(135) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA6 (MID(136) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB6 (MID(137) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF6    (MID(138) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF6    (MID(139) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_CCMPA7  (MID(140) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CCMPB7  (MID(141) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPC7   (MID(142) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPD7   (MID(143) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPE7   (MID(147) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_CMPF7   (MID(148) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGA7 (MID(149) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_ADTRGB7 (MID(150) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_OVF7    (MID(151) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_UNF7    (MID(152) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_S12DI0    (MID(153) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(0))
#define DMA_S12GBADI0 (MID(154) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(0))
#define DMA_S12GCADI0 (MID(155) | RID(3) | TM(TM_BOTH) | AM(1) | HIEN1 | REQD(0))

#define DMA_INT_ssif_dma_rx_0 (MID(156) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INT_ssif_dma_tx_0 (MID(156) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_INT_ssif_dma_rx_1 (MID(157) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INT_ssif_dma_tx_1 (MID(157) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_INT_ssif_dma_rx_2 (MID(158) | RID(3) | TM(0) | AM(2) | HIEN1 | REQD(REQ_BOTH))
#define DMA_INT_ssif_dma_tx_2 (MID(158) | RID(3) | TM(0) | AM(2) | HIEN1 | REQD(REQ_BOTH))

#define DMA_INT_ssif_dma_rx_3 (MID(159) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INT_ssif_dma_tx_3 (MID(159) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_SPDIFRXI (MID(160) | RID(3) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_SPDIFTXI (MID(161) | RID(3) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_INTRIICRI0 (MID(162) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INTRIICTI0 (MID(162) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_INTRIICRI1 (MID(163) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INTRIICTI1 (MID(163) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_INTRIICRI2 (MID(164) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INTRIICTI2 (MID(164) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_INTRIICRI3 (MID(165) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_INTRIICTI3 (MID(165) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_RXI0 (MID(166) | RID(2) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(0))
#define DMA_TXI0 (MID(166) | RID(1) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(1))

#define DMA_RXI1 (MID(167) | RID(2) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(0))
#define DMA_TXI1 (MID(167) | RID(1) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(1))

#define DMA_RXI2 (MID(168) | RID(2) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(0))
#define DMA_TXI2 (MID(168) | RID(1) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(1))

#define DMA_RXI3 (MID(169) | RID(2) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(0))
#define DMA_TXI3 (MID(169) | RID(1) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(1))

#define DMA_RXI4 (MID(170) | RID(2) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(0))
#define DMA_TXI4 (MID(170) | RID(1) | TM(0) | AM(4) | LVL1 | HIEN1 | REQD(1))

#define DMA_RXF_DMA0 (MID(171) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA1 (MID(172) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA2 (MID(173) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA3 (MID(174) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA4 (MID(175) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA5 (MID(176) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA6 (MID(177) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RXF_DMA7 (MID(178) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_COM_DMA0 (MID(179) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_COM_DMA1 (MID(180) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_SPRI0 (MID(181) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_SPTI0 (MID(181) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_SPRI1 (MID(182) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_SPTI1 (MID(182) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_SPRI2 (MID(183) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_SPTI2 (MID(183) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_SRXI0 (MID(184) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_STXI0 (MID(184) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define DMA_SRXI1 (MID(185) | RID(2) | TM(0) | AM(2) | HIEN1 | REQD(0))
#define DMA_STXI1 (MID(185) | RID(1) | TM(0) | AM(2) | HIEN1 | REQD(1))

#define IPLS (MID(186) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_RDRDY1 (MID(187) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_RDRDY0 (MID(188) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_WRRDY4 (MID(189) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_WRRDY1 (MID(191) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_WRRDY0 (MID(240) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_IRRDY  (MID(241) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))
#define DMA_IWRRDY (MID(242) | RID(3) | TM(0) | AM(1) | HIEN1 | REQD(REQ_BOTH))

#define DMA_PAE0 (MID(243) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_PAF0 (MID(243) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_PAE1 (MID(244) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_PAF1 (MID(244) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_PAE2 (MID(245) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_PAF2 (MID(245) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_PAE3 (MID(246) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_PAF3 (MID(246) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_PAE4 (MID(247) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_PAF4 (MID(247) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#define DMA_PAE5 (MID(248) | RID(2) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(0))
#define DMA_PAF5 (MID(248) | RID(1) | TM(0) | AM(2) | LVL1 | HIEN1 | REQD(1))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_RZA2_DMA_H_ */
