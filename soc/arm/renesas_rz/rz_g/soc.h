/*
 * Copyright (c) 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifndef _ASMLANGUAGE

/* __CM33_REV                0x0004U   Core revision r0p4
 * M33 core0 - no FPU, M33 core1 - with FPU
 */
#define __SAUREGION_PRESENT CONFIG_CPU_HAS_ARM_SAU
#define __VTOR_PRESENT      CONFIG_CPU_CORTEX_M_HAS_VTOR
#define __MPU_PRESENT       CONFIG_CPU_HAS_ARM_MPU
#define __FPU_PRESENT       CONFIG_CPU_HAS_FPU /* FPU present */
#define __DSP_PRESENT       CONFIG_ARMV8_M_DSP /* DSP ext present - no */
#define __DCACHE_PRESENT    CONFIG_CPU_HAS_DCACHE /* DCACHE present - no */
#define __ICACHE_PRESENT    CONFIG_CPU_HAS_ICACHE /* ICACHE present - no */

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
