/* cache.c - d-cache support for AARCH64 CPUs */

/*
 * Copyright 2020-2021 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief d-cache manipulation
 *
 * This module contains functions for manipulation of the d-cache.
 */

#include <zephyr/arch/arm64/cache.h>
#include <zephyr/cache.h>

#define	CTR_EL0_DMINLINE_SHIFT		16
#define	CTR_EL0_DMINLINE_MASK		BIT_MASK(4)
#define	CTR_EL0_CWG_SHIFT		24
#define	CTR_EL0_CWG_MASK		BIT_MASK(4)

/* clidr_el1 */
#define CLIDR_EL1_LOC_SHIFT		24
#define CLIDR_EL1_LOC_MASK		BIT_MASK(3)
#define CLIDR_EL1_CTYPE_SHIFT(level)	((level) * 3)
#define CLIDR_EL1_CTYPE_MASK		BIT_MASK(3)

/* ccsidr_el1 */
#define CCSIDR_EL1_LN_SZ_SHIFT		0
#define CCSIDR_EL1_LN_SZ_MASK		BIT_MASK(3)
#define CCSIDR_EL1_WAYS_SHIFT		3
#define CCSIDR_EL1_WAYS_MASK		BIT_MASK(10)
#define CCSIDR_EL1_SETS_SHIFT		13
#define CCSIDR_EL1_SETS_MASK		BIT_MASK(15)

#define dc_ops(op, val)							\
({									\
	__asm__ volatile ("dc " op ", %0" :: "r" (val) : "memory");	\
})

static size_t dcache_line_size;

size_t arch_dcache_line_size_get(void)
{
	uint64_t ctr_el0;
	uint32_t dminline;

	if (dcache_line_size) {
		return dcache_line_size;
	}

	ctr_el0 = read_sysreg(CTR_EL0);

	dminline = (ctr_el0 >> CTR_EL0_DMINLINE_SHIFT) & CTR_EL0_DMINLINE_MASK;

	dcache_line_size = 4 << dminline;

	return dcache_line_size;
}

/*
 * operation for data cache by virtual address to PoC
 * ops:  K_CACHE_INVD: invalidate
 *	 K_CACHE_WB: clean
 *	 K_CACHE_WB_INVD: clean and invalidate
 */
int arm64_dcache_range(void *addr, size_t size, int op)
{
	size_t line_size;
	uintptr_t start_addr = (uintptr_t)addr;
	uintptr_t end_addr = start_addr + size;

	if (op != K_CACHE_INVD && op != K_CACHE_WB && op != K_CACHE_WB_INVD) {
		return -ENOTSUP;
	}

	line_size = arch_dcache_line_size_get();

	/*
	 * For the data cache invalidate operation, clean and invalidate
	 * the partial cache lines at both ends of the given range to
	 * prevent data corruption.
	 *
	 * For example (assume cache line size is 64 bytes):
	 * There are 2 consecutive 32-byte buffers, which can be cached in
	 * one line like below.
	 *			+------------------+------------------+
	 *	 Cache line:	| buffer 0 (dirty) |     buffer 1     |
	 *			+------------------+------------------+
	 * For the start address not aligned case, when invalidate the
	 * buffer 1, the full cache line will be invalidated, if the buffer
	 * 0 is dirty, its data will be lost.
	 * The same logic applies to the not aligned end address.
	 */
	if (op == K_CACHE_INVD) {
		if (end_addr & (line_size - 1)) {
			end_addr &= ~(line_size - 1);
			dc_ops("civac", end_addr);
		}

		if (start_addr & (line_size - 1)) {
			start_addr &= ~(line_size - 1);
			if (start_addr == end_addr) {
				goto done;
			}
			dc_ops("civac", start_addr);
			start_addr += line_size;
		}
	}

	/* Align address to line size */
	start_addr &= ~(line_size - 1);

	while (start_addr < end_addr) {
		if (op == K_CACHE_INVD) {
			dc_ops("ivac", start_addr);
		} else if (op == K_CACHE_WB) {
			dc_ops("cvac", start_addr);
		} else if (op == K_CACHE_WB_INVD) {
			dc_ops("civac", start_addr);
		}

		start_addr += line_size;
	}

done:
	dsb();

	return 0;
}
