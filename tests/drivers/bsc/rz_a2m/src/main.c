/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#define TEST_MEM_VALUE 0xdeadbeef

ZTEST(bsc_rza2_suite, test_mem_set)
{
	/* External SDRAM connected to the subboard should be initialized on start */
	long *p = (long *)0x0C000000;
	*p = TEST_MEM_VALUE;
	zassert_equal(*p, TEST_MEM_VALUE, "Unable to set value to the external SDRAM address");
}

ZTEST_SUITE(bsc_rza2_suite, NULL, NULL, NULL, NULL, NULL);
