/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * uImage helper structures and defines
 */
#ifndef __XEN_DOM0_UIMAGE_H__
#define __XEN_DOM0_UIMAGE_H__

#include <zephyr/kernel.h>

#define UIMAGE_MAGIC 0x27051956
#define UIMAGE_NMLEN 32

struct uimage_hdr {
	uint32_t magic_be32;
	uint32_t hcrc_be32;
	uint32_t time_be32;
	uint32_t size_be32;
	uint32_t load_be32;
	uint32_t ep_be32;
	uint32_t dcrc_be32;
	uint8_t os;
	uint8_t arch;
	uint8_t type;
	uint8_t comp;
	uint8_t name[UIMAGE_NMLEN];
};

#endif /* __XEN_DOM0_UIMAGE_H__ */
