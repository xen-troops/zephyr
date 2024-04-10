/* SPDX-License-Identifier: MIT */
/******************************************************************************
 * version.h
 *
 * Xen version, type, and compile information.
 *
 * Copyright (c) 2005, Nguyen Anh Quynh <aquynh@gmail.com>
 * Copyright (c) 2005, Keir Fraser <keir@xensource.com>
 */

#ifndef __XEN_PUBLIC_VERSION_H__
#define __XEN_PUBLIC_VERSION_H__

#include "xen.h"

/* NB. All ops return zero on success, except XENVER_{version,pagesize}
 * XENVER_{version,pagesize,build_id}
 */

/* arg == NULL; returns major:minor (16:16). */
#define XENVER_version      0

/* arg == xen_extraversion_t. */
#define XENVER_extraversion 1
typedef char xen_extraversion_t[16];
#define XEN_EXTRAVERSION_LEN (sizeof(xen_extraversion_t))

/* arg == xen_compile_info_t. */
#define XENVER_compile_info 2
struct xen_compile_info {
	char compiler[64];
	char compile_by[16];
	char compile_domain[32];
	char compile_date[32];
};
typedef struct xen_compile_info xen_compile_info_t;

#define XENVER_capabilities 3
typedef char xen_capabilities_info_t[1024];
#define XEN_CAPABILITIES_INFO_LEN (sizeof(xen_capabilities_info_t))

#define XENVER_changeset 4
typedef char xen_changeset_info_t[64];
#define XEN_CHANGESET_INFO_LEN (sizeof(xen_changeset_info_t))

/*
 * This API is problematic.
 *
 * It is only applicable to guests which share pagetables with Xen (x86 PV
 * guests), but unfortunately has leaked into other guest types and
 * architectures with an expectation of never failing.
 *
 * It is intended to identify the virtual address split between guest kernel
 * and Xen.
 *
 * For 32bit PV guests, there is a split, and it is variable (between two
 * fixed bounds), and this boundary is reported to guests.  The detail missing
 * from the hypercall is that the second boundary is the 32bit architectural
 * boundary at 4G.
 *
 * For 64bit PV guests, Xen lives at the bottom of the upper canonical range.
 * This hypercall happens to report the architectural boundary, not the one
 * which would be necessary to make a variable split work.  As such, this
 * hypercall entirely useless for 64bit PV guests, and all inspected
 * implementations at the time of writing were found to have compile time
 * expectations about the split.
 *
 * For architectures where this hypercall is implemented, for backwards
 * compatibility with the expectation of the hypercall never failing Xen will
 * return 0 instead of failing with -ENOSYS in cases where the guest should
 * not be making the hypercall.
 */
#define XENVER_platform_parameters 5
struct xen_platform_parameters {
	xen_ulong_t virt_start;
};
typedef struct xen_platform_parameters xen_platform_parameters_t;

#define XENVER_get_features 6
struct xen_feature_info {
	uint32_t     submap_idx;    /* IN: which 32-bit submap to return */
	uint32_t     submap;        /* OUT: 32-bit submap */
};
typedef struct xen_feature_info xen_feature_info_t;

/* Declares the features reported by XENVER_get_features. */
#include "features.h"

/* arg == NULL; returns host memory page size. */
#define XENVER_pagesize 7

/* arg == xen_domain_handle_t.
 *
 * The toolstack fills it out for guest consumption. It is intended to hold
 * the UUID of the guest.
 */
#define XENVER_guest_handle 8

#define XENVER_commandline 9
typedef char xen_commandline_t[1024];

/*
 * Return value is the number of bytes written, or XEN_Exx on error.
 * Calling with empty parameter returns the size of build_id.
 */
#define XENVER_build_id 10
struct xen_build_id {
	/* IN: size of buf[]. */
	uint32_t        len;
	/* OUT: Variable length buffer with build_id. */
	unsigned char   buf[XEN_FLEX_ARRAY_DIM];
};
typedef struct xen_build_id xen_build_id_t;

#endif /* __XEN_PUBLIC_VERSION_H__ */
