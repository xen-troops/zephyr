# Copyright (c) 2023 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(optee.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/arch/${ARCH}/include)
