# Copyright (c) 2020 Linumiz
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LZ4)

  set(LZ4_DIR ${ZEPHYR_CURRENT_MODULE_DIR})

  zephyr_library()

  zephyr_include_directories(${LZ4_DIR}/lib)

  zephyr_library_sources(
    ${LZ4_DIR}/lib/lz4.c
    ${LZ4_DIR}/lib/lz4frame.c
    ${LZ4_DIR}/lib/xxhash.c
  )

endif()
