#
# Copyright (c) 2026 EPAM Systems
#
# SPDX-License-Identifier: Apache-2.0
#

if (CONFIG_OPTEE_CLIENT)

  zephyr_library()

  zephyr_library_sources(opteelib.c)
  zephyr_library_sources(teec_trace.c)
  zephyr_library_sources(ree_fs.c)
  zephyr_library_sources(tee_supplicant.c)

  zephyr_library_include_directories(
      ${ZEPHYR_OPTEELIB_MODULE_DIR}/libteec/include
      ${ZEPHYR_OPTEELIB_MODULE_DIR}/libckteec/include
      ${ZEPHYR_OPTEELIB_MODULE_DIR}/libseteec/include
      ${ZEPHYR_OPTEELIB_MODULE_DIR}/tee_supplicant/include
      ${ZEPHYR_OPTEELIB_MODULE_DIR}/public
    )

  add_subdirectory_ifdef(CONFIG_OPTEE_LIBTEEC ${ZEPHYR_OPTEELIB_MODULE_DIR}/libteec libteec_build)
  add_subdirectory_ifdef(CONFIG_OPTEE_LIBCKTEEC ${ZEPHYR_OPTEELIB_MODULE_DIR}/libckteec libckteec_build)
  add_subdirectory_ifdef(CONFIG_OPTEE_TEE_SUPPLICANT ${ZEPHYR_OPTEELIB_MODULE_DIR}/tee_supplicant tee_supplicant_build)
  add_subdirectory_ifdef(CONFIG_OPTEE_LIBSETEEC ${ZEPHYR_OPTEELIB_MODULE_DIR}/libseteec libseteec_build)

endif()