# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(base.c common.c)

zephyr_library_sources_ifdef(CONFIG_ARM_SCMI_TRANSPORT_SMC driver_smc.c shmem.c)

zephyr_library_sources_ifdef(CONFIG_ARM_SCMI_SHELL shell.c)
