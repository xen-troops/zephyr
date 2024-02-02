# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SOC_SERIES_RZ_G_ARM_FPU)
	board_runner_args(jlink "--device=R9A08G045S33_M33_1" "--speed=15000")
elseif(CONFIG_SOC_SERIES_RZ_G_ARM)
	board_runner_args(jlink "--device=R9A08G045S33_M33_0" "--speed=15000")
endif ()
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

set(RELEASE_NOTES_SRC index.rst)
set(RELEASE_NOTES_OUT rzg3s_release_notes.pdf)

add_custom_target(doc
    WORKING_DIRECTORY ${ZEPHYR_BASE}/boards/${ARCH}/${BOARD}/doc
    COMMAND rst2pdf ${RELEASE_NOTES_SRC} -o ${ZEPHYR_BINARY_DIR}/${RELEASE_NOTES_OUT}
)