# SPDX-License-Identifier: Apache-2.0

if (CONFIG_BOARD_RZ_G3S_FPU)
	board_runner_args(jlink "--device=R9A08G045S33_M33_1" "--speed=15000")
else ()
	board_runner_args(jlink "--device=R9A08G045S33_M33_0" "--speed=15000")
endif ()
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

set(RELEASE_NOTES_SRC .)
set(RELEASE_NOTES_OUT rzg3s_release_notes.pdf)

set(KERNEL_SREC_NAME   ${KERNEL_NAME}.srec)

add_custom_target(doc
    WORKING_DIRECTORY ${ZEPHYR_BASE}/boards/${ARCH}/${BOARD}/doc
    COMMAND sphinx-build -M latexpdf ${RELEASE_NOTES_SRC} ${ZEPHYR_BINARY_DIR}
    COMMAND cp ${ZEPHYR_BINARY_DIR}/latex/rzg3s_release_notes.pdf  ${ZEPHYR_BINARY_DIR}/${RELEASE_NOTES_OUT}
    COMMAND rm -Rf ${ZEPHYR_BINARY_DIR}/latex
    COMMAND rm -Rf ${ZEPHYR_BINARY_DIR}/doctrees
)

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_OBJCOPY} -O srec --srec-forceS3 ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME} ${ZEPHYR_BINARY_DIR}/${KERNEL_SREC_NAME}
)
