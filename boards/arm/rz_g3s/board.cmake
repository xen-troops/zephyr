# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=R7S921053VCBG" "--speed=15000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

set(RELEASE_NOTES_SRC index.rst)
set(RELEASE_NOTES_OUT rzg3s_release_notes.pdf)
add_custom_target(doc
    WORKING_DIRECTORY ${ZEPHYR_BASE}/boards/${ARCH}/${BOARD}/doc
    COMMAND rst2pdf ${RELEASE_NOTES_SRC} -o ${ZEPHYR_BINARY_DIR}/${RELEASE_NOTES_OUT}
)
