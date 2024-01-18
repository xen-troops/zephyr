# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=R7S921053VCBG" "--speed=15000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

set(RELEASE_NOTES_SRC rza2m_release_notes.tex)
add_custom_target(doc
    WORKING_DIRECTORY ${ZEPHYR_BASE}/boards/${ARCH}/${BOARD}/release_notes
    COMMAND pdflatex -output-directory=${ZEPHYR_BINARY_DIR} ${RELEASE_NOTES_SRC}
)
