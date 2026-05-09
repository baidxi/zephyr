# Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=V3S")
board_runner_args(openocd)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
