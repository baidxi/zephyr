# Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MUSB)
  zephyr_library()

  zephyr_include_directories(.)

  # glue (strong musb_platform_info_get) must come before
  # core (weak fallback) so the linker picks the strong symbol
  # from the .a archive.
  add_subdirectory(glue)

  zephyr_library_sources(musb_core.c)
endif()
