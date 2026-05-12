# Copyright (c) 2026 (Your Company)
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_HAPTIC_DA7280)
  zephyr_library()
  zephyr_library_sources(da7280.c)
  zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../include)
endif()
