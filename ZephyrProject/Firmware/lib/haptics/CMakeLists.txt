# SPDX-License-Identifier: Apache-2.0

target_sources(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/haptics.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# The DA7280 driver itself lives in modules/da7280-haptic and is enabled by
# CONFIG_HAPTIC_DA7280; the mux comes from the in-tree CONFIG_I2C_TCA954X.
# Both are pulled in by prj.conf.
