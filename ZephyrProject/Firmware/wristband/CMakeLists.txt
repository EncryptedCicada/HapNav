cmake_minimum_required(VERSION 3.20.0)

# Register the out-of-tree DA7280 haptic driver as a Zephyr module before
# find_package() so its DT bindings, Kconfig, and CMakeLists are picked up.
list(APPEND ZEPHYR_EXTRA_MODULES
    "${CMAKE_CURRENT_LIST_DIR}/../modules/da7280-haptic"
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(hapnav_wristband VERSION 0.1.0)

target_sources(app PRIVATE
    src/main.c
    src/ble.c
)

# Shared BLE protocol header (attaches itself to `app`).
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/ble ${CMAKE_BINARY_DIR}/_lib_ble)

# High-level haptic policy + selftest harness (talks to the DA7280 driver
# which is provided by the modules/da7280-haptic Zephyr module above).
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/haptics ${CMAKE_BINARY_DIR}/_lib_haptics)
