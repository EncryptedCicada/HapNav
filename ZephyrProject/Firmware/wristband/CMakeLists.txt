cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(hapnav_wristband VERSION 0.1.0)

target_sources(app PRIVATE
    src/main.c
    src/ble.c
)

# Shared BLE protocol header (attaches itself to `app`).
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/ble ${CMAKE_BINARY_DIR}/_lib_ble)

# Haptic stack: PCA9546A mux + 4× DA7280 + drive policy.
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/haptics ${CMAKE_BINARY_DIR}/_lib_haptics)
