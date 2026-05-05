cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(hapnav_pin VERSION 0.1.0)

target_sources(app PRIVATE
    src/main.c
    src/ble.c
    src/sensors.c
)

# Pull in shared sources/headers (each subdirectory attaches itself to `app`).
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/ble     ${CMAKE_BINARY_DIR}/_lib_ble)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../lib/sensors ${CMAKE_BINARY_DIR}/_lib_sensors)
