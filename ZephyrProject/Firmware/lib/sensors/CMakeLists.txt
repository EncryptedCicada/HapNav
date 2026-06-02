# Vendor STMicroelectronics platform-independent drivers live outside the
# Firmware tree at Implementation_CodeBase/Drivers/.
set(STDRV_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers)

# VL53L5CX ULD lives inside the stm32-vl53l5cx submodule. We use only the
# `modules/` subtree (the platform-independent Ultra Light Driver) plus our
# own platform.h/.c replacement under vendor/vl53l5cx/.
set(VL53L5CX_ULD_DIR ${STDRV_DIR}/stm32-vl53l5cx/modules)
set(VL53L5CX_PORT_DIR ${CMAKE_CURRENT_LIST_DIR}/vendor/vl53l5cx)

# We attach to the app target rather than creating a separate Zephyr library
# so the sources pick up Zephyr's autoconf, kernel headers, and compile flags.
target_sources(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/bno055.c
    ${CMAKE_CURRENT_LIST_DIR}/src/vl53l5cx.c
    ${CMAKE_CURRENT_LIST_DIR}/src/obstacle.c
    ${CMAKE_CURRENT_LIST_DIR}/src/dropoff.c
    ${VL53L5CX_ULD_DIR}/vl53l5cx_api.c
    ${VL53L5CX_PORT_DIR}/platform.c
)

# Order matters: our platform.h replacement must be found before any header
# that ships in the upstream submodule.
target_include_directories(app PRIVATE
    ${VL53L5CX_PORT_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${VL53L5CX_ULD_DIR}
)
