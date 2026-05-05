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
    ${CMAKE_CURRENT_LIST_DIR}/src/lsm6dso.c
    ${CMAKE_CURRENT_LIST_DIR}/src/lis2mdl.c
    ${CMAKE_CURRENT_LIST_DIR}/src/vl53l5cx.c
    ${CMAKE_CURRENT_LIST_DIR}/src/madgwick.c
    ${STDRV_DIR}/lsm6dso-pid/lsm6dso_reg.c
    ${STDRV_DIR}/lis2mdl-pid/lis2mdl_reg.c
    ${VL53L5CX_ULD_DIR}/vl53l5cx_api.c
    ${VL53L5CX_PORT_DIR}/platform.c
)

# Order matters: our platform.h replacement must be found before any header
# that ships in the upstream submodule.
target_include_directories(app PRIVATE
    ${VL53L5CX_PORT_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${STDRV_DIR}/lsm6dso-pid
    ${STDRV_DIR}/lis2mdl-pid
    ${VL53L5CX_ULD_DIR}
)
