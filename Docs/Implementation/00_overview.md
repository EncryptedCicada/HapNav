# HapNav — System Overview

HapNav is a wearable proximity-cueing system for blind / low-vision users. A
chest-mounted "pin" senses the environment and a wristband converts that
data into directional haptic feedback. The two units talk over BLE.

```
┌────────────────────────────┐    BLE GATT    ┌──────────────────────┐
│  CHEST PIN                 │ Write-No-Resp  │  WRISTBAND           │
│  XIAO ESP32-S3             │ ─────────────▶ │  XIAO nRF52840       │
│                            │  219-byte frame│                      │
│  • LSM6DSO IMU             │     @10 Hz     │  • PCA9546A I2C mux  │
│  • LIS2MDL mag             │                │  • 4× DA7280 + LRA   │
│  • VL53L5CX 8×8 ToF        │                │  • TPS62827 buck on  │
│      (45°-rolled, −8° tilt)│                │    BQ25185 power-path│
│  • VL53L1X head ToF        │                │  • Drive policy 20 Hz│
│      (+20° world tilt)     │                │  • Selftest harness  │
│  • Madgwick AHRS           │                │                      │
│  • Obstacle + drop-off +   │                │                      │
│    head-clearance pipeline │                │                      │
└────────────────────────────┘                └──────────────────────┘
```

## Documentation map

| File | Subsystem |
|------|-----------|
| [00_system_overview.yaml](00_system_overview.yaml) | Whole-system summary (machine-readable) |
| [01_chest_pin.md](01_chest_pin.md)                 | Pin firmware: sensors + main loop |
| [02_wristband.md](02_wristband.md)                 | Wristband firmware: bring-up + threading |
| [02_wristband.yaml](02_wristband.yaml)             | Wristband + selftest detail (machine-readable) |
| [03_ble_protocol.md](03_ble_protocol.md)           | GATT service, frame layout, MTU |
| [04_sensor_fusion.md](04_sensor_fusion.md)         | Madgwick AHRS configuration |
| [05_obstacle_detection.md](05_obstacle_detection.md) | ToF → urgency pipeline |
| [06_dropoff_detection.md](06_dropoff_detection.md) | Negative-space cliff detection |
| [07_haptic_policy.md](07_haptic_policy.md)         | Wristband: urgency → LRA drive |
| [08_power_and_assembly.md](08_power_and_assembly.md) | Wristband hardware: BQ25185 → buck → mux → drivers |
| [09_head_clearance.md](09_head_clearance.md)         | VL53L1X head sensor + future down-facing height calibration |

## Geometry

| Quantity | Value |
|----------|-------|
| User height (assumed) | 175 cm |
| Sensor mount height | 1.35 m |
| Sensor downtilt | 8° |
| ToF FoV (per axis) | 44.5° |
| ToF grid | 8 × 8 (64 zones) |
| Operating range | 0.3 – 3.0 m |
| Walking-speed prior | 0.7 m/s |

## Coordinate frames

- **Sensor frame:** `+X` right, `+Y` up, `+Z` forward (optical axis)
- **Body frame:** `+X` right, `+Y` back, `+Z` up (`-Y` is user forward)
- **World frame:** Gravity-aligned, `Z` = up

## Toolchain

- Zephyr 4.4.99, west / CMake
- Pin: `xtensa-espressif_esp32s3_zephyr-elf`
- Wristband: `arm-zephyr-eabi`
- Provided via `nix develop ~/nixos-dotfiles#zephyr` (sets `ZEPHYR_BASE`,
  provides `west`)
