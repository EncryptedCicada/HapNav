# Chest Pin

Seeed XIAO ESP32-S3. Hosts the sensors, runs fusion + obstacle detection,
streams the result over BLE as the central / GATT client.

## Hardware

| Component | Bus | Address |
|-----------|-----|---------|
| LSM6DSO 6-axis IMU | I2C | 0x6B |
| LIS2MDL magnetometer | I2C | 0x1E |
| VL53L5CX 8×8 ToF array | I2C | 0x29 (7-bit) / 0x52 (8-bit) |
| 2× VL53L1X aux ToF (planned) | I2C | XSHUT-shifted at boot |

All on `i2c0` (GPIO5 = SDA, GPIO6 = SCL, 400 kHz). Magnetometer X-axis is
mounted opposite the IMU; firmware flips `mag_raw[0]` before fusion.

## Firmware layout

```
Firmware/pin/
├── prj.conf
├── boards/xiao_esp32s3.overlay   # i2c0 enabled
├── CMakeLists.txt
└── src/
    ├── main.c       # 10 Hz sample loop
    ├── sensors.c/h  # init + sample → hapnav_frame
    └── ble.c/h      # central role; finds wristband, writes frame
```

Shared sensor glue lives at `Firmware/lib/sensors/` (Zephyr wrappers around
the STMicro platform-independent `*_reg.c` drivers + Madgwick + the
obstacle / drop-off modules).

## Sample loop

1. `hapnav_lsm6dso_read_raw()` → accel + gyro int16
2. `hapnav_lis2mdl_read_raw()` → mag int16 (X negated)
3. `madgwick_update()` with engineering-unit conversions baked in
4. `hapnav_vl53l5cx_poll()` — non-blocking; latches a 8×8 frame when ready
5. `hapnav_obstacle_step()` → fills `hapnav_obstacles` block
6. BLE write of the full 219-byte frame to the wristband

Period is `SAMPLE_PERIOD_MS = 100` (10 Hz) — fast enough for walking-speed
proximity, slow enough that radio + ToF stays comfortably within budget.

## Build

```sh
cd ZephyrProject/Firmware/pin
west build -b xiao_esp32s3
west flash
```
