# Chest Pin

Seeed XIAO ESP32-S3. Hosts the sensors, runs fusion + obstacle detection,
streams the result over BLE as the central / GATT client.

## Hardware

| Component | Bus | Address | Notes |
|-----------|-----|---------|-------|
| LSM6DSO 6-axis IMU | i2c0 | 0x6B | SparkFun Qwiic |
| LIS2MDL magnetometer | i2c0 | 0x1E | X axis mounted opposite IMU |
| VL53L5CX 8×8 ToF array | i2c0 | **0x29 → 0x2A at boot** | rolled 45° about optical axis |
| VL53L1X head ToF | i2c0 | 0x29 (after VL53L5CX moves) | mounted at +20° world tilt |
| VL53L1X height ToF | i2c0 | _planned_ | for live mount-height calibration; see [09_head_clearance.md](09_head_clearance.md) |

All on `i2c0`, remapped from the board-default D4/D5 to **D2 = SDA /
GPIO0_3** and **D3 = SCL / GPIO0_4** in the overlay's pinctrl block
(the original D4/D5 pads were dead on the current prototype). 400 kHz.
Both ToF sensors default to I²C address `0x29`; the firmware uses the
VL53L1X's `XSHUT` pin (**D7 = `&gpio1 12`**) to keep it offline while
the VL53L5CX is moved to `0x2A`, then releases `XSHUT` so the in-tree
Zephyr `st,vl53l1x` driver can probe at `0x29` unobstructed. See
[09_head_clearance.md](09_head_clearance.md) for the full dance.

## Firmware layout

```
Firmware/pin/
├── prj.conf                        # CONFIG_VL53L1X, CONFIG_SENSOR, …
├── boards/xiao_esp32s3.overlay     # i2c0 + tof_head node + xshut alias
├── CMakeLists.txt
└── src/
    ├── main.c       # 10 Hz sample loop
    ├── sensors.c/h  # init + sample → hapnav_frame
    └── ble.c/h      # central role; finds wristband, writes frame
```

Shared sensor glue lives at `Firmware/lib/sensors/` (Zephyr wrappers
around the STMicro platform-independent `*_reg.c` drivers + Madgwick +
the obstacle / drop-off modules).

## Sample loop

1. `hapnav_lsm6dso_read_raw()` → accel + gyro int16
2. `hapnav_lis2mdl_read_raw()` → mag int16 (X negated)
3. `madgwick_update()` with engineering-unit conversions baked in
4. `hapnav_vl53l5cx_poll()` — non-blocking; latches an 8×8 frame when ready
5. `sensor_sample_fetch()` + `sensor_channel_get()` on the VL53L1X — head
   distance in mm, or −1 when unreliable / out of range
6. `hapnav_obstacle_step()` → fills `hapnav_obstacles` block. Applies the
   45° optical-axis roll + 8° downtilt to the VL53L5CX rays, classifies
   floor/ceil/obstacle in world frame, computes per-bin urgency, runs
   drop-off + head-clearance checks
7. BLE write of the full 219-byte frame to the wristband

Period is `SAMPLE_PERIOD_MS = 100` (10 Hz) — fast enough for walking-speed
proximity, slow enough that radio + ToF stays comfortably within budget.

## Bench-demo mode

`CONFIG_HAPNAV_BENCH_MODE=y` (in `prj.conf`) rescales the obstacle /
drop-off / head-clearance constants for a desktop demo where the pin
sits 2–4 cm above the bench surface and objects move within a
10–50 cm envelope. Pair it with `CONFIG_HAPNAV_HEAD_TOF=n` to skip the
VL53L1X entirely — the VL53L5CX's ±22.3° cone is wide enough at bench
scale to cover the working volume on its own (a card waved above the
desk-mounted pin will register on the grid's top-row pixels and pulse
all four LRAs, an acceptable head-clearance proxy for the demo).

| Constant | Real-world | Bench |
|---|---|---|
| `SENSOR_HEIGHT_M`           | 1.35 m | 0.04 m |
| `FLOOR_BAND_HIGH_M`         | −1.20 m | −0.03 m |
| `HEAD_TOP_BAND_M`           | 0.70 m | 0.10 m |
| `MIN_RANGE_MM`              | 300    | 50 |
| `MAX_RANGE_MM`              | 3000   | 500 |
| `CLUSTER_DELTA_MM`          | 200    | 30 |
| `HEAD_PROXIMITY_MAX_MM`     | 2200   | 150 |
| dropoff predicted-range cap | 3.5 m  | 0.5 m |
| stationary-detector threshold | (live) | effectively disabled |

The boot log calls out which mode the binary was built in
(`Obstacle pipeline ready [BENCH MODE] …`) so a UART capture is
unambiguous. Flip back to `=n` for the worn build.

## Build

```sh
cd ZephyrProject/Firmware/pin
west build -b xiao_esp32s3/esp32s3/procpu
west flash
```
