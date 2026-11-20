# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

HapNav is a wearable haptic navigation aid for visually impaired users. A **chest pin** senses obstacles with Time-of-Flight (ToF) sensors and streams frames over BLE to a **wristband**, which drives 4 LRA vibration motors. This is a hardware project: firmware changes can only be fully verified on the physical boards.

**Docs drift warning:** the top-level `README.md`, `ZephyrProject/Firmware/README.md` and parts of `Docs/Implementation/` describe an older design (STM32, LSM6DSO + LIS2MDL + Madgwick, 20 Hz/10 Hz, `Implementation_CodeBase/Firmware` paths). The code is the source of truth:
- Pin = **Seeed XIAO ESP32-S3**; wristband = **Seeed XIAO nRF52840** (`xiao_ble`).
- Orientation comes from a **BNO055** (onboard NDOF fusion → quaternion), not LSM6DSO/LIS2MDL.
- Pin loop runs at **~83 Hz** (`SAMPLE_PERIOD_MS 12`, defined in both `pin/src/main.c` and `pin/src/sensors.c`; keep them in sync).
- Firmware lives at `ZephyrProject/Firmware/`.

## Build & flash (Zephyr, pinned to v3.7.0 in `ZephyrProject/Firmware/west.yml`)

The author uses a Nix shell that sets `ZEPHYR_BASE`: `nix develop ~/nixos-dotfiles#zephyr`. Otherwise, from `ZephyrProject/Firmware/`, run `west init -l . && west update`.

```zsh
# Chest pin (ESP32-S3)
cd ZephyrProject/Firmware/pin
west build -p always -b xiao_esp32s3/esp32s3/procpu .   # older Zephyr: -b xiao_esp32s3
west flash

# Scripted demo pin build (no real sensors needed): layers prj_demo.conf
west build -p always -b xiao_esp32s3/esp32s3/procpu . -- -DEXTRA_CONF_FILE=prj_demo.conf

# Laptop-link pin build: pin streams to a laptop over BLE instead of the wristband
# (overlays can be combined: -DEXTRA_CONF_FILE="prj_laptop.conf;prj_demo.conf")
west build -p always -b xiao_esp32s3/esp32s3/procpu . -- -DEXTRA_CONF_FILE=prj_laptop.conf

# Wristband (nRF52840)
cd ZephyrProject/Firmware/wristband
west build -p always -b xiao_ble .
west flash
```

Board overlays: `pin/boards/xiao_esp32s3_procpu.overlay`, `wristband/boards/xiao_ble.overlay`. Kconfig options can be overridden per-build with `-- -DCONFIG_FOO=y`.

Sensor driver submodules (`ZephyrProject/Drivers/{stm32-vl53l5cx,lis2mdl-pid,lsm6dso-pid}`) must be checked out: `git submodule update --init --recursive`. The pin build compiles the VL53L5CX ULD from `Drivers/stm32-vl53l5cx/modules/`. It is paired with our own `lib/sensors/vendor/vl53l5cx/platform.{c,h}`, which must stay first on the include path.

## Firmware architecture

Two Zephyr apps share code under `ZephyrProject/Firmware/lib/`. Each lib's `CMakeLists.txt` attaches its sources directly to the `app` target, so there is no separate Zephyr library. Each app's `Kconfig` `rsource`s the lib Kconfig it needs.

- `lib/ble/include/hapnav/ble_proto.h`: **the wire contract.** It defines one GATT service with one write-without-response characteristic carrying a packed 219-byte `struct hapnav_frame`: timestamp, quaternion, 8×8 distances + status, and a 7-byte `hapnav_obstacles` (4 urgency bytes, nearest range, flags). If you change it, rebuild **both** boards. Both sides set the ATT MTU to 247 so a frame fits in one PDU.
- `lib/sensors/` (pin only): `vl53l5cx.c` (ULD wrapper), `bno055.c` (register-level I²C, no in-tree driver), `obstacle.c` (ToF grid → 4 azimuth-bin urgencies, habituation, stationary/yaw-slew/blocked flags), `dropoff.c` (down-ToF cliff detection).
- `lib/haptics/` (wristband only): `haptics.c` is the drive policy. A 20 Hz work-queue tick maps urgency → amplitude, with a 250 ms watchdog that mutes on stale frames, plus fatigue taper, urgency "knocks" and override patterns for DROPOFF/HEAD flags. It also contains the `CONFIG_HAPNAV_SELFTEST` bench-test sequence, which disables BLE-driven reactions.
- `modules/da7280-haptic/`: out-of-tree Zephyr module (DA7280 LRA driver + DT binding), registered via `ZEPHYR_EXTRA_MODULES` in `wristband/CMakeLists.txt`. Kconfig `HAPTIC_DA7280_TOTAL_DRIVE_CAP` is a hardware-safety cap. (`ZephyrProject/Drivers/da7280/da7280.c` is a stray copy and is not built.)
- **Pin flow:** `main.c` loop → `pin_sensors_sample()` (`pin/src/sensors.c`) fills a frame and runs the pipeline → `pin_ble_send_frame()`. The pin is the BLE **central**: it scans for the wristband's service UUID, connects, and discovers the characteristic.
- **Laptop link (`CONFIG_HAPNAV_LINK_LAPTOP`, `pin/prj_laptop.conf`):** `pin/CMakeLists.txt` compiles `src/ble_laptop.c` instead of `src/ble.c`. Both implement the `pin_ble_*` API in `src/ble.h`. In this mode the pin is a BLE **peripheral** advertising `HapNav-Pin` with its own UUIDs (`HAPNAV_UUID_LAPTOP_*`). Frames go out as notifications, split into chunks with a 1-byte `seq<<4|idx` header so they fit any MTU; `ble_proto.h` documents the format. `Tools/ble_receiver.py` (needs `pip install bleak`) reassembles the chunks, prints frames, and logs them to `Logs/ble_*.jsonl`.
- **Wristband flow:** it is the BLE **peripheral** (advertises `HapNav-Wrist`). `wristband/src/ble.c` `frame_write` → `hapnav_haptics_consume_frame()` → the haptics worker drives the motors.

### I²C topology
- Pin: everything sits behind a TCA9548A mux @0x70 on i2c0 (D2/D3): ch0 head VL53L1X, ch1 front VL53L5CX, ch2 down VL53L1X (all @0x29), ch3 BNO055 @0x28. Zephyr's `I2C_TCA954X` exposes each channel as a virtual bus.
- Wristband: a PCA9546A mux (same TCA954X driver) with 4× DA7280.

### Pin Kconfig modes (`lib/sensors/Kconfig`; defaults in `pin/prj.conf`)
- `HAPNAV_BENCH_MODE`: rescales geometry constants for a pin standing on a desk (10–100 cm range). It is currently **=y** in `prj.conf`; set it to n for the worn build.
- `HAPNAV_HEAD_TOF`, `HAPNAV_DOWN_TOF`, `HAPNAV_DROPOFF`: enable the extra sensors and the cliff detector (currently off for bench).
- `HAPNAV_DEMO_MODE`: replaces sensor reads with a looping scripted scenario. The pipeline, BLE and haptics still run normally.
- `HAPNAV_HABITUATION`: fades alerts for obstacles that stay stationary relative to the pin.

## Serial output & logging

- The pin prints `PIN {...}` trace lines at ~83 Hz, with the full 8×8 grid about once per second. The wristband prints `RX {...}` per received frame. These are human-oriented and not strict JSON (e.g. `"head":123 mm`).
- `Tools/serial_logger.py` reads both boards concurrently, host-timestamps and interleaves their lines, and writes to `Logs/` (gitignored): `python Tools/serial_logger.py --wrist /dev/ttyACM0 --pin /dev/ttyACM1`.

## Visualizer (Python, host-side demo tool)

`Visualizer/` is a 3D point-cloud viewer built on viser at http://localhost:8080. It expects bare JSON lines starting with `{` containing `distances`, `status`, `quat`, and optionally `head_mm`/`down_mm`/obstacle flags. That is the format of the Arduino sketch `Visualizer/firmware/vl53l5cx_reader/`, **not** the Zephyr pin's `PIN {...}` trace.

```bash
cd Visualizer
pip install -r viewer/requirements.txt
python -m viewer --port /dev/cu.usbmodemXXXX [--baud 115200] [--debug]
python -m pytest viewer/tests                                   # all tests
python -m pytest viewer/tests/test_geometry.py -k <name>        # single test
```

## Other directories
- `Docs/Implementation/`: per-subsystem design notes (obstacle pipeline, drop-off, haptic policy, power). They are useful for intent, but check details against the code.
- `Reference_CodeBase/`: standalone sensor examples. `CAD/`: FreeCAD enclosures.
