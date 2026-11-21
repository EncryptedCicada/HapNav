# HapNav work notes: 2026-11-20

Chest pin → laptop BLE link, toolchain setup, and Visualizer BLE support.

## What we achieved

1. **Rebuilt the firmware toolchain** on macOS and worked out how to compile and flash both boards.
2. **The chest pin streams to the laptop over BLE.** The original pin → wristband mode is still the default build.
3. **Fixed three problems:** a Zephyr 4.x compile error, advertising that never started, and missing ToF data.
4. **Laptop tools:** a command-line receiver/logger, plus live BLE viewing and logging in the 3D Visualizer.

## Committed: `f451b93` on `Eric_0603`, "Add laptop BLE link mode for the chest pin"

| File | Change |
|---|---|
| `ZephyrProject/Firmware/pin/src/ble_laptop.c` (new) | Pin acts as a BLE peripheral advertising `HapNav-Pin`. It sends each 219-byte frame as notifications, split into labelled pieces to fit any packet size (ATT MTU), and restarts advertising after a disconnect. Works on Zephyr 3.7 and 4.x |
| `ZephyrProject/Firmware/pin/prj_laptop.conf` (new) | Turns laptop mode on (`-DEXTRA_CONF_FILE=prj_laptop.conf`) |
| `ZephyrProject/Firmware/pin/Kconfig` | New `CONFIG_HAPNAV_LINK_LAPTOP` option, off by default |
| `ZephyrProject/Firmware/pin/CMakeLists.txt` | Builds `ble_laptop.c` or the original `ble.c`, depending on the option |
| `ZephyrProject/Firmware/lib/ble/include/hapnav/ble_proto.h` | Separate UUIDs for the laptop link, plus documentation of the piece-label format |
| `ZephyrProject/Firmware/pin/src/main.c` | **Starts Bluetooth before the sensors**, so the radio comes up even if I²C stalls. This also affects the wristband build |
| `Tools/ble_receiver.py` (new) | Laptop CLI: finds the pin, reassembles and prints frames, logs to `Logs/ble_*.jsonl` |
| `CLAUDE.md` (new) | Project guide for Claude Code: build commands, architecture, laptop mode, and notes on where the docs are out of date |

## Not committed yet (as of this note)

| File | Change | Tested? |
|---|---|---|
| `ZephyrProject/Firmware/pin/src/sensors.c` | Probes the I²C mux first; if the bus is stuck, tries `i2c_recover_bus()` and fails within ~1 s instead of a 2-minute scan | Compiles, but **not tested on the board** |
| `Visualizer/viewer/ble_reader.py` (new) | `BleReader`, a BLE data source with the same interface as `SerialReader`, plus optional JSONL logging | 10 unit tests ✅, works live ✅ |
| `Visualizer/viewer/viewer.py` | New options `--ble`, `--ble-name`, `--log`, `--log-dir` | ✅ |
| `Visualizer/viewer/requirements.txt` | Adds `bleak` (optional) | n/a |
| `Visualizer/viewer/tests/test_ble_reader.py` (new) | Tests for decoding, reassembly, lost pieces, reader state and logging | ✅ |

## Problems solved

| Symptom | Cause | Fix |
|---|---|---|
| `west: command not found` | No Zephyr toolchain on this Mac | Set up `~/zephyrproject` with a venv, Zephyr and the SDK |
| `BT_LE_ADV_CONN undeclared` | Local Zephyr is 4.4; the project was written for 3.7 | Use `BT_LE_ADV_CONN_FAST_2` when available, and restart advertising from the `recycled` callback |
| Pin not advertising | Startup froze on the I²C scan before Bluetooth started | Start BLE first |
| No ToF data after flashing | A USB-only reset (flash/RTS) restarts the ESP32 but not the sensors; one can keep holding the I²C bus, so every transfer times out | **Unplug and replug the USB cable after flashing.** The recovery code above is not yet tested |
| `externally-managed-environment` on `pip install` | Homebrew Python blocks system-wide pip | Installed `bleak` and the viewer dependencies into `~/zephyrproject/.venv` |
| "No variation" in the viewer | 0–4 m colour scale, small on-screen motion, status-5-only filter | Explained; the data itself varies by 49–467 mm per zone. No code change yet |

## BLE frame contents (219 bytes, `struct hapnav_frame`)

| Field | Size | Contents |
|---|---|---|
| `timestamp_ms` | 4 B | Pin uptime (ms) |
| `quat[4]` | 16 B | BNO055 orientation `[w, x, y, z]` |
| `distances_mm[64]` | 128 B | VL53L5CX 8×8 grid in row-major order (mm) |
| `target_status[64]` | 64 B | Per-zone status (5 = valid, 255 = no target) |
| `obstacles` | 7 B | `urgency[4]` (L, CL, CR, R), `nearest_range_mm`, `flags` |

Not sent over BLE: the head/down VL53L1X distances and the raw accelerometer/gyroscope readings. Those appear only in the pin's USB serial `PIN {...}` log.

Each notification is `[1-byte header][payload]`, with header = `seq << 4 | chunk_idx`.
Concatenating chunks 0..N of one sequence number gives the 219-byte frame.

## Commands

```zsh
# Every new terminal
source ~/zephyrproject/.venv/bin/activate
source ~/zephyrproject/zephyr/zephyr-env.sh

# Build + flash the pin in laptop mode, then UNPLUG/REPLUG the USB cable
cd ~/Desktop/Git/UCI/KeyStone_Project/HapNav/ZephyrProject/Firmware/pin
west build -p always -b xiao_esp32s3/esp32s3/procpu . -- -DEXTRA_CONF_FILE=prj_laptop.conf
west flash

# Original wristband-mode pin build: same command without the "--" part
# Wristband: cd ../wristband && west build -p always -b xiao_ble . && west flash -r uf2

# On the laptop (pick one; the pin accepts only one BLE connection)
python Tools/ble_receiver.py                              # CLI print + log
cd Visualizer && python -m viewer --ble --log             # 3D view + log → http://localhost:8080

# Visualizer tests
cd Visualizer && python -m pytest viewer/tests -q
```

Tips:
- Disconnect nRF Connect on your phone (and stop any other receiver) before connecting from the laptop.
- macOS asks for Bluetooth permission for the terminal app the first time.
- Run the viewer as `python -m viewer`, not `python viewer/viewer.py`, because it uses package-relative imports.
- Logs grow by about 2.5 MB per minute at ~33 frames/s.

## Still open

- **Commit** the Visualizer changes. Keep the `sensors.c` I²C recovery separate until it's tested on the board.
- **Test the I²C recovery:** flash, then do a USB-only reset, and check whether the sensors come back without unplugging.
- **Ignore `__pycache__/`:** viewer runs create untracked `*.cpython-314.pyc` files, and some older `.pyc` files are tracked.
- **Old docs:** `README.md` and `ZephyrProject/Firmware/README.md` still describe STM32 boards, the LSM6DSO sensor and old paths.
- **Rate:** BLE delivers ~33 Hz, likely because of macOS's 30 ms connection interval.
- **Viewer tuning:** the colour range (0–4 m) and status filter (5 only) could be tuned for bench mode (status 9/10 are also usable).
- **Zephyr version:** the local workspace is 4.4 dev, but the project pins v3.7.0. The wristband build hasn't been tried on 4.4.
- **Existing test failures:** 2 tests in `Visualizer/viewer/tests/test_filters.py` fail with or without today's changes. The plane-fit function returns 4 values; the tests expect 3.
