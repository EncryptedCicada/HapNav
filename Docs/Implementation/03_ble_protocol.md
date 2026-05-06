# BLE Protocol

One custom GATT primary service, one writable characteristic. The pin
(central) connects to the wristband (peripheral) and pushes the latest
fused frame using `Write Without Response`.

Defined in `Firmware/lib/ble/include/hapnav/ble_proto.h`; this header is
the single source of truth shared by both targets.

## Service & characteristic

| | UUID base |
|-|-----------|
| Service | `5a9c0001-7e6f-4abc-9c12-aabbccdd1100` |
| Frame   | `5a9c0004-7e6f-4abc-9c12-aabbccdd1100` |

UUIDs are derived via `HAPNAV_UUID_VAL(0x000N)`.

## MTU

```
CONFIG_BT_L2CAP_TX_MTU=247
CONFIG_BT_BUF_ACL_RX_SIZE=251
CONFIG_BT_BUF_ACL_TX_SIZE=251
```

Negotiated MTU is 247, which fits the 219-byte frame in a single ATT PDU
and leaves 4 bytes of L2CAP header inside a 251-byte ACL buffer.

## Frame layout (219 bytes, packed)

| Offset | Field | Type | Notes |
|-------:|-------|------|-------|
|   0 | `timestamp_ms`    | `uint32` | `k_uptime_get_32()` at frame assembly |
|   4 | `quat[4]`         | `float32` | unit quaternion `[w, x, y, z]`, Madgwick |
|  20 | `distances_mm[64]`| `int16`  | VL53L5CX 8×8 distances |
| 148 | `target_status[64]`| `uint8` | VL53L5CX status flags |
| 212 | `obstacles.urgency[4]` | `uint8` | L, CL, CR, R (0–255) |
| 216 | `obstacles.nearest_range_mm` | `int16` | -1 if clear |
| 218 | `obstacles.flags` | `uint8` | bitfield, see below |

```
HAPNAV_OBS_FLAG_STATIONARY      (1 << 0)
HAPNAV_OBS_FLAG_SENSOR_BLOCKED  (1 << 1)
HAPNAV_OBS_FLAG_MOSTLY_INVALID  (1 << 2)
HAPNAV_OBS_FLAG_YAW_SLEWING     (1 << 3)
HAPNAV_OBS_FLAG_DROPOFF         (1 << 4)
```

## Channel mapping (azimuth bins)

| index | name          | azimuth bin |
|------:|---------------|-------------|
| 0 | LEFT          | [-22.3°, -11.1°] |
| 1 | CENTER-LEFT   | [-11.1°,   0°  ] |
| 2 | CENTER-RIGHT  | [   0° , +11.1°] |
| 3 | RIGHT         | [+11.1°, +22.3°] |

One bin = two columns of the 8-wide ToF grid.

## What the wristband consumes

Of the 219 bytes the pin sends, the haptic policy only acts on:

- `timestamp_ms`            — for the watchdog
- `obstacles.urgency[4]`    — per-channel drive target
- `obstacles.flags`         — gating + drop-off pulse pattern

The full frame is still printed as JSON over UART for bring-up debugging
(`print_frame_json()` in `wristband/src/ble.c`).
