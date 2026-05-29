# Obstacle Detection

`Firmware/lib/sensors/src/obstacle.c`. Runs on the pin once per frame
(10 Hz). Inputs: 64-pixel VL53L5CX frame + Madgwick quaternion + gyro +
accel + the VL53L1X head distance. Output: the 7-byte
`hapnav_obstacles` block consumed by the wristband.

## Sensor orientation

The VL53L5CX is mounted **axis-aligned** in the chest-pin enclosure
(no optical-axis roll) and tilted **−8° downward** in world frame. The
pipeline applies the −8° tilt to the pre-computed per-pixel ray table
at init: each pixel `(r, c)` runs through `S → B mapping → R_x(+8°)`
and lands in `g_ray_B[]`.

Earlier design iterations considered a +45° physical roll on the
VL53L5CX to widen the lateral coverage from ±22.3° to ±27.5°, but the
final enclosure mounts the sensor axis-aligned. The pipeline has no
software roll to mirror.

## Pipeline

| # | Stage | Notes |
|--:|-------|-------|
| 1 | **Validity mask** | keep pixels whose `target_status ∈ {5, 6, 9}` and `range ∈ [300, 3000] mm`. |
| 2 | **Sensor-frame ray table** | pre-computed at init: 64 rays with the −8° downtilt baked in. |
| 3 | **World projection** | rotate every ray B→W using the current quaternion. |
| 4 | **Height classification** | floor: `world Z < −1.20 m`; ceiling: `world Z > +0.40 m` (head + 5 cm); else **candidate**. |
| 5 | **Per-bin nearest cluster** | 4 lateral bins, each 2 columns wide: `c=0,1 → LEFT`, `c=2,3 → CL`, `c=4,5 → CR`, `c=6,7 → RIGHT`. Δd threshold 200 mm, min cluster size 2 px. |
| 6 | **Closing rate** | frame-to-frame Δrange per azimuth bin; suspended while yaw is slewing (correlation breaks down). |
| 7 | **Urgency score** | `urgency = max(proximity_score, ttc_score)`, gated by motion flags. |
| 8 | **Drop-off** | bottom-row pixels feed the world-frame floor-strike check. See [06_dropoff_detection.md](06_dropoff_detection.md). |
| 9 | **Head clearance** | VL53L1X slant range ≤ 2.2 m → `HEAD_OBSTACLE`. Suppressed during yaw-slew. See [09_head_clearance.md](09_head_clearance.md). |

## Azimuth bins

```
   azimuth → −22.3° ───── −11.1° ───── 0° ───── +11.1° ───── +22.3°
              LEFT      CENTER-LEFT  │   CENTER-RIGHT      RIGHT
              c=0,1      c=2,3      │    c=4,5            c=6,7
```

Each bin is two columns wide (≈ 11.1° lateral coverage per bin). The
4-bin output drives the four LRAs on the wristband 1:1.

## Flags emitted

| Flag | Set when |
|------|----------|
| `STATIONARY`     | gyro magnitude + lateral accel below threshold for N frames |
| `SENSOR_BLOCKED` | majority of pixels near-field invalid (clothing, hand, etc.) |
| `MOSTLY_INVALID` | majority of pixels out-of-range (open space) |
| `YAW_SLEWING`    | body-yaw rate exceeds threshold — closing-rate suspended |
| `DROPOFF`        | drop-off detector trips — see [06_dropoff_detection.md](06_dropoff_detection.md) |
| `HEAD_OBSTACLE`  | VL53L1X reads ≤ 2.2 m and yaw is steady — see [09_head_clearance.md](09_head_clearance.md) |

## Tuning constants live in code

The thresholds above (200 mm, 1.20 m, 0.40 m, 2.2 m, etc.) are in
`obstacle.c`, `dropoff.c`, and `sensors.c` on the pin side. They are
intentionally *not* exposed via Kconfig because they're tied to the
sensor downtilt and mount height, which are fixed by the chest-pin
enclosure geometry. If the geometry changes, walk through
[`00_system_overview.yaml > geometry_and_physics`](00_system_overview.yaml)
and update the thresholds together.

Exception: `CONFIG_HAPNAV_BENCH_MODE` swaps the whole set of
distance-tied constants for a bench-scale equivalent so the pipeline
can be exercised on a desk. See [`01_chest_pin.md > Bench-demo
mode`](01_chest_pin.md) for the full re-scaling table.
