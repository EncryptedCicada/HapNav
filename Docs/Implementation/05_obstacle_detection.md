# Obstacle Detection

`Firmware/lib/sensors/src/obstacle.c`. Runs on the pin once per frame
(10 Hz). Inputs: 64-pixel VL53L5CX frame + Madgwick quaternion + gyro +
accel. Output: the 7-byte `hapnav_obstacles` block consumed by the
wristband.

## Pipeline

| # | Stage | Notes |
|--:|-------|-------|
| 1 | **Validity mask** | keep pixels whose `target_status ∈ {5, 6, 9}` and `range ∈ [300, 3000] mm`. |
| 2 | **Sensor-frame ray table** | pre-computed at init: 64 rays with the 8° downtilt baked in. |
| 3 | **World projection** | rotate every ray B→W using the current quaternion. |
| 4 | **Height classification** | floor: `world Z < -1.20 m`; ceiling: `world Z > +0.40 m` (head + 5 cm); else **candidate**. |
| 5 | **Clustering** | 8-connectivity, Δd threshold 200 mm, min cluster size 2 px (suppresses single-pixel speckle). |
| 6 | **Closing rate** | frame-to-frame Δrange per azimuth bin; suspended while yaw is slewing (correlation breaks down). |
| 7 | **Urgency score** | `urgency = max(proximity_score, ttc_score)`, gated by motion flags. |

## Azimuth bins

Four bins, each two columns wide, mapping to the wristband's four LRAs:

```
   LEFT    CL   CR    RIGHT
[-22.3°, -11.1°,  0°, +11.1°, +22.3°]
```

## Flags emitted

| Flag | Set when |
|------|----------|
| `STATIONARY`     | gyro magnitude + lateral accel below threshold for N frames |
| `SENSOR_BLOCKED` | majority of pixels near-field invalid (clothing, hand, etc.) |
| `MOSTLY_INVALID` | majority of pixels out-of-range (open space) |
| `YAW_SLEWING`    | body-yaw rate exceeds threshold — closing-rate suspended |
| `DROPOFF`        | drop-off detector trips — see [06_dropoff_detection.md](06_dropoff_detection.md) |

## Tuning constants live in code

The thresholds above (200 mm, 1.20 m, 0.40 m, etc.) are in `obstacle.c` /
`dropoff.c`. They are intentionally *not* exposed via Kconfig because
they're tied to the sensor downtilt and mount height, which are fixed by
the chest-pin enclosure geometry. If the geometry changes, walk through
[`Docs/impl.yaml > geometry_and_physics`](../impl.yaml) and update the
thresholds together.
