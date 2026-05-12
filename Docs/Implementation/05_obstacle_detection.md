# Obstacle Detection

`Firmware/lib/sensors/src/obstacle.c`. Runs on the pin once per frame
(10 Hz). Inputs: 64-pixel VL53L5CX frame + Madgwick quaternion + gyro +
accel + the VL53L1X head distance. Output: the 7-byte
`hapnav_obstacles` block consumed by the wristband.

## Sensor orientation

The VL53L5CX is **physically rolled +45° around its optical axis** and
the chest-pin enclosure tilts the whole thing **−8° downward** in
world frame. The pipeline mirrors the roll in software so the ray
table stays accurate: each pixel `(r, c)` runs through
`R_z(+45°) → S→B mapping → R_x(+8°)` before being stored in the
init-time ray table.

After the roll the 8×8 grid sits as a **diamond** in world space:

```
                 .             ← original (0,0)  pixel → straight up
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲          ← upper triangle: overlaps head-clearance
             ╱   │   ╲           cone; serves as backup detection.
   (7,0)   .─────●─────.  (0,7)
            ╲   │   ╱          ← anti-diagonal: the 8-pixel horizontal
             ╲  │  ╱             row at elevation 0° in the sensor frame.
              ╲ │ ╱            ← lower triangle: looks down-and-forward,
               ╲│╱               feeds floor-strike + drop-off detection.
                '
                 ← original (7,7) pixel → straight down (−27.5°)
```

Four cardinal corner pixels of the original 8×8 now point at
**±27.5° azimuth and ±27.5° elevation** (sensor frame, before mount
tilt). After applying the −8° downtilt the diamond bottom corner
strikes the floor at `1.35 m / tan(35.5°) ≈ 1.89 m` forward — close
enough to bracket the user's next step.

## Pipeline

| # | Stage | Notes |
|--:|-------|-------|
| 1 | **Validity mask** | keep pixels whose `target_status ∈ {5, 6, 9}` and `range ∈ [300, 3000] mm`. |
| 2 | **Sensor-frame ray table** | pre-computed at init: 64 rays with the **45° optical-axis roll** and the **8° downtilt** baked in. |
| 3 | **World projection** | rotate every ray B→W using the current quaternion. |
| 4 | **Height classification** | floor: `world Z < −1.20 m`; ceiling: `world Z > +0.40 m` (head + 5 cm); else **candidate**. |
| 5 | **Per-bin nearest cluster** | 4 lateral bins, each ≈ 13.75° wide. Pixel → bin via the post-roll azimuth LUT `g_az_bin[64]` (see below). Δd threshold 200 mm, min cluster size 2 px (suppresses single-pixel speckle). |
| 6 | **Closing rate** | frame-to-frame Δrange per azimuth bin; suspended while yaw is slewing (correlation breaks down). |
| 7 | **Urgency score** | `urgency = max(proximity_score, ttc_score)`, gated by motion flags. |
| 8 | **Drop-off** | bottom-of-diamond pixels feed the world-frame floor-strike check. Module unchanged by the roll — it uses world Z directly. See [06_dropoff_detection.md](06_dropoff_detection.md). |
| 9 | **Head clearance** | VL53L1X slant range ≤ 2.2 m → `HEAD_OBSTACLE`. Suppressed during yaw-slew. See [09_head_clearance.md](09_head_clearance.md). |

## Azimuth bins after the roll

Bin boundaries at `±13.75°` post-roll azimuth (corner-to-corner extent
is 55° = ±27.5°). The mapping is computed pixel-by-pixel at init by
projecting each pre-roll `(az, el)` through `R_z(+45°)` and using
`atan2(X_S', Z_S')` as the bin key.

```
   azimuth → −27.5° ───── −13.75° ───── 0° ───── +13.75° ───── +27.5°
              LEFT      CENTER-LEFT  │   CENTER-RIGHT      RIGHT
              bin 0       bin 1      │     bin 2          bin 3
```

Pixels that classify as floor or ceiling are filtered out of the
per-bin nearest-cluster search, so the upper-triangle pixels (which
share azimuth with center bins but elevate far above head) don't
contaminate lateral urgency. Same for the lower-triangle pixels —
they feed drop-off and stay out of the urgency computation.

## Lateral coverage win vs. unrotated

| | Unrotated 8×8 | After +45° roll |
|---|---|---|
| Horizontal extent | ±22.3° (44.5°) | **±27.5° (55°)** |
| Pixels per lateral bin | 16 (2 cols × 8 rows) | varies — corners 1 px, center ~14 px |
| Floor strike (corner pixel) | 2.31 m forward | 1.89 m forward |
| Ceiling strike (corner pixel) | 1.79 m forward at +14.25° world | — (head sensor covers it) |

The user perceives the same 4-bin lateral haptic output, but obstacles
that previously cleared the FoV by a centimetre at the shoulder now
catch the outer bin. The cost is coarser angular resolution **within**
a bin — adequate because the haptic output is a 4-LRA quantised cue, not
a fine-grained map.

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

The thresholds above (200 mm, 1.20 m, 0.40 m, 13.75°, 2.2 m, etc.) are
in `obstacle.c`, `dropoff.c`, and `sensors.c` on the pin side. They are
intentionally *not* exposed via Kconfig because they're tied to the
sensor downtilt, mount height, and physical roll, which are fixed by
the chest-pin enclosure geometry. If the geometry changes, walk through
[`00_system_overview.yaml > geometry_and_physics`](00_system_overview.yaml)
and update the thresholds together.
