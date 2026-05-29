# Sensor Fusion

Madgwick AHRS (MARG variant) running on the pin every 100 ms.

| Knob | Value |
|------|-------|
| Algorithm | Madgwick AHRS, MARG |
| `beta`    | 0.1 |
| Sample dt | 0.1 s (10 Hz, matches main loop) |

## Unit conversions before `madgwick_update`

| Sensor | LSB | Units fed in |
|--------|-----|--------------|
| Accelerometer | `0.061 mg` per LSB | m/s² (×0.00980665) |
| Gyroscope     | `70.0 mdps` per LSB | rad/s (×π/180000) |
| Magnetometer  | `1.5 mGauss` per LSB | mGauss (Madgwick only cares about ratios) |

Conversion constants are in `hapnav/ble_proto.h` (`HAPNAV_*_PER_LSB`).

## Magnetometer axis remap

The LSM6DSO and the LIS2MDL share a mounting plane but their silkscreen
axes don't line up. Empirically (and consistently with right-handed
orientation rules):

| IMU axis | Same physical direction as | Polarity |
|---|---|---|
| `IMU.X` | `Mag.Y` | same |
| `IMU.Y` | `Mag.X` | same |
| `IMU.Z` | `Mag.Z` | **opposite** |

So the pin firmware applies the mapping `(mag_imu.x, mag_imu.y,
mag_imu.z) = (mag.y, mag.x, -mag.z)` before passing the magnetic vector
to Madgwick. The X↔Y swap with a Z negation is what falls out of the two
sensors being co-planar but rotated 90° about Z and flipped about the
board plane — the right-handed coordinate convention forces the Z sign
flip once the X/Y swap is done. Consumers downstream (obstacle pipeline,
visualizer) see a single coherent right-handed sensor frame.

## What the obstacle pipeline assumes

The obstacle module receives:

- `quat_wxyz[4]` — body→world rotation
- `gyro_radps[3]` — body-frame ω
- `accel_g[3]` — body-frame specific force, gravity included

`accel_g` is used for the "stationary" detector (gravity-projected lateral
acceleration); `gyro_radps[1]` (yaw rate in body) gates the
`HAPNAV_OBS_FLAG_YAW_SLEWING` flag.
