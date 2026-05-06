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

## Magnetometer X-flip

The LIS2MDL is mounted with its X-axis opposite the LSM6DSO's. The pin
firmware flips `mag_raw[0]` once on read, so Madgwick sees a coherent
right-handed sensor frame and so consumers downstream (obstacle pipeline,
visualizer) don't have to know.

## What the obstacle pipeline assumes

The obstacle module receives:

- `quat_wxyz[4]` — body→world rotation
- `gyro_radps[3]` — body-frame ω
- `accel_g[3]` — body-frame specific force, gravity included

`accel_g` is used for the "stationary" detector (gravity-projected lateral
acceleration); `gyro_radps[1]` (yaw rate in body) gates the
`HAPNAV_OBS_FLAG_YAW_SLEWING` flag.
