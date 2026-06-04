# HapNav pin live visualizer

Real-time 3D viewer for the HapNav chest pin's sensor stack — three muxed
ToFs and a BNO055 IMU — rendered through [viser](https://viser.studio/).

The viewer connects to the pin's USB-Serial-JTAG console, parses the
firmware's per-sample `PIN { … }` JSON trace, and shows:

- **Front VL53L5CX (8×8 grid)** — point cloud + 64 zone rays (with optional
  clip-to-measurement and least-squares / RANSAC plane fitting). Mount: pin
  face, 8° downtilt.
- **Head VL53L1X (single zone)** — one ray and one hit point. Mount: top of
  pin face, +20° uptilt.
- **Down VL53L1X (single zone)** — one ray and one hit point. Mount: bottom
  of pin face, 45° downtilt.
- **BNO055** — every frame's quaternion rotates the entire `/pin` frame,
  so all three ToFs rotate together with the physical pin.
- **Obstacle overlay** — per-bin urgency (`L CL CR R`), nearest range, and
  decoded flag bits (`STATIONARY`, `SENSOR_BLOCKED`, `MOSTLY_INVALID`,
  `YAW_SLEWING`, `DROPOFF`, `HEAD_OBSTACLE`).

All three ToFs share the `vl53l5cx-atlas.png` texture as a placeholder
until proper VL53L1X breakout assets land — sensor kind (`GRID_8X8` vs
`SINGLE_ZONE`) drives the actual rendering, not the texture.

## Install

```bash
pip install -r viewer/requirements.txt
```

## Run

```bash
python -m viewer --port /dev/ttyACM1
```

Open <http://localhost:8080> in your browser.

**Options:**

| Flag | Default | Purpose |
|---|---|---|
| `--port`, `-p` | `/dev/ttyACM1` | Serial device for the pin's USB-Serial-JTAG |
| `--baud`, `-b` | `115200` | Baud rate |
| `--host` | `0.0.0.0` | Viser bind address |
| `--viser-port` | `8080` | Viser server port |
| `--debug`, `-d` | off | Verbose logging |

The pin's USB-Serial-JTAG typically enumerates as `/dev/ttyACMn` on Linux
(the previous Adafruit ESP32 viewer defaulted to `/dev/cu.usbserial-0001`
on macOS — adjust for your setup).

## Serial protocol the viewer expects

The pin firmware emits one line per sample (~83 Hz at 50 ms loop), prefixed
with `PIN `:

```
PIN {"ts":4763,
     "a":[ax,ay,az],          # body-frame accel (g)        — informational
     "g":[gx,gy,gz],           # body-frame gyro (rad/s)      — informational
     "q":[w,x,y,z],            # body→world quaternion (BNO055 NDOF, already remapped)
     "head":mm_or_-1,          # head VL53L1X slant range
     "down":mm_or_-1,          # down VL53L1X slant range
     "obs":{"urg":[u0,u1,u2,u3], "near":mm_or_-1, "flags":"0xNN"},
     "d":[..64..],             # front grid distances (mm) — only every ~20th sample
     "st":[..64..]             # front grid statuses     — only every ~20th sample
    }
```

The `d`/`st` grid is emitted at a fraction of the sample rate (firmware's
`dump_grid` divisor) to keep the trace stream compact; the viewer caches
the last-good grid and re-renders it between updates.

## Sensor specs

**Front VL53L5CX** ([datasheet](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf)):

- FoV 65° diagonal (≈ 46° per axis, ±22.3°)
- 8×8 zones → 5.575° angular resolution per zone
- Reports perpendicular (z-axis) distance, not radial — chip does the
  conversion internally
- Range gate the viewer enforces: 50–2000 mm (matches firmware bench mode)

**Head / Down VL53L1X** (single-zone, narrow FoV ~15–27° configurable):

- Range gate the viewer enforces: 40–4000 mm
- Rendered as one ray along the sensor optical axis with a single hit
  point at the measured slant range

## Pin body / world frame conventions

The viewer uses the same convention as the firmware (`obstacle.c`):

- **Pin body frame:** +X right, +Y back, +Z up. So "forward" is **-Y**.
- **World frame:** axis-aligned with pin body in its rest pose. The BNO055
  axis remap is done in the firmware (`CONFIG=0x18`, `SIGN=0x01`), so the
  quaternion the viewer receives is already body→world — no further
  rotation correction is applied.

The pin assembly is anchored at `PIN_WORLD_POSITION` (default `(0, 0, 0.17)`,
matching the ~17 cm bench-mode mount height); tune in `viewer/config.py`.

## Mount tilt angles (configurable per ToF)

| ToF | Position on pin | Optical axis tilt |
|---|---|---|
| Front VL53L5CX | face, middle | 8° below horizontal |
| Head VL53L1X | face, top | 20° above horizontal |
| Down VL53L1X | face, bottom | 45° below horizontal |

Each ToF's mount quaternion is computed from a `R_x(90° + tilt)` rotation
in `config.py`'s `_x_axis_quat_deg()` helper — positive tilt = downward,
negative = upward.

## License

MIT
