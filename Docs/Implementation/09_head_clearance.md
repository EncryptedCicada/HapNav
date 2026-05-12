# Head Clearance & Future Height Calibration

The chest pin now carries a single-zone **VL53L1X** alongside the
**VL53L5CX**. The two sensors share `i2c0`, divide the pin's field of view
between them, and rely on an `XSHUT`-driven address shuffle at boot
because they both default to 7-bit `0x29`.

```
                                    ┌──────────────┐
        head-clearance cone         │              │
        (VL53L1X, +20° tilt,        │  user        │
        ≈ 27° FoV)                  │              │
                ╲╲╲                 │              │
                  ╲╲╲╲              │  ◄─ obstacle │
                    ╲╲╲╲            │              │
   pin   ────────────────────►       1.75 m ▲
   1.35 m    ───────────────────►            │
   above     ╱ ╱ ╱ ╱ ╱ ╱  forward            │
   floor   ╱ ╱ ╱ ╱ ╱        VL53L5CX         │
         ╱ ╱ ╱ ╱   (45°-rolled diamond,      │
       ╱ ╱ ╱ ╱     −8° downtilt)      floor ▼
     ╱ ╱ ╱
   floor
```

## VL53L1X (head) — geometry

| Parameter | Value | Notes |
|---|---|---|
| Mount height | 1.35 m | shared with VL53L5CX |
| Optical-axis tilt | **+20° above world horizontal** | nose-up relative to VL53L5CX |
| Tilt relative to VL53L5CX face | **+28°** | the chest pin face is at −8° downtilt |
| Cone FoV | ≈ 27° full-cone | sensor default; no ROI shrink |
| Useful range | 0.3 – 4.0 m | datasheet typical |
| Sample rate | 10 Hz | locks with VL53L5CX cadence |
| Proximity flag threshold | 2.2 m slant range | tuned for the envelope below |

### Detection envelope

Cone vertical span at distance L: `L × (tan(33.5°) − tan(6.5°)) ≈ L × 0.55`.

| Forward distance | Cone bottom (world Z) | Cone top (world Z) | Catches |
|---|---|---|---|
| 1.0 m | 1.46 m | 2.01 m | forehead, jaw, chin-level shelf edges |
| 1.5 m | 1.52 m | 2.34 m | forehead → above doorframes |
| 2.0 m | 1.58 m | 2.67 m | low-hanging signs, low branches |
| 2.5 m | 1.64 m | 3.01 m | door frames at the far edge of reaction time |

At a 0.7 m/s walking-speed prior the user has ≈ 2.1 s to react when an
obstacle first enters the cone at 1.5 m forward. The 2.2 m proximity
threshold is conservative — by the time we flag, the obstacle is inside
the cone and within ~1.6 m forward (allowing for some slant on the cone).

### Why this tilt, not something else

- `+15°` was the alternative. It buys extra range on far-forward
  obstacles (catches a doorframe at 3.5 m) but loses the close-in
  envelope inside ~80 cm — exactly the distance where a user has
  already started leaning into the obstacle. Trade-off rejected.
- `+25°` ducks the close envelope tighter but starts to miss
  standard 2.0 m doorframes at the 2 m forward mark, which is the
  primary failure case we're trying to prevent.

## I²C address shuffle (bring-up sequence)

Both VL53L5CX and VL53L1X power up at 7-bit `0x29`. Until a mux is
present on the pin we sequence them with the VL53L1X's `XSHUT` line:

1. **Hold `XSHUT` low** before any I²C traffic. The pin firmware does
   this in `pin_sensors_init()` via `gpio_pin_configure_dt(&head_xshut,
   GPIO_OUTPUT_INACTIVE)`. Active-high in DT means asserted-high = on;
   inactive = low = sensor in reset.
2. **Bring up the VL53L5CX** at 7-bit `0x29` (vendor ULD).
3. **Move the VL53L5CX to 7-bit `0x2A`** using the ULD's
   `vl53l5cx_set_i2c_address()`. The new address survives until the
   next power cycle.
4. **`device_init(tof_head)`** — Zephyr's in-tree VL53L1X driver is
   marked `zephyr,deferred-init` so it hasn't probed yet. The
   `device_init()` call drives `XSHUT` high, waits the chip's 1.2 ms
   boot time, and probes at the now-free `0x29`.

Addresses are not persistent on either chip. On every power cycle the
VL53L5CX boots back at `0x29`, and the shuffle repeats.

The DT alias `hapnav-tof-head` points at the VL53L1X node so the
firmware can resolve the `XSHUT` GPIO without hard-coding which `&gpioN
M` it lives on. Pin assignment is `D2 = &gpio0 3`.

## Wristband response — `HAPNAV_OBS_FLAG_HEAD_OBSTACLE`

When the head sensor sees a target inside the proximity threshold, the
pin sets the new `(1U << 5)` flag in `hapnav_obstacles.flags`. The
wristband's haptic policy reacts:

- **Outer pair only** (LEFT + RIGHT) at full amplitude, pulsing at 10 Hz
  (50 ms on / 50 ms off). Centre channels stay quiet.
- Distinct from drop-off (all-4 at 3.3 Hz, slower) and from
  directional-obstacle drive (graded amplitude, no pulse).
- Suppressed during yaw-slew so a head turn past a sign doesn't
  spuriously trigger.
- Drop-off wins if both flags fire — falling outranks bumping.

Selftest case "head-obstacle flag" in T7 exercises the pattern with no
hardware needed (injects the flag for 700 ms).

## Future: down-facing VL53L1X for height calibration

When the second VL53L1X arrives — currently blocked on procuring an
I²C mux for the pin (or, alternatively, the same XSHUT/address dance
applied to a third chip) — it earns a fixed slot on the pin enclosure
pointing **straight down**, possibly tilted −80° to clear the user's
feet during a casual look-down.

### What it does

One-shot reading at startup (or when triggered by a "stood still for 3
seconds" detector via the IMU). The measured distance is the live
sensor mount height, which today is hard-coded as `SENSOR_HEIGHT_M =
1.35 f` in `obstacle.c` and `dropoff.c`. With per-user calibration the
floor-classification band, drop-off predicted slant range, and head
clearance envelope all retune themselves.

### Why not just use the IMU + accelerometer?

The IMU gives orientation, not absolute height above floor. Without a
direct measurement we'd have to ask the user (height in cm) and assume
where they wear the pin — both error-prone for a wearable for blind
users.

### Address-conflict options for the second VL53L1X

We have three. None is in the firmware today; this is a roadmap.

| Option | Pros | Cons |
|---|---|---|
| **TCA9548A I²C mux** on the pin | Same model as the wristband. Future-proof if a fourth sensor (e.g. flank-facing ToF) lands later. | Adds a part. |
| **XSHUT + address swap on the second VL53L1X** | No new silicon. Mirrors the current head-sensor dance. | Two `XSHUT` GPIOs on the pin (D2 already used; add D3). |
| **Defer to head-sensor's spare cycles** | No new bus mods at all — re-use the same VL53L1X by temporarily re-aiming it. | Mechanically infeasible on a fixed-mount pin. |

When the second sensor lands, the XSHUT option is what this doc
recommends; the mux is justified only when we know a third sensor is
imminent. Document the choice in this section before any code lands.
