# Haptic Drive Policy

`Firmware/lib/haptics/src/haptics.c`. Translates each incoming
`hapnav_obstacles` block into per-LRA drive amplitudes, while keeping
the wearer comfortable and the actuators within thermal budget.

## Why a separate worker, not the BLE callback

The pin's BLE arrival rate is **10 Hz**, but smoothing, drop-off pulses,
fatigue decay, and the watchdog mute all want a stable clock independent
of radio jitter. So:

```
BT RX  ──▶  latch (mutex)  ──▶  haptics_tick (20 Hz)  ──▶  4× LRA
```

`haptics_tick` runs on the system work-queue every 50 ms. It always
ticks, even with no recent frame — that's how the watchdog can mute.

## Stages, in order

### 1. Watchdog

If `now − latch.rx_time_ms > 250 ms`, target = `[0, 0, 0, 0]`. Combined
with stage 7's rate limit, the wristband ramps to silence in ~250 ms,
not abruptly.

### 2. Flag gating

Before honoring `urgency[]` at all:

| Flag | Action |
|------|--------|
| `STATIONARY` ∨ `SENSOR_BLOCKED` ∨ `MOSTLY_INVALID` | mute (target = 0) |
| `DROPOFF` | drop-off pattern (stage 3a); urgency ignored |
| `HEAD_OBSTACLE` | head-obstacle pattern (stage 3b) — outer pair, fast pulse; urgency ignored |
| `YAW_SLEWING` | dampen target × 0.4 |

Priority when multiple flags fire: drop-off > head-obstacle > urgency.
Falling outranks bumping; bumping outranks brushing.

### 3a. Drop-off pattern

When `DROPOFF` is set, all four channels are driven in lock-step:
3 ticks at `DRIVE_MAX`, 3 ticks at 0 (~150 ms / 150 ms square wave,
≈ 3.3 Hz). This deliberately uses a coordination pattern that normal
direction-coded output never produces, so the user can disambiguate
cliff from wall.

### 3b. Head-obstacle pattern

When `HEAD_OBSTACLE` is set (and `DROPOFF` is not), only the **outer
pair** of LRAs (LEFT + RIGHT) pulse at `DRIVE_MAX`, alternating
1 tick on / 1 tick off (~50 ms / 50 ms, **10 Hz**). The centre
channels stay quiet. The bracketing fast pulse is perceptually
distinct from the slower all-four drop-off pattern and from any
direction-graded urgency, so the user reads it as "duck" rather than
"stop" or "veer". The flag also defers to yaw-slew (the pin
suppresses it on its end during head turns).

### 4. Urgency curve

```
urgency < 32           → 0      (perceptual floor; suppresses faint clutter)
urgency = u, u ≥ 32    → DRIVE_MAX × (u-32)/(255-32)
result < 8             → 0      (LRA dead-band)
```

`DRIVE_MAX` is **60 / 127** (≈ 47 %). The DA7280 in DRO mode with
acceleration enabled treats `TOP_CTL2` as 7-bit signed; the 60 cap
matches the DT `per-channel-drive-cap` on each VL53L1X-adjacent DA7280
instance and keeps four LRAs at simultaneous full drive well inside
the BQ25185 LOAD pin budget on the wristband's dedicated buck rail.

### 5. Fatigue (per channel)

A leaky 5 s bucket per channel:

```
on_ms[i] += 50 if cur_amp[i] > 0
on_ms[i] −= 50 each tick (clamped ≥ 0, ≤ 5000)
duty       = on_ms[i] / 5000
```

When `duty > 0.7`, target tapers linearly from 1.0× down to **0.4×** at
`duty = 1.0`. This both protects the actuator and prevents user
habituation: a constant stimulus stops being informative.

### 6. Rate limit

`±24` per tick. Worst-case full-scale ramp = `110 / 24 ≈ 5 ticks` =
**250 ms**. Slow enough that step transitions feel like cresting, fast
enough that genuine "obstacle just appeared" still reads as urgent.

### 7. Drive

For each channel whose limited amplitude differs from last tick:

```
da7280_set_amplitude(lras[ch], amp);
```

The four `lras[]` are device handles resolved from DT aliases
(`hapnav-lra-l/cl/cr/r`). Zephyr's in-tree `ti,tca9546a` mux driver
silently selects the right child bus before every I²C transaction, so
this layer never touches the PCA9546A directly. I²C writes are
skipped at the policy layer when the amplitude is unchanged, so a
steady state (say, all-zero) costs zero I²C traffic.

## Safety

| Concern | Mitigation |
|---------|-----------|
| Pin reboot / link drop | watchdog mute (stage 1) |
| Stuck-on amplitude | fatigue taper (stage 5) caps duty automatically |
| Voltage transients | DA7280 ABS / NOM = 2.5 Vrms hard programmed; `DRIVE_MAX` = 110 keeps below acceleration-mode max |
| Boot order | `main.c` calls `hapnav_haptics_init()` *before* BLE, so a stale `latch` (`valid = false`) keeps motors silent during BT advertising |

## Tuning constants

All policy knobs live as `#define`s at the top of `haptics.c`:

```
DRIVE_PERIOD_MS         50      # worker tick
WATCHDOG_MS             250
URGENCY_FLOOR           32      # below this → silent
DRIVE_MAX               60      # per-channel cap (out of 127)
DRIVE_MAX_TOTAL         150     # policy-side sum-cap; proportional scaling
DRIVE_MIN_PERCEPT       8       # drive below this → 0
RATE_LIMIT_PER_TICK     24
YAW_SLEW_GAIN_NUM/DEN   2/5     # 40 % during head turns
DROPOFF_HALF_TICKS      3       # 3 × 50 ms = 150 ms half-period
HEAD_HALF_TICKS         1       # 1 × 50 ms = 50 ms half-period (10 Hz)
FATIGUE_WINDOW_MS       5000
FATIGUE_HIGH_NUM/DEN    7/10    # taper above 70 % duty
FATIGUE_FLOOR_NUM/DEN   2/5     # taper bottoms at 40 %
```

Anything user-perceivable should be tuned by physical trial on a real
wearer; reading these numbers off paper isn't enough.

## Bench testing without a pin

`hapnav_haptics_inject(struct hapnav_obstacles *)` is a public API used
in place of `hapnav_haptics_consume_frame()`. Wire it from `main.c`
behind a Kconfig or a button to script urgency patterns and verify the
policy in isolation.
