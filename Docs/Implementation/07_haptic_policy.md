# Haptic Drive Policy

`Firmware/lib/haptics/src/haptics.c`. Translates each incoming
`hapnav_obstacles` block into per-LRA pulses — short, hard hits whose
*rate* encodes urgency. Bench testing on the SparkFun haptic stack
showed continuous graded-amplitude drive was hard to feel through the
wrist strap; short bangs are clearly perceptible.

## Why a separate worker, not the BLE callback

The pin's BLE arrival rate is **10 Hz**, but the pulse cycles, drop-off
square wave, fatigue decay, and watchdog mute all want a stable clock
independent of radio jitter. So:

```
BT RX  ──▶  latch (mutex)  ──▶  haptics_tick (20 Hz)  ──▶  4× LRA
```

`haptics_tick` runs on the system work-queue every 50 ms. It always
ticks, even with no recent frame — that's how the watchdog can mute.

## Stage overview

```
worker_tick():
    read latch under mutex
    if watchdog tripped: drive all 0, mark was_stale, continue
    if any override (drop-off / head / mute): drive that pattern
    else for each channel:
        target  = urgency → intensity (yaw-slew dampened, fatigue-attenuated)
        period  = target → ticks between pulses
        pulse(channel, period)
```

The pulse model means a channel is **either** firing a single tick at
`DRIVE_MAX` or sitting silent — there is no held intermediate level.
Intensity is encoded purely as **pulse rate**.

### 1. Watchdog

If `now − latch.rx_time_ms > 250 ms`, every channel goes immediately to
0 and the per-channel pulse phases reset. With the policy idle thereafter
the wristband is silent within one tick (~50 ms).

### 2. Flag gating

Before deriving pulse rates from `urgency[]`:

| Flag | Action |
|------|--------|
| `STATIONARY` ∨ `SENSOR_BLOCKED` ∨ `MOSTLY_INVALID` | mute (drive 0, reset pulse phases) |
| `DROPOFF` | drop-off override (stage 3a); urgency ignored |
| `HEAD_OBSTACLE` | head-obstacle override (stage 3b); urgency ignored |
| `YAW_SLEWING` | dampen target intensity × 0.4 → longer pulse period |

Priority when multiple flags fire: **drop-off > head-obstacle > mute >
urgency-pulse**. Falling outranks bumping; bumping outranks brushing.

### 3a. Drop-off override

When `DROPOFF` is set, all four channels are driven in lock-step:
3 ticks at `DRIVE_MAX`, 3 ticks at 0 (≈ 150 ms / 150 ms square wave,
≈ 3.3 Hz). This deliberately uses a coordination pattern that the
urgency-pulse path never produces, so the user can disambiguate cliff
from wall.

### 3b. Head-obstacle override

When `HEAD_OBSTACLE` is set (and `DROPOFF` is not), only the **outer
pair** of LRAs (LEFT + RIGHT) pulse at `DRIVE_MAX`, alternating
1 tick on / 1 tick off (≈ 50 ms / 50 ms, 10 Hz). Centre channels stay
quiet. The fast bracketing pulse is perceptually distinct from the
slower all-four drop-off pattern.

### 4. Urgency intensity (target)

For each channel that's not under an override:

```
urgency < 32           → target = 0      (perceptual floor)
urgency ≥ 32           → target = DRIVE_MAX × (u − 32) / (255 − 32)
target  < 8            → target = 0      (LRA dead-band)
```

If `YAW_SLEWING` is set, `target` is multiplied by 0.4 — the gain
reduction translates downstream into a slower pulse rate, not a softer
hit.

`DRIVE_MAX` is **60 / 127** (≈ 47 %). The DA7280 in DRO mode with
acceleration enabled treats `TOP_CTL2` as 7-bit signed; the 60 cap
matches the DT `per-channel-drive-cap` and keeps four channels at
simultaneous firing inside the BQ25185 LOAD pin budget.

### 5. Fatigue (per channel)

A leaky bucket measured against actual pulse-on ticks:

```
on_ms[i] += 50  when this tick fired (cur_amp == DRIVE_MAX)
on_ms[i] −= 50  when this tick was silent (clamped ≥ 0, ≤ 5000)
duty       = on_ms[i] / 5000
```

When `duty > 0.7`, `target` tapers linearly down to **0.4×** at
`duty = 1.0`. Because the pulse rate is derived from `target`, this
shows up as **fewer pulses**, not weaker pulses. A channel pulsing
flat-out (6.7 Hz) drops to ~ 1.3 Hz once the bucket fills, which
re-introduces perceptual novelty between hits.

### 6. Pulse period

```
period_ticks =  PULSE_PERIOD_SLOW
              − (target − DRIVE_MIN_PERCEPT) × (SLOW − FAST) / (DRIVE_MAX − DRIVE_MIN_PERCEPT)
```

`PULSE_PERIOD_FAST_TICKS = 3` (150 ms = 6.7 Hz) at full intensity.
`PULSE_PERIOD_SLOW_TICKS = 20` (1 s = 1 Hz) just above the perceptual
floor. Below the floor → period = 0 = channel muted.

### 7. Drive

For each channel whose current pulse phase rolls over the computed
period, fire one tick (50 ms) at `DRIVE_MAX`; otherwise hold 0:

```
da7280_set_amplitude(lras[ch], fire ? DRIVE_MAX : 0);
```

The four `lras[]` are device handles resolved from DT aliases
(`hapnav-lra-l/cl/cr/r`). Zephyr's in-tree `ti,tca9546a` mux driver
silently selects the right child bus before every I²C transaction, so
this layer never touches the PCA9546A directly. The DA7280's
acceleration mode + `dlg,rapid-stop-enable` handle the mechanical
attack/decay inside that 50 ms window — the chip brakes the LRA back
to 0 cleanly, which is what makes the pulses feel discrete instead of
ringing into each other.

The policy keeps a per-channel `cur_amp` so it only writes I²C when the
drive level actually changes (every tick of the gap is a no-op after
the first).

## Safety

| Concern | Mitigation |
|---------|-----------|
| Pin reboot / link drop | watchdog mute (stage 1) |
| Stuck-on amplitude | fatigue (stage 5) slows pulse rate automatically |
| Voltage transients | DA7280 ABS / NOM = 2.5 Vrms hard programmed; `DRIVE_MAX = 60` keeps below acceleration-mode max |
| Simultaneous 4-channel pulse | per-channel cap × 4 = 240, `CONFIG_HAPTIC_DA7280_TOTAL_DRIVE_CAP = 250` backstops |
| Boot order | `main.c` calls `hapnav_haptics_init()` *before* BLE, so a stale `latch` (`valid = false`) keeps motors silent during BT advertising |

## Tuning constants

All policy knobs live as `#define`s at the top of `haptics.c`:

```
DRIVE_PERIOD_MS         50      # worker tick
WATCHDOG_MS             250
URGENCY_FLOOR           32      # below this → silent
DRIVE_MAX               60      # per-channel cap (out of 127)
DRIVE_MIN_PERCEPT       8       # target below this → 0
PULSE_ON_TICKS          1       # 50 ms hit per pulse
PULSE_PERIOD_FAST_TICKS 3       # 150 ms cycle at max urgency (6.7 Hz)
PULSE_PERIOD_SLOW_TICKS 20      # 1 s cycle just above floor (1 Hz)
YAW_SLEW_GAIN_NUM/DEN   2/5     # 40 % during head turns
DROPOFF_HALF_TICKS      3       # 3 × 50 ms = 150 ms half-period (3.3 Hz)
HEAD_HALF_TICKS         1       # 1 × 50 ms = 50 ms half-period (10 Hz)
FATIGUE_WINDOW_MS       5000
FATIGUE_HIGH_NUM/DEN    7/10    # taper above 70 % bucket fill
FATIGUE_FLOOR_NUM/DEN   2/5     # taper bottoms at 40 % of original intensity
```

Anything user-perceivable should be tuned by physical trial on a real
wearer; reading these numbers off paper isn't enough.

## Bench testing without a pin

`hapnav_haptics_inject(struct hapnav_obstacles *)` is a public API used
in place of `hapnav_haptics_consume_frame()`. The selftest harness
(`CONFIG_HAPNAV_SELFTEST=y`) uses it during the simulated-obstacle
phase. Note: with the pulse model, `log_cur_amps()` snapshots the
instantaneous drive level (0 or `DRIVE_MAX`), not a held average — read
it as *"did this channel just fire?"* rather than *"how loud is this
channel?"*.
