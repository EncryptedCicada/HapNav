# Wristband

Seeed XIAO nRF52840. BLE peripheral / GATT server. Receives the pin's
frame, runs the haptic policy, drives four LRAs.

For the supply-side topology (BQ25185 charger → TPS62827 buck → mux +
drivers), see [`08_power_and_assembly.md`](08_power_and_assembly.md).

## Hardware

| Component | Bus | Address |
|-----------|-----|---------|
| PCA9546A 4-channel I2C mux | i2c1 | 0x70 |
| 4× DA7280 haptic driver    | virtual i2c per mux ch | 0x4A |
| 4× G1040003D LRA           | DA7280 OUT pins | — |

All four DA7280s share the same 7-bit I2C address; the PCA9546A isolates
them. We use Zephyr's in-tree `ti,tca9546a` driver — its binding header
explicitly notes "compatible with NXP PCA9546A" — and it publishes each
mux channel as a virtual i2c bus, so each DA7280 becomes an ordinary DT
node at 0x4A on its own child bus.

The parent bus is `i2c1` on the SoC — the XIAO routes that to D4 (SDA,
P0.04) and D5 (SCL, P0.05), which is the board's Qwiic-compatible
pinout. The board common dtsi already enables `i2c1` with pinctrl; the
overlay raises the bitrate to 400 kHz, instantiates the mux at 0x70 with
`i2c-mux-idle-disconnect`, and creates four `dlg,da7280` nodes.

Channel-to-role mapping (matches `hapnav_obstacles.urgency[]` indexing
on the pin):

| Mux ch | Alias            | Role          | LRA position |
|--------|------------------|---------------|--------------|
| 0 | `hapnav-lra-l`  | LEFT          | outer-left   |
| 1 | `hapnav-lra-cl` | CENTER-LEFT   | inner-left   |
| 2 | `hapnav-lra-cr` | CENTER-RIGHT  | inner-right  |
| 3 | `hapnav-lra-r`  | RIGHT         | outer-right  |

The high-level policy resolves the four DT aliases into device handles
and never has to touch the mux directly.

## Firmware layout

```
Firmware/
├── modules/da7280-haptic/        # out-of-tree Zephyr module
│   ├── zephyr/module.yml
│   ├── drivers/haptic/{da7280.c, CMakeLists.txt, Kconfig}
│   ├── dts/bindings/i2c/dlg,da7280.yaml
│   └── include/zephyr/drivers/haptic/da7280.h
│
├── lib/haptics/                  # high-level policy + selftest harness
│   ├── include/hapnav/haptics.h
│   ├── src/haptics.c
│   ├── Kconfig                   # HAPNAV_HAPTICS, HAPNAV_SELFTEST
│   └── CMakeLists.txt
│
└── wristband/
    ├── prj.conf
    ├── boards/xiao_ble.overlay   # i2c1 + tca9546a@70 + 4 DA7280 nodes
    ├── CMakeLists.txt            # registers da7280-haptic via ZEPHYR_EXTRA_MODULES
    ├── Kconfig                   # sources lib/haptics/Kconfig
    └── src/
        ├── main.c                # haptics_init() then ble_init()
        └── ble.c/h               # GATT peripheral; latches obstacles, prints JSON
```

## Threading

- **BT RX context** — `frame_write()` GATT callback hands the 7-byte
  `hapnav_obstacles` block to `hapnav_haptics_consume_frame()`, which
  stores it under a mutex with a `k_uptime_get_32()` stamp. In selftest
  mode this is a no-op (the JSON debug print still runs so the link is
  observable on UART).
- **System work-queue** — `worker_tick()` runs every 50 ms (20 Hz). It
  reads the latch, applies the [drive policy](07_haptic_policy.md),
  and issues `da7280_set_amplitude()` whenever a channel's amplitude
  actually changed.
- **Selftest thread** (only when `CONFIG_HAPNAV_SELFTEST=y`) — a
  dedicated thread executes T0–T10. T0–T6 drive chips directly; the
  worker is held back until T7 so it doesn't fight the raw-chip phase.
  T7–T10 inject synthetic obstacles and rely on the worker to evolve
  rate-limit / fatigue / watchdog state.

The drive cadence is decoupled from BLE arrival, so smoothing, drop-off
pulses, and the watchdog mute all run on a stable clock even if the
radio link stutters.

## DA7280 driver

Lives at `Firmware/modules/da7280-haptic/` as a proper out-of-tree
Zephyr module: `DT_DRV_COMPAT dlg_da7280`, per-instance config from DT,
uses `i2c_dt_spec` so the in-tree mux driver transparently selects the
channel before every transaction.

Coverage:

- CHIP_REV detection, full TOP_CFG1..5 setup, V2I calibration
- DRO hot path: TOP_CTL2 single-byte amplitude write
- Auto INACTIVE↔DRO mode toggle on amp 0↔non-zero
- Global drive budget (`CONFIG_HAPTIC_DA7280_TOTAL_DRIVE_CAP`)
- IRQ workqueue (irq-gpios optional; we don't wire one)
- `CONFIG_HAPTIC_DA7280_SAFE_INIT` — defang prologue (TOP_CTL2=0,
  OP_MODE=INACTIVE) before any config, to survive host-side resets
  while the chip keeps power
- VDD ADC and CALIB_IMP readbacks for telemetry
- RTWM/ETWM and waveform-memory upload exist in the driver but the
  policy doesn't use them yet

The Linux-kernel reference at `ZephyrProject/Drivers/da7280/da7280.c`
is kept for documentation — it uses regmap, `work_struct`, threaded
IRQs, and the FF input subsystem, none of which exist on Zephyr, so we
don't try to compile it.

## Selftest

`CONFIG_HAPNAV_SELFTEST=y` swaps the BLE-driven path for an isolated
bench-test sequence. See [`02_wristband.yaml`](02_wristband.yaml)
for the full T0–T10 list; in summary:

| Phase | Tests | What it exercises |
|-------|-------|-------------------|
| Raw chip | T0 probe, T1 channel walk, T2 ramp, T3 crossfade, T4 all-on stress, T5 update-rate stress, T6 dropoff square wave | DA7280 driver + mux, supply rail under load, I2C bandwidth, IRQ surveillance |
| Policy   | T7 simulated obstacles, T8 fatigue taper, T9 watchdog mute, T10 idle hold | Full pipeline (latch → worker → set_amplitude) on synthetic input |

## Build

```sh
cd ZephyrProject/Firmware/wristband
west build -b xiao_ble
west flash
```

Toggle the selftest gate in `prj.conf`:

```kconfig
CONFIG_HAPNAV_SELFTEST=y    # bench validation (default for v4)
CONFIG_HAPNAV_SELFTEST=n    # normal navigation — react to BLE frames
```
