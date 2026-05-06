# Wristband

Seeed XIAO nRF52840. BLE peripheral / GATT server. Receives the pin's
frame, runs the haptic policy, drives four LRAs.

## Hardware

| Component | Bus | Address |
|-----------|-----|---------|
| PCA9546A 4-channel I2C mux | I2C | 0x70 |
| 4× DA7280 haptic driver    | I2C (per mux ch) | 0x4A |
| 4× G1040003D LRA           | DA7280 OUT pins | — |

All four DA7280s share the same I2C address; the PCA9546A isolates them.
The bus is `i2c1` on the SoC — the XIAO routes that to D4 (SDA, P0.04)
and D5 (SCL, P0.05), which is the board's Qwiic-compatible pinout.
The board common dtsi already enables `i2c1` with pinctrl; the overlay
only raises the bitrate to 400 kHz.

Channel-to-role mapping:

| Mux ch | Role | LRA position |
|--------|------|--------------|
| 0 | LEFT          | outer-left  |
| 1 | CENTER-LEFT   | inner-left  |
| 2 | CENTER-RIGHT  | inner-right |
| 3 | RIGHT         | outer-right |

This matches `hapnav_obstacles.urgency[]` indexing on the pin, so the
wristband can stay a thin "drive ch[k] at urgency[k]" loop after the
policy step.

## Firmware layout

```
Firmware/wristband/
├── prj.conf
├── boards/xiao_ble.overlay         # i2c0 enabled @ 400 kHz
├── CMakeLists.txt
└── src/
    ├── main.c   # haptics_init() then ble_init()
    ├── ble.c/h  # peripheral role; receives frame, prints JSON, feeds policy
```

Haptic stack in `Firmware/lib/haptics/`:

| File | Purpose |
|------|---------|
| `pca9546a.{c,h}` | mux abstraction; exclusive channel select with caching |
| `da7280.{c,h}`   | Zephyr-native DA7280 driver (DRO mode only) |
| `haptics.{c,h}`  | high-level: init all drivers, run 20 Hz drive policy |

## Threading

- BT RX context: `frame_write()` GATT callback latches the 7-byte
  `hapnav_obstacles` block under a mutex and stamps `k_uptime_get_32()`.
- System work-queue: `haptics_tick` runs every 50 ms (20 Hz). It reads
  the latch, applies the [drive policy](07_haptic_policy.md), and issues
  I2C writes only when an amplitude actually changed.

The drive cadence is decoupled from BLE arrival, so smoothing, drop-off
pulses and the watchdog mute all run on a stable clock even if the radio
link stutters.

## Why a custom DA7280 port

`Drivers/da7280/da7280.c` is the Linux-kernel reference (regmap,
`work_struct`, threaded IRQs, FF input subsystem). None of those exist in
Zephyr. We only need fixed-frequency open-loop LRA drive in DRO mode, so
`Firmware/lib/haptics/src/da7280.c` is a from-scratch port that covers
the bring-up sequence and the single-byte hot path at `TOP_CTL2`.

Intentionally **not** ported:

- PWM / RTWM / ETWM modes
- Waveform-memory (snippet/sequence) playback
- BEMF sensing & frequency tracking telemetry
- Force-feedback effect upload

If any of those become useful, they should land as additions to the
Zephyr port — don't try to wrap the kernel driver.

## Build

```sh
cd ZephyrProject/Firmware/wristband
west build -b xiao_ble
west flash
```
