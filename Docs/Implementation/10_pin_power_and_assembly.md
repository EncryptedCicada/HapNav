# Power & Assembly — Chest Pin

How the chest pin is wired in v1. Same charger / boost-buck / I²C topology
as the wristband ([08_power_and_assembly.md](08_power_and_assembly.md)),
just with different downstream loads (sensors instead of LRAs).

## Block diagram

```
       USB-C (host)              Single-cell Li-ion
            │                            │
            ▼                            ▼
 ┌──────────────────────────────────────────────┐
 │   BQ25185 charger board (USB aux + power-path)│
 │   LOAD = Vsys (≈ Vbat or VUSB-drop)           │
 └──────────────┬───────────────────┬───────────┘
                │                   │
                ▼                   ▼
   ┌────────────────────┐   ┌─────────────────────────┐
   │ 5 V boost converter│   │ 3.3 V buck converter    │
   │  → XIAO 5V pin     │   │  → I²C bus rail         │
   └────────┬───────────┘   └─────────────┬───────────┘
            │ 5 V                          │ 3.3 V (sensor bus)
            ▼                              │
 ┌──────────────────────────┐              │
 │ Seeed XIAO ESP32-S3      │              │
 │ (BLE central, fusion)    │              │
 └─────┬────────────────┬───┘              │
       │ I2C0            │ GND ── common ──┤
       │ SDA = D2 (GPIO3)│                 │
       │ SCL = D3 (GPIO4)│                 ▼
       │                 │      ┌──────────────────────────┐
       │                 │      │  Qwiic chain on 3.3 V    │
       └─────────────────┴──────┤                          │
                                ├── LSM6DSO IMU      @ 0x6B │
                                ├── LIS2MDL mag      @ 0x1E │
                                ├── VL53L5CX ToF     @ 0x29 → 0x2A (shifted by app)
                                └── VL53L1X head ToF @ 0x29 (after VL53L5CX moves)
```

## Wiring rules that matter

1. **Common ground.** XIAO GND, BQ25185 GND, buck output GND, and boost
   output GND must all be tied. Same rule as the wristband, same failure
   mode if violated — I²C signals reference ground.

2. **Match the I²C rail voltage to the MCU's I/O VDD.** The ESP32-S3's
   I/O switches at its internal 3.3 V supply. The pull-ups on the Qwiic
   boards live on the **buck's** 3.3 V output. Both rails must measure
   close to 3.3 V — if the buck reads 3.0 V or 3.6 V, the bus pull-up
   reference is wrong relative to the MCU's input thresholds and you
   can get sustained NAKs on a bus that otherwise looks healthy.

3. **No additional I²C pull-ups.** Each SparkFun Qwiic board carries
   10 kΩ. With four daisy-chained boards (IMU, mag, VL53L5CX, VL53L1X)
   that's ~ 2.5 kΩ parallel — fine for 400 kHz.

4. **Boot sequence matters for the dual-VL53.** Both VL53L5CX and VL53L1X
   default to 7-bit `0x29`. The pin firmware holds the VL53L1X in `XSHUT`
   (D7 / GPIO1_12 = ESP32-S3 GPIO44 low) until after the VL53L5CX has
   been re-addressed to `0x2A` via the vendor ULD; only then does the
   deferred Zephyr driver release `XSHUT` and bring the VL53L1X up at
   `0x29`. See [09_head_clearance.md](09_head_clearance.md) for the
   full dance.

5. **Mounting holes on the BQ25185 are mechanical-only.** The screw holes
   on the SparkFun breakout are not connected to any signal or rail
   (worst case they tie to the chip's GND-thermal-pad). Tying an
   otherwise-unused ESP32-S3 GPIO to one for mechanical anchoring is
   safe — **except** if you use a strapping pin. Avoid:

   | XIAO pin | ESP32-S3 GPIO | Why |
   |---|---|---|
   | D2 | GPIO0_3 | strapping pin (configures JTAG vs. USB-Serial-JTAG); now in use as **I²C0 SDA** with the Qwiic pull-ups holding it high at reset |
   | D3 | GPIO0_4 | now in use as **I²C0 SCL** |
   | D7 | GPIO1_12 (= GPIO44) | now in use as **VL53L1X XSHUT** (default UART0 RX, but we use USB-Serial-JTAG for the console) |
   | — | GPIO0    | strapping pin (low = enter download mode at reset) — not on the XIAO header but exposed on some adapters |
   | — | GPIO45 / GPIO46 | flash-voltage / ROM-message strapping — not on the XIAO header |

   On the current pin revision D2/D3 carry the I²C bus and D7 is XSHUT,
   so D0, D1, D6, D8, D9, D10 are the remaining safe anchor candidates.

## Sensor positioning

The two ToF sensors do not share an axis with the chest pin's main face —
their angles are fixed by the enclosure geometry:

| Sensor | World-frame tilt | Relative to pin face |
|---|---|---|
| VL53L5CX (forward, 8×8 grid)     | −8° (downward) | 0° (face flush) |
| VL53L1X (head clearance, 1-zone) | +20° (upward)  | +28° wedge above VL53L5CX face |

Derivations and the alternative tilts considered: see
[09_head_clearance.md](09_head_clearance.md).

## Current budget

The pin draws much less than the wristband — no LRAs, no high-current
spikes. At full activity (10 Hz ToF + 10 Hz IMU + 10 Hz BLE write):

| Source | Estimate |
|---|---|
| ESP32-S3 with BLE TX bursts | ~ 80–120 mA average, ~ 200 mA peaks |
| VL53L5CX continuous ranging | ~ 25 mA |
| VL53L1X continuous ranging  | ~ 15 mA |
| LSM6DSO + LIS2MDL idle      | ~ 1 mA total |
| **Sum**                     | **≈ 150 mA** |

Well within the BQ25185's 1 A LOAD pin rating. The 5 V boost handles
the radio peaks; the 3.3 V buck handles the sensors.

## Verifying assembly before first power-up

The same multimeter checks as the wristband apply (continuity from XIAO
GND to all GNDs; resistance between XIAO 3V3 and the buck 3V3 should be
**open** — those are intentionally separate rails). Then plug USB and
watch the UART log:

```
*** Booting Zephyr OS build v4.4.0-XXX ***
[xx] <inf> hapnav_pin: HapNav pin: boot
[xx] <inf> hapnav_obstacle: Obstacle pipeline ready (tilt 8.0°, …)
[xx] <inf> pin_sensors: I²C scan on i2c@60013000:
[xx] <inf> pin_sensors:   found 0x1E (LIS2MDL magnetometer)
[xx] <inf> pin_sensors:   found 0x29 (VL53L5CX or VL53L1X — default)
[xx] <inf> pin_sensors:   found 0x6B (LSM6DSO IMU — SA0 high)
[xx] <inf> pin_sensors:   3 device(s) on the bus
```

That `3 device(s) on the bus` line is the single best confirmation that
the power topology is right and the bus is healthy. If it reads 0, the
bus is electrically dead — the diagnostic tree under "Hardware
diagnostics" in the pin's bring-up notes lists the checks in order.
