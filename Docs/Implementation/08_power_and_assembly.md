# Power & Assembly — Wristband

How the wristband is wired in v4. Built around three commercial breakouts so
the whole supply path is socketable for bench work; the goal is no shared
rail between the radio and the LRAs (the v3 brownouts that drove the policy
caps came from exactly that mistake).

## Block diagram

```
       USB-C (host PC)                      Single-cell Li-ion
            │                                        │
            ▼                                        ▼
 ┌──────────────────────────┐         ┌────────────────────────────┐
 │   Seeed XIAO nRF52840    │         │    BQ25185 charger board   │
 │   (logic + BLE radio)    │         │    USB aux input + power-  │
 │                          │         │    path output  (LOAD pin) │
 └─────┬────────────────┬───┘         └──────────────┬─────────────┘
       │ I2C1            │                           │ Vsys (≈ Vbat or VUSB-drop)
       │ SCL = D5 (P0.05)│ GND ───── common ─────────┤
       │ SDA = D4 (P0.04)│                           ▼
       │                 │              ┌──────────────────────────┐
       │                 │              │  TPS62827 buck (3.3 V)   │
       │                 │              │  EN tied high (default)  │
       │                 │              └─────────────┬────────────┘
       │                 │                            │ 3.3 V (haptic rail)
       │                 │      ┌─────────────────────┼─────────────────────┐
       │                 │      │                     │                     │
       │                 │      ▼                     ▼                     ▼
       │            ┌─────────────────┐  Qwiic chain  passes 3.3 V / GND / SDA / SCL
       └────────────│ PCA9546A mux @  │── ch0 → DA7280 + LRA  (LEFT)
                    │   0x70  (Qwiic) │── ch1 → DA7280 + LRA  (CENTER-LEFT)
                    │                 │── ch2 → DA7280 + LRA  (CENTER-RIGHT)
                    │                 │── ch3 → DA7280 + LRA  (RIGHT)
                    └─────────────────┘
```

## Boards & roles

| Block | Part | Role |
|-------|------|------|
| MCU      | Seeed XIAO nRF52840 (USB-powered)      | BLE peripheral, I2C master |
| Charger  | BQ25185 breakout                       | Battery charging, power-path Vsys to LOAD |
| Buck     | TPS62827 breakout (3.3 V output)       | Dedicated 3.3 V rail for the haptic stack |
| Mux      | PCA9546A breakout @ 0x70 (Qwiic)       | 4-channel I2C switch, daisy-chains 3.3 V to driver boards |
| Driver   | 4× SparkFun DA7280 + Jinlong G1040003D | One LRA per mux channel, all share I2C addr 0x4A |

## Wiring rules that matter

1. **Common ground.** XIAO GND **and** the buck output GND **and** the BQ25185
   ground **must** be tied together. I2C signals reference ground; the two
   halves of the system floating apart would NAK every transaction even
   though every chip is alive. This is the single most likely thing to
   forget on first assembly.

2. **No additional I2C pull-ups.** The Qwiic mux board carries its own 10 kΩ
   pull-ups, and each DA7280 board does too. With the four daisy-chained
   driver boards that totals 5 × 10 kΩ ≈ 2 kΩ on each line, which is healthy
   for a 400 kHz bus on a short cable run. Adding more would over-drive the
   bus and increase rise-time crosstalk.

3. **Power the haptic rail from the buck only.** Don't bridge the XIAO's
   3V3 output to the mux VCC even "as a backup." That re-creates the v3
   shared-rail problem the moment you also have USB plugged in.

4. **Mux comes up before the nRF probes I2C.** This is the natural order on
   plug-in: BQ25185 LOAD is stable in microseconds, the buck soft-starts in
   <1 ms, the nRF needs ~10 ms before driver init runs. No firmware delay
   needed. (See "Why we don't gate `EN` from the nRF" below.)

5. **`i2c-mux-idle-disconnect` is set in the overlay.** The in-tree TCA954X
   driver re-writes the channel mask before every transaction, which means
   the mux can never be left in a stale state and the PCA9546A `/RST` pin
   doesn't need to be wired. If you ever assemble a production rev that
   wants a hardware-recovery hammer, route `/RST` to a free nRF GPIO; it's
   not load-bearing today.

## Current budget

Worst case at the all-on-stress step (selftest T4, amp = 60 on all four
channels simultaneously):

| Source | Value |
|--------|-------|
| 4 × LRA peak current             | ≈ 4 × 165 mA = 660 mA |
| 4 × DA7280 + mux quiescent       | ≈ 30 mA |
| Buck conversion overhead         | ≈ 50 mA equivalent at LOAD |
| **Sum at BQ25185 LOAD pin**      | **≈ 740 mA** |
| BQ25185 LOAD rating              | 1 A |

Comfortable, not luxurious. The TPS62827 (2.5–4 A capable) is over-sized;
the BQ25185 LOAD pin is the bottleneck. If a future hardware rev wants
headroom for additional actuators, swap to a higher-current power-path PMIC
before touching the buck.

## Boot order

1. USB plug-in → VBUS rises.
2. XIAO onboard regulator brings nRF up; ROM bootloader runs (~5 ms).
3. In parallel: BQ25185 routes Vsys to LOAD; TPS62827 soft-starts to 3.3 V
   (<1 ms after EN sees Vin); mux + four DA7280 boards are I2C-ready.
4. Zephyr early init runs; I2C controller is configured at the
   `I2C_INIT_PRIORITY` pass.
5. `ti,tca9546a` driver probes 0x70 (writes `0x00`, expects ACK) at
   `CONFIG_I2C_TCA954X_ROOT_INIT_PRIO=I2C_INIT_PRIORITY`.
6. `dlg,da7280` driver probes 0x4A on each child bus at
   `CONFIG_HAPTIC_DA7280_INIT_PRIORITY=80` (after the channel buses come up
   at `CONFIG_I2C_TCA954X_CHANNEL_INIT_PRIO=70`). `SAFE_INIT` writes
   `TOP_CTL2=0` and forces `OP_MODE=INACTIVE` before any config — this
   defangs the chip if it survived a host reset with non-zero amplitude.
7. `main()` runs `hapnav_haptics_init()` → if `CONFIG_HAPNAV_SELFTEST=y`
   the bench runner thread starts; otherwise the 20 Hz worker is scheduled.
8. `wrist_ble_init()` brings up the BLE peripheral and starts advertising
   as `HapNav-Wrist`.

## Why we don't gate `EN` from the nRF (yet)

The TPS62827's `EN` pin is left tied high so the buck is on whenever the
charger is providing Vsys. Wiring `EN` to a free nRF GPIO would let the
firmware cut the entire haptic subsystem (saves the buck's quiescent draw
and the four DA7280 standby currents during long idle stretches). The
trade-off is a startup dependency: the nRF would have to assert `EN`,
wait ~2 ms for the buck to settle, and only then init I2C — otherwise the
mux probe NAKs.

Worth doing for battery-life work later; not needed for the selftest or
for normal navigation. If you do wire it, give the GPIO a DT alias
(`hapnav-buck-en`), assert it at the very top of `hapnav_haptics_init()`
with a `k_msleep(2)` settle, and bump `CONFIG_I2C_TCA954X_ROOT_INIT_PRIO`
above the GPIO init priority.

## Verifying assembly before first power-up

Quick continuity / smoke checks before you plug USB:

1. Multimeter continuity from XIAO GND pad → BQ25185 GND → TPS62827 GND →
   PCA9546A GND. Single common-ground island.
2. Multimeter resistance from XIAO 3V3 to PCA9546A 3.3 V pin. Should be
   **open** (no continuity). If it reads ohms-level, you've bridged the
   nRF rail to the haptic rail — fix before powering.
3. Inspect SDA/SCL between XIAO D4/D5 and the mux SCL/SDA pins; verify
   not crossed.

Then plug USB, watch the UART log, and confirm:

- One `*** Booting Zephyr OS build ***` line (not multiple — that means
  brownout reboots).
- Four `DA7280 @0x4a initialized: LRA, 170 Hz …` lines.
- `haptics: SELFTEST mode (4/4 channels ready)`.
- T0 reports `rev=0xba` for all four channels (the chip's WHO_AM_I).

Anything short of 4/4 means a mux channel is mis-wired or a DA7280 board
isn't seated.
