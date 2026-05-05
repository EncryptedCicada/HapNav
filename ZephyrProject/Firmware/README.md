# HapNav Firmware

Two Zephyr applications that share a small library tree.

```
Firmware/
├── lib/ble/         shared GATT UUIDs and packet structs (header-only)
├── lib/sensors/     Zephyr glue around STMicro lsm6dso/lis2mdl drivers
├── pin/             XIAO ESP32-S3 chest pin (BLE central + sensors)
└── wristband/       XIAO nRF52840 haptic wristband (BLE peripheral)
```

The vendor STMicro `*_reg.{c,h}` files live outside this tree at
`Implementation_CodeBase/Drivers/{lsm6dso,lis2mdl}-pid/` and are pulled in
by `lib/sensors/CMakeLists.txt`.

## Build

Enter the toolchain shell first:

```zsh
nix develop ~/nixos-dotfiles#zephyr
```

### Pin (ESP32-S3)

```zsh
cd Implementation_CodeBase/Firmware/pin
west build -p always -b xiao_esp32s3 .
west flash
```

### Wristband (nRF52840)

```zsh
cd Implementation_CodeBase/Firmware/wristband
west build -p always -b xiao_ble .
west flash
```

## Runtime behaviour

After both boards are flashed:

1. Wristband boots, advertises as `HapNav-Wrist`.
2. Pin boots, scans, finds the wristband by service UUID, connects.
3. Pin discovers the two characteristics (Blink Rate, IMU Data) and caches handles.
4. Pin reads LSM6DSO + LIS2MDL at 20 Hz and streams 22-byte samples.
5. Wristband logs each sample to its serial console (a/g/m in engineering units).
6. Every 5 s the pin also bumps the wristband's blink rate (1 → 2 → … → 5 → 1).

## I2C addresses on the pin

| Sensor   | Addr | Notes                           |
|----------|------|---------------------------------|
| LSM6DSO  | 0x6B | Sparkfun Qwiic default (SA0=1)  |
| LIS2MDL  | 0x1E | Fixed                           |

If your LSM6DSO is at 0x6A instead, change `LSM6DSO_I2C_ADDR` in
`pin/src/sensors.c`.
