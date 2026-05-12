# HapNav — Haptic Navigation System for the Visually Impaired

⚠️ **This is NOT a pure software project.** HapNav requires physical hardware to build and run. You cannot simulate the full system in software alone. See the [Hardware Requirements](#hardware-requirements) section before cloning.

---

## Table of Contents

- [Overview](#overview)  
- [System Architecture](#system-architecture)  
- [Repository Structure](#repository-structure)  
- [Hardware Requirements](#hardware-requirements)  
- [Software Dependencies](#software-dependencies)  
- [Getting Started](#getting-started)  
- [Submodules](#submodules)  
- [Open Issues & Roadmap](#open-issues--roadmap)  
- [Contributing](#contributing)  
- [License](#license)

---

## Overview

HapNav is a wearable haptic navigation aid designed for visually impaired adults. The device consists of two physical components:

- A **chest-mounted pin** equipped with a Time-of-Flight (ToF) sensor array that detects obstacles in the user's path.  
- A **wrist-worn band** containing haptic actuators that translate proximity data into directional vibration feedback.

When the ToF sensor detects an obstacle within a configurable range, the system computes its angular position and distance, then drives the appropriate haptic motor on the wrist to alert the user — all in real time, with no visual or audio dependency.

**Target users:** Visually impaired adults seeking a lightweight, non-intrusive navigation supplement to white canes or guide dogs.

---

## System Architecture

```
+-----------------------------------------------+
|               Chest Unit (Pin)                |
|                                               |
|  VL53L5CX (ToF Array)                         |
|      |  8x8 depth map @ configurable rate     |
|      v                                        |
|  STM32 MCU (Zephyr RTOS)                      |
|      |  Sensor fusion: ToF + IMU + Mag        |
|      |  (LSM6DSO accel/gyro + LIS2MDL mag)    |
|      |                                        |
|      |  BLE / UART                            |
|      v                                        |
+------+----------------------------------------+
       |
       v
+-----------------------------------------------+
|               Wrist Unit (Band)               |
|                                               |
|  Haptic Driver(s)                             |
|      +-- Motor selection based on             |
|      |   obstacle angle                       |
|      +-- Intensity based on distance          |
+------+----------------------------------------+
       |
       v  (optional, for development & demo)
+-----------------------------------------------+
|        Visualizer (Host PC / Python)          |
|  Real-time ToF depth map + IMU data plot      |
+-----------------------------------------------+
```

### Data Flow

1. The VL53L5CX ToF sensor captures an 8×8 depth matrix at configurable intervals.  
2. The LSM6DSO IMU provides accelerometer and gyroscope readings for orientation correction.  
3. The LIS2MDL magnetometer provides heading data to compensate for body rotation.  
4. Sensor fusion logic on the STM32 (running Zephyr RTOS) maps the obstacle's angular position to a specific haptic driver.  
5. The haptic driver intensity is modulated based on the obstacle's proximity — closer obstacles produce stronger vibrations.  
6. Optionally, raw sensor data is streamed to the host PC for visualization and debugging via the Python Visualizer.

---

## Repository Structure

HapNav/

```
HapNav/
├── Docs/                        # Learning materials, algorithm explanations,
│                                # and component specifications
├── Reference_CodeBase/          # Simple examples to use those sensors
├── Visualizer/                  # Python-based visualization tool for
│                                # demonstrating how the ToF sensor and IMU work
├── ZephyrProject/               # Firmware for both the chest pin and the wrist band
│   ├── Drivers/                 # Git submodules for sensor drivers
│   │   ├── stm32-vl53l5cx/      # VL53L5CX ToF sensor driver (submodule)
│   │   ├── lis2mdl-pid/         # LIS2MDL magnetometer driver (submodule)
│   │   └── lsm6dso-pid/         # LSM6DSO IMU driver (submodule)
│   ├── Firmware/
│   │   ├── modules/
│   │   │   └── da7280-haptic/   # Haptic driver for Zephyr RTOS
│   │   ├── pin/                 # Chest pin firmware (ToF sensing + sensor fusion)
│   │   └── wristband/           # Wrist band firmware (haptic actuation)
│   ├── CMakeLists.txt           # Zephyr CMake build entry
│   └── prj.conf                 # Zephyr project configuration (Kconfig)
├── .gitmodules                  # Submodule definitions
└── README.md
```

### Directory Details

**`Docs/`** Contains all reference and learning materials accumulated during development — including algorithm explanations (e.g., obstacle angle computation, haptic mapping logic), component datasheets, and hardware specifications for the VL53L5CX, LSM6DSO, and LIS2MDL sensors.

**`Visualizer/`** A Python-based host tool used to demonstrate how the ToF sensor and IMU work in practice. It connects to the MCU over UART and renders the ToF depth map and IMU readings in real time. Primarily used for presentations and audience demonstrations, not required for device operation.

**`ZephyrProject/`** The core firmware of the system, split into two subdirectories targeting each physical device:

- **`pin/`** — Firmware for the chest pin. Handles ToF sensor initialization, IMU and magnetometer data acquisition, sensor fusion, and streaming processed obstacle data to the wrist band.  
- **`wristband/`** — Firmware for the wrist band. Receives obstacle data from the chest pin and drives the haptic actuators accordingly.

Both targets are written in C and built on top of Zephyr RTOS. Sensor drivers are pulled in as Git submodules under `Drivers/`.

---

## Hardware Requirements

### Required Components

| Component | Part Number | Role |
| :---- | :---- | :---- |
| MCU Board | STM32-based (e.g., NUCLEO or custom) | Main processor |
| ToF Sensor | VL53L5CX (STMicroelectronics) | Obstacle detection (8×8 depth array) |
| IMU | LSM6DSO (STMicroelectronics) | Accelerometer \+ Gyroscope |
| Magnetometer | LIS2MDL (STMicroelectronics) | Heading / orientation |
| Haptic Drivers | ERM or LRA motor drivers | Wrist vibration feedback |
| Wrist Band PCB | Custom | Houses haptic actuators |
| Chest Mount | Custom enclosure / 3D print | Houses ToF sensor \+ MCU |

### Wiring Notes

- The VL53L5CX communicates over **I2C** (address configurable via XSHUT pin).  
- The LSM6DSO and LIS2MDL also use **I2C** (or SPI for LSM6DSO depending on board config).  
- Haptic drivers are controlled via **GPIO PWM** or **I2C** depending on the driver IC chosen.  
- Refer to `Docs/` for schematics and pin assignment details.

---

## Software Dependencies

### Firmware (ZephyrProject)

| Dependency | Notes |
| :---- | :---- |
| [Zephyr RTOS](https://zephyrproject.org/) ≥ 3.x | Build system and RTOS |
| CMake ≥ 3.20 | Build toolchain |
| West (latest) | Zephyr meta-tool for workspace management |
| ARM GCC Toolchain (`zephyr-sdk`) | Cross-compiler for STM32 targets |

### Visualizer (Python)

| Dependency | Notes |
| :---- | :---- |
| Python 3.8+ | Runtime |
| `pyserial` | UART communication with MCU |
| `numpy` | Sensor data processing |
| `matplotlib` | Real-time depth map / sensor plotting |

Install Python dependencies:

pip install pyserial numpy matplotlib

---

## Getting Started

### 1\. Clone the Repository (with Submodules)

```git clone \--recurse-submodules https://github.com/EncryptedCicada/HapNav.git ```

```cd HapNav```

If you already cloned without submodules:

```git submodule update \--init \--recursive```

### 2\. Set Up the Zephyr Environment

Follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) to install West and the Zephyr SDK. Then initialize the workspace:

```west init \-l ZephyrProject```

```west update```

### 3\. Build the Firmware

Build the chest pin firmware:

```cd ZephyrProject/pin```

```west build \-b \<your\_board\> .```

Build the wrist band firmware:

```cd ZephyrProject/wristband```

```west build \-b \<your\_board\> .```

Replace `<your_board>` with your specific STM32 board identifier (e.g., `nucleo_f401re`). Board overlay files in `boards/` may need to be configured for your exact hardware setup.

### 4\. Flash to the MCU

```west flash```

Flash each firmware to its respective MCU. Ensure your board is connected via USB/SWD and the appropriate debug probe is configured in your West environment.

### 5\. Run the Visualizer (Optional)

```cd Visualizer```

```python main.py \--port /dev/ttyUSB0 \--baud 115200```

Adjust `--port` to match your system's serial port. The Visualizer renders the live ToF depth map and IMU data, useful for both development debugging and demonstrating sensor behavior to an audience.

---

## Submodules

HapNav uses three forked STMicroelectronics sensor driver libraries as Git submodules, all located under `ZephyrProject/Drivers/`:

| Submodule | Path | Description |
| :---- | :---- | :---- |
| [stm32-vl53l5cx](https://github.com/EncryptedCicada/stm32-vl53l5cx) | `ZephyrProject/Drivers/stm32-vl53l5cx` | VL53L5CX 8×8 ToF ranging sensor driver, ported for Zephyr |
| [lis2mdl-pid](https://github.com/EncryptedCicada/lis2mdl-pid) | `ZephyrProject/Drivers/lis2mdl-pid` | LIS2MDL magnetometer platform-independent driver |
| [lsm6dso-pid](https://github.com/EncryptedCicada/lsm6dso-pid) | `ZephyrProject/Drivers/lsm6dso-pid` | LSM6DSO IMU (accel \+ gyro) platform-independent driver |

All three are maintained as separate forks to allow Zephyr-specific adaptations without diverging from upstream STMicroelectronics releases.

---


## Contributing

Contributions are welcome, but please note the hardware dependency — testing any firmware changes requires the physical device. If you are contributing to the Visualizer only, a serial data replay file (if available in `Docs/`) can be used for offline testing.

1. Fork the repository  
2. Create a feature branch: `git checkout -b feature/your-feature`  
3. Commit your changes: `git commit -m "Add your feature"`  
4. Push to your fork and open a Pull Request

Please open an issue first for any significant changes to discuss the approach before implementation.

---

## License

This project does not currently specify a license. Please contact the repository owner before using, reproducing, or distributing any part of this codebase.  
