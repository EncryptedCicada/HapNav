"""HapNav pin sensor configuration.

The original viewer was built around a breadboard carrying a single VL53L5CX
and a separate BNO08x IMU. The pin is laid out differently: an integrated
BNO055 (whose firmware does the body-frame axis remap so the quaternion it
emits is already pin body → world) plus three ToFs muxed on one I²C bus —

  * FRONT  VL53L5CX  (8×8 grid)  — optical axis aligned with pin "forward"
                                   plus 8°  downtilt
  * HEAD   VL53L1X   (single)    — optical axis aligned with pin "forward"
                                   plus 20° uptilt
  * DOWN   VL53L1X   (single)    — optical axis aligned with pin "forward"
                                   plus 45° downtilt

All three ToFs render with the VL53L5CX board texture for now (a placeholder
until the proper assets/configs for the single-zone breakouts land).

Pin body frame (matches the firmware's `obstacle.c`):
  +X right, +Y back, +Z up — so "forward" is **-Y**.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np


# ────────────────────────────────────────────────────────────────────────────
# Front VL53L5CX specs (drives the 8×8 grid pipeline).
# ────────────────────────────────────────────────────────────────────────────

RESOLUTION = 8                 # 8×8 zones
NUM_ZONES = 64
FOV_DIAGONAL_DEG = 65.0        # diagonal FoV; per-axis is FoV/√2 ≈ 45.96°

# Range gate for the front grid. The firmware uses 50–2000 mm in bench mode
# (`obstacle.c:42`); we follow that here so the colour ramp and the ray cull
# share the same bounds.
MIN_RANGE_MM = 50
MAX_RANGE_MM = 2000

# VL53L1X (head + down) usable distance gate, in mm.
SINGLE_ZONE_MIN_MM = 40
SINGLE_ZONE_MAX_MM = 4000

# Visualization frame rate.
TARGET_FPS = 30
FRAME_TIME = 1.0 / TARGET_FPS

# Mapping mode thresholds (front grid only; see viewer.py).
DOWNSAMPLE_POINT_THRESHOLD = 500
DOWNSAMPLE_BUFFER_THRESHOLD = 15


# ────────────────────────────────────────────────────────────────────────────
# Sensor mount config.
# ────────────────────────────────────────────────────────────────────────────

class SensorKind(Enum):
    """How many rays a ToF emits and therefore how it's rendered."""

    GRID_8X8 = "grid_8x8"
    SINGLE_ZONE = "single_zone"


@dataclass
class ToFMount:
    """Where one ToF sits on the pin and how it's oriented.

    `pin_position` is the location of the sensor's optical centre in pin-body
    coordinates (meters).

    `pin_orientation_wxyz` rotates the sensor's local frame
    (+X right, +Y up, +Z forward = optical axis) into the pin body frame
    (+X right, +Y back, +Z up). For "forward + tilt-down θ" the canonical
    rotation is R_x(90° + θ): the first 90° around X takes sensor +Z onto
    pin -Y (= forward), and the extra θ tilts the optical axis from -Y
    toward -Z. Negative θ for uptilt.

    `sensor_offset` is the chip-aperture offset relative to the board mesh
    centre — used to place the mesh around the sensor's optical centre so
    the textured breakout looks attached to the rays, not floating.
    """

    name: str
    pin_position: tuple[float, float, float]
    pin_orientation_wxyz: tuple[float, float, float, float]
    sensor_offset: tuple[float, float, float]
    dimensions: tuple[float, float, float]
    texture: str
    kind: SensorKind
    is_atlas: bool = True
    fallback_color: tuple[int, int, int, int] = (0, 100, 0, 255)


@dataclass
class IMUMount:
    """Position of the BNO055 chip inside the pin. Purely cosmetic — the
    quaternion still drives the *whole* pin frame's rotation, not just this
    breakout. The board mesh just helps the viewer "look" like a pin."""

    name: str
    pin_position: tuple[float, float, float]
    pin_orientation_wxyz: tuple[float, float, float, float]
    sensor_offset: tuple[float, float, float]
    dimensions: tuple[float, float, float]
    texture: str
    is_atlas: bool = True
    fallback_color: tuple[int, int, int, int] = (128, 0, 128, 255)


def _x_axis_quat_deg(angle_deg: float) -> tuple[float, float, float, float]:
    """Helper: WXYZ quaternion for a rotation around the pin's +X axis."""
    a = np.deg2rad(angle_deg) / 2.0
    return (float(np.cos(a)), float(np.sin(a)), 0.0, 0.0)


# Pin anchor in the world frame. With the BENCH_MODE pin at ~17 cm above the
# desk, Z = 0.17 puts the world "floor" plane at z=0 and the pin at a
# realistic perch above it. Tune freely; rays/points are children of the pin.
PIN_WORLD_POSITION = (0.0, 0.0, 0.17)


# All three ToF breakouts re-use the VL53L5CX texture for now — when proper
# VL53L1X assets land, swap the `texture` field per mount.
_VL53L5CX_TEXTURE = "vl53l5cx-atlas.png"


# Front VL53L5CX — pin "face" with 8° downtilt.
TOF_FRONT = ToFMount(
    name="Front (VL53L5CX, 8x8)",
    pin_position=(0.0, -0.012, 0.005),
    pin_orientation_wxyz=_x_axis_quat_deg(90.0 + 8.0),
    sensor_offset=(0.0, 0.004, 0.0005),
    dimensions=(0.010, 0.016, 0.001),
    texture=_VL53L5CX_TEXTURE,
    kind=SensorKind.GRID_8X8,
)

# Head VL53L1X — top of pin face, +20° uptilt (i.e. optical axis above horizon).
TOF_HEAD = ToFMount(
    name="Head (VL53L1X, +20°)",
    pin_position=(0.0, -0.012, 0.020),
    pin_orientation_wxyz=_x_axis_quat_deg(90.0 - 20.0),
    sensor_offset=(0.0, 0.004, 0.0005),
    dimensions=(0.010, 0.016, 0.001),
    texture=_VL53L5CX_TEXTURE,
    kind=SensorKind.SINGLE_ZONE,
    fallback_color=(160, 80, 80, 255),
)

# Down VL53L1X — bottom of pin face, 45° downtilt.
TOF_DOWN = ToFMount(
    name="Down (VL53L1X, 45°)",
    pin_position=(0.0, -0.012, -0.010),
    pin_orientation_wxyz=_x_axis_quat_deg(90.0 + 45.0),
    sensor_offset=(0.0, 0.004, 0.0005),
    dimensions=(0.010, 0.016, 0.001),
    texture=_VL53L5CX_TEXTURE,
    kind=SensorKind.SINGLE_ZONE,
    fallback_color=(80, 80, 160, 255),
)

TOFS: tuple[ToFMount, ...] = (TOF_FRONT, TOF_HEAD, TOF_DOWN)


# IMU — sit it slightly behind the front ToF, inside the pin body. The chip's
# orientation here is just for the breakout mesh; the actual rotation comes
# from the BNO055 quaternion and is applied to the whole /pin frame.
#
# `sensor_offset` is zero here because the BNO055+BMP280 BFF asset is laid
# out so the board centre coincides with the chip's observation origin —
# nothing to compensate for.
IMU = IMUMount(
    name="BNO055",
    pin_position=(0.0, 0.005, 0.000),
    pin_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    sensor_offset=(0.0, 0.0, 0.0),
    dimensions=(0.015, 0.026, 0.001),
    texture="bno055+bmp280-atlas.png",
)


# ────────────────────────────────────────────────────────────────────────────
# ST-calibrated lookup tables for the VL53L5CX. Unchanged from the original —
# only the front grid uses these.
# ────────────────────────────────────────────────────────────────────────────

# fmt: off
ST_PITCH_ANGLES_DEG = [
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
    64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
    67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
    70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
    70.00, 74.90, 80.50, 85.75, 85.75, 80.50, 74.90, 70.00,
    67.50, 72.90, 77.40, 80.50, 80.50, 77.40, 72.90, 67.50,
    64.00, 70.00, 72.90, 74.90, 74.90, 72.90, 70.00, 64.00,
    59.00, 64.00, 67.50, 70.00, 70.00, 67.50, 64.00, 59.00,
]

ST_YAW_ANGLES_DEG = [
    135.00, 125.40, 113.20,  98.13,  81.87,  66.80,  54.60,  45.00,
    144.60, 135.00, 120.96, 101.31,  78.69,  59.04,  45.00,  35.40,
    156.80, 149.04, 135.00, 108.45,  71.55,  45.00,  30.96,  23.20,
    171.87, 168.69, 161.55, 135.00,  45.00,  18.45,  11.31,   8.13,
    188.13, 191.31, 198.45, 225.00, 315.00, 341.55, 348.69, 351.87,
    203.20, 210.96, 225.00, 251.55, 288.45, 315.00, 329.04, 336.80,
    215.40, 225.00, 239.04, 258.69, 281.31, 300.96, 315.00, 324.60,
    225.00, 234.60, 246.80, 261.87, 278.13, 293.20, 305.40, 315.00,
]
# fmt: on
