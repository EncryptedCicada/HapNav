"""VL53L5CX sensor configuration constants."""

from dataclasses import dataclass


# Package version (semantic versioning) - must match firmware
VERSION = "0.1.0"

# Sensor resolution
RESOLUTION = 8  # 8x8 zones
NUM_ZONES = 64

# Field of view
FOV_DIAGONAL_DEG = 65.0  # Diagonal field of view in degrees

# Range limits
MAX_RANGE_MM = 4000
MIN_RANGE_MM = 20


@dataclass
class BoardConfig:
    """Configuration for a sensor board."""

    # Board position in world coordinates (meters)
    world_position: tuple[float, float, float]
    # Sensor position relative to board center (meters)
    sensor_offset: tuple[float, float, float]
    # Board physical dimensions: width, length, height (meters)
    dimensions: tuple[float, float, float]
    # Texture filename (looked up in assets directory)
    texture: str
    # Sensor yaw rotation around Z axis (degrees) - corrects sensor orientation
    sensor_yaw_deg: float = 0.0
    # Whether texture is a vertical atlas (top/bottom faces)
    is_atlas: bool = False
    # Fallback color if texture not found (RGBA)
    fallback_color: tuple[int, int, int, int] = (128, 128, 128, 255)


# IMU board: BNO08x on 15mm x 26mm breakout
# World position: at origin, sensor chip is the reference point
# Sensor offset: chip is ~4mm above board center in Y, flush with top surface in Z
IMU_BOARD = BoardConfig(
    world_position=(0.0, 0.0, 0.0),
    sensor_offset=(0.0, 0.004, 0.0005),  # Sensor above and forward of board center
    dimensions=(0.015, 0.026, 0.001),
    texture="bno08x-atlas.png",
    is_atlas=True,
    fallback_color=(128, 0, 128, 255),  # Purple
)

# ToF board: VL53L5CX on 10mm x 16mm breakout
# World position: ~1 inch (25.4mm) in -Y direction from IMU
# Sensor offset: aperture is ~4mm from top edge, flush with top surface
# Sensor yaw: 90° CCW to align sensor's internal coordinate system with world
TOF_BOARD = BoardConfig(
    world_position=(0.0, -0.0254, 0.0),
    sensor_offset=(0.0, 0.004, 0.0005),  # Sensor above and forward of board center
    dimensions=(0.010, 0.016, 0.001),
    texture="vl53l5cx-atlas.png",
    sensor_yaw_deg=90.0,  # 90° CCW rotation to align with world frame
    is_atlas=True,
    fallback_color=(0, 100, 0, 255),  # Green
)

# Visualization settings
TARGET_FPS = 30  # Target visualization frame rate
FRAME_TIME = 1.0 / TARGET_FPS  # Time per frame in seconds

# Mapping mode thresholds
DOWNSAMPLE_POINT_THRESHOLD = 500  # Trigger downsampling after this many new points
DOWNSAMPLE_BUFFER_THRESHOLD = 15  # Or after this many frame buffers

# ST-calibrated lookup tables for VL53L5CX coordinate conversion
# Source: https://community.st.com/t5/imaging-sensors/vl53l5cx-multi-zone-sensor-get-x-y-z-of-points-relative-to/td-p/172929
# These tables account for actual lens optical characteristics (non-uniform angular coverage)
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
