"""Viser scene construction for the HapNav pin visualizer.

Scene hierarchy:

  /world                         world origin + reference grid
  /pin                           pin body frame  (wxyz follows BNO055)
    /pin/imu                     BNO055 cosmetic breakout
      /pin/imu/mesh
      /pin/imu/sensor            chip-aperture sub-frame
    /pin/tof_front               VL53L5CX 8×8 mount on pin face
      /pin/tof_front/mesh
      /pin/tof_front/sensor
        /pin/tof_front/sensor/rays/ray_N
        /pin/tof_front/sensor/points
    /pin/tof_head                VL53L1X single-zone +20° up
      /pin/tof_head/mesh
      /pin/tof_head/sensor
        /pin/tof_head/sensor/ray
        /pin/tof_head/sensor/point
    /pin/tof_down                VL53L1X single-zone 45° down
      /pin/tof_down/mesh
      /pin/tof_down/sensor
        /pin/tof_down/sensor/ray
        /pin/tof_down/sensor/point

Everything below /pin moves with the IMU rotation — the front grid's point
cloud, head/down hit points, and all rays are children of their respective
sensor frames, so they pick up parent transforms for free.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image
import trimesh
import viser

from . import config
from .config import IMUMount, SensorKind, ToFMount
from .geometry import ZoneAngles


@dataclass
class ToFHandles:
    """Per-ToF scene handles."""

    mount: ToFMount
    board: viser.FrameHandle
    mesh: viser.MeshHandle
    sensor: viser.FrameHandle
    rays: list = field(default_factory=list)  # one ray for SINGLE_ZONE, 64 for GRID_8X8
    point_path: str = ""                       # set on creation; updated by viewer


@dataclass
class SceneHandles:
    """Top-level scene handles."""

    pin: viser.FrameHandle
    imu_board: viser.FrameHandle
    imu_mesh: viser.MeshHandle
    imu_sensor: viser.FrameHandle
    tofs: list[ToFHandles]


# ────────────────────────────────────────────────────────────────────────────
# Reference grid (floor plane).
# ────────────────────────────────────────────────────────────────────────────

def create_grid(server: viser.ViserServer, size: float = 2.0) -> list:
    handles = []
    for i in range(-10, 11):
        x_line = server.scene.add_spline_catmull_rom(
            f"/world/grid/line_x_{i}",
            positions=np.array([[-size, i * 0.2, 0], [size, i * 0.2, 0]]),
            color=(160, 160, 160),
            line_width=1.0,
        )
        y_line = server.scene.add_spline_catmull_rom(
            f"/world/grid/line_y_{i}",
            positions=np.array([[i * 0.2, -size, 0], [i * 0.2, size, 0]]),
            color=(160, 160, 160),
            line_width=1.0,
        )
        handles += [x_line, y_line]
    return handles


# ────────────────────────────────────────────────────────────────────────────
# Board mesh helper — shared between IMU + all ToFs.
# ────────────────────────────────────────────────────────────────────────────

def _board_mesh(
    server: viser.ViserServer,
    scene_path: str,
    dimensions: tuple[float, float, float],
    texture_name: str,
    is_atlas: bool,
    fallback_color: tuple[int, int, int, int],
    assets_dir: Path,
) -> viser.MeshHandle:
    width, length, height = dimensions
    mesh = trimesh.creation.box(extents=[width, length, height])

    texture_path = assets_dir / texture_name
    if texture_path.exists():
        texture_image = Image.open(texture_path)
        uv = np.zeros((len(mesh.vertices), 2))
        for i, v in enumerate(mesh.vertices):
            u = (v[0] + width / 2) / width
            v_coord = (v[1] + length / 2) / length
            if is_atlas:
                # Vertical atlas: top half on top face, bottom half on bottom.
                if v[2] > 0:
                    v_coord = 0.5 + v_coord * 0.5
                else:
                    v_coord = v_coord * 0.5
            uv[i, 0] = u
            uv[i, 1] = v_coord
        mesh.visual = trimesh.visual.TextureVisuals(
            uv=uv,
            material=trimesh.visual.material.PBRMaterial(
                baseColorTexture=texture_image,
                metallicFactor=0.0,
                roughnessFactor=1.0,
            ),
        )
    else:
        mesh.visual.face_colors = fallback_color

    return server.scene.add_mesh_trimesh(scene_path, mesh=mesh)


# ────────────────────────────────────────────────────────────────────────────
# Front grid: 64 zone rays in sensor-local coordinates.
# ────────────────────────────────────────────────────────────────────────────

def _create_grid_rays(
    server: viser.ViserServer,
    parent_path: str,
    zone_angles: ZoneAngles,
) -> list:
    min_z = config.MIN_RANGE_MM / 1000.0
    max_z = config.MAX_RANGE_MM / 1000.0

    rays = []
    for i in range(config.NUM_ZONES):
        start = [
            min_z * zone_angles.tan_x[i],
            min_z * zone_angles.tan_y[i],
            min_z,
        ]
        end = [
            max_z * zone_angles.tan_x[i],
            max_z * zone_angles.tan_y[i],
            max_z,
        ]
        rays.append(server.scene.add_spline_catmull_rom(
            f"{parent_path}/rays/ray_{i}",
            positions=np.array([start, end], dtype=np.float32),
            color=(100, 150, 255),
            line_width=1.0,
        ))
    return rays


# ────────────────────────────────────────────────────────────────────────────
# Single-zone ToF: one ray along +Z_sensor (optical axis).
# ────────────────────────────────────────────────────────────────────────────

def _create_single_zone_ray(
    server: viser.ViserServer,
    parent_path: str,
    color: tuple[int, int, int],
) -> list:
    min_z = config.SINGLE_ZONE_MIN_MM / 1000.0
    max_z = config.SINGLE_ZONE_MAX_MM / 1000.0
    ray = server.scene.add_spline_catmull_rom(
        f"{parent_path}/ray",
        positions=np.array([[0, 0, min_z], [0, 0, max_z]], dtype=np.float32),
        color=color,
        line_width=2.0,
    )
    return [ray]


# ────────────────────────────────────────────────────────────────────────────
# Top-level hierarchy.
# ────────────────────────────────────────────────────────────────────────────

def create_scene_hierarchy(
    server: viser.ViserServer,
    assets_dir: Path,
    zone_angles: ZoneAngles,
) -> SceneHandles:
    # World root + reference grid live under /world (camera anchor is /world/origin).
    server.scene.add_frame("/world", show_axes=False)
    server.scene.add_frame("/world/origin", axes_length=0.02, axes_radius=0.001)

    # Pin body frame — anchored at PIN_WORLD_POSITION, rotated by IMU.
    pin = server.scene.add_frame(
        "/pin",
        show_axes=True,
        axes_length=0.03,
        axes_radius=0.0015,
        position=config.PIN_WORLD_POSITION,
        wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    # IMU breakout.
    imu_board = server.scene.add_frame(
        "/pin/imu",
        show_axes=False,
        position=tuple(np.array(config.IMU.pin_position) - np.array(config.IMU.sensor_offset)),
        wxyz=config.IMU.pin_orientation_wxyz,
    )
    imu_mesh = _board_mesh(
        server,
        scene_path="/pin/imu/mesh",
        dimensions=config.IMU.dimensions,
        texture_name=config.IMU.texture,
        is_atlas=config.IMU.is_atlas,
        fallback_color=config.IMU.fallback_color,
        assets_dir=assets_dir,
    )
    imu_sensor = server.scene.add_frame(
        "/pin/imu/sensor",
        show_axes=True,
        axes_length=0.008,
        axes_radius=0.0008,
        position=config.IMU.sensor_offset,
    )

    # Three ToFs.
    tofs: list[ToFHandles] = []
    for mount in config.TOFS:
        path = slug_path(mount.name)
        board_path = f"/pin/{path}"
        mesh_path = f"{board_path}/mesh"
        sensor_path = f"{board_path}/sensor"

        # Board frame: the board's *centre* is offset from the sensor active
        # point so the texture sits behind the optical axis, not on top of it.
        board_position = tuple(
            np.array(mount.pin_position)
            - _rotate(mount.pin_orientation_wxyz, mount.sensor_offset)
        )
        board = server.scene.add_frame(
            board_path,
            show_axes=False,
            position=board_position,
            wxyz=mount.pin_orientation_wxyz,
        )
        mesh = _board_mesh(
            server,
            scene_path=mesh_path,
            dimensions=mount.dimensions,
            texture_name=mount.texture,
            is_atlas=mount.is_atlas,
            fallback_color=mount.fallback_color,
            assets_dir=assets_dir,
        )

        # Sensor frame: sensor's optical centre at mount.pin_position with
        # the same orientation as the board frame (so rays go straight out
        # along +Z_sensor).
        sensor = server.scene.add_frame(
            sensor_path,
            show_axes=True,
            axes_length=0.008,
            axes_radius=0.0008,
            position=mount.sensor_offset,
        )

        if mount.kind is SensorKind.GRID_8X8:
            rays = _create_grid_rays(server, sensor_path, zone_angles)
            point_path = f"{sensor_path}/points"
        else:
            color = mount.fallback_color[:3] if mount.fallback_color else (255, 200, 80)
            rays = _create_single_zone_ray(server, sensor_path, color)
            point_path = f"{sensor_path}/point"

        tofs.append(ToFHandles(
            mount=mount,
            board=board,
            mesh=mesh,
            sensor=sensor,
            rays=rays,
            point_path=point_path,
        ))

    return SceneHandles(
        pin=pin,
        imu_board=imu_board,
        imu_mesh=imu_mesh,
        imu_sensor=imu_sensor,
        tofs=tofs,
    )


# ────────────────────────────────────────────────────────────────────────────
# Front-grid ray updater (visible toggle + optional clip-to-measurement).
# ────────────────────────────────────────────────────────────────────────────

def update_grid_rays(
    server: viser.ViserServer,
    tof: ToFHandles,
    zone_angles: ZoneAngles,
    visible: bool,
    distances_mm: np.ndarray | None = None,
) -> list:
    """Recreate the front grid's 64 rays, optionally clipping each to the
    last-measured distance. Returns the new handles; the caller should
    replace its stored list with the result."""

    if tof.mount.kind is not SensorKind.GRID_8X8:
        return tof.rays

    parent_path = f"/pin/{slug_path(tof.mount.name)}/sensor"
    min_z = config.MIN_RANGE_MM / 1000.0
    max_z = config.MAX_RANGE_MM / 1000.0

    new_rays = []
    for i in range(config.NUM_ZONES):
        if distances_mm is not None and distances_mm[i] >= config.MIN_RANGE_MM:
            end_z = min(distances_mm[i] / 1000.0, max_z)
        else:
            end_z = max_z

        start = [
            min_z * zone_angles.tan_x[i],
            min_z * zone_angles.tan_y[i],
            min_z,
        ]
        end = [
            end_z * zone_angles.tan_x[i],
            end_z * zone_angles.tan_y[i],
            end_z,
        ]
        new_rays.append(server.scene.add_spline_catmull_rom(
            f"{parent_path}/rays/ray_{i}",
            positions=np.array([start, end], dtype=np.float32),
            color=(100, 150, 255),
            line_width=1.0,
            visible=visible,
        ))
    tof.rays = new_rays
    return new_rays


def update_single_zone_ray(
    server: viser.ViserServer,
    tof: ToFHandles,
    visible: bool,
    distance_mm: float,
) -> None:
    """Update a single-zone ToF's ray to reflect the latest measurement."""
    if tof.mount.kind is not SensorKind.SINGLE_ZONE:
        return

    parent_path = f"/pin/{slug_path(tof.mount.name)}/sensor"
    min_z = config.SINGLE_ZONE_MIN_MM / 1000.0
    if distance_mm is not None and distance_mm >= config.SINGLE_ZONE_MIN_MM:
        end_z = min(distance_mm / 1000.0, config.SINGLE_ZONE_MAX_MM / 1000.0)
    else:
        end_z = config.SINGLE_ZONE_MAX_MM / 1000.0

    color = tof.mount.fallback_color[:3] if tof.mount.fallback_color else (255, 200, 80)
    ray = server.scene.add_spline_catmull_rom(
        f"{parent_path}/ray",
        positions=np.array([[0, 0, min_z], [0, 0, end_z]], dtype=np.float32),
        color=color,
        line_width=2.0,
        visible=visible,
    )
    tof.rays = [ray]


# ────────────────────────────────────────────────────────────────────────────
# Helpers.
# ────────────────────────────────────────────────────────────────────────────

def slug_path(name: str) -> str:
    """Cheap slug for use as a Viser scene-path component."""
    s = name.lower()
    for ch in " ,()°":
        s = s.replace(ch, "_")
    while "__" in s:
        s = s.replace("__", "_")
    return s.strip("_")


def _rotate(wxyz: tuple[float, float, float, float], vec: tuple[float, float, float]) -> np.ndarray:
    """Rotate a 3-vector by a unit WXYZ quaternion."""
    w, x, y, z = wxyz
    vx, vy, vz = vec
    # quaternion rotation: v' = q * v * q⁻¹
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.array([
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ])
