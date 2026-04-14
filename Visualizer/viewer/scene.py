"""Viser scene setup for VL53L5CX viewer."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image
import trimesh
import viser

from . import config
from .config import BoardConfig
from .geometry import CoordinateMethod, ZoneAngles


def _yaw_to_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    """Convert a yaw angle (rotation around Z) to wxyz quaternion."""
    yaw_rad = np.deg2rad(yaw_deg)
    w = np.cos(yaw_rad / 2)
    z = np.sin(yaw_rad / 2)
    return (float(w), 0.0, 0.0, float(z))


@dataclass
class SceneHandles:
    """Handles to the scene hierarchy.

    Hierarchy:
        /breadboard                     # Breadboard frame (world origin)
            /breadboard/imu             # IMU board frame
                /breadboard/imu/mesh    # Board mesh
                /breadboard/imu/sensor  # Sensor origin frame
            /breadboard/tof             # ToF board frame
                /breadboard/tof/mesh    # Board mesh
                /breadboard/tof/sensor  # Sensor origin frame (with yaw)
                    /breadboard/tof/sensor/rays/ray_N  # Zone rays
                    /breadboard/tof/sensor/points      # Point cloud
                    /breadboard/tof/sensor/plane       # Fitted plane
    """

    breadboard: viser.FrameHandle
    imu_board: viser.FrameHandle
    imu_mesh: viser.MeshHandle
    imu_sensor: viser.FrameHandle
    tof_board: viser.FrameHandle
    tof_mesh: viser.MeshHandle
    tof_sensor: viser.FrameHandle
    zone_rays: list


def create_grid(server: viser.ViserServer, size: float = 2.0) -> list:
    """Create a reference grid on the XY plane."""
    grid_handles = []
    for i in range(-10, 11):
        start_x = [-size, i * 0.2, 0]
        end_x = [size, i * 0.2, 0]
        handle_x = server.scene.add_spline_catmull_rom(
            f"/grid/line_x_{i}",
            positions=np.array([start_x, end_x]),
            color=(160, 160, 160),
            line_width=1.0,
        )
        grid_handles.append(handle_x)

        start_y = [i * 0.2, -size, 0]
        end_y = [i * 0.2, size, 0]
        handle_y = server.scene.add_spline_catmull_rom(
            f"/grid/line_y_{i}",
            positions=np.array([start_y, end_y]),
            color=(160, 160, 160),
            line_width=1.0,
        )
        grid_handles.append(handle_y)

    return grid_handles


def _create_board_mesh(
    server: viser.ViserServer,
    scene_path: str,
    board_config: BoardConfig,
    assets_dir: Path,
):
    """Create a board mesh at the origin of its parent frame."""
    width, length, height = board_config.dimensions
    board_mesh = trimesh.creation.box(extents=[width, length, height])

    texture_path = assets_dir / board_config.texture
    if texture_path.exists():
        texture_image = Image.open(texture_path)
        uv = np.zeros((len(board_mesh.vertices), 2))
        for i, v in enumerate(board_mesh.vertices):
            u = (v[0] + width / 2) / width
            v_coord = (v[1] + length / 2) / length

            if board_config.is_atlas:
                if v[2] > 0:
                    v_coord = 0.5 + v_coord * 0.5
                else:
                    v_coord = v_coord * 0.5

            uv[i, 0] = u
            uv[i, 1] = v_coord

        material = trimesh.visual.material.PBRMaterial(
            baseColorTexture=texture_image,
            metallicFactor=0.0,
            roughnessFactor=1.0,
        )
        board_mesh.visual = trimesh.visual.TextureVisuals(uv=uv, material=material)
    else:
        board_mesh.visual.face_colors = board_config.fallback_color

    return server.scene.add_mesh_trimesh(scene_path, mesh=board_mesh)


def create_scene_hierarchy(
    server: viser.ViserServer,
    assets_dir: Path,
    zone_angles: ZoneAngles,
) -> SceneHandles:
    """Create the complete scene hierarchy.

    The hierarchy flows: breadboard -> board -> sensor
    Points and rays are children of the sensor frame, so they automatically
    inherit the sensor's world transform.
    """
    # Breadboard frame at world origin
    breadboard = server.scene.add_frame("/breadboard", show_axes=False)

    # IMU board frame (positioned at board center in world)
    # Board center = world_position - sensor_offset (sensor is at world_position)
    imu_board_pos = tuple(
        np.array(config.IMU_BOARD.world_position) - np.array(config.IMU_BOARD.sensor_offset)
    )
    imu_board = server.scene.add_frame(
        "/breadboard/imu",
        show_axes=False,
        position=imu_board_pos,
    )
    imu_mesh = _create_board_mesh(
        server,
        scene_path="/breadboard/imu/mesh",
        board_config=config.IMU_BOARD,
        assets_dir=assets_dir,
    )
    # IMU sensor frame (at sensor_offset from board center)
    imu_sensor = server.scene.add_frame(
        "/breadboard/imu/sensor",
        show_axes=True,
        axes_length=0.01,
        axes_radius=0.001,
        position=config.IMU_BOARD.sensor_offset,
        wxyz=_yaw_to_wxyz(config.IMU_BOARD.sensor_yaw_deg),
    )

    # ToF board frame (positioned at board center in world)
    tof_board_pos = tuple(
        np.array(config.TOF_BOARD.world_position) - np.array(config.TOF_BOARD.sensor_offset)
    )
    tof_board = server.scene.add_frame(
        "/breadboard/tof",
        show_axes=False,
        position=tof_board_pos,
    )
    tof_mesh = _create_board_mesh(
        server,
        scene_path="/breadboard/tof/mesh",
        board_config=config.TOF_BOARD,
        assets_dir=assets_dir,
    )
    # ToF sensor frame (at sensor_offset from board center, with yaw correction)
    tof_sensor = server.scene.add_frame(
        "/breadboard/tof/sensor",
        show_axes=True,
        axes_length=0.01,
        axes_radius=0.001,
        position=config.TOF_BOARD.sensor_offset,
        wxyz=_yaw_to_wxyz(config.TOF_BOARD.sensor_yaw_deg),
    )

    # Zone rays as children of ToF sensor (in sensor-local coordinates)
    zone_rays = _create_zone_rays(server, zone_angles)

    return SceneHandles(
        breadboard=breadboard,
        imu_board=imu_board,
        imu_mesh=imu_mesh,
        imu_sensor=imu_sensor,
        tof_board=tof_board,
        tof_mesh=tof_mesh,
        tof_sensor=tof_sensor,
        zone_rays=zone_rays,
    )


def _create_zone_rays(
    server: viser.ViserServer,
    zone_angles: ZoneAngles,
) -> list:
    """Create zone ray visualization in sensor-local coordinates."""
    min_z = config.MIN_RANGE_MM / 1000
    max_z = config.MAX_RANGE_MM / 1000

    zone_rays = []
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
        ray = server.scene.add_spline_catmull_rom(
            f"/breadboard/tof/sensor/rays/ray_{i}",
            positions=np.array([start, end], dtype=np.float32),
            color=(100, 150, 255),
            line_width=1.0,
        )
        zone_rays.append(ray)

    return zone_rays


def update_zone_rays(
    server: viser.ViserServer,
    zone_angles: ZoneAngles,
    method: CoordinateMethod,
    visible: bool = True,
    distances: np.ndarray | None = None,
) -> list:
    """Update zone ray positions based on coordinate method.

    Args:
        server: Viser server instance.
        zone_angles: Pre-computed zone angle data.
        method: Coordinate transform method.
        visible: Whether rays should be visible.
        distances: Optional per-zone distances in mm. If provided, rays are clipped
            to the measured distance instead of MAX_RANGE_MM.

    Returns the new ray handles (the old ones become stale).
    """
    min_range = config.MIN_RANGE_MM / 1000
    max_range = config.MAX_RANGE_MM / 1000

    new_rays = []
    for i in range(config.NUM_ZONES):
        if method == CoordinateMethod.UNIFORM:
            # Use tangent-based directions
            dir_x = zone_angles.tan_x[i]
            dir_y = zone_angles.tan_y[i]
            dir_z = 1.0
        else:
            # Use ST lookup ray directions (already normalized)
            dir_x = zone_angles.st_ray_dir_x[i]
            dir_y = zone_angles.st_ray_dir_y[i]
            dir_z = zone_angles.st_ray_dir_z[i]
            # Scale to match tangent-style (z=1 convention)
            if dir_z > 0:
                dir_x = dir_x / dir_z
                dir_y = dir_y / dir_z
                dir_z = 1.0

        # Use measured distance if provided and valid, otherwise max range
        if distances is not None and distances[i] >= config.MIN_RANGE_MM:
            end_range = distances[i] / 1000
        else:
            end_range = max_range

        start = np.array([
            min_range * dir_x,
            min_range * dir_y,
            min_range * dir_z,
        ], dtype=np.float32)
        end = np.array([
            end_range * dir_x,
            end_range * dir_y,
            end_range * dir_z,
        ], dtype=np.float32)

        ray = server.scene.add_spline_catmull_rom(
            f"/breadboard/tof/sensor/rays/ray_{i}",
            positions=np.array([start, end], dtype=np.float32),
            color=(100, 150, 255),
            line_width=1.0,
            visible=visible,
        )
        new_rays.append(ray)

    return new_rays
