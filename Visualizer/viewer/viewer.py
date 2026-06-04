#!/usr/bin/env python3
"""HapNav pin live visualizer.

Connects to the pin's USB-Serial-JTAG console (default `/dev/ttyACM1`,
matching the firmware's per-sample `PIN {...}` trace), parses the JSON
stream, and renders the three muxed ToFs + BNO055 IMU orientation in a
Viser scene.

Hierarchy / behaviour:
  /pin                anchored at config.PIN_WORLD_POSITION, wxyz = BNO055 quaternion
    /pin/imu          cosmetic BNO055 breakout
    /pin/tof_front    VL53L5CX 8×8 — point cloud + 64 zone rays
    /pin/tof_head     VL53L1X +20° — single ray + hit point
    /pin/tof_down     VL53L1X 45° — single ray + hit point
"""

import argparse
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import viser

from . import config
from .config import SensorKind
from .filters import TemporalFilter, fit_plane, fit_plane_ransac
from .geometry import (
    CoordinateMethod,
    compute_zone_angles,
    distances_to_points,
    get_colors,
)
from .logging_config import setup_logging
from .scene import (
    SceneHandles,
    ToFHandles,
    create_grid,
    create_scene_hierarchy,
    slug_path,
    update_grid_rays,
    update_single_zone_ray,
)
from .serial_reader import SerialReader

logger = logging.getLogger("hapnav_pin_viewer.main")


# Bit positions of the obstacle flag byte (mirrors firmware ble_proto.h).
FLAG_NAMES = [
    (0x01, "STATIONARY"),
    (0x02, "SENSOR_BLOCKED"),
    (0x04, "MOSTLY_INVALID"),
    (0x08, "YAW_SLEWING"),
    (0x10, "DROPOFF"),
    (0x20, "HEAD_OBSTACLE"),
]


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton-convention WXYZ quaternion product, a ⊗ b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float32)


@dataclass
class MappingState:
    """World-space accumulation for the front 8×8 grid only."""

    points: list[np.ndarray] = field(default_factory=list)
    colors: list[np.ndarray] = field(default_factory=list)
    _clear_requested: bool = False

    def request_clear(self):
        self._clear_requested = True

    def process_clear_if_requested(self) -> bool:
        if self._clear_requested:
            self._clear_requested = False
            self.points.clear()
            self.colors.clear()
            return True
        return False

    def add(self, pts: np.ndarray, cols: np.ndarray):
        self.points.append(pts)
        self.colors.append(cols)

    def get_display_data(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.points:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)
        if len(self.points) == 1:
            return self.points[0], self.colors[0]
        return np.vstack(self.points), np.vstack(self.colors)

    def total_points(self) -> int:
        return sum(len(p) for p in self.points)

    def downsample(self, voxel_size: float, max_points: int):
        if not self.points:
            return
        all_pts = np.vstack(self.points)
        all_cols = np.vstack(self.colors)
        all_pts, all_cols = voxel_downsample(all_pts, all_cols, voxel_size)
        if len(all_pts) > max_points:
            all_pts = all_pts[-max_points:]
            all_cols = all_cols[-max_points:]
        self.points = [all_pts]
        self.colors = [all_cols]


def voxel_downsample(points: np.ndarray, colors: np.ndarray, voxel_size: float):
    if len(points) == 0:
        return points, colors
    vi = np.ascontiguousarray(np.floor(points / voxel_size).astype(np.int64))
    keys = vi.view(dtype=[("x", np.int64), ("y", np.int64), ("z", np.int64)]).ravel()
    _, unique_idx = np.unique(keys, return_index=True)
    return points[unique_idx], colors[unique_idx]


class HapNavViewer:
    """Live viewer for one HapNav pin instance."""

    def __init__(self, port: str, baud: int = 115200):
        self.serial_reader = SerialReader(port, baud)
        self.zone_angles = compute_zone_angles()
        self.temporal_filter = TemporalFilter()

    # ── setup ───────────────────────────────────────────────────────────

    def _setup_scene(self, server: viser.ViserServer):
        create_grid(server)
        assets_dir = Path(__file__).parent.parent / "assets"
        self.scene: SceneHandles = create_scene_hierarchy(
            server, assets_dir, self.zone_angles
        )

    def _setup_gui(self, server: viser.ViserServer, mapping_state: MappingState):
        with server.gui.add_folder("Pin status"):
            self.fps_text = server.gui.add_text("Frames/s", initial_value="0")
            self.imu_text = server.gui.add_text("IMU", initial_value="Not connected")
            self.front_text = server.gui.add_text("Front 8×8", initial_value="Waiting…")
            self.head_text = server.gui.add_text("Head ToF (mm)", initial_value="—")
            self.down_text = server.gui.add_text("Down ToF (mm)", initial_value="—")
            self.urgency_text = server.gui.add_text("Urgency [L CL CR R]", initial_value="0 0 0 0")
            self.flags_text = server.gui.add_text("Obstacle flags", initial_value="—")

        with server.gui.add_folder("Display"):
            self.point_size = server.gui.add_slider(
                "Point Size", min=0.001, max=0.020, step=0.001, initial_value=0.005
            )
            self.show_rays = server.gui.add_checkbox("Show Zone Rays", initial_value=True)
            self.clip_rays = server.gui.add_checkbox("Clip to Measurement", initial_value=False)

            @self.show_rays.on_update
            def _(_event):
                self.clip_rays.disabled = not self.show_rays.value

            server.gui.add_markdown("---")
            self.coord_method = server.gui.add_dropdown(
                "Front grid method",
                options=[m.value for m in CoordinateMethod],
                initial_value=CoordinateMethod.UNIFORM.value,
            )

            server.gui.add_markdown("---")
            self.apply_imu = server.gui.add_checkbox(
                "Apply IMU Rotation", initial_value=True
            )
            server.gui.add_markdown("---")
            self.filter_on = server.gui.add_checkbox("Front grid filtering", initial_value=False)
            self.filter_strength = server.gui.add_slider(
                "Filter Strength", min=0.0, max=1.0, step=0.05,
                initial_value=0.5, disabled=True,
            )

            @self.filter_on.on_update
            def _(_event):
                self.filter_strength.disabled = not self.filter_on.value
                if not self.filter_on.value:
                    self.temporal_filter.reset()

            server.gui.add_markdown("---")
            self.fit_plane_on = server.gui.add_checkbox("Fit Plane (front)", initial_value=False)
            self.plane_method = server.gui.add_dropdown(
                "Plane method",
                options=["Least Squares", "RANSAC"],
                initial_value="Least Squares",
                disabled=True,
            )
            self.ransac_thr = server.gui.add_slider(
                "RANSAC Threshold (mm)", min=1, max=50, step=1,
                initial_value=10, visible=False,
            )
            self.plane_rmse = server.gui.add_text("Plane RMSE (mm)", initial_value="—")

            @self.fit_plane_on.on_update
            def _(_event):
                self.plane_method.disabled = not self.fit_plane_on.value
                self.ransac_thr.visible = (
                    self.fit_plane_on.value
                    and self.plane_method.value == "RANSAC"
                )
                if not self.fit_plane_on.value:
                    self.plane_rmse.value = "—"

            @self.plane_method.on_update
            def _(_event):
                self.ransac_thr.visible = self.plane_method.value == "RANSAC"

        with server.gui.add_folder("Mapping (front grid)"):
            self.mapping_on = server.gui.add_checkbox("Mapping Mode", initial_value=False)
            self.voxel = server.gui.add_slider(
                "Voxel Size (mm)", min=5, max=50, step=5, initial_value=10
            )
            self.max_pts = server.gui.add_slider(
                "Max Points (k)", min=10, max=500, step=10, initial_value=100
            )
            self.point_count = server.gui.add_text("Points", initial_value="0")
            clear_btn = server.gui.add_button("Clear Map")

            @clear_btn.on_click
            def _(_event):
                mapping_state.request_clear()

    # ── per-frame ───────────────────────────────────────────────────────

    def _apply_pin_rotation(self, quaternion: np.ndarray, imu_connected: bool):
        """Set /pin's wxyz to the BNO055 quaternion, after applying the
        configurable world-frame correction (`IMU_QUAT_CORRECTION_WXYZ`).

        The firmware's axis remap aligns the BNO055 output with our pin
        body frame, but the BNO055's gravity+magnetic-north world frame may
        still differ from the viewer's world frame by a fixed rotation —
        that's what the correction quat compensates for. Defaults to
        identity; tune in `config.py` if the rendered pin doesn't track its
        physical orientation.
        """
        if self.apply_imu.value and imu_connected:
            corrected = _quat_multiply(
                np.asarray(config.IMU_QUAT_CORRECTION_WXYZ, dtype=np.float32),
                quaternion,
            )
            self.scene.pin.wxyz = tuple(float(x) for x in corrected)
        else:
            self.scene.pin.wxyz = (1.0, 0.0, 0.0, 0.0)

    def _update_front_grid(
        self,
        server: viser.ViserServer,
        tof: ToFHandles,
        distances: np.ndarray,
        status: np.ndarray,
        front_seen: bool,
        mapping_state: MappingState,
        plane_handle,
    ):
        """Update the front 8×8 grid: point cloud, mapping accumulation,
        plane fit, and per-zone ray clipping."""
        if not front_seen:
            self.front_text.value = "Waiting…"
            return plane_handle

        if self.filter_on.value:
            distances = self.temporal_filter.apply(
                distances, self.filter_strength.value
            )

        method = next(
            m for m in CoordinateMethod if m.value == self.coord_method.value
        )

        # The pin's firmware emits raw VL53L5CX status codes (5 = valid).
        valid_mask = (status == 5) & (distances >= config.MIN_RANGE_MM)
        points_local = distances_to_points(distances, self.zone_angles, method)
        colors = get_colors(distances, status)

        if not np.any(valid_mask):
            self.front_text.value = "No valid pixels"
            return plane_handle

        valid_local = points_local[valid_mask].astype(np.float32)
        valid_colors = colors[valid_mask]

        if self.mapping_on.value:
            # Accumulate in pin-local space; viser's parent transform will
            # carry the IMU rotation onto the rendered points.
            mapping_state.add(valid_local, valid_colors)
            if (
                mapping_state.total_points() > config.DOWNSAMPLE_POINT_THRESHOLD
                or len(mapping_state.points) > config.DOWNSAMPLE_BUFFER_THRESHOLD
            ):
                mapping_state.downsample(self.voxel.value / 1000.0, self.max_pts.value * 1000)
            display_points, display_colors = mapping_state.get_display_data()
            self.point_count.value = f"{len(display_points):,}"
            server.scene.add_point_cloud(
                tof.point_path,
                points=display_points,
                colors=display_colors,
                point_size=self.point_size.value,
                point_shape="circle",
            )
        else:
            server.scene.add_point_cloud(
                tof.point_path,
                points=valid_local,
                colors=valid_colors,
                point_size=self.point_size.value,
                point_shape="circle",
            )

        # Plane fit on the live front-grid points (sensor-local frame).
        if self.fit_plane_on.value and len(valid_local) >= 3:
            if self.plane_method.value == "RANSAC":
                plane_fit = fit_plane_ransac(
                    valid_local, threshold=self.ransac_thr.value / 1000.0
                )
            else:
                plane_fit = fit_plane(valid_local)
            if plane_fit is not None:
                pos, wxyz, size, rmse_mm = plane_fit
                self.plane_rmse.value = f"{rmse_mm:.2f}"
                plane_path = f"/pin/{slug_path(tof.mount.name)}/sensor/plane"
                plane_handle = server.scene.add_box(
                    plane_path,
                    dimensions=(size, size, 0.0001),
                    position=pos,
                    wxyz=wxyz,
                    color=(255, 255, 0),
                    opacity=0.5,
                )

        valid_distances = distances[valid_mask]
        self.front_text.value = (
            f"{int(valid_distances.min())}–{int(valid_distances.max())} mm "
            f"({int(valid_mask.sum())}/{config.NUM_ZONES} valid)"
        )

        # Per-zone ray update.
        if self.show_rays.value:
            update_grid_rays(
                server, tof, self.zone_angles,
                visible=True,
                distances_mm=distances if self.clip_rays.value else None,
            )
        else:
            for r in tof.rays:
                r.visible = False

        return plane_handle

    def _update_single_zone(
        self,
        server: viser.ViserServer,
        tof: ToFHandles,
        distance_mm: float,
        gui_text,
    ):
        """Update a head/down ToF: ray length + single hit point."""
        # GUI label.
        gui_text.value = "—" if distance_mm < 0 else f"{int(distance_mm)}"

        # Ray.
        update_single_zone_ray(
            server, tof,
            visible=self.show_rays.value,
            distance_mm=distance_mm,
        )

        # Single hit point in sensor-local coordinates.
        if distance_mm >= config.SINGLE_ZONE_MIN_MM:
            pt_z = min(distance_mm / 1000.0, config.SINGLE_ZONE_MAX_MM / 1000.0)
            color = tof.mount.fallback_color[:3] if tof.mount.fallback_color else (255, 255, 0)
            server.scene.add_point_cloud(
                tof.point_path,
                points=np.array([[0.0, 0.0, pt_z]], dtype=np.float32),
                colors=np.array([color], dtype=np.uint8),
                point_size=max(self.point_size.value * 2.0, 0.006),
                point_shape="circle",
            )
        else:
            server.scene.remove_by_name(tof.point_path)

    def _update_overlay(self, urgency: np.ndarray, nearest_mm: int, flags: int):
        self.urgency_text.value = " ".join(str(int(v)) for v in urgency)
        active_flags = [name for bit, name in FLAG_NAMES if flags & bit]
        if not active_flags:
            label = f"0x{flags:02x}"
        else:
            label = f"0x{flags:02x}  " + " | ".join(active_flags)
        if nearest_mm >= 0:
            label += f"   near={nearest_mm} mm"
        self.flags_text.value = label

    def _process_frame(
        self,
        server: viser.ViserServer,
        mapping_state: MappingState,
        plane_handle,
    ):
        frame = self.serial_reader.get_frame()

        # Status panel.
        self.fps_text.value = f"{self.serial_reader.data_fps:.1f}"
        self.imu_text.value = "Connected" if frame["imu_connected"] else "Not connected"

        # Pin orientation.
        self._apply_pin_rotation(frame["quaternion"], frame["imu_connected"])

        # Mapping-mode clear handling (must run before adding new points).
        if mapping_state.process_clear_if_requested():
            self.point_count.value = "0"
            # Remove every front-grid mapping point cloud explicitly.
            for tof in self.scene.tofs:
                if tof.mount.kind is SensorKind.GRID_8X8:
                    server.scene.remove_by_name(tof.point_path)

        # Per-ToF updates. Mount identity (config.TOF_*) drives which gauge
        # / GUI label gets the reading.
        for tof in self.scene.tofs:
            if tof.mount is config.TOF_FRONT:
                plane_handle = self._update_front_grid(
                    server, tof,
                    frame["front_distances"], frame["front_status"],
                    frame["front_seen"],
                    mapping_state, plane_handle,
                )
            elif tof.mount is config.TOF_HEAD:
                self._update_single_zone(
                    server, tof, frame["head_distance_mm"], self.head_text
                )
            elif tof.mount is config.TOF_DOWN:
                self._update_single_zone(
                    server, tof, frame["down_distance_mm"], self.down_text
                )

        # Obstacle overlay.
        self._update_overlay(frame["urgency"], frame["nearest_mm"], frame["flags"])

        if plane_handle is not None:
            plane_handle.visible = self.fit_plane_on.value

        return plane_handle

    # ── main loop ───────────────────────────────────────────────────────

    def run(self, host: str = "0.0.0.0", port: int = 8080):
        self.serial_reader.connect()
        self.serial_reader.start()

        server = viser.ViserServer(host=host, port=port)
        logger.info("Viser server at http://localhost:%d", port)

        @server.on_client_connect
        def _on_client_connect(client: viser.ClientHandle):
            client.camera.position = (0.6, -0.6, 0.5)
            client.camera.look_at = config.PIN_WORLD_POSITION
            client.camera.up = (0.0, 0.0, 1.0)
            client.camera.near = 0.001
            client.camera.fov = 0.55

        mapping_state = MappingState()
        self._setup_scene(server)
        self._setup_gui(server, mapping_state)

        plane_handle = None
        try:
            while True:
                t0 = time.time()
                plane_handle = self._process_frame(server, mapping_state, plane_handle)
                dt = time.time() - t0
                if dt < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - dt)
        except KeyboardInterrupt:
            logger.info("Shutting down…")
        finally:
            self.serial_reader.stop()


def main():
    parser = argparse.ArgumentParser(description="HapNav pin live visualizer")
    parser.add_argument(
        "--port", "-p",
        default="/dev/ttyACM1",
        help="Serial port the pin is on (default: /dev/ttyACM1)",
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Viser server host (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--viser-port", type=int, default=8080,
        help="Viser server port (default: 8080)"
    )
    parser.add_argument(
        "--debug", "-d", action="store_true", help="Enable debug logging"
    )
    args = parser.parse_args()

    setup_logging(level=logging.DEBUG if args.debug else logging.INFO)

    viewer = HapNavViewer(port=args.port, baud=args.baud)
    viewer.run(host=args.host, port=args.viser_port)


if __name__ == "__main__":
    main()
