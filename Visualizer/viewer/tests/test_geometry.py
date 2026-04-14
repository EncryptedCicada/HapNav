"""Tests for geometry module."""

import numpy as np
import pytest

from viewer.geometry import correct_imu_to_tof_frame, rotate_points_by_quaternion, compute_zone_angles, distances_to_points


class TestCorrectImuToTofFrame:
    """Tests for IMU-to-ToF frame correction."""

    def test_applies_90_degree_clockwise_rotation(self):
        """Correction should rotate 90° clockwise around Z."""
        # Identity IMU quaternion
        identity = np.array([1, 0, 0, 0], dtype=np.float32)

        corrected = correct_imu_to_tof_frame(identity)

        # Result should be 90° clockwise around Z: [cos(-45°), 0, 0, sin(-45°)]
        expected = np.array([0.7071068, 0, 0, -0.7071068], dtype=np.float32)
        np.testing.assert_allclose(corrected, expected, atol=1e-5)

    def test_correction_composes_with_imu_rotation(self):
        """Correction should compose correctly with IMU rotation."""
        # If IMU reports 90° counterclockwise (the physical offset),
        # correction should result in identity
        imu_90_ccw = np.array([0.7071068, 0, 0, 0.7071068], dtype=np.float32)

        corrected = correct_imu_to_tof_frame(imu_90_ccw)

        # 90° CW * 90° CCW = identity
        expected_identity = np.array([1, 0, 0, 0], dtype=np.float32)
        np.testing.assert_allclose(np.abs(corrected), np.abs(expected_identity), atol=1e-5)


class TestRotatePointsByQuaternion:
    """Tests for quaternion rotation function."""

    def test_identity_quaternion_no_change(self):
        """Identity quaternion should not change points."""
        points = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
        identity_quat = np.array([1, 0, 0, 0], dtype=np.float32)  # wxyz

        rotated = rotate_points_by_quaternion(points, identity_quat)

        np.testing.assert_allclose(rotated, points, atol=1e-6)

    def test_90_degree_rotation_around_z(self):
        """90 degree rotation around Z should map X to Y."""
        points = np.array([[1, 0, 0]], dtype=np.float32)
        # 90 degrees around Z: w=cos(45°)=0.707, z=sin(45°)=0.707
        z_rot_quat = np.array([0.7071068, 0, 0, 0.7071068], dtype=np.float32)

        rotated = rotate_points_by_quaternion(points, z_rot_quat)

        np.testing.assert_allclose(rotated[0], [0, 1, 0], atol=1e-5)

    def test_180_degree_rotation_around_z(self):
        """180 degree rotation around Z should negate X and Y."""
        points = np.array([[1, 0, 0], [0, 1, 0]], dtype=np.float32)
        # 180 degrees around Z: w=0, z=1
        z_rot_quat = np.array([0, 0, 0, 1], dtype=np.float32)

        rotated = rotate_points_by_quaternion(points, z_rot_quat)

        np.testing.assert_allclose(rotated[0], [-1, 0, 0], atol=1e-5)
        np.testing.assert_allclose(rotated[1], [0, -1, 0], atol=1e-5)

    def test_preserves_point_count(self):
        """Rotation should preserve number of points."""
        points = np.random.rand(64, 3).astype(np.float32)
        quat = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float32)  # Arbitrary rotation

        rotated = rotate_points_by_quaternion(points, quat)

        assert rotated.shape == points.shape


class TestComputeZoneAngles:
    """Tests for zone angle computation."""

    def test_returns_correct_shape(self):
        """Should return arrays with 64 elements."""
        zone_angles = compute_zone_angles()

        assert zone_angles.tan_x.shape == (64,)
        assert zone_angles.tan_y.shape == (64,)
        assert zone_angles.ray_dir_x.shape == (64,)
        assert zone_angles.ray_dir_y.shape == (64,)
        assert zone_angles.ray_dir_z.shape == (64,)

    def test_ray_directions_normalized(self):
        """Ray directions should be unit vectors."""
        zone_angles = compute_zone_angles()

        norms = np.sqrt(
            zone_angles.ray_dir_x**2 +
            zone_angles.ray_dir_y**2 +
            zone_angles.ray_dir_z**2
        )

        np.testing.assert_allclose(norms, 1.0, atol=1e-6)


class TestDistancesToPoints:
    """Tests for distance to point conversion."""

    def test_converts_mm_to_meters(self):
        """Distances in mm should convert to meters for z-coordinate."""
        zone_angles = compute_zone_angles()
        distances = np.full(64, 1000.0, dtype=np.float32)  # 1000mm = 1m

        points = distances_to_points(distances, zone_angles)

        # Center zone should have z ≈ 1.0m
        center_idx = 27  # Approximately center
        assert 0.9 < points[center_idx, 2] < 1.1

    def test_output_shape(self):
        """Output should be Nx3 array."""
        zone_angles = compute_zone_angles()
        distances = np.ones(64, dtype=np.float32) * 500

        points = distances_to_points(distances, zone_angles)

        assert points.shape == (64, 3)
