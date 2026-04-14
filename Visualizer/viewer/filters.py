"""Data filtering and plane fitting for VL53L5CX."""

import logging

import numpy as np
from scipy.spatial.transform import Rotation

from . import config
from .geometry import rotation_matrix_from_vectors

logger = logging.getLogger("vl53l5cx_viewer.filters")


class TemporalFilter:
    """Exponential moving average filter for distance data."""

    def __init__(self):
        self.filtered_distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self.initialized = False

    def reset(self):
        """Reset the filter state."""
        self.initialized = False

    def apply(self, distances: np.ndarray, strength: float) -> np.ndarray:
        """Apply exponential moving average filtering to distance data.

        Args:
            distances: Raw distance measurements (64 values in mm)
            strength: 0.0 = no smoothing, 1.0 = maximum smoothing

        Returns:
            Filtered distance array
        """
        # Alpha controls how much of the new value to use
        # Higher strength = more smoothing = lower alpha
        alpha = 1.0 - strength

        if not self.initialized:
            # Initialize filter buffer with first valid frame
            self.filtered_distances = distances.copy()
            self.initialized = True
            return self.filtered_distances.copy()

        # EMA formula: filtered = alpha * new + (1 - alpha) * old
        self.filtered_distances = alpha * distances + (1.0 - alpha) * self.filtered_distances

        return self.filtered_distances.copy()


def _fit_plane_from_points(
    points: np.ndarray,
    padding: float = 1.2,
) -> tuple[np.ndarray, np.ndarray, float, float] | None:
    """Fit a plane to 3D points using least squares and return visualization params.

    Internal helper that fits z = ax + by + c and returns position, orientation, size, error.

    Args:
        points: Nx3 array of 3D points (must have at least 3 points)
        padding: Multiplier for plane size (1.0 = exact fit)

    Returns:
        Tuple of (position, wxyz_quaternion, size, rmse_mm) or None if fitting fails
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Build design matrix for least squares: [x, y, 1] @ [a, b, c].T = z
    A = np.column_stack([x, y, np.ones_like(x)])

    try:
        coeffs, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
        a, b, c = coeffs
    except np.linalg.LinAlgError as e:
        logger.debug("Plane fitting failed: %s", e)
        return None

    # Plane equation: z = ax + by + c
    # Normal vector: n = (-a, -b, 1) (unnormalized)
    normal = np.array([-a, -b, 1.0])
    normal = normal / np.linalg.norm(normal)

    # Compute point-to-plane distances for RMSE
    # Distance from point (x,y,z) to plane ax + by - z + c = 0 is:
    # |ax + by - z + c| / sqrt(a^2 + b^2 + 1)
    denom = np.sqrt(a**2 + b**2 + 1)
    residuals = np.abs(a * x + b * y - z + c) / denom
    rmse_m = np.sqrt(np.mean(residuals**2))
    rmse_mm = rmse_m * 1000  # Convert to mm

    # Compute centroid of points
    centroid = points.mean(axis=0)

    # Compute plane size based on XY span of points
    x_span = x.max() - x.min()
    y_span = y.max() - y.min()
    plane_size = max(x_span, y_span) * padding

    # Ensure minimum size for visibility
    plane_size = max(plane_size, 0.05)  # At least 5cm

    # Position: centroid adjusted to lie on fitted plane
    plane_z = a * centroid[0] + b * centroid[1] + c
    position = np.array([centroid[0], centroid[1], plane_z])

    # Build rotation matrix to align Z-axis with plane normal
    rotation = rotation_matrix_from_vectors(
        np.array([0, 0, 1]),
        normal,
    )

    # Convert rotation matrix to quaternion (wxyz format)
    r = Rotation.from_matrix(rotation)
    quat_xyzw = r.as_quat()  # scipy returns xyzw
    wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

    return position, wxyz, plane_size, rmse_mm


def fit_plane(
    points: np.ndarray,
    padding: float = 1.2,
) -> tuple[np.ndarray, np.ndarray, float, float] | None:
    """Fit a plane to 3D points using least squares.

    Args:
        points: Nx3 array of valid 3D points
        padding: Multiplier for plane size (1.0 = exact fit)

    Returns:
        Tuple of (position, wxyz_quaternion, size, rmse_mm) or None if fitting fails
    """
    if len(points) < 3:
        return None
    return _fit_plane_from_points(points, padding)


def fit_plane_ransac(
    points: np.ndarray,
    threshold: float = 0.01,
    iterations: int = 100,
    padding: float = 1.2,
) -> tuple[np.ndarray, np.ndarray, float, float] | None:
    """Fit a plane to 3D points using RANSAC for robust outlier rejection.

    Args:
        points: Nx3 array of valid 3D points
        threshold: Distance threshold (meters) for a point to be considered an inlier
        iterations: Number of RANSAC iterations
        padding: Multiplier for plane size (1.0 = exact fit)

    Returns:
        Tuple of (position, wxyz_quaternion, size, rmse_mm) or None if fitting fails
    """
    if len(points) < 3:
        return None

    # Use deterministic seed based on input data for reproducible results
    # This ensures same points always produce the same fitted plane
    seed = int(np.abs(points.sum() * 1e6)) % (2**31)
    rng = np.random.default_rng(seed)

    best_inliers = None
    best_inlier_count = 0

    for _ in range(iterations):
        # Randomly sample 3 points to define a plane
        indices = rng.choice(len(points), 3, replace=False)
        sample = points[indices]

        # Compute plane from 3 points
        # Plane defined by point p0 and normal n = (p1-p0) x (p2-p0)
        p0, p1, p2 = sample
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)

        # Check for degenerate case (collinear points)
        norm = np.linalg.norm(normal)
        if norm < 1e-10:
            continue
        normal = normal / norm

        # Compute distance of all points to this plane
        # Distance = |dot(point - p0, normal)|
        distances = np.abs(np.dot(points - p0, normal))

        # Count inliers
        inlier_mask = distances < threshold
        inlier_count = np.sum(inlier_mask)

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inliers = inlier_mask

    # Need at least 3 inliers to fit a plane
    if best_inliers is None or best_inlier_count < 3:
        return None

    # Refit plane using only inliers
    inlier_points = points[best_inliers]
    return _fit_plane_from_points(inlier_points, padding)
