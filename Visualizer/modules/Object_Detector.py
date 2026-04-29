"""
tof_detection/object_detector.py
=================================
Object detection pipeline for VL53L5CX 8x8 ToF sensor with LSM6DSO/LIS2MDL IMU.

Pipeline stages:
  1. ToFFilter       – Hysteresis spike rejection + Weighted Moving Average smoothing
  2. GroundPlane     – RANSAC plane fit in world-frame 3D points (IMU-compensated)
  3. RegionGrower    – BFS seed-region-growing on 8x8 grid with dynamic depth threshold
  4. ObjectTracker   – Frame-to-frame greedy nearest-neighbour tracking + persistence gate
  5. HapticMapper    – Distance + TTC → intensity + pattern commands

Usage (drop into your serial read loop):
    from tof_detection.object_detector import ObjectDetector

    detector = ObjectDetector(sensor_height_m=1.1, fps=15)

    while True:
        line = ser.readline().decode().strip()
        result = detector.process_json(line)

        # result keys: filtered, ground_mask, obstacle_mask,
        #              clusters, tracks, haptic, frame
        for cmd in result['haptic']:
            print(cmd)  # send to haptic driver
"""

import json
import numpy as np
from collections import deque

try:
    from scipy.spatial.transform import Rotation as _Rotation
    _SCIPY = True
except ImportError:
    _SCIPY = False

# ──────────────────────────────────────────────────────────────────────────────
# Sensor geometry constants
# ──────────────────────────────────────────────────────────────────────────────

GRID_N      = 8          # 8x8 grid
NUM_ZONES   = 64
VALID_STATUS = 5          # VL53L5CX "ranging valid" status code
FOV_DEG     = 45.0        # VL53L5CX 8x8 FoV per axis (±22.5°)

# Precomputed per-zone unit vectors in sensor frame.
# Convention: z forward (away from sensor face), x right, y up.
# Zone ordering: row-major, row 0 = top row.
def _build_zone_geometry():
    step   = FOV_DEG / GRID_N
    half   = FOV_DEG / 2.0 - step / 2.0
    rows   = np.arange(GRID_N)
    cols   = np.arange(GRID_N)
    # row angle: positive = downward in sensor frame → negate for y
    row_degs = (rows * step - half)          # shape (8,)
    col_degs = (cols * step - half)          # shape (8,)
    row_rads, col_rads = np.deg2rad(row_degs), np.deg2rad(col_degs)

    # outer product to get all (row, col) combinations, row-major
    R, C = np.meshgrid(row_rads, col_rads, indexing='ij')   # (8,8)
    x =  np.sin(C)
    y = -np.sin(R)   # negative: row↓ → y↓
    z =  np.cos(R) * np.cos(C)
    uvec = np.stack([x, y, z], axis=-1).reshape(NUM_ZONES, 3)  # (64,3)
    return uvec

ZONE_UVEC = _build_zone_geometry()   # (64, 3) unit vectors, sensor frame


def _quat_rotate(points, quat_wxyz):
    """
    Rotate an (N,3) array of points by a quaternion [w, x, y, z].
    Uses scipy if available, otherwise a pure-numpy Rodriguez formula.
    """
    w, x, y, z = quat_wxyz
    if _SCIPY:
        rot = _Rotation.from_quat([x, y, z, w])   # scipy: (x,y,z,w)
        return rot.apply(points)
    else:
        # Rodrigues via quaternion sandwich product (pure numpy)
        q = np.array([x, y, z], dtype=float)
        q_norm = np.linalg.norm(q)
        if q_norm < 1e-9:
            return points.copy()
        q = q / q_norm
        t = 2.0 * np.cross(q, points)                    # (N,3)
        return points + w * t + np.cross(q, t)


# ──────────────────────────────────────────────────────────────────────────────
# Stage 1: ToF Filter
# ──────────────────────────────────────────────────────────────────────────────

class ToFFilter:
    """
    Per-zone filter combining:
      • Hysteresis spike rejection – single-frame jumps > ``hysteresis_mm``
        are discarded and replaced with the previous valid reading.
      • Weighted Moving Average (WMA) – linearly increasing weights so the
        most recent sample dominates while older samples smooth noise.

    Parameters
    ----------
    wma_window : int
        Number of frames used in the WMA (3–7 is typical).
    hysteresis_mm : float
        Maximum acceptable single-frame distance change in mm.
        Typical: 30–60 mm for a static scene; raise if tracking fast motion.
    max_stale_frames : int
        How many frames an invalid zone is held at its last value before
        being marked NaN again (prevents ghost readings).
    """

    def __init__(self, n_zones=NUM_ZONES, wma_window=5,
                 hysteresis_mm=40, max_stale_frames=8):
        self.wma_window      = wma_window
        self.hysteresis_mm   = hysteresis_mm
        self.max_stale_frames = max_stale_frames

        self._hist      = [deque(maxlen=wma_window) for _ in range(n_zones)]
        self._last_val  = np.full(n_zones, np.nan)
        self._stale_cnt = np.zeros(n_zones, dtype=int)

    @staticmethod
    def _wma_weights(n):
        w = np.arange(1, n + 1, dtype=float)
        return w / w.sum()

    def update(self, distances, statuses):
        """
        Parameters
        ----------
        distances : array-like (64,)  raw mm readings
        statuses  : array-like (64,)  VL53L5CX status codes

        Returns
        -------
        filtered : np.ndarray (64,)  smoothed mm values, NaN = unknown
        """
        distances = np.asarray(distances, dtype=float)
        statuses  = np.asarray(statuses,  dtype=int)
        out       = np.full(NUM_ZONES, np.nan)

        for i in range(NUM_ZONES):
            if statuses[i] == VALID_STATUS and distances[i] > 0:
                raw = distances[i]

                # Spike gate
                if not np.isnan(self._last_val[i]):
                    if abs(raw - self._last_val[i]) > self.hysteresis_mm:
                        raw = self._last_val[i]   # reject spike, hold last

                self._hist[i].append(raw)
                self._last_val[i]  = raw
                self._stale_cnt[i] = 0

                h = np.array(self._hist[i])
                out[i] = np.dot(h, self._wma_weights(len(h)))
            else:
                # Invalid measurement
                self._stale_cnt[i] += 1
                if (not np.isnan(self._last_val[i]) and
                        self._stale_cnt[i] <= self.max_stale_frames):
                    out[i] = self._last_val[i]
                # else stays NaN

        return out


# ──────────────────────────────────────────────────────────────────────────────
# Stage 2: Ground Plane Estimator
# ──────────────────────────────────────────────────────────────────────────────

class GroundPlaneEstimator:
    """
    Estimates the ground plane in world-frame 3D space using a lightweight
    RANSAC over the 64 ToF measurements, with IMU quaternion compensation.

    How it works
    ------------
    1. Each valid zone's distance is turned into a 3-D point in sensor frame
       using the precomputed unit vectors.
    2. The quaternion rotates those points into the world frame.
    3. RANSAC fits a plane (3-point sample → normal + offset).
    4. Inliers (points within ``inlier_thresh_mm`` of the best plane) are
       labelled as ground; everything else is a potential obstacle.

    The estimated plane is cached and reused when < ``min_valid`` valid points
    are present in the current frame.

    Parameters
    ----------
    sensor_height_m : float
        Nominal height of the sensor above ground (used to seed normal bias).
    ransac_iters : int
        RANSAC iterations. 30–60 is enough for 64 points.
    inlier_thresh_mm : float
        Point-to-plane distance below which a zone counts as ground.
    min_valid : int
        Minimum valid zones required to run RANSAC this frame.
    """

    def __init__(self, sensor_height_m=1.0, ransac_iters=50,
                 inlier_thresh_mm=80, min_valid=8):
        self.sensor_height_m  = sensor_height_m
        self.ransac_iters     = ransac_iters
        self.inlier_thresh_mm = inlier_thresh_mm
        self.min_valid        = min_valid

        # Cached plane (normal, d) in world frame
        self._plane_normal = np.array([0.0, 1.0, 0.0])
        self._plane_d      = -sensor_height_m * 1000.0   # mm
        self._plane_valid  = False

    def _zones_to_world(self, distances, quat_wxyz):
        """Returns (world_points, valid_indices)."""
        valid = np.where(~np.isnan(distances) & (distances > 0))[0]
        if len(valid) == 0:
            return None, valid

        pts_sensor = ZONE_UVEC[valid] * distances[valid, np.newaxis]  # (M,3) mm
        pts_world  = _quat_rotate(pts_sensor, quat_wxyz)
        return pts_world, valid

    def classify(self, distances, quat_wxyz):
        """
        Returns
        -------
        ground_mask : np.ndarray bool (64,)  True = ground zone
        """
        pts_world, valid = self._zones_to_world(distances, quat_wxyz)
        ground_mask = np.zeros(NUM_ZONES, dtype=bool)

        if pts_world is None or len(valid) < self.min_valid:
            # Fall back to cached plane
            if self._plane_valid and pts_world is not None:
                dists = np.abs(pts_world @ self._plane_normal + self._plane_d)
                ground_mask[valid[dists < self.inlier_thresh_mm]] = True
            return ground_mask

        # ── RANSAC ────────────────────────────────────────────────────────────
        n          = len(pts_world)
        best_n     = 0
        best_mask  = None
        best_norm  = self._plane_normal.copy()
        best_d     = self._plane_d

        rng = np.random.default_rng()
        for _ in range(self.ransac_iters):
            if n < 3:
                break
            idx = rng.choice(n, 3, replace=False)
            v1  = pts_world[idx[1]] - pts_world[idx[0]]
            v2  = pts_world[idx[2]] - pts_world[idx[0]]
            nm  = np.cross(v1, v2)
            nm_len = np.linalg.norm(nm)
            if nm_len < 1e-6:
                continue
            nm /= nm_len
            if nm[1] < 0:   # prefer upward-facing normal (world y = up)
                nm = -nm
            d   = -np.dot(nm, pts_world[idx[0]])

            dists   = np.abs(pts_world @ nm + d)
            inliers = dists < self.inlier_thresh_mm

            if inliers.sum() > best_n:
                best_n    = inliers.sum()
                best_mask = inliers.copy()
                best_norm = nm
                best_d    = d

        # Refit on all inliers for a better plane (least-squares)
        if best_mask is not None and best_mask.sum() >= 3:
            inlier_pts = pts_world[best_mask]
            centroid   = inlier_pts.mean(axis=0)
            _, _, Vt   = np.linalg.svd(inlier_pts - centroid)
            best_norm  = Vt[-1]
            if best_norm[1] < 0:
                best_norm = -best_norm
            best_d = -np.dot(best_norm, centroid)

        self._plane_normal = best_norm
        self._plane_d      = best_d
        self._plane_valid  = True

        # Map inlier mask back to 64-zone array
        if best_mask is not None:
            ground_mask[valid[best_mask]] = True

        return ground_mask


# ──────────────────────────────────────────────────────────────────────────────
# Stage 3: Region Grower
# ──────────────────────────────────────────────────────────────────────────────

class RegionGrower:
    """
    BFS seed-region-growing on the 8x8 grid.

    For each unvisited obstacle zone (seeds ordered closest-first),
    BFS expands to 4-connected neighbours whose depth difference falls
    within a dynamic threshold that scales with distance.

    Parameters
    ----------
    base_threshold_mm : float
        Depth-difference threshold at 1 m. Scaled linearly with range so
        far objects (which are noisier) get a looser gate.
    min_cluster_size : int
        Clusters smaller than this are discarded as noise.
    max_clusters : int
        Hard cap on returned clusters (prevents runaway on cluttered scenes).
    use_8_connected : bool
        If True, also expand to diagonal neighbours (8-connected).
    """

    def __init__(self, base_threshold_mm=150, min_cluster_size=2,
                 max_clusters=12, use_8_connected=False):
        self.base_threshold_mm = base_threshold_mm
        self.min_cluster_size  = min_cluster_size
        self.max_clusters      = max_clusters
        self._8conn            = use_8_connected

        # Precompute neighbour lookup
        self._nbrs = self._build_nbr_table()

    def _build_nbr_table(self):
        dirs4 = [(-1,0),(1,0),(0,-1),(0,1)]
        dirs8 = dirs4 + [(-1,-1),(-1,1),(1,-1),(1,1)]
        dirs  = dirs8 if self._8conn else dirs4
        table = []
        for z in range(NUM_ZONES):
            r, c = divmod(z, GRID_N)
            nbrs = []
            for dr, dc in dirs:
                r2, c2 = r+dr, c+dc
                if 0 <= r2 < GRID_N and 0 <= c2 < GRID_N:
                    nbrs.append(r2 * GRID_N + c2)
            table.append(nbrs)
        return table

    def _threshold(self, depth_mm):
        """Dynamic depth threshold – linear scale with range."""
        return self.base_threshold_mm * max(1.0, depth_mm / 1000.0)

    def grow(self, distances, obstacle_mask):
        """
        Parameters
        ----------
        distances     : np.ndarray (64,)  filtered mm values
        obstacle_mask : np.ndarray bool (64,)

        Returns
        -------
        clusters : list of dict, each containing:
            indices      – list of zone indices in cluster
            centroid_idx – zone closest to mean depth
            mean_dist    – mean depth mm
            min_dist     – nearest zone depth mm
            size         – number of zones
            bbox         – (row_min, col_min, row_max, col_max)
        """
        eligible = np.where(obstacle_mask & ~np.isnan(distances))[0]
        if len(eligible) == 0:
            return []

        # Seed order: closest first (more likely to be real obstacles)
        seeds_order = eligible[np.argsort(distances[eligible])]
        unvisited   = set(eligible.tolist())
        clusters    = []

        for seed in seeds_order:
            if seed not in unvisited:
                continue

            cluster = []
            queue   = deque([seed])
            unvisited.discard(seed)

            while queue:
                cur = queue.popleft()
                cluster.append(cur)
                d_cur = distances[cur]

                for nb in self._nbrs[cur]:
                    if nb not in unvisited:
                        continue
                    d_nb = distances[nb]
                    if np.isnan(d_nb):
                        continue
                    if abs(d_cur - d_nb) < self._threshold(max(d_cur, d_nb)):
                        unvisited.discard(nb)
                        queue.append(nb)

            if len(cluster) < self.min_cluster_size:
                continue

            c_arr = np.array(cluster)
            c_d   = distances[c_arr]
            mean_d = float(np.nanmean(c_d))
            centroid = int(c_arr[np.argmin(np.abs(c_d - mean_d))])
            rows = c_arr // GRID_N
            cols = c_arr  % GRID_N

            clusters.append({
                'indices'     : cluster,
                'centroid_idx': centroid,
                'mean_dist'   : mean_d,
                'min_dist'    : float(np.nanmin(c_d)),
                'size'        : len(cluster),
                'bbox'        : (int(rows.min()), int(cols.min()),
                                 int(rows.max()), int(cols.max())),
            })

            if len(clusters) >= self.max_clusters:
                break

        return clusters


# ──────────────────────────────────────────────────────────────────────────────
# Stage 4: Object Tracker
# ──────────────────────────────────────────────────────────────────────────────

class ObjectTracker:
    """
    Greedy nearest-neighbour frame-to-frame tracker with temporal persistence.

    Each cluster from RegionGrower is matched to an existing track by
    minimising a combined metric of grid-centroid distance + depth difference.
    New clusters spawn new tracks. Tracks age out if unmatched for
    ``max_age`` frames. A track is only "confirmed" (returned as a real
    obstacle) after ``min_frames`` consecutive valid matches, suppressing
    single-frame false positives.

    Parameters
    ----------
    grid_dist_weight : float
        Weight for grid-pixel distance vs depth difference in matching cost.
    depth_dist_weight : float
        Weight for depth difference.
    max_match_cost : float
        Tracks and clusters with cost above this are not matched.
    min_frames : int
        Frames before a track is considered confirmed.
    max_age : int
        Frames of absence before a track is deleted.
    velocity_alpha : float
        EMA smoothing for velocity estimate (0=no smoothing, 1=frozen).
    """

    def __init__(self, grid_dist_weight=120.0, depth_dist_weight=0.3,
                 max_match_cost=500.0, min_frames=3,
                 max_age=6, velocity_alpha=0.3):
        self.gdw         = grid_dist_weight
        self.ddw         = depth_dist_weight
        self.max_cost    = max_match_cost
        self.min_frames  = min_frames
        self.max_age     = max_age
        self.vel_alpha   = velocity_alpha

        self._tracks  = {}   # id → track dict
        self._next_id = 0

    def _match_cost(self, cluster, track):
        cr, cc = divmod(cluster['centroid_idx'], GRID_N)
        tr, tc = divmod(track['centroid_idx'],  GRID_N)
        grid_d  = np.sqrt((cr-tr)**2 + (cc-tc)**2)
        depth_d = abs(cluster['mean_dist'] - track['mean_dist'])
        return self.gdw * grid_d + self.ddw * depth_d

    def update(self, clusters):
        """
        Parameters
        ----------
        clusters : list of dicts from RegionGrower.grow()

        Returns
        -------
        confirmed : list of track dicts (frames >= min_frames),
                    each including 'id', 'velocity_mm_f', 'confirmed'.
        """
        # Age tracks; prune dead ones
        for tid in list(self._tracks.keys()):
            self._tracks[tid]['age'] += 1
            if self._tracks[tid]['age'] > self.max_age:
                del self._tracks[tid]

        matched_tids = set()

        for cluster in clusters:
            best_tid  = None
            best_cost = self.max_cost

            for tid, track in self._tracks.items():
                if tid in matched_tids:
                    continue
                cost = self._match_cost(cluster, track)
                if cost < best_cost:
                    best_cost = cost
                    best_tid  = tid

            if best_tid is not None:
                t  = self._tracks[best_tid]
                # EMA velocity (mm per frame; negative = approaching)
                raw_vel = cluster['mean_dist'] - t['mean_dist']
                t['velocity_mm_f'] = (self.vel_alpha * t.get('velocity_mm_f', 0.0)
                                      + (1 - self.vel_alpha) * raw_vel)
                t['mean_dist']    = cluster['mean_dist']
                t['min_dist']     = cluster['min_dist']
                t['centroid_idx'] = cluster['centroid_idx']
                t['indices']      = cluster['indices']
                t['size']         = cluster['size']
                t['bbox']         = cluster['bbox']
                t['age']          = 0
                t['frames']      += 1
                t['confirmed']    = t['frames'] >= self.min_frames
                matched_tids.add(best_tid)
            else:
                # New track
                tid = self._next_id
                self._next_id += 1
                self._tracks[tid] = {
                    'id'           : tid,
                    'mean_dist'    : cluster['mean_dist'],
                    'min_dist'     : cluster['min_dist'],
                    'centroid_idx' : cluster['centroid_idx'],
                    'indices'      : cluster['indices'],
                    'size'         : cluster['size'],
                    'bbox'         : cluster['bbox'],
                    'age'          : 0,
                    'frames'       : 1,
                    'confirmed'    : False,
                    'velocity_mm_f': 0.0,
                }

        return [t for t in self._tracks.values() if t['confirmed']]


# ──────────────────────────────────────────────────────────────────────────────
# Stage 5: Haptic Mapper
# ──────────────────────────────────────────────────────────────────────────────

class HapticMapper:
    """
    Converts confirmed obstacle tracks into haptic feedback commands.

    Intensity (0.0–1.0) is driven by:
      • Proximity (inverse distance, dominates at >1 m).
      • Time-to-Collision (TTC) urgency (dominates at <3 s).

    Patterns reflect urgency:
      ─────────────────────────────────────────────────────
      TTC        Intensity   Pattern
      ─────────────────────────────────────────────────────
      < 0.8 s    ~1.0        CRITICAL  (rapid triple burst)
      0.8–2 s    high        URGENT    (fast double pulse)
      2–4 s      medium      WARNING   (single slow pulse)
      > 4 s / ∞  low         PRESENT   (continuous low hum)
      ─────────────────────────────────────────────────────

    Parameters
    ----------
    min_dist_mm, max_dist_mm : float
        Clamp range for distance-based intensity.
    fps : float
        Sensor frame rate (used to convert velocity/frame to velocity/s).
    ttc_weights : tuple
        (distance_weight, ttc_weight) for combined intensity.
    """

    PATTERN_CRITICAL = 'CRITICAL'
    PATTERN_URGENT   = 'URGENT'
    PATTERN_WARNING  = 'WARNING'
    PATTERN_PRESENT  = 'PRESENT'

    def __init__(self, min_dist_mm=150.0, max_dist_mm=2500.0,
                 fps=10.0, ttc_weights=(0.55, 0.45)):
        self.min_dist  = min_dist_mm
        self.max_dist  = max_dist_mm
        self.fps       = fps
        self.tw        = ttc_weights

    def _ttc(self, dist_mm, vel_mm_per_frame):
        """Seconds to collision. Returns inf if not approaching."""
        vel_mm_s = vel_mm_per_frame * self.fps
        if vel_mm_s >= -1e-3:      # moving away or stationary
            return float('inf')
        return dist_mm / (-vel_mm_s)

    def _dist_intensity(self, dist_mm):
        clamped = np.clip(dist_mm, self.min_dist, self.max_dist)
        return 1.0 - (clamped - self.min_dist) / (self.max_dist - self.min_dist)

    def _ttc_intensity(self, ttc_s):
        if ttc_s == float('inf'):
            return 0.0
        clamped = np.clip(ttc_s, 0.3, 5.0)
        return 1.0 - (clamped - 0.3) / (5.0 - 0.3)

    def _pattern(self, ttc_s, intensity):
        if ttc_s < 0.8:
            return self.PATTERN_CRITICAL
        if ttc_s < 2.0:
            return self.PATTERN_URGENT
        if ttc_s < 4.0 or intensity > 0.65:
            return self.PATTERN_WARNING
        return self.PATTERN_PRESENT

    def map(self, tracks):
        """
        Parameters
        ----------
        tracks : confirmed track dicts from ObjectTracker.update()

        Returns
        -------
        commands : list of dicts, sorted by intensity descending.
            Keys: id, intensity, pattern, dist_mm, ttc_s,
                  centroid_idx, bbox, size, approaching
        """
        cmds = []
        for t in tracks:
            ttc = self._ttc(t['mean_dist'], t['velocity_mm_f'])
            di  = self._dist_intensity(t['mean_dist'])
            ti  = self._ttc_intensity(ttc)
            intensity = float(np.clip(
                self.tw[0] * di + self.tw[1] * ti, 0.0, 1.0))

            cmds.append({
                'id'          : t['id'],
                'intensity'   : round(intensity, 3),
                'pattern'     : self._pattern(ttc, intensity),
                'dist_mm'     : round(t['mean_dist'], 1),
                'min_dist_mm' : round(t['min_dist'], 1),
                'ttc_s'       : round(ttc, 2) if ttc != float('inf') else None,
                'centroid_idx': t['centroid_idx'],
                'bbox'        : t['bbox'],
                'size'        : t['size'],
                'approaching' : t['velocity_mm_f'] < -5.0,
                'velocity_mms': round(t['velocity_mm_f'] * self.fps, 1),
            })

        cmds.sort(key=lambda c: c['intensity'], reverse=True)
        return cmds


# ──────────────────────────────────────────────────────────────────────────────
# Top-level pipeline
# ──────────────────────────────────────────────────────────────────────────────

class ObjectDetector:
    """
    Single entry-point pipeline.  Instantiate once; call ``process()`` or
    ``process_json()`` per serial frame.

    Parameters
    ----------
    sensor_height_m : float
        Approximate height of sensor above floor (for RANSAC plane seeding).
    fps : float
        Sensor frame rate in Hz (affects TTC calculation).
    filter_kw, ground_kw, grower_kw, tracker_kw, haptic_kw : dict
        Optional keyword overrides for each sub-component (see their
        respective class docstrings for available parameters).

    Example
    -------
    detector = ObjectDetector(sensor_height_m=1.1, fps=15)
    result   = detector.process_json(serial_line)
    for cmd in result['haptic']:
        drive_motor(cmd['intensity'], cmd['pattern'])
    """

    def __init__(self, sensor_height_m=1.0, fps=10.0,
                 filter_kw=None, ground_kw=None, grower_kw=None,
                 tracker_kw=None, haptic_kw=None):
        self.fps         = fps
        self._filter     = ToFFilter(**(filter_kw  or {}))
        self._ground     = GroundPlaneEstimator(
                               sensor_height_m=sensor_height_m,
                               **(ground_kw or {}))
        self._grower     = RegionGrower(**(grower_kw  or {}))
        self._tracker    = ObjectTracker(**(tracker_kw or {}))
        self._haptic     = HapticMapper(fps=fps, **(haptic_kw or {}))
        self.frame_count = 0

    # ── public API ────────────────────────────────────────────────────────────

    def process(self, distances_raw, statuses_raw, quat_wxyz):
        """
        Run the full pipeline for one frame.

        Parameters
        ----------
        distances_raw : array-like (64,)   raw mm readings from VL53L5CX
        statuses_raw  : array-like (64,)   status codes
        quat_wxyz     : array-like (4,)    IMU quaternion [w, x, y, z]

        Returns
        -------
        result : dict
            filtered      – np.ndarray (64,)  smoothed distances
            ground_mask   – np.ndarray bool (64,)
            obstacle_mask – np.ndarray bool (64,)
            clusters      – list of raw cluster dicts
            tracks        – list of confirmed track dicts
            haptic        – list of haptic command dicts (sorted by urgency)
            frame         – int frame counter
        """
        self.frame_count += 1
        statuses = np.asarray(statuses_raw, dtype=int)

        # 1. Filter
        filtered = self._filter.update(distances_raw, statuses)

        # 2. Ground plane classification
        ground_mask = self._ground.classify(filtered, quat_wxyz)

        # 3. Obstacle mask: valid, not ground, not NaN
        valid_mask    = (statuses == VALID_STATUS)
        obstacle_mask = valid_mask & ~ground_mask & ~np.isnan(filtered)

        # 4. Region growing
        clusters = self._grower.grow(filtered, obstacle_mask)

        # 5. Temporal tracking
        tracks = self._tracker.update(clusters)

        # 6. Haptic commands
        haptic_cmds = self._haptic.map(tracks)

        return {
            'filtered'     : filtered,
            'ground_mask'  : ground_mask,
            'obstacle_mask': obstacle_mask,
            'clusters'     : clusters,
            'tracks'       : tracks,
            'haptic'       : haptic_cmds,
            'frame'        : self.frame_count,
        }

    def process_json(self, json_str):
        """Parse a raw JSON serial frame and run the pipeline."""
        data = json.loads(json_str)
        return self.process(
            data['distances'],
            data['status'],
            data['quat'],      # [w, x, y, z]
        )
