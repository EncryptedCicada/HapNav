#include <hapnav/obstacle.h>
#include <hapnav/dropoff.h>

#include <zephyr/logging/log.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(hapnav_obstacle, LOG_LEVEL_INF);

/* ── Geometry constants (mirror the values frozen in the design discussion) ── */

#define TOF_ROWS                 8
#define TOF_COLS                 8

/* Per-axis FoV = 63° diagonal / √2 ≈ 44.5°; half is ±22.3°. */
#define TOF_HALF_FOV_DEG         22.3f
#define TOF_DEG_PER_ZONE         (2.0f * TOF_HALF_FOV_DEG / TOF_ROWS)

/* The sensor is physically rolled by +45° around its optical axis: the
 * original main diagonal (pixel (0,0)→(7,7)) becomes the world-frame
 * horizontal line, so the four extreme directions (cardinal ±27.5°
 * about each axis) line up with up/right/down/left. The pipeline mirrors
 * the physical roll in software so the ray table stays accurate. */
#define TOF_ROLL_DEG             45.0f

/* After the roll the lateral extent is the *diagonal* of the original
 * grid — ±27.5° horizontal — split evenly across the 4 urgency bins. */
#define TOF_BIN_BOUNDARY_DEG     13.75f

#define SENSOR_DOWNTILT_DEG      8.0f
#define SENSOR_HEIGHT_M          1.35f
#define USER_HEAD_ABOVE_SENSOR_M 0.40f         /* 1.75 m head − 1.35 m sensor */
#define HEAD_CLEARANCE_M         0.30f
#define HEAD_TOP_BAND_M          (USER_HEAD_ABOVE_SENSOR_M + HEAD_CLEARANCE_M)
#define FLOOR_BAND_HIGH_M        (-1.20f)      /* below sensor; tolerates 15 cm of floor irregularity */

#define MIN_RANGE_MM             300
#define MAX_RANGE_MM             3000

#define DEG2RAD                  0.017453293f

/* ── Detection thresholds ────────────────────────────────────────────────── */

#define CLUSTER_DELTA_MM           200
#define MIN_CLUSTER_PIXELS         2
#define STATIONARY_VAR_THRESH_G2   0.0025f     /* (0.05 g)² */
#define STATIONARY_WINDOW_FRAMES   10           /* 1 s @ 10 Hz */
#define YAW_SLEW_RADPS             (60.0f * DEG2RAD)

#define TTC_FULL_URGENCY_S         1.5f
#define TTC_NO_URGENCY_S           5.0f

#define SENSOR_BLOCKED_NEAR_PIX    32          /* half the array reads sub-min */
#define MOSTLY_INVALID_VALID_PIX   8           /* fewer than 1/8 valid */

/* ── State ────────────────────────────────────────────────────────────────── */

static float   g_ray_B[HAPNAV_TOF_ZONES][3];   /* unit rays in body frame, roll + tilt baked in */
static uint8_t g_az_bin[HAPNAV_TOF_ZONES];     /* per-pixel azimuth bin after the 45° roll */

static float g_accel_mag_hist[STATIONARY_WINDOW_FRAMES];
static int   g_accel_hist_idx;
static int   g_accel_hist_count;

static int16_t g_prev_nearest_mm[4];
static bool    g_prev_valid[4];

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static inline float clamp01(float v)
{
	return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static inline uint8_t to_u8(float v01)
{
	float u = v01 * 255.0f;
	if (u < 0.0f)   return 0;
	if (u > 255.0f) return 255;
	return (uint8_t)u;
}

static void quat_to_R_WB(const float q[4], float R[3][3])
{
	float w = q[0], x = q[1], y = q[2], z = q[3];
	R[0][0] = 1.0f - 2.0f*(y*y + z*z);
	R[0][1] = 2.0f*(x*y - w*z);
	R[0][2] = 2.0f*(x*z + w*y);
	R[1][0] = 2.0f*(x*y + w*z);
	R[1][1] = 1.0f - 2.0f*(x*x + z*z);
	R[1][2] = 2.0f*(y*z - w*x);
	R[2][0] = 2.0f*(x*z - w*y);
	R[2][1] = 2.0f*(y*z + w*x);
	R[2][2] = 1.0f - 2.0f*(x*x + y*y);
}

/*
 * Build the per-zone unit ray in body frame and the azimuth-bin LUT.
 *
 * Sensor frame (S):  +X right, +Y up, +Z forward (optical axis).
 * Body frame (B):    +X right, +Y back, +Z up — so -Y_B is "user forward".
 *
 * Order of operations:
 *   1. raw (az, el) → unit ray in S (no roll, no tilt)
 *   2. R_z(+TOF_ROLL_DEG) — sensor's own 45° optical-axis roll
 *   3. S → B mapping:  X_B = X_S, Y_B = -Z_S, Z_B = Y_S
 *   4. R_x(+SENSOR_DOWNTILT_DEG) about body X — nose-down pitch
 *
 * The post-rotation azimuth (in S after step 2) decides the 4-bin
 * urgency lane each pixel feeds into; the world-frame classifier in
 * hapnav_obstacle_step() independently rejects pixels that come back
 * as floor or ceiling.
 */
static void build_ray_table(void)
{
	const float cr = cosf(TOF_ROLL_DEG     * DEG2RAD);
	const float sr = sinf(TOF_ROLL_DEG     * DEG2RAD);
	const float ct = cosf(SENSOR_DOWNTILT_DEG * DEG2RAD);
	const float st = sinf(SENSOR_DOWNTILT_DEG * DEG2RAD);

	for (int r = 0; r < TOF_ROWS; r++) {
		for (int c = 0; c < TOF_COLS; c++) {
			float az = (c - 3.5f) * TOF_DEG_PER_ZONE * DEG2RAD;
			float el = (3.5f - r) * TOF_DEG_PER_ZONE * DEG2RAD;

			float xs0 = sinf(az) * cosf(el);
			float ys0 = sinf(el);
			float zs0 = cosf(az) * cosf(el);

			/* 2. Roll about optical axis: X' = Xc − Ys, Y' = Xs + Yc */
			float xs = xs0 * cr - ys0 * sr;
			float ys = xs0 * sr + ys0 * cr;
			float zs = zs0;

			/* 3. S → B (no tilt) */
			float xb0 =  xs;
			float yb0 = -zs;
			float zb0 =  ys;

			/* 4. R_x(+θ): Y' = Y·c − Z·s, Z' = Y·s + Z·c */
			float xb = xb0;
			float yb = yb0 * ct - zb0 * st;
			float zb = yb0 * st + zb0 * ct;

			int idx = r * TOF_COLS + c;
			g_ray_B[idx][0] = xb;
			g_ray_B[idx][1] = yb;
			g_ray_B[idx][2] = zb;

			/* Post-roll sensor-frame azimuth selects the lateral bin.
			 * atan2(X_S, Z_S) — independent of elevation and of
			 * the subsequent body-frame mapping. */
			float az_rot_deg = atan2f(xs, zs) * (180.0f / 3.14159265f);
			uint8_t bin;
			if      (az_rot_deg < -TOF_BIN_BOUNDARY_DEG) bin = 0;   /* LEFT  */
			else if (az_rot_deg <  0.0f)                 bin = 1;   /* CL    */
			else if (az_rot_deg <  TOF_BIN_BOUNDARY_DEG) bin = 2;   /* CR    */
			else                                         bin = 3;   /* RIGHT */
			g_az_bin[idx] = bin;
		}
	}
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void hapnav_obstacle_init(void)
{
	build_ray_table();

	g_accel_hist_idx   = 0;
	g_accel_hist_count = 0;
	memset(g_accel_mag_hist,  0, sizeof(g_accel_mag_hist));
	memset(g_prev_nearest_mm, 0, sizeof(g_prev_nearest_mm));
	memset(g_prev_valid,      0, sizeof(g_prev_valid));

	LOG_INF("Obstacle pipeline ready (roll %.0f°, tilt %.1f°, sensor %.2fm above floor)",
		(double)TOF_ROLL_DEG, (double)SENSOR_DOWNTILT_DEG,
		(double)SENSOR_HEIGHT_M);
}

void hapnav_obstacle_step(const int16_t distances_mm[HAPNAV_TOF_ZONES],
			  const uint8_t target_status[HAPNAV_TOF_ZONES],
			  const float   quat_wxyz[4],
			  const float   gyro_radps[3],
			  const float   accel_g[3],
			  int16_t       head_distance_mm,
			  int16_t       head_proximity_max_mm,
			  float         dt_s,
			  struct hapnav_obstacles *out)
{
	memset(out, 0, sizeof(*out));
	out->nearest_range_mm = -1;

	/* ── 0a. Stationary detection (rolling 1 s variance of |a|) ───────── */

	float a_mag = sqrtf(accel_g[0]*accel_g[0] +
			    accel_g[1]*accel_g[1] +
			    accel_g[2]*accel_g[2]);
	g_accel_mag_hist[g_accel_hist_idx] = a_mag;
	g_accel_hist_idx = (g_accel_hist_idx + 1) % STATIONARY_WINDOW_FRAMES;
	if (g_accel_hist_count < STATIONARY_WINDOW_FRAMES) {
		g_accel_hist_count++;
	}

	bool stationary = false;
	if (g_accel_hist_count == STATIONARY_WINDOW_FRAMES) {
		float mean = 0.0f;
		for (int i = 0; i < STATIONARY_WINDOW_FRAMES; i++) {
			mean += g_accel_mag_hist[i];
		}
		mean /= STATIONARY_WINDOW_FRAMES;
		float var = 0.0f;
		for (int i = 0; i < STATIONARY_WINDOW_FRAMES; i++) {
			float d = g_accel_mag_hist[i] - mean;
			var += d * d;
		}
		var /= STATIONARY_WINDOW_FRAMES;
		stationary = (var < STATIONARY_VAR_THRESH_G2);
	}

	bool yaw_slewing = (fabsf(gyro_radps[2]) > YAW_SLEW_RADPS);

	/* ── 1. Validity mask + cheap aggregate counts ────────────────────── */

	bool valid[HAPNAV_TOF_ZONES];
	int  n_valid = 0;
	int  n_near  = 0;

	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		uint8_t s = target_status[i];
		int16_t r = distances_mm[i];
		bool status_ok = (s == 5 || s == 6 || s == 9);

		valid[i] = (status_ok && r >= MIN_RANGE_MM && r <= MAX_RANGE_MM);
		if (valid[i]) {
			n_valid++;
		}
		if (status_ok && r >= 0 && r < MIN_RANGE_MM) {
			n_near++;
		}
	}

	bool sensor_blocked = (n_near  >= SENSOR_BLOCKED_NEAR_PIX);
	bool mostly_invalid = (n_valid <  MOSTLY_INVALID_VALID_PIX);

	/* ── 2-3. Body→world ray rotation; lift to point cloud where valid ── */

	float R_WB[3][3];
	quat_to_R_WB(quat_wxyz, R_WB);

	float ray_W[HAPNAV_TOF_ZONES][3];
	float pt_W_z[HAPNAV_TOF_ZONES];   /* only Z needed for classification */

	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		const float *rb = g_ray_B[i];
		ray_W[i][0] = R_WB[0][0]*rb[0] + R_WB[0][1]*rb[1] + R_WB[0][2]*rb[2];
		ray_W[i][1] = R_WB[1][0]*rb[0] + R_WB[1][1]*rb[1] + R_WB[1][2]*rb[2];
		ray_W[i][2] = R_WB[2][0]*rb[0] + R_WB[2][1]*rb[1] + R_WB[2][2]*rb[2];

		if (valid[i]) {
			pt_W_z[i] = ray_W[i][2] * (distances_mm[i] * 0.001f);
		} else {
			pt_W_z[i] = 0.0f;
		}
	}

	/* ── 4. Per-zone classification (floor / ceiling / candidate) ──────── */

	enum { CLS_INVALID = 0, CLS_FLOOR, CLS_CEIL, CLS_OBSTACLE };
	uint8_t cls[HAPNAV_TOF_ZONES];

	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		if (!valid[i]) {
			cls[i] = CLS_INVALID;
			continue;
		}
		float h = pt_W_z[i];
		if (h < FLOOR_BAND_HIGH_M)        cls[i] = CLS_FLOOR;
		else if (h > HEAD_TOP_BAND_M)     cls[i] = CLS_CEIL;
		else                              cls[i] = CLS_OBSTACLE;
	}

	/* ── 5. Per-azimuth-bin nearest cluster (with ≥ MIN_CLUSTER_PIXELS support) ── */

	int16_t bin_nearest_mm[4] = { -1, -1, -1, -1 };

	for (int b = 0; b < 4; b++) {
		int16_t min_r = -1;

		for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
			if (g_az_bin[i] != b) continue;
			if (cls[i] != CLS_OBSTACLE) continue;
			if (min_r < 0 || distances_mm[i] < min_r) {
				min_r = distances_mm[i];
			}
		}

		if (min_r < 0) continue;

		int support = 0;
		for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
			if (g_az_bin[i] != b) continue;
			if (cls[i] != CLS_OBSTACLE) continue;
			if (abs(distances_mm[i] - min_r) <= CLUSTER_DELTA_MM) {
				support++;
			}
		}
		if (support >= MIN_CLUSTER_PIXELS) {
			bin_nearest_mm[b] = min_r;
		}
	}

	/* ── 6. Closing rate per bin (frame-to-frame) ─────────────────────── */

	float closing_mps[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	if (!yaw_slewing && dt_s > 0.001f) {
		for (int b = 0; b < 4; b++) {
			if (bin_nearest_mm[b] >= 0 && g_prev_valid[b]) {
				float dr_m = (g_prev_nearest_mm[b] - bin_nearest_mm[b]) * 0.001f;
				closing_mps[b] = dr_m / dt_s;
			}
		}
	}

	for (int b = 0; b < 4; b++) {
		if (bin_nearest_mm[b] >= 0) {
			g_prev_nearest_mm[b] = bin_nearest_mm[b];
			g_prev_valid[b]      = true;
		} else {
			g_prev_valid[b] = false;
		}
	}

	/* ── 7. Urgency formation ─────────────────────────────────────────── */

	const float r_min = MIN_RANGE_MM * 0.001f;
	const float r_max = MAX_RANGE_MM * 0.001f;

	for (int b = 0; b < 4; b++) {
		if (bin_nearest_mm[b] < 0) {
			out->urgency[b] = 0;
			continue;
		}
		float r_m = bin_nearest_mm[b] * 0.001f;

		float prox = clamp01((r_max - r_m) / (r_max - r_min));

		float ttc_score = 0.0f;
		if (closing_mps[b] > 0.05f) {
			float ttc = r_m / closing_mps[b];
			ttc_score = clamp01((TTC_NO_URGENCY_S - ttc) /
					    (TTC_NO_URGENCY_S - TTC_FULL_URGENCY_S));
		}

		float urg = (prox > ttc_score) ? prox : ttc_score;
		if (stationary)  urg *= 0.5f;
		if (yaw_slewing) urg *= 0.5f;

		out->urgency[b] = to_u8(urg);
	}

	if (sensor_blocked || mostly_invalid) {
		out->urgency[0] = out->urgency[1] = out->urgency[2] = out->urgency[3] = 0;
	}

	/* In-path nearest = min over the two centre bins. */
	int16_t nearest = -1;
	for (int b = 1; b <= 2; b++) {
		if (bin_nearest_mm[b] >= 0 &&
		    (nearest < 0 || bin_nearest_mm[b] < nearest)) {
			nearest = bin_nearest_mm[b];
		}
	}
	out->nearest_range_mm = nearest;

	/* ── 8. Drop-off check ────────────────────────────────────────────── */

	bool dropoff = hapnav_dropoff_check(distances_mm, target_status,
					    ray_W, SENSOR_HEIGHT_M);

	/* ── 9. Head clearance from VL53L1X ───────────────────────────────── */

	bool head_obstacle = false;
	if (head_distance_mm > 0 &&
	    head_proximity_max_mm > 0 &&
	    head_distance_mm <= head_proximity_max_mm &&
	    !yaw_slewing) {
		head_obstacle = true;
	}

	/* ── Flags ───────────────────────────────────────────────────────── */

	out->flags = 0;
	if (stationary)     out->flags |= HAPNAV_OBS_FLAG_STATIONARY;
	if (sensor_blocked) out->flags |= HAPNAV_OBS_FLAG_SENSOR_BLOCKED;
	if (mostly_invalid) out->flags |= HAPNAV_OBS_FLAG_MOSTLY_INVALID;
	if (yaw_slewing)    out->flags |= HAPNAV_OBS_FLAG_YAW_SLEWING;
	if (dropoff)        out->flags |= HAPNAV_OBS_FLAG_DROPOFF;
	if (head_obstacle)  out->flags |= HAPNAV_OBS_FLAG_HEAD_OBSTACLE;
}
