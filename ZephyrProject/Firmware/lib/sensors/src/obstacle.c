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

#define SENSOR_DOWNTILT_DEG      8.0f

#if defined(CONFIG_HAPNAV_BENCH_MODE)
/* Bench-demo geometry: the pin stands UPRIGHT on a desk, so the VL53L5CX
 * optical centre sits ~1–2 cm above the desk surface and grazes along it
 * rather than looking down at it. Objects move within a 10–50 cm envelope.
 * The stationary detector is defanged so a pin resting on the bench doesn't
 * mute the pipeline.
 *
 * Because the sensor grazes the desk, every desk pixel lands at a roughly
 * constant world-frame height of −(sensor height) ≈ −1.5 cm. FLOOR_BAND_HIGH
 * must therefore sit just below the sensor (−0.5 cm) so the desk plane is
 * classified as FLOOR and excluded — otherwise the whole desk reads as a
 * wall of obstacles. The cost is a ~0.5–1.5 cm "blind band" right at desk
 * level: objects must rise to within ~0.5 cm of the sensor height to be
 * seen, which is fine for the cups/hands/boxes used in the demo. */
#define SENSOR_HEIGHT_M          0.015f
#define USER_HEAD_ABOVE_SENSOR_M 0.05f
#define HEAD_CLEARANCE_M         0.05f
#define HEAD_TOP_BAND_M          (USER_HEAD_ABOVE_SENSOR_M + HEAD_CLEARANCE_M)
#define FLOOR_BAND_HIGH_M        (-0.005f)
#define MIN_RANGE_MM             50
#define MAX_RANGE_MM             500
#define CLUSTER_DELTA_MM         30
/* Variance is always ≥ 0, so a threshold of −1 makes `var < THRESH`
 * unsatisfiable → stationary never fires. (Earlier I set this to 1e9
 * thinking "huge threshold = disabled"; that actually inverted the
 * logic and made stationary always-on, which muted the wristband for
 * every frame on the bench.) */
#define STATIONARY_VAR_THRESH_G2 (-1.0f)
#else
/* Worn / real-world geometry. */
#define SENSOR_HEIGHT_M          1.35f
#define USER_HEAD_ABOVE_SENSOR_M 0.40f         /* 1.75 m head − 1.35 m sensor */
#define HEAD_CLEARANCE_M         0.30f
#define HEAD_TOP_BAND_M          (USER_HEAD_ABOVE_SENSOR_M + HEAD_CLEARANCE_M)
#define FLOOR_BAND_HIGH_M        (-1.20f)      /* below sensor; tolerates 15 cm of floor irregularity */
#define MIN_RANGE_MM             300
#define MAX_RANGE_MM             3000
#define CLUSTER_DELTA_MM         200
#define STATIONARY_VAR_THRESH_G2 0.0025f       /* (0.05 g)² */
#endif

#define DEG2RAD                  0.017453293f

/* ── Detection thresholds ────────────────────────────────────────────────── */

#define MIN_CLUSTER_PIXELS         2
#define STATIONARY_WINDOW_FRAMES   10           /* 1 s @ 10 Hz */
#define YAW_SLEW_RADPS             (60.0f * DEG2RAD)

#define TTC_FULL_URGENCY_S         1.5f
#define TTC_NO_URGENCY_S           5.0f

#define SENSOR_BLOCKED_NEAR_PIX    32          /* half the array reads sub-min */
/* Hysteresis band on the valid-pixel count so per-pixel status noise (heavy
 * at the grazing desk angle) doesn't flip the whole-frame mute every frame.
 * Latch MOSTLY_INVALID below ENTER, release only once comfortably above EXIT. */
#define MOSTLY_INVALID_ENTER_PIX   6
#define MOSTLY_INVALID_EXIT_PIX    10

/* ── Stationary-obstacle habituation ──────────────────────────────────────
 * A bin whose nearest range holds within HABIT_DEADBAND_MM of where it was
 * first seen is "stationary w.r.t. the pin": its urgency decays by
 * HABIT_DECAY each frame toward HABIT_FLOOR, fading from a firm cue to (near)
 * silence over ~2-3 s so a parked obstacle stops nagging. Any range change
 * beyond the deadband — the object moved, or the pin moved — snaps it back to
 * full. HABIT_DEADBAND_MM is the sensitivity knob: widen it to tolerate more
 * movement before re-alerting; raise HABIT_FLOOR for a faint "still there"
 * reminder instead of full silence. */
#if defined(CONFIG_HAPNAV_HABITUATION)
#if defined(CONFIG_HAPNAV_BENCH_MODE)
#define HABIT_DEADBAND_MM   40
#else
#define HABIT_DEADBAND_MM   150
#endif
#define HABIT_DECAY         0.88f
#define HABIT_FLOOR         0.0f
#endif

/* ── State ────────────────────────────────────────────────────────────────── */

static float   g_ray_B[HAPNAV_TOF_ZONES][3];   /* unit rays in body frame, tilt baked in */
static uint8_t g_col_bin[TOF_COLS] = { 0, 0, 1, 1, 2, 2, 3, 3 };

static float g_accel_mag_hist[STATIONARY_WINDOW_FRAMES];
static int   g_accel_hist_idx;
static int   g_accel_hist_count;

static int16_t g_prev_nearest_mm[4];
static bool    g_prev_valid[4];
static bool    g_mostly_invalid;   /* latched, hysteretic (see thresholds) */

#if defined(CONFIG_HAPNAV_HABITUATION)
static float   g_habit[4];         /* per-bin urgency scale, decays when still */
static int16_t g_habit_ref_mm[4];  /* range the current habituation is anchored to */
#endif

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

#if defined(CONFIG_HAPNAV_HABITUATION)
/* Urgency scale in [HABIT_FLOOR, 1] that fades while a bin's nearest
 * obstacle holds still and snaps back to 1 the moment it moves. */
static float update_habituation(int b, int16_t nearest_mm)
{
	if (g_habit_ref_mm[b] < 0 ||
	    abs(nearest_mm - g_habit_ref_mm[b]) > HABIT_DEADBAND_MM) {
		g_habit_ref_mm[b] = nearest_mm;   /* (re)arm from here */
		g_habit[b]        = 1.0f;
		return 1.0f;
	}
	g_habit[b] *= HABIT_DECAY;
	if (g_habit[b] < HABIT_FLOOR) {
		g_habit[b] = HABIT_FLOOR;
	}
	return g_habit[b];
}
#endif

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
 * Build the per-zone unit ray in body frame.
 *
 * Sensor frame (S):  +X right, +Y up, +Z forward (optical axis).
 * Body frame (B):    +X right, +Y back, +Z up — so -Y_B is "user forward".
 * No-tilt mapping S → B:  X_B = X_S, Y_B = -Z_S, Z_B = Y_S.
 * Then apply R_x(+θ) for downtilt: rotates the optical axis (-Y_B before
 * tilt) toward -Z_B, i.e. nose-down by θ.
 */
static void build_ray_table(void)
{
	const float ct = cosf(SENSOR_DOWNTILT_DEG * DEG2RAD);
	const float st = sinf(SENSOR_DOWNTILT_DEG * DEG2RAD);

	for (int r = 0; r < TOF_ROWS; r++) {
		for (int c = 0; c < TOF_COLS; c++) {
			float az = (c - 3.5f) * TOF_DEG_PER_ZONE * DEG2RAD;
			float el = (3.5f - r) * TOF_DEG_PER_ZONE * DEG2RAD;

			float xs = sinf(az) * cosf(el);
			float ys = sinf(el);
			float zs = cosf(az) * cosf(el);

			/* S → B (no tilt) */
			float xb0 =  xs;
			float yb0 = -zs;
			float zb0 =  ys;

			/* Apply downtilt R_x(+θ): Y' = Y·c − Z·s, Z' = Y·s + Z·c */
			float xb = xb0;
			float yb = yb0 * ct - zb0 * st;
			float zb = yb0 * st + zb0 * ct;

			int idx = r * TOF_COLS + c;
			g_ray_B[idx][0] = xb;
			g_ray_B[idx][1] = yb;
			g_ray_B[idx][2] = zb;
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
	g_mostly_invalid = false;

#if defined(CONFIG_HAPNAV_HABITUATION)
	for (int b = 0; b < 4; b++) {
		g_habit[b]        = 1.0f;
		g_habit_ref_mm[b] = -1;
	}
#endif

	LOG_INF("Obstacle pipeline ready%s (tilt %.1f°, sensor %.2fm above floor, "
		"range %d..%dmm)",
#if defined(CONFIG_HAPNAV_BENCH_MODE)
		" [BENCH MODE]",
#else
		"",
#endif
		(double)SENSOR_DOWNTILT_DEG, (double)SENSOR_HEIGHT_M,
		MIN_RANGE_MM, MAX_RANGE_MM);
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

	if (n_valid < MOSTLY_INVALID_ENTER_PIX) {
		g_mostly_invalid = true;
	} else if (n_valid >= MOSTLY_INVALID_EXIT_PIX) {
		g_mostly_invalid = false;
	}
	bool mostly_invalid = g_mostly_invalid;

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

		for (int r = 0; r < TOF_ROWS; r++) {
			for (int c = 0; c < TOF_COLS; c++) {
				if (g_col_bin[c] != b) continue;
				int idx = r * TOF_COLS + c;
				if (cls[idx] != CLS_OBSTACLE) continue;
				if (min_r < 0 || distances_mm[idx] < min_r) {
					min_r = distances_mm[idx];
				}
			}
		}

		if (min_r < 0) continue;

		int support = 0;
		for (int r = 0; r < TOF_ROWS; r++) {
			for (int c = 0; c < TOF_COLS; c++) {
				if (g_col_bin[c] != b) continue;
				int idx = r * TOF_COLS + c;
				if (cls[idx] != CLS_OBSTACLE) continue;
				if (abs(distances_mm[idx] - min_r) <= CLUSTER_DELTA_MM) {
					support++;
				}
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
#if defined(CONFIG_HAPNAV_HABITUATION)
			/* Bin cleared — re-arm so the next obstacle alerts fully. */
			g_habit[b]        = 1.0f;
			g_habit_ref_mm[b] = -1;
#endif
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
#if defined(CONFIG_HAPNAV_HABITUATION)
		urg *= update_habituation(b, bin_nearest_mm[b]);
#endif
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

#if defined(CONFIG_HAPNAV_DROPOFF)
	bool dropoff = hapnav_dropoff_check(distances_mm, target_status,
					    ray_W, SENSOR_HEIGHT_M);
#else
	bool dropoff = false;
#endif

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
