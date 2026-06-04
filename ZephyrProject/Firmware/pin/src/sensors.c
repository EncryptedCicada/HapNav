#include "sensors.h"

#include <hapnav/bno055.h>
#include <hapnav/vl53l5cx.h>
#include <hapnav/obstacle.h>
#include <hapnav/dropoff.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(pin_sensors, LOG_LEVEL_INF);

/*
 * Pin sensor bring-up (v3 — everything on the mux):
 *   TCA9548A mux on i2c0 with four populated branches:
 *     ch0 (alias hapnav-tof-head)      → VL53L1X head (Zephyr sensor)
 *     ch1 (alias hapnav-tof-front-bus) → VL53L5CX via vendor ULD
 *     ch2 (alias hapnav-tof-down)      → VL53L1X down (Zephyr sensor)
 *     ch3 (alias hapnav-imu-bus)       → BNO055 9-DOF IMU @ 0x28 (NDOF fusion)
 *
 * Each ToF lives at its default 0x29 on its own muxed bus — no XSHUT, no
 * runtime address shift, no warm-reset retained-address handling. The BNO055
 * sits on its own muxed bus too, isolated from the ToFs except when ch3 is
 * routed. The Zephyr i2c-mux driver picks the channel transparently per
 * transfer.
 */

#define SAMPLE_PERIOD_MS  12            /* ~83 Hz (4× the previous 20 Hz cadence) */
#define SAMPLE_DT_S       (SAMPLE_PERIOD_MS / 1000.0f)

/* How often each of head + down VL53L1X actually gets sampled, in loop
 * counts. Head and down are 180° out of phase, so at the steady-state
 * effective ~50 Hz loop rate this puts each VL53L1X at ~6 Hz — well above
 * what slow-changing head-clearance / dropoff needs. Lower this for
 * snappier VL53L1X latency at the cost of lower main-loop rate. */
#define VL53L1X_SAMPLE_PERIOD_LOOPS  8

/* Head-clearance trigger envelope. Worn-pose default catches doorframes at
 * 1.0–2.5 m forward; bench mode shrinks it to a 15 cm slant. */
#if defined(CONFIG_HAPNAV_BENCH_MODE)
#define HEAD_PROXIMITY_MAX_MM       150
#else
#define HEAD_PROXIMITY_MAX_MM       2200
#endif
#define HEAD_PROXIMITY_MIN_MM        40   /* below this the reading is suspect */

/* DT handles */
#define TOF_FRONT_BUS_NODE   DT_ALIAS(hapnav_tof_front_bus)
#define TOF_HEAD_NODE        DT_ALIAS(hapnav_tof_head)
#define TOF_DOWN_NODE        DT_ALIAS(hapnav_tof_down)
#define IMU_BUS_NODE         DT_ALIAS(hapnav_imu_bus)

#define BNO055_I2C_ADDR      0x28   /* Adafruit BFF default (COM3=GND) */

static const struct device *const tof_front_bus = DEVICE_DT_GET(TOF_FRONT_BUS_NODE);
static const struct device *const tof_head      = DEVICE_DT_GET(TOF_HEAD_NODE);
static const struct device *const tof_down      = DEVICE_DT_GET(TOF_DOWN_NODE);
static const struct device *const imu_bus       = DEVICE_DT_GET(IMU_BUS_NODE);

static struct hapnav_bno055       bno055;
static struct hapnav_vl53l5cx     tof;
static struct hapnav_dropoff_state dropoff_state;

static bool    bno055_ok;
static bool    tof_ok;
static bool    tof_have_frame;     /* true after first vl53l5cx_poll == 1 */
static bool    tof_head_ok;
static bool    tof_down_ok;

static int16_t head_distance_mm = -1;
static int16_t down_distance_mm = -1;

/* ── I2C bus-stall recovery (parent of the mux) ─────────────────────────── */

#define I2C_RECOVER_AFTER_FAILS  2
static const struct device   *i2c_bus;
static int                     i2c_fail_streak;

/* ── Per-pixel ToF smoothing state ──────────────────────────────────────── */

#define TOF_EMA_NUM 3
#define TOF_EMA_DEN 10
static int16_t smoothed_dist_mm[HAPNAV_TOF_ZONES];
static bool    tof_smooth_init;

static void smooth_tof_frame(const int16_t raw_dist[HAPNAV_TOF_ZONES],
			     const uint8_t raw_status[HAPNAV_TOF_ZONES],
			     int16_t       out_dist[HAPNAV_TOF_ZONES],
			     uint8_t       out_status[HAPNAV_TOF_ZONES])
{
	if (!tof_smooth_init) {
		for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
			smoothed_dist_mm[i] = raw_dist[i];
		}
		tof_smooth_init = true;
	}
	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		bool valid = (raw_status[i] == 5 || raw_status[i] == 6 ||
			      raw_status[i] == 9);
		if (valid) {
			int32_t s = ((int32_t)smoothed_dist_mm[i] *
				     (TOF_EMA_DEN - TOF_EMA_NUM) +
				     (int32_t)raw_dist[i] * TOF_EMA_NUM) /
				    TOF_EMA_DEN;
			smoothed_dist_mm[i] = (int16_t)s;
		}
		out_dist[i]   = smoothed_dist_mm[i];
		out_status[i] = raw_status[i];
	}
}

/* ── I2C bus diagnostics ────────────────────────────────────────────────── */

static void i2c_scan_log(const struct device *i2c)
{
	if (!device_is_ready(i2c)) {
		LOG_ERR("I²C bus %s not ready — can't scan", i2c->name);
		return;
	}
	LOG_INF("I²C scan on %s (mux parent — child devices hide behind 0x70):", i2c->name);
	int found = 0;
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t b;
		if (i2c_read(i2c, &b, 1, addr) != 0) {
			continue;
		}
		const char *hint = "";
		switch (addr) {
		case 0x70: hint = " (TCA9548A mux)"; break;
		default: break;
		}
		LOG_INF("  found 0x%02x%s", addr, hint);
		found++;
	}
	if (found == 0) {
		LOG_ERR("  no devices responded — check 3V3 / GND / SDA / SCL");
	}
}

static void i2c_health_update(int err)
{
	if (err == 0) {
		if (i2c_fail_streak) {
			LOG_INF("I²C: reads healthy again after %d failure(s)",
				i2c_fail_streak);
		}
		i2c_fail_streak = 0;
		return;
	}
	if (++i2c_fail_streak < I2C_RECOVER_AFTER_FAILS) {
		return;
	}
	LOG_WRN("I²C: %d consecutive failures (last %d) — recovering parent bus",
		i2c_fail_streak, err);
	int rc = i2c_recover_bus(i2c_bus);
	if (rc == -ENOSYS) {
		LOG_ERR("I²C: controller has no recover_bus support");
	} else if (rc) {
		LOG_ERR("I²C: recover_bus failed: %d", rc);
	} else {
		LOG_INF("I²C: bus recovered — stuck slave released");
	}
	i2c_fail_streak = 0;
}

/* ── Init ───────────────────────────────────────────────────────────────── */

int pin_sensors_init(void)
{
	hapnav_obstacle_init();
	hapnav_dropoff_init(&dropoff_state);

	i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	i2c_scan_log(i2c_bus);

	if (!device_is_ready(tof_front_bus)) {
		LOG_ERR("TCA9548A channel 0 (front) not ready");
		tof_ok = false;
	} else {
		int rc = hapnav_vl53l5cx_init(&tof, tof_front_bus,
					      HAPNAV_VL53L5CX_DEFAULT_ADDR_8BIT);
		if (rc) {
			LOG_WRN("VL53L5CX (front) init failed: %d", rc);
			tof_ok = false;
		} else {
			tof_ok = true;
		}
	}

	/* Head + down VL53L1X nodes are marked zephyr,deferred-init so the
	 * in-tree driver doesn't try to talk to a maybe-absent chip during
	 * POST_KERNEL. We call device_init() here only when the corresponding
	 * Kconfig says we actually intend to use that sensor. */
#if defined(CONFIG_HAPNAV_HEAD_TOF)
	if (device_init(tof_head) == 0 && device_is_ready(tof_head)) {
		tof_head_ok = true;
		LOG_INF("VL53L1X (head) ready on mux ch0");
	} else {
		LOG_WRN("VL53L1X (head) bring-up failed");
		tof_head_ok = false;
	}
#else
	LOG_INF("VL53L1X (head) disabled via CONFIG_HAPNAV_HEAD_TOF=n");
	tof_head_ok = false;
#endif

#if defined(CONFIG_HAPNAV_DOWN_TOF)
	if (device_init(tof_down) == 0 && device_is_ready(tof_down)) {
		tof_down_ok = true;
		LOG_INF("VL53L1X (down) ready on mux ch2 (downtilt %.1f°)",
			(double)HAPNAV_DOWN_TOF_DOWNTILT_DEG);
	} else {
		LOG_WRN("VL53L1X (down) bring-up failed");
		tof_down_ok = false;
	}
#else
	LOG_INF("VL53L1X (down) disabled via CONFIG_HAPNAV_DOWN_TOF=n");
	tof_down_ok = false;
#endif

	int rc = hapnav_bno055_init(&bno055, imu_bus, BNO055_I2C_ADDR);
	if (rc) {
		LOG_ERR("BNO055 init failed: %d (proceeding without IMU)", rc);
		bno055_ok = false;
	} else {
		bno055_ok = true;

		/* Mounting-specific axis remap. The chip is glued into the
		 * pin such that, when the pin is worn upright facing forward,
		 *   chip −Z  → user forward  (body −Y)
		 *   chip +Y  → world up      (body +Z)
		 *   chip +X  → wearer's left (so body +X right = chip −X)
		 *
		 * That makes the chip→body mapping
		 *   body X = −chip X
		 *   body Y =  chip Z
		 *   body Z =  chip Y
		 *
		 * which encodes as REMAP_CONFIG = 0x18, REMAP_SIGN = 0x01
		 * (datasheet §3.4, a right-handed permutation, det = +1).
		 * After this the BNO055 reports accel / gyro / quaternion
		 * directly in our pipeline's body frame — no extra rotation
		 * downstream. */
		rc = hapnav_bno055_set_axis_remap(&bno055, 0x18, 0x01);
		if (rc) {
			LOG_WRN("BNO055 axis remap failed: %d (using chip default)",
				rc);
		} else {
			LOG_INF("BNO055 axis remap applied (CONFIG=0x18, SIGN=0x01)");
		}
	}

	/* ── Hardware-check summary ─────────────────────────────────────────
	 * One last block so the boot log makes it instantly obvious which
	 * sensors landed and which didn't, without scrolling through the
	 * interleaved per-device init logs. The mux gets its own probe (one
	 * 1-byte read from 0x70) because nothing else can work without it;
	 * if it's FAIL, all the ToF rows below it will be FAIL too and the
	 * "0/3 ToFs ready" result is the keystone-failure diagnostic. */
	uint8_t mux_probe;
	const bool mux_ok = (i2c_read(i2c_bus, &mux_probe, 1, 0x70) == 0);
	const int tofs_ready = (tof_ok ? 1 : 0) +
			       (tof_head_ok ? 1 : 0) +
			       (tof_down_ok ? 1 : 0);

	LOG_INF("-- Sensor hardware check -----------------------------------");
	if (mux_ok) {
		LOG_INF("  Mux  TCA9548A  @ 0x70  : OK");
	} else {
		LOG_WRN("  Mux  TCA9548A  @ 0x70  : FAIL");
	}
	if (tof_ok) {
		LOG_INF("  ch1  VL53L5CX  (front) : OK");
	} else {
		LOG_WRN("  ch1  VL53L5CX  (front) : FAIL");
	}
	if (tof_head_ok) {
		LOG_INF("  ch0  VL53L1X   (head)  : OK");
	} else {
		LOG_WRN("  ch0  VL53L1X   (head)  : FAIL");
	}
	if (tof_down_ok) {
		LOG_INF("  ch2  VL53L1X   (down)  : OK");
	} else {
		LOG_WRN("  ch2  VL53L1X   (down)  : FAIL");
	}
	if (tofs_ready == 3) {
		LOG_INF("  Result: %d/3 ToFs ready", tofs_ready);
	} else {
		LOG_WRN("  Result: %d/3 ToFs ready", tofs_ready);
	}
	LOG_INF("------------------------------------------------------------");

	return 0;
}

/* ── Sample loop ────────────────────────────────────────────────────────── */

static void update_head_distance(void)
{
#if !defined(CONFIG_HAPNAV_HEAD_TOF)
	head_distance_mm = -1;
	return;
#else
	if (!tof_head_ok) {
		head_distance_mm = -1;
		return;
	}
	if (sensor_sample_fetch(tof_head)) {
		head_distance_mm = -1;
		return;
	}
	struct sensor_value val;
	if (sensor_channel_get(tof_head, SENSOR_CHAN_DISTANCE, &val)) {
		head_distance_mm = -1;
		return;
	}
	int32_t mm = val.val1;
	if (mm < HEAD_PROXIMITY_MIN_MM || mm > 4000) {
		head_distance_mm = -1;
		return;
	}
	head_distance_mm = (mm > INT16_MAX) ? INT16_MAX : (int16_t)mm;
#endif
}

static void update_down_distance(void)
{
#if !defined(CONFIG_HAPNAV_DOWN_TOF)
	down_distance_mm = -1;
	return;
#else
	if (!tof_down_ok) {
		down_distance_mm = -1;
		return;
	}
	if (sensor_sample_fetch(tof_down)) {
		down_distance_mm = -1;
		return;
	}
	struct sensor_value val;
	if (sensor_channel_get(tof_down, SENSOR_CHAN_DISTANCE, &val)) {
		down_distance_mm = -1;
		return;
	}
	int32_t mm = val.val1;
	if (mm < 0 || mm > 4000) {
		down_distance_mm = -1;
		return;
	}
	down_distance_mm = (mm > INT16_MAX) ? INT16_MAX : (int16_t)mm;
#endif
}

/* Extract body-frame pitch from a body→world quaternion [w,x,y,z]. Used by
 * the dropoff module's pitch-compensation slot (reserved). */
static inline float quat_pitch(const float q[4])
{
	float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
	if (sinp >  1.0f) sinp =  1.0f;
	if (sinp < -1.0f) sinp = -1.0f;
	return asinf(sinp);
}

int pin_sensors_sample(struct hapnav_frame *out)
{
	memset(out, 0, sizeof(*out));
	out->timestamp_ms = k_uptime_get_32();

	/* ── BNO055 motion (quat + accel + gyro in one UART burst) ──────── */
	float accel_g[3]    = {0};
	float gyro_radps[3] = {0};
	if (bno055_ok) {
		int err = hapnav_bno055_read_motion(&bno055, out->quat,
						    accel_g, gyro_radps);
		if (err) {
			LOG_WRN("BNO055 read failed: %d", err);
			/* identity quat so the rest of the pipeline doesn't NaN */
			out->quat[0] = 1.0f;
		}
	} else {
		out->quat[0] = 1.0f;
	}

	/* ── Front ToF (VL53L5CX, vendor ULD on mux ch1) ────────────────── */
	if (tof_ok) {
		int rc = hapnav_vl53l5cx_poll(&tof);
		if (rc < 0) {
			LOG_WRN("ToF poll error: %d", rc);
			i2c_health_update(rc);
		} else {
			if (rc == 1) {
				tof_have_frame = true;
			}
			i2c_health_update(0);
		}
		if (tof_have_frame) {
			int16_t raw_dist[HAPNAV_TOF_ZONES];
			uint8_t raw_status[HAPNAV_TOF_ZONES];
			for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
				raw_dist[i]   = tof.results.distance_mm[i];
				raw_status[i] = tof.results.target_status[i];
			}
			smooth_tof_frame(raw_dist, raw_status,
					 out->distances_mm, out->target_status);
		}
	}

	/* ── Head + down ToFs ──────────────────────────────────────────── */
	/* ── Head + down VL53L1X — skip-framed ─────────────────────────────
	 * The Zephyr in-tree VL53L1X driver's `sensor_channel_get` blocks on
	 * `VL53L1_WaitMeasurementDataReady`, which sits for the chip's full
	 * timing-budget window (~33 ms in LONG mode). Doing both reads every
	 * loop pins the sample rate to ~12 Hz floor. Sampling each one every
	 * VL53L1X_SAMPLE_PERIOD_LOOPS iterations, staggered, lets the main
	 * loop ride near its k_msleep target while the heads still update
	 * fast enough for slow-changing head-clearance and dropoff signals
	 * (each ~7-10 Hz at a 50-60 Hz loop). The intervening loops just
	 * reuse the last-cached `head_distance_mm` / `down_distance_mm`. */
	static uint32_t vl53l1x_phase;
	if ((vl53l1x_phase % VL53L1X_SAMPLE_PERIOD_LOOPS) == 0) {
		update_head_distance();
	}
	if ((vl53l1x_phase % VL53L1X_SAMPLE_PERIOD_LOOPS) ==
	    VL53L1X_SAMPLE_PERIOD_LOOPS / 2) {
		update_down_distance();
	}
	vl53l1x_phase++;

	/* ── Obstacle pipeline ─────────────────────────────────────────── */
	hapnav_obstacle_step(out->distances_mm, out->target_status,
			     out->quat, gyro_radps, accel_g,
			     head_distance_mm, HEAD_PROXIMITY_MAX_MM,
			     SAMPLE_DT_S, &out->obstacles);

	/* ── Down-ToF dropoff → OR the flag in ─────────────────────────── */
#if defined(CONFIG_HAPNAV_DROPOFF)
	float pitch = quat_pitch(out->quat);
	bool dropoff = hapnav_dropoff_check_down(&dropoff_state,
						 down_distance_mm,
						 0.0f /* sensor_height_m reserved */,
						 pitch);
	if (dropoff) {
		out->obstacles.flags |= HAPNAV_OBS_FLAG_DROPOFF;
	}
#endif

	/* ── Trace ──────────────────────────────────────────────────────── */
	{
		static uint32_t trace_div;
		bool dump_grid = (trace_div++ % 1) == 0;   /* Per Frame */
		const struct hapnav_obstacles *o = &out->obstacles;

		printk("PIN {\"ts\":%u,"
		       "\"a\":[%.3f,%.3f,%.3f],\"g\":[%.3f,%.3f,%.3f],"
		       "\"q\":[%.3f,%.3f,%.3f,%.3f],"
		       "\"head\":%d,\"down\":%d,"
		       "\"obs\":{\"urg\":[%u,%u,%u,%u],\"near\":%d,\"flags\":\"0x%02x\"}",
		       out->timestamp_ms,
		       (double)accel_g[0], (double)accel_g[1], (double)accel_g[2],
		       (double)gyro_radps[0], (double)gyro_radps[1], (double)gyro_radps[2],
		       (double)out->quat[0], (double)out->quat[1],
		       (double)out->quat[2], (double)out->quat[3],
		       (int)head_distance_mm, (int)down_distance_mm,
		       o->urgency[0], o->urgency[1], o->urgency[2], o->urgency[3],
		       (int)o->nearest_range_mm, (unsigned)o->flags);

		if (dump_grid) {
			printk(",\"d\":[");
			for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
				printk("%s%d", i ? "," : "", (int)out->distances_mm[i]);
			}
			printk("],\"st\":[");
			for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
				printk("%s%u", i ? "," : "", (unsigned)out->target_status[i]);
			}
			printk("]");
		}
		printk("}\n");
	}

	return 0;
}
