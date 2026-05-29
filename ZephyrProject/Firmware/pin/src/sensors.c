#include "sensors.h"

#include <hapnav/lsm6dso.h>
#include <hapnav/lis2mdl.h>
#include <hapnav/vl53l5cx.h>
#include <hapnav/madgwick.h>
#include <hapnav/obstacle.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(pin_sensors, LOG_LEVEL_INF);

#define LSM6DSO_I2C_ADDR        0x6B
#define LIS2MDL_I2C_ADDR        0x1E

/* The VL53L5CX defaults to 7-bit 0x29 (ULD uses the 8-bit form 0x52). We
 * move it off 0x29 at boot so the head-facing VL53L1X — which is fixed at
 * 7-bit 0x29 — can take its place once XSHUT is released. */
#define VL53L5CX_DEFAULT_ADDR_8BIT  0x52
#define VL53L5CX_SHIFTED_ADDR_8BIT  0x54  /* 7-bit 0x2A */

/* When the VL53L1X sees something within this range we treat it as a
 * head-clearance hit and set HAPNAV_OBS_FLAG_HEAD_OBSTACLE. The default
 * is calibrated for a 1.35 m sensor mount with the VL53L1X tilted +20°
 * above horizontal: obstacles 1.0–2.5 m forward at 1.7–2.1 m height
 * fall inside this slant range. Bench mode shrinks the envelope to a
 * 15 cm slant so a card waved over the desk-mounted pin triggers it. */
#if defined(CONFIG_HAPNAV_BENCH_MODE)
#define HEAD_PROXIMITY_MAX_MM       150
#else
#define HEAD_PROXIMITY_MAX_MM       2200
#endif
/* Sensor minimum reliable range; readings below this are status-flagged. */
#define HEAD_PROXIMITY_MIN_MM       40

#define SAMPLE_PERIOD_MS  100        /* must match the main-loop cadence */
#define SAMPLE_DT_S       (SAMPLE_PERIOD_MS / 1000.0f)
#define MADGWICK_BETA     0.1f

/* mg → m/s²; mdps → rad/s; mGauss is unitless to Madgwick (only ratios matter). */
#define MG_TO_MPS2          (HAPNAV_ACCEL_MG_PER_LSB * 0.00980665f)
#define MDPS_TO_RADPS       (HAPNAV_GYRO_MDPS_PER_LSB * (3.14159265f / 180000.0f))
#define ACCEL_G_PER_LSB     (HAPNAV_ACCEL_MG_PER_LSB * 0.001f)

#define TOF_HEAD_NODE     DT_ALIAS(hapnav_tof_head)
static const struct device   *const tof_head = DEVICE_DT_GET(TOF_HEAD_NODE);
static const struct gpio_dt_spec head_xshut  =
	GPIO_DT_SPEC_GET(TOF_HEAD_NODE, xshut_gpios);

static struct hapnav_lsm6dso  imu;
static struct hapnav_lis2mdl  mag;
static struct hapnav_vl53l5cx tof;
static struct madgwick        ahrs;
static bool                   tof_ok;
static bool                   tof_have_frame;     /* true after first poll==1 */
static bool                   tof_head_ok;
static int16_t                head_distance_mm = -1;

/* I2C bus-stall recovery state. The ESP32 controller times out a wedged
 * transfer after ~500 ms (it does NOT block forever), so a slave that holds
 * SDA mid-byte shows up as a run of failed reads — and a frozen-looking
 * heartbeat, since each sample then burns a 500 ms timeout and prints
 * nothing. After a short streak we issue the controller's recover_bus op
 * (9 SCL pulses + STOP) to release the stuck slave. */
#define I2C_RECOVER_AFTER_FAILS  2
static const struct device   *i2c_bus;   /* cached handle for recovery */
static int                     i2c_fail_streak;

/* Per-pixel ToF smoothing state.
 *
 * Exponential moving average on each pixel's distance with EMA_NUM/EMA_DEN
 * = 3/10 as the new-sample weight. On the rare valid update the smoothed
 * value moves 30 % of the way toward the raw reading, so an outlier needs
 * to persist for a few frames to dominate. Invalid status carries through
 * unchanged so the obstacle pipeline's validity mask still rejects those
 * pixels, but the smoothed distance is preserved for the next valid frame
 * (sticky behaviour — avoids the EMA dropping to zero just because the
 * sensor blinks invalid on one cycle). */
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
		out_status[i] = raw_status[i];   /* pass through unchanged */
	}
}

/* Enumerate the I²C bus so failures in individual sensor init can be
 * disambiguated between "chip missing / dead" and "bus wiring broken".
 * Each address is probed with a single-byte read; an ACK from the slave
 * is the existence proof. Runs after XSHUT pulls the head VL53L1X low,
 * so the VL53L1X (at default 0x29) shouldn't appear yet — but the
 * VL53L5CX (also default 0x29) should.
 *
 * Common HapNav addresses are hinted in the output so the user can
 * spot a missing chip at a glance. */
static void i2c_scan_log(const struct device *i2c)
{
	if (!device_is_ready(i2c)) {
		LOG_ERR("I²C bus %s not ready — can't scan", i2c->name);
		return;
	}

	LOG_INF("I²C scan on %s:", i2c->name);
	int found = 0;
	for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
		uint8_t b;
		if (i2c_read(i2c, &b, 1, addr) != 0) {
			continue;
		}
		const char *hint = "";
		switch (addr) {
		case 0x1E: hint = " (LIS2MDL magnetometer)"; break;
		case 0x29: hint = " (VL53L5CX or VL53L1X — default)"; break;
		case 0x2A: hint = " (VL53L5CX — post-shift)"; break;
		case 0x6A: hint = " (LSM6DSO IMU — SA0 low)"; break;
		case 0x6B: hint = " (LSM6DSO IMU — SA0 high)"; break;
		default: break;
		}
		LOG_INF("  found 0x%02x%s", addr, hint);
		found++;
	}
	if (found == 0) {
		LOG_ERR("  no devices responded — check 3V3 / GND / SDA / SCL on the Qwiic chain");
	} else {
		LOG_INF("  %d device(s) on the bus", found);
	}
}

#if defined(CONFIG_HAPNAV_HEAD_TOF)
static int bring_up_head_sensor(void)
{
	/* The Zephyr in-tree VL53L1X driver is marked zephyr,deferred-init in
	 * DT. The address-conflict dance:
	 *
	 *   1. App pulls XSHUT low                   → VL53L1X off-bus
	 *   2. App inits VL53L5CX at 0x29 / shifts it to 0x2A
	 *   3. App calls device_init() on the head sensor → driver pulls
	 *      XSHUT high and probes at 0x29 unobstructed.
	 *
	 * On a power cycle the VL53L5CX boots back at 0x29 and the dance
	 * repeats — addresses are not persistent on either chip. */

	int err = device_init(tof_head);
	if (err) {
		LOG_WRN("VL53L1X (head) deferred init failed: %d (continuing)", err);
		return err;
	}
	if (!device_is_ready(tof_head)) {
		LOG_WRN("VL53L1X (head) not ready after device_init (continuing)");
		return -ENODEV;
	}
	LOG_INF("VL53L1X (head) ready");
	return 0;
}
#endif /* CONFIG_HAPNAV_HEAD_TOF */

int pin_sensors_init(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	int err;

	i2c_bus = i2c;   /* stash for bus-stall recovery during sampling */

	madgwick_init(&ahrs, MADGWICK_BETA);
	hapnav_obstacle_init();

	/* Hold the VL53L1X in reset *before* we touch the I²C bus so its
	 * default 0x29 address doesn't collide with the VL53L5CX init. */
	if (head_xshut.port == NULL || !device_is_ready(head_xshut.port)) {
		LOG_ERR("VL53L1X XSHUT GPIO controller not ready");
	} else {
		err = gpio_pin_configure_dt(&head_xshut, GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("Failed to drive VL53L1X XSHUT low: %d", err);
		}
		k_msleep(2);   /* let the chip see the reset edge */
	}

	/* Enumerate the bus before any individual sensor init so a missing
	 * chip is unambiguously diagnosable from the log. The VL53L1X is
	 * held in XSHUT so it should NOT appear; everything else (IMU, mag,
	 * VL53L5CX at default 0x29) should. */
	i2c_scan_log(i2c);

	err = hapnav_lsm6dso_init(&imu, i2c, LSM6DSO_I2C_ADDR);
	if (err) {
		LOG_ERR("LSM6DSO init failed: %d", err);
		return err;
	}

	err = hapnav_lis2mdl_init(&mag, i2c, LIS2MDL_I2C_ADDR);
	if (err) {
		LOG_ERR("LIS2MDL init failed: %d", err);
		return err;
	}

	/* The VL53L5CX retains its software-set I²C address as long as it
	 * keeps power. On a warm MCU reset the buck rail to the sensor never
	 * dropped, so the chip is still at the shifted 0x2A from a previous
	 * boot. Probe the default first, fall back to the shifted address,
	 * and skip the shift step on the warm path. */
	bool already_shifted = false;
	err = hapnav_vl53l5cx_init(&tof, i2c, VL53L5CX_DEFAULT_ADDR_8BIT);
	if (err) {
		LOG_INF("VL53L5CX not at 0x29 — probing retained 0x2A (warm reset?)");
		err = hapnav_vl53l5cx_init(&tof, i2c, VL53L5CX_SHIFTED_ADDR_8BIT);
		if (err == 0) {
			already_shifted = true;
		}
	}

	if (err) {
		LOG_WRN("VL53L5CX init failed at 0x29 and 0x2A: %d "
			"(continuing without ToF)", err);
		tof_ok = false;
	} else if (already_shifted) {
		LOG_INF("VL53L5CX already at 7-bit 0x%02x from a previous boot",
			VL53L5CX_SHIFTED_ADDR_8BIT >> 1);
		tof_ok = true;
	} else {
		/* Cold path: chip just booted at 0x29, move it off so the head
		 * VL53L1X can claim that address. */
		uint8_t st = vl53l5cx_set_i2c_address(&tof.cfg,
						      VL53L5CX_SHIFTED_ADDR_8BIT);
		if (st != 0) {
			LOG_ERR("VL53L5CX address shift to 0x%02x failed: %u",
				VL53L5CX_SHIFTED_ADDR_8BIT, st);
			tof_ok = false;
		} else {
			LOG_INF("VL53L5CX moved to 7-bit 0x%02x",
				VL53L5CX_SHIFTED_ADDR_8BIT >> 1);
			tof_ok = true;
		}
	}

	/* Now the bus is clear at 0x29 — bring up the head sensor (if
	 * enabled). When disabled via Kconfig, XSHUT stays low (configured
	 * above) so a still-wired VL53L1X chip remains in reset and can't
	 * contend at 0x29. */
#if defined(CONFIG_HAPNAV_HEAD_TOF)
	tof_head_ok = (bring_up_head_sensor() == 0);
#else
	LOG_INF("VL53L1X (head) disabled via CONFIG_HAPNAV_HEAD_TOF=n; "
		"XSHUT held low");
	tof_head_ok = false;
#endif

	return 0;
}

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
	int rc = sensor_sample_fetch(tof_head);
	if (rc) {
		head_distance_mm = -1;
		return;
	}
	struct sensor_value val;
	rc = sensor_channel_get(tof_head, SENSOR_CHAN_DISTANCE, &val);
	if (rc) {
		head_distance_mm = -1;
		return;
	}
	int32_t mm = val.val1;    /* driver returns RangeMilliMeter in val1 */
	if (mm < HEAD_PROXIMITY_MIN_MM || mm > 4000) {
		head_distance_mm = -1; /* unreliable */
		return;
	}
	head_distance_mm = (mm > INT16_MAX) ? INT16_MAX : (int16_t)mm;
#endif /* CONFIG_HAPNAV_HEAD_TOF */
}

/* Track consecutive I2C read failures and recover the bus after a short
 * streak. Fed with the IMU/mag read result each sample — those two
 * stateless reads at the top of the cycle are the canary for bus health.
 * A successful read clears the streak; a wedged bus produces an unbounded
 * run of failures and trips recovery. */
static void i2c_health_update(int err)
{
	if (err == 0) {
		if (i2c_fail_streak) {
			LOG_INF("I2C: reads healthy again after %d failure(s)",
				i2c_fail_streak);
		}
		i2c_fail_streak = 0;
		return;
	}

	if (++i2c_fail_streak < I2C_RECOVER_AFTER_FAILS) {
		return;
	}

	LOG_WRN("I2C: %d consecutive read failures (last %d) — recovering bus",
		i2c_fail_streak, err);
	int rc = i2c_recover_bus(i2c_bus);
	if (rc == -ENOSYS) {
		LOG_ERR("I2C: controller has no recover_bus support");
	} else if (rc) {
		LOG_ERR("I2C: recover_bus failed: %d", rc);
	} else {
		LOG_INF("I2C: bus recovered — stuck slave released");
	}
	/* Clean slate so the next samples re-prove the bus is back (or trip
	 * recovery again if the same device keeps wedging it). */
	i2c_fail_streak = 0;
}

int pin_sensors_sample(struct hapnav_frame *out)
{
	int16_t accel_raw[3], gyro_raw[3], mag_raw[3];
	int err;

	memset(out, 0, sizeof(*out));
	out->timestamp_ms = k_uptime_get_32();

	err = hapnav_lsm6dso_read_raw(&imu, accel_raw, gyro_raw);
	if (err) {
		i2c_health_update(err);
		return err;
	}
	err = hapnav_lis2mdl_read_raw(&mag, mag_raw);
	if (err) {
		i2c_health_update(err);
		return err;
	}
	i2c_health_update(0);   /* both core reads OK → bus is healthy */

	/* ── Stage 1: LIS2MDL silkscreen → LSM6DSO (IMU) chip frame ────────
	 *
	 *   IMU.X = Mag.Y   (same polarity)
	 *   IMU.Y = Mag.X   (same polarity)
	 *   IMU.Z = -Mag.Z  (opposite polarity)
	 */
	int16_t mag_in_imu[3] = {
		 mag_raw[1],   /* X_IMU = Y_mag */
		 mag_raw[0],   /* Y_IMU = X_mag */
		-mag_raw[2],   /* Z_IMU = -Z_mag */
	};

	/* ── Stage 2: IMU chip frame → body frame ──────────────────────────
	 *
	 * The body frame the obstacle pipeline expects is
	 * "+X right, +Y back, +Z up". The pin v1 mounting has the LSM6DSO
	 * with its +Y axis pointing up (gravity reads ≈ +1 g on accel[1]
	 * when the pin sits upright), so the chip frame is rotated 90°
	 * about +X from the body frame. The remap that makes the two
	 * agree is:
	 *
	 *   body.X = imu.X
	 *   body.Y = -imu.Z       (chip +Z is forward; body +Y is back)
	 *   body.Z = imu.Y        (chip +Y is up;      body +Z is up)
	 *
	 * Without this remap, Madgwick was being fed gravity on +Y and
	 * "fixing" the frame mismatch by producing a quaternion that
	 * rotated every body ray by ~90° — which scrambled the
	 * world-frame Z classification in obstacle.c (everything became
	 * floor or ceiling, DROPOFF flagged every frame).
	 *
	 * If the pin enclosure is re-oriented in a future revision,
	 * update this remap; gravity-on-+Z (≈ +1 g on accel[2] after the
	 * remap with the pin upright) is the verification.
	 */
	int16_t accel_body[3] = { accel_raw[0], -accel_raw[2], accel_raw[1] };
	int16_t gyro_body[3]  = { gyro_raw[0],  -gyro_raw[2],  gyro_raw[1]  };
	int16_t mag_body[3]   = { mag_in_imu[0], -mag_in_imu[2], mag_in_imu[1] };

	/* Stash the body-frame mag back into mag_raw so the trace line
	 * below shows what Madgwick actually saw. */
	memcpy(mag_raw, mag_body, sizeof(mag_raw));
	memcpy(accel_raw, accel_body, sizeof(accel_raw));
	memcpy(gyro_raw, gyro_body, sizeof(gyro_raw));

	float accel_g[3] = {
		accel_raw[0] * ACCEL_G_PER_LSB,
		accel_raw[1] * ACCEL_G_PER_LSB,
		accel_raw[2] * ACCEL_G_PER_LSB,
	};
	float gyro_radps[3] = {
		gyro_raw[0] * MDPS_TO_RADPS,
		gyro_raw[1] * MDPS_TO_RADPS,
		gyro_raw[2] * MDPS_TO_RADPS,
	};

	madgwick_update(&ahrs,
		accel_raw[0] * MG_TO_MPS2,
		accel_raw[1] * MG_TO_MPS2,
		accel_raw[2] * MG_TO_MPS2,
		gyro_radps[0], gyro_radps[1], gyro_radps[2],
		mag_raw[0]   * HAPNAV_MAG_MGAUSS_PER_LSB,
		mag_raw[1]   * HAPNAV_MAG_MGAUSS_PER_LSB,
		mag_raw[2]   * HAPNAV_MAG_MGAUSS_PER_LSB,
		SAMPLE_DT_S);

	memcpy(out->quat, ahrs.q, sizeof(out->quat));

	if (tof_ok) {
		int rc = hapnav_vl53l5cx_poll(&tof);
		if (rc < 0) {
			LOG_WRN("ToF poll error: %d", rc);
		} else if (rc == 1) {
			tof_have_frame = true;
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

	update_head_distance();

	hapnav_obstacle_step(out->distances_mm, out->target_status,
			     out->quat, gyro_radps, accel_g,
			     head_distance_mm, HEAD_PROXIMITY_MAX_MM,
			     SAMPLE_DT_S, &out->obstacles);

	/* Trace each sample as a compact line on UART. Two formats:
	 *   - every frame (10 Hz): summary — timestamp, raw IMU/mag, quat,
	 *     head distance, obstacles block.
	 *   - every 10th frame (1 Hz): the same fields PLUS the 64-pixel
	 *     distance + status arrays. Throttling the heavy line keeps the
	 *     log readable without losing the ToF detail entirely. */
	{
		static uint32_t trace_div;
		bool dump_grid = (trace_div++ % 10) == 0;
		const struct hapnav_obstacles *o = &out->obstacles;

		printk("PIN {\"ts\":%u,"
		       "\"a\":[%d,%d,%d],\"g\":[%d,%d,%d],\"m\":[%d,%d,%d],"
		       "\"q\":[%.3f,%.3f,%.3f,%.3f],\"head\":%d,"
		       "\"obs\":{\"urg\":[%u,%u,%u,%u],\"near\":%d,\"flags\":\"0x%02x\"}",
		       out->timestamp_ms,
		       accel_raw[0], accel_raw[1], accel_raw[2],
		       gyro_raw[0],  gyro_raw[1],  gyro_raw[2],
		       mag_raw[0],   mag_raw[1],   mag_raw[2],
		       (double)out->quat[0], (double)out->quat[1],
		       (double)out->quat[2], (double)out->quat[3],
		       (int)head_distance_mm,
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
