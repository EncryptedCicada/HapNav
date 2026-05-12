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
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
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
 * head-clearance hit and set HAPNAV_OBS_FLAG_HEAD_OBSTACLE. Calibrated
 * for a 1.35 m sensor mount with the VL53L1X tilted +20° above horizontal:
 * obstacles 1.0–2.5 m forward at 1.7–2.1 m height fall inside this slant
 * range. */
#define HEAD_PROXIMITY_MAX_MM       2200
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

int pin_sensors_init(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	int err;

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

	err = hapnav_vl53l5cx_init(&tof, i2c, VL53L5CX_DEFAULT_ADDR_8BIT);
	if (err) {
		LOG_WRN("VL53L5CX init failed: %d (continuing without ToF)", err);
		tof_ok = false;
	} else {
		/* Move VL53L5CX off 0x29 so the head VL53L1X can claim it. */
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

	/* Now the bus is clear at 0x29 — bring up the head sensor. */
	tof_head_ok = (bring_up_head_sensor() == 0);

	return 0;
}

static void update_head_distance(void)
{
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
}

int pin_sensors_sample(struct hapnav_frame *out)
{
	int16_t accel_raw[3], gyro_raw[3], mag_raw[3];
	int err;

	memset(out, 0, sizeof(*out));
	out->timestamp_ms = k_uptime_get_32();

	err = hapnav_lsm6dso_read_raw(&imu, accel_raw, gyro_raw);
	if (err) {
		return err;
	}
	err = hapnav_lis2mdl_read_raw(&mag, mag_raw);
	if (err) {
		return err;
	}

	/* Mag-X is mounted opposite IMU-X on the board; flip so Madgwick
	 * sees a magnetic vector consistent with the IMU's frame. */
	mag_raw[0] = -mag_raw[0];

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
			for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
				out->distances_mm[i]  = tof.results.distance_mm[i];
				out->target_status[i] = tof.results.target_status[i];
			}
		}
	}

	update_head_distance();

	hapnav_obstacle_step(out->distances_mm, out->target_status,
			     out->quat, gyro_radps, accel_g,
			     head_distance_mm, HEAD_PROXIMITY_MAX_MM,
			     SAMPLE_DT_S, &out->obstacles);

	return 0;
}
