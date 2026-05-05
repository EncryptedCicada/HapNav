#include "sensors.h"

#include <hapnav/lsm6dso.h>
#include <hapnav/lis2mdl.h>
#include <hapnav/vl53l5cx.h>
#include <hapnav/madgwick.h>
#include <hapnav/obstacle.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(pin_sensors, LOG_LEVEL_INF);

#define LSM6DSO_I2C_ADDR  0x6B
#define LIS2MDL_I2C_ADDR  0x1E
#define VL53L5CX_I2C_ADDR 0x52  /* 8-bit; Zephyr sees 7-bit 0x29 */

#define SAMPLE_PERIOD_MS  100        /* must match the main-loop cadence */
#define SAMPLE_DT_S       (SAMPLE_PERIOD_MS / 1000.0f)
#define MADGWICK_BETA     0.1f

/* mg → m/s²; mdps → rad/s; mGauss is unitless to Madgwick (only ratios matter). */
#define MG_TO_MPS2          (HAPNAV_ACCEL_MG_PER_LSB * 0.00980665f)
#define MDPS_TO_RADPS       (HAPNAV_GYRO_MDPS_PER_LSB * (3.14159265f / 180000.0f))
#define ACCEL_G_PER_LSB     (HAPNAV_ACCEL_MG_PER_LSB * 0.001f)

static struct hapnav_lsm6dso  imu;
static struct hapnav_lis2mdl  mag;
static struct hapnav_vl53l5cx tof;
static struct madgwick        ahrs;
static bool                   tof_ok;
static bool                   tof_have_frame;  /* true after first poll==1 */

int pin_sensors_init(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	int err;

	madgwick_init(&ahrs, MADGWICK_BETA);
	hapnav_obstacle_init();

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

	err = hapnav_vl53l5cx_init(&tof, i2c, VL53L5CX_I2C_ADDR);
	if (err) {
		LOG_WRN("VL53L5CX init failed: %d (continuing without ToF)", err);
		tof_ok = false;
	} else {
		tof_ok = true;
	}

	return 0;
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

	hapnav_obstacle_step(out->distances_mm, out->target_status,
			     out->quat, gyro_radps, accel_g,
			     SAMPLE_DT_S, &out->obstacles);

	return 0;
}
