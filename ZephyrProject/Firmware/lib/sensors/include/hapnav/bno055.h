/*
 * Thin I²C driver for the Bosch BNO055 in NDOF (9-DOF fusion) mode.
 *
 * The chip's onboard fusion firmware delivers a body→world unit quaternion at
 * up to 100 Hz, which replaces the LSM6DSO + LIS2MDL + Madgwick stack we ran
 * on the pin previously. The same chip also exposes raw accel and gyro that
 * the obstacle pipeline still consumes (for stationary detection and the
 * yaw-slew gate).
 *
 * The BNO055 lives on the TCA9548A mux's ch3 at 7-bit 0x28 — the Adafruit
 * BNO055+BMP280 BFF's stock I²C-mode address (PS1=PS0=0, COM3 pulled low).
 * No in-tree Zephyr driver; this wrapper takes the channel-bus device handle
 * + address and talks register-level I²C against it.
 *
 * Coordinate frames: BNO055 internal axes are set by AXIS_REMAP_CONFIG /
 * AXIS_REMAP_SIGN. Until the chip's physical orientation on the pin is
 * confirmed (verify gravity reads on body +Z when the pin is upright), this
 * driver leaves the remap at chip default (P1, identity). Adjust by calling
 * hapnav_bno055_set_axis_remap() once the mounting is known.
 */
#ifndef HAPNAV_BNO055_H_
#define HAPNAV_BNO055_H_

#include <zephyr/device.h>
#include <stdint.h>

struct hapnav_bno055 {
	const struct device *i2c_bus;
	uint16_t             addr;       /* 7-bit, 0x28 default */
};

/*
 * Bring the chip up: probe CHIP_ID, switch to NDOF fusion mode. The boot
 * delay built into the firmware (CONFIG_BOOT_DELAY=2000 ms) is more than
 * enough for the BNO055's ~650 ms power-on; init still retries a few times
 * to ride out any stragglers.
 *
 * Returns 0 on success, negative errno otherwise.
 */
int hapnav_bno055_init(struct hapnav_bno055 *b,
		       const struct device *i2c_bus,
		       uint16_t             addr);

/*
 * One I²C burst pulls accel (m/s² → g), gyro (dps → rad/s), and the fused
 * quaternion in a single transaction.
 *   quat[]      = {w, x, y, z}, body→world, unit length
 *   accel_g[3]  = body-frame acceleration in g (gravity included)
 *   gyro_rad[3] = body-frame angular rate in rad/s
 *
 * Returns 0 on success, negative errno on I²C error.
 */
int hapnav_bno055_read_motion(struct hapnav_bno055 *b,
			      float quat[4],
			      float accel_g[3],
			      float gyro_rad[3]);

/*
 * Read the four calibration counters (sys/gyr/acc/mag, 0..3 each). Useful
 * for telling the user when fusion is stable enough to trust. Any pointer
 * may be NULL.
 */
int hapnav_bno055_get_calib(struct hapnav_bno055 *b,
			    uint8_t *sys, uint8_t *gyr,
			    uint8_t *acc, uint8_t *mag);

/*
 * Reconfigure the axis remap. `config` goes into AXIS_REMAP_CONFIG, `sign`
 * into AXIS_REMAP_SIGN; see the BNO055 datasheet §3.4 for the eight standard
 * placements (P0..P7) and their byte values. The chip is bounced through
 * CONFIG mode internally so the caller does not need to worry about it.
 */
int hapnav_bno055_set_axis_remap(struct hapnav_bno055 *b,
				 uint8_t config, uint8_t sign);

#endif /* HAPNAV_BNO055_H_ */
