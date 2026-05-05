/*
 * Thin Zephyr wrapper around STMicro's lsm6dso_reg driver.
 *
 * The vendor driver is platform-agnostic; we provide the I2C read/write
 * callbacks via Zephyr's i2c_burst_{read,write}_dt-style API and surface
 * a small read-raw API for the app.
 */
#ifndef HAPNAV_LSM6DSO_H_
#define HAPNAV_LSM6DSO_H_

#include <zephyr/device.h>
#include <stdint.h>
#include "lsm6dso_reg.h"

struct hapnav_lsm6dso {
	stmdev_ctx_t        ctx;
	const struct device *i2c_bus;
	uint8_t             addr_7bit;
};

/*
 * @preferred_addr: 0x6A (SA0=0) or 0x6B (SA0=1). The driver probes the
 *                   preferred address first and falls back to the other if
 *                   WHO_AM_I doesn't match — covers both Qwiic variants.
 */
int hapnav_lsm6dso_init(struct hapnav_lsm6dso *dev,
			const struct device *i2c_bus, uint8_t preferred_addr);

int hapnav_lsm6dso_read_raw(struct hapnav_lsm6dso *dev,
			    int16_t accel[3], int16_t gyro[3]);

#endif /* HAPNAV_LSM6DSO_H_ */
