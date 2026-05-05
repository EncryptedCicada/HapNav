/*
 * Thin Zephyr wrapper around STMicro's lis2mdl_reg driver.
 */
#ifndef HAPNAV_LIS2MDL_H_
#define HAPNAV_LIS2MDL_H_

#include <zephyr/device.h>
#include <stdint.h>
#include "lis2mdl_reg.h"

struct hapnav_lis2mdl {
	stmdev_ctx_t        ctx;
	const struct device *i2c_bus;
	uint8_t             addr_7bit;
};

/*
 * @addr_7bit: LIS2MDL fixed at 0x1E.
 */
int hapnav_lis2mdl_init(struct hapnav_lis2mdl *dev,
			const struct device *i2c_bus, uint8_t addr_7bit);

int hapnav_lis2mdl_read_raw(struct hapnav_lis2mdl *dev, int16_t mag[3]);

#endif /* HAPNAV_LIS2MDL_H_ */
