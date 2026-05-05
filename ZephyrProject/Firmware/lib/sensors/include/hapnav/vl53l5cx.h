/*
 * Thin Zephyr wrapper around STMicro's VL53L5CX ULD.
 *
 * The vendor driver is platform-agnostic; we provide the I²C transfers and
 * timing via Zephyr APIs (see vendor/vl53l5cx/platform.c) and surface a small
 * init / poll-and-read API to the app.
 */
#ifndef HAPNAV_VL53L5CX_H_
#define HAPNAV_VL53L5CX_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#include "vl53l5cx_api.h"  /* pulls in our platform.h replacement */

#define HAPNAV_VL53L5CX_DEFAULT_ADDR_8BIT  0x52  /* 7-bit 0x29 */

struct hapnav_vl53l5cx {
	VL53L5CX_Configuration cfg;       /* large — keep in BSS, not stack */
	VL53L5CX_ResultsData   results;
};

/*
 * Bring the sensor up at 8x8, 10 Hz continuous, single target per zone.
 * `i2c_bus` must already be ready. `addr_8bit` is the device's full I²C
 * address byte (0x52 by default — Zephyr will use the 7-bit form internally).
 */
int hapnav_vl53l5cx_init(struct hapnav_vl53l5cx *dev,
			 const struct device *i2c_bus, uint16_t addr_8bit);

/*
 * Non-blocking poll. Returns:
 *   1 — fresh frame stored in dev->results
 *   0 — no new frame yet (caller should reuse last results)
 *  <0 — I²C / driver error
 */
int hapnav_vl53l5cx_poll(struct hapnav_vl53l5cx *dev);

#endif /* HAPNAV_VL53L5CX_H_ */
