/*
 * PCA9546A — 4-channel I2C bus switch.
 *
 * Single control register (no register address): one write byte where
 * bit[N] enables downstream channel N. Multiple bits set means parallel
 * fan-out; we only ever select one channel at a time so all DA7280s can
 * share the same 7-bit address (0x4A).
 */
#ifndef HAPNAV_PCA9546A_H_
#define HAPNAV_PCA9546A_H_

#include <zephyr/device.h>
#include <stdint.h>

struct hapnav_pca9546a {
	const struct device *i2c_bus;
	uint8_t              addr_7bit;   /* default 0x70                 */
	int8_t               selected;    /* -1 = none, else channel idx  */
};

int hapnav_pca9546a_init(struct hapnav_pca9546a *mux,
			 const struct device *i2c_bus, uint8_t addr_7bit);

/* Selects exactly one downstream channel (0..3). Idempotent. */
int hapnav_pca9546a_select(struct hapnav_pca9546a *mux, uint8_t channel);

/* Disables all channels. Useful before sleep / on shutdown. */
int hapnav_pca9546a_deselect_all(struct hapnav_pca9546a *mux);

#endif /* HAPNAV_PCA9546A_H_ */
