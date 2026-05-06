#include <hapnav/pca9546a.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(hapnav_pca9546a, LOG_LEVEL_INF);

#define PCA9546A_NUM_CHANNELS 4

int hapnav_pca9546a_init(struct hapnav_pca9546a *mux,
			 const struct device *i2c_bus, uint8_t addr_7bit)
{
	if (!device_is_ready(i2c_bus)) {
		return -ENODEV;
	}
	mux->i2c_bus   = i2c_bus;
	mux->addr_7bit = addr_7bit;
	mux->selected  = -1;

	/* Probe by writing 0x00 (all channels off). If the device NAKs we'll
	 * get -EIO and the caller can decide how to handle it. */
	uint8_t off = 0x00;
	int err = i2c_write(i2c_bus, &off, 1, addr_7bit);
	if (err) {
		LOG_ERR("PCA9546A probe failed at 0x%02x: %d", addr_7bit, err);
		return err;
	}
	LOG_INF("PCA9546A ready at 0x%02x", addr_7bit);
	return 0;
}

int hapnav_pca9546a_select(struct hapnav_pca9546a *mux, uint8_t channel)
{
	if (channel >= PCA9546A_NUM_CHANNELS) {
		return -EINVAL;
	}
	/* No cache: always re-issue the channel-select write. The earlier
	 * cached version was an optimization but it also masked any drift
	 * in mux state. Adafruit's TCA9548A driver deliberately writes the
	 * channel mask on every operation for the same reason. The extra
	 * 1-byte writes are negligible at 400 kHz. */
	uint8_t mask = (uint8_t)(1U << channel);
	int err = i2c_write(mux->i2c_bus, &mask, 1, mux->addr_7bit);
	if (err) {
		return err;
	}
	mux->selected = (int8_t)channel;
	return 0;
}

int hapnav_pca9546a_deselect_all(struct hapnav_pca9546a *mux)
{
	uint8_t off = 0x00;
	int err = i2c_write(mux->i2c_bus, &off, 1, mux->addr_7bit);
	if (err == 0) {
		mux->selected = -1;
	}
	return err;
}
