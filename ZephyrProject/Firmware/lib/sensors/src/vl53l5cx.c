#include <hapnav/vl53l5cx.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "vl53l5cx_api.h"

LOG_MODULE_REGISTER(hapnav_vl53l5cx, LOG_LEVEL_INF);

int hapnav_vl53l5cx_init(struct hapnav_vl53l5cx *dev,
			 const struct device *i2c_bus, uint16_t addr_8bit)
{
	if (!device_is_ready(i2c_bus)) {
		LOG_ERR("I²C bus not ready");
		return -ENODEV;
	}

	memset(dev, 0, sizeof(*dev));
	dev->cfg.platform.i2c_bus = i2c_bus;
	dev->cfg.platform.address = addr_8bit;

	uint8_t alive = 0;
	uint8_t status = vl53l5cx_is_alive(&dev->cfg, &alive);
	if (status != 0 || alive != 1) {
		LOG_ERR("VL53L5CX not alive (status=%u alive=%u) at 0x%02x",
			status, alive, addr_8bit);
		return -ENODEV;
	}

	/* Loads the ~80 KB firmware blob over I²C; takes ~1 s. */
	status = vl53l5cx_init(&dev->cfg);
	if (status != 0) {
		LOG_ERR("vl53l5cx_init failed: %u", status);
		return -EIO;
	}

	status |= vl53l5cx_set_resolution(&dev->cfg, VL53L5CX_RESOLUTION_8X8);
	status |= vl53l5cx_set_ranging_frequency_hz(&dev->cfg, 10);
	status |= vl53l5cx_set_ranging_mode(&dev->cfg,
					    VL53L5CX_RANGING_MODE_CONTINUOUS);
	status |= vl53l5cx_start_ranging(&dev->cfg);
	if (status != 0) {
		LOG_ERR("VL53L5CX configure/start failed: %u", status);
		return -EIO;
	}

	LOG_INF("VL53L5CX ready (8x8, 10 Hz, continuous)");
	return 0;
}

int hapnav_vl53l5cx_poll(struct hapnav_vl53l5cx *dev)
{
	uint8_t ready = 0;
	uint8_t status = vl53l5cx_check_data_ready(&dev->cfg, &ready);
	if (status != 0) {
		return -EIO;
	}
	if (!ready) {
		return 0;
	}
	status = vl53l5cx_get_ranging_data(&dev->cfg, &dev->results);
	if (status != 0) {
		return -EIO;
	}
	return 1;
}
