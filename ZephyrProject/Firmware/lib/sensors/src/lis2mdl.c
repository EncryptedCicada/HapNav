#include <hapnav/lis2mdl.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(hapnav_lis2mdl, LOG_LEVEL_INF);

static int32_t plat_write(void *handle, uint8_t reg,
			  const uint8_t *buf, uint16_t len)
{
	struct hapnav_lis2mdl *dev = handle;
	return i2c_burst_write(dev->i2c_bus, dev->addr_7bit, reg, buf, len);
}

static int32_t plat_read(void *handle, uint8_t reg,
			 uint8_t *buf, uint16_t len)
{
	struct hapnav_lis2mdl *dev = handle;
	return i2c_burst_read(dev->i2c_bus, dev->addr_7bit, reg, buf, len);
}

static void plat_mdelay(uint32_t ms)
{
	k_msleep(ms);
}

int hapnav_lis2mdl_init(struct hapnav_lis2mdl *dev,
			const struct device *i2c_bus, uint8_t addr_7bit)
{
	if (!device_is_ready(i2c_bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	dev->i2c_bus       = i2c_bus;
	dev->addr_7bit     = addr_7bit;
	dev->ctx.write_reg = plat_write;
	dev->ctx.read_reg  = plat_read;
	dev->ctx.mdelay    = plat_mdelay;
	dev->ctx.handle    = dev;

	k_msleep(20); /* boot time */

	uint8_t whoami = 0;
	if (lis2mdl_device_id_get(&dev->ctx, &whoami) != 0) {
		LOG_ERR("WHO_AM_I read failed");
		return -EIO;
	}
	if (whoami != LIS2MDL_ID) {
		LOG_ERR("WHO_AM_I mismatch: 0x%02x (expected 0x%02x)",
			whoami, LIS2MDL_ID);
		return -ENODEV;
	}

	lis2mdl_reset_set(&dev->ctx, PROPERTY_ENABLE);
	uint8_t rst;
	do {
		lis2mdl_reset_get(&dev->ctx, &rst);
	} while (rst);

	lis2mdl_block_data_update_set(&dev->ctx, PROPERTY_ENABLE);
	lis2mdl_data_rate_set(&dev->ctx, LIS2MDL_ODR_50Hz);
	lis2mdl_set_rst_mode_set(&dev->ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
	lis2mdl_offset_temp_comp_set(&dev->ctx, PROPERTY_ENABLE);
	lis2mdl_operating_mode_set(&dev->ctx, LIS2MDL_CONTINUOUS_MODE);

	LOG_INF("LIS2MDL ready (addr 0x%02x, 50 Hz continuous)", addr_7bit);
	return 0;
}

int hapnav_lis2mdl_read_raw(struct hapnav_lis2mdl *dev, int16_t mag[3])
{
	if (lis2mdl_magnetic_raw_get(&dev->ctx, mag) != 0) {
		return -EIO;
	}
	return 0;
}
