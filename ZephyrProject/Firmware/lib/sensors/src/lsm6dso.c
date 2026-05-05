#include <hapnav/lsm6dso.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(hapnav_lsm6dso, LOG_LEVEL_INF);

/* ── platform glue invoked by stmdev_ctx_t ────────────────────────────────── */

static int32_t plat_write(void *handle, uint8_t reg,
			  const uint8_t *buf, uint16_t len)
{
	struct hapnav_lsm6dso *dev = handle;
	return i2c_burst_write(dev->i2c_bus, dev->addr_7bit, reg, buf, len);
}

static int32_t plat_read(void *handle, uint8_t reg,
			 uint8_t *buf, uint16_t len)
{
	struct hapnav_lsm6dso *dev = handle;
	return i2c_burst_read(dev->i2c_bus, dev->addr_7bit, reg, buf, len);
}

static void plat_mdelay(uint32_t ms)
{
	k_msleep(ms);
}

/* ── public API ───────────────────────────────────────────────────────────── */

int hapnav_lsm6dso_init(struct hapnav_lsm6dso *dev,
			const struct device *i2c_bus, uint8_t preferred_addr)
{
	if (!device_is_ready(i2c_bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	dev->i2c_bus       = i2c_bus;
	dev->ctx.write_reg = plat_write;
	dev->ctx.read_reg  = plat_read;
	dev->ctx.mdelay    = plat_mdelay;
	dev->ctx.handle    = dev;

	k_msleep(10); /* boot time */

	/* Probe the preferred address first, then the alternate (SA0 may be
	 * tied either way depending on the breakout).
	 */
	const uint8_t alt = (preferred_addr == 0x6B) ? 0x6A : 0x6B;
	const uint8_t candidates[2] = { preferred_addr, alt };
	uint8_t whoami = 0;
	bool found = false;

	for (size_t i = 0; i < ARRAY_SIZE(candidates); i++) {
		dev->addr_7bit = candidates[i];
		int32_t rc = lsm6dso_device_id_get(&dev->ctx, &whoami);
		if (rc == 0 && whoami == LSM6DSO_ID) {
			found = true;
			break;
		}
		LOG_DBG("0x%02x: rc=%d whoami=0x%02x", dev->addr_7bit, rc, whoami);
	}

	if (!found) {
		LOG_ERR("LSM6DSO not found at 0x6A or 0x6B (last whoami=0x%02x)", whoami);
		return -ENODEV;
	}
	LOG_INF("LSM6DSO found at 0x%02x", dev->addr_7bit);

	lsm6dso_reset_set(&dev->ctx, PROPERTY_ENABLE);
	uint8_t rst;
	do {
		lsm6dso_reset_get(&dev->ctx, &rst);
	} while (rst);

	lsm6dso_i3c_disable_set(&dev->ctx, LSM6DSO_I3C_DISABLE);
	lsm6dso_block_data_update_set(&dev->ctx, PROPERTY_ENABLE);

	lsm6dso_xl_data_rate_set(&dev->ctx, LSM6DSO_XL_ODR_104Hz);
	lsm6dso_gy_data_rate_set(&dev->ctx, LSM6DSO_GY_ODR_104Hz);
	lsm6dso_xl_full_scale_set(&dev->ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev->ctx, LSM6DSO_2000dps);

	LOG_INF("LSM6DSO ready (104 Hz, ±2 g, ±2000 dps)");
	return 0;
}

int hapnav_lsm6dso_read_raw(struct hapnav_lsm6dso *dev,
			    int16_t accel[3], int16_t gyro[3])
{
	if (lsm6dso_acceleration_raw_get(&dev->ctx, accel) != 0) {
		return -EIO;
	}
	if (lsm6dso_angular_rate_raw_get(&dev->ctx, gyro) != 0) {
		return -EIO;
	}
	return 0;
}
