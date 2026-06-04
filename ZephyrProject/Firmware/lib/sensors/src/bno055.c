#include <hapnav/bno055.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <errno.h>

LOG_MODULE_REGISTER(hapnav_bno055, LOG_LEVEL_INF);

/* ── Register map (page 0, datasheet §4.2.1) ─────────────────────────────── */

#define REG_CHIP_ID            0x00
#define REG_PAGE_ID            0x07
#define REG_ACC_DATA           0x08    /* 6 bytes — accel x/y/z */
#define REG_GYR_DATA           0x14    /* 6 bytes — gyro  x/y/z */
#define REG_QUA_DATA           0x20    /* 8 bytes — quat  w/x/y/z (Q14) */
#define REG_CALIB_STAT         0x35
#define REG_UNIT_SEL           0x3B
#define REG_OPR_MODE           0x3D
#define REG_POWER_MODE         0x3E
#define REG_SYS_TRIGGER        0x3F
#define REG_AXIS_REMAP_CONFIG  0x41
#define REG_AXIS_REMAP_SIGN    0x42

#define CHIP_ID_BNO055         0xA0

#define OPR_MODE_CONFIG        0x00
#define OPR_MODE_NDOF          0x0C

#define POWER_MODE_NORMAL      0x00

#define SYS_TRIGGER_INTERNAL_CLK 0x00  /* CLK_SEL bit 7 = 0, no triggers */
#define SYS_TRIGGER_RST_SYS      0x20  /* bit 5: trigger full system reset */

#define POR_DELAY_MS           700     /* datasheet: ~650 ms after POR */

/* Quaternion is fixed-point Q14: integer / 16384 = unit value. */
#define QUAT_LSB_PER_UNIT      16384.0f

/* Accel reg returns m/s² × 100 LSB (UNIT_SEL.acc_unit = 0, default). */
#define ACC_LSB_PER_MPS2       100.0f
#define MPS2_PER_G             9.80665f

/* Gyro reg returns dps × 16 LSB (UNIT_SEL.gyr_unit = 0, default). */
#define GYR_LSB_PER_DPS        16.0f
#define DEG_TO_RAD             0.01745329252f

/* Datasheet §3.6: mode-change latency is ≥7 ms going *from* CONFIG and ≥19 ms
 * going *to* CONFIG. Round up to 25 ms either way — well within budget for an
 * init-time call. */
#define MODE_CHANGE_DELAY_MS   25

/* ── Public API ──────────────────────────────────────────────────────────── */

int hapnav_bno055_init(struct hapnav_bno055 *b,
		       const struct device *i2c_bus,
		       uint16_t             addr)
{
	if (!device_is_ready(i2c_bus)) {
		LOG_ERR("I²C bus %s not ready",
			i2c_bus ? i2c_bus->name : "(null)");
		return -ENODEV;
	}
	b->i2c_bus = i2c_bus;
	b->addr    = addr;

	uint8_t id = 0;
	int rc = -EIO;
	for (int retry = 0; retry < 10; retry++) {
		rc = i2c_reg_read_byte(i2c_bus, addr, REG_CHIP_ID, &id);
		if (rc == 0 && id == CHIP_ID_BNO055) {
			break;
		}
		k_msleep(50);
	}
	if (rc || id != CHIP_ID_BNO055) {
		LOG_ERR("BNO055 not found at 0x%02x (chip_id=0x%02x, last rc=%d)",
			addr, id, rc);
		return -ENODEV;
	}

	/* Linux-style configure sequence (bno055.c:355-387): always force
	 * CONFIG mode, set the clock source explicitly, then power mode, then
	 * units. POR defaults *should* leave everything correct but a warm
	 * boot with the chip still powered can land in odd states; being
	 * explicit costs us four extra writes and ~25 ms. */
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_OPR_MODE, OPR_MODE_CONFIG);
	k_msleep(MODE_CHANGE_DELAY_MS);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_PAGE_ID, 0x00);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_POWER_MODE, POWER_MODE_NORMAL);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_SYS_TRIGGER,
				 SYS_TRIGGER_INTERNAL_CLK);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_UNIT_SEL, 0x00);

	/* Before entering any fusion mode, do a full system reset and rebuild
	 * the configuration. This mirrors what the Linux driver does in
	 * bno055_operation_mode_set() (bno055.c:401-415): the fusion
	 * algorithm holds internal state that survives an OPR_MODE write, so
	 * to guarantee a clean start we have to bounce the chip. */
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_SYS_TRIGGER,
				 SYS_TRIGGER_RST_SYS);
	k_msleep(POR_DELAY_MS);

	/* Re-probe after reset (registers cleared, chip back at defaults). */
	for (int retry = 0; retry < 10; retry++) {
		rc = i2c_reg_read_byte(i2c_bus, addr, REG_CHIP_ID, &id);
		if (rc == 0 && id == CHIP_ID_BNO055) {
			break;
		}
		k_msleep(50);
	}
	if (rc || id != CHIP_ID_BNO055) {
		LOG_ERR("BNO055 lost after soft reset (id=0x%02x rc=%d)",
			id, rc);
		return -ENODEV;
	}

	(void)i2c_reg_write_byte(i2c_bus, addr, REG_OPR_MODE, OPR_MODE_CONFIG);
	k_msleep(MODE_CHANGE_DELAY_MS);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_PAGE_ID, 0x00);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_POWER_MODE, POWER_MODE_NORMAL);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_SYS_TRIGGER,
				 SYS_TRIGGER_INTERNAL_CLK);
	(void)i2c_reg_write_byte(i2c_bus, addr, REG_UNIT_SEL, 0x00);

	rc = i2c_reg_write_byte(i2c_bus, addr, REG_OPR_MODE, OPR_MODE_NDOF);
	if (rc) {
		LOG_ERR("BNO055 NDOF entry failed: %d", rc);
		return rc;
	}
	/* Give the fusion loop a moment to produce a first valid sample. */
	k_msleep(50);

	LOG_INF("BNO055 ready at 0x%02x (chip_id=0xA0, NDOF mode)", addr);
	return 0;
}

int hapnav_bno055_set_axis_remap(struct hapnav_bno055 *b,
				 uint8_t config, uint8_t sign)
{
	int rc;
	rc = i2c_reg_write_byte(b->i2c_bus, b->addr, REG_OPR_MODE,
				OPR_MODE_CONFIG);
	if (rc) return rc;
	k_msleep(MODE_CHANGE_DELAY_MS);

	rc = i2c_reg_write_byte(b->i2c_bus, b->addr,
				REG_AXIS_REMAP_CONFIG, config);
	if (rc) goto out;
	rc = i2c_reg_write_byte(b->i2c_bus, b->addr,
				REG_AXIS_REMAP_SIGN, sign);
out:
	(void)i2c_reg_write_byte(b->i2c_bus, b->addr, REG_OPR_MODE,
				 OPR_MODE_NDOF);
	k_msleep(50);
	return rc;
}

int hapnav_bno055_get_calib(struct hapnav_bno055 *b,
			    uint8_t *sys, uint8_t *gyr,
			    uint8_t *acc, uint8_t *mag)
{
	uint8_t s;
	int rc = i2c_reg_read_byte(b->i2c_bus, b->addr, REG_CALIB_STAT, &s);
	if (rc) {
		return rc;
	}
	if (sys) *sys = (s >> 6) & 0x03;
	if (gyr) *gyr = (s >> 4) & 0x03;
	if (acc) *acc = (s >> 2) & 0x03;
	if (mag) *mag =  s        & 0x03;
	return 0;
}

int hapnav_bno055_read_motion(struct hapnav_bno055 *b,
			      float quat[4],
			      float accel_g[3],
			      float gyro_rad[3])
{
	/* One burst from 0x08..0x27 (32 bytes) covers accel, mag, gyro, euler,
	 * and quaternion. We only unpack the three we care about, but pulling
	 * them in one transaction is far cheaper than three round-trips and
	 * keeps the values mutually time-aligned. */
	uint8_t buf[32];
	int rc = i2c_burst_read(b->i2c_bus, b->addr, REG_ACC_DATA, buf,
				sizeof(buf));
	if (rc) {
		return rc;
	}

	/* Accel: 6 bytes at offset 0, int16 LE, LSB = m/s² × 100. */
	int16_t ax = (int16_t)(buf[0]  | (buf[1]  << 8));
	int16_t ay = (int16_t)(buf[2]  | (buf[3]  << 8));
	int16_t az = (int16_t)(buf[4]  | (buf[5]  << 8));
	accel_g[0] = (ax / ACC_LSB_PER_MPS2) / MPS2_PER_G;
	accel_g[1] = (ay / ACC_LSB_PER_MPS2) / MPS2_PER_G;
	accel_g[2] = (az / ACC_LSB_PER_MPS2) / MPS2_PER_G;

	/* Gyro: 6 bytes at offset 12, LSB = dps × 16. */
	int16_t gx = (int16_t)(buf[12] | (buf[13] << 8));
	int16_t gy = (int16_t)(buf[14] | (buf[15] << 8));
	int16_t gz = (int16_t)(buf[16] | (buf[17] << 8));
	gyro_rad[0] = (gx / GYR_LSB_PER_DPS) * DEG_TO_RAD;
	gyro_rad[1] = (gy / GYR_LSB_PER_DPS) * DEG_TO_RAD;
	gyro_rad[2] = (gz / GYR_LSB_PER_DPS) * DEG_TO_RAD;

	/* Quaternion: 8 bytes at offset 24, Q14 fixed-point. */
	int16_t qw = (int16_t)(buf[24] | (buf[25] << 8));
	int16_t qx = (int16_t)(buf[26] | (buf[27] << 8));
	int16_t qy = (int16_t)(buf[28] | (buf[29] << 8));
	int16_t qz = (int16_t)(buf[30] | (buf[31] << 8));
	quat[0] = qw / QUAT_LSB_PER_UNIT;
	quat[1] = qx / QUAT_LSB_PER_UNIT;
	quat[2] = qy / QUAT_LSB_PER_UNIT;
	quat[3] = qz / QUAT_LSB_PER_UNIT;

	return 0;
}
