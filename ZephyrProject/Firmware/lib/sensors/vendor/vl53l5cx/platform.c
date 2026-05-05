/*
 * Zephyr-side implementation of the VL53L5CX ULD platform callbacks.
 *
 * The VL53L5CX uses 16-bit register addressing — Zephyr's i2c_burst_*() helpers
 * only handle 8-bit registers, so we drop down to i2c_write_dt-style messages.
 *
 * The ULD writes some long blocks (firmware blob is ~80 KB) so WrMulti has to
 * tolerate large `size`; a stack scratch would blow the main stack. We send the
 * 2-byte register prefix and the payload as separate i2c_msg entries in a single
 * transfer (no STOP between them).
 */

#include "platform.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <string.h>

static inline uint8_t addr_7bit(const VL53L5CX_Platform *p)
{
	return (uint8_t)(p->address >> 1);
}

uint8_t RdByte(VL53L5CX_Platform *p, uint16_t reg, uint8_t *value)
{
	return RdMulti(p, reg, value, 1);
}

uint8_t WrByte(VL53L5CX_Platform *p, uint16_t reg, uint8_t value)
{
	return WrMulti(p, reg, &value, 1);
}

uint8_t RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t size)
{
	uint8_t reg_be[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
	int rc = i2c_write_read(p->i2c_bus, addr_7bit(p),
				reg_be, sizeof(reg_be), buf, size);
	return (rc == 0) ? 0 : 255;
}

uint8_t WrMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t size)
{
	uint8_t reg_be[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
	struct i2c_msg msgs[2] = {
		{ .buf = reg_be,           .len = sizeof(reg_be),
		  .flags = I2C_MSG_WRITE },
		{ .buf = (uint8_t *)buf,   .len = size,
		  .flags = I2C_MSG_WRITE | I2C_MSG_STOP },
	};
	int rc = i2c_transfer(p->i2c_bus, msgs, 2, addr_7bit(p));
	return (rc == 0) ? 0 : 255;
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
	for (uint16_t i = 0; i < size; i += 4) {
		uint32_t tmp = ((uint32_t)buffer[i]     << 24) |
			       ((uint32_t)buffer[i + 1] << 16) |
			       ((uint32_t)buffer[i + 2] <<  8) |
			       ((uint32_t)buffer[i + 3]);
		memcpy(&buffer[i], &tmp, 4);
	}
}

uint8_t WaitMs(VL53L5CX_Platform *p, uint32_t TimeMs)
{
	ARG_UNUSED(p);
	k_msleep(TimeMs);
	return 0;
}
