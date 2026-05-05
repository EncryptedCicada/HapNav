/*
 * Zephyr replacement for ST's VL53L5CX ULD platform.h template.
 *
 * The ULD source includes "platform.h" and only uses the symbols below; we
 * keep the same names and semantics so the upstream submodule stays untouched.
 *
 * The VL53L5CX_Platform struct is augmented with a Zephyr device pointer for
 * the I²C bus, since the ULD's Read/Write callbacks receive only the address.
 */
#ifndef HAPNAV_VL53L5CX_PLATFORM_H_
#define HAPNAV_VL53L5CX_PLATFORM_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/device.h>

/* ── ULD compile-time configuration ───────────────────────────────────────── */

#ifndef VL53L5CX_NB_TARGET_PER_ZONE
#define VL53L5CX_NB_TARGET_PER_ZONE  (1U)
#endif

/* Trim outputs we don't currently need to shrink RAM and I²C traffic. */
#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_AMBIENT_DMAX
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
/* keep DISTANCE_MM, TARGET_STATUS, NB_TARGET_DETECTED, MOTION_INDICATOR */

#define PROCESSOR_LITTLE_ENDIAN
#ifdef PROCESSOR_LITTLE_ENDIAN
  #define SWAP_UINT16(x) (x)
  #define SWAP_UINT32(x) (x)
#else
  #define SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))
  #define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) \
		       | (((x) & 0x0000FF00) << 8) | ((x) << 24))
#endif

/* ── platform context the ULD passes back to our callbacks ────────────────── */

typedef struct {
	uint16_t             address;   /* 8-bit I²C addr (e.g. 0x52) */
	const struct device *i2c_bus;   /* Zephyr I²C controller */
} VL53L5CX_Platform;

/* ── platform callbacks the ULD calls (return 0 on success) ───────────────── */

uint8_t RdByte(VL53L5CX_Platform *p, uint16_t reg, uint8_t *value);
uint8_t WrByte(VL53L5CX_Platform *p, uint16_t reg, uint8_t value);
uint8_t RdMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t size);
uint8_t WrMulti(VL53L5CX_Platform *p, uint16_t reg, uint8_t *buf, uint32_t size);
void    SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53L5CX_Platform *p, uint32_t TimeMs);

#endif /* HAPNAV_VL53L5CX_PLATFORM_H_ */
