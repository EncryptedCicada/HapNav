/*
 * DA7280 — Dialog/Renesas haptic driver IC, Zephyr-native minimal port.
 *
 * Why not the upstream Linux driver? The kernel `da7280.c` we keep at
 * Drivers/da7280/ uses regmap, devm_*, work_struct, threaded_irq, and the
 * Force-Feedback input subsystem — none of which exist on Zephyr. Porting
 * it would be a rewrite. We only need DRO (I2C-direct vibration) mode
 * with fixed-frequency open-loop LRA drive, so this module covers exactly
 * that surface.
 *
 * Coverage:
 *   - LRA open-loop, fixed resonance (no auto-tracking, no BEMF sensing)
 *   - DRO mode: drive amplitude is one byte at register TOP_CTL2
 *   - Acceleration mode ON (default): TOP_CTL2 is signed 7-bit, max 127
 *
 * Per-actuator parameters target the SparkFun Qwiic LRA (G1040003D):
 *   resonance 170 Hz, R = 13.8 Ω, V_NOM = V_ABS = 2.5 Vrms, I_max = 170 mA.
 */
#ifndef HAPNAV_DA7280_H_
#define HAPNAV_DA7280_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#define DA7280_DEFAULT_ADDR     0x4A
#define DA7280_CHIP_REV_VALUE   0xBA
#define DA7280_AMP_MAX          0x7F  /* 7-bit when acceleration on */

struct hapnav_da7280_cfg {
	float    resonant_freq_hz;     /* 170 for G1040003D                  */
	float    nom_voltage_vrms;     /* 2.5                                */
	float    abs_voltage_vrms;     /* 2.5                                */
	float    impedance_ohm;        /* 13.8                               */
	float    imax_ma;              /* 170                                */
	bool     acceleration_en;      /* true → 7-bit signed amplitude      */
	bool     rapid_stop_en;        /* true                               */
	bool     freq_track_en;        /* false (open loop)                  */
	bool     bemf_sense_en;        /* false                              */
};

/* Sensible defaults for the SparkFun Qwiic LRA. */
extern const struct hapnav_da7280_cfg HAPNAV_DA7280_CFG_DEFAULT;

struct hapnav_da7280 {
	const struct device *i2c_bus;
	uint8_t              addr_7bit;
	bool                 active;
};

/*
 * Run the chip's init sequence. The caller is responsible for ensuring
 * that any upstream I2C mux is already pointed at this driver before
 * each call into this module.
 */
int hapnav_da7280_init(struct hapnav_da7280 *dev,
		       const struct device *i2c_bus, uint8_t addr_7bit,
		       const struct hapnav_da7280_cfg *cfg);

/*
 * Drive the LRA at `amplitude` (0..DA7280_AMP_MAX). Setting 0 stops
 * vibration but keeps the chip in DRO mode so the next non-zero write
 * starts immediately (no mode-switch latency).
 */
int hapnav_da7280_set_amplitude(struct hapnav_da7280 *dev, uint8_t amplitude);

/* Force the chip into INACTIVE (low-power, no drive). */
int hapnav_da7280_stop(struct hapnav_da7280 *dev);

/*
 * Bring the chip back from INACTIVE into DRO at amplitude 0, without
 * re-running the full init. Cheap recovery path for the watchdog
 * when a BLE link returns after a stale interval.
 */
int hapnav_da7280_resume(struct hapnav_da7280 *dev);

/* Read & clear all latched IRQ events; returns the cleared event bits. */
int hapnav_da7280_clear_irq(struct hapnav_da7280 *dev, uint8_t *events_out);

#endif /* HAPNAV_DA7280_H_ */
