#include <hapnav/da7280.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(hapnav_da7280, LOG_LEVEL_INF);

/* Register map (subset). See Renesas DA7280 datasheet Rev. A2. */
#define REG_CHIP_REV             0x00
#define REG_IRQ_EVENT1           0x03
#define REG_IRQ_EVENT_WARN_DIAG  0x04
#define REG_IRQ_EVENT_SEQ_DIAG   0x05
#define REG_IRQ_MASK1            0x07
#define REG_FRQ_LRA_PER_H        0x0A
#define REG_FRQ_LRA_PER_L        0x0B
#define REG_ACTUATOR1            0x0C  /* nommax */
#define REG_ACTUATOR2            0x0D  /* absmax */
#define REG_ACTUATOR3            0x0E  /* imax  bits[4:0] */
#define REG_CALIB_V2I_H          0x0F
#define REG_CALIB_V2I_L          0x10
#define REG_TOP_CFG1             0x13
#define REG_TOP_CFG4             0x16
#define REG_TOP_INT_CFG1         0x17
#define REG_TOP_CTL1             0x22
#define REG_TOP_CTL2             0x23
#define REG_SEQ_CTL2             0x28
#define REG_GPI_0_CTL            0x29
#define REG_GPI_1_CTL            0x2A
#define REG_GPI_2_CTL            0x2B

/* TOP_CFG1 bit layout. */
#define CFG1_BEMF_SENSE_EN   (1U << 6)
#define CFG1_ACTUATOR_LRA    (0U << 5)
#define CFG1_BEMF_FAULT_LIM  (1U << 4)
#define CFG1_FREQ_TRACK_EN   (1U << 3)
#define CFG1_ACCELERATION_EN (1U << 2)
#define CFG1_RAPID_STOP_EN   (1U << 1)
#define CFG1_AMP_PID_EN      (1U << 0)

/* TOP_CTL1 operation modes (bits[2:0]). */
#define MODE_INACTIVE        0x00
#define MODE_DRO             0x01

/* DA7280 LRA period encoding: T_resolution = 1.333 µs.
 *   period_lsb = 1 / (f_hz * 1333.32e-9 s)
 *   FRQ_LRA_PER_H[7:0] = period_lsb >> 7
 *   FRQ_LRA_PER_L[7:1] = period_lsb & 0x7F  (bit[0] reserved) */
#define LRA_PERIOD_RES_NS    1333.32f

const struct hapnav_da7280_cfg HAPNAV_DA7280_CFG_DEFAULT = {
	.resonant_freq_hz = 170.0f,
	/*
	 * Match SparkFun's `defaultMotor()` working values rather than the
	 * G1040003D rated maximum (2.5 Vrms). Driving at 2.5 Vrms steady-
	 * state with acceleration on means there's no headroom for the
	 * acceleration overshoot, and on bus-powered wristbands the
	 * combined current of 4 LRAs near rated voltage is enough to
	 * brown out the supply during bring-up. 2.106 / 2.26 is what the
	 * SparkFun Arduino library has been shipping with for years.
	 */
	.nom_voltage_vrms = 2.106f,
	.abs_voltage_vrms = 2.26f,
	.impedance_ohm    = 13.8f,
	.imax_ma          = 165.4f,  /* matches SparkFun default */
	.acceleration_en  = true,
	.rapid_stop_en    = true,
	.freq_track_en    = false,
	.bemf_sense_en    = false,
};

static int reg_write(struct hapnav_da7280 *dev, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return i2c_write(dev->i2c_bus, buf, sizeof(buf), dev->addr_7bit);
}

static int reg_read(struct hapnav_da7280 *dev, uint8_t reg, uint8_t *val)
{
	return i2c_write_read(dev->i2c_bus, dev->addr_7bit, &reg, 1, val, 1);
}

static int reg_update(struct hapnav_da7280 *dev, uint8_t reg,
		      uint8_t mask, uint8_t bits)
{
	uint8_t cur;
	int err = reg_read(dev, reg, &cur);
	if (err) return err;
	cur = (uint8_t)((cur & ~mask) | (bits & mask));
	return reg_write(dev, reg, cur);
}

static int program_lra_period(struct hapnav_da7280 *dev, float freq_hz)
{
	float    period_ns = 1.0e9f / freq_hz;
	uint16_t lsb = (uint16_t)(period_ns / LRA_PERIOD_RES_NS + 0.5f);
	uint8_t  hi  = (uint8_t)(lsb >> 7);
	uint8_t  lo  = (uint8_t)((lsb & 0x7F) << 1);  /* bit[0] reserved */
	int err = reg_write(dev, REG_FRQ_LRA_PER_H, hi);
	if (err) return err;
	return reg_write(dev, REG_FRQ_LRA_PER_L, lo);
}

static int program_voltages(struct hapnav_da7280 *dev,
			    float v_nom_vrms, float v_abs_vrms)
{
	/* DA7280: 23.4 mV per LSB. */
	uint8_t nom = (uint8_t)((v_nom_vrms / 0.0234f) + 0.5f);
	uint8_t abs_ = (uint8_t)((v_abs_vrms / 0.0234f) + 0.5f);
	int err = reg_write(dev, REG_ACTUATOR1, nom);
	if (err) return err;
	return reg_write(dev, REG_ACTUATOR2, abs_);
}

static int program_imax_and_impedance(struct hapnav_da7280 *dev,
				      float imax_ma, float impedance_ohm)
{
	/* IMAX bits[4:0]: imax_code = (imax_ma - 28.6) / 7.2 */
	int code = (int)(((imax_ma - 28.6f) / 7.2f) + 0.5f);
	if (code < 0)  code = 0;
	if (code > 31) code = 31;
	int err = reg_update(dev, REG_ACTUATOR3, 0x1F, (uint8_t)code);
	if (err) return err;

	/* v2i = impedance * (imax_code + 4) / 1.6104   (datasheet calibration). */
	uint16_t v2i = (uint16_t)((impedance_ohm * (code + 4)) / 1.6104f + 0.5f);
	err = reg_write(dev, REG_CALIB_V2I_L, (uint8_t)(v2i & 0xFF));
	if (err) return err;
	return reg_write(dev, REG_CALIB_V2I_H, (uint8_t)(v2i >> 8));
}

int hapnav_da7280_init(struct hapnav_da7280 *dev,
		       const struct device *i2c_bus, uint8_t addr_7bit,
		       const struct hapnav_da7280_cfg *cfg)
{
	if (!device_is_ready(i2c_bus)) {
		return -ENODEV;
	}
	dev->i2c_bus   = i2c_bus;
	dev->addr_7bit = addr_7bit;
	dev->active    = false;
	if (!cfg) cfg = &HAPNAV_DA7280_CFG_DEFAULT;

	k_msleep(2); /* boot/I2C-ready */

	uint8_t whoami = 0;
	int err = reg_read(dev, REG_CHIP_REV, &whoami);
	if (err) {
		LOG_ERR("WHO_AM_I read failed at 0x%02x: %d", addr_7bit, err);
		return err;
	}
	if (whoami != DA7280_CHIP_REV_VALUE) {
		LOG_ERR("WHO_AM_I mismatch at 0x%02x: 0x%02x", addr_7bit, whoami);
		return -ENODEV;
	}

	/* Stop drive while we reconfigure. */
	err = reg_update(dev, REG_TOP_CTL1, 0x07, MODE_INACTIVE);
	if (err) return err;

	err = program_lra_period(dev, cfg->resonant_freq_hz);
	if (err) return err;
	err = program_voltages(dev, cfg->nom_voltage_vrms, cfg->abs_voltage_vrms);
	if (err) return err;
	err = program_imax_and_impedance(dev, cfg->imax_ma, cfg->impedance_ohm);
	if (err) return err;

	/* TOP_CFG1: LRA, no BEMF sense, freq-track per cfg, accel/rapid-stop on. */
	uint8_t cfg1 = CFG1_ACTUATOR_LRA;
	if (cfg->bemf_sense_en)   cfg1 |= CFG1_BEMF_SENSE_EN;
	if (cfg->freq_track_en)   cfg1 |= CFG1_FREQ_TRACK_EN;
	if (cfg->acceleration_en) cfg1 |= CFG1_ACCELERATION_EN;
	if (cfg->rapid_stop_en)   cfg1 |= CFG1_RAPID_STOP_EN;
	err = reg_write(dev, REG_TOP_CFG1, cfg1);
	if (err) return err;

	/* Disable GPI inputs. The DA7280 has three GPI pins which, if
	 * configured non-zero, can trigger waveform-memory sequence
	 * playback on any electrical edge. The SparkFun Qwiic board
	 * exposes the GPI pads but doesn't pull them; floating pins +
	 * radiated BLE noise can spuriously start sequences that drive
	 * the LRA from internal memory and IGNORE TOP_CTL2. Writing 0 to
	 * each GPI_*_CTL puts them in a benign single-trigger mode with
	 * sequence ID 0 and active-low polarity, so noise can't fire
	 * anything. The Linux driver does the equivalent in its init. */
	(void)reg_write(dev, REG_GPI_0_CTL, 0x00);
	(void)reg_write(dev, REG_GPI_1_CTL, 0x00);
	(void)reg_write(dev, REG_GPI_2_CTL, 0x00);

	/* Clear any sequence selection so even if SEQ_START or RTWM mode
	 * ever toggled, the chip wouldn't have a real sequence to play. */
	(void)reg_write(dev, REG_SEQ_CTL2, 0x00);

	/* Mask all IRQs (no IRQ pin wired); clear all three event banks
	 * — main, warning/diag, and sequence/diag. Pending events from a
	 * previous boot or a transient fault can otherwise leave the chip
	 * in a state where it ignores normal mode/amplitude writes. */
	(void)reg_write(dev, REG_IRQ_MASK1,           0xFF);
	(void)reg_write(dev, REG_IRQ_EVENT1,          0xFF);
	(void)reg_write(dev, REG_IRQ_EVENT_WARN_DIAG, 0xFF);
	(void)reg_write(dev, REG_IRQ_EVENT_SEQ_DIAG,  0xFF);

	/* Enter DRO mode at amplitude 0 — drive is ready, motor is silent. */
	err = reg_write(dev, REG_TOP_CTL2, 0x00);
	if (err) return err;
	err = reg_update(dev, REG_TOP_CTL1, 0x07, MODE_DRO);
	if (err) return err;

	/* Read back key registers and log them. If something is still off
	 * we want the actual chip state on the wire, not what we hoped to
	 * write. */
	uint8_t ctl1 = 0, cfg1_rb = 0;
	(void)reg_read(dev, REG_TOP_CTL1, &ctl1);
	(void)reg_read(dev, REG_TOP_CFG1, &cfg1_rb);

	dev->active = true;
	LOG_INF("DA7280 ready at 0x%02x (LRA, %.1f Hz, CTL1=0x%02x CFG1=0x%02x)",
		addr_7bit, (double)cfg->resonant_freq_hz, ctl1, cfg1_rb);
	return 0;
}

int hapnav_da7280_set_amplitude(struct hapnav_da7280 *dev, uint8_t amplitude)
{
	if (!dev->active) {
		return -ENODEV;
	}
	if (amplitude > DA7280_AMP_MAX) {
		amplitude = DA7280_AMP_MAX;
	}
	return reg_write(dev, REG_TOP_CTL2, amplitude);
}

int hapnav_da7280_stop(struct hapnav_da7280 *dev)
{
	/* Order matters: zero amplitude first so the chip latches a benign
	 * value before mode bits change, then drop OPERATION_MODE to
	 * INACTIVE. The Linux driver does these together in
	 * `da7280_haptic_deactivate()` and the chip won't reliably halt
	 * the LRA from amplitude alone — the mode change is what stops
	 * the drive stage. */
	(void)reg_write(dev, REG_TOP_CTL2, 0x00);
	int err = reg_update(dev, REG_TOP_CTL1, 0x07, MODE_INACTIVE);
	if (err == 0) {
		dev->active = false;
	}
	return err;
}

int hapnav_da7280_resume(struct hapnav_da7280 *dev)
{
	int err = reg_write(dev, REG_TOP_CTL2, 0x00);
	if (err) return err;
	err = reg_update(dev, REG_TOP_CTL1, 0x07, MODE_DRO);
	if (err) return err;
	dev->active = true;
	return 0;
}

int hapnav_da7280_clear_irq(struct hapnav_da7280 *dev, uint8_t *events_out)
{
	uint8_t ev = 0;
	int err = reg_read(dev, REG_IRQ_EVENT1, &ev);
	if (err) return err;
	if (events_out) *events_out = ev;
	if (ev) {
		(void)reg_write(dev, REG_IRQ_EVENT1, ev);
	}
	return 0;
}
