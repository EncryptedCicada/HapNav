/*
 * Copyright (c) 2026 (Your Company)
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the Renesas DA7280 haptic driver.
 *
 * This driver targets DRO mode (I2C direct amplitude write to TOP_CTL2).
 * PWM, RTWM, and ETWM modes are not implemented.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_HAPTIC_DA7280_H_
#define ZEPHYR_INCLUDE_DRIVERS_HAPTIC_DA7280_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fault flags reported through the IRQ callback.
 *
 * These map onto the DA7280 IRQ_EVENT1 register bits, with two extra
 * synthetic flags surfaced from the diagnostic registers.
 */
enum da7280_fault {
	DA7280_FAULT_NONE             = 0,
	DA7280_FAULT_SEQ_CONTINUE     = BIT(0),
	DA7280_FAULT_UVLO             = BIT(1), /**< Under-voltage lockout (brownout) */
	DA7280_FAULT_SEQ_DONE         = BIT(2),
	DA7280_FAULT_OVERTEMP_CRIT    = BIT(3),
	DA7280_FAULT_SEQ_FAULT        = BIT(4),
	DA7280_FAULT_WARNING          = BIT(5), /**< See da7280_get_warnings() */
	DA7280_FAULT_ACTUATOR         = BIT(6), /**< BEMF / actuator fault */
	DA7280_FAULT_OVERCURRENT      = BIT(7),
};

/**
 * @brief Warning flags from IRQ_EVENT_WARNING_DIAG (raised when
 *        DA7280_FAULT_WARNING is set).
 */
enum da7280_warning {
	DA7280_WARN_NONE              = 0,
	DA7280_WARN_OVERTEMP          = BIT(3),
	DA7280_WARN_MEM_TYPE          = BIT(4),
	DA7280_WARN_LIM_DRIVE_ACC     = BIT(6),
	DA7280_WARN_LIM_DRIVE         = BIT(7),
};

/**
 * @brief IRQ callback signature.
 *
 * Invoked from a system workqueue context (NOT ISR) after the driver has
 * already cleared the interrupt and, for serious faults, forced the channel
 * back to INACTIVE.
 *
 * @param dev      DA7280 device that raised the interrupt.
 * @param faults   Bitmask of @ref da7280_fault flags.
 * @param user_data Opaque pointer registered with da7280_set_irq_callback().
 */
typedef void (*da7280_irq_callback_t)(const struct device *dev,
				      uint8_t faults, void *user_data);

/**
 * @brief Operation modes (TOP_CTL1.OPERATION_MODE values, datasheet 5.2.2).
 */
enum da7280_op_mode {
	DA7280_OP_INACTIVE  = 0, /**< Chip in IDLE/STANDBY, no drive. */
	DA7280_OP_DRO       = 1, /**< Direct Register Override (I2C amplitude). */
	DA7280_OP_PWM       = 2, /**< PWM input on GPI_0/PWM pin. */
	DA7280_OP_RTWM      = 3, /**< Register-Triggered Waveform Memory. */
	DA7280_OP_ETWM      = 4, /**< Edge-Triggered Waveform Memory (GPI). */
};

/**
 * @brief GPI single/multi-pattern selection (GPIx_CTL.GPIx_MODE).
 */
enum da7280_gpi_mode {
	DA7280_GPI_MODE_SINGLE = 0, /**< Always plays GPIx_SEQUENCE_ID. */
	DA7280_GPI_MODE_MULTI  = 1, /**< Odd events: ID, even events: ID+1. */
};

/**
 * @brief GPI edge polarity selection (GPIx_CTL.GPIx_POLARITY).
 */
enum da7280_gpi_polarity {
	DA7280_GPI_POL_RISING  = 0,
	DA7280_GPI_POL_FALLING = 1,
	DA7280_GPI_POL_BOTH    = 2,
};

/**
 * @brief Set the drive amplitude (DRO mode).
 *
 * Valid only when the chip is in DA7280_OP_DRO. The driver enters DRO
 * automatically on the first non-zero amplitude after probe; pass 0 to
 * stop and return to INACTIVE.
 *
 * In acceleration mode, @p amplitude is treated as a signed value with
 * magnitude up to 127. With acceleration disabled, the driver writes the
 * unsigned 0-255 form. The value is clamped first to the per-instance
 * `per-channel-drive-cap` from device tree, then rejected if it would push
 * the global drive budget above CONFIG_HAPTIC_DA7280_TOTAL_DRIVE_CAP.
 *
 * @retval 0        On success.
 * @retval -EBUSY   Total drive budget exceeded; nothing was written.
 * @retval -EIO     I2C transaction failed.
 */
int da7280_set_amplitude(const struct device *dev, int amplitude);

/** Stop drive immediately (equivalent to da7280_set_amplitude(0)). */
int da7280_stop(const struct device *dev);

/** Hard stop suitable for fault handlers (no locking, releases budget). */
int da7280_emergency_stop(const struct device *dev);

/** Read the latest IRQ_STATUS1 byte. */
int da7280_get_status(const struct device *dev, uint8_t *status);

/** Read and clear pending events from IRQ_EVENT1. */
int da7280_get_and_clear_events(const struct device *dev, uint8_t *events);

/** Read warning diagnostic byte (only meaningful when WARNING flag set). */
int da7280_get_warnings(const struct device *dev, uint8_t *warnings);

/** Register a fault callback (NULL to deregister). */
int da7280_set_irq_callback(const struct device *dev,
			    da7280_irq_callback_t cb,
			    void *user_data);

/** Read CHIP_REV (whoami = 0xBA on DA7280). */
int da7280_read_chip_rev(const struct device *dev, uint8_t *rev);

/* -- Mode control ------------------------------------------------- */

/**
 * @brief Set the chip's OPERATION_MODE.
 *
 * For DA7280_OP_PWM, you must additionally have a PWM input applied to
 * the GPI_0/PWM pin (the driver does not generate PWM itself), and
 * CONFIG_HAPTIC_DA7280_ALLOW_PWM_MODE must be enabled.
 *
 * For DA7280_OP_RTWM and DA7280_OP_ETWM, configure PS_SEQ_ID/loop and
 * (for ETWM) GPI registers first, then call da7280_seq_start() to fire.
 */
int da7280_set_mode(const struct device *dev, uint8_t mode);
int da7280_get_mode(const struct device *dev, uint8_t *mode);

/* -- Sequence trigger (RTWM / ETWM) ------------------------------- */

/** Set PS_SEQ_ID and PS_SEQ_LOOP (each 0..15). */
int da7280_set_sequence(const struct device *dev,
			uint8_t seq_id, uint8_t loops);

/** Trigger sequence playback (sets TOP_CTL1.SEQ_START). */
int da7280_seq_start(const struct device *dev);

/** Stop sequence playback (clears TOP_CTL1.SEQ_START). */
int da7280_seq_stop(const struct device *dev);

/** Request back-to-back playback of the next sequence. */
int da7280_seq_continue(const struct device *dev);

/* -- GPI configuration (ETWM) ------------------------------------- */

/**
 * @brief Configure one GPI pin's trigger.
 *
 * @param gpi_idx 0=GPI_0/PWM, 1=GPI_1, 2=GPI_2.
 * @param seq_id   Waveform memory sequence ID (0..15).
 * @param mode     enum da7280_gpi_mode value.
 * @param polarity enum da7280_gpi_polarity value.
 */
int da7280_set_gpi(const struct device *dev, unsigned int gpi_idx,
		   uint8_t seq_id, uint8_t mode, uint8_t polarity);

/* -- Waveform memory ---------------------------------------------- */

/**
 * @brief Upload a waveform memory image to the chip.
 *
 * The image is up to 100 bytes formatted per datasheet section 5.8 (the
 * driver does not validate the format). The chip must not be actively
 * driving when this is called; SparkFun's tools and Renesas SmartCanvas
 * GUI both produce compatible images.
 *
 * @retval -EBUSY   chip currently driving
 * @retval -EINVAL  zero length or oversize
 */
int da7280_mem_write(const struct device *dev,
		     const uint8_t *image, size_t len);

/* -- Wideband / custom waveform ----------------------------------- */

/** Update the LRA period registers for off-resonance / wideband drive. */
int da7280_set_lra_freq_hz(const struct device *dev, uint32_t hz);

/** Read the chip's measured LRA period and convert to Hz. */
int da7280_get_lra_period_hz(const struct device *dev, uint32_t *hz);

/**
 * @brief Configure the custom-wave generator coefficients.
 *
 * @param c1,c2,c3 SWG coefficients (datasheet table 14; defaults form a
 *                 sine wave at 0x61, 0xB4, 0xEC).
 * @param enable   if true, sets WAVEGEN_MODE so memory playback uses the
 *                 custom wave instead of step/ramp.
 */
int da7280_set_custom_wave(const struct device *dev,
			   uint8_t c1, uint8_t c2, uint8_t c3, bool enable);

/* -- Diagnostics --------------------------------------------------- */

/** Read CALIB_IMP_H/L and convert to milliohms (datasheet eq. 12). */
int da7280_get_impedance_milliohm(const struct device *dev, uint32_t *milliohm);

/** Read ADC_VDD and convert to millivolts (datasheet eq. 13). */
int da7280_get_vdd_mv(const struct device *dev, uint32_t *mv);

/** Read POLARITY register (toggles every LRA half-period during drive). */
int da7280_get_polarity(const struct device *dev, bool *polarity);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_HAPTIC_DA7280_H_ */
