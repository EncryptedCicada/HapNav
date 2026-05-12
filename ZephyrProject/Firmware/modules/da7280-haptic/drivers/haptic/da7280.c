/*
 * Copyright (c) 2026 (Your Company)
 * SPDX-License-Identifier: Apache-2.0
 *
 * Renesas (formerly Dialog) DA7280 haptic driver for Zephyr.
 *
 * Targets DRO (Direct Register Override) mode: amplitude is written directly
 * to TOP_CTL2 over I2C. Acceleration mode (TOP_CFG1.ACCELERATION_EN) selects
 * the encoding of TOP_CTL2:
 *   - acceleration on : 7-bit signed, magnitude up to 127
 *   - acceleration off: 8-bit unsigned, up to 255
 *
 * PWM, RTWM, and ETWM modes are not implemented in this driver.
 */

#define DT_DRV_COMPAT dlg_da7280

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/haptic/da7280.h>

LOG_MODULE_REGISTER(da7280, CONFIG_HAPTIC_DA7280_LOG_LEVEL);

/* ===================================================================
 * Register map (subset used here). Bit positions match the datasheet.
 * =================================================================== */

#define DA7280_CHIP_REV				0x00
#define DA7280_CHIP_REV_VAL			0xBA

#define DA7280_IRQ_EVENT1			0x03
#define DA7280_IRQ_EVENT_WARNING_DIAG		0x04
#define DA7280_IRQ_EVENT_SEQ_DIAG		0x05
#define DA7280_IRQ_STATUS1			0x06
#define DA7280_IRQ_MASK1			0x07

#define DA7280_FRQ_LRA_PER_H			0x0A
#define DA7280_FRQ_LRA_PER_L			0x0B
#define DA7280_ACTUATOR1			0x0C
#define DA7280_ACTUATOR2			0x0D
#define DA7280_ACTUATOR3			0x0E
#define DA7280_CALIB_V2I_H			0x0F
#define DA7280_CALIB_V2I_L			0x10
#define DA7280_CALIB_IMP_H			0x11
#define DA7280_CALIB_IMP_L			0x12

#define DA7280_TOP_CFG1				0x13
#define DA7280_TOP_CFG2				0x14
#define DA7280_TOP_CFG4				0x16
#define DA7280_TOP_INT_CFG1			0x17

#define DA7280_TOP_CTL1				0x22
#define DA7280_TOP_CTL2				0x23

#define DA7280_SEQ_CTL1				0x24
#define DA7280_SWG_C1				0x25
#define DA7280_SWG_C2				0x26
#define DA7280_SWG_C3				0x27
#define DA7280_SEQ_CTL2				0x28
#define DA7280_GPI_0_CTL			0x29
#define DA7280_GPI_1_CTL			0x2A
#define DA7280_GPI_2_CTL			0x2B
#define DA7280_MEM_CTL1				0x2C
#define DA7280_MEM_CTL2				0x2D
#define DA7280_ADC_DATA_H1			0x2E
#define DA7280_ADC_DATA_L1			0x2F

#define DA7280_POLARITY_REG			0x43
#define DA7280_LRA_AVR_H			0x44
#define DA7280_LRA_AVR_L			0x45
#define DA7280_FRQ_LRA_PER_ACT_H		0x46
#define DA7280_FRQ_LRA_PER_ACT_L		0x47
#define DA7280_FRQ_PHASE_H			0x48
#define DA7280_FRQ_PHASE_L			0x49
#define DA7280_FRQ_CTL				0x4C
#define DA7280_TRIM3				0x5F
#define DA7280_TRIM4				0x60
#define DA7280_TRIM6				0x62

#define DA7280_TOP_CFG3				0x15

#define DA7280_TOP_CFG5				0x6E

#define DA7280_IRQ_MASK2			0x83

#define DA7280_SNP_MEM_BASE_DEFAULT		0x84  /* WAV_MEM_BASE_ADDR POR */
#define DA7280_SNP_MEM_END			0xE7
#define DA7280_SNP_MEM_SIZE			100

/* TOP_CFG1 bits */
#define DA7280_AMP_PID_EN			BIT(0)
#define DA7280_RAPID_STOP_EN			BIT(1)
#define DA7280_ACCELERATION_EN			BIT(2)
#define DA7280_FREQ_TRACK_EN			BIT(3)
#define DA7280_BEMF_SENSE_EN			BIT(4)
#define DA7280_ACTUATOR_TYPE_BIT		BIT(5)

/* TOP_CFG2 bits */
#define DA7280_FULL_BRAKE_THR_MASK		GENMASK(3, 0)
#define DA7280_MEM_DATA_SIGNED			BIT(4)

/* TOP_CFG4 bits */
#define DA7280_TST_CALIB_IMPEDANCE_DIS		BIT(6)
#define DA7280_V2I_FACTOR_FREEZE		BIT(7)

/* TOP_INT_CFG1 bits */
#define DA7280_BEMF_FAULT_LIM_MASK		GENMASK(1, 0)

/* TOP_CTL1 bits */
#define DA7280_OPERATION_MODE_MASK		GENMASK(2, 0)
#define DA7280_STANDBY_EN			BIT(3)
#define DA7280_SEQ_START			BIT(4)

/* TOP_CFG5 bits */
#define DA7280_V2I_FACTOR_OFFSET_EN		BIT(0)
#define DA7280_FRQ_PAUSE_ON_POLARITY_CHANGE	BIT(1)
#define DA7280_DELAY_BYPASS			BIT(2)

/* SEQ_CTL1 bits */
#define DA7280_SEQ_CONTINUE			BIT(0)
#define DA7280_WAVEGEN_MODE			BIT(1)
#define DA7280_FREQ_WAVEFORM_TIMEBASE		BIT(2)

/* SEQ_CTL2 bits */
#define DA7280_PS_SEQ_ID_MASK			GENMASK(3, 0)
#define DA7280_PS_SEQ_LOOP_MASK			GENMASK(7, 4)

/* GPI_x_CTL bits (same layout for x = 0,1,2) */
#define DA7280_GPI_POLARITY_MASK		GENMASK(1, 0)
#define DA7280_GPI_MODE_BIT			BIT(2)
#define DA7280_GPI_SEQ_ID_MASK			GENMASK(6, 3)

/* MEM_CTL2 bits */
#define DA7280_WAV_MEM_LOCK			BIT(7)

/* TOP_CFG3 bits */
#define DA7280_VDD_MARGIN_MASK			GENMASK(3, 0)

/* TOP_CFG1 extra: EMBEDDED_MODE */
#define DA7280_EMBEDDED_MODE			BIT(7)

/* FRQ_CTL bits */
#define DA7280_FREQ_TRACKING_FORCE_ON		BIT(0)
#define DA7280_FREQ_TRACKING_AUTO_ADJ		BIT(1)

/* TRIM3 bits */
#define DA7280_REF_UVLO_THRES_MASK		GENMASK(4, 3)
#define DA7280_LOOP_FILT_LOW_BW			BIT(5)
#define DA7280_LOOP_IDAC_DOUBLE_RANGE		BIT(6)

/* FRQ_PHASE_L bits */
#define DA7280_DELAY_SHIFT_L_MASK		GENMASK(2, 0)
#define DA7280_DELAY_FREEZE			BIT(7)

/* IRQ_MASK1 bits (for selectively masking events) */
#define DA7280_SEQ_DONE_M			BIT(2)
#define DA7280_SEQ_FAULT_M			BIT(4)

/* ACTUATOR3 */
#define DA7280_IMAX_MASK			GENMASK(4, 0)

/* IRQ_MASK2 */
#define DA7280_ADC_SAT_M			BIT(7)

/* Operation modes */
#define DA7280_INACTIVE				0
#define DA7280_DRO_MODE				1
#define DA7280_PWM_MODE				2
#define DA7280_RTWM_MODE			3
#define DA7280_ETWM_MODE			4

/* Actuator types (driver-internal). The chip's TOP_CFG1.ACTUATOR_TYPE bit
 * only distinguishes LRA from ERM; we further track ERM-coin separately
 * because it needs feature flags forced off (no BEMF sensing possible). */
#define DA7280_ACTUATOR_LRA			0
#define DA7280_ACTUATOR_ERM_BAR			1
#define DA7280_ACTUATOR_ERM_COIN		2

/* Encoding constants */
#define DA7280_VOLT_STEP_UV			23400U
#define DA7280_IMAX_STEP_UA			7200U
#define DA7280_IMAX_OFFSET_UA			28600U
#define DA7280_V2I_DENOM			1610400U
#define DA7280_LRA_CLK_HZ			750000U  /* 750 kHz internal */

/* TOP_CTL2 amplitude limits */
#define DA7280_AMP_MAX_ACC			127
#define DA7280_AMP_MAX_NOACC			255

/* Bit value used in raw fault mask for "any serious fault that should
 * force inactive" - covers UVLO, overtemp, actuator, and OC. SEQ_FAULT
 * is also forced inactive (per Linux driver). */
#define DA7280_FORCE_INACTIVE_MASK					\
	(DA7280_FAULT_UVLO | DA7280_FAULT_OVERTEMP_CRIT |		\
	 DA7280_FAULT_SEQ_FAULT | DA7280_FAULT_ACTUATOR |		\
	 DA7280_FAULT_OVERCURRENT)

/* ===================================================================
 * Per-instance config (const, from device tree) and runtime state.
 * =================================================================== */

struct da7280_gpi_dt {
	uint8_t seq_id;        /* 0..15 */
	uint8_t mode;          /* 0 single-pattern, 1 multi-pattern */
	uint8_t polarity;      /* 0 rising, 1 falling, 2 both */
};

struct da7280_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec irq_gpio;
	bool has_irq;

	uint32_t nom_uv;
	uint32_t abs_uv;
	uint32_t imax_ua;
	uint32_t impd_uohm;
	uint32_t resonant_hz;
	uint8_t  actuator_type;       /* DA7280_ACTUATOR_LRA / ERM_BAR / ERM_COIN */
	uint8_t  per_channel_cap;
	uint8_t  full_brake_thr;
	uint8_t  bemf_fault_lim;
	uint8_t  vdd_margin;
	uint8_t  ps_seq_id;
	uint8_t  ps_seq_loop;
	bool acceleration_en;
	bool rapid_stop_en;
	bool freq_track_en;
	bool bemf_sense_en;
	bool amp_pid_en;
	bool embedded_mode;
	bool standby_after_drive;

	struct da7280_gpi_dt gpi[3];

	/* Optional waveform memory image to upload at probe */
	const uint8_t *mem_image;
	size_t mem_image_len;
};

struct da7280_data {
	const struct device *self;
	struct k_mutex lock;            /* per-instance config mutex */
	struct gpio_callback irq_cb;
	struct k_work irq_work;
	da7280_irq_callback_t user_cb;
	void *user_data;
	int16_t last_amp_signed;        /* last DRO value written (with sign) */
	uint8_t last_amp_abs;            /* this instance's contribution to global budget */
	uint8_t mode;                    /* DA7280_INACTIVE / DRO / PWM / RTWM / ETWM */
	bool active;                     /* device is currently driving */
};

/* ===================================================================
 * Module-level globals: total drive budget and dedicated workqueue.
 * =================================================================== */

K_MUTEX_DEFINE(da7280_budget_lock);
static unsigned int da7280_budget_used; /* sum of |last_amp_abs| */

#if !defined(CONFIG_HAPTIC_DA7280_USE_SYSTEM_WORKQUEUE)
K_THREAD_STACK_DEFINE(da7280_wq_stack,
		      CONFIG_HAPTIC_DA7280_WORKQUEUE_STACK_SIZE);
static struct k_work_q da7280_wq;
static bool da7280_wq_started;
#endif

static inline void da7280_submit_work(struct k_work *work)
{
#if defined(CONFIG_HAPTIC_DA7280_USE_SYSTEM_WORKQUEUE)
	k_work_submit(work);
#else
	k_work_submit_to_queue(&da7280_wq, work);
#endif
}

/* Forward declaration: defined in init section but called from
 * da7280_mem_write() in the public API section. */
static int da7280_mem_upload(const struct device *dev,
			     const uint8_t *image, size_t len);

/* ===================================================================
 * Low-level register helpers.
 * =================================================================== */

static int da7280_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct da7280_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

static int da7280_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct da7280_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int da7280_update_bits(const struct device *dev, uint8_t reg,
			      uint8_t mask, uint8_t val)
{
	const struct da7280_config *cfg = dev->config;

	return i2c_reg_update_byte_dt(&cfg->i2c, reg, mask, val & mask);
}

/* ===================================================================
 * Encoding helpers (integer math, no floats).
 * =================================================================== */

static uint8_t da7280_uv_to_code(uint32_t uv)
{
	/*
	 * Linux encoding: code = uV / 23400 + 1, capped at 0xFF.
	 * The "+1" is a deliberate round-up so the chip never undershoots
	 * the requested voltage; for uV = 0 it produces code = 1 which
	 * matches the kernel driver's behavior. Caller is responsible for
	 * passing a sane value (binding range-checks the DT property).
	 */
	uint32_t code = uv / DA7280_VOLT_STEP_UV + 1U;

	return (uint8_t)MIN(code, 0xFFU);
}

static uint8_t da7280_imax_to_code(uint32_t ua)
{
	/*
	 * Linux encoding: code = (uA - 28600) / 7200 + 1, capped at 0x1F.
	 * Mirrors the voltage rule: round-up so the headroom errs high.
	 */
	uint32_t code;

	if (ua < DA7280_IMAX_OFFSET_UA) {
		return 0;
	}
	code = (ua - DA7280_IMAX_OFFSET_UA) / DA7280_IMAX_STEP_UA + 1U;
	return (uint8_t)MIN(code, 0x1FU);
}

static uint16_t da7280_v2i_factor(uint32_t impd_uohm, uint8_t imax_code)
{
	/*
	 * v2i_factor = impedance(Ohm) * (imax_code + 4) / 1.6104
	 * In micro-ohm units this is impd_uohm * (code+4) / 1610400.
	 * Promote to 64-bit for the multiply: 1.5e9 * 35 fits in u64
	 * trivially, and keeps the result well under 0xFFFF.
	 */
	uint64_t num = (uint64_t)impd_uohm * (imax_code + 4U);

	return (uint16_t)(num / DA7280_V2I_DENOM);
}

static void da7280_freq_to_period(uint32_t freq_hz,
				  uint8_t *hi, uint8_t *lo)
{
	/*
	 * Linux encoding: count = 1000000000 / (freq_hz * 1333).
	 *   hi = (count >> 7) & 0xFF
	 *   lo = count & 0x7F           (FRQ_LRA_PER_L bit 7 is reserved)
	 *
	 * 1333 ns is the chip's nominal reference period (~750.18 kHz).
	 * Linux clamps freq_hz to the [50, 300] Hz spec range; outside
	 * that, it falls back to the chip default. We mirror that.
	 */
	uint32_t cnt;

	if (freq_hz < 50U || freq_hz > 300U) {
		*hi = 0x39U;  /* DA7280_RESONT_FREQH_DFT */
		*lo = 0x32U;  /* DA7280_RESONT_FREQL_DFT */
		return;
	}

	cnt = 1000000000UL / (freq_hz * 1333U);
	*hi = (uint8_t)((cnt >> 7) & 0xFFU);
	*lo = (uint8_t)(cnt & 0x7FU);
}

/* ===================================================================
 * Drive-budget bookkeeping.
 *
 * Each instance contributes |last_amp_abs| to a global running sum.
 * Reservation logic is a single-mutex critical section because the cap
 * check and the running-sum update need to be atomic together.
 * =================================================================== */

static int da7280_budget_reserve(struct da7280_data *data, uint8_t new_abs)
{
	const unsigned int cap = CONFIG_HAPTIC_DA7280_TOTAL_DRIVE_CAP;
	int ret = 0;

	if (cap == 0U) {
		data->last_amp_abs = new_abs;
		return 0;
	}

	k_mutex_lock(&da7280_budget_lock, K_FOREVER);
	{
		unsigned int prospective = da7280_budget_used -
					   data->last_amp_abs + new_abs;

		if (prospective > cap) {
			LOG_WRN("budget: req=%u+%u-%u=%u > cap=%u",
				da7280_budget_used, new_abs,
				data->last_amp_abs, prospective, cap);
			ret = -EBUSY;
		} else {
			da7280_budget_used = prospective;
			data->last_amp_abs = new_abs;
		}
	}
	k_mutex_unlock(&da7280_budget_lock);

	return ret;
}

static void da7280_budget_release(struct da7280_data *data)
{
	k_mutex_lock(&da7280_budget_lock, K_FOREVER);
	{
		if (da7280_budget_used >= data->last_amp_abs) {
			da7280_budget_used -= data->last_amp_abs;
		} else {
			/* shouldn't happen, but stay safe */
			da7280_budget_used = 0;
		}
		data->last_amp_abs = 0;
	}
	k_mutex_unlock(&da7280_budget_lock);
}

/* ===================================================================
 * Activation / deactivation primitives.
 * =================================================================== */

static int da7280_set_op_mode(const struct device *dev, uint8_t mode)
{
	return da7280_update_bits(dev, DA7280_TOP_CTL1,
				  DA7280_OPERATION_MODE_MASK, mode);
}

static int da7280_write_amp_register(const struct device *dev,
				     const struct da7280_config *cfg,
				     int amplitude)
{
	uint8_t reg_val;

	if (cfg->acceleration_en) {
		/* 7-bit signed: clamp to [-127, +127] */
		if (amplitude > DA7280_AMP_MAX_ACC) {
			amplitude = DA7280_AMP_MAX_ACC;
		} else if (amplitude < -DA7280_AMP_MAX_ACC) {
			amplitude = -DA7280_AMP_MAX_ACC;
		}
		reg_val = (uint8_t)((int8_t)amplitude) & 0xFFU;
	} else {
		/* 8-bit unsigned: clamp to [0, 255] */
		if (amplitude < 0) {
			amplitude = 0;
		} else if (amplitude > DA7280_AMP_MAX_NOACC) {
			amplitude = DA7280_AMP_MAX_NOACC;
		}
		reg_val = (uint8_t)amplitude;
	}
	return da7280_write_reg(dev, DA7280_TOP_CTL2, reg_val);
}

/* ===================================================================
 * Public API.
 * =================================================================== */

int da7280_set_amplitude(const struct device *dev, int amplitude)
{
	const struct da7280_config *cfg = dev->config;
	struct da7280_data *data = dev->data;
	int ret;
	int clamp;

	/* Per-channel clamp first, then sign-aware budget check. */
	clamp = (int)cfg->per_channel_cap;
	if (amplitude > clamp) {
		amplitude = clamp;
	} else if (amplitude < -clamp) {
		amplitude = -clamp;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Stop path: write 0, set inactive, release budget. */
	if (amplitude == 0) {
		ret = da7280_write_reg(dev, DA7280_TOP_CTL2, 0);
		if (ret == 0) {
			ret = da7280_set_op_mode(dev, DA7280_INACTIVE);
		}
		data->last_amp_signed = 0;
		da7280_budget_release(data);
		data->active = false;
		k_mutex_unlock(&data->lock);
		return ret;
	}

	/* Reserve budget BEFORE touching hardware. */
	ret = da7280_budget_reserve(data, (uint8_t)abs(amplitude));
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = da7280_write_amp_register(dev, cfg, amplitude);
	if (ret != 0) {
		LOG_ERR("set_amplitude: TOP_CTL2 write failed: %d", ret);
		da7280_budget_release(data);
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	if (!data->active) {
		ret = da7280_set_op_mode(dev, DA7280_DRO_MODE);
		if (ret != 0) {
			LOG_ERR("set_amplitude: enter DRO failed: %d", ret);
			(void)da7280_write_reg(dev, DA7280_TOP_CTL2, 0);
			da7280_budget_release(data);
			k_mutex_unlock(&data->lock);
			return -EIO;
		}
		data->active = true;
	}

	data->last_amp_signed = (int16_t)amplitude;

	k_mutex_unlock(&data->lock);
	return 0;
}

int da7280_stop(const struct device *dev)
{
	return da7280_set_amplitude(dev, 0);
}

int da7280_emergency_stop(const struct device *dev)
{
	struct da7280_data *data = dev->data;
	int ret;

	/* No locking, no retry: best-effort hard stop. Safe from any context
	 * that allows blocking I2C (i.e. NOT a true ISR -- callers from an
	 * ISR should defer to a work item). */
	ret = da7280_write_reg(dev, DA7280_TOP_CTL2, 0);
	(void)da7280_set_op_mode(dev, DA7280_INACTIVE);

	data->last_amp_signed = 0;
	data->active = false;
	da7280_budget_release(data);

	return ret;
}

int da7280_get_status(const struct device *dev, uint8_t *status)
{
	if (status == NULL) {
		return -EINVAL;
	}
	return da7280_read_reg(dev, DA7280_IRQ_STATUS1, status);
}

int da7280_get_and_clear_events(const struct device *dev, uint8_t *events)
{
	uint8_t e;
	int ret;

	if (events == NULL) {
		return -EINVAL;
	}
	ret = da7280_read_reg(dev, DA7280_IRQ_EVENT1, &e);
	if (ret != 0) {
		return ret;
	}
	*events = e;
	if (e == 0U) {
		return 0;
	}
	/* Clear by writing the bits back -- this register is W1C. */
	return da7280_write_reg(dev, DA7280_IRQ_EVENT1, e);
}

int da7280_get_warnings(const struct device *dev, uint8_t *warnings)
{
	if (warnings == NULL) {
		return -EINVAL;
	}
	return da7280_read_reg(dev, DA7280_IRQ_EVENT_WARNING_DIAG, warnings);
}

int da7280_set_irq_callback(const struct device *dev,
			    da7280_irq_callback_t cb, void *user_data)
{
	struct da7280_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->user_cb = cb;
	data->user_data = user_data;
	k_mutex_unlock(&data->lock);
	return 0;
}

int da7280_read_chip_rev(const struct device *dev, uint8_t *rev)
{
	if (rev == NULL) {
		return -EINVAL;
	}
	return da7280_read_reg(dev, DA7280_CHIP_REV, rev);
}

/* -------------------------------------------------------------------
 * Mode control.
 *
 * The chip's OPERATION_MODE bits select between INACTIVE / DRO / PWM /
 * RTWM / ETWM. We additionally maintain `data->mode` so the driver knows
 * what activation logic to apply on the next start.
 *
 * Budget: any non-DRO active mode reserves the per-channel cap (worst
 * case) on activation. DRO continues to reserve dynamically per
 * set_amplitude(). Modes are mutually exclusive per instance.
 * ------------------------------------------------------------------- */

int da7280_set_mode(const struct device *dev, uint8_t mode)
{
	struct da7280_data *data = dev->data;
	int ret;

	if (mode > DA7280_ETWM_MODE) {
		return -EINVAL;
	}
	if (mode == DA7280_PWM_MODE && !IS_ENABLED(CONFIG_HAPTIC_DA7280_ALLOW_PWM_MODE)) {
		/* PWM mode needs an external PWM signal on GPI_0/PWM. The
		 * driver does not provision a PWM peripheral itself; we
		 * gate this behind a Kconfig so users opt in explicitly. */
		LOG_WRN("PWM mode requested but CONFIG_HAPTIC_DA7280_ALLOW_PWM_MODE=n");
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = da7280_set_op_mode(dev, mode);
	if (ret == 0) {
		data->mode = mode;
		if (mode == DA7280_INACTIVE) {
			data->active = false;
			da7280_budget_release(data);
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int da7280_get_mode(const struct device *dev, uint8_t *mode)
{
	uint8_t ctl1;
	int ret;

	if (mode == NULL) {
		return -EINVAL;
	}
	ret = da7280_read_reg(dev, DA7280_TOP_CTL1, &ctl1);
	if (ret != 0) {
		return ret;
	}
	*mode = ctl1 & DA7280_OPERATION_MODE_MASK;
	return 0;
}

/* -------------------------------------------------------------------
 * Sequence trigger (RTWM / ETWM).
 * ------------------------------------------------------------------- */

int da7280_set_sequence(const struct device *dev,
			uint8_t seq_id, uint8_t loops)
{
	uint8_t val;

	if (seq_id > 0x0F || loops > 0x0F) {
		return -EINVAL;
	}
	val = ((loops << 4) & DA7280_PS_SEQ_LOOP_MASK) |
	      (seq_id & DA7280_PS_SEQ_ID_MASK);
	return da7280_write_reg(dev, DA7280_SEQ_CTL2, val);
}

int da7280_seq_start(const struct device *dev)
{
	const struct da7280_config *cfg = dev->config;
	struct da7280_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->mode != DA7280_RTWM_MODE && data->mode != DA7280_ETWM_MODE) {
		k_mutex_unlock(&data->lock);
		return -EINVAL;
	}

	/* Reserve the worst-case amplitude for this channel; a memory
	 * sequence can ramp anywhere up to the per-channel cap. */
	ret = da7280_budget_reserve(data, cfg->per_channel_cap);
	if (ret != 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	ret = da7280_update_bits(dev, DA7280_TOP_CTL1,
				 DA7280_SEQ_START, DA7280_SEQ_START);
	if (ret != 0) {
		da7280_budget_release(data);
	} else {
		data->active = true;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

int da7280_seq_stop(const struct device *dev)
{
	struct da7280_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = da7280_update_bits(dev, DA7280_TOP_CTL1,
				 DA7280_SEQ_START, 0);
	data->active = false;
	da7280_budget_release(data);
	k_mutex_unlock(&data->lock);
	return ret;
}

int da7280_seq_continue(const struct device *dev)
{
	/* SEQ_CONTINUE in SEQ_CTL1 is self-clearing (see datasheet 6.2 /
	 * table 48): writing 1 starts a back-to-back sequence playback. */
	return da7280_update_bits(dev, DA7280_SEQ_CTL1,
				  DA7280_SEQ_CONTINUE, DA7280_SEQ_CONTINUE);
}

/* -------------------------------------------------------------------
 * GPI configuration (for ETWM mode, datasheet 5.2.7).
 *
 * gpi_idx: 0..2 (GPI_0/PWM, GPI_1, GPI_2)
 * polarity: 0 rising, 1 falling, 2 both
 * mode: 0 single-pattern (always plays seq_id), 1 multi-pattern
 *       (odd events play seq_id, even events play seq_id+1)
 * ------------------------------------------------------------------- */

int da7280_set_gpi(const struct device *dev, unsigned int gpi_idx,
		   uint8_t seq_id, uint8_t mode, uint8_t polarity)
{
	uint8_t val;

	if (gpi_idx > 2 || seq_id > 0x0F || mode > 1 || polarity > 2) {
		return -EINVAL;
	}
	val = ((seq_id << 3) & DA7280_GPI_SEQ_ID_MASK) |
	      (mode ? DA7280_GPI_MODE_BIT : 0) |
	      (polarity & DA7280_GPI_POLARITY_MASK);
	return da7280_write_reg(dev, DA7280_GPI_0_CTL + gpi_idx, val);
}

/* -------------------------------------------------------------------
 * Waveform memory upload at runtime.
 *
 * The image is up to 100 bytes laid out per datasheet 5.8: header
 * (snippet count, sequence count, end-pointers) followed by snippet
 * PWL data and sequence frames. The driver does not parse this format;
 * it just streams the bytes into the chip starting at WAV_MEM_BASE_ADDR.
 * ------------------------------------------------------------------- */

int da7280_mem_write(const struct device *dev,
		     const uint8_t *image, size_t len)
{
	struct da7280_data *data = dev->data;
	int ret;

	if (image == NULL || len == 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->active) {
		k_mutex_unlock(&data->lock);
		return -EBUSY;
	}
	ret = da7280_mem_upload(dev, image, len);
	k_mutex_unlock(&data->lock);
	return ret;
}

/* -------------------------------------------------------------------
 * Wideband / custom-waveform helpers.
 *
 * Wideband: drive the LRA off-resonance by writing a new period to
 *           FRQ_LRA_PER_H/L. Caller must have disabled freq tracking,
 *           accel, and rapid-stop in DT (or via the chip already).
 * ------------------------------------------------------------------- */

int da7280_set_lra_freq_hz(const struct device *dev, uint32_t hz)
{
	uint8_t hi, lo;
	int ret;

	da7280_freq_to_period(hz, &hi, &lo);
	ret = da7280_write_reg(dev, DA7280_FRQ_LRA_PER_H, hi);
	if (ret != 0) {
		return ret;
	}
	return da7280_update_bits(dev, DA7280_FRQ_LRA_PER_L, 0x7F, lo);
}

int da7280_get_lra_period_hz(const struct device *dev, uint32_t *hz)
{
	uint8_t hi, lo;
	uint32_t cnt;
	int ret;

	if (hz == NULL) {
		return -EINVAL;
	}
	ret = da7280_read_reg(dev, DA7280_FRQ_LRA_PER_ACT_H, &hi);
	if (ret != 0) {
		return ret;
	}
	ret = da7280_read_reg(dev, DA7280_FRQ_LRA_PER_ACT_L, &lo);
	if (ret != 0) {
		return ret;
	}
	cnt = ((uint32_t)hi << 7) | (lo & 0x7FU);
	if (cnt == 0U) {
		*hz = 0;
		return 0;
	}
	/* Inverse of count = 1e9 / (hz * 1333). */
	*hz = 1000000000UL / (cnt * 1333U);
	return 0;
}

int da7280_set_custom_wave(const struct device *dev,
			   uint8_t c1, uint8_t c2, uint8_t c3, bool enable)
{
	int ret;

	ret = da7280_write_reg(dev, DA7280_SWG_C1, c1);
	if (ret != 0) {
		return ret;
	}
	ret = da7280_write_reg(dev, DA7280_SWG_C2, c2);
	if (ret != 0) {
		return ret;
	}
	ret = da7280_write_reg(dev, DA7280_SWG_C3, c3);
	if (ret != 0) {
		return ret;
	}
	return da7280_update_bits(dev, DA7280_SEQ_CTL1,
				  DA7280_WAVEGEN_MODE,
				  enable ? DA7280_WAVEGEN_MODE : 0);
}

/* -------------------------------------------------------------------
 * Diagnostic readbacks.
 * ------------------------------------------------------------------- */

int da7280_get_impedance_milliohm(const struct device *dev, uint32_t *milliohm)
{
	uint8_t hi, lo;
	int ret;

	if (milliohm == NULL) {
		return -EINVAL;
	}
	ret = da7280_read_reg(dev, DA7280_CALIB_IMP_H, &hi);
	if (ret != 0) {
		return ret;
	}
	ret = da7280_read_reg(dev, DA7280_CALIB_IMP_L, &lo);
	if (ret != 0) {
		return ret;
	}
	/* Datasheet eq 12: Z = (H * 4 + L) * 0.0625 ohm = (H*4 + L) * 62.5 milliohm. */
	*milliohm = ((uint32_t)hi * 4U + (lo & 0x3U)) * 625U / 10U;
	return 0;
}

int da7280_get_vdd_mv(const struct device *dev, uint32_t *mv)
{
	uint8_t hi, lo;
	uint32_t code;
	int ret;

	if (mv == NULL) {
		return -EINVAL;
	}
	ret = da7280_read_reg(dev, DA7280_ADC_DATA_H1, &hi);
	if (ret != 0) {
		return ret;
	}
	ret = da7280_read_reg(dev, DA7280_ADC_DATA_L1, &lo);
	if (ret != 0) {
		return ret;
	}
	/* Datasheet eq 13: VDD = (H * 128 + L) * 0.1831 mV. We use 1831/10000
	 * to keep this in integer arithmetic; result is in millivolts. */
	code = ((uint32_t)hi << 7) | (lo & 0x7FU);
	*mv = code * 1831U / 10000U;
	return 0;
}

int da7280_get_polarity(const struct device *dev, bool *polarity)
{
	uint8_t val;
	int ret;

	if (polarity == NULL) {
		return -EINVAL;
	}
	ret = da7280_read_reg(dev, DA7280_POLARITY_REG, &val);
	if (ret != 0) {
		return ret;
	}
	*polarity = (val & 0x01U) != 0U;
	return 0;
}

/* ===================================================================
 * IRQ work handler.
 *
 * Runs in workqueue context: reads the three IRQ event registers, clears
 * them, and on a serious fault forces this channel to INACTIVE before
 * calling the user callback. This matches the Linux driver behavior so
 * the chip doesn't keep re-asserting the same fault.
 * =================================================================== */

static void da7280_irq_work_handler(struct k_work *work)
{
	struct da7280_data *data =
		CONTAINER_OF(work, struct da7280_data, irq_work);
	const struct device *dev = data->self;
	uint8_t events[3] = {0};
	int ret;

	ret = i2c_burst_read_dt(&((const struct da7280_config *)dev->config)->i2c,
				DA7280_IRQ_EVENT1, events, sizeof(events));
	if (ret != 0) {
		LOG_ERR("IRQ: failed to read events: %d", ret);
		return;
	}

	/* Clear by writing event[0] back to IRQ_EVENT1 (W1C). */
	if (events[0] != 0U) {
		(void)da7280_write_reg(dev, DA7280_IRQ_EVENT1, events[0]);
	}

	if (events[0] & DA7280_FORCE_INACTIVE_MASK) {
		LOG_WRN("forcing INACTIVE on fault 0x%02x (warn=0x%02x seq=0x%02x)",
			events[0], events[1], events[2]);
		(void)da7280_emergency_stop(dev);
	} else if (events[0] & DA7280_FAULT_SEQ_DONE) {
		/* Sequence finished naturally. Mark inactive and free our
		 * share of the global drive budget. The chip auto-returns
		 * to IDLE/STANDBY per STANDBY_EN. */
		k_mutex_lock(&data->lock, K_FOREVER);
		data->active = false;
		da7280_budget_release(data);
		k_mutex_unlock(&data->lock);
	}

	if (events[0] & DA7280_FAULT_WARNING) {
		if (events[1] & DA7280_WARN_LIM_DRIVE) {
			LOG_WRN("limit drive warning - reduce amplitude");
		}
		if (events[1] & DA7280_WARN_LIM_DRIVE_ACC) {
			LOG_WRN("limit drive accel warning");
		}
		if (events[1] & DA7280_WARN_OVERTEMP) {
			LOG_WRN("over-temperature warning");
		}
	}

	if (data->user_cb != NULL) {
		data->user_cb(dev, events[0], data->user_data);
	}
}

static void da7280_gpio_irq(const struct device *port,
			    struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct da7280_data *data = CONTAINER_OF(cb, struct da7280_data, irq_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	da7280_submit_work(&data->irq_work);
}

/* ===================================================================
 * Init.
 *
 * Order mirrors the Linux driver (drivers/input/misc/da7280.c
 * da7280_init): config registers first, STANDBY_EN last, then GPI/IRQ
 * setup. Linux relies on the chip's POR state (OP_MODE=0, TOP_CTL2=0)
 * for safety; if your platform can reset the host while the DA7280
 * keeps power, enable HAPTIC_DA7280_SAFE_INIT to add an explicit
 * defang-prologue (write TOP_CTL2=0, force OP_MODE=INACTIVE) before
 * touching anything else.
 * =================================================================== */

#if defined(CONFIG_HAPTIC_DA7280_SAFE_INIT)
static int da7280_safe_init_prologue(const struct device *dev)
{
	int ret;

	/*
	 * Defensive: write the amplitude register to zero and force the
	 * operation mode to INACTIVE *before* applying any configuration.
	 * This is only needed when the chip can survive a host reset
	 * with non-zero TOP_CTL2 / non-zero OP_MODE. In Linux this is
	 * not required because the chip is power-cycled with the SoC.
	 */
	ret = da7280_write_reg(dev, DA7280_TOP_CTL2, 0);
	if (ret != 0) {
		LOG_ERR("safe_init: TOP_CTL2 zero failed: %d", ret);
		return ret;
	}
	ret = da7280_set_op_mode(dev, DA7280_INACTIVE);
	if (ret != 0) {
		LOG_ERR("safe_init: force INACTIVE failed: %d", ret);
		return ret;
	}
	return 0;
}
#endif /* CONFIG_HAPTIC_DA7280_SAFE_INIT */

/* Helper: upload waveform memory image to the chip.
 *
 * Mirrors Linux's da7280_haptic_mem_update(): the chip must be in INACTIVE
 * mode and the WAV_MEM_LOCK bit must be deasserted (bit cleared = unlocked
 * per datasheet table 57: "0x1 = Unlocked"). The image is written starting
 * at WAV_MEM_BASE_ADDR (default 0x84) up to 0xE7.
 */
static int da7280_mem_upload(const struct device *dev,
			     const uint8_t *image, size_t len)
{
	const struct da7280_config *cfg = dev->config;
	uint8_t base, status, lock;
	int ret;

	if (image == NULL || len == 0U) {
		return 0;
	}
	if (len > DA7280_SNP_MEM_SIZE) {
		LOG_ERR("mem image too large: %zu > %u", len, DA7280_SNP_MEM_SIZE);
		return -EINVAL;
	}

	/* Refuse to upload while a sequence is running. */
	ret = da7280_read_reg(dev, DA7280_IRQ_STATUS1, &status);
	if (ret != 0) {
		return ret;
	}
	if (status & DA7280_FAULT_WARNING) {
		LOG_WRN("mem upload: chip warning bit set, refusing");
		return -EBUSY;
	}

	/* WAV_MEM_LOCK = 1 means *unlocked* per datasheet table 57. */
	ret = da7280_read_reg(dev, DA7280_MEM_CTL2, &lock);
	if (ret != 0) {
		return ret;
	}
	if ((lock & DA7280_WAV_MEM_LOCK) == 0U) {
		LOG_WRN("mem upload: WAV_MEM_LOCK is asserted (locked); unlocking");
		ret = da7280_update_bits(dev, DA7280_MEM_CTL2,
					 DA7280_WAV_MEM_LOCK, DA7280_WAV_MEM_LOCK);
		if (ret != 0) {
			return ret;
		}
	}

	/* Force INACTIVE before writing memory (datasheet 5.6.4). */
	ret = da7280_set_op_mode(dev, DA7280_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	/* Find the snippet memory base. The Linux driver reads MEM_CTL1 and
	 * uses that as the start address, which lets the chip relocate the
	 * snippet area; on POR this is 0x84. We follow the same pattern. */
	ret = da7280_read_reg(dev, DA7280_MEM_CTL1, &base);
	if (ret != 0) {
		return ret;
	}
	if (base < DA7280_SNP_MEM_BASE_DEFAULT || base > DA7280_SNP_MEM_END) {
		LOG_ERR("mem upload: implausible base 0x%02x", base);
		return -EIO;
	}

	{
		uint8_t buf[DA7280_SNP_MEM_SIZE + 1];
		size_t writable = (size_t)(DA7280_SNP_MEM_END - base + 1U);
		size_t to_write = MIN(len, writable);

		buf[0] = base;
		memcpy(&buf[1], image, to_write);
		ret = i2c_write_dt(&cfg->i2c, buf, to_write + 1U);
		if (ret != 0) {
			LOG_ERR("mem upload: i2c_write failed: %d", ret);
			return ret;
		}
	}

	return 0;
}

static int da7280_apply_settings(const struct device *dev)
{
	const struct da7280_config *cfg = dev->config;
	uint8_t mask, val, frq_h, frq_l, imax_code;
	uint16_t v2i;
	int ret, i;
	bool acc_en = cfg->acceleration_en;
	bool rapid_stop_en = cfg->rapid_stop_en;
	bool amp_pid_en = cfg->amp_pid_en;
	uint8_t bemf_fault_lim = cfg->bemf_fault_lim;
	bool top_cfg4_force = false;

	/* === ERM-coin special case (datasheet 5.7.18, mirrors Linux) ===
	 * The coin ERM has time-varying impedance and no observable BEMF, so
	 * the closed-loop features must be off and impedance calibration
	 * disabled. We enforce these regardless of DT, because the chip will
	 * mis-behave otherwise. BEMF fault limit is forced off too. */
	if (cfg->actuator_type == DA7280_ACTUATOR_ERM_COIN) {
		acc_en = false;
		rapid_stop_en = false;
		amp_pid_en = false;
		bemf_fault_lim = 0;
		top_cfg4_force = true;
	}

	/* --- LRA resonant period (write before TOP_CFG1, like Linux) --- */
	if (cfg->actuator_type == DA7280_ACTUATOR_LRA) {
		da7280_freq_to_period(cfg->resonant_hz, &frq_h, &frq_l);
		ret = da7280_write_reg(dev, DA7280_FRQ_LRA_PER_H, frq_h);
		if (ret != 0) {
			return ret;
		}
		ret = da7280_write_reg(dev, DA7280_FRQ_LRA_PER_L, frq_l);
		if (ret != 0) {
			return ret;
		}
	} else if (top_cfg4_force) {
		/* Coin ERM: zero the BEMF fault limit field. */
		ret = da7280_update_bits(dev, DA7280_TOP_INT_CFG1,
					 DA7280_BEMF_FAULT_LIM_MASK, 0);
		if (ret != 0) {
			return ret;
		}
		ret = da7280_update_bits(dev, DA7280_TOP_CFG4,
					 DA7280_TST_CALIB_IMPEDANCE_DIS |
						 DA7280_V2I_FACTOR_FREEZE,
					 DA7280_TST_CALIB_IMPEDANCE_DIS |
						 DA7280_V2I_FACTOR_FREEZE);
		if (ret != 0) {
			return ret;
		}
	}

	/* --- TOP_CFG1: actuator type and feature flags --- */
	mask = DA7280_ACTUATOR_TYPE_BIT | DA7280_BEMF_SENSE_EN |
	       DA7280_FREQ_TRACK_EN | DA7280_ACCELERATION_EN |
	       DA7280_RAPID_STOP_EN | DA7280_AMP_PID_EN |
	       DA7280_EMBEDDED_MODE;
	val = 0;
	if (cfg->actuator_type != DA7280_ACTUATOR_LRA) {
		val |= DA7280_ACTUATOR_TYPE_BIT;
	}
	if (cfg->bemf_sense_en) {
		val |= DA7280_BEMF_SENSE_EN;
	}
	if (cfg->freq_track_en) {
		val |= DA7280_FREQ_TRACK_EN;
	}
	if (acc_en) {
		val |= DA7280_ACCELERATION_EN;
	}
	if (rapid_stop_en) {
		val |= DA7280_RAPID_STOP_EN;
	}
	if (amp_pid_en) {
		val |= DA7280_AMP_PID_EN;
	}
	if (cfg->embedded_mode) {
		val |= DA7280_EMBEDDED_MODE;
	}
	ret = da7280_update_bits(dev, DA7280_TOP_CFG1, mask, val);
	if (ret != 0) {
		return ret;
	}

	/* --- TOP_CFG5: enable V2I_FACTOR_OFFSET when accel is on --- */
	ret = da7280_update_bits(dev, DA7280_TOP_CFG5,
				 DA7280_V2I_FACTOR_OFFSET_EN,
				 acc_en ? DA7280_V2I_FACTOR_OFFSET_EN : 0);
	if (ret != 0) {
		return ret;
	}

	/* --- TOP_CFG2: MEM_DATA_SIGNED inverts with acceleration ---
	 * Accel ON  => memory data is unsigned (MEM_DATA_SIGNED=0).
	 * Accel OFF => memory data is signed.
	 */
	ret = da7280_update_bits(dev, DA7280_TOP_CFG2,
				 DA7280_MEM_DATA_SIGNED |
					 DA7280_FULL_BRAKE_THR_MASK,
				 (acc_en ? 0 : DA7280_MEM_DATA_SIGNED) |
				 (cfg->full_brake_thr &
					 DA7280_FULL_BRAKE_THR_MASK));
	if (ret != 0) {
		return ret;
	}

	/* --- TOP_CFG3: VDD margin (auto output limiting, datasheet 5.7.13) --- */
	ret = da7280_update_bits(dev, DA7280_TOP_CFG3,
				 DA7280_VDD_MARGIN_MASK,
				 cfg->vdd_margin & DA7280_VDD_MARGIN_MASK);
	if (ret != 0) {
		return ret;
	}

	/* --- BEMF fault limit (only for LRA / ERM-bar) --- */
	if (!top_cfg4_force) {
		ret = da7280_update_bits(dev, DA7280_TOP_INT_CFG1,
					 DA7280_BEMF_FAULT_LIM_MASK,
					 bemf_fault_lim &
						 DA7280_BEMF_FAULT_LIM_MASK);
		if (ret != 0) {
			return ret;
		}
	}

	/* --- ACTUATOR1/2/3: NOMMAX, ABSMAX, IMAX --- */
	ret = da7280_write_reg(dev, DA7280_ACTUATOR1,
			       da7280_uv_to_code(cfg->nom_uv));
	if (ret != 0) {
		return ret;
	}
	ret = da7280_write_reg(dev, DA7280_ACTUATOR2,
			       da7280_uv_to_code(cfg->abs_uv));
	if (ret != 0) {
		return ret;
	}
	imax_code = da7280_imax_to_code(cfg->imax_ua);
	ret = da7280_update_bits(dev, DA7280_ACTUATOR3,
				 DA7280_IMAX_MASK, imax_code);
	if (ret != 0) {
		return ret;
	}

	/* --- V2I calibration factor --- */
	v2i = da7280_v2i_factor(cfg->impd_uohm, imax_code);
	ret = da7280_write_reg(dev, DA7280_CALIB_V2I_L, v2i & 0xFFU);
	if (ret != 0) {
		return ret;
	}
	ret = da7280_write_reg(dev, DA7280_CALIB_V2I_H, (v2i >> 8) & 0xFFU);
	if (ret != 0) {
		return ret;
	}

	/* --- Take chip out of standby (Linux step 8) --- */
	ret = da7280_update_bits(dev, DA7280_TOP_CTL1,
				 DA7280_STANDBY_EN, DA7280_STANDBY_EN);
	if (ret != 0) {
		return ret;
	}

	/* --- Optional waveform memory upload --- */
	if (cfg->mem_image != NULL && cfg->mem_image_len > 0U) {
		ret = da7280_mem_upload(dev, cfg->mem_image,
					cfg->mem_image_len);
		if (ret != 0) {
			LOG_ERR("waveform memory upload failed: %d", ret);
			return ret;
		}
	}

	/* --- SEQ_CTL2: PS_SEQ_ID and PS_SEQ_LOOP --- */
	val = ((cfg->ps_seq_loop << 4) & DA7280_PS_SEQ_LOOP_MASK) |
	       (cfg->ps_seq_id & DA7280_PS_SEQ_ID_MASK);
	ret = da7280_write_reg(dev, DA7280_SEQ_CTL2, val);
	if (ret != 0) {
		return ret;
	}

	/* --- GPI[0..2]_CTL --- */
	for (i = 0; i < 3; i++) {
		val = ((cfg->gpi[i].seq_id << 3) & DA7280_GPI_SEQ_ID_MASK) |
		      (cfg->gpi[i].mode ? DA7280_GPI_MODE_BIT : 0) |
		      (cfg->gpi[i].polarity & DA7280_GPI_POLARITY_MASK);
		ret = da7280_write_reg(dev, DA7280_GPI_0_CTL + i, val);
		if (ret != 0) {
			return ret;
		}
	}

	/* --- Mask ADC saturation by default; clear & unmask faults --- */
	ret = da7280_update_bits(dev, DA7280_IRQ_MASK2,
				 DA7280_ADC_SAT_M, DA7280_ADC_SAT_M);
	if (ret != 0) {
		return ret;
	}

	ret = da7280_write_reg(dev, DA7280_IRQ_EVENT1, 0xFF);
	if (ret != 0) {
		return ret;
	}

	/* Linux unmasks SEQ_FAULT and SEQ_DONE specifically (writes 0 to
	 * those bits in IRQ_MASK1). We unmask everything by default; users
	 * who want to mask noisy events (e.g. SEQ_CONTINUE) can do so via
	 * regmap-style direct register access. */
	ret = da7280_update_bits(dev, DA7280_IRQ_MASK1, 0xFF, 0x00);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int da7280_init_irq(const struct device *dev)
{
	const struct da7280_config *cfg = dev->config;
	struct da7280_data *data = dev->data;
	int ret;

	if (!cfg->has_irq) {
		return 0;
	}

	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO controller not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("IRQ pin configure: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->irq_cb, da7280_gpio_irq,
			   BIT(cfg->irq_gpio.pin));

	ret = gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("IRQ enable failed: %d", ret);
		return ret;
	}

	return 0;
}

static int da7280_init(const struct device *dev)
{
	const struct da7280_config *cfg = dev->config;
	struct da7280_data *data = dev->data;
	uint8_t rev;
	int ret;

	data->self = dev;
	k_mutex_init(&data->lock);
	k_work_init(&data->irq_work, da7280_irq_work_handler);

#if !defined(CONFIG_HAPTIC_DA7280_USE_SYSTEM_WORKQUEUE)
	/* One-shot start of the dedicated workqueue, shared by all
	 * instances of this driver. */
	if (!da7280_wq_started) {
		k_work_queue_init(&da7280_wq);
		k_work_queue_start(&da7280_wq, da7280_wq_stack,
				   K_THREAD_STACK_SIZEOF(da7280_wq_stack),
				   K_PRIO_PREEMPT(7), NULL);
		da7280_wq_started = true;
	}
#endif

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	ret = da7280_read_reg(dev, DA7280_CHIP_REV, &rev);
	if (ret != 0) {
		LOG_ERR("I2C probe @0x%02x failed: %d",
			cfg->i2c.addr, ret);
		return ret;
	}
	if (rev != DA7280_CHIP_REV_VAL) {
		LOG_ERR("Unexpected chip rev 0x%02x (expected 0x%02x)",
			rev, DA7280_CHIP_REV_VAL);
		return -ENODEV;
	}

#if defined(CONFIG_HAPTIC_DA7280_SAFE_INIT)
	/*
	 * Optional: defang the chip before applying any configuration.
	 * Use this when the host MCU can reset while the DA7280 keeps
	 * power (e.g. brownout-and-reboot scenarios). Linux does not do
	 * this -- it relies on the chip's POR state matching its own
	 * power-cycle.
	 */
	ret = da7280_safe_init_prologue(dev);
	if (ret != 0) {
		return ret;
	}
#endif

	ret = da7280_apply_settings(dev);
	if (ret != 0) {
		LOG_ERR("apply_settings failed: %d", ret);
		return ret;
	}

	ret = da7280_init_irq(dev);
	if (ret != 0) {
		return ret;
	}

	{
		const char *type_str = "LRA";

		if (cfg->actuator_type == DA7280_ACTUATOR_ERM_BAR) {
			type_str = "ERM-bar";
		} else if (cfg->actuator_type == DA7280_ACTUATOR_ERM_COIN) {
			type_str = "ERM-coin";
		}
		LOG_INF("DA7280 @0x%02x initialized: %s, %u Hz, nom=%u uV, "
			"abs=%u uV, imax=%u uA, accel=%d, rapid_stop=%d, "
			"per_ch_cap=%u",
			cfg->i2c.addr, type_str,
			cfg->resonant_hz, cfg->nom_uv, cfg->abs_uv,
			cfg->imax_ua, cfg->acceleration_en, cfg->rapid_stop_en,
			cfg->per_channel_cap);
	}

	return 0;
}

/* ===================================================================
 * DT_INST instance plumbing.
 * =================================================================== */

/* Map the dlg,actuator-type enum to the driver's internal type:
 *   0: "LRA"      -> DA7280_ACTUATOR_LRA
 *   1: "ERM-bar"  -> DA7280_ACTUATOR_ERM_BAR
 *   2: "ERM-coin" -> DA7280_ACTUATOR_ERM_COIN  (forces accel/rapid-stop/etc OFF)
 */
#define DA7280_ACTUATOR_TYPE_DT(inst)					\
	(DT_INST_ENUM_IDX(inst, dlg_actuator_type) == 0 ?		\
		DA7280_ACTUATOR_LRA :					\
	 DT_INST_ENUM_IDX(inst, dlg_actuator_type) == 1 ?		\
		DA7280_ACTUATOR_ERM_BAR : DA7280_ACTUATOR_ERM_COIN)

/* GPI mode/polarity enums map directly: index 0 = first listed value. */
#define DA7280_GPI_MODE_DT(inst, n)					\
	DT_INST_ENUM_IDX_OR(inst, dlg_gpi##n##_mode, 0)
#define DA7280_GPI_POL_DT(inst, n)					\
	DT_INST_ENUM_IDX_OR(inst, dlg_gpi##n##_polarity, 0)

/* Optional waveform memory image expressed as a uint8 DT array. We keep
 * the storage const and let probe stream it to the chip after init. */
#define DA7280_MEM_IMAGE_DECL(inst)					\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dlg_mem_array),		\
		(static const uint8_t da7280_mem_image_##inst[] =	\
			DT_INST_PROP(inst, dlg_mem_array);))

#define DA7280_MEM_IMAGE_PTR(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dlg_mem_array),		\
		    (da7280_mem_image_##inst), (NULL))

#define DA7280_MEM_IMAGE_LEN(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dlg_mem_array),		\
		    (sizeof(da7280_mem_image_##inst)), (0))

#define DA7280_DEFINE(inst)						\
	DA7280_MEM_IMAGE_DECL(inst)					\
	static struct da7280_data da7280_data_##inst;			\
	static const struct da7280_config da7280_cfg_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		.has_irq = DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
		.irq_gpio = COND_CODE_1(				\
			DT_INST_NODE_HAS_PROP(inst, irq_gpios),		\
			(GPIO_DT_SPEC_INST_GET(inst, irq_gpios)),	\
			({0})),						\
		.nom_uv         = DT_INST_PROP(inst, dlg_nom_microvolt),\
		.abs_uv         = DT_INST_PROP(inst, dlg_abs_max_microvolt),\
		.imax_ua        = DT_INST_PROP(inst, dlg_imax_microamp),\
		.impd_uohm      = DT_INST_PROP(inst, dlg_impd_micro_ohms),\
		.resonant_hz    = DT_INST_PROP(inst, dlg_resonant_freq_hz),\
		.actuator_type  = DA7280_ACTUATOR_TYPE_DT(inst),	\
		.per_channel_cap = DT_INST_PROP(inst, per_channel_drive_cap),\
		.full_brake_thr  = DT_INST_PROP(inst, full_brake_thr),	\
		.bemf_fault_lim  = DT_INST_PROP(inst, bemf_fault_limit),\
		.vdd_margin      = DT_INST_PROP(inst, vdd_margin),	\
		.ps_seq_id       = DT_INST_PROP(inst, dlg_ps_seq_id),	\
		.ps_seq_loop     = DT_INST_PROP(inst, dlg_ps_seq_loop),	\
		.acceleration_en = DT_INST_PROP(inst, dlg_acc_enable),	\
		.rapid_stop_en   = DT_INST_PROP(inst, dlg_rapid_stop_enable),\
		.freq_track_en   = DT_INST_PROP(inst, dlg_freq_track_enable),\
		.bemf_sense_en   = DT_INST_PROP(inst, dlg_bemf_sens_enable),\
		.amp_pid_en      = DT_INST_PROP(inst, dlg_amp_pid_enable),\
		.embedded_mode   = DT_INST_PROP(inst, dlg_embedded_mode),\
		.standby_after_drive = DT_INST_PROP(inst, dlg_standby_after_drive),\
		.gpi[0] = {						\
			.seq_id = DT_INST_PROP(inst, dlg_gpi0_seq_id),	\
			.mode = DA7280_GPI_MODE_DT(inst, 0),		\
			.polarity = DA7280_GPI_POL_DT(inst, 0),		\
		},							\
		.gpi[1] = {						\
			.seq_id = DT_INST_PROP(inst, dlg_gpi1_seq_id),	\
			.mode = DA7280_GPI_MODE_DT(inst, 1),		\
			.polarity = DA7280_GPI_POL_DT(inst, 1),		\
		},							\
		.gpi[2] = {						\
			.seq_id = DT_INST_PROP(inst, dlg_gpi2_seq_id),	\
			.mode = DA7280_GPI_MODE_DT(inst, 2),		\
			.polarity = DA7280_GPI_POL_DT(inst, 2),		\
		},							\
		.mem_image = DA7280_MEM_IMAGE_PTR(inst),		\
		.mem_image_len = DA7280_MEM_IMAGE_LEN(inst),		\
	};								\
	DEVICE_DT_INST_DEFINE(inst,					\
			      da7280_init, NULL,			\
			      &da7280_data_##inst, &da7280_cfg_##inst,	\
			      POST_KERNEL,				\
			      CONFIG_HAPTIC_DA7280_INIT_PRIORITY,	\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(DA7280_DEFINE)
