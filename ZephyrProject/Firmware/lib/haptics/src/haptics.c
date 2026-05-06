#include <hapnav/haptics.h>
#include <hapnav/da7280.h>
#include <hapnav/pca9546a.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(hapnav_haptics, LOG_LEVEL_INF);

/* ── Hardware topology ──────────────────────────────────────────────────── */

#define MUX_ADDR        0x70   /* PCA9546A default                         */
#define DRIVER_ADDR     DA7280_DEFAULT_ADDR

/* Channel-to-mux-port mapping (per the user's wiring):
 *   urgency[0] LEFT          → mux ch 0
 *   urgency[1] CENTER-LEFT   → mux ch 1
 *   urgency[2] CENTER-RIGHT  → mux ch 2
 *   urgency[3] RIGHT         → mux ch 3
 */
#define HAPTICS_NUM_CH  4

/* ── Drive-policy tuning ────────────────────────────────────────────────── */

/* 20 Hz drive update — twice the pin's sample rate, so we can run smoothing
 * and the drop-off pulse pattern on a stable clock independent of BLE jitter. */
#define DRIVE_PERIOD_MS         50

/* If we haven't seen a frame in this long, gradually mute everything. The
 * pin samples at 10 Hz (100 ms), so 250 ms means ~2 missed frames. */
#define WATCHDOG_MS             250

/* Below this raw urgency, the channel is silent — keeps the wristband from
 * buzzing on faint clutter at the edge of perception. */
#define URGENCY_FLOOR           32

/* Maximum amplitude written to TOP_CTL2 on a single channel. The chip
 * allows 127 in acceleration mode. We cap at 60 to keep individual-LRA
 * current draw well below the supply budget — even if a wall-shaped
 * obstacle yields high urgency on all four channels simultaneously,
 * the per-channel cap plus the global cap below keeps the wristband
 * from browning out. */
#define DRIVE_MAX               60

/* Global cap on the SUM of all four channels' drive amplitudes. With
 * DRIVE_MAX=60 and 4 channels, the unconstrained worst case is 240;
 * we proportionally scale all channels down whenever the total wants
 * to exceed this budget, preserving directional ratios. The number
 * tracks the supply ceiling, not the chip's max drive. */
#define DRIVE_MAX_TOTAL         150

/* Diagnostic: every DIAG_PERIOD_TICKS, read back one chip's actual
 * register state and log it alongside our shadow. Catches the case
 * where TOP_CTL2 writes silently aren't landing or the chip has
 * drifted into a fault state. Channel rotates each diagnostic round
 * so we sample all four every ~4 seconds. */
#define DIAG_PERIOD_TICKS       20   /* 20 * 50 ms = 1 s */

/* Below this amplitude the LRA doesn't reliably move — quantise to zero. */
#define DRIVE_MIN_PERCEPT       8

/* Per-tick rate limit: ramps a full-scale jump in ~5 ticks (250 ms). */
#define RATE_LIMIT_PER_TICK     24

/* While the user is turning, dampen drive rather than muting outright —
 * lets sudden close hits still register but avoids spurious slewing buzz. */
#define YAW_SLEW_GAIN_NUM       2
#define YAW_SLEW_GAIN_DEN       5    /* 40 % */

/* Drop-off pattern: 150 ms on / 150 ms off, all four channels in lock-step.
 * Distinct enough from normal proximity buzz that the user can tell a
 * cliff-edge apart from a wall.                                          */
#define DROPOFF_HALF_TICKS      3    /* 3 * 50 ms                         */

/* Fatigue model: a 5 s leaky bucket per channel. Above 70 % duty we start
 * attenuating linearly down to 40 % at full duty, encouraging the actuator
 * (and the wearer's mechanoreceptors) to recover. */
#define FATIGUE_WINDOW_MS       5000U
#define FATIGUE_HIGH_NUM        7
#define FATIGUE_HIGH_DEN        10   /* 0.70 */
#define FATIGUE_FLOOR_NUM       2
#define FATIGUE_FLOOR_DEN       5    /* 0.40 */

/* ── State ──────────────────────────────────────────────────────────────── */

struct latch {
	struct hapnav_obstacles obs;
	uint32_t                rx_time_ms;
	bool                    valid;
};

static struct {
	struct hapnav_pca9546a mux;
	struct hapnav_da7280   drv[HAPTICS_NUM_CH];
	bool                   ch_ok[HAPTICS_NUM_CH];
	bool                   ready;

	struct k_mutex         latch_lock;
	struct latch           latch;

	uint8_t                cur_amp[HAPTICS_NUM_CH];
	uint32_t               on_ms[HAPTICS_NUM_CH];   /* leaky-bucket fatigue */
	uint8_t                dropoff_phase;
	bool                   was_stale;

	struct k_work_delayable tick;
} g;

/* ── Mux-aware driver helpers ───────────────────────────────────────────── */

/*
 * Pattern (mirrors Adafruit's TCA9548A try_lock/unlock):
 *   1. select channel  → mux passes traffic to channel only
 *   2. do the chip work
 *   3. deselect all    → mux is dark; nothing on the upstream bus reaches
 *                        any downstream chip until the next select.
 *
 * The deselect is the safety net: if any later code (or stray noise on
 * the bus) ever ends up writing to 0x4A while a channel is "still on",
 * exactly one chip would react. With deselect, none does.
 */

static int drive_one(int ch, uint8_t amp)
{
	if (!g.ch_ok[ch]) return 0;
	int err = hapnav_pca9546a_select(&g.mux, (uint8_t)ch);
	if (err) return err;
	err = hapnav_da7280_set_amplitude(&g.drv[ch], amp);
	(void)hapnav_pca9546a_deselect_all(&g.mux);
	return err;
}

static int silence_one(int ch)
{
	if (!g.ch_ok[ch]) return 0;
	int err = hapnav_pca9546a_select(&g.mux, (uint8_t)ch);
	if (err) return err;
	err = hapnav_da7280_stop(&g.drv[ch]);
	(void)hapnav_pca9546a_deselect_all(&g.mux);
	return err;
}

static int rearm_one(int ch)
{
	if (!g.ch_ok[ch]) return 0;
	int err = hapnav_pca9546a_select(&g.mux, (uint8_t)ch);
	if (err) return err;
	err = hapnav_da7280_resume(&g.drv[ch]);
	(void)hapnav_pca9546a_deselect_all(&g.mux);
	return err;
}

/*
 * Boot-time hardware diagnostic. Drives each motor solo, in order, at
 * a gentle amplitude. With a working mux you should physically feel
 * exactly one motor at a time, in the order printed. If you feel two
 * or more motors on a single channel, the mux is being bypassed.
 *
 * Three guards against the brownout-then-stuck-on failure mode:
 *   1. Low amplitude (~30 / 127, well under the LRA's rated current).
 *   2. Hard silence_one() between drives — INACTIVE mode halts the
 *      drive stage even if a transient i2c error would have left amp=0
 *      partially applied.
 *   3. rearm_one() afterwards so the chip is back in DRO mode, ready
 *      for the policy worker.
 *
 * Boot delay: 4 channels × (300 ms drive + 300 ms gap) ≈ 2.4 s.
 */
static void haptics_channel_walk(void)
{
	static const char * const role[HAPTICS_NUM_CH] = {
		"LEFT", "CENTER-LEFT", "CENTER-RIGHT", "RIGHT"
	};
	LOG_INF("haptics: channel walk -- one motor at a time");
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		if (!g.ch_ok[ch]) {
			LOG_INF("  ch%d %s skipped (init failed)", ch, role[ch]);
			continue;
		}
		LOG_INF("  ch%d %s on", ch, role[ch]);
		(void)drive_one(ch, 30);
		k_msleep(300);
		LOG_INF("  ch%d %s off", ch, role[ch]);
		(void)silence_one(ch);   /* INACTIVE -- hard halt */
		k_msleep(300);
		(void)rearm_one(ch);     /* back to DRO at amp=0 */
	}
	LOG_INF("haptics: channel walk done");
}

/* ── Policy ─────────────────────────────────────────────────────────────── */

static uint8_t urgency_to_target(uint8_t u)
{
	if (u < URGENCY_FLOOR) {
		return 0;
	}
	uint32_t scaled = ((uint32_t)(u - URGENCY_FLOOR) * DRIVE_MAX) /
			  (255U - URGENCY_FLOOR);
	if (scaled < DRIVE_MIN_PERCEPT) {
		return 0;
	}
	return (uint8_t)scaled;
}

static uint8_t apply_rate_limit(uint8_t cur, uint8_t target)
{
	int d = (int)target - (int)cur;
	if (d >  RATE_LIMIT_PER_TICK) d =  RATE_LIMIT_PER_TICK;
	if (d < -RATE_LIMIT_PER_TICK) d = -RATE_LIMIT_PER_TICK;
	int next = (int)cur + d;
	if (next < 0)   next = 0;
	if (next > DRIVE_MAX) next = DRIVE_MAX;
	return (uint8_t)next;
}

static void update_fatigue(int ch)
{
	/* Linear leak: lose DRIVE_PERIOD_MS of accumulated on-time per tick,
	 * so a fully-saturated bucket drains in FATIGUE_WINDOW_MS even with
	 * zero drive. */
	if (g.on_ms[ch] >= DRIVE_PERIOD_MS) {
		g.on_ms[ch] -= DRIVE_PERIOD_MS;
	} else {
		g.on_ms[ch] = 0;
	}
	if (g.cur_amp[ch] > 0) {
		g.on_ms[ch] += DRIVE_PERIOD_MS;
		if (g.on_ms[ch] > FATIGUE_WINDOW_MS) {
			g.on_ms[ch] = FATIGUE_WINDOW_MS;
		}
	}
}

static uint8_t apply_fatigue(int ch, uint8_t target)
{
	/* duty = on_ms / window in 8-bit fixed point. */
	uint32_t duty_n = ((uint32_t)g.on_ms[ch] * 256U) / FATIGUE_WINDOW_MS;
	uint32_t high_n = (256U * FATIGUE_HIGH_NUM) / FATIGUE_HIGH_DEN;
	if (duty_n <= high_n) {
		return target;
	}
	uint32_t over = duty_n - high_n;             /* 0..(256-high_n)         */
	uint32_t span = 256U - high_n;               /* range over which we taper */
	uint32_t floor_n = (256U * FATIGUE_FLOOR_NUM) / FATIGUE_FLOOR_DEN;
	/* k = 1 - over/span * (1 - floor); clamp at floor. */
	uint32_t k_n = 256U - (over * (256U - floor_n)) / span;
	if (k_n < floor_n) k_n = floor_n;
	return (uint8_t)((target * k_n) / 256U);
}

static void compute_targets(const struct hapnav_obstacles *obs,
			    bool stale, uint8_t target_out[HAPTICS_NUM_CH])
{
	if (stale) {
		memset(target_out, 0, HAPTICS_NUM_CH);
		return;
	}

	const uint8_t flags = obs->flags;
	const bool mute = flags & (HAPNAV_OBS_FLAG_STATIONARY |
				   HAPNAV_OBS_FLAG_SENSOR_BLOCKED |
				   HAPNAV_OBS_FLAG_MOSTLY_INVALID);
	const bool slew = flags & HAPNAV_OBS_FLAG_YAW_SLEWING;
	const bool drop = flags & HAPNAV_OBS_FLAG_DROPOFF;

	if (drop) {
		/* Tick the drop-off square wave; phase wraps every 2 halves. */
		g.dropoff_phase = (uint8_t)((g.dropoff_phase + 1) %
					    (2U * DROPOFF_HALF_TICKS));
		uint8_t amp = (g.dropoff_phase < DROPOFF_HALF_TICKS) ? DRIVE_MAX : 0;
		for (int i = 0; i < HAPTICS_NUM_CH; i++) target_out[i] = amp;
		return;
	}

	g.dropoff_phase = 0;

	if (mute) {
		memset(target_out, 0, HAPTICS_NUM_CH);
		return;
	}

	for (int i = 0; i < HAPTICS_NUM_CH; i++) {
		uint8_t t = urgency_to_target(obs->urgency[i]);
		if (slew) {
			t = (uint8_t)(((uint32_t)t * YAW_SLEW_GAIN_NUM) /
				      YAW_SLEW_GAIN_DEN);
		}
		target_out[i] = t;
	}

	/* Global drive cap. When a wall-shaped obstacle drives all four
	 * channels high simultaneously, the unconstrained sum can exceed
	 * what the wristband supply can deliver and brownout the nRF.
	 * Scale all channels proportionally so directional ratios survive
	 * but the total stays inside the budget. */
	uint32_t total = (uint32_t)target_out[0] + target_out[1] +
			 target_out[2] + target_out[3];
	if (total > DRIVE_MAX_TOTAL) {
		for (int i = 0; i < HAPTICS_NUM_CH; i++) {
			target_out[i] = (uint8_t)
				(((uint32_t)target_out[i] * DRIVE_MAX_TOTAL) /
				 total);
		}
	}
}

/* ── Worker ─────────────────────────────────────────────────────────────── */

static void haptics_tick(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!g.ready) {
		return;
	}

	struct hapnav_obstacles obs;
	uint32_t rx_ms;
	bool valid;

	k_mutex_lock(&g.latch_lock, K_FOREVER);
	obs   = g.latch.obs;
	rx_ms = g.latch.rx_time_ms;
	valid = g.latch.valid;
	k_mutex_unlock(&g.latch_lock);

	const uint32_t now = k_uptime_get_32();
	const bool stale = !valid || (now - rx_ms) > WATCHDOG_MS;

	/* Edge-triggered watchdog. Going stale puts every chip into
	 * OPERATION_MODE = INACTIVE — zeroing TOP_CTL2 alone does not
	 * reliably halt the LRA on the DA7280; the mode bit change is
	 * what gates the drive stage. Coming back online re-arms each
	 * chip into DRO mode at amplitude 0 so the next normal-path
	 * write to TOP_CTL2 takes effect. */
	if (stale && !g.was_stale) {
		LOG_INF("BLE link stale — halting all motors (INACTIVE mode)");
		for (int i = 0; i < HAPTICS_NUM_CH; i++) {
			int err = silence_one(i);
			if (err) {
				LOG_WRN("silence ch%d failed: %d", i, err);
			}
			g.cur_amp[i] = 0;
			g.on_ms[i]   = 0;
		}
		g.was_stale = true;
	} else if (!stale && g.was_stale) {
		LOG_INF("BLE link active — re-arming motors (DRO mode)");
		for (int i = 0; i < HAPTICS_NUM_CH; i++) {
			int err = rearm_one(i);
			if (err) {
				LOG_WRN("rearm ch%d failed: %d", i, err);
			}
			g.cur_amp[i] = 0;
		}
		g.was_stale = false;
	}

	/* While stale, the chips are in INACTIVE mode. Don't bother
	 * touching TOP_CTL2 — the driver would reject the write anyway
	 * (set_amplitude requires dev->active). */
	if (stale) {
		k_work_reschedule(&g.tick, K_MSEC(DRIVE_PERIOD_MS));
		return;
	}

	uint8_t target[HAPTICS_NUM_CH];
	compute_targets(&obs, false, target);

	for (int i = 0; i < HAPTICS_NUM_CH; i++) {
		uint8_t fat = apply_fatigue(i, target[i]);
		uint8_t lim = apply_rate_limit(g.cur_amp[i], fat);

		/* Always assert to the chip — even when lim hasn't changed.
		 * A single transient i2c hiccup that we missed would otherwise
		 * leave the chip's TOP_CTL2 out of sync with our shadow forever,
		 * since the change-gate would never re-issue the write. 80
		 * writes/sec at 400 kHz is ~1.6 % bus utilisation. */
		int err = drive_one(i, lim);
		if (err) {
			LOG_WRN("drive ch%d failed: %d", i, err);
			/* Still advance the shadow — if we keep the old value
			 * the rate limit can't make progress and a stuck-on
			 * channel never recovers. */
		}
		g.cur_amp[i] = lim;
		update_fatigue(i);
	}

	k_work_reschedule(&g.tick, K_MSEC(DRIVE_PERIOD_MS));
}

/* ── Init ───────────────────────────────────────────────────────────────── */

int hapnav_haptics_init(void)
{
	const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c)) {
		LOG_ERR("i2c1 not ready");
		return -ENODEV;
	}

	k_mutex_init(&g.latch_lock);
	memset(&g.latch, 0, sizeof(g.latch));
	memset(g.cur_amp, 0, sizeof(g.cur_amp));
	memset(g.on_ms, 0, sizeof(g.on_ms));
	g.was_stale = true;  /* No frame yet — boot in the same state we'd be
	                      * after a watchdog mute, so the first arriving
	                      * frame logs a clean "re-arming" transition. */

	int err = hapnav_pca9546a_init(&g.mux, i2c, MUX_ADDR);
	if (err) {
		LOG_ERR("PCA9546A init failed: %d", err);
		return err;
	}

	int ok_count = 0;
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		err = hapnav_pca9546a_select(&g.mux, (uint8_t)ch);
		if (err) {
			LOG_WRN("mux select ch%d failed: %d", ch, err);
			g.ch_ok[ch] = false;
			continue;
		}
		err = hapnav_da7280_init(&g.drv[ch], i2c, DRIVER_ADDR,
					 &HAPNAV_DA7280_CFG_DEFAULT);
		g.ch_ok[ch] = (err == 0);
		if (err) {
			LOG_WRN("DA7280 init ch%d failed: %d", ch, err);
		} else {
			ok_count++;
		}
	}
	(void)hapnav_pca9546a_deselect_all(&g.mux);

	if (ok_count == 0) {
		LOG_ERR("No haptic drivers responded");
		return -ENODEV;
	}
	LOG_INF("Haptics ready: %d/%d channels online", ok_count, HAPTICS_NUM_CH);

	/* Diagnostic: with the mux deselected, is anything still answering
	 * at the DA7280 address? A successful read here proves a chip is on
	 * the upstream side of the mux — i.e., not isolated, regardless of
	 * what we tell the mux to do. */
	{
		(void)hapnav_pca9546a_deselect_all(&g.mux);
		uint8_t reg = 0x00, val = 0;
		int probe = i2c_write_read(i2c, DRIVER_ADDR, &reg, 1, &val, 1);
		if (probe == 0) {
			LOG_ERR("DA7280 ANSWERED at 0x%02x with mux deselected — "
				"a chip is on the upstream bus, not behind the "
				"mux. Mux isolation is bypassed; all four chips "
				"will respond in unison. Check that each DA7280 "
				"is wired to its own mux channel output (not "
				"daisy-chained on one channel).", DRIVER_ADDR);
		} else {
			LOG_INF("Mux isolation OK: nothing at 0x%02x when "
				"deselected (probe=%d).", DRIVER_ADDR, probe);
		}
	}

	g.ready = true;
	haptics_channel_walk();
	k_work_init_delayable(&g.tick, haptics_tick);
	k_work_reschedule(&g.tick, K_MSEC(DRIVE_PERIOD_MS));
	return 0;
}

void hapnav_haptics_consume_frame(const struct hapnav_frame *frame)
{
	if (!frame) return;
	hapnav_haptics_inject(&frame->obstacles);
}

void hapnav_haptics_inject(const struct hapnav_obstacles *obs)
{
	if (!obs) return;
	k_mutex_lock(&g.latch_lock, K_FOREVER);
	g.latch.obs        = *obs;
	g.latch.rx_time_ms = k_uptime_get_32();
	g.latch.valid      = true;
	k_mutex_unlock(&g.latch_lock);
}
