/*
 * HapNav wristband haptic policy + selftest harness.
 *
 * The four DA7280s sit behind a TI/NXP TCA9546A mux that Zephyr exposes
 * as four virtual i2c buses. Each DA7280 has its own DT node (alias
 * hapnav-lra-{l,cl,cr,r}) and the in-tree mux driver handles channel
 * selection transparently — this layer just calls da7280_set_amplitude()
 * with a device handle.
 *
 * Two operating modes share the same code:
 *
 *   normal:    BLE frames → consume_frame() → latch → 20 Hz worker
 *              → policy → da7280_set_amplitude() per channel.
 *
 *   selftest:  BLE frames are ignored; a dedicated thread executes a
 *              comprehensive bench-test sequence (probes, channel walk,
 *              ramps, crossfades, all-on stress, update-rate stress,
 *              drop-off pattern, simulated obstacles, fatigue taper,
 *              watchdog mute, idle-hold). The 20 Hz worker still runs
 *              so simulated-obstacle tests exercise the full policy.
 */
#include <hapnav/haptics.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/haptic/da7280.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <string.h>

LOG_MODULE_REGISTER(hapnav_haptics, LOG_LEVEL_INF);

/* ── Channel mapping ───────────────────────────────────────────────────── */

#define HAPTICS_NUM_CH  4

enum { CH_L = 0, CH_CL = 1, CH_CR = 2, CH_R = 3 };

static const struct device *const lras[HAPTICS_NUM_CH] = {
	DEVICE_DT_GET(DT_ALIAS(hapnav_lra_l)),
	DEVICE_DT_GET(DT_ALIAS(hapnav_lra_cl)),
	DEVICE_DT_GET(DT_ALIAS(hapnav_lra_cr)),
	DEVICE_DT_GET(DT_ALIAS(hapnav_lra_r)),
};

static const char *const role[HAPTICS_NUM_CH] = {
	"LEFT", "CENTER-LEFT", "CENTER-RIGHT", "RIGHT"
};

/* ── Drive-policy tuning ───────────────────────────────────────────────── */

/* 20 Hz drive update — twice the pin's sample rate so smoothing, drop-off
 * pulse, and fatigue evolve on a stable clock independent of BLE jitter. */
#define DRIVE_PERIOD_MS         50

/* If we haven't seen a frame in this long, mute everything. The pin
 * samples at 10 Hz (100 ms), so 250 ms means ~2 missed frames. */
#define WATCHDOG_MS             250

/* Below this raw urgency, the channel is silent — keeps the wristband
 * from buzzing on faint clutter at the edge of perception. */
#define URGENCY_FLOOR           32

/* Per-channel amplitude cap. Matches per-channel-drive-cap in the DT
 * binding so the chip and the policy agree on the ceiling. */
#define DRIVE_MAX               60

/* Global cap on the SUM of the four channels' amplitudes. The driver
 * also enforces CONFIG_HAPTIC_DA7280_TOTAL_DRIVE_CAP as a hard backstop;
 * we scale proportionally here so directional ratios survive when the
 * unconstrained sum would exceed the supply budget. */
#define DRIVE_MAX_TOTAL         150

/* Below this amplitude the LRA doesn't reliably move — quantise to zero. */
#define DRIVE_MIN_PERCEPT       8

/* Per-tick rate limit: ramps a full-scale jump in ~5 ticks (250 ms). */
#define RATE_LIMIT_PER_TICK     24

/* While the user is turning, dampen drive rather than muting outright. */
#define YAW_SLEW_GAIN_NUM       2
#define YAW_SLEW_GAIN_DEN       5    /* 40 % */

/* Drop-off pattern: 150 ms on / 150 ms off, all four channels in lock-step. */
#define DROPOFF_HALF_TICKS      3    /* 3 * 50 ms                          */

/* Head-obstacle pattern: faster pulse than drop-off so the two cliff-edge
 * cues stay perceptually distinct. 50 ms on / 50 ms off → 10 Hz on the
 * outer (LEFT+RIGHT) channels only; centre channels are left muted so the
 * "ducking" feel is associated with the bracketing pair. */
#define HEAD_HALF_TICKS         1    /* 1 * 50 ms                          */

/* Fatigue model: 5 s leaky bucket per channel. Above 70 % duty we taper
 * linearly down to 40 % at full duty. */
#define FATIGUE_WINDOW_MS       5000U
#define FATIGUE_HIGH_NUM        7
#define FATIGUE_HIGH_DEN        10   /* 0.70 */
#define FATIGUE_FLOOR_NUM       2
#define FATIGUE_FLOOR_DEN       5    /* 0.40 */

/* ── State ────────────────────────────────────────────────────────────── */

struct latch {
	struct hapnav_obstacles obs;
	uint32_t                rx_time_ms;
	bool                    valid;
};

static struct {
	bool                   ch_ok[HAPTICS_NUM_CH];
	bool                   ready;

	struct k_mutex         latch_lock;
	struct latch           latch;

	uint8_t                cur_amp[HAPTICS_NUM_CH];
	uint32_t               on_ms[HAPTICS_NUM_CH];   /* leaky-bucket fatigue */
	uint8_t                dropoff_phase;
	uint8_t                head_phase;
	bool                   was_stale;

	struct k_work_delayable tick;
} g;

/* ── Per-channel drive helpers ────────────────────────────────────────── */

static int drive_one(int ch, uint8_t amp)
{
	if (!g.ch_ok[ch]) {
		return 0;
	}
	int err = da7280_set_amplitude(lras[ch], (int)amp);
	if (err && err != -EBUSY) {
		LOG_WRN("ch%d set_amplitude(%u) → %d", ch, amp, err);
	}
	return err;
}

/* ── Boot probe ───────────────────────────────────────────────────────── */

static void probe_one(int ch)
{
	uint8_t rev = 0, status = 0;
	uint32_t vdd = 0, imp = 0;

	(void)da7280_read_chip_rev(lras[ch], &rev);
	(void)da7280_get_status(lras[ch], &status);
	(void)da7280_get_vdd_mv(lras[ch], &vdd);
	(void)da7280_get_impedance_milliohm(lras[ch], &imp);

	LOG_INF("  ch%d %-12s rev=0x%02x status=0x%02x vdd=%u mV imp=%u mΩ",
		ch, role[ch], rev, status, vdd, imp);
}

/* ── Policy ───────────────────────────────────────────────────────────── */

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
	/* True leaky-bucket: fill while driving, leak only while idle. The
	 * earlier "leak every tick + fill every tick when active" cancelled
	 * to zero net at 100 % duty, so the bucket never climbed past one
	 * tick worth and the taper never engaged. */
	if (g.cur_amp[ch] > 0) {
		g.on_ms[ch] += DRIVE_PERIOD_MS;
		if (g.on_ms[ch] > FATIGUE_WINDOW_MS) {
			g.on_ms[ch] = FATIGUE_WINDOW_MS;
		}
	} else if (g.on_ms[ch] >= DRIVE_PERIOD_MS) {
		g.on_ms[ch] -= DRIVE_PERIOD_MS;
	} else {
		g.on_ms[ch] = 0;
	}
}

static uint8_t apply_fatigue(int ch, uint8_t target)
{
	uint32_t duty_n = ((uint32_t)g.on_ms[ch] * 256U) / FATIGUE_WINDOW_MS;
	uint32_t high_n = (256U * FATIGUE_HIGH_NUM) / FATIGUE_HIGH_DEN;
	if (duty_n <= high_n) {
		return target;
	}
	uint32_t over = duty_n - high_n;
	uint32_t span = 256U - high_n;
	uint32_t floor_n = (256U * FATIGUE_FLOOR_NUM) / FATIGUE_FLOOR_DEN;
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
	const bool head = flags & HAPNAV_OBS_FLAG_HEAD_OBSTACLE;

	if (drop) {
		g.dropoff_phase = (uint8_t)((g.dropoff_phase + 1) %
					    (2U * DROPOFF_HALF_TICKS));
		uint8_t amp = (g.dropoff_phase < DROPOFF_HALF_TICKS) ? DRIVE_MAX : 0;
		for (int i = 0; i < HAPTICS_NUM_CH; i++) target_out[i] = amp;
		g.head_phase = 0;
		return;
	}

	g.dropoff_phase = 0;

	if (head) {
		/* Fast pulse on outer pair (LEFT + RIGHT). Centre channels
		 * stay muted — the bracketing feel is the cue to duck. */
		g.head_phase = (uint8_t)((g.head_phase + 1) %
					 (2U * HEAD_HALF_TICKS));
		uint8_t amp = (g.head_phase < HEAD_HALF_TICKS) ? DRIVE_MAX : 0;
		target_out[0] = amp;
		target_out[1] = 0;
		target_out[2] = 0;
		target_out[3] = amp;
		return;
	}

	g.head_phase = 0;

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

/* ── Worker ───────────────────────────────────────────────────────────── */

static void worker_tick(struct k_work *work)
{
	struct latch snap;

	k_mutex_lock(&g.latch_lock, K_FOREVER);
	snap = g.latch;
	k_mutex_unlock(&g.latch_lock);

	uint32_t now = k_uptime_get_32();
	bool stale = !snap.valid || (now - snap.rx_time_ms) > WATCHDOG_MS;

	if (stale && !g.was_stale) {
		LOG_INF("BLE link stale — halting all motors (INACTIVE)");
		for (int i = 0; i < HAPTICS_NUM_CH; i++) {
			(void)drive_one(i, 0);
			g.cur_amp[i] = 0;
			g.on_ms[i] = 0;
		}
		g.was_stale = true;
	}

	uint8_t target[HAPTICS_NUM_CH];
	compute_targets(&snap.obs, stale, target);

	for (int i = 0; i < HAPTICS_NUM_CH; i++) {
		uint8_t fatigued = apply_fatigue(i, target[i]);
		uint8_t next = apply_rate_limit(g.cur_amp[i], fatigued);
		if (next != g.cur_amp[i]) {
			(void)drive_one(i, next);
			g.cur_amp[i] = next;
		}
		update_fatigue(i);
	}

	if (!stale) {
		g.was_stale = false;
	}

	k_work_schedule(&g.tick, K_MSEC(DRIVE_PERIOD_MS));
}

/* ── Public API ──────────────────────────────────────────────────────── */

void hapnav_haptics_consume_frame(const struct hapnav_frame *frame)
{
	if (!g.ready || frame == NULL) {
		return;
	}
#if defined(CONFIG_HAPNAV_SELFTEST)
	/* Selftest mode ignores BLE-driven reactions; obstacles get their
	 * stimulation from the selftest thread via inject() instead. */
	(void)frame;
	return;
#else
	k_mutex_lock(&g.latch_lock, K_FOREVER);
	g.latch.obs = frame->obstacles;
	g.latch.rx_time_ms = k_uptime_get_32();
	g.latch.valid = true;
	k_mutex_unlock(&g.latch_lock);
#endif
}

void hapnav_haptics_inject(const struct hapnav_obstacles *obs)
{
	if (!g.ready || obs == NULL) {
		return;
	}
	k_mutex_lock(&g.latch_lock, K_FOREVER);
	g.latch.obs = *obs;
	g.latch.rx_time_ms = k_uptime_get_32();
	g.latch.valid = true;
	k_mutex_unlock(&g.latch_lock);
}

/* ── Selftest harness ────────────────────────────────────────────────── */

#if defined(CONFIG_HAPNAV_SELFTEST)

static void log_irq_events(int ch, const char *tag)
{
	uint8_t events = 0, warn = 0;

	(void)da7280_get_and_clear_events(lras[ch], &events);
	if (events & DA7280_FAULT_WARNING) {
		(void)da7280_get_warnings(lras[ch], &warn);
	}
	if (events) {
		LOG_WRN("  ch%d %-8s IRQ_EVENT1=0x%02x WARN=0x%02x",
			ch, tag, events, warn);
	} else {
		LOG_INF("  ch%d %-8s IRQ_EVENT1=0x00", ch, tag);
	}
}

/* T0: probe + telemetry + clear any stale latched events. */
static void test0_probe(void)
{
	LOG_INF("");
	LOG_INF("T0: probe + telemetry");
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		if (!g.ch_ok[ch]) {
			LOG_INF("  ch%d %-12s SKIPPED (init failed)", ch, role[ch]);
			continue;
		}
		probe_one(ch);
		log_irq_events(ch, "preclr");
	}
}

/* T1: solo channel walk — drive each motor on its own at low/mid/high. */
static void test1_channel_walk(void)
{
	static const uint8_t levels[] = { 30, 50, DRIVE_MAX };

	LOG_INF("");
	LOG_INF("T1: channel walk (solo, low→high)");
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		if (!g.ch_ok[ch]) continue;
		LOG_INF("  ch%d %s", ch, role[ch]);
		for (size_t i = 0; i < ARRAY_SIZE(levels); i++) {
			(void)drive_one(ch, levels[i]);
			k_msleep(250);
		}
		(void)drive_one(ch, 0);
		k_msleep(200);
		log_irq_events(ch, "post");
	}
}

/* T2: per-channel ramp 0→max→0 over ~1.5 s, with mid-ramp VDD readback. */
static void test2_per_channel_ramp(void)
{
	LOG_INF("");
	LOG_INF("T2: per-channel ramp (0→%d→0)", DRIVE_MAX);
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		if (!g.ch_ok[ch]) continue;
		uint32_t vdd_mid = 0;
		for (int amp = 0; amp <= DRIVE_MAX; amp += 5) {
			(void)drive_one(ch, (uint8_t)amp);
			if (amp == DRIVE_MAX) {
				(void)da7280_get_vdd_mv(lras[ch], &vdd_mid);
			}
			k_msleep(60);
		}
		for (int amp = DRIVE_MAX; amp >= 0; amp -= 5) {
			(void)drive_one(ch, (uint8_t)amp);
			k_msleep(60);
		}
		(void)drive_one(ch, 0);
		LOG_INF("  ch%d %-12s peak vdd=%u mV", ch, role[ch], vdd_mid);
		log_irq_events(ch, "post");
		k_msleep(150);
	}
}

/* T3: pairwise crossfade — keeps the sum at DRIVE_MAX while ramping
 * one channel down and the next one up. Validates concurrent drive
 * at the per-channel-cap boundary. */
static void test3_pairwise_crossfade(void)
{
	static const int pairs[][2] = { {0, 1}, {1, 2}, {2, 3} };
	static const int steps = 20;

	LOG_INF("");
	LOG_INF("T3: pairwise crossfade");
	for (size_t p = 0; p < ARRAY_SIZE(pairs); p++) {
		int a = pairs[p][0], b = pairs[p][1];
		if (!g.ch_ok[a] || !g.ch_ok[b]) continue;
		LOG_INF("  %s ↔ %s", role[a], role[b]);
		for (int s = 0; s <= steps; s++) {
			uint8_t va = (uint8_t)((DRIVE_MAX * (steps - s)) / steps);
			uint8_t vb = (uint8_t)((DRIVE_MAX * s) / steps);
			(void)drive_one(a, va);
			(void)drive_one(b, vb);
			k_msleep(40);
		}
		(void)drive_one(a, 0);
		(void)drive_one(b, 0);
		log_irq_events(a, "post-a");
		log_irq_events(b, "post-b");
		k_msleep(150);
	}
}

/* T4: all-on stress — drive all 4 simultaneously, stepping the per-channel
 * amplitude up by 10 every 500 ms. Bails on the first chip-side fault. */
static void test4_all_on_stress(void)
{
	LOG_INF("");
	LOG_INF("T4: all-on stress (4 channels concurrently)");
	bool fault = false;
	for (int amp = 10; amp <= DRIVE_MAX && !fault; amp += 10) {
		for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
			(void)drive_one(ch, (uint8_t)amp);
		}
		k_msleep(500);
		uint32_t vdd0 = 0;
		(void)da7280_get_vdd_mv(lras[0], &vdd0);
		LOG_INF("  amp=%2d  vdd[ch0]=%u mV", amp, vdd0);
		for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
			uint8_t events = 0;
			(void)da7280_get_and_clear_events(lras[ch], &events);
			if (events & (DA7280_FAULT_UVLO |
				      DA7280_FAULT_OVERTEMP_CRIT |
				      DA7280_FAULT_OVERCURRENT |
				      DA7280_FAULT_ACTUATOR)) {
				LOG_ERR("  ch%d FAULT 0x%02x at amp=%d",
					ch, events, amp);
				fault = true;
			}
		}
	}
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		(void)drive_one(ch, 0);
	}
	k_msleep(150);
	LOG_INF("T4: %s", fault ? "stopped on fault" : "passed full sweep");
}

/* T5: update-rate stress — issue alternating-amplitude writes to a
 * single channel at increasing frequencies. Manufacturer constraint on
 * TOP_CTL2 is essentially I2C bandwidth (no internal latching gate);
 * this test confirms the chip + bus can keep up. We alternate between
 * two non-zero amplitudes so the chip stays in DRO mode (avoid mode
 * changes that go INACTIVE↔DRO at every step). */
static void test5_update_rate_stress(void)
{
	static const int rates_hz[] = { 10, 50, 100, 200, 500 };
	const int ch = CH_L;

	LOG_INF("");
	LOG_INF("T5: update-rate stress on %s (alt 25↔45)", role[ch]);
	if (!g.ch_ok[ch]) {
		LOG_INF("T5: skipped (ch0 not ready)");
		return;
	}
	for (size_t r = 0; r < ARRAY_SIZE(rates_hz); r++) {
		int hz = rates_hz[r];
		int period_us = 1000000 / hz;
		int n = hz * 2;       /* 2 s of toggling */
		int errs = 0;
		uint8_t toggle = 25;
		uint32_t t0 = k_uptime_get_32();
		for (int i = 0; i < n; i++) {
			int e = da7280_set_amplitude(lras[ch],
						     (int)toggle);
			if (e && e != -EBUSY) errs++;
			toggle = (toggle == 25) ? 45 : 25;
			k_busy_wait(period_us);
		}
		uint32_t dt = k_uptime_get_32() - t0;
		(void)drive_one(ch, 0);
		uint8_t events = 0;
		(void)da7280_get_and_clear_events(lras[ch], &events);
		LOG_INF("  %4d Hz  n=%d  dt=%u ms  errs=%d  IRQ=0x%02x",
			hz, n, dt, errs, events);
		k_msleep(150);
	}
}

/* T6: drop-off pattern — exercises the policy's square-wave path on
 * all four channels. Bypasses the worker by driving directly so timing
 * matches DROPOFF_HALF_TICKS exactly. */
static void test6_dropoff_pattern(void)
{
	LOG_INF("");
	LOG_INF("T6: drop-off square wave (all 4 channels)");
	for (int cycle = 0; cycle < 5; cycle++) {
		for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
			(void)drive_one(ch, DRIVE_MAX);
		}
		k_msleep(150);
		for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
			(void)drive_one(ch, 0);
		}
		k_msleep(150);
	}
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		log_irq_events(ch, "post");
	}
}

/* Inject helper: builds a synthetic obstacles struct and drops it in
 * the latch. The 20 Hz worker picks it up on its next tick. */
static void inject_pattern(uint8_t u_l, uint8_t u_cl,
			   uint8_t u_cr, uint8_t u_r, uint8_t flags)
{
	struct hapnav_obstacles obs = {
		.urgency = { u_l, u_cl, u_cr, u_r },
		.nearest_range_mm = 500,
		.flags = flags,
	};
	hapnav_haptics_inject(&obs);
}

static void log_cur_amps(const char *tag)
{
	LOG_INF("  %-22s amp=[%3u %3u %3u %3u]",
		tag,
		g.cur_amp[0], g.cur_amp[1], g.cur_amp[2], g.cur_amp[3]);
}

/* T7: simulated obstacle reactions — feed the policy worker realistic
 * urgency patterns and observe the resulting per-channel amplitudes. */
static void test7_simulated_obstacles(void)
{
	struct {
		const char *desc;
		uint8_t u[4];
		uint8_t flags;
		uint32_t hold_ms;
	} cases[] = {
		{ "obstacle on LEFT",         {220,   0,   0,   0}, 0, 800 },
		{ "obstacle on RIGHT",        {  0,   0,   0, 220}, 0, 800 },
		{ "obstacle CENTER (CL+CR)",  {  0, 200, 200,   0}, 0, 800 },
		{ "wall (all 4 high)",        {220, 220, 220, 220}, 0, 800 },
		{ "yaw slewing dampen",       {220, 220, 220, 220},
		  HAPNAV_OBS_FLAG_YAW_SLEWING, 800 },
		{ "stationary mute",          {220, 220, 220, 220},
		  HAPNAV_OBS_FLAG_STATIONARY,  500 },
		{ "drop-off flag",            {  0,   0,   0,   0},
		  HAPNAV_OBS_FLAG_DROPOFF,     900 },
		{ "head-obstacle flag",       {  0,   0,   0,   0},
		  HAPNAV_OBS_FLAG_HEAD_OBSTACLE, 700 },
		{ "clear field",              {  0,   0,   0,   0}, 0, 400 },
	};

	LOG_INF("");
	LOG_INF("T7: simulated obstacle reactions");
	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		LOG_INF("  case: %s", cases[i].desc);
		uint32_t t_end = k_uptime_get_32() + cases[i].hold_ms;
		while (k_uptime_get_32() < t_end) {
			inject_pattern(cases[i].u[0], cases[i].u[1],
				       cases[i].u[2], cases[i].u[3],
				       cases[i].flags);
			k_msleep(80);
		}
		log_cur_amps(cases[i].desc);
	}
	/* Quiet the policy before we leave the test. */
	inject_pattern(0, 0, 0, 0, 0);
	k_msleep(300);
}

/* T8: fatigue taper — hold one channel at high urgency and watch the
 * amplitude attenuate as the leaky-bucket fills. */
static void test8_fatigue_taper(void)
{
	LOG_INF("");
	LOG_INF("T8: fatigue taper (LEFT held at high urgency for 8 s)");
	uint32_t t_end = k_uptime_get_32() + 8000;
	uint32_t next_log = k_uptime_get_32();
	while (k_uptime_get_32() < t_end) {
		inject_pattern(220, 0, 0, 0, 0);
		uint32_t now = k_uptime_get_32();
		if (now >= next_log) {
			LOG_INF("  t=%u ms  amp[L]=%u  bucket=%u ms",
				now - (t_end - 8000),
				g.cur_amp[0], g.on_ms[0]);
			next_log = now + 1000;
		}
		k_msleep(80);
	}
	inject_pattern(0, 0, 0, 0, 0);
	k_msleep(500);
}

/* T9: watchdog mute — inject high urgency briefly, then stop and
 * verify the worker brings everything to 0 within ~250 ms. */
static void test9_watchdog_mute(void)
{
	LOG_INF("");
	LOG_INF("T9: watchdog mute (no inject for 500 ms after burst)");
	uint32_t t_end = k_uptime_get_32() + 400;
	while (k_uptime_get_32() < t_end) {
		inject_pattern(200, 0, 0, 200, 0);
		k_msleep(80);
	}
	log_cur_amps("after burst");
	k_msleep(500);
	log_cur_amps("after 500 ms idle");
	bool muted = (g.cur_amp[0] | g.cur_amp[1] |
		      g.cur_amp[2] | g.cur_amp[3]) == 0;
	LOG_INF("T9: %s", muted ? "PASS (all silent)" : "FAIL (still driving)");
}

/* T10: idle hold — leave everything quiet for 5 s and surveil
 * IRQ_EVENT1 on each chip. Anything non-zero is a spontaneous fault. */
static void test10_idle_hold(void)
{
	LOG_INF("");
	LOG_INF("T10: idle hold (5 s, watching IRQ_EVENT1)");
	/* Clear anything that prior heavy-drive tests latched (e.g. a
	 * brief UVLO during the T9 burst) so we only report events that
	 * fire spontaneously inside this idle window. */
	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		uint8_t scratch = 0;
		(void)da7280_get_and_clear_events(lras[ch], &scratch);
	}
	for (int s = 0; s < 5; s++) {
		k_msleep(1000);
		for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
			uint8_t events = 0;
			(void)da7280_get_and_clear_events(lras[ch], &events);
			if (events) {
				LOG_WRN("  t=%ds ch%d spontaneous IRQ 0x%02x",
					s + 1, ch, events);
			}
		}
	}
	LOG_INF("T10: idle surveillance complete");
}

static void selftest_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	/* Boot delay so the first BLE adverts and the haptics init logs
	 * have already drained the UART buffer. */
	k_msleep(800);

	LOG_INF("");
	LOG_INF("════════════ HapNav haptics SELFTEST ════════════");

	test0_probe();
	test1_channel_walk();
	test2_per_channel_ramp();
	test3_pairwise_crossfade();
	test4_all_on_stress();
	test5_update_rate_stress();
	test6_dropoff_pattern();

	/* Hand the chips back to the policy worker for the simulated-input
	 * phase. From here on, all drive writes go through the worker and
	 * the selftest thread only injects synthetic obstacles. */
	LOG_INF("");
	LOG_INF("--- starting policy worker ---");
	k_work_schedule(&g.tick, K_MSEC(DRIVE_PERIOD_MS));

	test7_simulated_obstacles();
	test8_fatigue_taper();
	test9_watchdog_mute();
	test10_idle_hold();

	LOG_INF("");
	LOG_INF("════════════ selftest complete ════════════");
	LOG_INF("BLE remains up; consume_frame() is a no-op in selftest mode.");
	LOG_INF("Use west monitor to scrape the log; reset the board to rerun.");
}

K_THREAD_STACK_DEFINE(selftest_stack, CONFIG_HAPNAV_SELFTEST_THREAD_STACK_SIZE);
static struct k_thread selftest_tcb;

#endif /* CONFIG_HAPNAV_SELFTEST */

/* ── Init ─────────────────────────────────────────────────────────────── */

int hapnav_haptics_init(void)
{
	int ready_count = 0;

	k_mutex_init(&g.latch_lock);
	k_work_init_delayable(&g.tick, worker_tick);

	for (int ch = 0; ch < HAPTICS_NUM_CH; ch++) {
		if (!device_is_ready(lras[ch])) {
			LOG_ERR("ch%d %s not ready", ch, role[ch]);
			g.ch_ok[ch] = false;
			continue;
		}
		g.ch_ok[ch] = true;
		ready_count++;
		probe_one(ch);
	}

	if (ready_count == 0) {
		LOG_ERR("No DA7280 channels initialised — aborting");
		return -ENODEV;
	}

	g.ready = true;

#if defined(CONFIG_HAPNAV_SELFTEST)
	LOG_INF("haptics: SELFTEST mode (%d/%d channels ready)",
		ready_count, HAPTICS_NUM_CH);
	(void)k_thread_create(&selftest_tcb, selftest_stack,
			      K_THREAD_STACK_SIZEOF(selftest_stack),
			      selftest_thread, NULL, NULL, NULL,
			      K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&selftest_tcb, "hapnav_selftest");
	/* The worker is started later (from inside the selftest thread,
	 * just before T7) so the raw-chip phase isn't fighting periodic
	 * "stale → drive 0" writes from the worker. */
#else
	LOG_INF("haptics: %d/%d channels ready, worker @ %d Hz",
		ready_count, HAPTICS_NUM_CH, 1000 / DRIVE_PERIOD_MS);
	k_work_schedule(&g.tick, K_MSEC(DRIVE_PERIOD_MS));
#endif

	return 0;
}
