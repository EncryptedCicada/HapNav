/*
 * Wristband haptic stack — high-level policy on top of the DA7280 driver.
 *
 *   BLE frame  ── ISR ──▶  shared latch (mutex) ──▶ 20 Hz worker ──▶ 4×LRA
 *
 * The four DA7280s sit behind a TI/NXP TCA9546A I2C mux which Zephyr
 * exposes as four virtual i2c buses, so this layer never sees the mux —
 * it just calls da7280_set_amplitude() on the right device handle. Device
 * handles are bound to DT aliases:
 *
 *   hapnav-lra-l   urgency[0]  LEFT          outer-left LRA
 *   hapnav-lra-cl  urgency[1]  CENTER-LEFT   inner-left LRA
 *   hapnav-lra-cr  urgency[2]  CENTER-RIGHT  inner-right LRA
 *   hapnav-lra-r   urgency[3]  RIGHT         outer-right LRA
 *
 * The worker is decoupled from BLE arrival jitter; even at the pin's
 * 10 Hz sample rate it ticks every 50 ms so smoothing, fatigue decay,
 * the drop-off pulse pattern, and the watchdog mute all evolve on a
 * stable clock independent of the radio link.
 *
 * Selftest mode (CONFIG_HAPNAV_SELFTEST=y) keeps the same API but
 * makes consume_frame() a no-op and runs a comprehensive bench-test
 * sequence in a dedicated thread instead. inject() still works in both
 * modes for synthetic stimulation.
 */
#ifndef HAPNAV_HAPTICS_H_
#define HAPNAV_HAPTICS_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>

/*
 * Bring up the four DA7280 channels and start the 20 Hz worker. In
 * selftest mode this also spawns the selftest thread that exercises
 * the chips in isolation. Returns 0 on success; on failure the worker
 * is not started, and the consume_frame()/inject() entry points are
 * silent.
 */
int hapnav_haptics_init(void);

/*
 * Hand over the latest BLE frame. Safe to call from BT RX context.
 * Only the obstacles block + timestamp are consumed. In selftest mode
 * this is a no-op (frames still arrive over BLE for parsability, but
 * the wristband does not react to them).
 */
void hapnav_haptics_consume_frame(const struct hapnav_frame *frame);

/*
 * Inject a synthetic obstacles report — used by both bench testing and
 * the selftest harness's simulated-input phase. Always honored.
 */
void hapnav_haptics_inject(const struct hapnav_obstacles *obs);

#endif /* HAPNAV_HAPTICS_H_ */
