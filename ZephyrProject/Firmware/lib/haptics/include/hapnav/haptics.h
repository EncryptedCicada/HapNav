/*
 * Wristband haptic stack.
 *
 *   BLE frame  ── ISR ──▶  shared latch (mutex) ──▶ 20 Hz worker ──▶ 4×LRA
 *
 * The worker is decoupled from BLE arrival jitter. Even at the pin's
 * 10 Hz sample rate, the worker still ticks every 50 ms so smoothing,
 * fatigue decay, drop-off pulse pattern, and watchdog mute all evolve
 * with stable time steps regardless of the radio link.
 *
 * Channel mapping (matches hapnav_obstacles.urgency[] indexing on the pin):
 *
 *      [0]  LEFT          outer-left LRA
 *      [1]  CENTER-LEFT   inner-left LRA
 *      [2]  CENTER-RIGHT  inner-right LRA
 *      [3]  RIGHT         outer-right LRA
 */
#ifndef HAPNAV_HAPTICS_H_
#define HAPNAV_HAPTICS_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>

/*
 * Stand up the I2C mux + four DA7280s and start the 20 Hz worker.
 * Returns 0 on success. On failure the worker is not started; the
 * caller can still call hapnav_haptics_consume_frame() but it is a
 * no-op.
 */
int hapnav_haptics_init(void);

/*
 * Hand over the latest BLE frame. Safe to call from BT RX context.
 * Only the obstacles block + timestamp are consumed; the rest is
 * ignored (logging happens elsewhere).
 */
void hapnav_haptics_consume_frame(const struct hapnav_frame *frame);

/*
 * Inject a synthetic obstacles report for bench testing without a pin
 * present. Same latching semantics as consume_frame().
 */
void hapnav_haptics_inject(const struct hapnav_obstacles *obs);

#endif /* HAPNAV_HAPTICS_H_ */
