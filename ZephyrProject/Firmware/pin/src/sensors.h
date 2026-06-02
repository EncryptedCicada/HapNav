/*
 * Pin-side sensor orchestration. Owns the BNO055 (UART, NDOF fusion) and
 * the three muxed ToFs (front VL53L5CX, head VL53L1X, down VL53L1X), and
 * gives the rest of the app one call to assemble a complete BLE frame.
 */
#ifndef HAPNAV_PIN_SENSORS_H_
#define HAPNAV_PIN_SENSORS_H_

#include <hapnav/ble_proto.h>

int  pin_sensors_init(void);

/*
 * Read the BNO055 (quaternion + accel + gyro), poll each ToF, run the
 * obstacle pipeline and dropoff check, and pack everything into `out`.
 * Returns 0 on success. Caller is expected to invoke this on a fixed
 * schedule (currently 20 Hz, SAMPLE_PERIOD_MS in main.c).
 */
int  pin_sensors_sample(struct hapnav_frame *out);

#endif /* HAPNAV_PIN_SENSORS_H_ */
