/*
 * Pin-side sensor orchestration. Owns the LSM6DSO + LIS2MDL + VL53L5CX trio
 * and a Madgwick AHRS, and gives the rest of the app one call to assemble
 * a complete BLE frame.
 */
#ifndef HAPNAV_PIN_SENSORS_H_
#define HAPNAV_PIN_SENSORS_H_

#include <hapnav/ble_proto.h>

int  pin_sensors_init(void);

/*
 * Read IMU + mag, advance the orientation filter, refresh the ToF cache if a
 * new frame is ready, and pack everything into `out`. Returns 0 on success.
 * Caller is expected to invoke this on a fixed schedule (used as Madgwick dt).
 */
int  pin_sensors_sample(struct hapnav_frame *out);

#endif /* HAPNAV_PIN_SENSORS_H_ */
