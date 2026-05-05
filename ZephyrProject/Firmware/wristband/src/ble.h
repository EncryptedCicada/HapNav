/*
 * Wristband-side BLE (peripheral role).
 *
 *   wrist_ble_init() — start the stack, register GATT, advertise.
 *
 * After init, all work happens in BT callbacks: incoming IMU samples are
 * unpacked, converted to engineering units, and logged with a one-way
 * latency estimate.
 */
#ifndef HAPNAV_WRIST_BLE_H_
#define HAPNAV_WRIST_BLE_H_

int wrist_ble_init(void);

#endif /* HAPNAV_WRIST_BLE_H_ */
