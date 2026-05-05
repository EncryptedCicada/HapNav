/*
 * Pin-side BLE (central role).
 *
 *   pin_ble_init()         — start the stack and begin scanning
 *   pin_ble_is_ready()     — true once connected, frame char discovered, MTU OK
 *   pin_ble_send_frame()   — write 212-byte frame (write-without-response)
 */
#ifndef HAPNAV_PIN_BLE_H_
#define HAPNAV_PIN_BLE_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>
#include <stdbool.h>

int  pin_ble_init(void);
bool pin_ble_is_ready(void);
int  pin_ble_send_frame(const struct hapnav_frame *frame);

#endif /* HAPNAV_PIN_BLE_H_ */
