/*
 * Pin-side BLE.
 *
 * Public API (used by main.c, implemented in ble_link.c):
 *   pin_ble_init()         — start the stack and bring up every enabled link
 *   pin_ble_is_ready()     — true once at least one link can take frames
 *   pin_ble_send_frame()   — send one frame to every ready link
 *
 * Link back-ends (selected by Kconfig, any combination):
 *   CONFIG_HAPNAV_LINK_WRIST  → ble.c:        central, scans for the wristband,
 *                                             write-without-response of the frame
 *   CONFIG_HAPNAV_LINK_LAPTOP → ble_laptop.c: peripheral, advertises HapNav-Pin,
 *                                             chunked notifications to a laptop
 */
#ifndef HAPNAV_PIN_BLE_H_
#define HAPNAV_PIN_BLE_H_

#include <hapnav/ble_proto.h>
#include <stdint.h>
#include <stdbool.h>

int  pin_ble_init(void);
bool pin_ble_is_ready(void);
int  pin_ble_send_frame(const struct hapnav_frame *frame);

/* Back-ends: called by ble_link.c only, after bt_enable() has succeeded. */
int  pin_wrist_start(void);
bool pin_wrist_is_ready(void);
int  pin_wrist_send(const struct hapnav_frame *frame);

int  pin_laptop_start(void);
bool pin_laptop_is_ready(void);
int  pin_laptop_send(const struct hapnav_frame *frame);

#endif /* HAPNAV_PIN_BLE_H_ */
