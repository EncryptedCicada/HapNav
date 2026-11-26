/*
 * Pin-side BLE front-end: owns the pin_ble_* API used by main.c and fans each
 * frame out to whichever links are enabled (wristband, laptop, or both).
 * The links are independent: one being down never blocks the other.
 */
#include "ble.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>

LOG_MODULE_REGISTER(pin_ble_link, LOG_LEVEL_INF);

#if !defined(CONFIG_HAPNAV_LINK_WRIST) && !defined(CONFIG_HAPNAV_LINK_LAPTOP)
#error "Enable at least one of CONFIG_HAPNAV_LINK_WRIST / CONFIG_HAPNAV_LINK_LAPTOP"
#endif

int pin_ble_init(void)
{
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return err;
	}
	LOG_INF("Bluetooth initialised (links:%s%s)",
		IS_ENABLED(CONFIG_HAPNAV_LINK_WRIST) ? " wristband" : "",
		IS_ENABLED(CONFIG_HAPNAV_LINK_LAPTOP) ? " laptop" : "");

	int rc = 0;
#if defined(CONFIG_HAPNAV_LINK_WRIST)
	err = pin_wrist_start();
	if (err) {
		LOG_ERR("Wristband link start failed: %d", err);
		rc = err;
	}
#endif
#if defined(CONFIG_HAPNAV_LINK_LAPTOP)
	err = pin_laptop_start();
	if (err) {
		LOG_ERR("Laptop link start failed: %d", err);
		rc = rc ? rc : err;
	}
#endif
	return rc;
}

bool pin_ble_is_ready(void)
{
	bool ready = false;
#if defined(CONFIG_HAPNAV_LINK_WRIST)
	ready = ready || pin_wrist_is_ready();
#endif
#if defined(CONFIG_HAPNAV_LINK_LAPTOP)
	ready = ready || pin_laptop_is_ready();
#endif
	return ready;
}

/* Returns 0 if at least one ready link accepted the frame, otherwise the
 * first error seen (or -ENOTCONN if no link was ready). */
int pin_ble_send_frame(const struct hapnav_frame *frame)
{
	int first_err = -ENOTCONN;
	bool any_ok = false;

#if defined(CONFIG_HAPNAV_LINK_WRIST)
	if (pin_wrist_is_ready()) {
		int err = pin_wrist_send(frame);
		if (err == 0) {
			any_ok = true;
		} else if (first_err == -ENOTCONN) {
			first_err = err;
		}
	}
#endif
#if defined(CONFIG_HAPNAV_LINK_LAPTOP)
	if (pin_laptop_is_ready()) {
		int err = pin_laptop_send(frame);
		if (err == 0) {
			any_ok = true;
		} else if (first_err == -ENOTCONN) {
			first_err = err;
		}
	}
#endif
	return any_ok ? 0 : first_err;
}
