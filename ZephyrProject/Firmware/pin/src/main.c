#include "ble.h"
#include "sensors.h"

#include <hapnav/ble_proto.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hapnav_pin, LOG_LEVEL_INF);

#define SAMPLE_PERIOD_MS  100  /* 10 Hz; keep in sync with sensors.c */

int main(void)
{
	bool sensors_ok = (pin_sensors_init() == 0);
	if (!sensors_ok) {
		LOG_ERR("Sensor init failed — frame stream disabled, BLE link only");
	}

	if (pin_ble_init() != 0) {
		return -EIO;
	}

	struct hapnav_frame frame;
	while (1) {
		if (sensors_ok && pin_sensors_sample(&frame) == 0) {
			if (pin_ble_is_ready()) {
				int rc = pin_ble_send_frame(&frame);
				if (rc) {
					static uint32_t last_err_log_ms;
					uint32_t now = k_uptime_get_32();
					if (now - last_err_log_ms > 1000) {
						LOG_WRN("BLE send failed: %d", rc);
						last_err_log_ms = now;
					}
				}
			}
		}
		k_msleep(SAMPLE_PERIOD_MS);
	}
	return 0;
}
