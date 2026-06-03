#include "ble.h"
#include "sensors.h"

#include <hapnav/ble_proto.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hapnav_pin, LOG_LEVEL_INF);

#define SAMPLE_PERIOD_MS  12   /* ~83 Hz (4× the previous 20 Hz cadence); keep in sync with sensors.c */

/* Onboard LED acts as a hardware-level "I'm alive" indicator independent
 * of the USB-CDC console: a slow blink in steady-state means the sample
 * loop is running even if UART logs aren't reaching the host. A fast
 * blink during early init signals "still bringing sensors up". */
static const struct gpio_dt_spec led =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

static void heartbeat_tick(uint32_t loop_count, bool ble_ready)
{
	if (led.port == NULL || !device_is_ready(led.port)) {
		return;
	}
	/* ~83 Hz loop → toggle every 40 ticks ≈ 1 Hz blink when BLE up,
	 * every 16 ticks ≈ 2.5 Hz when BLE down (visible "no link yet"). */
	uint32_t period = ble_ready ? 40 : 16;
	if ((loop_count % period) == 0) {
		gpio_pin_toggle_dt(&led);
	}
}

int main(void)
{
	LOG_INF("HapNav pin: boot");

	if (led.port != NULL && device_is_ready(led.port)) {
		(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	}

	bool sensors_ok = (pin_sensors_init() == 0);
	if (!sensors_ok) {
		LOG_ERR("Sensor init failed — frame stream disabled, BLE link only");
	}

	if (pin_ble_init() != 0) {
		LOG_ERR("BLE init failed");
		return -EIO;
	}

	LOG_INF("HapNav pin: entering ~83 Hz sample loop");

	struct hapnav_frame frame;
	uint32_t loop_count = 0;
	while (1) {
		bool ble_ready = pin_ble_is_ready();

		if (sensors_ok && pin_sensors_sample(&frame) == 0) {
			if (ble_ready) {
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

		heartbeat_tick(loop_count, ble_ready);
		loop_count++;
		k_msleep(SAMPLE_PERIOD_MS);
	}
	return 0;
}
