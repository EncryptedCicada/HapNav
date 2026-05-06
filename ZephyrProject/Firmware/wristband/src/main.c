#include "ble.h"

#include <hapnav/haptics.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hapnav_wristband, LOG_LEVEL_INF);

int main(void)
{
	int err = hapnav_haptics_init();
	if (err) {
		/* Boot without motors rather than dying — BLE still useful
		 * for debugging the link and inspecting incoming frames. */
		LOG_WRN("Haptics init failed (%d) — running without motors", err);
	}

	if (wrist_ble_init() != 0) {
		return -EIO;
	}
	/* Everything else happens in BT callbacks + the haptics worker. */
	return 0;
}
