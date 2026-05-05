#include "ble.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hapnav_wristband, LOG_LEVEL_INF);

int main(void)
{
	if (wrist_ble_init() != 0) {
		return -EIO;
	}
	/* Everything else happens in BT callbacks. */
	return 0;
}
