#include "ble.h"

#include <hapnav/ble_proto.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>

#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(wrist_ble, LOG_LEVEL_INF);

static struct bt_uuid_128 svc_uuid   = BT_UUID_INIT_128(HAPNAV_UUID_SVC_VAL);
static struct bt_uuid_128 frame_uuid = BT_UUID_INIT_128(HAPNAV_UUID_FRAME_VAL);

/*
 * Emit one frame as a single-line JSON object on the console. printk goes
 * straight to UART (no Zephyr LOG decoration), so the resulting stream is
 * trivial to parse downstream.
 */
static void print_frame_json(const struct hapnav_frame *f)
{
	printk("{\"distances\":[");
	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		printk("%s%d", (i ? "," : ""), f->distances_mm[i]);
	}
	printk("],\"status\":[");
	for (int i = 0; i < HAPNAV_TOF_ZONES; i++) {
		printk("%s%u", (i ? "," : ""), f->target_status[i]);
	}
	printk("],\"quat\":[%.4f,%.4f,%.4f,%.4f]}\n",
	       (double)f->quat[0], (double)f->quat[1],
	       (double)f->quat[2], (double)f->quat[3]);
}

static ssize_t frame_write(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len,
			   uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0 || len != HAPNAV_FRAME_SIZE) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	struct hapnav_frame f;
	memcpy(&f, buf, sizeof(f));
	print_frame_json(&f);
	return len;
}

BT_GATT_SERVICE_DEFINE(hn_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),

	BT_GATT_CHARACTERISTIC(&frame_uuid.uuid,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, frame_write, NULL),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, HAPNAV_UUID_SVC_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "HapNav-Wrist", sizeof("HapNav-Wrist") - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	ARG_UNUSED(conn);
	if (err) {
		LOG_ERR("Connection failed (0x%02x)", err);
	} else {
		LOG_INF("Connected to central");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("Disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

int wrist_ble_init(void)
{
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return err;
	}
	LOG_INF("Bluetooth initialised");

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			      ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed: %d", err);
		return err;
	}
	LOG_INF("Advertising as HapNav-Wrist");
	return 0;
}
