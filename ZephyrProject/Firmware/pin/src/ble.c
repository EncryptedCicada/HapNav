#include "ble.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>

#include <string.h>

LOG_MODULE_REGISTER(pin_ble, LOG_LEVEL_INF);

static struct bt_uuid_128 svc_uuid   = BT_UUID_INIT_128(HAPNAV_UUID_SVC_VAL);
static struct bt_uuid_128 frame_uuid = BT_UUID_INIT_128(HAPNAV_UUID_FRAME_VAL);

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_exchange_params mtu_xchg_params;
static uint16_t frame_handle;

static void start_scan(void);

/* ── MTU exchange ─────────────────────────────────────────────────────────── */

static void mtu_exchanged(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);
	if (err) {
		LOG_WRN("MTU exchange failed (0x%02x)", err);
	} else {
		LOG_INF("MTU exchanged: ATT MTU = %u", bt_gatt_get_mtu(conn));
	}
}

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	ARG_UNUSED(conn);
	LOG_INF("ATT MTU updated: tx=%u rx=%u", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

/* ── discovery: PRIMARY service → IMU characteristic ──────────────────────── */

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	if (!attr) {
		LOG_INF("Discovery exhausted (frame=%u)", frame_handle);
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		discover_params.uuid         = &frame_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;
		(void)bt_gatt_discover(conn, &discover_params);
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		const struct bt_gatt_chrc *chrc = attr->user_data;
		frame_handle = chrc->value_handle;
		LOG_INF("Frame char handle = %u", frame_handle);
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static void start_discovery(struct bt_conn *conn)
{
	discover_params.uuid         = &svc_uuid.uuid;
	discover_params.func         = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

	int err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		LOG_ERR("Service discovery failed: %d", err);
	}
}

/* ── scan / connect ───────────────────────────────────────────────────────── */

static bool ad_match(struct bt_data *data, void *user_data)
{
	bool *found = user_data;

	if (data->type != BT_DATA_UUID128_ALL && data->type != BT_DATA_UUID128_SOME) {
		return true;
	}
	if (data->data_len % 16 != 0) {
		return true;
	}
	for (int i = 0; i < data->data_len; i += 16) {
		struct bt_uuid_128 u = { .uuid.type = BT_UUID_TYPE_128 };
		memcpy(u.val, &data->data[i], 16);
		if (bt_uuid_cmp(&u.uuid, &svc_uuid.uuid) == 0) {
			*found = true;
			return false;
		}
	}
	return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *ad)
{
	if (default_conn) {
		return;
	}
	if (adv_type != BT_GAP_ADV_TYPE_ADV_IND &&
	    adv_type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bool found = false;
	bt_data_parse(ad, ad_match, &found);
	if (!found) {
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	LOG_INF("Wristband seen at %s (RSSI %d), connecting", addr_str, rssi);

	if (bt_le_scan_stop()) {
		LOG_WRN("Scan stop failed");
	}

	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				    BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		LOG_ERR("Create conn failed: %d", err);
		start_scan();
	}
}

static void start_scan(void)
{
	int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
	if (err) {
		LOG_ERR("Scan start failed: %d", err);
		return;
	}
	LOG_INF("Scanning for HapNav-Wrist");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connect failed (0x%02x)", err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		start_scan();
		return;
	}
	LOG_INF("Connected");

	mtu_xchg_params.func = mtu_exchanged;
	int merr = bt_gatt_exchange_mtu(conn, &mtu_xchg_params);
	if (merr) {
		LOG_WRN("MTU exchange request failed: %d", merr);
	}

	start_discovery(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)", reason);
	bt_conn_unref(default_conn);
	default_conn = NULL;
	frame_handle = 0;
	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

/* ── public API ───────────────────────────────────────────────────────────── */

int pin_ble_init(void)
{
	bt_gatt_cb_register(&gatt_callbacks);

	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return err;
	}
	LOG_INF("Bluetooth initialised");
	start_scan();
	return 0;
}

bool pin_ble_is_ready(void)
{
	if (!default_conn || !frame_handle) {
		return false;
	}
	return bt_gatt_get_mtu(default_conn) >=
	       sizeof(struct hapnav_frame) + 3;
}

int pin_ble_send_frame(const struct hapnav_frame *frame)
{
	if (!default_conn || !frame_handle) {
		return -ENOTCONN;
	}
	return bt_gatt_write_without_response(default_conn, frame_handle,
					      frame, sizeof(*frame), false);
}
