/*
 * Pin-side BLE, laptop link (CONFIG_HAPNAV_LINK_LAPTOP=y).
 *
 * Can run alone or alongside the wristband link (src/ble.c), so connection
 * callbacks only act on peripheral-role connections. The
 * pin advertises as a connectable peripheral exposing one notify
 * characteristic. Once a laptop connects and enables notifications, every
 * hapnav_frame is pushed as one or more chunked notifications (chunk header
 * format documented in hapnav/ble_proto.h). Receiver: Tools/ble_receiver.py.
 */
#include "ble.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <string.h>

LOG_MODULE_REGISTER(pin_ble_laptop, LOG_LEVEL_INF);

#define CHUNK_HDR_LEN      1
#define ATT_NOTIFY_OVERHEAD 3   /* opcode + handle */
#define MAX_ATT_PAYLOAD    (CONFIG_BT_L2CAP_TX_MTU - ATT_NOTIFY_OVERHEAD)

/* Ask for a 7.5–15 ms interval so ~83 Hz frames fit; the laptop's OS has the
 * final say (macOS typically settles on 15 ms or 30 ms). */
#define LAPTOP_CONN_PARAM  BT_LE_CONN_PARAM(6, 12, 0, 400)

static struct bt_uuid_128 svc_uuid   = BT_UUID_INIT_128(HAPNAV_UUID_LAPTOP_SVC_VAL);
static struct bt_uuid_128 frame_uuid = BT_UUID_INIT_128(HAPNAV_UUID_LAPTOP_FRAME_VAL);

static struct bt_conn *current_conn;      /* guarded by conn_lock */
static struct k_spinlock conn_lock;
static atomic_t notify_enabled;

/* ── GATT service ─────────────────────────────────────────────────────────── */

static void frame_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	bool on = (value == BT_GATT_CCC_NOTIFY);

	atomic_set(&notify_enabled, on);
	LOG_INF("Laptop %s frame notifications", on ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(laptop_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),
	BT_GATT_CHARACTERISTIC(&frame_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(frame_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* attrs[1] is the characteristic declaration; bt_gatt_notify() resolves it
 * to the value attribute (same convention as Zephyr's own samples). */
#define FRAME_ATTR (&laptop_svc.attrs[1])

/* True for connections where we are peripheral, i.e. the laptop link. The
 * wristband link (central role) gets the same callbacks and must be ignored. */
static bool is_laptop_conn(struct bt_conn *conn)
{
	struct bt_conn_info info;

	return bt_conn_get_info(conn, &info) == 0 &&
	       info.role == BT_CONN_ROLE_PERIPHERAL;
}

/* ── advertising ──────────────────────────────────────────────────────────── */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, HAPNAV_UUID_LAPTOP_SVC_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Zephyr 4.x renamed BT_LE_ADV_CONN → BT_LE_ADV_CONN_FAST_2 (same 100–150 ms
 * interval) and dropped auto-resume of advertising after a connection;
 * we restart explicitly from the `recycled` callback below either way. */
#ifdef BT_LE_ADV_CONN_FAST_2
#define LAPTOP_ADV_PARAM BT_LE_ADV_CONN_FAST_2
#else
#define LAPTOP_ADV_PARAM BT_LE_ADV_CONN
#endif

static void adv_start(void)
{
	if (current_conn) {
		return;   /* one laptop at a time; restart after it disconnects */
	}
	int err = bt_le_adv_start(LAPTOP_ADV_PARAM, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err == -EALREADY) {
		return;   /* stack already resumed advertising on its own */
	}
	if (err) {
		LOG_ERR("Advertising start failed: %d", err);
		return;
	}
	LOG_INF("Advertising as \"%s\", waiting for laptop", CONFIG_BT_DEVICE_NAME);
}

/* Restart from the system workqueue, not from inside the BT callback. */
static void adv_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	adv_start();
}
static K_WORK_DEFINE(adv_work, adv_work_handler);

/* ── connection callbacks ─────────────────────────────────────────────────── */

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (!is_laptop_conn(conn)) {
		return;
	}
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err) {
		LOG_ERR("Laptop connect from %s failed (0x%02x)", addr, err);
		k_work_submit(&adv_work);
		return;
	}
	LOG_INF("Laptop connected: %s (ATT MTU %u)", addr, bt_gatt_get_mtu(conn));

	k_spinlock_key_t key = k_spin_lock(&conn_lock);
	current_conn = bt_conn_ref(conn);
	k_spin_unlock(&conn_lock, key);

	int perr = bt_conn_le_param_update(conn, LAPTOP_CONN_PARAM);
	if (perr) {
		LOG_WRN("Conn param update request failed: %d", perr);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != current_conn) {
		return;   /* wristband link, or a stale conn — not ours */
	}
	LOG_INF("Laptop disconnected (reason 0x%02x)", reason);

	atomic_set(&notify_enabled, false);

	k_spinlock_key_t key = k_spin_lock(&conn_lock);
	struct bt_conn *old = current_conn;
	current_conn = NULL;
	k_spin_unlock(&conn_lock, key);

	if (old) {
		bt_conn_unref(old);
	}
	/* Advertising restarts from recycled(): the conn object isn't free yet
	 * here, so bt_le_adv_start() could fail for lack of a conn slot. */
}

/* Fires when *any* conn object is freed (laptop or wristband); adv_start()
 * is a no-op while a laptop is still connected. */
static void recycled(void)
{
	k_work_submit(&adv_work);
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	if (!is_laptop_conn(conn)) {
		return;
	}
	LOG_INF("Laptop conn params: interval %u.%02u ms, latency %u, timeout %u ms",
		interval * 125 / 100, (interval * 125) % 100, latency, timeout * 10);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled,
	.le_param_updated = le_param_updated,
};

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	if (!is_laptop_conn(conn)) {
		return;
	}
	LOG_INF("Laptop ATT MTU updated: tx=%u rx=%u", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

/* ── back-end API (called from ble_link.c) ──────────────────────────────────── */

int pin_laptop_start(void)
{
	bt_gatt_cb_register(&gatt_callbacks);
	adv_start();
	return 0;
}

bool pin_laptop_is_ready(void)
{
	return current_conn != NULL && atomic_get(&notify_enabled);
}

int pin_laptop_send(const struct hapnav_frame *frame)
{
	static uint8_t seq;
	uint8_t buf[CHUNK_HDR_LEN + MAX_ATT_PAYLOAD];

	if (!atomic_get(&notify_enabled)) {
		return -ENOTCONN;
	}

	k_spinlock_key_t key = k_spin_lock(&conn_lock);
	struct bt_conn *conn = current_conn ? bt_conn_ref(current_conn) : NULL;
	k_spin_unlock(&conn_lock, key);
	if (!conn) {
		return -ENOTCONN;
	}

	size_t max_chunk = bt_gatt_get_mtu(conn) - ATT_NOTIFY_OVERHEAD - CHUNK_HDR_LEN;
	max_chunk = MIN(max_chunk, sizeof(buf) - CHUNK_HDR_LEN);

	const uint8_t *src = (const uint8_t *)frame;
	size_t off = 0;
	uint8_t idx = 0;
	int err = 0;

	while (off < sizeof(*frame)) {
		size_t n = MIN(max_chunk, sizeof(*frame) - off);

		buf[0] = (uint8_t)(((seq & 0x0F) << 4) | (idx & 0x0F));
		memcpy(&buf[CHUNK_HDR_LEN], src + off, n);

		err = bt_gatt_notify(conn, FRAME_ATTR, buf, CHUNK_HDR_LEN + n);
		if (err) {
			break;   /* receiver drops the partial frame on next idx 0 */
		}
		off += n;
		idx++;
	}

	seq++;
	bt_conn_unref(conn);
	return err;
}
