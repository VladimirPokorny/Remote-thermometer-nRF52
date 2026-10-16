/*
 * Test client for the remote thermometer, for a second node in the emulator.
 *
 * Scans for "Remote_Thermometer", connects, discovers the temperature service, reads the "all
 * sensors" characteristic once and then prints its notifications (one per measurement round).
 * Reconnects when the link drops.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(client, LOG_LEVEL_INF);

#define THERMOMETER_NAME "Remote_Thermometer"
#define SENSOR_COUNT 4
#define VALUE_SIZE 4

#define BT_UUID_OUR_VAL(uuid16) \
	BT_UUID_128_ENCODE((uuid16), 0x5468, 0x616e, 0x6b73, 0x4d6f6d446164)

static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(BT_UUID_OUR_VAL(0xABCD));
/* All four temperatures, 4 x int32: read and notify */
static const struct bt_uuid_128 all_uuid = BT_UUID_INIT_128(BT_UUID_OUR_VAL(0x5000));

static struct bt_conn *conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_read_params read_params;
static uint16_t all_handle;

static void start_scan(void);

static void print_temperatures(const uint8_t *data, uint16_t length, const char *how)
{
	if (length != SENSOR_COUNT * VALUE_SIZE) {
		LOG_WRN("Unexpected length %u", length);
		return;
	}

	for (int i = 0; i < SENSOR_COUNT; i++) {
		uint32_t raw = sys_get_le32(data + i * VALUE_SIZE);

		if (raw == 0xEFBEADDE) {
			LOG_INF("Sensor %d: no measurement (DE AD BE EF) [%s]", i + 1, how);
			continue;
		}

		int32_t milli = (int32_t)raw;

		LOG_INF("Sensor %d: %s%d.%03d C [%s]", i + 1, milli < 0 ? "-" : "",
			abs(milli) / 1000, abs(milli) % 1000, how);
	}
}

static uint8_t notify_func(struct bt_conn *c, struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	print_temperatures(data, length, "notify");
	return BT_GATT_ITER_CONTINUE;
}

static void subscribed(struct bt_conn *c, uint8_t err, struct bt_gatt_subscribe_params *params)
{
	if (err) {
		LOG_ERR("Subscribe failed (0x%02x)", err);
	} else {
		LOG_INF("Notifications on");
	}
}

/* After the read, subscribe. The thermometer puts the CCC right after the value. */
static uint8_t read_func(struct bt_conn *c, uint8_t err, struct bt_gatt_read_params *params,
			 const void *data, uint16_t length)
{
	if (err) {
		LOG_ERR("Read failed (0x%02x)", err);
	} else if (data) {
		print_temperatures(data, length, "read");
	}

	subscribe_params = (struct bt_gatt_subscribe_params){
		.notify = notify_func,
		.subscribe = subscribed,
		.value = BT_GATT_CCC_NOTIFY,
		.value_handle = all_handle,
		.ccc_handle = all_handle + 1,
	};
	int ret = bt_gatt_subscribe(c, &subscribe_params);

	if (ret && ret != -EALREADY) {
		LOG_ERR("Subscribe failed (%d)", ret);
	}
	return BT_GATT_ITER_STOP;
}

static void read_and_subscribe(void)
{
	if (!all_handle) {
		LOG_ERR("\"All sensors\" characteristic not found (old thermometer firmware?)");
		return;
	}

	read_params = (struct bt_gatt_read_params){
		.func = read_func,
		.handle_count = 1,
		.single.handle = all_handle,
	};
	int err = bt_gatt_read(conn, &read_params);

	if (err) {
		LOG_ERR("Read failed (%d)", err);
	}
}

static uint8_t discover_func(struct bt_conn *c, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	if (!attr) {
		if (params->type == BT_GATT_DISCOVER_PRIMARY) {
			LOG_ERR("Temperature service not found");
			return BT_GATT_ITER_STOP;
		}
		LOG_INF("Discovery complete");
		read_and_subscribe();
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		const struct bt_gatt_service_val *svc = attr->user_data;

		LOG_INF("Temperature service: handles 0x%04x-0x%04x", attr->handle, svc->end_handle);

		/* Second pass: all characteristics in the service */
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->end_handle = svc->end_handle;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		int err = bt_gatt_discover(c, params);

		if (err) {
			LOG_ERR("Characteristic discovery failed (%d)", err);
		}
		return BT_GATT_ITER_STOP;
	}

	const struct bt_gatt_chrc *chrc = attr->user_data;

	if (!bt_uuid_cmp(chrc->uuid, &all_uuid.uuid)) {
		all_handle = chrc->value_handle;
		LOG_INF("All sensors: value handle 0x%04x", all_handle);
	}
	return BT_GATT_ITER_CONTINUE;
}

static void connected(struct bt_conn *c, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(c), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Connecting to %s failed (0x%02x)", addr, err);
		bt_conn_unref(conn);
		conn = NULL;
		start_scan();
		return;
	}

	LOG_INF("Connected to %s", addr);
	all_handle = 0;

	discover_params = (struct bt_gatt_discover_params){
		.uuid = &service_uuid.uuid,
		.func = discover_func,
		.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
		.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
		.type = BT_GATT_DISCOVER_PRIMARY,
	};
	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		LOG_ERR("Service discovery failed (%d)", err);
	}
}

static void disconnected(struct bt_conn *c, uint8_t reason)
{
	LOG_INF("Disconnected (0x%02x)", reason);

	if (c != conn) {
		return;
	}
	bt_conn_unref(conn);
	conn = NULL;
	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static bool ad_has_name(struct bt_data *data, void *user_data)
{
	bool *match = user_data;

	if ((data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) &&
	    data->data_len == strlen(THERMOMETER_NAME) &&
	    !memcmp(data->data, THERMOMETER_NAME, data->data_len)) {
		*match = true;
		return false;
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	bool match = false;
	char addr_str[BT_ADDR_LE_STR_LEN];

	if (conn || (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND)) {
		return;
	}

	bt_data_parse(ad, ad_has_name, &match);
	if (!match) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	LOG_INF("Found %s at %s (RSSI %d)", THERMOMETER_NAME, addr_str, rssi);

	if (bt_le_scan_stop()) {
		return;
	}

	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &conn);

	if (err) {
		LOG_ERR("Create connection failed (%d)", err);
		start_scan();
	}
}

static void start_scan(void)
{
	int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);

	if (err) {
		LOG_ERR("Scanning failed to start (%d)", err);
		return;
	}
	LOG_INF("Scanning for %s", THERMOMETER_NAME);
}

int main(void)
{
	int err = bt_enable(NULL);

	if (err) {
		LOG_ERR("Bluetooth init failed (%d)", err);
		return 0;
	}

	start_scan();
	return 0;
}
