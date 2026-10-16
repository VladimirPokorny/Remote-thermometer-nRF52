#include "temp_service.h"
#include "app_config.h"

#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(temp_service, LOG_LEVEL_INF);

/* 128-bit base 64 61 44 6D 6F 4D 73 6B 6E 61 68 54 xx xx 00 00 (little endian), with the
 * 16-bit service/characteristic UUID in bytes 12-13, as sd_ble_uuid_vs_add() built them. */
#define BT_UUID_OUR_VAL(uuid16) \
	BT_UUID_128_ENCODE((uuid16), 0x5468, 0x616e, 0x6b73, 0x4d6f6d446164)

#define BT_UUID_OUR_SERVICE BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0xABCD))
#define BT_UUID_TEMPERATURE_1 BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0x1000))
#define BT_UUID_TEMPERATURE_2 BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0x2000))
#define BT_UUID_TEMPERATURE_3 BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0x3000))
#define BT_UUID_TEMPERATURE_4 BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0x4000))
#define BT_UUID_TEMPERATURE_ALL BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0x5000))
#define BT_UUID_CONFIG BT_UUID_DECLARE_128(BT_UUID_OUR_VAL(0x6000))

/* Config characteristic layout, little endian (see temp_service.h) */
#define CONFIG_VERSION 1
#define CONFIG_SIZE 8
#define CONFIG_FLAG_MEASURE_ONLY_CONNECTED BIT(0)

#define VALUE_SIZE 4

/* All four values in a row: values[i] is sensor i's characteristic, the whole array is "All" */
static uint8_t values[TEMP_SERVICE_SENSOR_COUNT][VALUE_SIZE] = {
	{0xDE, 0xAD, 0xBE, 0xEF},
	{0xDE, 0xAD, 0xBE, 0xEF},
	{0xDE, 0xAD, 0xBE, 0xEF},
	{0xDE, 0xAD, 0xBE, 0xEF},
};

/* Measurement thread writes, BT RX thread reads */
static K_MUTEX_DEFINE(values_lock);

static ssize_t read_copy(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset, size_t size)
{
	uint8_t copy[sizeof(values)];

	k_mutex_lock(&values_lock, K_FOREVER);
	memcpy(copy, attr->user_data, size);
	k_mutex_unlock(&values_lock);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, copy, size);
}

/* One sensor: user_data points to its 4 bytes */
static ssize_t read_sensor(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	return read_copy(conn, attr, buf, len, offset, VALUE_SIZE);
}

/* All sensors: user_data points to the whole array */
static ssize_t read_all(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			uint16_t len, uint16_t offset)
{
	return read_copy(conn, attr, buf, len, offset, sizeof(values));
}

static ssize_t read_config(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	struct app_config cfg;
	uint8_t value[CONFIG_SIZE] = {
		CONFIG_VERSION,
	};

	app_config_get(&cfg);
	value[1] = cfg.sensor_mask;
	value[2] = cfg.measure_only_connected ? CONFIG_FLAG_MEASURE_ONLY_CONNECTED : 0;
	sys_put_le32(cfg.interval_ms, &value[4]);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

/* A client writes the whole configuration at once; out-of-range values are rejected */
static ssize_t write_config(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *value = buf;

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != CONFIG_SIZE) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	if (value[0] != CONFIG_VERSION) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	struct app_config cfg = {
		.sensor_mask = value[1],
		.measure_only_connected = value[2] & CONFIG_FLAG_MEASURE_ONLY_CONNECTED,
		.interval_ms = sys_get_le32(&value[4]),
	};

	if (app_config_set(&cfg)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	return len;
}

#define TEMPERATURE_CHARACTERISTIC(uuid, idx)                                                     \
	BT_GATT_CHARACTERISTIC(uuid, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_sensor, NULL,      \
			       values[idx])

BT_GATT_SERVICE_DEFINE(our_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_OUR_SERVICE),
	TEMPERATURE_CHARACTERISTIC(BT_UUID_TEMPERATURE_1, 0),
	TEMPERATURE_CHARACTERISTIC(BT_UUID_TEMPERATURE_2, 1),
	TEMPERATURE_CHARACTERISTIC(BT_UUID_TEMPERATURE_3, 2),
	TEMPERATURE_CHARACTERISTIC(BT_UUID_TEMPERATURE_4, 3),
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE_ALL, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_all, NULL, values),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_CONFIG, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_config, write_config, NULL),
);

/* Attribute layout: [0] service, [1..8] sensors 1-4 (declaration, value), [9] all: declaration,
 * [10] all: value, [11] CCC, [12..13] config (declaration, value) */
#define ALL_VALUE_ATTR (&our_service.attrs[10])

void temp_service_update(uint8_t sensor, int32_t milli_celsius)
{
	if (sensor >= TEMP_SERVICE_SENSOR_COUNT) {
		return;
	}

	k_mutex_lock(&values_lock, K_FOREVER);
	sys_put_le32((uint32_t)milli_celsius, values[sensor]);
	k_mutex_unlock(&values_lock);
}

void temp_service_clear(uint8_t sensor)
{
	if (sensor >= TEMP_SERVICE_SENSOR_COUNT) {
		return;
	}

	k_mutex_lock(&values_lock, K_FOREVER);
	memcpy(values[sensor], (uint8_t[]){0xDE, 0xAD, 0xBE, 0xEF}, VALUE_SIZE);
	k_mutex_unlock(&values_lock);
}

void temp_service_publish(void)
{
	uint8_t copy[sizeof(values)];

	k_mutex_lock(&values_lock, K_FOREVER);
	memcpy(copy, values, sizeof(copy));
	k_mutex_unlock(&values_lock);

	/* NULL connection: every connected client that enabled notifications */
	int err = bt_gatt_notify(NULL, ALL_VALUE_ATTR, copy, sizeof(copy));

	if (err && err != -ENOTCONN) {
		LOG_WRN("Notification failed (%d)", err);
	}
}
