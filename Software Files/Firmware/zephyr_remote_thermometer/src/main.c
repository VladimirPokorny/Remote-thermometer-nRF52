/**
 * Copyright (c) Vladimir Pokorny
 *    email:  pokorny.v@email.cz
 *    web:    www.vladimirpokorny.cz
 *
 *    https://www.vutbr.cz/studenti/zav-prace/detail/142726
 *
 ****************************************************************************************
 * BLE application for reading temperatures from RTD sensors by MAX31865.
 * It reads temperatures from up to 4 sensors and sends them by BLE. Each sensor can be
 * enabled or disabled. More about this work is written in the final Master thesis:
 *
 * https://www.vut.cz/studenti/zav-prace/detail/142726
 *
 * Zephyr / nRF Connect SDK port of the nRF5 SDK 17.1.0 + S140 firmware
 * (Software Files/Firmware/ble_remote_thermometer). Same BLE service and UUIDs, see
 * temp_service.h. Sensor wiring (pins, R0, reference resistors, 2/3/4-wire) is in the
 * board devicetree overlays in boards/.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "max31865.h"
#include "temp_service.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/***************************************************************************************************************/
/*  Sampling interval and enabled sensors: CONFIG_APP_* in prj.conf / boards/<board>.conf (see Kconfig).       */
/*  The characteristics of the RTD sensors and the reference resistors are in the board overlay (boards/).     */
/*  More about this work, functions and variables is written in the final Master thesis:                       */
/*  https://www.vut.cz/studenti/zav-prace/detail/142726                                                        */
/***************************************************************************************************************/

#define APP_ADV_INTERVAL    160             // The advertising interval (in units of 0.625 ms, 100 ms)

#define SENSOR_COUNT        TEMP_SERVICE_SENSOR_COUNT
#define ZEPHYR_USER         DT_PATH(zephyr_user)

/* One measuring channel: MAX31865 chip, the load switch powering it and its indicator LED */
struct rtd_channel {
	struct max31865 max;
	struct gpio_dt_spec enable;
	struct gpio_dt_spec led;        /* .port NULL: no LED */
	bool used;
};

/* LED n mirrors the load switch of sensor n (CONFIG_APP_SENSOR_LEDS) */
#define SENSOR_LED(alias)                                                                        \
	COND_CODE_1(CONFIG_APP_SENSOR_LEDS, (GPIO_DT_SPEC_GET_OR(DT_ALIAS(alias), gpios, {0})), ({0}))

#define RTD_CHANNEL(label, idx, led_alias, use)                                                  \
	{                                                                                        \
		.max = MAX31865_DT_SPEC_GET(DT_NODELABEL(label)),                                \
		.enable = GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER, rtd_enable_gpios, idx),           \
		.led = SENSOR_LED(led_alias),                                                    \
		.used = (use),                                                                   \
	}

static const struct rtd_channel channels[SENSOR_COUNT] = {
	RTD_CHANNEL(rtd1, 0, led0, IS_ENABLED(CONFIG_APP_SENSOR_1)),
	RTD_CHANNEL(rtd2, 1, led1, IS_ENABLED(CONFIG_APP_SENSOR_2)),
	RTD_CHANNEL(rtd3, 2, led2, IS_ENABLED(CONFIG_APP_SENSOR_3)),
	RTD_CHANNEL(rtd4, 3, led3, IS_ENABLED(CONFIG_APP_SENSOR_4)),
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Advertising: every 100 ms for CONFIG_APP_ADV_FAST_TIMEOUT_S after boot or a disconnect, so a
 * client finds the thermometer quickly, then every CONFIG_APP_ADV_SLOW_INTERVAL_MS to save power. */
#define APP_ADV_SLOW_INTERVAL   (CONFIG_APP_ADV_SLOW_INTERVAL_MS * 8 / 5)   // in units of 0.625 ms

static atomic_t is_connected;

/* Given when a client connects: the measurement thread starts a round at once */
static K_SEM_DEFINE(measure_now, 0, 1);

static int advertise(uint32_t interval)
{
	return bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, interval, interval, NULL),
			       ad, ARRAY_SIZE(ad), NULL, 0);
}

static void advertising_slow(struct k_work *work)
{
	if (atomic_get(&is_connected)) {
		return;
	}
	bt_le_adv_stop();

	int err = advertise(APP_ADV_SLOW_INTERVAL);

	if (err) {
		LOG_ERR("Slow advertising failed to start (%d)", err);
		return;
	}
	LOG_INF("Slow advertising (every %d ms).", CONFIG_APP_ADV_SLOW_INTERVAL_MS);
}

static K_WORK_DELAYABLE_DEFINE(adv_slow_work, advertising_slow);

static void advertising_start(struct k_work *work)
{
	if (CONFIG_APP_ADV_FAST_TIMEOUT_S == 0) {
		advertising_slow(NULL);
		return;
	}

	int err = advertise(APP_ADV_INTERVAL);

	if (err) {
		LOG_ERR("Advertising failed to start (%d)", err);
		return;
	}
	LOG_INF("Fast advertising (%d s).", CONFIG_APP_ADV_FAST_TIMEOUT_S);
	k_work_reschedule(&adv_slow_work, K_SECONDS(CONFIG_APP_ADV_FAST_TIMEOUT_S));
}

static K_WORK_DEFINE(advertising_work, advertising_start);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("Connection failed (0x%02x)", err);
		return;
	}
	atomic_set(&is_connected, 1);
	k_work_cancel_delayable(&adv_slow_work);
	LOG_INF("Connected.");

	if (IS_ENABLED(CONFIG_APP_MEASURE_ONLY_CONNECTED)) {
		k_sem_give(&measure_now);       /* first values right away */
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	atomic_set(&is_connected, 0);
	LOG_INF("Disconnected (0x%02x).", reason);
}

/* The client accepted (or chose) new connection parameters, see CONFIG_BT_PERIPHERAL_PREF_* */
static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout)
{
	LOG_INF("Connection parameters: interval %u.%02u ms, peripheral latency %u, timeout %u ms",
		interval * 125 / 100, interval * 125 % 100, latency, timeout * 10);
}

/* The connection object is free again: advertise for the next client */
static void recycled(void)
{
	k_work_submit(&advertising_work);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_WRN("Pairing failed (%d), disconnecting.", err);
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		return;
	}
	LOG_INF("Security level %d.", level);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
	.recycled = recycled,
	.security_changed = security_changed,
};

static int gpio_init(void)
{
	int err = 0;

	for (int i = 0; i < SENSOR_COUNT && !err; i++) {
		const struct rtd_channel *ch = &channels[i];

		if (!gpio_is_ready_dt(&ch->enable) || (ch->led.port && !gpio_is_ready_dt(&ch->led))) {
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&ch->enable, GPIO_OUTPUT_INACTIVE);
		if (!err && ch->led.port) {
			err = gpio_pin_configure_dt(&ch->led, GPIO_OUTPUT_INACTIVE);
		}
	}
	return err;
}

/* Load switch of the channel and its LED */
static void channel_power(const struct rtd_channel *ch, int on)
{
	gpio_pin_set_dt(&ch->enable, on);
	if (ch->led.port) {
		gpio_pin_set_dt(&ch->led, on);
	}
}

/* Power the channel, read the temperature, power it down again. The load switch powers the whole
 * channel (MAX31865, bus buffer, status LED), so the channel draws nothing between measurements. */
static int measure(const struct rtd_channel *ch, float *temperature)
{
	channel_power(ch, 1);
	k_msleep(2);

	/* The MAX31865 starts from its reset configuration after every power-up: configure it and
	 * convert in 3 SPI transfers */
	int err = readTemperatureAfterPowerUp(&ch->max, temperature);

	channel_power(ch, 0);
	return err;
}

/* Measurement thread: every CONFIG_APP_SAMPLING_INTERVAL_MS read all enabled sensors, then publish
 * them over BLE in one notification. Replaces the app_timer flag polled in the nRF5 SDK main loop.
 * Between rounds the thread sleeps and the CPU idles in System ON sleep (tickless kernel).
 * With CONFIG_APP_MEASURE_ONLY_CONNECTED the sensors stay off while no client is connected. */
static void measurement_thread(void *p1, void *p2, void *p3)
{
	int64_t next = k_uptime_get();

	for (;;) {
		if (IS_ENABLED(CONFIG_APP_MEASURE_ONLY_CONNECTED) && !atomic_get(&is_connected)) {
			LOG_INF("No client connected: sensors off.");
			k_sem_take(&measure_now, K_FOREVER);
			next = k_uptime_get();
			continue;       /* the client may have left again meanwhile */
		}

		for (int i = 0; i < SENSOR_COUNT; i++) {
			float temperature;

			if (!channels[i].used) {
				continue;
			}

			int err = measure(&channels[i], &temperature);

			if (err) {
				LOG_ERR("Sensor %d: SPI error %d", i + 1, err);
				continue;
			}

			int32_t milli = (int32_t)(temperature * 1000);

			LOG_INF("Sensor %d: %s%d.%03d C", i + 1, milli < 0 ? "-" : "",
				abs(milli) / 1000, abs(milli) % 1000);
			temp_service_update(i, milli);
		}

		/* One BLE packet for the whole round */
		temp_service_publish();

		/* Fixed period from round start to round start. A round longer than the interval
		 * starts the next one at once instead of running a backlog of late rounds. */
		next += CONFIG_APP_SAMPLING_INTERVAL_MS;
		if (next < k_uptime_get()) {
			next = k_uptime_get();
		}

		/* Sleep until the next round, or until a client connects: then measure at once */
		if (k_sem_take(&measure_now, K_TIMEOUT_ABS_MS(next)) == 0) {
			next = k_uptime_get();
		}
	}
}

K_THREAD_DEFINE(measurement_tid, 2048, measurement_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(7), K_FP_REGS, -1);

int main(void)
{
	int err;

	err = gpio_init();
	if (err) {
		LOG_ERR("GPIO init failed (%d)", err);
		return 0;
	}

	// The MAX31865 chips are unpowered until measured: measure() configures them after power-up
	for (int i = 0; i < SENSOR_COUNT; i++) {
		if (channels[i].used && !spi_is_ready_dt(&channels[i].max.spi)) {
			LOG_ERR("SPI bus of sensor %d is not ready", i + 1);
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (%d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();            // bonds
	}

	k_work_submit(&advertising_work);

	// Start execution.
	LOG_INF("Measuring temperature started: every %d ms, sensors%s%s%s%s.",
		CONFIG_APP_SAMPLING_INTERVAL_MS,
		channels[0].used ? " 1" : "", channels[1].used ? " 2" : "",
		channels[2].used ? " 3" : "", channels[3].used ? " 4" : "");
	k_thread_name_set(measurement_tid, "measurement");
	k_thread_start(measurement_tid);

	return 0;
}
