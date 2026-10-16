/**
 * Runtime configuration of the thermometer: sampling interval, enabled sensors, measuring only while
 * connected. Defaults come from Kconfig (CONFIG_APP_*); a BLE client changes them through the Config
 * characteristic (temp_service.h). With CONFIG_SETTINGS (hardware build) they are kept in flash.
 */

#ifndef APP_CONFIG_H__
#define APP_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>

/* Sampling interval limits [ms], as in Kconfig APP_SAMPLING_INTERVAL_MS */
#define APP_CONFIG_INTERVAL_MIN_MS 100
#define APP_CONFIG_INTERVAL_MAX_MS 3600000

#define APP_CONFIG_SENSOR_COUNT 4

struct app_config {
	uint32_t interval_ms;           /* time between two measurement rounds */
	uint8_t sensor_mask;            /* bit n: sensor n+1 is powered, read and published */
	bool measure_only_connected;    /* sensors off while no client is connected */
};

/** Called after the configuration changed (from the thread that changed it). */
typedef void (*app_config_changed_cb)(const struct app_config *cfg);

/** Set the defaults from Kconfig and register the change callback. Call before settings_load(). */
void app_config_init(app_config_changed_cb cb);

/** Copy of the current configuration. */
void app_config_get(struct app_config *cfg);

/**
 * Check, apply and (with CONFIG_SETTINGS) store a new configuration.
 *
 * @return 0, or -EINVAL if a value is out of range.
 */
int app_config_set(const struct app_config *cfg);

#endif /* APP_CONFIG_H__ */
