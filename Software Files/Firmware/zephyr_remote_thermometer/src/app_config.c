#include "app_config.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(app_config, LOG_LEVEL_INF);

/* Stored in flash as "thermo/cfg" (CONFIG_SETTINGS). Version byte first, so a later firmware can
 * convert an older layout instead of misreading it. */
#define STORED_VERSION 1

struct stored_config {
	uint8_t version;
	uint8_t sensor_mask;
	uint8_t measure_only_connected;
	uint8_t reserved;
	uint32_t interval_ms;
} __packed;

static struct app_config config;
static K_MUTEX_DEFINE(config_lock);
static app_config_changed_cb changed_cb;

static bool valid(const struct app_config *cfg)
{
	return cfg->interval_ms >= APP_CONFIG_INTERVAL_MIN_MS &&
	       cfg->interval_ms <= APP_CONFIG_INTERVAL_MAX_MS &&
	       (cfg->sensor_mask & ~BIT_MASK(APP_CONFIG_SENSOR_COUNT)) == 0;
}

void app_config_init(app_config_changed_cb cb)
{
	changed_cb = cb;
	config = (struct app_config){
		.interval_ms = CONFIG_APP_SAMPLING_INTERVAL_MS,
		.sensor_mask = (IS_ENABLED(CONFIG_APP_SENSOR_1) ? BIT(0) : 0) |
			       (IS_ENABLED(CONFIG_APP_SENSOR_2) ? BIT(1) : 0) |
			       (IS_ENABLED(CONFIG_APP_SENSOR_3) ? BIT(2) : 0) |
			       (IS_ENABLED(CONFIG_APP_SENSOR_4) ? BIT(3) : 0),
		.measure_only_connected = IS_ENABLED(CONFIG_APP_MEASURE_ONLY_CONNECTED),
	};
}

void app_config_get(struct app_config *cfg)
{
	k_mutex_lock(&config_lock, K_FOREVER);
	*cfg = config;
	k_mutex_unlock(&config_lock);
}

int app_config_set(const struct app_config *cfg)
{
	if (!valid(cfg)) {
		return -EINVAL;
	}

	k_mutex_lock(&config_lock, K_FOREVER);
	config = *cfg;
	k_mutex_unlock(&config_lock);

	LOG_INF("Configuration: every %u ms, sensors 0x%x, measure only when connected: %s",
		cfg->interval_ms, cfg->sensor_mask, cfg->measure_only_connected ? "yes" : "no");

#if defined(CONFIG_SETTINGS)
	struct stored_config stored = {
		.version = STORED_VERSION,
		.sensor_mask = cfg->sensor_mask,
		.measure_only_connected = cfg->measure_only_connected,
		.interval_ms = cfg->interval_ms,
	};
	int err = settings_save_one("thermo/cfg", &stored, sizeof(stored));

	if (err) {
		LOG_WRN("Saving the configuration failed (%d), it is lost at the next reset", err);
	}
#endif

	if (changed_cb) {
		changed_cb(cfg);
	}
	return 0;
}

#if defined(CONFIG_SETTINGS)
static int config_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	struct stored_config stored;
	const char *next;

	if (!settings_name_steq(name, "cfg", &next) || next) {
		return -ENOENT;
	}
	if (len != sizeof(stored) || read_cb(cb_arg, &stored, sizeof(stored)) != sizeof(stored) ||
	    stored.version != STORED_VERSION) {
		LOG_WRN("Ignoring a stored configuration of an unknown format, using the defaults");
		return 0;
	}

	struct app_config cfg = {
		.interval_ms = stored.interval_ms,
		.sensor_mask = stored.sensor_mask,
		.measure_only_connected = stored.measure_only_connected,
	};

	if (!valid(&cfg)) {
		LOG_WRN("Ignoring an invalid stored configuration, using the defaults");
		return 0;
	}

	k_mutex_lock(&config_lock, K_FOREVER);
	config = cfg;
	k_mutex_unlock(&config_lock);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(thermo, "thermo", NULL, config_settings_set, NULL, NULL);
#endif
