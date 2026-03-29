/*
 * GPS sample — MAX-M10S via GNSS subsystem
 * Prints position, fix status, and satellite count.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gps_sample, LOG_LEVEL_INF);

static const struct device *gnss_dev = DEVICE_DT_GET(DT_NODELABEL(gnss));

static const char *fix_str(enum gnss_fix_status status)
{
	switch (status) {
	case GNSS_FIX_STATUS_NO_FIX:
		return "NO FIX";
	case GNSS_FIX_STATUS_GNSS_FIX:
		return "GNSS FIX";
	case GNSS_FIX_STATUS_DGNSS_FIX:
		return "DGNSS FIX";
	case GNSS_FIX_STATUS_ESTIMATED_FIX:
		return "ESTIMATED";
	default:
		return "UNKNOWN";
	}
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	if (data->info.fix_status >= GNSS_FIX_STATUS_GNSS_FIX) {
		int64_t lat = data->nav_data.latitude;
		int64_t lon = data->nav_data.longitude;

		LOG_INF("[%s] sats=%u  lat=%lld.%09lld  lon=%lld.%09lld  alt=%d.%03dm",
			fix_str(data->info.fix_status),
			data->info.satellites_cnt,
			lat / 1000000000LL,
			llabs(lat % 1000000000LL),
			lon / 1000000000LL,
			llabs(lon % 1000000000LL),
			data->nav_data.altitude / 1000,
			abs(data->nav_data.altitude % 1000));
	} else {
		LOG_INF("[%s] sats=%u  waiting for fix ...",
			fix_str(data->info.fix_status),
			data->info.satellites_cnt);
	}
}

GNSS_DATA_CALLBACK_DEFINE(NULL, gnss_data_cb);

static void gnss_satellites_cb(const struct device *dev,
				const struct gnss_satellite *sats,
				uint16_t size)
{
	unsigned int used = 0;

	for (uint16_t i = 0; i < size; i++) {
		if (sats[i].is_tracked) {
			used++;
		}
	}
	LOG_INF("Satellites: %u visible, %u tracked", size, used);
}

GNSS_SATELLITES_CALLBACK_DEFINE(NULL, gnss_satellites_cb);

int main(void)
{
	if (!device_is_ready(gnss_dev)) {
		LOG_ERR("GNSS device not ready");
		return -ENODEV;
	}

	LOG_INF("GPS sample started — waiting for data from MAX-M10S");
	LOG_INF("Place the device outdoors for a satellite fix.");

	/* GNSS callbacks fire asynchronously; nothing to do here. */
	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
