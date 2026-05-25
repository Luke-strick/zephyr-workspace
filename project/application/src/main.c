/*
 * Sailing Tracker — main application
 *
 * Initialisation order:
 *   1. Power domains (3V3 → sensors/LoRa/GPS, 5V → LCD)
 *   2. NVS settings + config
 *   3. AHRS (sensors)
 *   4. Nav library
 *   5. Data engine thread
 *   6. Data processor thread
 *   7. LoRa comms threads (if enabled)
 *   8. Display engine thread
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config.h"
#include "data_engine.h"
#include "data_processor.h"
#include "lora_comms.h"
#include "display_engine.h"

#include <ahrs.h>
#include <nav.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	int ret;

	/* 1. Power up 3V3 domain (sensors, LoRa, GPS).
	 *    DTS startup-delay-us = 2000; pm_device_runtime_get waits for it. */
	ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));
	if (ret < 0) {
		LOG_ERR("Failed to enable 3V3 domain: %d", ret);
	}

	/* 2. Power up 5V domain (LCD).
	 *    DTS startup-delay-us = 5000. */
	ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_5v)));
	if (ret < 0) {
		LOG_ERR("Failed to enable 5V domain: %d", ret);
	}

	/* 3. Extra delay for SX1262 BUSY deassertion (~100 ms after power). */
	k_msleep(100);

	/* 4. Load config from NVS flash. */
	ret = config_init();
	if (ret < 0) {
		LOG_WRN("config_init failed (%d), continuing with defaults", ret);
	}

	const struct app_config *cfg = config_get();

	/* 5. Initialise AHRS with calibration values from config. */
	const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));
	const struct device *mag = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));

	ret = ahrs_init(imu, mag);
	if (ret < 0) {
		LOG_ERR("ahrs_init failed: %d", ret);
	} else {
		ahrs_set_gyro_bias(cfg->gyro_bias_x_mdps,
				   cfg->gyro_bias_y_mdps,
				   cfg->gyro_bias_z_mdps);
		ahrs_set_mag_calibration(cfg->hard_iron,
					 (const float (*)[3])cfg->soft_iron);
	}

	/* 6. Initialise nav library. */
	nav_init();

	/* 7. Start data engine (spawns thread; GNSS callback registered statically). */
	ret = data_engine_init();
	if (ret < 0) {
		LOG_ERR("data_engine_init failed: %d", ret);
	}

	/* 8. Start data processor. */
	ret = data_processor_init();
	if (ret < 0) {
		LOG_ERR("data_processor_init failed: %d", ret);
	}

	/* 9. Start LoRa comms if enabled in config. */
	if (cfg->lora.enabled) {
		ret = lora_comms_init();
		if (ret < 0) {
			LOG_ERR("lora_comms_init failed: %d", ret);
		}
	} else {
		LOG_INF("LoRa disabled in config");
	}

	/* 10. Start display engine (last — benefits from fully initialised system). */
	ret = display_engine_init();
	if (ret < 0) {
		LOG_ERR("display_engine_init failed: %d", ret);
	}

	LOG_INF("Tracker started");

	config_set_sample_rate(DATA_RATE_10HZ);

	/* main() returns; all work is in spawned threads. */
	return 0;
}
