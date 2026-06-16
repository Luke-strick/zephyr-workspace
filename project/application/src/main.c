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
#include <zephyr/pm/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ── UBX helpers ─────────────────────────────────────────────────────────── */

static const struct device *gps_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));

static void ubx_send(uint8_t cls, uint8_t id,
		     const uint8_t *payload, uint16_t len)
{
	uint8_t ck_a = 0, ck_b = 0;

#define UBX_FEED(b)  do { uint8_t _b = (b); ck_a += _b; ck_b += ck_a; \
			   uart_poll_out(gps_uart, _b); } while (0)

	uart_poll_out(gps_uart, 0xB5);
	uart_poll_out(gps_uart, 0x62);

	UBX_FEED(cls);
	UBX_FEED(id);
	UBX_FEED((uint8_t)(len & 0xFF));
	UBX_FEED((uint8_t)(len >> 8));

	for (uint16_t i = 0; i < len; i++) {
		UBX_FEED(payload[i]);
	}

#undef UBX_FEED

	uart_poll_out(gps_uart, ck_a);
	uart_poll_out(gps_uart, ck_b);
}

static void ubx_enable_constellations(void)
{
	static const uint8_t payload[] = {
		0x00, 0x00, 0xFF, 0x03,
		/* GPS:     gnssId=0, resTrkCh=8,  maxTrkCh=16, L1C/A */
		0x00, 0x08, 0x10, 0x00,  0x01, 0x00, 0x01, 0x00,
		/* SBAS:    gnssId=1, resTrkCh=1,  maxTrkCh=3,  L1C/A */
		0x01, 0x01, 0x03, 0x00,  0x01, 0x00, 0x01, 0x00,
		/* GLONASS: gnssId=6, resTrkCh=8,  maxTrkCh=14, L1   */
		0x06, 0x08, 0x0E, 0x00,  0x01, 0x00, 0x01, 0x00,
	};

	ubx_send(0x06, 0x3E, payload, sizeof(payload));
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
	int ret;

	/* 1. Power up 3V3 domain (sensors, LoRa, GPS).
	 *    DTS startup-delay-us = 2000; pm_device_runtime_get waits for it. */
	ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));
	if (ret < 0) {
		LOG_ERR("Failed to enable 3V3 domain: %d", ret);
	}

	/* 2. Give the BN220 time to boot before sending UBX config. */
	k_msleep(200);

	/* 3. Enable GPS + SBAS + GLONASS on the M8030. */
	ubx_enable_constellations();

	/* 4. Resume the GNSS driver — starts suspended with CONFIG_PM_DEVICE=y.
	 *    pm_device_action_run (not runtime_get) because gnss_nmea_generic
	 *    calls pm_device_init_suspended, not pm_device_runtime_enable. */
	ret = pm_device_action_run(DEVICE_DT_GET(DT_NODELABEL(gnss)),
				   PM_DEVICE_ACTION_RESUME);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("GNSS resume failed: %d", ret);
	}

	/* 5. Power up 5V domain (LCD).
	 *    DTS startup-delay-us = 5000. */
	ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_5v)));
	if (ret < 0) {
		LOG_ERR("Failed to enable 5V domain: %d", ret);
	}

	/* 6. Extra delay for SX1262 BUSY deassertion (~100 ms after power). */
	k_msleep(100);

	/* 7. Load config from NVS flash. */
	ret = config_init();
	if (ret < 0) {
		LOG_WRN("config_init failed (%d), continuing with defaults", ret);
	}

	const struct app_config *cfg = config_get();

	/* 8. Initialise AHRS with calibration values from config. */
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

	/* 9. Initialise nav library. */
	nav_init();

	/* 10. Start data engine (spawns thread; GNSS callback registered statically). */
	ret = data_engine_init();
	if (ret < 0) {
		LOG_ERR("data_engine_init failed: %d", ret);
	}

	/* 11. Start data processor. */
	ret = data_processor_init();
	if (ret < 0) {
		LOG_ERR("data_processor_init failed: %d", ret);
	}

	/* 12. Start LoRa comms if enabled in config. */
	if (cfg->lora.enabled) {
		ret = lora_comms_init();
		if (ret < 0) {
			LOG_ERR("lora_comms_init failed: %d", ret);
		}
	} else {
		LOG_INF("LoRa disabled in config");
	}

	/* 13. Start display engine (last — benefits from fully initialised system). */
	ret = display_engine_init();
	if (ret < 0) {
		LOG_ERR("display_engine_init failed: %d", ret);
	}

	LOG_INF("Tracker started");

	config_set_sample_rate(DATA_RATE_10HZ);

	/* main() returns; all work is in spawned threads. */
	return 0;
}
