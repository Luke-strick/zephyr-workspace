/*
 * LoRa sample — E22-900M22S (SX1262)
 * Transmits a counter every 10 s, then listens for a reply.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(lora_sample, LOG_LEVEL_INF);

#define LORA_FREQUENCY     915000000   /* 915 MHz — change to 868 MHz for EU */
#define TX_INTERVAL_SEC    10
#define RX_TIMEOUT_SEC     5

static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static int configure_lora(bool tx)
{
	struct lora_modem_config config = {
		.frequency    = LORA_FREQUENCY,
		.bandwidth    = BW_125_KHZ,
		.datarate     = SF_7,
		.coding_rate  = CR_4_5,
		.preamble_len = 8,
		.tx_power     = 14,
		.tx           = tx,
	};

	return lora_config(lora_dev, &config);
}

int main(void)
{
	uint32_t counter = 0;
	char tx_buf[64];
	uint8_t rx_buf[255];
	int16_t rssi, snr;
	int ret, len;

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa device not ready");
		return -ENODEV;
	}

	LOG_INF("LoRa sample started — %u MHz, SF7/BW125", LORA_FREQUENCY / 1000000);

	while (1) {
		/* — Transmit — */
		ret = configure_lora(true);
		if (ret < 0) {
			LOG_ERR("TX config failed: %d", ret);
			k_sleep(K_SECONDS(TX_INTERVAL_SEC));
			continue;
		}

		len = snprintf(tx_buf, sizeof(tx_buf), "tracker #%u", counter++);
		ret = lora_send(lora_dev, tx_buf, len);
		if (ret < 0) {
			LOG_ERR("Send failed: %d", ret);
		} else {
			LOG_INF("TX [%u]: \"%s\"", counter - 1, tx_buf);
		}

		/* — Receive — */
		ret = configure_lora(false);
		if (ret < 0) {
			LOG_ERR("RX config failed: %d", ret);
			k_sleep(K_SECONDS(TX_INTERVAL_SEC));
			continue;
		}

		LOG_INF("Listening for %d s ...", RX_TIMEOUT_SEC);
		len = lora_recv(lora_dev, rx_buf, sizeof(rx_buf),
				K_SECONDS(RX_TIMEOUT_SEC), &rssi, &snr);
		if (len > 0) {
			rx_buf[MIN(len, sizeof(rx_buf) - 1)] = '\0';
			LOG_INF("RX: \"%s\"  RSSI=%d dBm  SNR=%d dB",
				rx_buf, rssi, snr);
		} else if (len == -EAGAIN) {
			LOG_INF("No reply (timeout)");
		} else {
			LOG_ERR("Receive error: %d", len);
		}

		k_sleep(K_SECONDS(TX_INTERVAL_SEC - RX_TIMEOUT_SEC));
	}

	return 0;
}
