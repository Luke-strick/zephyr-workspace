/*
 * LoRa ping-pong test — E22-900M22S (SX1262)
 *
 * Two boards run identical firmware.  Set BOARD_ID to 0 on one
 * and 1 on the other.
 *
 *   Board 0: transmit → listen → sleep → repeat
 *   Board 1: listen → transmit → sleep → repeat
 *
 * Both boards print every TX and RX with RSSI and SNR.
 * Output: "TX  id=0 cnt=5"
 *         "RX  id=1 cnt=3  rssi=-67 snr=9"
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/pm/device_runtime.h>
#include <stdio.h>

/* ============================================================
 * CONFIGURATION — change BOARD_ID to 1 on the second device
 * ============================================================ */
#define BOARD_ID        0           /* 0 or 1 */

#define LORA_FREQ       915000000   /* Hz — 868000000 for EU */
#define LORA_TX_POWER   14          /* dBm */
#define LORA_SF         SF_7
#define LORA_BW         BW_125_KHZ
#define LORA_CR         CR_4_5

#define TX_INTERVAL_MS  5000        /* ms between transmissions */
#define RX_TIMEOUT_MS   4500        /* ms to wait for a reply  */
/* ============================================================ */

static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static int lora_set_mode(bool tx)
{
	struct lora_modem_config cfg = {
		.frequency    = LORA_FREQ,
		.bandwidth    = LORA_BW,
		.datarate     = LORA_SF,
		.coding_rate  = LORA_CR,
		.preamble_len = 8,
		.tx_power     = LORA_TX_POWER,
		.tx           = tx,
	};

	return lora_config(lora_dev, &cfg);
}

static void do_tx(uint32_t counter)
{
	char buf[32];
	int len = snprintf(buf, sizeof(buf), "id=%d cnt=%u", BOARD_ID, counter);

	if (lora_set_mode(true) < 0) {
		printk("TX config failed\n");
		return;
	}

	if (lora_send(lora_dev, buf, len) < 0) {
		printk("TX send failed\n");
	} else {
		printk("TX  %s\n", buf);
	}
}

static void do_rx(void)
{
	uint8_t buf[64];
	int16_t rssi, snr;

	if (lora_set_mode(false) < 0) {
		printk("RX config failed\n");
		return;
	}

	int len = lora_recv(lora_dev, buf, sizeof(buf) - 1,
			    K_MSEC(RX_TIMEOUT_MS), &rssi, &snr);

	if (len > 0) {
		buf[len] = '\0';
		printk("RX  %s  rssi=%d snr=%d\n", buf, rssi, snr);
	} else if (len == -EAGAIN) {
		printk("RX  timeout\n");
	} else {
		printk("RX  error %d\n", len);
	}
}

int main(void)
{
	const struct device *pwr_3v3 = DEVICE_DT_GET(DT_NODELABEL(power_3v3));

	pm_device_runtime_get(pwr_3v3);
	k_msleep(100); /* SX1262 needs ~3.5 ms after power-on for BUSY to go low; 100 ms is safe */

	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	if (device_is_ready(usb_dev)) {
		usb_enable(NULL);
		uint32_t dtr = 0;

		while (!dtr) {
			uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
			k_sleep(K_MSEC(100));
		}
	}

	if (!device_is_ready(lora_dev)) {
		printk("LoRa device not ready\n");
		return -ENODEV;
	}

	printk("LoRa test  board=%d  freq=%u MHz  SF7/BW125\n",
	       BOARD_ID, LORA_FREQ / 1000000);

	uint32_t counter = 0;

	while (1) {
#if BOARD_ID == 0
		do_tx(counter++);
		do_rx();
#else
		do_rx();
		do_tx(counter++);
#endif
		k_msleep(TX_INTERVAL_MS - RX_TIMEOUT_MS);
	}

	return 0;
}
