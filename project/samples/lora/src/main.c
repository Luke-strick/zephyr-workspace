/*
 * LoRa ping-pong test for two Tracker boards (SX1262 / E22-900M22S)
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │  Set BOARD_ID to 0 on one board and 1 on the other.        │
 * │  Board 0 initiates; board 1 responds.                       │
 * │                                                             │
 * │  Board 0: TX "PING:<n>"  →  Board 1: TX "PONG:<n>"  →loop │
 * └─────────────────────────────────────────────────────────────┘
 *
 * Both boards print every packet with RSSI and SNR.
 * Board 0 retransmits automatically on RX timeout.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/pm/device_runtime.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ── Change this to 1 on the second board ───────────────────────────────── */
#define BOARD_ID  1

/* ── RF configuration ────────────────────────────────────────────────────── */
#define LORA_FREQ        915000000U  /* Hz — 868 MHz (EU).  915000000 for US/AU */
#define LORA_TX_POWER    14          /* dBm */
#define LORA_BW          BW_125_KHZ
#define LORA_SF          SF_10       /* SF10 @ 125 kHz ≈ 980 bps, ~1 km range  */
#define LORA_CR          CR_4_5
#define LORA_PREAMBLE    8

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define RX_TIMEOUT_MS    6000   /* board 0 retransmits if no reply in this time */
#define TX_TURNAROUND_MS  100   /* brief pause before replying                  */

/* ── Payload ─────────────────────────────────────────────────────────────── */
#define MAX_PAYLOAD  32

/* ─────────────────────────────────────────────────────────────────────────── */

static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static void send_packet(const char *type, uint32_t counter)
{
	char buf[MAX_PAYLOAD];
	int len = snprintf(buf, sizeof(buf), "%s:%u", type, counter);

	int ret = lora_send(lora_dev, (uint8_t *)buf, len);

	if (ret < 0) {
		printk("lora_send failed: %d\n", ret);
	} else {
		printk("TX  %-10s  counter=%u\n", buf, counter);
	}
}

static int recv_packet(char *buf, uint8_t size, int16_t *rssi, int8_t *snr)
{
	return lora_recv(lora_dev, (uint8_t *)buf, size,
			 K_MSEC(RX_TIMEOUT_MS), rssi, snr);
}

/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
	/* Power up the 3V3 rail; SX1262 needs ~100 ms after power-on
	 * before BUSY deasserts and commands are accepted.            */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));
	k_msleep(100);

	/* Wait for a USB CDC terminal to connect */
	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	// if (device_is_ready(usb_dev)) {
	// 	usb_enable(NULL);
	// 	uint32_t dtr = 0;

	// 	while (!dtr) {
	// 		uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
	// 		k_sleep(K_MSEC(100));
	// 	}
	// }

	if (!device_is_ready(lora_dev)) {
		printk("LoRa device not ready\n");
		return -ENODEV;
	}

	/* Configure radio once — lora_send/lora_recv handle mode switching */
	struct lora_modem_config cfg = {
		.frequency    = LORA_FREQ,
		.bandwidth    = LORA_BW,
		.datarate     = LORA_SF,
		.coding_rate  = LORA_CR,
		.preamble_len = LORA_PREAMBLE,
		.tx_power     = LORA_TX_POWER,
		.tx           = true,   /* must be true so tx_cfg.frequency is stored */
	};

	if (lora_config(lora_dev, &cfg) < 0) {
		printk("lora_config failed\n");
		return -EIO;
	}

	printk("LoRa ping-pong  board=%d  freq=%u Hz  sf=SF%d  bw=125kHz\n",
	       BOARD_ID, LORA_FREQ, 10 /* SF10 */);

	uint32_t counter = 0;
	char rx_buf[MAX_PAYLOAD];
	int16_t rssi;
	int8_t  snr;

#if BOARD_ID == 0
	/* Board 0 always fires the first shot */
	send_packet("PING", counter);
#endif

	while (1) {
		int len = recv_packet(rx_buf, sizeof(rx_buf) - 1, &rssi, &snr);

		if (len < 0) {
#if BOARD_ID == 0
			/* Timeout — retransmit so the link can recover */
			printk("RX timeout, retrying PING:%u\n", counter);
			send_packet("PING", counter);
#else
			printk("RX timeout, waiting for PING...\n");
#endif
			continue;
		}

		rx_buf[len] = '\0';
		printk("RX  %-10s  rssi=%d dBm  snr=%d dB\n",
		       rx_buf, rssi, (int)snr);

#if BOARD_ID == 0
		if (strncmp(rx_buf, "PONG:", 5) == 0) {
			counter++;
			k_msleep(TX_TURNAROUND_MS);
			send_packet("PING", counter);
		}
#else
		if (strncmp(rx_buf, "PING:", 5) == 0) {
			uint32_t cnt = (uint32_t)atoi(rx_buf + 5);

			k_msleep(TX_TURNAROUND_MS);
			send_packet("PONG", cnt);
		}
#endif
	}

	return 0;
}
