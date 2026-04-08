/*
 * lora_comms — Time-windowed LoRa multi-boat broadcast
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lora_comms.h"
#include "data_engine.h"
#include "data_processor.h"
#include "config.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(lora_comms, LOG_LEVEL_INF);

/* ── LoRa device ─────────────────────────────────────────────────────────── */

#define LORA_DEV  DEVICE_DT_GET(DT_ALIAS(lora0))

/* ── Pairing phase duration ──────────────────────────────────────────────── */

#define PAIRING_DURATION_MS  5000

typedef enum {
	PHASE_PAIRING = 0,
	PHASE_RUNNING,
} lora_phase_t;

/* ── Shared TX data (mutex-protected) ───────────────────────────────────── */

static K_MUTEX_DEFINE(tx_mutex);
static struct data_sample tx_sample;

/* ── Shared peer state (mutex-protected) ─────────────────────────────────── */

static K_MUTEX_DEFINE(peer_mutex);
static struct peer_boat peers[4];

/* ── RSSI of last received packet ────────────────────────────────────────── */

static int8_t last_rssi_rx;

/* ── CRC helper ──────────────────────────────────────────────────────────── */

static uint8_t pkt_crc(const struct boat_packet *pkt)
{
	/* CRC-8/MAXIM (poly 0x31, init 0x00) over bytes [0..22] */
	return crc8((const uint8_t *)pkt, 23, 0x31, 0x00, false);
}

/* ── Packet builder ──────────────────────────────────────────────────────── */

static void build_data_packet(struct boat_packet *pkt, const struct app_config *cfg,
			       const struct data_sample *s)
{
	memset(pkt, 0, sizeof(*pkt));

	pkt->type    = LORA_PKT_DATA;
	pkt->boat_id = cfg->lora.boat_id;
	pkt->n_boats = 0;

	/* Speed: mm/s → centi-knots */
	uint32_t speed_ckt = s->gps_speed_mm_s * 1944U / 100000U;

	pkt->speed_ckt    = (uint16_t)(speed_ckt > 0xFFFF ? 0xFFFF : speed_ckt);
	pkt->heading_cdeg = (uint16_t)((uint32_t)(s->heading_deg * 100.0f) % 36000U);
	pkt->bearing_cdeg = (uint16_t)(s->gps_bearing_mdeg / 10U);
	pkt->lat_100ndeg  = (int32_t)(s->gps_lat_ndeg / 100);
	pkt->lon_100ndeg  = (int32_t)(s->gps_lon_ndeg / 100);
	pkt->altitude_m   = (int16_t)(s->gps_altitude_mm / 1000);
	pkt->utc_min_ms   = 0; /* filled by TX thread from UTC time */
	pkt->rssi_rx      = last_rssi_rx;
	pkt->crc8         = pkt_crc(pkt);
}

static void build_hello_packet(struct boat_packet *pkt, const struct app_config *cfg)
{
	memset(pkt, 0, sizeof(*pkt));
	pkt->type    = LORA_PKT_HELLO;
	pkt->boat_id = cfg->lora.boat_id;
	pkt->n_boats = cfg->lora.n_boats;
	pkt->rssi_rx = last_rssi_rx;
	pkt->crc8    = pkt_crc(pkt);
}

/* ── RX thread ───────────────────────────────────────────────────────────── */

#define RX_STACK_SIZE  3072
#define RX_PRIORITY    3

static K_THREAD_STACK_DEFINE(rx_stack, RX_STACK_SIZE);
static struct k_thread rx_thread_data;

static void rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	const struct device *lora_dev = LORA_DEV;
	uint8_t buf[sizeof(struct boat_packet)];
	int16_t rssi;
	int8_t  snr;

	LOG_INF("LoRa RX thread started");

	while (1) {
		int len = lora_recv(lora_dev, buf, sizeof(buf),
				    K_MSEC(500), &rssi, &snr);

		if (len < 0) {
			/* Timeout or error — loop. */
			continue;
		}
		if (len != sizeof(struct boat_packet)) {
			LOG_WRN("RX: unexpected packet length %d", len);
			continue;
		}

		struct boat_packet *pkt = (struct boat_packet *)buf;

		/* Verify CRC. */
		if (pkt->crc8 != pkt_crc(pkt)) {
			LOG_WRN("RX: CRC mismatch from boat %d", pkt->boat_id);
			continue;
		}
		if (pkt->boat_id >= 4) {
			continue;
		}

		last_rssi_rx = (int8_t)(rssi > 127 ? 127 : rssi);

		/* Update peer table. */
		k_mutex_lock(&peer_mutex, K_FOREVER);
		peers[pkt->boat_id].boat_id     = pkt->boat_id;
		peers[pkt->boat_id].seen        = true;
		peers[pkt->boat_id].last_rx_ms  = k_uptime_get();
		peers[pkt->boat_id].speed_ckt   = pkt->speed_ckt;
		peers[pkt->boat_id].bearing_cdeg= pkt->bearing_cdeg;
		peers[pkt->boat_id].last_rssi   = last_rssi_rx;
		k_mutex_unlock(&peer_mutex);

		/* Forward to data_processor. */
		data_processor_push_peer(pkt);

		LOG_DBG("RX boat=%d speed=%d rssi=%d", pkt->boat_id, pkt->speed_ckt, rssi);
	}
}

/* ── TX thread ───────────────────────────────────────────────────────────── */

#define TX_STACK_SIZE  2048
#define TX_PRIORITY    4

static K_THREAD_STACK_DEFINE(tx_stack, TX_STACK_SIZE);
static struct k_thread tx_thread_data;

static void tx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	const struct device *lora_dev = LORA_DEV;
	const struct app_config *cfg  = config_get();
	lora_phase_t phase            = PHASE_PAIRING;
	int64_t phase_start_ms        = k_uptime_get();
	uint8_t n_boats               = cfg->lora.n_boats;

	LOG_INF("LoRa TX thread started (pairing for %d ms)", PAIRING_DURATION_MS);

	while (1) {
		int64_t now = k_uptime_get();

		if (phase == PHASE_PAIRING) {
			/* Broadcast HELLO at 1 Hz during pairing. */
			struct boat_packet pkt;

			build_hello_packet(&pkt, cfg);
			lora_send(lora_dev, (uint8_t *)&pkt, sizeof(pkt));
			LOG_DBG("TX HELLO boat_id=%d", cfg->lora.boat_id);

			if ((now - phase_start_ms) >= PAIRING_DURATION_MS) {
				/* Determine n_boats from peers seen. */
				k_mutex_lock(&peer_mutex, K_FOREVER);
				for (int i = 3; i >= 0; i--) {
					if (peers[i].seen) {
						n_boats = MAX(n_boats, (uint8_t)(i + 1));
						break;
					}
				}
				k_mutex_unlock(&peer_mutex);

				LOG_INF("Pairing done: %d boat(s) on channel", n_boats);
				phase = PHASE_RUNNING;
			}

			k_msleep(1000);
			continue;
		}

		/* ── Running phase: GPS-aligned TX windows ─────────────────── */
		uint32_t tx_period_ms  = 1000U / cfg->lora.tx_rate_hz;
		uint32_t slot_width_ms = tx_period_ms / n_boats;
		uint32_t tx_offset_ms  = cfg->lora.boat_id * slot_width_ms;

		/* Get UTC milliseconds within current minute from averages. */
		struct data_averages avg;

		data_engine_get_averages(&avg);

		uint32_t utc_min_ms = 0;

		if (avg.gps_fix_valid) {
			utc_min_ms = (uint32_t)avg.utc.minute * 60000U
				     + (uint32_t)avg.utc.millisecond;
		}

		uint32_t now_in_period = utc_min_ms % tx_period_ms;
		uint32_t until_slot = (tx_period_ms + tx_offset_ms - now_in_period)
				       % tx_period_ms;

		if (until_slot > 0) {
			k_msleep(until_slot);
		}

		/* Build and send DATA packet. */
		struct data_sample s;

		k_mutex_lock(&tx_mutex, K_FOREVER);
		s = tx_sample;
		k_mutex_unlock(&tx_mutex);

		struct boat_packet pkt;

		build_data_packet(&pkt, cfg, &s);

		/* Fill UTC field. */
		pkt.utc_min_ms = (uint16_t)(utc_min_ms % 65536U);
		pkt.crc8       = pkt_crc(&pkt);

		int ret = lora_send(lora_dev, (uint8_t *)&pkt, sizeof(pkt));

		if (ret < 0) {
			LOG_WRN("TX failed: %d", ret);
		} else {
			LOG_DBG("TX DATA boat=%d spd=%d", cfg->lora.boat_id, pkt.speed_ckt);
		}

		/* Sleep for remainder of period. */
		k_msleep(tx_period_ms - slot_width_ms);
	}
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int lora_comms_init(void)
{
	const struct device *lora_dev = LORA_DEV;
	const struct app_config *cfg  = config_get();

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa device not ready");
		return -ENODEV;
	}

	/* Configure radio: SF7, BW125, 24-byte packets ~50 ms on-air. */
	struct lora_modem_config radio_cfg = {
		.frequency    = cfg->lora.frequency,
		.bandwidth    = BW_125_KHZ,
		.datarate     = SF_7,
		.coding_rate  = CR_4_5,
		.preamble_len = 8,
		.tx_power     = cfg->lora.tx_power,
		.tx           = true,
	};

	int ret = lora_config(lora_dev, &radio_cfg);

	if (ret < 0) {
		LOG_ERR("lora_config failed: %d", ret);
		return ret;
	}

	memset(peers, 0, sizeof(peers));

	k_thread_create(&rx_thread_data, rx_stack,
			K_THREAD_STACK_SIZEOF(rx_stack),
			rx_thread, NULL, NULL, NULL,
			RX_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "lora_rx");

	k_thread_create(&tx_thread_data, tx_stack,
			K_THREAD_STACK_SIZEOF(tx_stack),
			tx_thread, NULL, NULL, NULL,
			TX_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "lora_tx");

	return 0;
}

void lora_comms_set_tx_data(const struct data_sample *s)
{
	k_mutex_lock(&tx_mutex, K_FOREVER);
	memcpy(&tx_sample, s, sizeof(tx_sample));
	k_mutex_unlock(&tx_mutex);
}

int lora_comms_get_peers(struct peer_boat peers_out[4])
{
	int count = 0;

	k_mutex_lock(&peer_mutex, K_FOREVER);
	memcpy(peers_out, peers, sizeof(peers));
	for (int i = 0; i < 4; i++) {
		if (peers[i].seen) {
			count++;
		}
	}
	k_mutex_unlock(&peer_mutex);

	return count;
}
