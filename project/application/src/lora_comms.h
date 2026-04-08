/*
 * lora_comms — Time-windowed LoRa multi-boat broadcast
 *
 * Up to 4 boats share one channel. GPS UTC time is used to divide the TX
 * period into per-boat slots. A 5 s pairing phase discovers how many boats
 * are on channel before entering the running phase.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LORA_COMMS_H
#define LORA_COMMS_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include "data_engine.h"

/* ── Wire packet ─────────────────────────────────────────────────────────── */

/**
 * Fixed 24-byte packet transmitted by every boat.
 * All multi-byte fields are little-endian.
 */
struct __attribute__((packed)) boat_packet {
	uint8_t  type;         /* LORA_PKT_HELLO=0x01, LORA_PKT_DATA=0x02   */
	uint8_t  boat_id;      /* 0–3                                         */
	uint8_t  n_boats;      /* total boats on channel (HELLO only, 0=DATA) */
	uint8_t  reserved;     /* 0x00                                        */
	uint16_t speed_ckt;    /* centi-knots                                 */
	uint16_t heading_cdeg; /* centidegrees 0–35999                        */
	uint16_t bearing_cdeg; /* GPS COG centidegrees                        */
	int32_t  lat_100ndeg;  /* nanodegrees / 100                           */
	int32_t  lon_100ndeg;  /* nanodegrees / 100                           */
	int16_t  altitude_m;   /* metres                                      */
	uint16_t utc_min_ms;   /* (minute*60000 + ms) % 65536                 */
	int8_t   rssi_rx;      /* RSSI of last received packet, dBm           */
	uint8_t  crc8;         /* CRC-8/MAXIM over bytes [0..22]              */
};

BUILD_ASSERT(sizeof(struct boat_packet) == 24, "boat_packet must be 24 bytes");

#define LORA_PKT_HELLO 0x01
#define LORA_PKT_DATA  0x02

/* ── Per-peer state (public for data_processor) ──────────────────────────── */

struct peer_boat {
	uint8_t  boat_id;
	bool     seen;
	int64_t  last_rx_ms;
	uint16_t speed_ckt;
	uint16_t bearing_cdeg;
	int8_t   last_rssi;
};

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialise LoRa hardware and spawn RX + TX threads.
 * config_init() must have been called first.
 *
 * @return 0 on success, negative errno on failure.
 */
int lora_comms_init(void);

/**
 * Update the data to be included in the next TX packet.
 * Thread-safe; called by data_engine after each capture.
 */
void lora_comms_set_tx_data(const struct data_sample *s);

/**
 * Copy current peer state into @p peers_out[4].
 * Returns the number of peers seen during pairing / running.
 * Thread-safe.
 */
int lora_comms_get_peers(struct peer_boat peers_out[4]);

#endif /* LORA_COMMS_H */
