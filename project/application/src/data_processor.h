/*
 * data_processor — Navigation algorithm runner
 *
 * Consumes data engine averages and peer LoRa packets; runs the nav library
 * to compute drift, tack/gybe events, and VMG differentials.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DATA_PROCESSOR_H
#define DATA_PROCESSOR_H

#include <stdbool.h>
#include <stdint.h>

/* Forward declaration — full definition in lora_comms.h */
struct boat_packet;

/* ── Result struct ───────────────────────────────────────────────────────── */

struct nav_results {
	float   drift_deg;         /* leeway angle, positive = set to starboard */
	bool    tack_detected;     /* self-cleared after read                   */
	bool    gybe_detected;     /* self-cleared after read                   */
	int32_t vmg_differential[4]; /* signed mm/s vs each peer (index=boat_id) */
	bool    valid;
};

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialise and spawn the data processor thread.
 * data_engine_init() must have been called first.
 *
 * @return 0 on success, negative errno on failure.
 */
int data_processor_init(void);

/**
 * Get a consistent snapshot of the latest nav results.
 * Thread-safe.
 */
void data_processor_get_results(struct nav_results *out);

/**
 * Deliver a received peer packet from the LoRa RX thread.
 * Non-blocking (drops if the internal queue is full).
 */
void data_processor_push_peer(const struct boat_packet *pkt);

#endif /* DATA_PROCESSOR_H */
