/*
 * data_processor — Navigation algorithm runner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "data_processor.h"
#include "data_engine.h"
#include "lora_comms.h"
#include "config.h"

#include <nav.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(data_processor, LOG_LEVEL_INF);

/* ── Peer packet queue (from lora RX thread) ─────────────────────────────── */

#define PEER_QUEUE_DEPTH  8

K_MSGQ_DEFINE(peer_msgq, sizeof(struct boat_packet), PEER_QUEUE_DEPTH, 4);

/* ── Shared results ──────────────────────────────────────────────────────── */

static K_MUTEX_DEFINE(results_mutex);
static struct nav_results shared_results;

/* ── Semaphore declared in data_engine.c ────────────────────────────────── */

extern struct k_sem data_engine_sem;

/* ── Processor thread ────────────────────────────────────────────────────── */

#define PROC_STACK_SIZE  2048
#define PROC_PRIORITY    6

static K_THREAD_STACK_DEFINE(proc_stack, PROC_STACK_SIZE);
static struct k_thread proc_thread_data;

static void processor_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	int64_t last_update_ms = k_uptime_get();

	LOG_INF("Data processor started");

	while (1) {
		/* Wait for the data engine to produce a new sample. */
		k_sem_take(&data_engine_sem, K_FOREVER);

		const struct app_config *cfg = config_get();
		struct data_averages avg;

		data_engine_get_averages(&avg);

		int64_t now = k_uptime_get();
		uint32_t dt_ms = (uint32_t)(now - last_update_ms);

		last_update_ms = now;

		/* Run nav algorithms. */
		nav_update(avg.avg_speed_mm_s,
			   (uint32_t)(avg.avg_heading_deg * 1000.0f),
			   avg.avg_bearing_mdeg,
			   cfg->true_wind_dir_deg,
			   dt_ms);

		/* Build result. */
		struct nav_results r = { 0 };

		r.drift_deg     = nav_get_drift_deg();
		r.tack_detected = nav_tack_detected();
		r.gybe_detected = nav_gybe_detected();
		r.valid         = true;

		/* VMG differentials — drain peer queue. */
		struct boat_packet pkt;

		while (k_msgq_get(&peer_msgq, &pkt, K_NO_WAIT) == 0) {
			if (pkt.boat_id < 4) {
				uint32_t peer_speed_mm_s =
					(uint32_t)pkt.speed_ckt * 100000U / 1944U;
				uint32_t peer_bearing_cdeg = pkt.bearing_cdeg;
				uint32_t my_vmg = nav_compute_vmg(
					avg.avg_speed_mm_s,
					(uint32_t)(avg.avg_heading_deg * 1000.0f),
					cfg->true_wind_dir_deg);
				uint32_t peer_vmg = nav_compute_vmg(
					peer_speed_mm_s,
					peer_bearing_cdeg * 10U, /* cdeg → mdeg */
					cfg->true_wind_dir_deg);

				r.vmg_differential[pkt.boat_id] =
					nav_get_differential(my_vmg, peer_vmg);
			}
		}

		if (r.tack_detected) {
			LOG_INF("Tack detected!");
		}
		if (r.gybe_detected) {
			LOG_INF("Gybe detected!");
		}

		k_mutex_lock(&results_mutex, K_FOREVER);
		shared_results = r;
		k_mutex_unlock(&results_mutex);
	}
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int data_processor_init(void)
{
	k_thread_create(&proc_thread_data, proc_stack,
			K_THREAD_STACK_SIZEOF(proc_stack),
			processor_thread, NULL, NULL, NULL,
			PROC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&proc_thread_data, "data_proc");

	return 0;
}

void data_processor_get_results(struct nav_results *out)
{
	k_mutex_lock(&results_mutex, K_FOREVER);
	*out = shared_results;
	k_mutex_unlock(&results_mutex);
}

void data_processor_push_peer(const struct boat_packet *pkt)
{
	/* Non-blocking; drops if full. */
	k_msgq_put(&peer_msgq, pkt, K_NO_WAIT);
}
