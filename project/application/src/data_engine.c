/*
 * data_engine — Sensor polling and ring-buffer averager
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "data_engine.h"
#include "config.h"

#include <ahrs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gnss.h>
#include <string.h>

LOG_MODULE_REGISTER(data_engine, LOG_LEVEL_INF);

/* ── Ring buffer ─────────────────────────────────────────────────────────── */

/* 60 s × 10 Hz = 600 samples maximum. */
#define RING_SIZE  600

static struct data_sample ring[RING_SIZE];
static uint32_t ring_head;   /* next write index */
static uint32_t ring_count;  /* valid samples, capped at RING_SIZE */

/* ── Pending GPS slot (spinlock-protected) ───────────────────────────────── */

static struct k_spinlock gps_lock;
static struct gnss_data pending_gps;
static bool             pending_gps_valid;

/* ── Shared averages (mutex-protected) ──────────────────────────────────── */

static K_MUTEX_DEFINE(avg_mutex);
static struct data_averages shared_avg;
static struct data_sample   latest_sample;

/* ── Semaphore to signal data_processor ─────────────────────────────────── */

K_SEM_DEFINE(data_engine_sem, 0, 1);

/* ── GNSS callback ───────────────────────────────────────────────────────── */

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	ARG_UNUSED(dev);
	k_spinlock_key_t key = k_spin_lock(&gps_lock);

	pending_gps       = *data;
	pending_gps_valid = true;
	k_spin_unlock(&gps_lock, key);
}

GNSS_DATA_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(gnss)), gnss_data_cb);

/* ── Average computation ─────────────────────────────────────────────────── */

static void recompute_averages(const struct app_config *cfg)
{
	uint32_t window = (uint32_t)cfg->sample_rate_hz * 60U;

	if (window > ring_count) {
		window = ring_count;
	}
	if (window == 0) {
		return;
	}

	uint64_t sum_speed = 0;
	uint64_t sum_bearing = 0;
	int64_t  sum_alt = 0;
	double   sum_hdg = 0.0, sum_roll = 0.0, sum_pitch = 0.0;
	uint32_t gps_count = 0;

	for (uint32_t i = 0; i < window; i++) {
		uint32_t idx = (ring_head + RING_SIZE - 1 - i) % RING_SIZE;
		const struct data_sample *s = &ring[idx];

		sum_hdg   += s->heading_deg;
		sum_roll  += s->roll_deg;
		sum_pitch += s->pitch_deg;

		if (s->gps_fix_valid) {
			sum_speed   += s->gps_speed_mm_s;
			sum_bearing += s->gps_bearing_mdeg;
			sum_alt     += s->gps_altitude_mm;
			gps_count++;
		}
	}

	struct data_averages avg = { 0 };

	avg.avg_heading_deg = (float)(sum_hdg  / window);
	avg.avg_roll_deg    = (float)(sum_roll / window);
	avg.avg_pitch_deg   = (float)(sum_pitch / window);

	if (gps_count > 0) {
		avg.avg_speed_mm_s   = (uint32_t)(sum_speed / gps_count);
		avg.avg_bearing_mdeg = (uint32_t)(sum_bearing / gps_count);
		avg.avg_altitude_mm  = (int32_t)(sum_alt / (int64_t)gps_count);
		avg.gps_fix_valid    = true;
	}

	/* Copy last-known position and UTC from the ring head. */
	if (ring_count > 0) {
		uint32_t last = (ring_head + RING_SIZE - 1) % RING_SIZE;

		avg.last_lat_ndeg = ring[last].gps_lat_ndeg;
		avg.last_lon_ndeg = ring[last].gps_lon_ndeg;
	}

	k_mutex_lock(&avg_mutex, K_FOREVER);
	shared_avg = avg;
	k_mutex_unlock(&avg_mutex);
}

/* ── Engine thread ───────────────────────────────────────────────────────── */

#define ENGINE_STACK_SIZE  2048
#define ENGINE_PRIORITY    5

static K_THREAD_STACK_DEFINE(engine_stack, ENGINE_STACK_SIZE);
static struct k_thread engine_thread_data;

/* AHRS runs at 10 Hz internally regardless of capture rate. */
#define AHRS_INTERVAL_MS  100

static void engine_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	int64_t  last_capture_ms = k_uptime_get();
	int64_t  last_ahrs_ms    = last_capture_ms;

	LOG_INF("Data engine started");

	while (1) {
		k_msleep(10); /* 10 ms base tick */

		const struct app_config *cfg = config_get();
		int64_t now = k_uptime_get();

		/* Run AHRS filter at 10 Hz regardless of capture rate. */
		if ((cfg->data_sources & DATA_SRC_AHRS) &&
		    (now - last_ahrs_ms) >= AHRS_INTERVAL_MS) {
			ahrs_update();
			last_ahrs_ms = now;
		}

		/* Capture sample at user-configured rate. */
		int64_t capture_interval_ms = 1000 / cfg->sample_rate_hz;

		if ((now - last_capture_ms) < capture_interval_ms) {
			continue;
		}
		last_capture_ms = now;

		struct data_sample s = { 0 };

		s.timestamp_ms = now;
		s.sources      = cfg->data_sources;

		/* Drain pending GPS slot. */
		k_spinlock_key_t key = k_spin_lock(&gps_lock);

		if (pending_gps_valid) {
			s.gps_speed_mm_s   = pending_gps.nav_data.speed;
			s.gps_bearing_mdeg = pending_gps.nav_data.bearing;
			s.gps_altitude_mm  = pending_gps.nav_data.altitude;
			s.gps_lat_ndeg     = pending_gps.nav_data.latitude;
			s.gps_lon_ndeg     = pending_gps.nav_data.longitude;
			s.gps_fix_valid    = (pending_gps.info.fix_status != GNSS_FIX_STATUS_NO_FIX);
			pending_gps_valid  = false;
		}
		k_spin_unlock(&gps_lock, key);

		/* Read AHRS. */
		if (cfg->data_sources & DATA_SRC_AHRS) {
			ahrs_get(&s.roll_deg, &s.pitch_deg, &s.heading_deg);
		}

		/* Write to ring. */
		ring[ring_head] = s;
		ring_head = (ring_head + 1) % RING_SIZE;
		if (ring_count < RING_SIZE) {
			ring_count++;
		}

		/* Update shared latest and averages. */
		k_mutex_lock(&avg_mutex, K_FOREVER);
		latest_sample = s;
		k_mutex_unlock(&avg_mutex);

		recompute_averages(cfg);

		/* Signal data_processor. */
		k_sem_give(&data_engine_sem);
	}
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int data_engine_init(void)
{
	memset(ring, 0, sizeof(ring));
	ring_head  = 0;
	ring_count = 0;

	k_thread_create(&engine_thread_data, engine_stack,
			K_THREAD_STACK_SIZEOF(engine_stack),
			engine_thread, NULL, NULL, NULL,
			ENGINE_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&engine_thread_data, "data_engine");

	return 0;
}

void data_engine_push_gps(const struct gnss_data *data)
{
	/* Called from GNSS workqueue — use spinlock, not mutex. */
	k_spinlock_key_t key = k_spin_lock(&gps_lock);

	pending_gps       = *data;
	pending_gps_valid = true;
	k_spin_unlock(&gps_lock, key);
}

void data_engine_get_averages(struct data_averages *out)
{
	k_mutex_lock(&avg_mutex, K_FOREVER);
	*out = shared_avg;
	k_mutex_unlock(&avg_mutex);
}

void data_engine_get_latest(struct data_sample *out)
{
	k_mutex_lock(&avg_mutex, K_FOREVER);
	*out = latest_sample;
	k_mutex_unlock(&avg_mutex);
}
