/*
 * data_engine — Sensor polling and ring-buffer averager
 *
 * Owns a 600-entry ring buffer (60 s × 10 Hz max) of raw sensor samples.
 * Runs in its own thread; signals data_processor after each capture.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DATA_ENGINE_H
#define DATA_ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/gnss.h>

/* ── Structs ─────────────────────────────────────────────────────────────── */

struct data_sample {
	int64_t  timestamp_ms;

	/* GPS fields */
	uint32_t gps_speed_mm_s;
	uint32_t gps_bearing_mdeg;
	int32_t  gps_altitude_mm;
	int64_t  gps_lat_ndeg;
	int64_t  gps_lon_ndeg;
	bool     gps_fix_valid;

	/* AHRS fields */
	float    heading_deg;
	float    roll_deg;
	float    pitch_deg;

	uint8_t  sources; /* data_src_flags_t bitmask */
};

struct data_averages {
	uint32_t avg_speed_mm_s;
	uint32_t avg_bearing_mdeg;
	int32_t  avg_altitude_mm;
	float    avg_heading_deg;
	float    avg_roll_deg;
	float    avg_pitch_deg;
	bool     gps_fix_valid;
	int64_t  last_lat_ndeg;
	int64_t  last_lon_ndeg;
	struct gnss_time utc; /* most recent GPS UTC time */
};

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialise the data engine and spawn its thread.
 * ahrs_init() must have been called before this.
 *
 * @return 0 on success, negative errno on failure.
 */
int data_engine_init(void);

/**
 * Called from the GNSS data callback to deliver a new GPS fix.
 * Thread-safe (uses spinlock internally).
 */
void data_engine_push_gps(const struct gnss_data *data);

/**
 * Get a consistent snapshot of the running averages.
 * Thread-safe; may be called from any thread.
 */
void data_engine_get_averages(struct data_averages *out);

/**
 * Get the most recently captured raw sample.
 * Thread-safe; may be called from any thread.
 */
void data_engine_get_latest(struct data_sample *out);

/**
 * Enable or disable magnetometer calibration collection.
 *
 * While enabled, the engine thread reads the raw magnetometer at
 * 10 Hz and feeds samples into the AHRS calibration buffer via
 * ahrs_cal_collect().  The normal AHRS filter continues running.
 *
 * Thread-safe; may be called from any thread.
 */
void data_engine_set_calibrating(bool cal);

/** Return true if calibration collection is active. */
bool data_engine_is_calibrating(void);

void data_engine_set_imu_calibrating(bool cal);
bool data_engine_is_imu_calibrating(void);

#endif /* DATA_ENGINE_H */
