/*
 * nav — Sailing navigation algorithms
 *
 * Tack and gybe detection are based on the True Wind Direction (TWD) set by
 * the user. VMG is computed against the same wind reference.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NAV_H
#define NAV_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialise the nav library. Clears all state.
 * Must be called once before any other nav_* function.
 */
void nav_init(void);

/**
 * Feed a new navigation sample into the library.
 *
 * Call this each time the data_processor has a fresh snapshot.
 *
 * @param speed_mm_s        Speed over ground in mm/s
 * @param heading_mdeg      Compass heading in millidegrees [0, 360000)
 * @param gps_bearing_mdeg  GPS course over ground in millidegrees [0, 360000)
 * @param wind_dir_deg      True wind direction in degrees (direction wind comes FROM)
 * @param dt_ms             Milliseconds since last nav_update() call (unused internally
 *                          but retained for future filtering)
 */
void nav_update(uint32_t speed_mm_s,
		uint32_t heading_mdeg,
		uint32_t gps_bearing_mdeg,
		float    wind_dir_deg,
		uint32_t dt_ms);

/**
 * Return the current leeway (drift) angle in degrees.
 *
 * Defined as (compass heading − GPS COG), normalised to (−180, +180].
 * Positive = boat is being set to starboard of its compass course.
 * Returns 0.0f if no GPS update has been received.
 */
float nav_get_drift_deg(void);

/**
 * Return true if a tack was detected since the last call that returned true.
 *
 * A tack is a port↔starboard transition while upwind (heading within ±90° of
 * the wind direction). A deadband of CONFIG_NAV_TACK_DEADBAND_DEG prevents
 * re-triggering while sailing close to head-to-wind.
 *
 * Self-clearing: returns true exactly once per detected tack.
 */
bool nav_tack_detected(void);

/**
 * Return true if a gybe was detected since the last call that returned true.
 *
 * A gybe is a port↔starboard transition while downwind (heading within ±90°
 * of wind_dir + 180°). Same deadband applies around dead-downwind.
 *
 * Self-clearing: returns true exactly once per detected gybe.
 */
bool nav_gybe_detected(void);

/**
 * Return the signed VMG differential between this boat and a peer.
 *
 * VMG = speed × cos(heading − wind_dir). Positive result means this boat
 * has higher VMG (faster to windward); negative means the peer is faster.
 *
 * @param my_vmg_mm_s     This boat's VMG in mm/s (from nav_compute_vmg)
 * @param other_vmg_mm_s  Peer boat's VMG in mm/s
 * @return                Signed differential in mm/s
 */
int32_t nav_get_differential(uint32_t my_vmg_mm_s, uint32_t other_vmg_mm_s);

/**
 * Compute VMG from speed, heading, and true wind direction.
 *
 * @param speed_mm_s    Speed in mm/s
 * @param heading_mdeg  Heading in millidegrees
 * @param wind_dir_deg  True wind direction in degrees
 * @return              VMG in mm/s (always non-negative)
 */
uint32_t nav_compute_vmg(uint32_t speed_mm_s, uint32_t heading_mdeg, float wind_dir_deg);

#endif /* NAV_H */
