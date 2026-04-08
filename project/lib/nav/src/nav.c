/*
 * nav — Sailing navigation algorithms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nav.h>
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD(d)  ((d) * (float)M_PI / 180.0f)
#define MDEG_TO_DEG(m) ((m) / 1000.0f)

/* Deadband around head-to-wind and dead-downwind (degrees). */
#define DEADBAND_DEG  ((float)CONFIG_NAV_TACK_DEADBAND_DEG)

/* ── Internal state ──────────────────────────────────────────────────────── */

static float last_drift_deg;
static bool  tack_pending;
static bool  gybe_pending;

/*
 * Side tracking for tack/gybe:
 *   0 = unknown / inside deadband
 *  +1 = starboard tack (wind coming from starboard, heading to left of wind)
 *  -1 = port tack (wind from port, heading to right of wind)
 */
static int8_t upwind_side;
static int8_t downwind_side;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/*
 * Normalise an angle to (−180, +180].
 */
static float normalise_180(float deg)
{
	while (deg >  180.0f) { deg -= 360.0f; }
	while (deg <= -180.0f) { deg += 360.0f; }
	return deg;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void nav_init(void)
{
	last_drift_deg = 0.0f;
	tack_pending   = false;
	gybe_pending   = false;
	upwind_side    = 0;
	downwind_side  = 0;
}

void nav_update(uint32_t speed_mm_s,
		uint32_t heading_mdeg,
		uint32_t gps_bearing_mdeg,
		float    wind_dir_deg,
		uint32_t dt_ms)
{
	(void)speed_mm_s;
	(void)dt_ms;

	float heading_deg = MDEG_TO_DEG(heading_mdeg);
	float bearing_deg = MDEG_TO_DEG(gps_bearing_mdeg);

	/* Drift: positive = set to starboard */
	last_drift_deg = normalise_180(heading_deg - bearing_deg);

	/*
	 * Angle from current heading to wind direction, normalised to (−180,+180].
	 * Negative = wind is to port (boat on starboard tack).
	 * Positive = wind is to starboard (boat on port tack).
	 */
	float delta = normalise_180(heading_deg - wind_dir_deg);

	/* ── Upwind / tack detection ─────────────────────────────────────────
	 * Boat is upwind if |delta| < 90°.
	 * Head-to-wind is delta == 0.  Deadband: |delta| < DEADBAND.
	 */
	if (fabsf(delta) < 90.0f) {
		if (fabsf(delta) >= DEADBAND_DEG) {
			/* Outside head-to-wind deadband — determine which tack. */
			int8_t new_side = (delta < 0.0f) ? -1 : 1;

			if (upwind_side != 0 && new_side != upwind_side) {
				tack_pending = true;
			}
			upwind_side = new_side;
		}
		/* Inside deadband: don't update side; prevents oscillation. */
	} else {
		upwind_side = 0;  /* downwind — reset upwind tracking */
	}

	/* ── Downwind / gybe detection ───────────────────────────────────────
	 * Boat is downwind if |delta| > 90°.
	 * Dead-downwind is delta == ±180.
	 * Remap: gybe_delta = delta − 180 (or + 180 if negative), so dead-downwind → 0.
	 */
	if (fabsf(delta) > 90.0f) {
		float gybe_delta = (delta > 0.0f) ? (delta - 180.0f) : (delta + 180.0f);

		if (fabsf(gybe_delta) >= DEADBAND_DEG) {
			int8_t new_side = (gybe_delta < 0.0f) ? -1 : 1;

			if (downwind_side != 0 && new_side != downwind_side) {
				gybe_pending = true;
			}
			downwind_side = new_side;
		}
	} else {
		downwind_side = 0;
	}
}

float nav_get_drift_deg(void)
{
	return last_drift_deg;
}

bool nav_tack_detected(void)
{
	bool result  = tack_pending;
	tack_pending = false;
	return result;
}

bool nav_gybe_detected(void)
{
	bool result   = gybe_pending;
	gybe_pending  = false;
	return result;
}

uint32_t nav_compute_vmg(uint32_t speed_mm_s, uint32_t heading_mdeg, float wind_dir_deg)
{
	float heading_deg = MDEG_TO_DEG(heading_mdeg);
	float angle_rad   = DEG_TO_RAD(heading_deg - wind_dir_deg);
	float vmg         = (float)speed_mm_s * fabsf(cosf(angle_rad));

	return (uint32_t)vmg;
}

int32_t nav_get_differential(uint32_t my_vmg_mm_s, uint32_t other_vmg_mm_s)
{
	return (int32_t)my_vmg_mm_s - (int32_t)other_vmg_mm_s;
}
