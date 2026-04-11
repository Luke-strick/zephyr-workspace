/*
 * ahrs — Attitude and Heading Reference System
 *
 * Combines an LSM6DSO accel/gyro with a LIS3MDL magnetometer to produce
 * roll, pitch, and tilt-compensated magnetic heading.
 *
 * Usage:
 *   1. ahrs_init(imu_dev, mag_dev)
 *   2. ahrs_set_gyro_bias(...)          // from acc_calibration sample
 *   3. ahrs_set_mag_calibration(...)    // from mag_calibration sample
 *   4. ahrs_set_imu_to_mag_rotation(...)// match PCB mounting
 *   5. Call ahrs_update() at CONFIG_AHRS_DT_MS interval
 *   6. ahrs_get(&roll, &pitch, &heading)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AHRS_H
#define AHRS_H

#include <zephyr/device.h>

/**
 * Initialise the library.
 *
 * Sets the sensor ODR to match CONFIG_AHRS_DT_MS and seeds roll/pitch
 * from the first accelerometer reading.
 *
 * @param imu  LSM6DSO device (accel + gyro).
 * @param mag  LIS3MDL device (magnetometer).
 * @return 0 on success, negative errno on failure.
 */
int ahrs_init(const struct device *imu, const struct device *mag);

/**
 * Set the gyro zero-rate bias (from acc_calibration sample).
 *
 * @param x_mdps  Bias on X axis in milli-degrees per second.
 * @param y_mdps  Bias on Y axis in milli-degrees per second.
 * @param z_mdps  Bias on Z axis in milli-degrees per second.
 */
void ahrs_set_gyro_bias(float x_mdps, float y_mdps, float z_mdps);

/**
 * Set the magnetometer calibration (from mag_calibration sample).
 *
 * @param hard_iron   3-element offset vector in µT.
 * @param soft_iron   3×3 correction matrix (dimensionless).
 */
void ahrs_set_mag_calibration(const float hard_iron[3],
			      const float soft_iron[3][3]);

/**
 * Set the rotation matrix from the IMU frame to the mag frame.
 *
 * This accounts for the physical mounting difference between the
 * LSM6DSO and LIS3MDL on the PCB.
 *
 * Common presets (Z-axis rotation only):
 *   0°:   { {1,0,0}, {0,1,0}, {0,0,1} }
 *   90°:  { {0,1,0}, {-1,0,0}, {0,0,1} }
 *   180°: { {-1,0,0}, {0,-1,0}, {0,0,1} }
 *   270°: { {0,-1,0}, {1,0,0}, {0,0,1} }
 *
 * @param rot  3×3 rotation matrix.
 */
void ahrs_set_imu_to_mag_rotation(const float rot[3][3]);

/**
 * Read sensors and advance the complementary filter by one step.
 *
 * Call this at CONFIG_AHRS_DT_MS intervals.
 *
 * @return 0 on success, -EIO if a sensor read fails (state unchanged).
 */
int ahrs_update(void);

/**
 * Retrieve the latest attitude and heading estimates.
 *
 * @param roll_deg     Roll angle in degrees  (positive = right side down).
 * @param pitch_deg    Pitch angle in degrees (positive = nose up).
 * @param heading_deg  Magnetic heading in degrees [0, 360).
 */
void ahrs_get(float *roll_deg, float *pitch_deg, float *heading_deg);

/* ── Real-time magnetometer calibration ─────────────────────────────────── */

/** Maximum samples the calibration buffer holds (configurable via Kconfig). */
#ifndef CONFIG_AHRS_CAL_MAX_SAMPLES
#define CONFIG_AHRS_CAL_MAX_SAMPLES 256
#endif

/** Angular resolution for gap detection (degrees, must divide 360). */
#define AHRS_CAL_GAP_BIN_DEG 5
#define AHRS_CAL_GAP_BINS    (360 / AHRS_CAL_GAP_BIN_DEG)

/**
 * Real-time calibration quality metrics.
 *
 * After each sample the caller can query these to decide whether to
 * keep the calibration or restart.
 *
 * Typical acceptance thresholds:
 *   gap       < 10 %      (most of the sphere covered)
 *   wobble    < 15 µT     (device kept reasonably level)
 *   fit_error <  3 µT     (corrected points form a clean sphere)
 */
struct ahrs_cal_quality {
	float    hard_iron[3];     /**< Hard-iron offset estimate (µT).       */
	float    soft_iron[3][3];  /**< Soft-iron correction matrix.          */
	float    gap_pct;          /**< Uncovered area (%).  Starts at 100,
	                                decreases toward 0 as the sphere
	                                fills in.                              */
	float    variance;         /**< Radius variance (µT²).                */
	float    wobble;           /**< Z-axis std-dev after correction (µT). */
	float    fit_error;        /**< RMS radius residual (µT).             */
	uint32_t sample_count;     /**< Samples collected so far.             */
};

/**
 * Start (or restart) a magnetometer calibration session.
 *
 * Clears the sample buffer and all accumulators.  The active AHRS
 * calibration is *not* touched until ahrs_cal_commit() is called.
 */
void ahrs_cal_start(void);

/**
 * Feed one raw magnetometer reading into the calibration engine.
 *
 * Call this each time the magnetometer is read while the user rotates
 * the device.  Samples beyond CONFIG_AHRS_CAL_MAX_SAMPLES are ignored.
 *
 * @param mx  Raw X-axis reading (µT).
 * @param my  Raw Y-axis reading (µT).
 * @param mz  Raw Z-axis reading (µT).
 */
void ahrs_cal_add_sample(float mx, float my, float mz);

/**
 * Read the magnetometer and feed the sample into the calibration buffer.
 *
 * Convenience wrapper: reads raw mag via the device registered in
 * ahrs_init(), converts to µT, and calls ahrs_cal_add_sample().
 * ahrs_init() must have been called first.
 *
 * @return 0 on success, -EIO on sensor failure.
 */
int ahrs_cal_collect(void);

/**
 * Compute and return the current calibration quality.
 *
 * Safe to call repeatedly — no side effects.  Results are only
 * meaningful once a reasonable number of samples have been collected
 * (≥ 20 or so).
 *
 * @param[out] q  Filled with the latest quality metrics.
 */
void ahrs_cal_get_quality(struct ahrs_cal_quality *q);

/**
 * Apply the current calibration estimate to the AHRS filter.
 *
 * Calls ahrs_set_mag_calibration() with the computed hard-iron and
 * soft-iron values.  Only call when the quality metrics are acceptable.
 */
void ahrs_cal_commit(void);

#endif /* AHRS_H */
