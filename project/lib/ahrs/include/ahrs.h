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

#endif /* AHRS_H */
