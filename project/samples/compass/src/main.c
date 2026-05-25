/*
 * Tilt-compensated compass — uses the ahrs library
 *
 * Set calibration values below to match your board.
 * Output (20 Hz): "AHR:<roll>,<pitch>,<heading>  degrees"
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device_runtime.h>
#include <ahrs.h>

/* ── Calibration — paste values from your calibration samples ────────────── */

/* Gyro zero-rate bias in milli-degrees per second (from acc_calibration) */
#define GYRO_BIAS_X_MDPS   288.0f
#define GYRO_BIAS_Y_MDPS  -215.0f
#define GYRO_BIAS_Z_MDPS  -599.0f

/* Mag hard-iron offset in µT (from mag_calibration) */
static const float hard_iron[3] = { -49.20f, 49.96f, -27.15f };

/* Mag soft-iron correction matrix (from mag_calibration) */
static const float soft_iron[3][3] = {
	{ +0.979f, +0.044f, -0.015f },
	{ +0.044f, +1.005f, -0.004f },
	{ -0.015f, -0.004f, +1.019f },
};

/* ─────────────────────────────────────────────────────────────────────────── */

int main(void)
{
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));

	const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));
	const struct device *mag = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));

	if (ahrs_init(imu, mag) < 0) {
		printk("Sensor init failed\n");
		return -ENODEV;
	}

	ahrs_set_gyro_bias(GYRO_BIAS_X_MDPS, GYRO_BIAS_Y_MDPS, GYRO_BIAS_Z_MDPS);
	ahrs_set_mag_calibration(hard_iron, soft_iron);

	printk("Tilt-compensated compass starting...\n");

	while (1) {
		if (ahrs_update() == 0) {
			float roll, pitch, heading;

			ahrs_get(&roll, &pitch, &heading);
			printk("AHR:%8.2f,%8.2f,%8.2f\n",
			       (double)roll, (double)pitch, (double)heading);
		}

		k_msleep(CONFIG_AHRS_DT_MS);
	}

	return 0;
}
