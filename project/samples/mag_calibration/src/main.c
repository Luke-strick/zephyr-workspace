/*
 * Magnetometer calibration — LIS3MDLTR
 *
 * Collects raw magnetometer samples while the user rotates the device,
 * computes hard-iron + soft-iron calibration with real-time quality
 * metrics (gap, variance, wobble, fit error).
 *
 * Output (20 Hz via RTT):
 *   "Raw:<mx>,<my>,<mz>"           — raw reading (µT × 10, integers)
 *   "CAL:<hx>,<hy>,<hz>"           — hard-iron offset (µT, floats)
 *   "SI:<s00>,<s11>,<s22>"         — soft-iron diagonal (floats)
 *   "Q:<gap>,<var>,<wob>,<fit>,<n>" — quality metrics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device_runtime.h>
#include <math.h>
#include <ahrs.h>

static const struct device *mag_dev = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));

static inline float sv(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
}

int main(void)
{
	const struct device *pwr_3v3 = DEVICE_DT_GET(DT_NODELABEL(power_3v3));

	pm_device_runtime_get(pwr_3v3);

	if (!device_is_ready(mag_dev)) {
		printk("LIS3MDL not ready\n");
		return -ENODEV;
	}

	/* Start a fresh calibration session */
	ahrs_cal_start();

	printk("Rotate device slowly in all directions...\n");

	struct sensor_value v[3];
	struct ahrs_cal_quality q;
	uint32_t tick = 0;

	while (1) {
		if (sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_MAGN_XYZ) == 0) {
			sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &v[0]);
			sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &v[1]);
			sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &v[2]);

			/* Gauss → µT */
			float rx = sv(&v[0]) * 100.0f;
			float ry = sv(&v[1]) * 100.0f;
			float rz = sv(&v[2]) * 100.0f;

			/* Feed into calibration engine */
			ahrs_cal_add_sample(rx, ry, rz);

			/* Print raw reading */
			printk("Raw:%d,%d,%d\n",
			       (int)(rx * 10.0f),
			       (int)(ry * 10.0f),
			       (int)(rz * 10.0f));

			/* Print quality metrics every 10 samples (2× per second) */
			tick++;
			if (tick % 10 == 0) {
				ahrs_cal_get_quality(&q);

				printk("CAL:%8.2f,%8.2f,%8.2f\n",
				       (double)q.hard_iron[0],
				       (double)q.hard_iron[1],
				       (double)q.hard_iron[2]);

				printk("SI:%6.3f,%6.3f,%6.3f\n",
				       (double)q.soft_iron[0][0],
				       (double)q.soft_iron[1][1],
				       (double)q.soft_iron[2][2]);

				printk("Q:gap=%.0f%%,var=%.2f,wob=%.2f,"
				       "fit=%.2f,n=%u\n",
				       (double)q.gap_pct,
				       (double)q.variance,
				       (double)q.wobble,
				       (double)q.fit_error,
				       q.sample_count);
			}
		}

		k_msleep(50);
	}

	return 0;
}
