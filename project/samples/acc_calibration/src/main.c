/*
 * Gyroscope calibration — LSM6DSO
 *
 * Phase 1 — CALIBRATION (16 seconds, device must be still):
 *   Averages raw gyro readings to find zero-rate bias.
 *   Prints bias in mdps.
 *
 * Phase 2 — STREAMING at 20 Hz:
 *   Applies gyro bias, integrates to yaw.
 *   Format: "RPY:0,0,<yaw>\n"  (degrees * 100, integer)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/pm/device_runtime.h>
#include <math.h>

#define M_PI 3.14159265f

#define CAL_DURATION_MS 6000
#define DT_MS  50
#define DT_S   (DT_MS / 1000.0f)

static const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));

static inline float sv(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
}

static int read_imu(float *ax, float *ay, float *az,
		    float *gx, float *gy, float *gz)
{
	struct sensor_value a[3], g[3];

	if (sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ) ||
	    sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ)) {
		return -EIO;
	}

	sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &a[0]);
	sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &a[1]);
	sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &a[2]);
	sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X,  &g[0]);
	sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y,  &g[1]);
	sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z,  &g[2]);

	*ax = sv(&a[0]); *ay = sv(&a[1]); *az = sv(&a[2]);
	*gx = sv(&g[0]); *gy = sv(&g[1]); *gz = sv(&g[2]);
	return 0;
}

int main(void)
{
	const struct device *pwr_3v3 = DEVICE_DT_GET(DT_NODELABEL(power_3v3));

	pm_device_runtime_get(pwr_3v3);

	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	if (device_is_ready(usb_dev)) {
		usb_enable(NULL);
		uint32_t dtr = 0;

		while (!dtr) {
			uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
			k_sleep(K_MSEC(100));
		}
	}

	if (!device_is_ready(imu_dev)) {
		printk("LSM6DSO not ready\n");
		return -ENODEV;
	}

	struct sensor_value odr = { .val1 = 20, .val2 = 0 };

	sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* ---- Phase 1: gyro bias calibration ---- */
	printk("Keep device STILL for %d seconds...\n", CAL_DURATION_MS / 1000);

	float ax, ay, az, gx, gy, gz;
	float sum_gx = 0, sum_gy = 0, sum_gz = 0;
	int n = 0;

	int64_t end = k_uptime_get() + CAL_DURATION_MS;

	while (k_uptime_get() < end) {
		if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
			sum_gx += gx; sum_gy += gy; sum_gz += gz;
			n++;
		}
		k_msleep(DT_MS);
	}

	float bias_gx = sum_gx / n;
	float bias_gy = sum_gy / n;
	float bias_gz = sum_gz / n;

	printk("Calibration done (%d samples)\n", n);
	printk("Gyro bias (mdps): %d %d %d\n",
	       (int)(bias_gx * 1000.0f * 180.0f / M_PI),
	       (int)(bias_gy * 1000.0f * 180.0f / M_PI),
	       (int)(bias_gz * 1000.0f * 180.0f / M_PI));

	/* ---- Phase 2: streaming ---- */
	float yaw = 0.0f;

	printk("Streaming RPY at 20 Hz...\n");

	while (1) {
		if (read_imu(&ax, &ay, &az, &gx, &gy, &gz)) {
			k_msleep(DT_MS);
			continue;
		}

		gx -= bias_gx; gy -= bias_gy; gz -= bias_gz;

		/* Roll and pitch from accel */
		float roll  = atan2f(ay, az) * (180.0f / M_PI);
		float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

		/* Yaw from gyro integration with deadband */
		if (gz > 0.001f || gz < -0.001f) {
			yaw += gz * (180.0f / M_PI) * DT_S;
		}
		if (yaw >  180.0f) { yaw -= 360.0f; }
		if (yaw < -180.0f) { yaw += 360.0f; }

		printk("RPY:%8.2f,%8.2f,%8.2f\n", (double)roll, (double)pitch, (double)yaw);

		k_msleep(DT_MS);
	}

	return 0;
}
