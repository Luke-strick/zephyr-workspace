/*
 * Magnetometer raw data streamer — LIS3MDLTR
 *
 * Prints "Raw:{mx},{my},{mz}" at 20 Hz over USB CDC ACM.
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

/* Fill with zeros before calibration attempt*/
static float hard_iron_offset[] = {-49.20f, 49.96f, -27.15f};
static float soft_iron_matrix[] = {
	0.979f, 0.044f, -0.0015f, 
	0.044f , 1.005f, -0.004f, 
	-0.015, -0.004f, 1.019f};

static const struct device *mag_dev = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));

static inline float sv(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
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

	if (!device_is_ready(mag_dev)) {
		printk("LIS3MDL not ready\n");
		return -ENODEV;
	}

	struct sensor_value v[3];

	while (1) {
		if (sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_MAGN_XYZ) == 0) {
			sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &v[0]);
			sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &v[1]);
			sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &v[2]);

			/* Convert Gauss -> µT */
			float rx = sv(&v[0]) * 100.0f;
			float ry = sv(&v[1]) * 100.0f;
			float rz = sv(&v[2]) * 100.0f;

			/* Hard-iron offset (µT) */
			rx -= hard_iron_offset[0];
			ry -= hard_iron_offset[1];
			rz -= hard_iron_offset[2];

			/* Soft-iron matrix */
			float cx = soft_iron_matrix[0] * rx + soft_iron_matrix[1]  * ry + soft_iron_matrix[2] * rz;
			float cy = soft_iron_matrix[3] * rx + soft_iron_matrix[4]  * ry + soft_iron_matrix[5] * rz;
			float cz = soft_iron_matrix[6] * rx + soft_iron_matrix[7]  * ry + soft_iron_matrix[8] * rz;

			int mx = (int)cx*10;
			int my = (int)cy*10;
			int mz = (int)cz*10;

			float heading = atan2f(-cy, cx) * (180.0f / (float)M_PI);

			if (heading < 0.0f) {
				heading += 360.0f;
			}

			int hdg = (int)heading;

			/* Used for calibration*/
			printk("Raw:0,0,0,0,0,0,%d,%d,%d\n", mx, my, mz);

			printk("Heading: %d", hdg);
		}

		k_msleep(50);
	}

	return 0;
}
