/*
 * Magnetometer calibration tool — LIS3MDLTR
 *
 * Calibrates for hard-iron and soft-iron errors, then prints a
 * flat (2D) heading to verify the result.  No accelerometer is used,
 * so the board must be kept level for accurate heading readings.
 *
 * Press Button 1 (sw0) at any time to re-run calibration.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>
#include <math.h>

#define M_PI 3.14159265f

LOG_MODULE_REGISTER(mag_cal, LOG_LEVEL_INF);

/* Calibration duration (ms) */
#define CAL_DURATION_MS 15000
/* Heading print interval (ms) */
#define HEADING_INTERVAL_MS 200

static const struct device *mag_dev = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/* Calibration data */
static struct {
	float offset[3]; /* hard-iron offsets */
	float scale[3];  /* soft-iron axis scaling */
	bool valid;
} cal;

static volatile bool cal_requested;
static struct gpio_callback btn_cb_data;

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	cal_requested = true;
}

/* ------------------------------------------------------------------ */

static inline float sv(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
}

static int read_mag(float *mx, float *my, float *mz)
{
	struct sensor_value v[3];
	int ret;

	ret = sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_MAGN_XYZ);
	if (ret) {
		return ret;
	}
	sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &v[0]);
	sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &v[1]);
	sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &v[2]);

	*mx = sv(&v[0]);
	*my = sv(&v[1]);
	*mz = sv(&v[2]);
	return 0;
}

/* ------------------------------------------------------------------ */

static void run_calibration(void)
{
	float mag_min[3] = { 1e9f,  1e9f,  1e9f};
	float mag_max[3] = {-1e9f, -1e9f, -1e9f};
	float m[3];
	int samples = 0;
	int64_t end;

	LOG_INF("=== CALIBRATION START ===");
	LOG_INF("Slowly rotate the device in all orientations for %d seconds.",
		CAL_DURATION_MS / 1000);

	end = k_uptime_get() + CAL_DURATION_MS;

	while (k_uptime_get() < end) {
		if (read_mag(&m[0], &m[1], &m[2]) == 0) {
			for (int i = 0; i < 3; i++) {
				if (m[i] < mag_min[i]) {
					mag_min[i] = m[i];
				}
				if (m[i] > mag_max[i]) {
					mag_max[i] = m[i];
				}
			}
			samples++;
		}
		k_msleep(20);
	}

	/* Hard-iron offsets: centre of the min/max box */
	for (int i = 0; i < 3; i++) {
		cal.offset[i] = (mag_max[i] + mag_min[i]) / 2.0f;
	}

	/* Soft-iron scale: normalise each axis range to the average range */
	float range[3], avg_range;

	for (int i = 0; i < 3; i++) {
		range[i] = (mag_max[i] - mag_min[i]) / 2.0f;
	}
	avg_range = (range[0] + range[1] + range[2]) / 3.0f;

	for (int i = 0; i < 3; i++) {
		cal.scale[i] = (range[i] > 1e-6f) ? (avg_range / range[i]) : 1.0f;
	}

	cal.valid = true;

	LOG_INF("=== CALIBRATION DONE  (%d samples) ===", samples);
	LOG_INF("Hard-iron offsets: %.4f  %.4f  %.4f",
		(double)cal.offset[0], (double)cal.offset[1],
		(double)cal.offset[2]);
	LOG_INF("Soft-iron scale:   %.4f  %.4f  %.4f",
		(double)cal.scale[0], (double)cal.scale[1],
		(double)cal.scale[2]);
	LOG_INF("Min: %.4f  %.4f  %.4f",
		(double)mag_min[0], (double)mag_min[1], (double)mag_min[2]);
	LOG_INF("Max: %.4f  %.4f  %.4f",
		(double)mag_max[0], (double)mag_max[1], (double)mag_max[2]);
	LOG_INF("Press Button 1 to re-calibrate at any time.\n");
}

/* ------------------------------------------------------------------ */

static const char *heading_label(float h)
{
	if (h >= 337.5f || h < 22.5f)  return "N";
	if (h < 67.5f)                 return "NE";
	if (h < 112.5f)                return "E";
	if (h < 157.5f)                return "SE";
	if (h < 202.5f)                return "S";
	if (h < 247.5f)                return "SW";
	if (h < 292.5f)                return "W";
	return "NW";
}

static float compute_heading(float mx, float my, float mz)
{
	/* Apply calibration */
	if (cal.valid) {
		mx = (mx - cal.offset[0]) * cal.scale[0];
		my = (my - cal.offset[1]) * cal.scale[1];
		mz = (mz - cal.offset[2]) * cal.scale[2];
	}

	/* Simple 2D heading — assumes device is level (X-Y is horizontal) */
	float heading = atan2f(-my, mx) * (180.0f / M_PI);

	if (heading < 0.0f) {
		heading += 360.0f;
	}
	return heading;
}

/* ------------------------------------------------------------------ */

int main(void)
{
	float mx, my, mz;

	/* Power on 3V3 rail (sensors) */
	const struct device *pwr_3v3 = DEVICE_DT_GET(DT_NODELABEL(power_3v3));

	pm_device_runtime_get(pwr_3v3);

	/* Enable USB CDC ACM console */
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
		LOG_ERR("LIS3MDL not ready");
		return -ENODEV;
	}

	/* Set up calibration button */
	if (gpio_is_ready_dt(&button)) {
		gpio_pin_configure_dt(&button, GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
		gpio_init_callback(&btn_cb_data, button_pressed, BIT(button.pin));
		gpio_add_callback(button.port, &btn_cb_data);
	}

	LOG_INF("Magnetometer calibration sample");
	LOG_INF("Starting calibration — rotate the board slowly in all directions.");
	LOG_INF("Keep the board LEVEL when reading heading (no tilt compensation).\n");

	/* Start with identity calibration */
	cal.offset[0] = cal.offset[1] = cal.offset[2] = 0.0f;
	cal.scale[0]  = cal.scale[1]  = cal.scale[2]  = 1.0f;
	cal.valid = false;

	run_calibration();

	while (1) {
		if (cal_requested) {
			cal_requested = false;
			run_calibration();
		}

		if (read_mag(&mx, &my, &mz)) {
			LOG_ERR("Sensor read failed");
			k_msleep(HEADING_INTERVAL_MS);
			continue;
		}

		float heading = compute_heading(mx, my, mz);

		LOG_INF("Heading: %6.1f  %s   [%s]  raw: %.3f %.3f %.3f",
			(double)heading,
			heading_label(heading),
			cal.valid ? "cal" : "uncal",
			(double)mx, (double)my, (double)mz);

		k_msleep(HEADING_INTERVAL_MS);
	}

	return 0;
}
