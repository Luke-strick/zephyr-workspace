/*
 * Tilt-compensated compass — LSM6DSOTR + LIS3MDLTR
 *
 * Press Button 1 (sw0) to start magnetometer calibration.
 * Rotate the device slowly in all orientations for ~15 seconds.
 * After calibration the console prints a continuous heading that
 * stays correct even when the board is tilted.
 *
 * Hard-iron + basic soft-iron (axis scaling) calibration is applied.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(compass, LOG_LEVEL_INF);

/* Calibration duration (ms) */
#define CAL_DURATION_MS 15000
/* Heading print interval (ms) */
#define HEADING_INTERVAL_MS 200

static const struct device *accel_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));
static const struct device *mag_dev   = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/* Calibration data */
static struct {
	float offset[3]; /* hard-iron offsets  */
	float scale[3];  /* soft-iron scaling  */
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

static int read_accel(float *ax, float *ay, float *az)
{
	struct sensor_value v[3];
	int ret;

	ret = sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ACCEL_XYZ);
	if (ret) {
		return ret;
	}
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &v[0]);
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &v[1]);
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &v[2]);

	*ax = sv(&v[0]);
	*ay = sv(&v[1]);
	*az = sv(&v[2]);
	return 0;
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
		}
		k_msleep(20);
	}

	/* Hard-iron offsets */
	for (int i = 0; i < 3; i++) {
		cal.offset[i] = (mag_max[i] + mag_min[i]) / 2.0f;
	}

	/* Soft-iron scale — normalise each axis range to the average range */
	float range[3], avg_range;

	for (int i = 0; i < 3; i++) {
		range[i] = (mag_max[i] - mag_min[i]) / 2.0f;
	}
	avg_range = (range[0] + range[1] + range[2]) / 3.0f;

	for (int i = 0; i < 3; i++) {
		cal.scale[i] = (range[i] > 1e-6f) ? (avg_range / range[i]) : 1.0f;
	}

	cal.valid = true;

	LOG_INF("=== CALIBRATION DONE ===");
	LOG_INF("Offsets: %.4f  %.4f  %.4f",
		(double)cal.offset[0], (double)cal.offset[1],
		(double)cal.offset[2]);
	LOG_INF("Scale:   %.4f  %.4f  %.4f",
		(double)cal.scale[0], (double)cal.scale[1],
		(double)cal.scale[2]);
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

static float compute_heading(float ax, float ay, float az,
			     float mx, float my, float mz)
{
	/* Apply calibration */
	if (cal.valid) {
		mx = (mx - cal.offset[0]) * cal.scale[0];
		my = (my - cal.offset[1]) * cal.scale[1];
		mz = (mz - cal.offset[2]) * cal.scale[2];
	}

	/* Roll and pitch from accelerometer */
	float roll  = atan2f(ay, az);
	float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

	/* Tilt-compensated magnetometer projection onto the horizontal plane */
	float cos_r = cosf(roll);
	float sin_r = sinf(roll);
	float cos_p = cosf(pitch);
	float sin_p = sinf(pitch);

	float mx_h = mx * cos_p + mz * sin_p;
	float my_h = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p;

	/* Heading in degrees [0, 360) */
	float heading = atan2f(-my_h, mx_h) * (180.0f / (float)M_PI);

	if (heading < 0.0f) {
		heading += 360.0f;
	}
	return heading;
}

/* ------------------------------------------------------------------ */

int main(void)
{
	float ax, ay, az, mx, my, mz;

	if (!device_is_ready(accel_dev)) {
		LOG_ERR("LSM6DSO not ready");
		return -ENODEV;
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

	LOG_INF("Tilt-compensated compass sample");
	LOG_INF("Press Button 1 to calibrate, then rotate the board.");
	LOG_INF("Readings will start immediately (uncalibrated until you press the button).\n");

	/* Start with identity calibration */
	cal.offset[0] = cal.offset[1] = cal.offset[2] = 0.0f;
	cal.scale[0]  = cal.scale[1]  = cal.scale[2]  = 1.0f;
	cal.valid = false;

	while (1) {
		if (cal_requested) {
			cal_requested = false;
			run_calibration();
		}

		if (read_accel(&ax, &ay, &az) || read_mag(&mx, &my, &mz)) {
			LOG_ERR("Sensor read failed");
			k_msleep(HEADING_INTERVAL_MS);
			continue;
		}

		float heading = compute_heading(ax, ay, az, mx, my, mz);

		LOG_INF("Heading: %6.1f°  %s   %s",
			(double)heading,
			heading_label(heading),
			cal.valid ? "[cal]" : "[uncal]");

		k_msleep(HEADING_INTERVAL_MS);
	}

	return 0;
}
