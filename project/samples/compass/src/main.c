/*
 * Tilt-compensated compass — LSM6DSOTR (accel/gyro) + LIS3MDLTR (mag)
 *
 * Hardcoded calibration:
 *   - Gyro zero-rate bias (from acc_calibration sample)
 *   - Mag hard-iron offset + soft-iron matrix (from mag_calibration sample)
 *
 * Roll and pitch are computed from a complementary filter (accel + gyro).
 * The mag vector is tilt-compensated using roll/pitch before computing heading.
 *
 * Configure IMU_TO_MAG_ROT below to match the physical mounting of your sensors.
 *
 * Output (20 Hz): "AHR:<roll>,<pitch>,<heading>  degrees"
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

#define M_PI  3.14159265f
#define D2R   (M_PI / 180.0f)
#define R2D   (180.0f / M_PI)

/* ============================================================
 * SENSOR ORIENTATION CONFIGURATION
 *
 * IMU_TO_MAG_ROT is a 3x3 rotation matrix that transforms a
 * vector from the LSM6DSO (accel/gyro) frame into the LIS3MDLTR
 * (mag) frame.  Set this to match how the two chips are mounted
 * relative to each other on your PCB.
 *
 * Common Z-axis presets (chip top-side up, rotated around Z):
 *
 *   0° (aligned):
 *     { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }
 *
 *   90° CCW (IMU rotated 90° relative to mag):
 *     { {0, 1, 0}, {-1, 0, 0}, {0, 0, 1} }
 *
 *   180°:
 *     { {-1, 0, 0}, {0, -1, 0}, {0, 0, 1} }
 *
 *   270° CCW (= 90° CW):
 *     { {0, -1, 0}, {1, 0, 0}, {0, 0, 1} }
 *
 *   IMU mounted upside-down (flipped around X, then 0° Z):
 *     { {1, 0, 0}, {0, -1, 0}, {0, 0, -1} }
 * ============================================================ */
static const float IMU_TO_MAG_ROT[3][3] = {
	{-1.0f,  0.0f,  0.0f},
	{0.0f,  -1.0f,  0.0f},
	{0.0f,  0.0f,  1.0f},
};

/* ============================================================
 * GYRO ZERO-RATE BIAS  (from acc_calibration, in mdps)
 * Convert: mdps * (pi/180) / 1000 = rad/s
 * ============================================================ */
static const float GYRO_BIAS_X =  288.0f * D2R / 1000.0f;
static const float GYRO_BIAS_Y = -215.0f * D2R / 1000.0f;
static const float GYRO_BIAS_Z = -599.0f * D2R / 1000.0f;

/* ============================================================
 * MAG CALIBRATION  (from mag_calibration sample, in µT)
 * Hard-iron offset applied first, then soft-iron matrix.
 * ============================================================ */
static const float MAG_HARD_IRON[3] = {-49.20f, 49.96f, -27.15f};

static const float MAG_SOFT_IRON[3][3] = {
	{+0.979f, +0.044f,  -0.0015f},
	{+0.044f, +1.005f,  -0.004f },
	{-0.015f, -0.004f,  +1.019f },
};

/* ============================================================
 * COMPLEMENTARY FILTER
 * ALPHA: how much to trust the gyro vs accel (0–1).
 * Higher = smoother but slower to correct drift.
 * ============================================================ */
#define ALPHA  0.96f
#define DT_MS  50
#define DT_S   (DT_MS / 1000.0f)

/* ---------------------------------------------------------- */

static const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));
static const struct device *mag_dev = DEVICE_DT_GET(DT_NODELABEL(lis3mdl));

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

static int read_mag(float *mx, float *my, float *mz)
{
	struct sensor_value v[3];

	if (sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_MAGN_XYZ)) {
		return -EIO;
	}

	sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_X, &v[0]);
	sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Y, &v[1]);
	sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_Z, &v[2]);

	/* Gauss -> µT */
	*mx = sv(&v[0]) * 100.0f;
	*my = sv(&v[1]) * 100.0f;
	*mz = sv(&v[2]) * 100.0f;
	return 0;
}

/* Apply a 3x3 matrix to a vector in-place */
static void mat3_apply(const float m[3][3],
		       float ix, float iy, float iz,
		       float *ox, float *oy, float *oz)
{
	*ox = m[0][0]*ix + m[0][1]*iy + m[0][2]*iz;
	*oy = m[1][0]*ix + m[1][1]*iy + m[1][2]*iz;
	*oz = m[2][0]*ix + m[2][1]*iy + m[2][2]*iz;
}

/* ---------------------------------------------------------- */

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
	if (!device_is_ready(mag_dev)) {
		printk("LIS3MDL not ready\n");
		return -ENODEV;
	}

	struct sensor_value odr = { .val1 = 20, .val2 = 0 };

	sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	printk("Tilt-compensated compass starting...\n");

	float ax, ay, az, gx, gy, gz, mx, my, mz;
	float roll = 0.0f, pitch = 0.0f;

	/* Seed roll/pitch from first accel reading */
	if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
		float ax_m, ay_m, az_m;

		mat3_apply(IMU_TO_MAG_ROT, ax, ay, az, &ax_m, &ay_m, &az_m);
		roll  = atan2f(ay_m, az_m);
		pitch = atan2f(-ax_m, sqrtf(ay_m*ay_m + az_m*az_m));
	}

	while (1) {
		if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) ||
		    read_mag(&mx, &my, &mz)) {
			k_msleep(DT_MS);
			continue;
		}

		/* Remove gyro bias */
		gx -= GYRO_BIAS_X;
		gy -= GYRO_BIAS_Y;
		gz -= GYRO_BIAS_Z;

		/* Rotate accel and gyro into the mag frame */
		float ax_m, ay_m, az_m;
		float gx_m, gy_m, gz_m;

		mat3_apply(IMU_TO_MAG_ROT, ax, ay, az, &ax_m, &ay_m, &az_m);
		mat3_apply(IMU_TO_MAG_ROT, gx, gy, gz, &gx_m, &gy_m, &gz_m);

		/* Accel-derived roll and pitch (radians) */
		float roll_acc  = atan2f(ay_m, az_m);
		float pitch_acc = atan2f(-ax_m, sqrtf(ay_m*ay_m + az_m*az_m));

		/* Complementary filter */
		roll  = ALPHA * (roll  + gx_m * DT_S) + (1.0f - ALPHA) * roll_acc;
		pitch = ALPHA * (pitch + gy_m * DT_S) + (1.0f - ALPHA) * pitch_acc;

		/* Apply mag hard-iron offset */
		mx -= MAG_HARD_IRON[0];
		my -= MAG_HARD_IRON[1];
		mz -= MAG_HARD_IRON[2];

		/* Apply mag soft-iron matrix */
		float cx, cy, cz;

		mat3_apply(MAG_SOFT_IRON, mx, my, mz, &cx, &cy, &cz);

		/* Tilt compensation: project mag onto the horizontal plane */
		float cos_r = cosf(roll);
		float sin_r = sinf(roll);
		float cos_p = cosf(pitch);
		float sin_p = sinf(pitch);

		float mx_h = cx * cos_p + cz * sin_p;
		float my_h = cx * sin_r * sin_p + cy * cos_r - cz * sin_r * cos_p;

		/* Heading [0, 360) */
		float heading = atan2f(-my_h, mx_h) * R2D;

		if (heading < 0.0f) {
			heading += 360.0f;
		}

		printk("AHR:%8.2f,%8.2f,%8.2f\n",
		       (double)(roll  * R2D),
		       (double)(pitch * R2D),
		       (double)heading);

		k_msleep(DT_MS);
	}

	return 0;
}
