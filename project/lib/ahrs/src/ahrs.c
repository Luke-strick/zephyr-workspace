/*
 * ahrs — Attitude and Heading Reference System
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ahrs.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <string.h>

#define M_PI_F  3.14159265f
#define D2R     (M_PI_F / 180.0f)
#define R2D     (180.0f / M_PI_F)

#define ALPHA   (CONFIG_AHRS_ALPHA_X1000 / 1000.0f)
#define DT_S    (CONFIG_AHRS_DT_MS / 1000.0f)
#define DEADBAND (CONFIG_AHRS_GYRO_DEADBAND_MRAD / 1000.0f)  /* rad/s */

/* ── State ───────────────────────────────────────────────────────────────── */

static const struct device *imu_dev;
static const struct device *mag_dev;

/* Gyro bias in rad/s */
static float gyro_bias[3];

/* Mag calibration */
static float hard_iron[3];
static float soft_iron[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f},
};

/* IMU → mag frame rotation */
static float imu_to_mag[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f},
};

/* Filter state */
static float roll;
static float pitch;
static float heading;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static inline float sensor_val_to_float(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
}

static void mat3_apply(const float m[3][3],
		       float ix, float iy, float iz,
		       float *ox, float *oy, float *oz)
{
	*ox = m[0][0]*ix + m[0][1]*iy + m[0][2]*iz;
	*oy = m[1][0]*ix + m[1][1]*iy + m[1][2]*iz;
	*oz = m[2][0]*ix + m[2][1]*iy + m[2][2]*iz;
}

static inline float deadband(float v)
{
	return (v > DEADBAND || v < -DEADBAND) ? v : 0.0f;
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

	*ax = sensor_val_to_float(&a[0]);
	*ay = sensor_val_to_float(&a[1]);
	*az = sensor_val_to_float(&a[2]);
	*gx = sensor_val_to_float(&g[0]);
	*gy = sensor_val_to_float(&g[1]);
	*gz = sensor_val_to_float(&g[2]);

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

	/* Gauss → µT */
	*mx = sensor_val_to_float(&v[0]) * 100.0f;
	*my = sensor_val_to_float(&v[1]) * 100.0f;
	*mz = sensor_val_to_float(&v[2]) * 100.0f;

	return 0;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int ahrs_init(const struct device *imu, const struct device *mag)
{
	if (!device_is_ready(imu) || !device_is_ready(mag)) {
		return -ENODEV;
	}

	imu_dev = imu;
	mag_dev = mag;

	/* Set sensor ODR to match the configured sample period */
	int odr_hz = 1000 / CONFIG_AHRS_DT_MS;
	struct sensor_value odr = { .val1 = odr_hz, .val2 = 0 };

	sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* Seed roll/pitch from the first accel reading */
	float ax, ay, az, gx, gy, gz;

	if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
		float ax_m, ay_m, az_m;

		mat3_apply(imu_to_mag, ax, ay, az, &ax_m, &ay_m, &az_m);
		roll  = atan2f(ay_m, az_m);
		pitch = atan2f(-ax_m, sqrtf(ay_m*ay_m + az_m*az_m));
	}

	return 0;
}

void ahrs_set_gyro_bias(float x_mdps, float y_mdps, float z_mdps)
{
	gyro_bias[0] = x_mdps * D2R / 1000.0f;
	gyro_bias[1] = y_mdps * D2R / 1000.0f;
	gyro_bias[2] = z_mdps * D2R / 1000.0f;
}

void ahrs_set_mag_calibration(const float hi[3], const float si[3][3])
{
	memcpy(hard_iron, hi, sizeof(hard_iron));
	memcpy(soft_iron, si, sizeof(soft_iron));
}

void ahrs_set_imu_to_mag_rotation(const float rot[3][3])
{
	memcpy(imu_to_mag, rot, sizeof(imu_to_mag));
}

int ahrs_update(void)
{
	float ax, ay, az, gx, gy, gz, mx, my, mz;

	if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) ||
	    read_mag(&mx, &my, &mz)) {
		return -EIO;
	}

	/* Remove gyro bias and apply deadband */
	gx = deadband(gx - gyro_bias[0]);
	gy = deadband(gy - gyro_bias[1]);
	gz = deadband(gz - gyro_bias[2]);

	/* Rotate accel and gyro into the mag frame */
	float ax_m, ay_m, az_m;
	float gx_m, gy_m, gz_m;

	mat3_apply(imu_to_mag, ax, ay, az, &ax_m, &ay_m, &az_m);
	mat3_apply(imu_to_mag, gx, gy, gz, &gx_m, &gy_m, &gz_m);

	/* Accel-derived roll and pitch */
	float roll_acc  = atan2f(ay_m, az_m);
	float pitch_acc = atan2f(-ax_m, sqrtf(ay_m*ay_m + az_m*az_m));

	/* Complementary filter */
	roll  = ALPHA * (roll  + gx_m * DT_S) + (1.0f - ALPHA) * roll_acc;
	pitch = ALPHA * (pitch + gy_m * DT_S) + (1.0f - ALPHA) * pitch_acc;

	/* Mag hard-iron correction */
	mx -= hard_iron[0];
	my -= hard_iron[1];
	mz -= hard_iron[2];

	/* Mag soft-iron correction */
	float cx, cy, cz;

	mat3_apply(soft_iron, mx, my, mz, &cx, &cy, &cz);

	/* Tilt compensation: project mag onto the horizontal plane */
	float cos_r = cosf(roll);
	float sin_r = sinf(roll);
	float cos_p = cosf(pitch);
	float sin_p = sinf(pitch);

	float mx_h = cx * cos_p + cz * sin_p;
	float my_h = cx * sin_r * sin_p + cy * cos_r - cz * sin_r * cos_p;

	/* Heading [0, 360) */
	heading = atan2f(-my_h, mx_h) * R2D;
	if (heading < 0.0f) {
		heading += 360.0f;
	}

	return 0;
}

void ahrs_get(float *roll_deg, float *pitch_deg, float *heading_deg)
{
	if (roll_deg)    { *roll_deg    = roll    * R2D; }
	if (pitch_deg)   { *pitch_deg   = pitch   * R2D; }
	if (heading_deg) { *heading_deg = heading; }
}
