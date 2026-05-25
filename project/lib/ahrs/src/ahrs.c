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

	/* Seed roll/pitch from the first accel reading.
	 * IMU: X=up, Y=right, Z=toward (bow).  Nav frame: X=fwd, Y=right, Z=up. */
	float ax, ay, az, gx, gy, gz;

	if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
		roll  = atan2f(ay, ax);
		pitch = atan2f(-az, sqrtf(ay*ay + ax*ax));
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

int ahrs_update(void)
{
	float ax, ay, az, gx, gy, gz, mx, my, mz;

	if (read_imu(&ax, &ay, &az, &gx, &gy, &gz) ||
	    read_mag(&mx, &my, &mz)) {
		return -EIO;
	}

	ARG_UNUSED(gx); ARG_UNUSED(gy); ARG_UNUSED(gz);

	/* ── Attitude ────────────────────────────────────────────────────────
	 * IMU physical axes: X=up, Y=right, Z=toward (bow).
	 * Nav frame used internally: X=fwd(bow), Y=right, Z=up.
	 *   nav_Y = imu_Y,  nav_Z = imu_X
	 * roll > 0 = starboard (right) side down.
	 * pitch negative = bow up (convention matches heading formula below). */
	roll  = atan2f(ay, ax);
	pitch = atan2f(-az, sqrtf(ay*ay + ax*ax));

	/* ── Tilt-compensated heading ────────────────────────────────────────
	 * MAG physical axes: X=down, Y=left, Z=toward (bow).
	 * Map corrected mag to nav frame: X_nav=fwd=mag_Z, Y_nav=right=-mag_Y,
	 *   Z_nav=up=-mag_X. */
	mx -= hard_iron[0];
	my -= hard_iron[1];
	mz -= hard_iron[2];

	float cx, cy, cz;

	mat3_apply(soft_iron, mx, my, mz, &cx, &cy, &cz);

	/* Corrected mag in nav frame */
	float cx_n = cz;    /* toward → forward */
	float cy_n = -cy;   /* left   → right   */
	float cz_n = -cx;   /* down   → up      */

	float cos_r = cosf(roll);
	float sin_r = sinf(roll);
	float cos_p = cosf(pitch);
	float sin_p = sinf(pitch);

	float mx_h = cx_n * cos_p + cz_n * sin_p;
	float my_h = cx_n * sin_r * sin_p + cy_n * cos_r - cz_n * sin_r * cos_p;

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

/* ── Magnetometer calibration engine ────────────────────────────────────── */

#define CAL_MAX  CONFIG_AHRS_CAL_MAX_SAMPLES
#define CAL_BINS AHRS_CAL_GAP_BINS

static float    cal_buf[CAL_MAX][3];
static uint32_t cal_count;
static uint8_t  cal_gap_bins[CAL_BINS];

void ahrs_cal_start(void)
{
	cal_count = 0;
	memset(cal_gap_bins, 0, sizeof(cal_gap_bins));
}

void ahrs_cal_add_sample(float mx, float my, float mz)
{
	if (cal_count >= CAL_MAX) {
		return;
	}

	cal_buf[cal_count][0] = mx;
	cal_buf[cal_count][1] = my;
	cal_buf[cal_count][2] = mz;
	cal_count++;

	/* Update gap coverage bin from raw XY heading */
	float angle = atan2f(my, mx) * R2D;

	if (angle < 0.0f) {
		angle += 360.0f;
	}

	int bin = (int)(angle / AHRS_CAL_GAP_BIN_DEG);

	if (bin >= CAL_BINS) {
		bin = CAL_BINS - 1;
	}

	cal_gap_bins[bin] = 1;
}

int ahrs_cal_collect(void)
{
	float mx, my, mz;

	if (read_mag(&mx, &my, &mz)) {
		return -EIO;
	}

	ahrs_cal_add_sample(mx, my, mz);
	return 0;
}

void ahrs_cal_get_quality(struct ahrs_cal_quality *q)
{
	if (!q) {
		return;
	}

	memset(q, 0, sizeof(*q));
	q->sample_count = cal_count;

	if (cal_count < 2) {
		q->gap_pct = 100.0f;
		return;
	}

	/* ── Hard-iron: min/max centre ──────────────────────────────────── */
	float mn[3] = { cal_buf[0][0], cal_buf[0][1], cal_buf[0][2] };
	float mx[3] = { cal_buf[0][0], cal_buf[0][1], cal_buf[0][2] };

	for (uint32_t i = 1; i < cal_count; i++) {
		for (int a = 0; a < 3; a++) {
			if (cal_buf[i][a] < mn[a]) {
				mn[a] = cal_buf[i][a];
			}
			if (cal_buf[i][a] > mx[a]) {
				mx[a] = cal_buf[i][a];
			}
		}
	}

	for (int a = 0; a < 3; a++) {
		q->hard_iron[a] = (mn[a] + mx[a]) * 0.5f;
	}

	/* ── Soft-iron: per-axis scale to average radius ────────────────── */
	float range[3];
	float avg_range = 0.0f;

	for (int a = 0; a < 3; a++) {
		range[a] = mx[a] - mn[a];
		avg_range += range[a];
	}

	avg_range /= 3.0f;

	/* Start with identity */
	memset(q->soft_iron, 0, sizeof(q->soft_iron));

	for (int a = 0; a < 3; a++) {
		q->soft_iron[a][a] = (range[a] > 1e-6f)
				   ? (avg_range / range[a])
				   : 1.0f;
	}

	/* ── Apply correction and gather statistics ─────────────────────── */
	float sum_r  = 0.0f;
	float sum_r2 = 0.0f;
	float sum_z  = 0.0f;
	float sum_z2 = 0.0f;

	for (uint32_t i = 0; i < cal_count; i++) {
		/* Hard-iron subtract */
		float cx = cal_buf[i][0] - q->hard_iron[0];
		float cy = cal_buf[i][1] - q->hard_iron[1];
		float cz = cal_buf[i][2] - q->hard_iron[2];

		/* Soft-iron scale */
		cx *= q->soft_iron[0][0];
		cy *= q->soft_iron[1][1];
		cz *= q->soft_iron[2][2];

		float r = sqrtf(cx * cx + cy * cy + cz * cz);

		sum_r  += r;
		sum_r2 += r * r;
		sum_z  += cz;
		sum_z2 += cz * cz;
	}

	float n      = (float)cal_count;
	float mean_r = sum_r / n;
	float mean_z = sum_z / n;

	/* Variance of radius (µT²) */
	q->variance = (sum_r2 / n) - (mean_r * mean_r);
	if (q->variance < 0.0f) {
		q->variance = 0.0f;
	}

	/* Fit error: RMS radius residual (µT) */
	q->fit_error = sqrtf(q->variance);

	/* Wobble: std-dev of corrected Z (µT) */
	float var_z = (sum_z2 / n) - (mean_z * mean_z);

	if (var_z < 0.0f) {
		var_z = 0.0f;
	}

	q->wobble = sqrtf(var_z);

	/* ── Gap: percentage of uncovered bins (100 % → 0 %) ──────────── */
	uint32_t empty = 0;

	for (int i = 0; i < CAL_BINS; i++) {
		if (cal_gap_bins[i] == 0) {
			empty++;
		}
	}

	q->gap_pct = (float)empty * 100.0f / (float)CAL_BINS;
}

void ahrs_cal_commit(void)
{
	struct ahrs_cal_quality q;

	ahrs_cal_get_quality(&q);
	ahrs_set_mag_calibration(q.hard_iron, q.soft_iron);
}

/* ── IMU static calibration ─────────────────────────────────────────────── */

static double   imu_cal_sum[6];   /* ax, ay, az, gx, gy, gz accumulator */
static uint32_t imu_cal_count;

void ahrs_imu_cal_start(void)
{
	imu_cal_count = 0;
	memset(imu_cal_sum, 0, sizeof(imu_cal_sum));
}

int ahrs_imu_cal_collect(void)
{
	float ax, ay, az, gx, gy, gz;

	if (read_imu(&ax, &ay, &az, &gx, &gy, &gz)) {
		return -EIO;
	}

	imu_cal_sum[0] += ax;
	imu_cal_sum[1] += ay;
	imu_cal_sum[2] += az;
	imu_cal_sum[3] += gx;
	imu_cal_sum[4] += gy;
	imu_cal_sum[5] += gz;
	imu_cal_count++;
	return 0;
}

uint32_t ahrs_imu_cal_get_count(void)
{
	return imu_cal_count;
}

void ahrs_imu_cal_commit(float *gyro_x_mdps, float *gyro_y_mdps, float *gyro_z_mdps)
{
	if (imu_cal_count == 0) {
		return;
	}

	double n      = (double)imu_cal_count;
	float gx      = (float)(imu_cal_sum[3] / n);
	float gy      = (float)(imu_cal_sum[4] / n);
	float gz      = (float)(imu_cal_sum[5] / n);
	float gx_mdps = gx * R2D * 1000.0f;
	float gy_mdps = gy * R2D * 1000.0f;
	float gz_mdps = gz * R2D * 1000.0f;

	ahrs_set_gyro_bias(gx_mdps, gy_mdps, gz_mdps);

	if (gyro_x_mdps) { *gyro_x_mdps = gx_mdps; }
	if (gyro_y_mdps) { *gyro_y_mdps = gy_mdps; }
	if (gyro_z_mdps) { *gyro_z_mdps = gz_mdps; }
}
