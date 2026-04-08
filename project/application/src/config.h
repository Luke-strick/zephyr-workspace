/*
 * config — Application configuration store
 *
 * Master config struct covering all subsystems. Persisted to NVS flash via
 * the Zephyr settings subsystem. Setters modify the live in-RAM struct;
 * call config_save() to persist to flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/* ── Enums ───────────────────────────────────────────────────────────────── */

typedef enum {
	DATA_RATE_1HZ  = 1,
	DATA_RATE_2HZ  = 2,
	DATA_RATE_4HZ  = 4,
	DATA_RATE_5HZ  = 5,
	DATA_RATE_10HZ = 10,
} data_rate_t;

typedef enum {
	DATA_SRC_GPS  = BIT(0),
	DATA_SRC_AHRS = BIT(1),
} data_src_flags_t;

typedef enum {
	ROW_SRC_HEADING_DEG = 0,
	ROW_SRC_SPEED_KT,
	ROW_SRC_ALTITUDE_M,
	ROW_SRC_ROLL_DEG,
	ROW_SRC_PITCH_DEG,
	ROW_SRC_DRIFT_DEG,
	ROW_SRC_COUNT,
} row_src_t;

/* ── Config structs ──────────────────────────────────────────────────────── */

struct lora_cfg {
	bool     enabled;
	uint8_t  boat_id;    /* 0–3 */
	uint8_t  n_boats;    /* 1–4 */
	uint8_t  tx_rate_hz; /* 1–5 */
	uint32_t frequency;  /* Hz */
	int8_t   tx_power;   /* dBm */
};

struct app_config {
	/* Data engine */
	data_rate_t      sample_rate_hz;
	data_src_flags_t data_sources;

	/* Display */
	row_src_t        row_src[3];

	/* LoRa */
	struct lora_cfg  lora;

	/* Wind */
	float            true_wind_dir_deg; /* direction wind comes FROM, 0–360 */

	/* AHRS calibration */
	float gyro_bias_x_mdps;
	float gyro_bias_y_mdps;
	float gyro_bias_z_mdps;
	float hard_iron[3];
	float soft_iron[3][3];
	float imu_to_mag_rot[3][3];
};

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialise the settings subsystem and load config from NVS.
 * Applies built-in defaults for any keys that are missing.
 * Must be called before config_get() or any setter.
 */
int config_init(void);

/** Return a const pointer to the live config struct. No locking required
 *  for reads — all writes come from the single display thread. */
const struct app_config *config_get(void);

/** Persist the current in-RAM config to NVS flash. */
int config_save(void);

/* Setters — modify live config; call config_save() to persist. */
void config_set_sample_rate(data_rate_t hz);
void config_set_data_sources(data_src_flags_t flags);
void config_set_row_src(int row, row_src_t src);
void config_set_lora_enabled(bool enabled);
void config_set_lora_boat_id(uint8_t id);
void config_set_lora_n_boats(uint8_t n);
void config_set_lora_tx_rate(uint8_t hz);
void config_set_lora_frequency(uint32_t freq_hz);
void config_set_true_wind_dir(float deg);
void config_set_ahrs_gyro_bias(float x, float y, float z);
void config_set_ahrs_mag_cal(const float hard[3], const float soft[3][3]);
void config_set_ahrs_rotation(const float rot[3][3]);

#endif /* CONFIG_H */
