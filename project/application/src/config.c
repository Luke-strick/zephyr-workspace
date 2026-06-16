/*
 * config — Application configuration store
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config.h"

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(config, LOG_LEVEL_INF);

/* ── Defaults ────────────────────────────────────────────────────────────── */


static const struct app_config cfg_defaults = {
	.sample_rate_hz = DATA_RATE_10HZ,
	.data_sources   = DATA_SRC_GPS | DATA_SRC_AHRS,
	.row_src        = ROW_SRC_ROLL_DEG,
	.box_src        = { BOX_SRC_SOG, BOX_SRC_COG, BOX_SRC_ROLL, BOX_SRC_TIMER },
	.timer_minutes  = 5,
	.lora = {
		.enabled    = false,
		.boat_id    = 0,
		.n_boats    = 1,
		.tx_rate_hz = 1,
		.frequency  = 915000000U,
		.tx_power   = 14,
	},
	.true_wind_dir_deg = 0.0f,
	.gyro_bias_x_mdps  = 288.0f,
	.gyro_bias_y_mdps  = -215.0f,
	.gyro_bias_z_mdps  = -599.0f,
	.hard_iron         = { -49.20f, 49.96f, -27.15f },
	.soft_iron         = {
		{ +1.0f, +0.0f, +0.0f },
		{ +0.0f, +1.0f, +0.0f },
		{ +0.0f, +0.0f, +1.0f },
	},
};

/* ── Live config ─────────────────────────────────────────────────────────── */

static struct app_config cfg;

/* ── Settings handler ────────────────────────────────────────────────────── */

static int h_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
#define LOAD(k, field)							\
	if (strcmp(key, k) == 0) {					\
		if (len != sizeof(cfg.field)) { return -EINVAL; }	\
		read_cb(cb_arg, &cfg.field, len);			\
		return 0;						\
	}

	LOAD("engine/rate_hz",   sample_rate_hz)
	LOAD("engine/sources",   data_sources)
	LOAD("disp/stat",        row_src)
	LOAD("disp/boxes",       box_src)
	LOAD("disp/timer_min",   timer_minutes)
	LOAD("lora/enabled",     lora.enabled)
	LOAD("lora/boat_id",     lora.boat_id)
	LOAD("lora/n_boats",     lora.n_boats)
	LOAD("lora/tx_rate_hz",  lora.tx_rate_hz)
	LOAD("lora/frequency",   lora.frequency)
	LOAD("lora/tx_power",    lora.tx_power)
	LOAD("wind/twd",         true_wind_dir_deg)
	LOAD("cal/gyro_x",       gyro_bias_x_mdps)
	LOAD("cal/gyro_y",       gyro_bias_y_mdps)
	LOAD("cal/gyro_z",       gyro_bias_z_mdps)
	LOAD("cal/hard_iron",    hard_iron)
	LOAD("cal/soft_iron",    soft_iron)

#undef LOAD
	return -ENOENT;
}

static int h_export(int (*export_fn)(const char *name, const void *val, size_t len))
{
#define SAVE(k, field) export_fn("tracker/" k, &cfg.field, sizeof(cfg.field))

	SAVE("engine/rate_hz",  sample_rate_hz);
	SAVE("engine/sources",  data_sources);
	SAVE("disp/stat",       row_src);
	SAVE("disp/boxes",      box_src);
	SAVE("disp/timer_min",  timer_minutes);
	SAVE("lora/enabled",    lora.enabled);
	SAVE("lora/boat_id",    lora.boat_id);
	SAVE("lora/n_boats",    lora.n_boats);
	SAVE("lora/tx_rate_hz", lora.tx_rate_hz);
	SAVE("lora/frequency",  lora.frequency);
	SAVE("lora/tx_power",   lora.tx_power);
	SAVE("wind/twd",        true_wind_dir_deg);
	SAVE("cal/gyro_x",      gyro_bias_x_mdps);
	SAVE("cal/gyro_y",      gyro_bias_y_mdps);
	SAVE("cal/gyro_z",      gyro_bias_z_mdps);
	SAVE("cal/hard_iron",   hard_iron);
	SAVE("cal/soft_iron",   soft_iron);

#undef SAVE
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(tracker, "tracker", NULL, h_set, NULL, h_export);

/* ── Public API ──────────────────────────────────────────────────────────── */

int config_init(void)
{
	/* Start from defaults so missing NVS keys are covered. */
	cfg = cfg_defaults;

	int ret = settings_subsys_init();

	if (ret) {
		LOG_ERR("settings_subsys_init failed: %d", ret);
		return ret;
	}
	
	ret = settings_load();
	if (ret) {
		LOG_WRN("settings_load failed: %d — using defaults", ret);
	}

	LOG_INF("Config loaded (rate=%dHz lora=%s twd=%.1f)",
		cfg.sample_rate_hz,
		cfg.lora.enabled ? "on" : "off",
		(double)cfg.true_wind_dir_deg);

	return 0;
}

const struct app_config *config_get(void)
{
	return &cfg;
}

int config_save(void)
{
	int ret = settings_save();

	if (ret) {
		LOG_ERR("settings_save failed: %d", ret);
	} else {
		LOG_INF("Config saved");
	}
	return ret;
}

void config_set_sample_rate(data_rate_t hz)     { cfg.sample_rate_hz = hz; }
void config_set_data_sources(data_src_flags_t f){ cfg.data_sources = f; }
void config_set_row_src(row_src_t src)           { cfg.row_src = src; }
void config_set_timer_minutes(uint8_t min)       { cfg.timer_minutes = min; }

void config_set_box_src(int slot, box_src_t src)
{
	if (slot >= 0 && slot < BOX_COUNT) {
		cfg.box_src[slot] = src;
	}
}
void config_set_lora_enabled(bool en)           { cfg.lora.enabled = en; }
void config_set_lora_boat_id(uint8_t id)        { cfg.lora.boat_id = id; }
void config_set_lora_n_boats(uint8_t n)         { cfg.lora.n_boats = n; }
void config_set_lora_tx_rate(uint8_t hz)        { cfg.lora.tx_rate_hz = hz; }
void config_set_lora_frequency(uint32_t freq)   { cfg.lora.frequency = freq; }

void config_set_true_wind_dir(float deg)
{
	while (deg >= 360.0f) { deg -= 360.0f; }
	while (deg <    0.0f) { deg += 360.0f; }
	cfg.true_wind_dir_deg = deg;
}

void config_set_ahrs_gyro_bias(float x, float y, float z)
{
	cfg.gyro_bias_x_mdps = x;
	cfg.gyro_bias_y_mdps = y;
	cfg.gyro_bias_z_mdps = z;
}

void config_set_ahrs_mag_cal(const float hard[3], const float soft[3][3])
{
	for (int i = 0; i < 3; i++) {
		cfg.hard_iron[i] = hard[i];
		for (int j = 0; j < 3; j++) {
			cfg.soft_iron[i][j] = soft[i][j];
		}
	}
}

