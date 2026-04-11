/*
 * display_engine — Display rendering and button input
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_engine.h"
#include "config.h"
#include "data_engine.h"
#include "data_processor.h"

#include <display_ui.h>
#include <ahrs.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(display_engine, LOG_LEVEL_INF);

/* ── Buttons ─────────────────────────────────────────────────────────────── */

static const struct gpio_dt_spec btn_scroll =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn_select =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

#define LONG_PRESS_MS   500
#define POLL_MS          10   /* button poll interval ms */
#define RENDER_PERIOD_MS 100  /* display update period = 5 Hz */

/* ── Row display definitions ─────────────────────────────────────────────── */

typedef int (*row_scale_fn_t)(const struct data_averages *avg,
			      const struct data_sample   *latest,
			      const struct nav_results   *nav);

struct row_display_def {
	display_ui_postfix_t postfix;
	const char          *label;
	row_scale_fn_t       scale;
	bool                 can_be_negative;
};

/* Attitude: use latest sample for immediate response. */
static int fn_heading(const struct data_averages *a, const struct data_sample *l,
		      const struct nav_results *n)
{
	ARG_UNUSED(a); ARG_UNUSED(n);
	int v = (int)l->heading_deg;
	return v < 0 ? 0 : (v > 359 ? 359 : v);
}

static int fn_roll(const struct data_averages *a, const struct data_sample *l,
		   const struct nav_results *n)
{
	ARG_UNUSED(a); ARG_UNUSED(n);
	return (int)(fabsf(l->roll_deg) + 0.5f);
}

static int fn_pitch(const struct data_averages *a, const struct data_sample *l,
		    const struct nav_results *n)
{
	ARG_UNUSED(a); ARG_UNUSED(n);
	return (int)(fabsf(l->pitch_deg) + 0.5f);
}

/* GPS: use latest sample for immediate response. */
static int fn_speed_kt(const struct data_averages *a, const struct data_sample *l,
		       const struct nav_results *n)
{
	ARG_UNUSED(a); ARG_UNUSED(n);
	uint32_t kt = l->gps_speed_mm_s * 1000U / 514444U;
	return (int)(kt > 99 ? 99 : kt);
}

static int fn_altitude(const struct data_averages *a, const struct data_sample *l,
		       const struct nav_results *n)
{
	ARG_UNUSED(a); ARG_UNUSED(n);
	int32_t m = l->gps_altitude_mm / 1000;
	return (int)(m < 0 ? 0 : (m > 999 ? 999 : m));
}

static int fn_drift(const struct data_averages *a, const struct data_sample *l,
		    const struct nav_results *n)
{
	ARG_UNUSED(a); ARG_UNUSED(l);
	return (int)(fabsf(n->drift_deg) + 0.5f);
}

static const struct row_display_def row_defs[ROW_SRC_COUNT] = {
	[ROW_SRC_HEADING_DEG] = { DISPLAY_UI_POSTFIX_DEG, "HDG", fn_heading,  false },
	[ROW_SRC_SPEED_KT]    = { DISPLAY_UI_POSTFIX_KT,  "SPD", fn_speed_kt, false },
	[ROW_SRC_ALTITUDE_M]  = { DISPLAY_UI_POSTFIX_M,   "ALT", fn_altitude, false },
	[ROW_SRC_ROLL_DEG]    = { DISPLAY_UI_POSTFIX_DEG, "ROL", fn_roll,     true  },
	[ROW_SRC_PITCH_DEG]   = { DISPLAY_UI_POSTFIX_DEG, "PCH", fn_pitch,    true  },
	[ROW_SRC_DRIFT_DEG]   = { DISPLAY_UI_POSTFIX_DEG, "DFT", fn_drift,    true  },
};

/* ── Menu state ──────────────────────────────────────────────────────────── */

static bool in_menu;
static int  menu_selected;
static const display_ui_menu_t *current_menu;

/* ── Calibration state ──────────────────────────────────────────────────── */

static bool in_cal;

/* Action / submenu callbacks — forward declarations. */
static const display_ui_menu_t menu_settings;
static const display_ui_menu_t menu_row_config;
static const display_ui_menu_t menu_row0_src;
static const display_ui_menu_t menu_row1_src;
static const display_ui_menu_t menu_row2_src;
static const display_ui_menu_t menu_lora;
static const display_ui_menu_t menu_boat_id;
static const display_ui_menu_t menu_n_boats;
static const display_ui_menu_t menu_tx_rate;

static const display_ui_menu_t *go_settings(void)   { return &menu_settings; }
static const display_ui_menu_t *go_row_config(void) { return &menu_row_config; }
static const display_ui_menu_t *go_row0(void)       { return &menu_row0_src; }
static const display_ui_menu_t *go_row1(void)       { return &menu_row1_src; }
static const display_ui_menu_t *go_row2(void)       { return &menu_row2_src; }
static const display_ui_menu_t *go_lora(void)       { return &menu_lora; }
static const display_ui_menu_t *go_boat_id(void)    { return &menu_boat_id; }
static const display_ui_menu_t *go_n_boats(void)    { return &menu_n_boats; }
static const display_ui_menu_t *go_tx_rate(void)    { return &menu_tx_rate; }

/* Actions */
static void act_set_wind(void)
{
	struct data_averages avg;

	data_engine_get_averages(&avg);
	config_set_true_wind_dir(avg.avg_heading_deg);
	config_save();
	LOG_INF("TWD set to %.1f deg", (double)avg.avg_heading_deg);
}

static void act_save(void)    { config_save(); }
static void act_exit(void)    { in_menu = false; }

static void act_mag_cal(void)
{
	ahrs_cal_start();
	data_engine_set_calibrating(true);
	in_cal  = true;
	in_menu = false;
	LOG_INF("Mag calibration started — rotate device");
}

static void act_lora_on(void)  { config_set_lora_enabled(true); }
static void act_lora_off(void) { config_set_lora_enabled(false); }

/* Row source setters */
#define ROW_SRC_ACTION(name, row, src) \
static void name(void) { config_set_row_src(row, src); }

ROW_SRC_ACTION(act_r0_hdg, 0, ROW_SRC_HEADING_DEG)
ROW_SRC_ACTION(act_r0_spd, 0, ROW_SRC_SPEED_KT)
ROW_SRC_ACTION(act_r0_alt, 0, ROW_SRC_ALTITUDE_M)
ROW_SRC_ACTION(act_r0_rol, 0, ROW_SRC_ROLL_DEG)
ROW_SRC_ACTION(act_r0_pch, 0, ROW_SRC_PITCH_DEG)
ROW_SRC_ACTION(act_r0_dft, 0, ROW_SRC_DRIFT_DEG)

ROW_SRC_ACTION(act_r1_hdg, 1, ROW_SRC_HEADING_DEG)
ROW_SRC_ACTION(act_r1_spd, 1, ROW_SRC_SPEED_KT)
ROW_SRC_ACTION(act_r1_alt, 1, ROW_SRC_ALTITUDE_M)
ROW_SRC_ACTION(act_r1_rol, 1, ROW_SRC_ROLL_DEG)
ROW_SRC_ACTION(act_r1_pch, 1, ROW_SRC_PITCH_DEG)
ROW_SRC_ACTION(act_r1_dft, 1, ROW_SRC_DRIFT_DEG)

ROW_SRC_ACTION(act_r2_hdg, 2, ROW_SRC_HEADING_DEG)
ROW_SRC_ACTION(act_r2_spd, 2, ROW_SRC_SPEED_KT)
ROW_SRC_ACTION(act_r2_alt, 2, ROW_SRC_ALTITUDE_M)
ROW_SRC_ACTION(act_r2_rol, 2, ROW_SRC_ROLL_DEG)
ROW_SRC_ACTION(act_r2_pch, 2, ROW_SRC_PITCH_DEG)
ROW_SRC_ACTION(act_r2_dft, 2, ROW_SRC_DRIFT_DEG)

/* LoRa sub-settings */
static void act_id0(void) { config_set_lora_boat_id(0); }
static void act_id1(void) { config_set_lora_boat_id(1); }
static void act_id2(void) { config_set_lora_boat_id(2); }
static void act_id3(void) { config_set_lora_boat_id(3); }

static void act_nb1(void) { config_set_lora_n_boats(1); }
static void act_nb2(void) { config_set_lora_n_boats(2); }
static void act_nb3(void) { config_set_lora_n_boats(3); }
static void act_nb4(void) { config_set_lora_n_boats(4); }

static void act_tx1(void) { config_set_lora_tx_rate(1); }
static void act_tx2(void) { config_set_lora_tx_rate(2); }
static void act_tx5(void) { config_set_lora_tx_rate(5); }

/* ── Menu definitions ────────────────────────────────────────────────────── */

static const display_ui_menu_item_t items_settings[] = {
	DISPLAY_UI_ACTION_ITEM ("SET WIND",  act_set_wind),
	DISPLAY_UI_ACTION_ITEM ("MAG CAL",   act_mag_cal),
	DISPLAY_UI_SUBMENU_ITEM("ROW CFG",   go_row_config),
	DISPLAY_UI_SUBMENU_ITEM("LORA",      go_lora),
	DISPLAY_UI_ACTION_ITEM ("SAVE",      act_save),
	DISPLAY_UI_ACTION_ITEM ("EXIT",      act_exit),
};
static const display_ui_menu_t menu_settings = {
	"SETTINGS", items_settings, ARRAY_SIZE(items_settings)
};

static const display_ui_menu_item_t items_row_config[] = {
	DISPLAY_UI_SUBMENU_ITEM("ROW 0", go_row0),
	DISPLAY_UI_SUBMENU_ITEM("ROW 1", go_row1),
	DISPLAY_UI_SUBMENU_ITEM("ROW 2", go_row2),
	DISPLAY_UI_SUBMENU_ITEM("BACK",  go_settings),
};
static const display_ui_menu_t menu_row_config = {
	"ROW CFG", items_row_config, ARRAY_SIZE(items_row_config)
};

#define ROW_SRC_ITEMS(r0, r1, r2, r3, r4, r5) \
	DISPLAY_UI_ACTION_ITEM ("HEADING", r0), \
	DISPLAY_UI_ACTION_ITEM ("SPEED",   r1), \
	DISPLAY_UI_ACTION_ITEM ("ALTITUDE",r2), \
	DISPLAY_UI_ACTION_ITEM ("ROLL",    r3), \
	DISPLAY_UI_ACTION_ITEM ("PITCH",   r4), \
	DISPLAY_UI_ACTION_ITEM ("DRIFT",   r5)

static const display_ui_menu_item_t items_row0[] = {
	ROW_SRC_ITEMS(act_r0_hdg, act_r0_spd, act_r0_alt,
		      act_r0_rol, act_r0_pch, act_r0_dft),
	DISPLAY_UI_SUBMENU_ITEM("BACK", go_row_config),
};
static const display_ui_menu_t menu_row0_src = {
	"ROW 0", items_row0, ARRAY_SIZE(items_row0)
};

static const display_ui_menu_item_t items_row1[] = {
	ROW_SRC_ITEMS(act_r1_hdg, act_r1_spd, act_r1_alt,
		      act_r1_rol, act_r1_pch, act_r1_dft),
	DISPLAY_UI_SUBMENU_ITEM("BACK", go_row_config),
};
static const display_ui_menu_t menu_row1_src = {
	"ROW 1", items_row1, ARRAY_SIZE(items_row1)
};

static const display_ui_menu_item_t items_row2[] = {
	ROW_SRC_ITEMS(act_r2_hdg, act_r2_spd, act_r2_alt,
		      act_r2_rol, act_r2_pch, act_r2_dft),
	DISPLAY_UI_SUBMENU_ITEM("BACK", go_row_config),
};
static const display_ui_menu_t menu_row2_src = {
	"ROW 2", items_row2, ARRAY_SIZE(items_row2)
};

static const display_ui_menu_item_t items_lora[] = {
	DISPLAY_UI_ACTION_ITEM ("LORA ON",  act_lora_on),
	DISPLAY_UI_ACTION_ITEM ("LORA OFF", act_lora_off),
	DISPLAY_UI_SUBMENU_ITEM("BOAT ID",  go_boat_id),
	DISPLAY_UI_SUBMENU_ITEM("N BOATS",  go_n_boats),
	DISPLAY_UI_SUBMENU_ITEM("TX RATE",  go_tx_rate),
	DISPLAY_UI_SUBMENU_ITEM("BACK",     go_settings),
};
static const display_ui_menu_t menu_lora = {
	"LORA", items_lora, ARRAY_SIZE(items_lora)
};

static const display_ui_menu_item_t items_boat_id[] = {
	DISPLAY_UI_ACTION_ITEM ("ID 0", act_id0),
	DISPLAY_UI_ACTION_ITEM ("ID 1", act_id1),
	DISPLAY_UI_ACTION_ITEM ("ID 2", act_id2),
	DISPLAY_UI_ACTION_ITEM ("ID 3", act_id3),
	DISPLAY_UI_SUBMENU_ITEM("BACK", go_lora),
};
static const display_ui_menu_t menu_boat_id = {
	"BOAT ID", items_boat_id, ARRAY_SIZE(items_boat_id)
};

static const display_ui_menu_item_t items_n_boats[] = {
	DISPLAY_UI_ACTION_ITEM ("1 BOAT",  act_nb1),
	DISPLAY_UI_ACTION_ITEM ("2 BOATS", act_nb2),
	DISPLAY_UI_ACTION_ITEM ("3 BOATS", act_nb3),
	DISPLAY_UI_ACTION_ITEM ("4 BOATS", act_nb4),
	DISPLAY_UI_SUBMENU_ITEM("BACK",    go_lora),
};
static const display_ui_menu_t menu_n_boats = {
	"N BOATS", items_n_boats, ARRAY_SIZE(items_n_boats)
};

static const display_ui_menu_item_t items_tx_rate[] = {
	DISPLAY_UI_ACTION_ITEM ("1 HZ", act_tx1),
	DISPLAY_UI_ACTION_ITEM ("2 HZ", act_tx2),
	DISPLAY_UI_ACTION_ITEM ("5 HZ", act_tx5),
	DISPLAY_UI_SUBMENU_ITEM("BACK", go_lora),
};
static const display_ui_menu_t menu_tx_rate = {
	"TX RATE", items_tx_rate, ARRAY_SIZE(items_tx_rate)
};

/* ── Data-screen rendering ───────────────────────────────────────────────── */

static void render_data_screen(void)
{
	const struct app_config *cfg = config_get();
	struct data_averages avg;
	struct nav_results   nav;

	data_engine_get_averages(&avg);
	data_processor_get_results(&nav);

	struct data_sample latest;

	data_engine_get_latest(&latest);

	display_ui_clear();

	for (int row = 0; row < 3; row++) {
		row_src_t src = cfg->row_src[row];

		if (src >= ROW_SRC_COUNT) {
			continue;
		}

		const struct row_display_def *def = &row_defs[src];
		int val = def->scale(&avg, &latest, &nav);

		display_ui_draw_row(row, val, def->postfix, def->label);

		/* Middle row is always inverted (white-on-black) for visual separation.
		 * For negative-capable sources the inversion is XOR'd on top. */
		bool invert = (row == 1);

		// if (def->can_be_negative) {
		// 	if (src == ROW_SRC_ROLL_DEG)  { invert ^= (latest.roll_deg  < 0.0f); }
		// 	if (src == ROW_SRC_PITCH_DEG) { invert ^= (latest.pitch_deg < 0.0f); }
		// 	if (src == ROW_SRC_DRIFT_DEG) { invert ^= (nav.drift_deg    < 0.0f); }
		// }

		if (invert) {
			display_ui_invert_row(row);
		}
	}

	display_ui_flush();
}

/* ── Calibration screen rendering ────────────────────────────────────────── */

static void render_cal_screen(void)
{
	struct ahrs_cal_quality q;

	ahrs_cal_get_quality(&q);

	int gap = (int)(q.gap_pct + 0.5f);
	int fit = (int)(q.fit_error * 10.0f + 0.5f);  /* µT × 10 */
	int n   = (int)q.sample_count;

	if (gap > 100) { gap = 100; }
	if (fit > 999) { fit = 999; }
	if (n   > 999) { n   = 999; }

	display_ui_clear();
	display_ui_draw_row(0, gap, DISPLAY_UI_POSTFIX_NONE, "GAP");
	display_ui_draw_row(1, fit, DISPLAY_UI_POSTFIX_NONE, "FIT");
	display_ui_invert_row(1);
	display_ui_draw_row(2, n,   DISPLAY_UI_POSTFIX_NONE, "N");
	display_ui_flush();
}

static void cal_finish(bool keep)
{
	data_engine_set_calibrating(false);
	in_cal = false;

	if (keep) {
		struct ahrs_cal_quality q;

		ahrs_cal_get_quality(&q);
		ahrs_cal_commit();
		config_set_ahrs_mag_cal(q.hard_iron,
					(const float (*)[3])q.soft_iron);
		config_save();
		LOG_INF("Mag cal saved: HI=[%.1f,%.1f,%.1f] "
			"gap=%.0f%% fit=%.2f wob=%.2f",
			(double)q.hard_iron[0],
			(double)q.hard_iron[1],
			(double)q.hard_iron[2],
			(double)q.gap_pct,
			(double)q.fit_error,
			(double)q.wobble);
	} else {
		LOG_INF("Mag cal cancelled");
	}
}

/* ── Display thread ──────────────────────────────────────────────────────── */

#define DISP_STACK_SIZE  2048
#define DISP_PRIORITY    7

static K_THREAD_STACK_DEFINE(disp_stack, DISP_STACK_SIZE);
static struct k_thread disp_thread_data;

static void display_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	bool    prev_scroll        = false;
	bool    prev_select        = false;
	int64_t scroll_press_start = 0;
	bool    scroll_long_fired  = false;
	int64_t last_render_ms     = k_uptime_get();

	in_menu        = false;
	current_menu   = &menu_settings;
	menu_selected  = 0;

	LOG_INF("Display engine started");

	while (1) {
		bool scroll = gpio_pin_get_dt(&btn_scroll) == 0; /* active-low */
		bool select = gpio_pin_get_dt(&btn_select) == 0;

		/* ── Calibration mode buttons ────────────────────────────── */
		if (in_cal) {
			/* Long-press sw0: cancel calibration */
			if (scroll && !prev_scroll) {
				scroll_press_start = k_uptime_get();
				scroll_long_fired  = false;
			}
			if (scroll && !scroll_long_fired) {
				if ((k_uptime_get() - scroll_press_start) >=
				    LONG_PRESS_MS) {
					scroll_long_fired = true;
					cal_finish(false);
				}
			}

			/* Short-press sw0: restart calibration */
			if (!scroll && prev_scroll && !scroll_long_fired) {
				ahrs_cal_start();
				LOG_INF("Mag cal restarted");
			}

			/* sw1: accept calibration, save */
			if (select && !prev_select) {
				cal_finish(true);
			}

			goto buttons_done;
		}

		/* ── Long-press detection for sw0 ────────────────────────── */
		if (scroll && !prev_scroll) {
			scroll_press_start = k_uptime_get();
			scroll_long_fired  = false;
		}
		if (scroll && !scroll_long_fired) {
			if ((k_uptime_get() - scroll_press_start) >= LONG_PRESS_MS) {
				scroll_long_fired = true;
				if (!in_menu) {
					in_menu       = true;
					current_menu  = &menu_settings;
					menu_selected = 0;
				}
			}
		}

		/* ── Short-press scroll in menu ──────────────────────────── */
		if (!scroll && prev_scroll && !scroll_long_fired && in_menu) {
			menu_selected = (menu_selected + 1) % current_menu->count;
		}

		/* ── Select button ───────────────────────────────────────── */
		if (select && !prev_select) {
			if (!in_menu) {
				/* Capture heading as True Wind Direction. */
				act_set_wind();
			} else {
				const display_ui_menu_t *next =
					display_ui_menu_activate(current_menu, menu_selected);

				if (next != current_menu) {
					current_menu  = next;
					menu_selected = 0;
				}
				/* act_exit() sets in_menu = false. */
			}
		}

buttons_done:

		prev_scroll = scroll;
		prev_select = select;

		/* ── Render at 5 Hz ─────────────────────────────────────── */
		int64_t now = k_uptime_get();

		if ((now - last_render_ms) >= RENDER_PERIOD_MS) {
			last_render_ms = now;
			if (in_cal) {
				render_cal_screen();
			} else if (in_menu) {
				display_ui_draw_menu(current_menu, menu_selected);
				display_ui_flush();
			} else {
				render_data_screen();
			}
		}

		k_msleep(POLL_MS);
	}
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int display_engine_init(void)
{
	const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	int ret;

	ret = display_ui_init(disp);
	if (ret < 0) {
		LOG_ERR("display_ui_init failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&btn_scroll, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("btn_scroll configure failed: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&btn_select, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("btn_select configure failed: %d", ret);
		return ret;
	}

	k_thread_create(&disp_thread_data, disp_stack,
			K_THREAD_STACK_SIZEOF(disp_stack),
			display_thread, NULL, NULL, NULL,
			DISP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&disp_thread_data, "display");

	return 0;
}
