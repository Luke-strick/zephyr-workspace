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
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(display_engine, LOG_LEVEL_INF);

/* ── Buttons ─────────────────────────────────────────────────────────────── */

static const struct gpio_dt_spec btn_scroll =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn_select =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

#define LONG_PRESS_MS   800
#define POLL_MS          10   /* button poll interval ms */
#define RENDER_PERIOD_MS 100  /* display update period = 5 Hz */

/* mm/s → tenths of knots (1 kt = 514.444 mm/s) */
static int speed_to_tenths_kt(uint32_t mm_s)
{
	uint32_t t = mm_s * 10000U / 514444U;
	return (int)(t > 999 ? 999 : t);
}

/* ── Race timer state ────────────────────────────────────────────────────── */

typedef enum { TMR_IDLE, TMR_RUN } tmr_state_t;

static tmr_state_t tmr_state;
static int64_t     tmr_target_ms;   /* uptime ms at which countdown hits 0 */

static void timer_reset(void)
{
	tmr_state = TMR_IDLE;
}

static void timer_press(void)
{
	int64_t now  = k_uptime_get();
	int64_t full = (int64_t)config_get()->timer_minutes * 60000;

	if (tmr_state == TMR_IDLE) {
		tmr_target_ms = now + full;
		tmr_state     = TMR_RUN;
		return;
	}

	int64_t rem = tmr_target_ms - now;

	if (rem > 0) {
		/* Sync: round remaining down to the nearest whole minute. */
		tmr_target_ms = now + (rem / 60000) * 60000;
	} else {
		/* Counting up — restart a fresh countdown from full duration. */
		tmr_target_ms = now + full;
	}
}

/* Format the timer as "M:SS" (countdown) or "-M:SS" (counting up). */
static void timer_format(char *buf, size_t n)
{
	if (tmr_state == TMR_IDLE) {
		snprintf(buf, n, "%u:00", config_get()->timer_minutes);
		return;
	}

	int64_t rem = tmr_target_ms - k_uptime_get();

	if (rem > 0) {
		int s = (int)((rem + 999) / 1000);   /* ceil to whole seconds */
		snprintf(buf, n, "%d:%02d", s / 60, s % 60);
	} else {
		int s = (int)((-rem) / 1000);
		snprintf(buf, n, "-%d:%02d", s / 60, s % 60);
	}
}

/* ── Display state ───────────────────────────────────────────────────────── */

static bool in_menu;
static int  menu_selected;
static const display_ui_menu_t *current_menu;

static bool in_cal;
static int  cal_view;    /* 0=GAP, 1=FIT, 2=N — cycled by scroll in cal mode */
static bool in_imu_cal;
static int  edit_slot;   /* box slot currently being edited via the menu */

/* ── Menu callbacks ──────────────────────────────────────────────────────── */

static const display_ui_menu_t menu_settings;
static const display_ui_menu_t menu_boxes;
static const display_ui_menu_t menu_box_src;
static const display_ui_menu_t menu_timer;
static const display_ui_menu_t menu_lora;
static const display_ui_menu_t menu_boat_id;
static const display_ui_menu_t menu_n_boats;
static const display_ui_menu_t menu_tx_rate;

static const display_ui_menu_t *go_settings(void)    { return &menu_settings; }
static const display_ui_menu_t *go_boxes(void)       { return &menu_boxes; }
static const display_ui_menu_t *go_timer(void)       { return &menu_timer; }
static const display_ui_menu_t *go_lora(void)        { return &menu_lora; }
static const display_ui_menu_t *go_boat_id(void)     { return &menu_boat_id; }
static const display_ui_menu_t *go_n_boats(void)     { return &menu_n_boats; }
static const display_ui_menu_t *go_tx_rate(void)     { return &menu_tx_rate; }

/* Select which box slot the source-picker will edit, then open the picker. */
static const display_ui_menu_t *go_box0(void) { edit_slot = 0; return &menu_box_src; }
static const display_ui_menu_t *go_box1(void) { edit_slot = 1; return &menu_box_src; }
static const display_ui_menu_t *go_box2(void) { edit_slot = 2; return &menu_box_src; }
static const display_ui_menu_t *go_box3(void) { edit_slot = 3; return &menu_box_src; }

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
	cal_view = 0;
	in_cal   = true;
	in_menu  = false;
	LOG_INF("Mag calibration started — rotate device");
}

#define IMU_CAL_N_TARGET  50   /* 5 s at 10 Hz */

static void act_imu_cal(void)
{
	ahrs_imu_cal_start();
	data_engine_set_imu_calibrating(true);
	in_imu_cal = true;
	in_menu    = false;
	LOG_INF("IMU calibration started — keep device flat and still");
}

static void imu_cal_finish(bool keep)
{
	data_engine_set_imu_calibrating(false);
	in_imu_cal = false;

	if (keep && ahrs_imu_cal_get_count() >= 10) {
		float gx, gy, gz;

		ahrs_imu_cal_commit(&gx, &gy, &gz);
		config_set_ahrs_gyro_bias(gx, gy, gz);
		config_save();
		LOG_INF("IMU cal saved: gyro=[%.1f,%.1f,%.1f]mdps",
			(double)gx, (double)gy, (double)gz);
	} else {
		LOG_INF("IMU cal cancelled");
	}
}

/* Box source selection — persists the chosen source for edit_slot, exits menu */
static void select_box(box_src_t src)
{
	config_set_box_src(edit_slot, src);
	config_save();
	in_menu = false;
}

static void act_box_speed(void) { select_box(BOX_SRC_SPEED);    }
static void act_box_cog(void)   { select_box(BOX_SRC_COG);      }
static void act_box_sog(void)   { select_box(BOX_SRC_SOG);      }
static void act_box_roll(void)  { select_box(BOX_SRC_ROLL);     }
static void act_box_pitch(void) { select_box(BOX_SRC_PITCH);    }
static void act_box_time(void)  { select_box(BOX_SRC_GPS_TIME); }
static void act_box_hdg(void)   { select_box(BOX_SRC_HEADING);  }
static void act_box_timer(void) { select_box(BOX_SRC_TIMER);    }

/* Timer duration selection — persists and exits menu */
static void select_timer_min(uint8_t m)
{
	config_set_timer_minutes(m);
	config_save();
	in_menu = false;
}

static void act_tmr1(void) { select_timer_min(1); }
static void act_tmr3(void) { select_timer_min(3); }
static void act_tmr5(void) { select_timer_min(5); }

static void act_lora_on(void)  { config_set_lora_enabled(true); }
static void act_lora_off(void) { config_set_lora_enabled(false); }

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
	DISPLAY_UI_ACTION_ITEM ("IMU CAL",   act_imu_cal),
	DISPLAY_UI_SUBMENU_ITEM("BOXES",     go_boxes),
	DISPLAY_UI_SUBMENU_ITEM("TIMER",     go_timer),
	DISPLAY_UI_SUBMENU_ITEM("LORA",      go_lora),
	DISPLAY_UI_ACTION_ITEM ("SAVE",      act_save),
	DISPLAY_UI_ACTION_ITEM ("EXIT",      act_exit),
};
static const display_ui_menu_t menu_settings = {
	"SETTINGS", items_settings, ARRAY_SIZE(items_settings)
};

static const display_ui_menu_item_t items_boxes[] = {
	DISPLAY_UI_SUBMENU_ITEM("BOX 1", go_box0),
	DISPLAY_UI_SUBMENU_ITEM("BOX 2", go_box1),
	DISPLAY_UI_SUBMENU_ITEM("BOX 3", go_box2),
	DISPLAY_UI_SUBMENU_ITEM("BOX 4", go_box3),
	DISPLAY_UI_SUBMENU_ITEM("BACK",  go_settings),
};
static const display_ui_menu_t menu_boxes = {
	"BOXES", items_boxes, ARRAY_SIZE(items_boxes)
};

static const display_ui_menu_item_t items_box_src[] = {
	DISPLAY_UI_ACTION_ITEM ("SPEED",    act_box_speed),
	DISPLAY_UI_ACTION_ITEM ("COG",      act_box_cog),
	DISPLAY_UI_ACTION_ITEM ("SOG",      act_box_sog),
	DISPLAY_UI_ACTION_ITEM ("ROLL",     act_box_roll),
	DISPLAY_UI_ACTION_ITEM ("PITCH",    act_box_pitch),
	DISPLAY_UI_ACTION_ITEM ("GPS TIME", act_box_time),
	DISPLAY_UI_ACTION_ITEM ("HEADING",  act_box_hdg),
	DISPLAY_UI_ACTION_ITEM ("TIMER",    act_box_timer),
	DISPLAY_UI_SUBMENU_ITEM("BACK",     go_boxes),
};
static const display_ui_menu_t menu_box_src = {
	"SOURCE", items_box_src, ARRAY_SIZE(items_box_src)
};

static const display_ui_menu_item_t items_timer[] = {
	DISPLAY_UI_ACTION_ITEM ("1 MIN", act_tmr1),
	DISPLAY_UI_ACTION_ITEM ("3 MIN", act_tmr3),
	DISPLAY_UI_ACTION_ITEM ("5 MIN", act_tmr5),
	DISPLAY_UI_SUBMENU_ITEM("BACK",  go_settings),
};
static const display_ui_menu_t menu_timer = {
	"TIMER", items_timer, ARRAY_SIZE(items_timer)
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

/* Fill title/value/unit strings for one box source. */
static void format_box(box_src_t src, const struct data_averages *avg,
		       const struct data_sample *latest,
		       char *title, char *value, char *unit, size_t n)
{
	unit[0] = '\0';

	switch (src) {
	case BOX_SRC_SPEED:
	case BOX_SRC_SOG: {
		int t = speed_to_tenths_kt(latest->gps_speed_mm_s);

		strncpy(title, src == BOX_SRC_SPEED ? "SPD" : "SOG", n);
		snprintf(value, n, "%d.%d", t / 10, t % 10);
		strncpy(unit, "KT", n);
		break;
	}
	case BOX_SRC_COG:
		strncpy(title, "COG", n);
		snprintf(value, n, "%u", latest->gps_bearing_mdeg / 1000U);
		strncpy(unit, "DEG", n);
		break;
	case BOX_SRC_ROLL:
		strncpy(title, "ROLL", n);
		snprintf(value, n, "%d", (int)(latest->roll_deg + 0.5f));
		strncpy(unit, "DEG", n);
		break;
	case BOX_SRC_PITCH:
		strncpy(title, "PCH", n);
		snprintf(value, n, "%d", (int)(latest->pitch_deg + 0.5f));
		strncpy(unit, "DEG", n);
		break;
	case BOX_SRC_HEADING: {
		int h = (int)(latest->heading_deg + 0.5f);

		if (h < 0)   { h += 360; }
		if (h > 359) { h = 359; }
		strncpy(title, "HDG", n);
		snprintf(value, n, "%d", h);
		strncpy(unit, "DEG", n);
		break;
	}
	case BOX_SRC_GPS_TIME:
		strncpy(title, "UTC", n);
		snprintf(value, n, "%02u:%02u", avg->utc.hour, avg->utc.minute);
		break;
	case BOX_SRC_TIMER:
		strncpy(title, "TMR", n);
		timer_format(value, n);
		break;
	default:
		title[0] = value[0] = '\0';
		break;
	}
}

static void render_data_screen(void)
{
	struct data_averages avg;
	struct data_sample   latest;

	data_engine_get_averages(&avg);
	data_engine_get_latest(&latest);

	const struct app_config *cfg = config_get();

	display_ui_clear();

	for (int slot = 0; slot < BOX_COUNT; slot++) {
		char title[12], value[12], unit[12];

		format_box(cfg->box_src[slot], &avg, &latest,
			   title, value, unit, sizeof(value));
		display_ui_draw_box(slot, title, value, unit);
	}

	display_ui_draw_roll_bar((int)(latest.roll_deg + (latest.roll_deg < 0 ? -0.5f : 0.5f)), 45);
	display_ui_flush();
}

/* ── Calibration screen rendering ────────────────────────────────────────── */

static const char * const cal_labels[] = { "GAP", "FIT", "N" };

static void render_cal_screen(void)
{
	struct ahrs_cal_quality q;

	ahrs_cal_get_quality(&q);

	int vals[3];

	vals[0] = (int)(q.gap_pct + 0.5f);
	vals[1] = (int)(q.fit_error * 10.0f + 0.5f);  /* µT × 10 */
	vals[2] = (int)q.sample_count;

	if (vals[0] > 100) { vals[0] = 100; }
	if (vals[1] > 999) { vals[1] = 999; }
	if (vals[2] > 999) { vals[2] = 999; }

	display_ui_clear();
	display_ui_draw_row(0, vals[cal_view], DISPLAY_UI_POSTFIX_NONE,
			    cal_labels[cal_view]);
	display_ui_invert_row(0);  /* black background makes cal screen distinct */
	display_ui_flush();
}

static void render_imu_cal_screen(void)
{
	uint32_t n = ahrs_imu_cal_get_count();

	if (n > 999) {
		n = 999;
	}

	display_ui_clear();
	display_ui_draw_row(0, (int)n, DISPLAY_UI_POSTFIX_NONE, "IMU");

	if (n >= IMU_CAL_N_TARGET) {
		display_ui_invert_row(0);   /* inverted = ready to accept */
	}

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
	int64_t select_press_start = 0;
	bool    select_long_fired  = false;
	int64_t last_render_ms     = k_uptime_get();

	in_menu       = false;
	current_menu  = &menu_settings;
	menu_selected = 0;
	cal_view      = 0;
	in_imu_cal    = false;
	timer_reset();

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

			/* Short-press sw0: cycle cal view (GAP → FIT → N) */
			if (!scroll && prev_scroll && !scroll_long_fired) {
				cal_view = (cal_view + 1) % 3;
			}

			/* sw1: accept calibration, save */
			if (select && !prev_select) {
				cal_finish(true);
			}

			goto buttons_done;
		}

		/* ── IMU calibration mode buttons ───────────────────────── */
		if (in_imu_cal) {
			/* Long-press sw0: cancel */
			if (scroll && !prev_scroll) {
				scroll_press_start = k_uptime_get();
				scroll_long_fired  = false;
			}
			if (scroll && !scroll_long_fired) {
				if ((k_uptime_get() - scroll_press_start) >= LONG_PRESS_MS) {
					scroll_long_fired = true;
					imu_cal_finish(false);
				}
			}

			/* sw1: accept (requires minimum samples) */
			if (select && !prev_select) {
				imu_cal_finish(true);
			}

			/* Auto-commit when target reached */
			if (ahrs_imu_cal_get_count() >= IMU_CAL_N_TARGET) {
				imu_cal_finish(true);
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

		/* ── Short-press scroll ──────────────────────────────────── */
		if (!scroll && prev_scroll && !scroll_long_fired) {
			if (in_menu) {
				menu_selected = (menu_selected + 1) % current_menu->count;
			}
			/* Data screen: sw0 short-press is a no-op. */
		}

		/* ── Select button (sw1) ─────────────────────────────────── */
		if (in_menu) {
			if (select && !prev_select) {
				const display_ui_menu_t *next =
					display_ui_menu_activate(current_menu, menu_selected);

				if (next != current_menu) {
					current_menu  = next;
					menu_selected = 0;
				}
				/* act_exit() sets in_menu = false. */
			}
		} else {
			/* Data screen: sw1 drives the race timer.
			 * Short-press = start/sync; long-press (hold) = reset. */
			if (select && !prev_select) {
				select_press_start = k_uptime_get();
				select_long_fired  = false;
			}
			if (select && !select_long_fired &&
			    (k_uptime_get() - select_press_start) >= LONG_PRESS_MS) {
				select_long_fired = true;
				timer_reset();
			}
			if (!select && prev_select && !select_long_fired) {
				timer_press();
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
			} else if (in_imu_cal) {
				render_imu_cal_screen();
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
