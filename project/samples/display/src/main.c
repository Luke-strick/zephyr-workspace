/*
 * Display sample — menu / options screen demo
 *
 * Button 1 (sw0): scroll to next item
 * Button 2 (sw1): activate selected item (run action or enter submenu)
 *
 * Menu structure:
 *   SETTINGS
 *     DISPLAY  →  DISPLAY
 *                   INVERT SPD  (action)
 *                   BACK        →  SETTINGS
 *     SENSORS  →  SENSORS
 *                   CALIBRATE   (action stub)
 *                   BACK        →  SETTINGS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device_runtime.h>
#include <display_ui.h>

/* ── Buttons ─────────────────────────────────────────────────────────────── */

static const struct gpio_dt_spec btn_scroll =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn_select =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

/* ── Action callbacks ────────────────────────────────────────────────────── */

static bool speed_inverted;

static void action_invert_speed(void)
{
	speed_inverted = !speed_inverted;
	printk("Speed row inverted: %s\n", speed_inverted ? "on" : "off");
}

static void action_calibrate(void)
{
	printk("Calibrate stub — not implemented\n");
}

/* ── Menu descriptors ────────────────────────────────────────────────────── */

/* Forward declarations so submenu callbacks can reference later menus. */
static const display_ui_menu_t menu_main;
static const display_ui_menu_t menu_display;
static const display_ui_menu_t menu_sensors;

static const display_ui_menu_t *go_main(void)    { return &menu_main; }
static const display_ui_menu_t *go_display(void) { return &menu_display; }
static const display_ui_menu_t *go_sensors(void) { return &menu_sensors; }

static const display_ui_menu_item_t items_main[] = {
	DISPLAY_UI_SUBMENU_ITEM("DISPLAY", go_display),
	DISPLAY_UI_SUBMENU_ITEM("SENSORS", go_sensors),
};

static const display_ui_menu_t menu_main = {
	.title = "SETTINGS",
	.items = items_main,
	.count = ARRAY_SIZE(items_main),
};

static const display_ui_menu_item_t items_display[] = {
	DISPLAY_UI_ACTION_ITEM("INVERT SPD", action_invert_speed),
	DISPLAY_UI_SUBMENU_ITEM("BACK",      go_main),
};

static const display_ui_menu_t menu_display = {
	.title = "DISPLAY",
	.items = items_display,
	.count = ARRAY_SIZE(items_display),
};

static const display_ui_menu_item_t items_sensors[] = {
	DISPLAY_UI_ACTION_ITEM("CALIBRATE", action_calibrate),
	DISPLAY_UI_SUBMENU_ITEM("BACK",     go_main),
};

static const display_ui_menu_t menu_sensors = {
	.title = "SENSORS",
	.items = items_sensors,
	.count = ARRAY_SIZE(items_sensors),
};

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_5v)));

	const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (display_ui_init(disp) < 0) {
		printk("Display not ready\n");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&btn_scroll, GPIO_INPUT);
	gpio_pin_configure_dt(&btn_select, GPIO_INPUT);

	const display_ui_menu_t *menu = &menu_main;
	int selected = 0;

	display_ui_draw_menu(menu, selected);
	display_ui_flush();

	bool prev_scroll = false;
	bool prev_select = false;

	while (1) {
		/* Active-low buttons: pressed = 0 */
		bool scroll = gpio_pin_get_dt(&btn_scroll) == 0;
		bool select = gpio_pin_get_dt(&btn_select) == 0;

		if (scroll && !prev_scroll) {
			selected = (selected + 1) % menu->count;
			display_ui_draw_menu(menu, selected);
			display_ui_flush();
		}

		if (select && !prev_select) {
			menu = display_ui_menu_activate(menu, selected);
			selected = 0;
			display_ui_draw_menu(menu, selected);
			display_ui_flush();
		}

		prev_scroll = scroll;
		prev_select = select;

		k_msleep(50);
	}

	return 0;
}
