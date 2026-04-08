/*
 * display_ui — 1bpp framebuffer UI for Sharp-style memory LCDs
 *
 * Two screens:
 *   Data screen  — three rows of large 7-segment digits (display_ui_draw_row)
 *   Menu screen  — scrollable list of labelled items with selection border
 *
 * Configure via Kconfig (CONFIG_DISPLAY_UI_*).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <zephyr/device.h>

/* ── Data screen ─────────────────────────────────────────────────────────── */

/**
 * Postfix symbol drawn to the right of each digit row.
 */
typedef enum {
	DISPLAY_UI_POSTFIX_NONE,
	DISPLAY_UI_POSTFIX_KT,   /* "kt" — knots   */
	DISPLAY_UI_POSTFIX_DEG,  /* "°"  — degrees */
	DISPLAY_UI_POSTFIX_M,    /* "m"  — metres  */
} display_ui_postfix_t;

/**
 * Initialise the library and take ownership of @p dev.
 * Turns blanking off. Call once before any draw/flush calls.
 *
 * @return 0 on success, negative errno on failure.
 */
int display_ui_init(const struct device *dev);

/** Fill the framebuffer with white (clears all pixels). */
void display_ui_clear(void);

/**
 * Draw a 3-digit number in row @p row.
 *
 * @param row    Row index (0 = top).
 * @param n      Value 0–999.
 * @param pf     Postfix symbol to draw beside the number.
 * @param label  Short string drawn vertically on the right edge of the row,
 *               or NULL / "" for none.
 */
void display_ui_draw_row(int row, int n, display_ui_postfix_t pf,
			 const char *label);

/**
 * XOR-invert all pixels in the background of @p row.
 * Call after draw_row to get white-on-black rendering.
 */
void display_ui_invert_row(int row);

/** Push the framebuffer to the display. */
void display_ui_flush(void);

/* ── Menu / options screen ───────────────────────────────────────────────── */

/* Forward declaration needed for self-referential callback type. */
struct display_ui_menu;
typedef struct display_ui_menu display_ui_menu_t;

/** Callback for an action item — called when the item is activated. */
typedef void (*display_ui_action_cb_t)(void);

/**
 * Callback for a submenu item — called when the item is activated.
 * Returns a pointer to the menu to navigate to.
 */
typedef const display_ui_menu_t *(*display_ui_submenu_cb_t)(void);

typedef enum {
	DISPLAY_UI_ITEM_ACTION,   /**< Calls a function, stays on current menu. */
	DISPLAY_UI_ITEM_SUBMENU,  /**< Navigates to another menu. */
} display_ui_item_type_t;

typedef struct {
	const char            *label;
	display_ui_item_type_t type;
	union {
		display_ui_action_cb_t  action;
		display_ui_submenu_cb_t submenu;
	} cb;
} display_ui_menu_item_t;

struct display_ui_menu {
	const char                   *title;
	const display_ui_menu_item_t *items;
	int                           count;
};

/**
 * Convenience initialisers for menu item arrays.
 *
 *   static const display_ui_menu_item_t items[] = {
 *       DISPLAY_UI_ACTION_ITEM("CALIBRATE", on_calibrate),
 *       DISPLAY_UI_SUBMENU_ITEM("BACK",     go_main),
 *   };
 */
#define DISPLAY_UI_ACTION_ITEM(_label, _fn) \
	{ .label = (_label), .type = DISPLAY_UI_ITEM_ACTION,  \
	  .cb = { .action  = (_fn) } }

#define DISPLAY_UI_SUBMENU_ITEM(_label, _fn) \
	{ .label = (_label), .type = DISPLAY_UI_ITEM_SUBMENU, \
	  .cb = { .submenu = (_fn) } }

/**
 * Draw the menu screen.
 *
 * Automatically scrolls the item list to keep @p selected visible.
 * Call display_ui_flush() afterwards to push to the display.
 *
 * @param menu      Menu descriptor.
 * @param selected  Index of the currently selected item.
 */
void display_ui_draw_menu(const display_ui_menu_t *menu, int selected);

/**
 * Activate the currently selected item.
 *
 * - DISPLAY_UI_ITEM_ACTION:  calls the callback, returns @p menu unchanged.
 * - DISPLAY_UI_ITEM_SUBMENU: calls the callback, returns the new menu
 *                             (falls back to @p menu if callback returns NULL).
 *
 * @return The menu that should be displayed next.
 */
const display_ui_menu_t *display_ui_menu_activate(const display_ui_menu_t *menu,
						   int selected);

#endif /* DISPLAY_UI_H */
