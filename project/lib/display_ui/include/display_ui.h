/*
 * display_ui — 1bpp framebuffer UI for Sharp-style memory LCDs
 *
 * Renders three rows of large 7-segment digits in portrait orientation.
 * Configure via Kconfig (CONFIG_DISPLAY_UI_*).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <zephyr/device.h>

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

/**
 * Fill the framebuffer with white (clears all pixels).
 */
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

/**
 * Push the framebuffer to the display.
 */
void display_ui_flush(void);

#endif /* DISPLAY_UI_H */
