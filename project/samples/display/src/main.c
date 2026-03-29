/*
 * Display sample — Sharp LS027B7DH01A memory LCD
 *
 * Draws a test pattern, then cycles a counter and status text
 * to demonstrate basic framebuffer usage on the 400x240 screen.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(display_sample, LOG_LEVEL_INF);

static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

int main(void)
{
	int ret;
	uint16_t rows, cols;
	uint8_t font_width, font_height;

	if (!device_is_ready(display)) {
		LOG_ERR("Display device not ready");
		return -ENODEV;
	}

	ret = cfb_framebuffer_init(display);
	if (ret) {
		LOG_ERR("CFB init failed: %d", ret);
		return ret;
	}

	cfb_framebuffer_clear(display, false);

	/* Pick the largest built-in font */
	int fonts = cfb_get_numof_fonts(display);

	LOG_INF("Display ready — %d font(s) available", fonts);

	for (int i = 0; i < fonts; i++) {
		cfb_get_font_size(display, i, &font_width, &font_height);
		LOG_INF("  font %d: %ux%u", i, font_width, font_height);
	}

	/* Use font 0 (typically the largest) */
	cfb_framebuffer_set_font(display, 0);
	cfb_get_font_size(display, 0, &font_width, &font_height);

	rows = cfb_get_display_parameter(display, CFB_DISPLAY_ROWS);
	cols = cfb_get_display_parameter(display, CFB_DISPLAY_COLS);
	LOG_INF("Grid: %u cols x %u rows  (font %ux%u)", cols, rows,
		font_width, font_height);

	/* Static header */
	cfb_print(display, "=== TRACKER ===", 0, 0);
	cfb_print(display, "LS027B7DH01A 400x240", 0, font_height);
	cfb_framebuffer_finalize(display);

	k_msleep(2000);

	/* Cycle a counter to show the screen refreshes */
	uint32_t count = 0;
	char buf[40];

	while (1) {
		cfb_framebuffer_clear(display, false);

		cfb_print(display, "=== TRACKER ===", 0, 0);

		snprintf(buf, sizeof(buf), "Count: %u", count++);
		cfb_print(display, buf, 0, font_height * 2);

		snprintf(buf, sizeof(buf), "Uptime: %llu s",
			 k_uptime_get() / 1000ULL);
		cfb_print(display, buf, 0, font_height * 3);

		cfb_print(display, "Sharp Memory LCD OK", 0, font_height * 5);

		cfb_framebuffer_finalize(display);

		k_msleep(1000);
	}

	return 0;
}
