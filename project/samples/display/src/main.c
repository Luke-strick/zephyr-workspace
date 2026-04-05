/*
 * Display sample — Sharp LS027B7DH01A memory LCD
 *
 * Counts 0–999 across three rows (heading, speed, altitude).
 * Middle row is colour-inverted.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/pm/device_runtime.h>
#include <display_ui.h>

int main(void)
{
	/* Power up the 3V3 and 5V rails */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_5v)));

	/* Wait for USB CDC terminal */
	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	if (device_is_ready(usb_dev)) {
		usb_enable(NULL);
		uint32_t dtr = 0;

		while (!dtr) {
			uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
			k_sleep(K_MSEC(100));
		}
	}

	/* Initialise display */
	const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (display_ui_init(disp) < 0) {
		printk("Display not ready\n");
		return -ENODEV;
	}

	for (int n = 0; n <= 999; n++) {
		display_ui_clear();
		display_ui_draw_row(0, n, DISPLAY_UI_POSTFIX_DEG, "HDG");
		display_ui_draw_row(1, n, DISPLAY_UI_POSTFIX_KT,  "SPD");
		display_ui_draw_row(2, n, DISPLAY_UI_POSTFIX_M,   "ALT");
		display_ui_invert_row(1);
		display_ui_flush();
		k_msleep(100);
	}

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
