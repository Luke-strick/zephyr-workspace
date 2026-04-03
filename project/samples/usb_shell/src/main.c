/*
 * USB Shell — Zephyr interactive shell over USB CDC ACM.
 *
 * Powers on the 3V3 and 5V domains so all peripherals are accessible
 * from the shell (i2c, spi, sensor, gpio, device commands).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_shell, LOG_LEVEL_INF);

int main(void)
{
	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	if (!device_is_ready(usb_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return -1;
	}

	int ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("Failed to enable USB (%d)", ret);
		return ret;
	}

	/* Power on both rails */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_5v)));

	/* Wait for host to open the port (DTR set). */
	uint32_t dtr = 0;
	while (!dtr) {
		uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	LOG_INF("USB shell ready — type 'help' for commands");
	LOG_INF("Try: device list, i2c scan, sensor get, gpio");

	return 0;
}
