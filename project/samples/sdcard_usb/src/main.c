/*
 * SD-card USB mass-storage passthrough
 *
 * Enables the USB mass-storage class so the SD card appears as
 * a removable drive when the board is plugged in over USB-C.
 * Console is routed to USB CDC ACM (same USB-C connector).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sdcard_usb, LOG_LEVEL_INF);

int main(void)
{
	int ret;
	uint32_t sector_count;
	uint32_t sector_size;

	/* Power up the 3V3 rail (SD card, LoRa, sensors) */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));

	/* Wait for USB CDC terminal before logging */
	const struct device *usb_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	if (device_is_ready(usb_dev)) {
		usb_enable(NULL);
		uint32_t dtr = 0;

		while (!dtr) {
			uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
			k_sleep(K_MSEC(100));
		}
	}

	LOG_INF("SD-card USB mass-storage sample");

	/* Give the SD card time to stabilise after 3V3 power-on.
	 * Some cards need up to 100 ms before the first command. */
	k_msleep(100);

	/* Check that the SD card is accessible */
	ret = disk_access_init("SD");
	if (ret) {
		LOG_ERR("disk_access_init(SD) failed: %d — is a card inserted?", ret);
		return ret;
	}

	disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	LOG_INF("SD card: %u sectors x %u bytes = %u MB",
		sector_count, sector_size,
		(sector_count / 1024) * (sector_size / 1024));

	LOG_INF("USB enabled — the SD card should appear as a removable drive.");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
