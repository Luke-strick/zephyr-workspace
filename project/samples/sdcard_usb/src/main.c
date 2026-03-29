/*
 * SD-card USB mass-storage passthrough
 *
 * Enables the USB mass-storage class so the SD card appears as
 * a removable drive when the board is plugged in over USB-C.
 * The console stays active on USART1 for logging.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sdcard_usb, LOG_LEVEL_INF);

int main(void)
{
	int ret;
	uint32_t sector_count;
	uint32_t sector_size;

	LOG_INF("SD-card USB mass-storage sample");

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

	/* Enable USB — the mass-storage class is registered automatically
	 * via Kconfig and uses MASS_STORAGE_DISK_NAME="SD".
	 */
	ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("usb_enable() failed: %d", ret);
		return ret;
	}

	LOG_INF("USB enabled — the SD card should appear as a removable drive.");

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
