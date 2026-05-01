/*
 * SD-card counter sample
 *
 * Mounts the SD card FAT filesystem and appends an incrementing
 * counter value once per second.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sdcard_counter, LOG_LEVEL_INF);

static FATFS fat_fs;

static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};

int main(void)
{
	int ret;
	uint32_t count = 0;

	/* Power up the 3V3 rail (SD card, LoRa, sensors) */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));

	LOG_INF("SD-card counter sample");

	/* Give the SD card time to stabilise after 3V3 power-on. */
	k_msleep(100);

	/* Check that the SD card is accessible */
	ret = disk_access_init("SD");
	if (ret) {
		LOG_ERR("disk_access_init(SD) failed: %d — is a card inserted?", ret);
		return ret;
	}

	/* Mount the FAT filesystem */
	ret = fs_mount(&mp);
	if (ret) {
		LOG_ERR("fs_mount failed: %d", ret);
		return ret;
	}

	LOG_INF("SD card mounted at %s", mp.mnt_point);

	while (1) {
		struct fs_file_t file;
		char buf[32];

		fs_file_t_init(&file);

		ret = fs_open(&file, "/SD:/count.txt",
			      FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
		if (ret) {
			LOG_ERR("fs_open failed: %d", ret);
		} else {
			int len = snprintf(buf, sizeof(buf), "%u\n", count);

			ret = fs_write(&file, buf, len);
			if (ret < 0) {
				LOG_ERR("fs_write failed: %d", ret);
			} else {
				LOG_INF("Wrote count %u", count);
			}
			fs_close(&file);
		}

		count++;
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
