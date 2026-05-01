/*
 * SD-card USB mass-storage passthrough with offline counter
 *
 * When USB-C is unplugged, the FAT filesystem is mounted and an
 * incrementing counter is appended to /SD:/count.txt once per second.
 * When USB-C is plugged in, the filesystem is unmounted and the
 * SD card is handed to the host as a USB mass-storage device.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sdcard_passthrough, LOG_LEVEL_INF);

static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = "/SD:",
};

static atomic_t usb_connected = ATOMIC_INIT(0);
static bool fs_mounted;

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	ARG_UNUSED(param);

	switch (status) {
	case USB_DC_CONNECTED:
	case USB_DC_CONFIGURED:
		atomic_set(&usb_connected, 1);
		break;
	case USB_DC_DISCONNECTED:
	case USB_DC_SUSPEND:
		atomic_set(&usb_connected, 0);
		break;
	default:
		break;
	}
}

static int write_count(uint32_t count)
{
	struct fs_file_t file;
	char buf[32];
	int ret;

	fs_file_t_init(&file);
	ret = fs_open(&file, "/SD:/count.txt",
		      FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
	if (ret) {
		return ret;
	}

	int len = snprintf(buf, sizeof(buf), "%u\n", count);

	ret = fs_write(&file, buf, len);
	fs_close(&file);
	return ret;
}

int main(void)
{
	int ret;
	uint32_t count = 0;

	/* Power up the 3V3 rail (SD card, LoRa, sensors) */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));

	LOG_INF("SD-card passthrough sample");

	/* Give the SD card time to stabilise after 3V3 power-on. */
	k_msleep(100);

	ret = disk_access_init("SD");
	if (ret) {
		LOG_ERR("disk_access_init(SD) failed: %d — is a card inserted?", ret);
		return ret;
	}

	ret = usb_enable(usb_status_cb);
	if (ret) {
		LOG_ERR("usb_enable failed: %d", ret);
		return ret;
	}

	while (1) {
		if (atomic_get(&usb_connected)) {
			if (fs_mounted) {
				LOG_INF("USB host connected — releasing SD card for mass storage");
				fs_unmount(&mp);
				fs_mounted = false;
			}
			k_sleep(K_MSEC(500));
			continue;
		}

		if (!fs_mounted) {
			ret = fs_mount(&mp);
			if (ret) {
				LOG_ERR("fs_mount failed: %d", ret);
				k_sleep(K_SECONDS(1));
				continue;
			}
			fs_mounted = true;
			LOG_INF("USB disconnected — counter writes resumed");
		}

		ret = write_count(count);
		if (ret < 0) {
			LOG_ERR("write_count failed: %d", ret);
		} else {
			LOG_INF("Wrote count %u", count);
			count++;
		}

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
