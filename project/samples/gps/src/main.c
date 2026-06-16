/*
 * GPS sample — BN220 (u-blox M8030) via GNSS subsystem
 *
 * Enables the 3V3 rail (GPS) and 5V rail (display), then:
 *  - Sends UBX-CFG-GNSS to enable GPS + SBAS + GLONASS
 *  - RTT: prints position, fix status, and satellite count each epoch
 *  - Display: SAT updates live; SOG and COG show 999 until fix
 *
 * Row layout:
 *   Row 0 — SOG  (speed over ground, knots)
 *   Row 1 — SAT  (tracked satellite count)
 *   Row 2 — COG  (course over ground, degrees)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <display_ui.h>

static const struct device *gnss_dev  = DEVICE_DT_GET(DT_NODELABEL(gnss));
static const struct device *gps_uart  = DEVICE_DT_GET(DT_NODELABEL(usart1));

/* ── UBX helpers ─────────────────────────────────────────────────────────── */

/*
 * Send a UBX frame via uart_poll_out.
 * Safe to call while modem_chat owns the RX IRQ — modem_chat never enables
 * the TX IRQ for a receive-only GNSS driver, so poll_out doesn't race.
 */
static void ubx_send(uint8_t cls, uint8_t id,
		     const uint8_t *payload, uint16_t len)
{
	uint8_t ck_a = 0, ck_b = 0;

#define UBX_FEED(b)  do { uint8_t _b = (b); ck_a += _b; ck_b += ck_a; \
			   uart_poll_out(gps_uart, _b); } while (0)

	uart_poll_out(gps_uart, 0xB5);  /* sync 1 — not in checksum */
	uart_poll_out(gps_uart, 0x62);  /* sync 2 — not in checksum */

	UBX_FEED(cls);
	UBX_FEED(id);
	UBX_FEED((uint8_t)(len & 0xFF));
	UBX_FEED((uint8_t)(len >> 8));

	for (uint16_t i = 0; i < len; i++) {
		UBX_FEED(payload[i]);
	}

#undef UBX_FEED

	uart_poll_out(gps_uart, ck_a);
	uart_poll_out(gps_uart, ck_b);
}

/*
 * UBX-CFG-GNSS (class 0x06, id 0x3E) — M8 constellation configuration.
 *
 * Enables GPS (L1C/A) + SBAS + GLONASS (L1) on the M8030.
 * The M8030 supports concurrent GPS+GLONASS with up to 32 tracking channels.
 *
 * Each 8-byte block: gnssId, resTrkCh, maxTrkCh, reserved, flags(LE u32)
 *   flags bit 0        — enable
 *   flags bits 16-23   — sigCfgMask (0x01 = L1C/A for GPS/SBAS, L1 for GLONASS)
 */
static void ubx_enable_constellations(void)
{
	static const uint8_t payload[] = {
		0x00,        /* msgVer */
		0x00,        /* numTrkChHw (read-only, ignored) */
		0xFF,        /* numTrkChUse (0xFF = use hardware max) */
		0x03,        /* numConfigBlocks */
		/* GPS: gnssId=0, resTrkCh=8, maxTrkCh=16, L1C/A enabled */
		0x00, 0x08, 0x10, 0x00,  0x01, 0x00, 0x01, 0x00,
		/* SBAS: gnssId=1, resTrkCh=1, maxTrkCh=3, L1C/A enabled */
		0x01, 0x01, 0x03, 0x00,  0x01, 0x00, 0x01, 0x00,
		/* GLONASS: gnssId=6, resTrkCh=8, maxTrkCh=14, L1 enabled */
		0x06, 0x08, 0x0E, 0x00,  0x01, 0x00, 0x01, 0x00,
	};

	ubx_send(0x06, 0x3E, payload, sizeof(payload));
}


/* Data shared between GNSS callbacks and the display loop. */
static atomic_t g_has_fix;
static atomic_t g_sog_kt;   /* speed over ground, tenths of knots (123 = 12.3 kt) */
static atomic_t g_cog_deg;  /* course over ground, whole degrees */
static atomic_t g_sat_cnt;  /* tracked satellite count */

/* ── GNSS callbacks ──────────────────────────────────────────────────────── */

static const char *fix_str(enum gnss_fix_status status)
{
	switch (status) {
	case GNSS_FIX_STATUS_NO_FIX:        return "NO_FIX";
	case GNSS_FIX_STATUS_GNSS_FIX:      return "FIX";
	case GNSS_FIX_STATUS_DGNSS_FIX:     return "DGNSS";
	case GNSS_FIX_STATUS_ESTIMATED_FIX: return "EST";
	default:                             return "UNK";
	}
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	bool fixed = data->info.fix_status >= GNSS_FIX_STATUS_GNSS_FIX;

	atomic_set(&g_has_fix, fixed ? 1 : 0);

	if (fixed) {
		/* speed: mm/s → tenths of knots  (1 kt = 514 mm/s, 0.1 kt = 51.4 mm/s) */
		int sog = MIN((int)(data->nav_data.speed * 10 / 514), 999);
		/* bearing: milli-degrees → degrees */
		int cog = MIN((int)(data->nav_data.bearing / 1000), 999);

		atomic_set(&g_sog_kt,  sog);
		atomic_set(&g_cog_deg, cog);

		int64_t lat = data->nav_data.latitude;
		int64_t lon = data->nav_data.longitude;

		printk("GPS:%s,sats=%u,lat=%lld.%09lld,lon=%lld.%09lld,alt=%d.%03dm,"
		       "sog=%d.%dkt,cog=%ddeg\n",
		       fix_str(data->info.fix_status),
		       data->info.satellites_cnt,
		       (long long)(lat / 1000000000LL),
		       (long long)llabs(lat % 1000000000LL),
		       (long long)(lon / 1000000000LL),
		       (long long)llabs(lon % 1000000000LL),
		       data->nav_data.altitude / 1000,
		       abs(data->nav_data.altitude % 1000),
		       sog / 10, sog % 10, cog);
	} else {
		printk("GPS:%s,sats=%u,waiting...\n",
		       fix_str(data->info.fix_status),
		       data->info.satellites_cnt);
	}
}

GNSS_DATA_CALLBACK_DEFINE(NULL, gnss_data_cb);

static void gnss_satellites_cb(const struct device *dev,
				const struct gnss_satellite *sats,
				uint16_t size)
{
	unsigned int tracked = 0;

	for (uint16_t i = 0; i < size; i++) {
		if (sats[i].is_tracked) {
			tracked++;
		}
	}

	atomic_set(&g_sat_cnt, MIN((int)tracked, 999));
	printk("SAT:visible=%u,tracked=%u\n", size, tracked);
}

GNSS_SATELLITES_CALLBACK_DEFINE(NULL, gnss_satellites_cb);

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
	/* Bring up 3V3 rail — powers GPS, LoRa, sensors */
	int ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_3v3)));

	printk("3V3 rail: %s (ret=%d)\n", ret == 0 ? "ON" : "FAILED", ret);

	/* Give the BN220 time to boot before sending UBX config */
	k_msleep(200);

	/* Enable GPS + SBAS + GLONASS — improves cold-start TTFF.
	 * M8 retains this in battery-backed RAM, but re-sending each boot
	 * is harmless. */
	ubx_enable_constellations();
	printk("UBX: constellation config sent\n");

	/* Resume the GNSS driver — with CONFIG_PM_DEVICE=y the driver starts
	 * suspended (pm_device_init_suspended) and never opens the modem pipe
	 * until explicitly resumed. Use pm_device_action_run (basic PM API)
	 * because gnss_nmea_generic_init does NOT call pm_device_runtime_enable,
	 * so pm_device_runtime_get would silently no-op. */
	ret = pm_device_action_run(gnss_dev, PM_DEVICE_ACTION_RESUME);
	if (ret < 0 && ret != -EALREADY) {
		printk("GNSS resume failed (ret=%d)\n", ret);
		return ret;
	}
	printk("GNSS resumed (ret=%d)\n", ret);

	/* Bring up 5V rail — powers display */
	pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(power_5v)));

	const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (display_ui_init(disp) < 0) {
		printk("Display not ready\n");
		return -ENODEV;
	}

	if (!device_is_ready(gnss_dev)) {
		printk("GNSS device not ready\n");
		return -ENODEV;
	}

	printk("GPS sample starting — waiting for BN220 data\n");
	printk("Place the device outdoors for a satellite fix.\n");

	while (1) {
		display_ui_clear();

		bool fixed = atomic_get(&g_has_fix);

		display_ui_draw_row_d1(0, fixed ? (int)atomic_get(&g_sog_kt) : 999,
				       DISPLAY_UI_POSTFIX_KT, "SOG");
		display_ui_draw_row(1, (int)atomic_get(&g_sat_cnt),
				    DISPLAY_UI_POSTFIX_NONE, "SAT");
		display_ui_draw_row(2, fixed ? (int)atomic_get(&g_cog_deg) : 999,
				    DISPLAY_UI_POSTFIX_DEG,  "COG");


		display_ui_flush();
		k_msleep(500);
	}

	return 0;
}
