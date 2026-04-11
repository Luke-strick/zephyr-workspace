/*
 * GPS sample — MAX-M10S via GNSS subsystem
 *
 * Enables the 3V3 rail (GPS) and 5V rail (display), then:
 *  - Sends UBX-CFG-VALSET to enable GPS + Galileo + GLONASS + BeiDou
 *  - RTT: prints position, fix status, and satellite count each epoch
 *  - Display: shows 999 on all rows until fix, then SOG / SAT / COG
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
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <display_ui.h>

static const struct device *gnss_dev  = DEVICE_DT_GET(DT_NODELABEL(gnss));
static const struct device *gps_uart  = DEVICE_DT_GET(DT_NODELABEL(usart3));

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
 * UBX-CFG-VALSET (class 0x06, id 0x8A) — M10 configuration interface.
 *
 * Enables GPS + Galileo + GLONASS + BeiDou and saves to RAM + BBR + Flash
 * so the setting survives power cycles.
 *
 * Key IDs (little-endian 32-bit, type L = 1-byte value):
 *   0x10310001  CFG-SIGNAL-GPS_ENA
 *   0x10310021  CFG-SIGNAL-GAL_ENA   (Galileo)
 *   0x10310025  CFG-SIGNAL-GLO_ENA   (GLONASS)
 *   0x10310022  CFG-SIGNAL-BDS_ENA   (BeiDou)
 *
 * Verify these against Table "CFG-SIGNAL" in the MAX-M10S Integration Manual
 * (UBX-22009821) if you are on a different firmware version.
 */
static void ubx_enable_all_constellations(void)
{
	static const uint8_t payload[] = {
		0x00,                         /* version */
		0x07,                         /* layers: RAM | BBR | Flash */
		0x00, 0x00,                   /* reserved */
		/* CFG-SIGNAL-GPS_ENA = 1 */
		0x01, 0x00, 0x31, 0x10,  0x01,
		/* CFG-SIGNAL-GAL_ENA = 1 (Galileo) */
		0x21, 0x00, 0x31, 0x10,  0x01,
		/* CFG-SIGNAL-GLO_ENA = 1 (GLONASS) */
		0x25, 0x00, 0x31, 0x10,  0x01,
		/* CFG-SIGNAL-BDS_ENA = 1 (BeiDou) */
		0x22, 0x00, 0x31, 0x10,  0x01,
	};

	ubx_send(0x06, 0x8A, payload, sizeof(payload));
}

/* ── UBX-MON-RF poll ────────────────────────────────────────────────────── */

/*
 * Read a single byte from the UART with a spin-loop timeout.
 * Returns 0 on success, -1 on timeout.
 */
static int ubx_poll_in(uint8_t *byte, k_timeout_t timeout)
{
	int64_t deadline = k_uptime_get() + k_ticks_to_ms_floor64(timeout.ticks);

	while (k_uptime_get() < deadline) {
		if (uart_poll_in(gps_uart, byte) == 0) {
			return 0;
		}
		k_usleep(100);
	}
	return -1;
}

/*
 * Poll UBX-MON-RF (class 0x0A, id 0x38) and print antenna status.
 *
 * Response per RF block (24 bytes each):
 *   antStatus: 0=INIT 1=DONTKNOW 2=OK 3=SHORT 4=OPEN
 *   antPower:  0=OFF  1=ON       2=DONTKNOW
 */
static void ubx_poll_mon_rf(void)
{
	/* Send poll request (empty payload) */
	ubx_send(0x0A, 0x38, NULL, 0);

	/* Wait for UBX sync header — skip any NMEA traffic in the stream */
	uint8_t b;
	int attempts = 0;

	while (attempts++ < 2000) {
		if (ubx_poll_in(&b, K_MSEC(500)) < 0) {
			printk("MON-RF: timeout waiting for response\n");
			return;
		}
		if (b != 0xB5) {
			continue;
		}
		if (ubx_poll_in(&b, K_MSEC(50)) < 0 || b != 0x62) {
			continue;
		}

		/* Read class + id */
		uint8_t hdr[4];

		for (int i = 0; i < 4; i++) {
			if (ubx_poll_in(&hdr[i], K_MSEC(50)) < 0) {
				printk("MON-RF: header read failed\n");
				return;
			}
		}

		uint8_t cls = hdr[0], id = hdr[1];
		uint16_t len = (uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8);

		if (cls != 0x0A || id != 0x38) {
			/* Not our message — skip payload + 2 checksum bytes */
			for (uint16_t i = 0; i < len + 2u; i++) {
				if (ubx_poll_in(&b, K_MSEC(50)) < 0) {
					return;
				}
			}
			continue;
		}

		/* Read MON-RF payload (max reasonable size: 4 + 2*24 = 52) */
		if (len > 128) {
			printk("MON-RF: unexpected length %u\n", len);
			return;
		}

		uint8_t payload[128];

		for (uint16_t i = 0; i < len; i++) {
			if (ubx_poll_in(&payload[i], K_MSEC(50)) < 0) {
				printk("MON-RF: payload read failed\n");
				return;
			}
		}

		/* Skip checksum */
		ubx_poll_in(&b, K_MSEC(50));
		ubx_poll_in(&b, K_MSEC(50));

		/* Parse header: version(1) nBlocks(1) reserved(2) */
		uint8_t n_blocks = payload[1];

		printk("MON-RF: %u RF block(s)\n", n_blocks);

		static const char *ant_status_str[] = {
			"INIT", "DONTKNOW", "OK", "SHORT", "OPEN"
		};
		static const char *ant_power_str[] = {
			"OFF", "ON", "DONTKNOW"
		};

		for (uint8_t blk = 0; blk < n_blocks; blk++) {
			uint8_t *p = &payload[4 + blk * 24];
			uint8_t block_id   = p[0];
			uint8_t jam_state  = p[1] & 0x03;
			uint8_t ant_status = p[2];
			uint8_t ant_power  = p[3];
			uint16_t noise     = (uint16_t)p[12] | ((uint16_t)p[13] << 8);
			uint16_t agc       = (uint16_t)p[14] | ((uint16_t)p[15] << 8);
			uint8_t jam_ind    = p[16];

			printk("  block %u: antStatus=%s antPower=%s "
			       "jamState=%u jamInd=%u noise=%u agc=%u\n",
			       block_id,
			       ant_status < 5 ? ant_status_str[ant_status] : "?",
			       ant_power  < 3 ? ant_power_str[ant_power]   : "?",
			       jam_state, jam_ind, noise, agc);
		}
		return;
	}

	printk("MON-RF: no matching response found\n");
}

/* Data shared between GNSS callbacks and the display loop. */
static atomic_t g_has_fix;
static atomic_t g_sog_kt;   /* speed over ground, whole knots  */
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
		/* speed: mm/s → knots  (1 kt = 514 mm/s) */
		int sog = MIN((int)(data->nav_data.speed / 514), 999);
		/* bearing: milli-degrees → degrees */
		int cog = MIN((int)(data->nav_data.bearing / 1000), 999);

		atomic_set(&g_sog_kt,  sog);
		atomic_set(&g_cog_deg, cog);

		int64_t lat = data->nav_data.latitude;
		int64_t lon = data->nav_data.longitude;

		printk("GPS:%s,sats=%u,lat=%lld.%09lld,lon=%lld.%09lld,alt=%d.%03dm,"
		       "sog=%dkt,cog=%ddeg\n",
		       fix_str(data->info.fix_status),
		       data->info.satellites_cnt,
		       (long long)(lat / 1000000000LL),
		       (long long)llabs(lat % 1000000000LL),
		       (long long)(lon / 1000000000LL),
		       (long long)llabs(lon % 1000000000LL),
		       data->nav_data.altitude / 1000,
		       abs(data->nav_data.altitude % 1000),
		       sog, cog);
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

	/* Give the MAX-M10S time to boot before sending UBX config */
	k_msleep(200);

	/* Enable all constellations — dramatically improves cold-start TTFF.
	 * Settings are saved to flash so this only needs to run once, but
	 * sending it every boot is harmless. */
	ubx_enable_all_constellations();
	printk("UBX: constellation config sent\n");

	/* One-shot poll for antenna / RF status */
	ubx_poll_mon_rf();

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

	printk("GPS sample starting — waiting for MAX-M10S data\n");
	printk("Place the device outdoors for a satellite fix.\n");

	while (1) {
		display_ui_clear();

		if (!atomic_get(&g_has_fix)) {
			display_ui_draw_row(0, 999, DISPLAY_UI_POSTFIX_KT,   "SOG");
			display_ui_draw_row(1, 999, DISPLAY_UI_POSTFIX_NONE, "SAT");
			display_ui_draw_row(2, 999, DISPLAY_UI_POSTFIX_DEG,  "COG");
		} else {
			display_ui_draw_row(0, (int)atomic_get(&g_sog_kt),
					    DISPLAY_UI_POSTFIX_KT,   "SOG");
			display_ui_draw_row(1, (int)atomic_get(&g_sat_cnt),
					    DISPLAY_UI_POSTFIX_NONE, "SAT");
			display_ui_draw_row(2, (int)atomic_get(&g_cog_deg),
					    DISPLAY_UI_POSTFIX_DEG,  "COG");
		}

		display_ui_flush();
		k_msleep(500);
	}

	return 0;
}
