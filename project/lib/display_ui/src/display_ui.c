/*
 * display_ui — 1bpp framebuffer UI library
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <display_ui.h>
#include <zephyr/drivers/display.h>
#include <string.h>

/* ── Framebuffer dimensions ─────────────────────────────────────────────── */

#define FB_W      CONFIG_DISPLAY_UI_FB_W
#define FB_H      CONFIG_DISPLAY_UI_FB_H
#define FB_STRIDE (FB_W / 8)
#define FB_SIZE   (FB_STRIDE * FB_H)

static uint8_t fb[FB_SIZE];
static const struct device *display_dev;

/* ── Portrait virtual canvas ─────────────────────────────────────────────── */
/*
 * Portrait mode rotates 90° CCW so the taller dimension is vertical.
 *   Physical (x,y): FB_W wide × FB_H tall
 *   Virtual  (px,py): P_W wide × P_H tall
 *
 * Mapping  (portrait):  cfb_x = P_H - 1 - py,  cfb_y = px
 * Mapping  (landscape): cfb_x = px,             cfb_y = py
 */
#if CONFIG_DISPLAY_UI_PORTRAIT
#  define P_W  FB_H
#  define P_H  FB_W
#else
#  define P_W  FB_W
#  define P_H  FB_H
#endif

/* ── Layout constants derived from Kconfig ───────────────────────────────── */

#define ROWS       CONFIG_DISPLAY_UI_ROWS
#define DIGIT_T    CONFIG_DISPLAY_UI_DIGIT_T
#define COL_GAP    CONFIG_DISPLAY_UI_COL_GAP
#define INSET      CONFIG_DISPLAY_UI_ROW_INSET

/* Each row occupies an equal vertical slice of the virtual canvas */
#define THIRD      (P_H / ROWS)

/*
 * Digit size auto-derived from available row height.
 * Ratios match the original 400×240 Sharp LCD layout:
 *   DIGIT_H ≈ 81% of THIRD, DIGIT_W ≈ 57% of DIGIT_H
 */
#define DIGIT_H    ((THIRD * 81) / 100)
#define DIGIT_W    ((DIGIT_H * 57) / 100)

/* Total width of three digits + two gaps */
#define TOTAL_ROW_W  (3 * DIGIT_W + 2 * COL_GAP)

/* Shift numbers left to make room for the postfix on the right */
#define NUMBER_SHIFT  CONFIG_DISPLAY_UI_NUMBER_SHIFT
#define START_PX      ((P_W - TOTAL_ROW_W) / 2 - NUMBER_SHIFT)

/* Postfix / label font scale */
#define KT_SCALE   2
#define DEG_SCALE  4
#define KT_W       (5 * KT_SCALE)
#define KT_GAP     (2 * KT_SCALE)
#define DEG_W      (5 * DEG_SCALE)

/* ── Raw framebuffer helpers ─────────────────────────────────────────────── */

void display_ui_clear(void)
{
	memset(fb, 0xFF, FB_SIZE);   /* MONO01: 1 = white */
}

void display_ui_flush(void)
{
	struct display_buffer_descriptor d = {
		.buf_size = FB_SIZE,
		.width    = FB_W,
		.height   = FB_H,
		.pitch    = FB_W,
	};

	display_write(display_dev, 0, 0, &d, fb);
}

/* Set one pixel to black (bit = 0, LSB-first) */
static inline void fb_set(int x, int y)
{
	if ((unsigned)x < FB_W && (unsigned)y < FB_H) {
		fb[y * FB_STRIDE + x / 8] &= ~BIT(x & 7);
	}
}

/* Fill a rectangle in physical framebuffer coordinates */
static void fb_fill(int x0, int y0, int x1, int y1)
{
	for (int y = y0; y <= y1; y++) {
		for (int x = x0; x <= x1; x++) {
			fb_set(x, y);
		}
	}
}

/* XOR-invert a rectangle in physical framebuffer coordinates */
static void fb_invert_region(int x0, int y0, int x1, int y1)
{
	for (int y = y0; y <= y1; y++) {
		uint8_t *row = &fb[y * FB_STRIDE];
		int bx0 = x0 / 8;
		int bx1 = x1 / 8;

		for (int b = bx0; b <= bx1; b++) {
			uint8_t mask = 0xFF;

			if (b == bx0) {
				mask &= 0xFF << (x0 & 7);
			}
			if (b == bx1) {
				mask &= 0xFF >> (7 - (x1 & 7));
			}
			row[b] ^= mask;
		}
	}
}

/* ── Virtual-canvas drawing ──────────────────────────────────────────────── */

/* Fill a rectangle in virtual portrait coordinates */
static void fill_rect(int px0, int py0, int px1, int py1)
{
#if CONFIG_DISPLAY_UI_PORTRAIT
	int cx0 = P_H - 1 - py1;
	int cx1 = P_H - 1 - py0;

	fb_fill(cx0, px0, cx1, px1);
#else
	fb_fill(px0, py0, px1, py1);
#endif
}

/* py origin for a given row, centred within its slice */
static inline int row_py(int row)
{
	return row * THIRD + (THIRD - DIGIT_H) / 2;
}

/* ── Mini bitmap font (5 wide × 8 tall, bit 4 = leftmost column) ─────────── */

static const uint8_t font_k[8] = {
	0x11, 0x12, 0x14, 0x18, 0x18, 0x14, 0x12, 0x11,
};
static const uint8_t font_t[8] = {
	0x04, 0x04, 0x1F, 0x04, 0x04, 0x04, 0x03, 0x00,
};
static const uint8_t font_deg[8] = {
	0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t font_m[8] = {
	0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11, 0x00,
};
static const uint8_t font_upper[26][8] = {
	{0x04,0x0A,0x11,0x1F,0x11,0x11,0x11,0x00}, /* A */
	{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E,0x00}, /* B */
	{0x0E,0x11,0x10,0x10,0x10,0x11,0x0E,0x00}, /* C */
	{0x1C,0x12,0x11,0x11,0x11,0x12,0x1C,0x00}, /* D */
	{0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F,0x00}, /* E */
	{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10,0x00}, /* F */
	{0x0E,0x11,0x10,0x17,0x11,0x11,0x0F,0x00}, /* G */
	{0x11,0x11,0x11,0x1F,0x11,0x11,0x11,0x00}, /* H */
	{0x0E,0x04,0x04,0x04,0x04,0x04,0x0E,0x00}, /* I */
	{0x07,0x02,0x02,0x02,0x02,0x12,0x0C,0x00}, /* J */
	{0x11,0x12,0x14,0x18,0x14,0x12,0x11,0x00}, /* K */
	{0x10,0x10,0x10,0x10,0x10,0x10,0x1F,0x00}, /* L */
	{0x11,0x1B,0x15,0x11,0x11,0x11,0x11,0x00}, /* M */
	{0x11,0x19,0x15,0x13,0x11,0x11,0x11,0x00}, /* N */
	{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E,0x00}, /* O */
	{0x1E,0x11,0x11,0x1E,0x10,0x10,0x10,0x00}, /* P */
	{0x0E,0x11,0x11,0x11,0x15,0x12,0x0D,0x00}, /* Q */
	{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11,0x00}, /* R */
	{0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E,0x00}, /* S */
	{0x1F,0x04,0x04,0x04,0x04,0x04,0x04,0x00}, /* T */
	{0x11,0x11,0x11,0x11,0x11,0x11,0x0E,0x00}, /* U */
	{0x11,0x11,0x11,0x11,0x0A,0x0A,0x04,0x00}, /* V */
	{0x11,0x11,0x11,0x15,0x15,0x1B,0x11,0x00}, /* W */
	{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11,0x00}, /* X */
	{0x11,0x11,0x0A,0x04,0x04,0x04,0x04,0x00}, /* Y */
	{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F,0x00}, /* Z */
};
static const uint8_t font_digits[10][8] = {
	{0x0E,0x11,0x13,0x15,0x19,0x11,0x0E,0x00}, /* 0 */
	{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E,0x00}, /* 1 */
	{0x0E,0x11,0x01,0x06,0x08,0x10,0x1F,0x00}, /* 2 */
	{0x0E,0x11,0x01,0x06,0x01,0x11,0x0E,0x00}, /* 3 */
	{0x02,0x06,0x0A,0x12,0x1F,0x02,0x02,0x00}, /* 4 */
	{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E,0x00}, /* 5 */
	{0x06,0x08,0x10,0x1E,0x11,0x11,0x0E,0x00}, /* 6 */
	{0x1F,0x01,0x02,0x04,0x08,0x08,0x08,0x00}, /* 7 */
	{0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E,0x00}, /* 8 */
	{0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C,0x00}, /* 9 */
};
static const uint8_t font_space[8]   = {0};
static const uint8_t font_unknown[8] = {
	0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x00,
};

static const uint8_t *get_char_bitmap(char c)
{
	if (c >= 'A' && c <= 'Z') { return font_upper[c - 'A']; }
	if (c >= 'a' && c <= 'z') { return font_upper[c - 'a']; }
	if (c >= '0' && c <= '9') { return font_digits[c - '0']; }
	if (c == ' ')              { return font_space; }
	return font_unknown;
}

static void draw_char(int px, int py, const uint8_t *bitmap, int scale)
{
	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 5; col++) {
			if (bitmap[row] & BIT(4 - col)) {
				fill_rect(px + col * scale,
					  py + row * scale,
					  px + col * scale + scale - 1,
					  py + row * scale + scale - 1);
			}
		}
	}
}

/* ── Postfix rendering ───────────────────────────────────────────────────── */

static void draw_postfix(int row, display_ui_postfix_t pf)
{
	if (pf == DISPLAY_UI_POSTFIX_NONE) {
		return;
	}

	int py = row_py(row);

	if (pf == DISPLAY_UI_POSTFIX_KT) {
		int px = P_W - 2 * KT_W - KT_GAP - 4;

		draw_char(px,                  py, font_k, KT_SCALE);
		draw_char(px + KT_W + KT_GAP, py, font_t, KT_SCALE);
	} else if (pf == DISPLAY_UI_POSTFIX_DEG) {
		int px = P_W - DEG_W - 4;

		draw_char(px, py, font_deg, DEG_SCALE);
	} else if (pf == DISPLAY_UI_POSTFIX_M) {
		int px = P_W - KT_W - 4;

		draw_char(px, py, font_m, KT_SCALE);
	}
}

/* ── Row label rendering ─────────────────────────────────────────────────── */

#define LABEL_RIGHT_MARGIN  3

static void draw_row_label(int row, const char *text, int scale)
{
	int len = 0;
	for (const char *p = text; *p; p++) { len++; }

	int char_h  = 8 * scale;
	int char_gap = scale;
	int total_h  = len * char_h + (len > 1 ? (len - 1) * char_gap : 0);
	int row_top  = row * THIRD;
	int py       = row_top + (THIRD - total_h) / 2;
	int px       = P_W - 5 * scale - LABEL_RIGHT_MARGIN;

	for (int i = 0; i < len; i++) {
		draw_char(px, py, get_char_bitmap(text[i]), scale);
		py += char_h + char_gap;
	}
}

/* ── 7-segment digit rendering ───────────────────────────────────────────── */

#define SEG_A  BIT(6)
#define SEG_B  BIT(5)
#define SEG_C  BIT(4)
#define SEG_D  BIT(3)
#define SEG_E  BIT(2)
#define SEG_F  BIT(1)
#define SEG_G  BIT(0)

static const uint8_t seg_map[10] = {
	SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F,        /* 0 */
	SEG_B|SEG_C,                                  /* 1 */
	SEG_A|SEG_B|SEG_D|SEG_E|SEG_G,               /* 2 */
	SEG_A|SEG_B|SEG_C|SEG_D|SEG_G,               /* 3 */
	SEG_B|SEG_C|SEG_F|SEG_G,                      /* 4 */
	SEG_A|SEG_C|SEG_D|SEG_F|SEG_G,               /* 5 */
	SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,         /* 6 */
	SEG_A|SEG_B|SEG_C,                            /* 7 */
	SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,  /* 8 */
	SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G,         /* 9 */
};

static void draw_digit(int col, int row, int d)
{
	int x = START_PX + col * (DIGIT_W + COL_GAP);
	int y = row_py(row);
	int W = DIGIT_W, H = DIGIT_H, T = DIGIT_T, I = INSET;
	uint8_t s = seg_map[d];

	if (s & SEG_A) { fill_rect(x+T+I,  y,         x+W-T-I, y+T);          }
	if (s & SEG_B) { fill_rect(x+W-T,  y+T+I,     x+W,     y+H/2-I);      }
	if (s & SEG_C) { fill_rect(x+W-T,  y+H/2+I,   x+W,     y+H-T-I);      }
	if (s & SEG_D) { fill_rect(x+T+I,  y+H-T,     x+W-T-I, y+H);          }
	if (s & SEG_E) { fill_rect(x,       y+H/2+I,   x+T,     y+H-T-I);      }
	if (s & SEG_F) { fill_rect(x,       y+T+I,     x+T,     y+H/2-I);      }
	if (s & SEG_G) { fill_rect(x+T+I,  y+H/2-T/2, x+W-T-I, y+H/2+T/2);   }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int display_ui_init(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	display_dev = dev;
	display_blanking_off(dev);
	display_ui_clear();

	return 0;
}

void display_ui_draw_row(int row, int n, display_ui_postfix_t pf,
			 const char *label)
{
	draw_digit(0, row, (n / 100) % 10);
	draw_digit(1, row, (n / 10)  % 10);
	draw_digit(2, row,  n        % 10);
	draw_postfix(row, pf);

	if (label && *label) {
		draw_row_label(row, label, CONFIG_DISPLAY_UI_LABEL_SCALE);
	}
}

void display_ui_invert_row(int row)
{
#if CONFIG_DISPLAY_UI_PORTRAIT
	/* Portrait row r spans virtual py = [r*THIRD, (r+1)*THIRD).
	 * cfb_x = P_H - 1 - py  →  x0 = P_H-(r+1)*THIRD, x1 = P_H-1-r*THIRD */
	int x0 = P_H - (row + 1) * THIRD;
	int x1 = P_H - 1 - row * THIRD;

	fb_invert_region(x0, 0, x1, FB_H - 1);
#else
	int y0 = row * THIRD;
	int y1 = (row + 1) * THIRD - 1;

	fb_invert_region(0, y0, FB_W - 1, y1);
#endif
}
