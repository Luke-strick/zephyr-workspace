/*
 * display_engine — Display rendering and button input
 *
 * Normal view: 3 configurable data rows.
 * Settings view: hierarchical menu.
 *
 * Button mapping:
 *   sw0 short-press (normal): no-op
 *   sw0 long-press  (normal): enter SETTINGS menu
 *   sw0 short-press (menu):   scroll to next item
 *   sw1 short-press (normal): capture current heading as True Wind Direction
 *   sw1 short-press (menu):   activate selected item
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DISPLAY_ENGINE_H
#define DISPLAY_ENGINE_H

/**
 * Initialise the display hardware, configure GPIO buttons, and spawn the
 * display thread.
 *
 * @return 0 on success, negative errno on failure.
 */
int display_engine_init(void);

#endif /* DISPLAY_ENGINE_H */
