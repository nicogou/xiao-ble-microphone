/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Round display UI – LVGL button and note label for the Seeed XIAO Round Display.
 *
 * Self-initialising: the display thread starts automatically at boot when
 * CONFIG_APP_DISPLAY_UI=y.  No explicit init call is required from main.c.
 *
 * Build with:  --shield seeed_xiao_round_display  (required for the DTS chosen
 * node zephyr,display and zephyr,touch to be present).
 */

#pragma once
