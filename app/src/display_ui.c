/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Round display UI – LVGL-based UI for the Seeed XIAO Round Display (240×240).
 *
 * Layout:
 *   - A "▶ Start / ■ Stop" button centred in the upper half of the screen.
 *   - A large note label (e.g. "A4", "C#3") centred in the lower half,
 *     visible only while pitch detection is running.
 *
 * Threading model:
 *   A dedicated thread owns all LVGL calls (lv_task_handler + widget updates).
 *   The pitch note callback runs in the pitch thread; it writes the detected
 *   note into a mutex-protected buffer that the display thread drains every
 *   iteration.  The button toggle request is handled in the display thread's
 *   main loop (not inside the event callback) so that pitch_stop() – which
 *   briefly blocks – does not freeze LVGL event processing.
 */

#include "display_ui.h"
#include "pitch.h"

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>

LOG_MODULE_REGISTER(display_ui, CONFIG_APP_LOG_LEVEL);

/* =========================================================================
 * Note buffer – written by the pitch thread, drained by the display thread
 * ========================================================================= */

static K_MUTEX_DEFINE(note_mutex);
static char note_buf[8];
static bool note_dirty;

/* =========================================================================
 * LVGL widget handles (owned by the display thread)
 * ========================================================================= */

static lv_obj_t *g_btn;
static lv_obj_t *g_btn_label;
static lv_obj_t *g_note_label;

/* =========================================================================
 * Toggle flag – set in the event callback, consumed in the main loop
 * ========================================================================= */

static volatile bool toggle_requested;

/* =========================================================================
 * Pitch note callback (pitch thread context)
 * ========================================================================= */

static void on_note_detected(const char *note, int octave)
{
    char tmp[8];

    snprintf(tmp, sizeof(tmp), "%s%d", note, octave);

    k_mutex_lock(&note_mutex, K_FOREVER);
    strncpy(note_buf, tmp, sizeof(note_buf) - 1);
    note_buf[sizeof(note_buf) - 1] = '\0';
    note_dirty = true;
    k_mutex_unlock(&note_mutex);
}

/* =========================================================================
 * LVGL button event handler (display thread context via lv_task_handler)
 * ========================================================================= */

static void btn_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        toggle_requested = true;
        /* Disable immediately to prevent double-press. */
        lv_obj_add_state(g_btn, LV_STATE_DISABLED);
    }
}

/* =========================================================================
 * Display thread
 * ========================================================================= */

#define DISPLAY_THREAD_STACK_SIZE 4096
#define DISPLAY_THREAD_PRIORITY   7  /* same level as the pitch thread */

static void display_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(disp)) {
        LOG_ERR("display not ready");
        return;
    }
    display_blanking_off(disp);

    /* ---- Build the UI ---- */

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

    /* Toggle button – upper centre */
    g_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(g_btn, 150, 60);
    lv_obj_align(g_btn, LV_ALIGN_CENTER, 0, -30);
    lv_obj_add_event_cb(g_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    g_btn_label = lv_label_create(g_btn);
    lv_label_set_text(g_btn_label, LV_SYMBOL_PLAY " Start");
    lv_obj_center(g_btn_label);

    /* Note label – lower centre, hidden until pitch is running */
    g_note_label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(g_note_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(g_note_label, lv_color_white(), 0);
    lv_label_set_text(g_note_label, "---");
    lv_obj_align(g_note_label, LV_ALIGN_CENTER, 0, 50);
    lv_obj_add_flag(g_note_label, LV_OBJ_FLAG_HIDDEN);

    pitch_set_note_cb(on_note_detected);

    bool was_active = false;

    while (true) {
        /* ---- Process deferred toggle (pitch_stop may block briefly) ---- */
        if (toggle_requested) {
            toggle_requested = false;
            if (pitch_is_active()) {
                pitch_stop();
            } else {
                pitch_start();
            }
        }

        /* ---- Sync button label and note visibility with pitch state ---- */
        bool is_active = pitch_is_active();

        if (is_active != was_active) {
            was_active = is_active;
            if (is_active) {
                lv_label_set_text(g_btn_label, LV_SYMBOL_STOP " Stop");
                lv_label_set_text(g_note_label, "---");
                lv_obj_clear_flag(g_note_label, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_label_set_text(g_btn_label, LV_SYMBOL_PLAY " Start");
                lv_obj_add_flag(g_note_label, LV_OBJ_FLAG_HIDDEN);
            }
            /* Re-enable button now that the transition is complete. */
            lv_obj_clear_state(g_btn, LV_STATE_DISABLED);
        }

        /* ---- Push latest detected note to the label ---- */
        if (is_active) {
            char tmp[8] = {0};
            bool dirty  = false;

            k_mutex_lock(&note_mutex, K_FOREVER);
            if (note_dirty) {
                memcpy(tmp, note_buf, sizeof(tmp));
                note_dirty = false;
                dirty = true;
            }
            k_mutex_unlock(&note_mutex);

            if (dirty) {
                lv_label_set_text(g_note_label, tmp);
            }
        }

        lv_task_handler();
        k_sleep(K_MSEC(10));
    }
}

K_THREAD_DEFINE(display_ui_thread, DISPLAY_THREAD_STACK_SIZE,
                display_thread_fn, NULL, NULL, NULL,
                DISPLAY_THREAD_PRIORITY, 0, 0);
