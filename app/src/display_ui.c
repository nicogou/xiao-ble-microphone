/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Round display UI – LVGL-based UI for the Seeed XIAO Round Display (240×240).
 *
 * Screens (cycled by significant motion detected via the IMU):
 *   0 – Note visualiser: white dot slides vertically (Ab=bottom … G=top) while
 *       pitch detection is active; note name shown below the dot.
 *   1 – Pitch detector: "▶ Start / ■ Stop" button + detected note label.
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
#include "imu.h"
#include "pitch.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
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
static int  note_pos;   /* 0=Ab(bottom) … 11=G(top); -1=unknown */
static bool note_dirty;

/* =========================================================================
 * LVGL widget handles (owned by the display thread)
 * ========================================================================= */

static lv_obj_t *g_screen_hello;
static lv_obj_t *g_screen_pitch;
static int        g_current_screen; /* 0 = hello world, 1 = pitch detector */

/* Screen 0 widgets */
static lv_obj_t *g_dot;
static lv_obj_t *g_dot_name;

/* Screen 1 widgets */
static lv_obj_t *g_btn;
static lv_obj_t *g_btn_label;
static lv_obj_t *g_note_label;

/* =========================================================================
 * Toggle flag – set in the event callback, consumed in the main loop
 * ========================================================================= */

static volatile bool toggle_requested;

/* =========================================================================
 * Screen-change flag – set by the IMU motion callback, consumed in the loop
 * ========================================================================= */

static volatile bool screen_change_requested;

/* =========================================================================
 * Note-name → vertical position (0 = Ab/bottom … 11 = G/top)
 * ========================================================================= */

static int note_to_pos(const char *note)
{
    /* Chromatic order starting at Ab so positions match musical pitch visually. */
    static const struct { const char *n; int p; } MAP[] = {
        {"Ab", 0}, {"A", 1}, {"Bb", 2}, {"B", 3},
        {"C",  4}, {"C#", 5}, {"D", 6}, {"Eb", 7},
        {"E",  8}, {"F", 9}, {"F#", 10}, {"G", 11},
    };
    for (int i = 0; i < 12; i++) {
        if (strcmp(note, MAP[i].n) == 0) {
            return MAP[i].p;
        }
    }
    return -1;
}

/* =========================================================================
 * IMU motion callback (system work-queue context)
 * ========================================================================= */

static void on_motion_detected(void)
{
    screen_change_requested = true;
}

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
    note_pos   = note_to_pos(note);
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

    /* ---- Screen 0: Note visualiser (dot moves Ab=bottom … G=top) ---- */
    g_screen_hello = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_screen_hello, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_screen_hello, LV_OPA_COVER, 0);
    lv_obj_clear_flag(g_screen_hello, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(g_screen_hello, 0, 0);

    /* Dot orbits the screen edge; note name stays at the centre. */
    g_dot = lv_obj_create(g_screen_hello);
    lv_obj_set_size(g_dot, 30, 30);
    lv_obj_set_style_radius(g_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_dot, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(g_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_dot, 0, 0);
    lv_obj_align(g_dot, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(g_dot, LV_OBJ_FLAG_HIDDEN);

    g_dot_name = lv_label_create(g_screen_hello);
    lv_obj_set_style_text_font(g_dot_name, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(g_dot_name, lv_color_white(), 0);
    lv_label_set_text(g_dot_name, "");
    lv_obj_center(g_dot_name);
    lv_obj_add_flag(g_dot_name, LV_OBJ_FLAG_HIDDEN);

    /* ---- Screen 1: Pitch detector ---- */
    g_screen_pitch = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_screen_pitch, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_screen_pitch, LV_OPA_COVER, 0);

    /* Toggle button – upper centre */
    g_btn = lv_btn_create(g_screen_pitch);
    lv_obj_set_size(g_btn, 150, 60);
    lv_obj_align(g_btn, LV_ALIGN_CENTER, 0, -30);
    lv_obj_add_event_cb(g_btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    g_btn_label = lv_label_create(g_btn);
    lv_label_set_text(g_btn_label, LV_SYMBOL_PLAY " Start");
    lv_obj_center(g_btn_label);

    /* Note label – lower centre, hidden until pitch is running */
    g_note_label = lv_label_create(g_screen_pitch);
    lv_obj_set_style_text_font(g_note_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(g_note_label, lv_color_white(), 0);
    lv_label_set_text(g_note_label, "---");
    lv_obj_align(g_note_label, LV_ALIGN_CENTER, 0, 50);
    lv_obj_add_flag(g_note_label, LV_OBJ_FLAG_HIDDEN);

    /* Start on the hello world screen */
    lv_scr_load(g_screen_hello);
    g_current_screen = 0;

    pitch_set_note_cb(on_note_detected);
    imu_set_motion_cb(on_motion_detected);
    pitch_start();

    bool was_active = false;

    while (true) {
        /* ---- Handle motion-triggered screen change ---- */
        if (screen_change_requested) {
            screen_change_requested = false;
            int prev_screen  = g_current_screen;
            g_current_screen = (g_current_screen + 1) % 2;
            lv_scr_load(g_current_screen == 0 ? g_screen_hello : g_screen_pitch);
            if (prev_screen == 0) {
                pitch_stop();
            } else {
                pitch_start();
            }
        }

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

        /* ---- Push latest detected note to widgets on both screens ---- */
        if (is_active) {
            char tmp[8] = {0};
            int  pos    = -1;
            bool dirty  = false;

            k_mutex_lock(&note_mutex, K_FOREVER);
            if (note_dirty) {
                memcpy(tmp, note_buf, sizeof(tmp));
                pos        = note_pos;
                note_dirty = false;
                dirty      = true;
            }
            k_mutex_unlock(&note_mutex);

            if (dirty) {
                /* Screen 1: note label (e.g. "A4") */
                lv_label_set_text(g_note_label, tmp);

                /* Screen 0: orbit dot around the edge, show note name at centre. */
                if (pos >= 0) {
                    /* Ab=6-o'clock (90°), clockwise 30° per semitone. */
#define DOT_SIZE   30
#define DOT_RADIUS 95
                    float a = (90.0f + pos * 30.0f) * (3.14159265f / 180.0f);
                    int dot_x = 120 + (int)(DOT_RADIUS * cosf(a)) - DOT_SIZE / 2;
                    int dot_y = 120 + (int)(DOT_RADIUS * sinf(a)) - DOT_SIZE / 2;
                    lv_obj_set_pos(g_dot, dot_x, dot_y);
                    /* Strip octave digit from tmp (e.g. "C#4" → "C#"). */
                    char name_only[4] = {0};
                    int  j = 0;
                    for (int i = 0; tmp[i] && !('0' <= tmp[i] && tmp[i] <= '9') && j < 3; i++) {
                        name_only[j++] = tmp[i];
                    }
                    lv_label_set_text(g_dot_name, name_only);
                    lv_obj_center(g_dot_name);
                    lv_obj_clear_flag(g_dot,      LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(g_dot_name, LV_OBJ_FLAG_HIDDEN);
                }
            }
        } else {
            /* Hide the dot when pitch is not running. */
            lv_obj_add_flag(g_dot,      LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(g_dot_name, LV_OBJ_FLAG_HIDDEN);
        }

        lv_task_handler();
        k_sleep(K_MSEC(10));
    }
}

K_THREAD_DEFINE(display_ui_thread, DISPLAY_THREAD_STACK_SIZE,
                display_thread_fn, NULL, NULL, NULL,
                DISPLAY_THREAD_PRIORITY, 0, 0);
