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
#include "reward_img.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(display_ui, CONFIG_APP_LOG_LEVEL);

/* =========================================================================
 * Note buffer – written by the pitch thread, drained by the display thread
 * ========================================================================= */

static K_MUTEX_DEFINE(note_mutex);
static char note_buf[8];
static int  note_pos;   /* 0=Ab(bottom) … 11=G(top); -1=unknown */
static bool note_dirty;

/* =========================================================================
 * Game state (display thread only)
 * ========================================================================= */

#define DOT_SIZE         30
#define DOT_RADIUS       95
#define NOTE_COUNT       12
#define HOLD_DURATION_MS 1500
#define STALE_THRESHOLD   30   /* 30 × 10 ms = 0.3 s without new note = silence */

static const char * const NOTE_NAMES[NOTE_COUNT] = {
    "Ab", "A", "Bb", "B", "C", "C#", "D", "Eb", "E", "F", "F#", "G"
};

static int     game_target_pos    = 4;    /* start: C */
static int64_t game_hold_start_ms = -1;  /* -1 = not currently holding */
static int64_t reward_show_until_ms = 0; /* wall-clock deadline to hide reward image */

/* =========================================================================
 * LVGL widget handles (owned by the display thread)
 * ========================================================================= */

static lv_obj_t *g_screen_hello;
static lv_obj_t *g_screen_pitch;
static int        g_current_screen; /* 0 = hello world, 1 = pitch detector */

/* Screen 0 widgets */
static lv_obj_t *g_dot;
static lv_obj_t *g_dot_name;

/* Game overlay widgets (screen 0) */
static lv_obj_t *g_target_dot;
static lv_obj_t *g_target_label;
static lv_obj_t *g_hold_arc;
static lv_obj_t *g_reward_img;

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
 * Game helper – pick a new target note and update the overlay widgets.
 * Must be called from the display thread.
 * ========================================================================= */

static void game_set_target(int pos)
{
    game_target_pos    = pos;
    game_hold_start_ms = -1;

    float a = (90.0f + pos * 30.0f) * (3.14159265f / 180.0f);
    int x = 120 + (int)(DOT_RADIUS * cosf(a)) - DOT_SIZE / 2;
    int y = 120 + (int)(DOT_RADIUS * sinf(a)) - DOT_SIZE / 2;
    lv_obj_set_pos(g_target_dot, x, y);
    lv_label_set_text(g_target_label, NOTE_NAMES[pos]);
    lv_arc_set_value(g_hold_arc, 0);
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

    /* ---- Game overlay: amber target dot created first so g_dot renders on top ---- */
    g_target_dot = lv_obj_create(g_screen_hello);
    lv_obj_set_size(g_target_dot, DOT_SIZE, DOT_SIZE);
    lv_obj_set_style_radius(g_target_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_target_dot, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_obj_set_style_bg_opa(g_target_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_target_dot, 0, 0);
    lv_obj_align(g_target_dot, LV_ALIGN_TOP_LEFT, 0, 0);   /* positioned by game_set_target */

    /* Detected dot and label drawn on top of the target dot. */
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
    lv_obj_align(g_dot_name, LV_ALIGN_CENTER, 0, 65);
    lv_obj_add_flag(g_dot_name, LV_OBJ_FLAG_HIDDEN);

    g_target_label = lv_label_create(g_screen_hello);
    lv_obj_set_style_text_font(g_target_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(g_target_label, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_label_set_text(g_target_label, "");
    lv_obj_align(g_target_label, LV_ALIGN_CENTER, 0, -65);

    g_hold_arc = lv_arc_create(g_screen_hello);
    lv_obj_set_size(g_hold_arc, 70, 70);
    lv_obj_center(g_hold_arc);
    lv_arc_set_range(g_hold_arc, 0, 100);
    lv_arc_set_value(g_hold_arc, 0);
    lv_arc_set_bg_angles(g_hold_arc, 0, 360);
    lv_obj_set_style_arc_color(g_hold_arc, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(g_hold_arc, lv_color_make(50, 50, 50), LV_PART_MAIN);
    lv_obj_set_style_arc_width(g_hold_arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(g_hold_arc, 6, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(g_hold_arc, LV_OPA_TRANSP, 0);
    lv_obj_remove_style(g_hold_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(g_hold_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(g_hold_arc, LV_OBJ_FLAG_HIDDEN);

    /* Reward image – shown briefly on note completion, created last (top layer). */
    g_reward_img = lv_image_create(g_screen_hello);
    lv_image_set_src(g_reward_img, &reward_img);
    lv_obj_center(g_reward_img);
    lv_obj_add_flag(g_reward_img, LV_OBJ_FLAG_HIDDEN);

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

    game_set_target(game_target_pos);

    bool was_active     = false;
    int  last_known_pos = -1;
    int  stale_ticks    = 0;

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
        bool is_active      = pitch_is_active();
        bool reward_showing = (reward_show_until_ms > 0 &&
                               k_uptime_get() < reward_show_until_ms);

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
                stale_ticks    = 0;
                last_known_pos = pos;

                /* Screen 1: note label (e.g. "A4") */
                lv_label_set_text(g_note_label, tmp);

                /* Screen 0: orbit dot around the edge, show note name below centre. */
                if (pos >= 0) {
                    /* Ab=6-o'clock (90°), clockwise 30° per semitone. */
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
                    lv_obj_align(g_dot_name, LV_ALIGN_CENTER, 0, 65);
                    if (!reward_showing) {
                        lv_obj_clear_flag(g_dot,      LV_OBJ_FLAG_HIDDEN);
                        lv_obj_clear_flag(g_dot_name, LV_OBJ_FLAG_HIDDEN);
                    }
                }
            } else {
                stale_ticks++;
            }
        } else {
            /* Hide the dot when pitch is not running. */
            last_known_pos = -1;
            stale_ticks    = 0;
            lv_obj_add_flag(g_dot,      LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(g_dot_name, LV_OBJ_FLAG_HIDDEN);
        }

        /* ---- Game: match target note for HOLD_THRESHOLD ticks to advance ---- */
        if (g_current_screen == 0) {
            if (reward_showing) {
                /* Keep all game overlay hidden during the reward flash. */
                lv_obj_add_flag(g_hold_arc, LV_OBJ_FLAG_HIDDEN);
            } else {
            bool holding = is_active
                        && stale_ticks < STALE_THRESHOLD
                        && last_known_pos >= 0
                        && last_known_pos == game_target_pos;

            /* Show/hide the arc based on pitch activity. */
            if (is_active) {
                lv_obj_clear_flag(g_hold_arc, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(g_hold_arc, LV_OBJ_FLAG_HIDDEN);
            }

            if (holding) {
                if (game_hold_start_ms < 0) {
                    game_hold_start_ms = k_uptime_get();
                }
                int64_t elapsed = k_uptime_get() - game_hold_start_ms;
                lv_arc_set_value(g_hold_arc,
                    (int)(elapsed * 100 / HOLD_DURATION_MS));
                lv_obj_set_style_bg_color(g_dot, lv_palette_main(LV_PALETTE_GREEN), 0);
                if (elapsed >= HOLD_DURATION_MS) {
                    int new_pos = (game_target_pos + 1 +
                                   (int)(sys_rand32_get() % (NOTE_COUNT - 1))) % NOTE_COUNT;
                    game_set_target(new_pos);
                    reward_show_until_ms = k_uptime_get() + 2000;
                    lv_obj_clear_flag(g_reward_img,   LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_dot,            LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_dot_name,       LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_target_dot,     LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_target_label,   LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_hold_arc,       LV_OBJ_FLAG_HIDDEN);
                }
            } else {
                if (game_hold_start_ms >= 0) {
                    game_hold_start_ms = -1;
                    lv_arc_set_value(g_hold_arc, 0);
                }
                lv_obj_set_style_bg_color(g_dot, lv_color_white(), 0);
            }
            } /* !reward_showing */
        }

        lv_task_handler();
        /* Hide reward image once its display window has elapsed. */
        if (reward_show_until_ms > 0 && k_uptime_get() >= reward_show_until_ms) {
            reward_show_until_ms = 0;
            lv_obj_add_flag(g_reward_img, LV_OBJ_FLAG_HIDDEN);
            /* Restore always-visible game overlay; arc/dots restored by loop logic. */
            lv_obj_clear_flag(g_target_dot,   LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(g_target_label, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(g_dot, lv_color_white(), 0);
        }
        k_sleep(K_MSEC(10));
    }
}

K_THREAD_DEFINE(display_ui_thread, DISPLAY_THREAD_STACK_SIZE,
                display_thread_fn, NULL, NULL, NULL,
                DISPLAY_THREAD_PRIORITY, 0, 0);
