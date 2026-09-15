/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Round display UI – LVGL-based UI for the Seeed XIAO Round Display (240×240).
 *
 * Screens (cycled by significant motion detected via the IMU):
 *   0 – Rocket launch: any sustained sound lifts the rocket; keep making noise
 *       to fly all the way up to the moon.
 *   1 – Note match: white dot orbits with the sung note (Ab=bottom … G=top);
 *       hold the amber target note to score.
 *   2 – Asteroid dodge: the UFO orbits with the sung note (same mapping as the
 *       dot on screen 1) while asteroids fly outwards from the centre in random
 *       directions; sing to move the UFO out of their way.
 *
 * Threading model:
 *   A dedicated thread owns all LVGL calls (lv_task_handler + widget updates).
 *   The pitch note callback runs in the pitch thread; it writes the detected
 *   note into a mutex-protected buffer that the display thread drains every
 *   iteration.  Pitch detection runs continuously – every screen needs it.
 */

#include "display_ui.h"
#include "imu.h"
#include "pitch.h"
#include "reward_img.h"
#include "ufo_img.h"
#include "asteroid_img.h"
#include "explosion_img.h"
#include "confetti_img.h"

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
static float note_cents; /* deviation from the note, [-50, +50] */
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
 * Asteroid dodge game state (screen 2, display thread only)
 * ========================================================================= */

#define SCREEN_COUNT      3
#define UFO_SIZE          34
#define AST_SIZE          22
#define UFO_RADIUS        95   /* orbit radius of the UFO centre */
#define ARENA_RADIUS     140   /* asteroid is recycled past this distance */
#define MAX_ASTEROIDS     10
#define COLLISION_DIST    24   /* centre-to-centre pixels counting as a hit */
#define CRASH_SHOW_MS   1800
#define EXPLOSION_SIZE    90

/* Speeds are in pixels per 10 ms tick; the game starts slow and ramps up. */
#define AST_SPEED_START    0.35f
#define AST_SPEED_MAX      1.10f
#define AST_SPEED_STEP     0.02f
#define SPAWN_INTERVAL_START_MS 2200
#define SPAWN_INTERVAL_MIN_MS    700
#define SPAWN_INTERVAL_STEP_MS    60

struct asteroid {
    lv_obj_t *img;
    float     x, y;    /* centre position in screen coordinates */
    float     vx, vy;  /* velocity in px per tick */
    bool      active;
};

static struct asteroid g_asteroids[MAX_ASTEROIDS];
static int64_t g_next_spawn_ms;
static int     g_spawn_interval_ms = SPAWN_INTERVAL_START_MS;
static float   g_ast_speed         = AST_SPEED_START;
static int     g_dodged;
static int64_t g_crash_until_ms;   /* 0 = playing, else deadline of crash banner */
static int     g_hit_x, g_hit_y;   /* impact point, used to place the explosion */

/* =========================================================================
 * Rocket launch game state (screen 0, display thread only)
 * ========================================================================= */

#define STAR_COUNT        12
#define ROCKET_Y_BOTTOM  195.0f   /* rocket centre when silent */
#define ROCKET_Y_TOP      60.0f   /* rocket centre at full thrust */
#define MOON_SIZE         70
#define MOON_Y_START     (-90)    /* moon centre before lift-off */
#define MOON_Y_END        60      /* moon centre when the goal is reached */
#define ROCKET_GOAL      450.0f   /* accumulated thrust needed to reach the moon */
#define WIN_SHOW_MS     2500

/* RMS window mapped to 0 … 100 % thrust; below the floor the rocket falls back. */
#define LEVEL_FLOOR      0.010f
#define LEVEL_CEIL       0.120f

static volatile float g_mic_level;   /* latest RMS, written by the pitch thread */
static float   g_rocket_y = ROCKET_Y_BOTTOM;
static float   g_altitude;
static float   g_star_y[STAR_COUNT];
static int64_t g_win_until_ms;

/* =========================================================================
 * LVGL widget handles (owned by the display thread)
 * ========================================================================= */

static lv_obj_t *g_screen_hello;
static int        g_current_screen; /* 0 = rocket launch, 1 = note match, 2 = asteroid dodge */

/* Screen 1 widgets */
static lv_obj_t *g_dot;
static lv_obj_t *g_dot_name;

/* Game overlay widgets (screen 1) */
static lv_obj_t *g_target_dot;
static lv_obj_t *g_target_label;
static lv_obj_t *g_hold_arc;
static lv_obj_t *g_reward_img;

/* Screen 2 widgets */
static lv_obj_t *g_screen_space;
static lv_obj_t *g_ufo;
static lv_obj_t *g_score_label;
static lv_obj_t *g_explosion;

/* Screen 0 widgets */
static lv_obj_t *g_screen_rocket;
static lv_obj_t *g_rocket;
static lv_obj_t *g_flame;
static lv_obj_t *g_moon;
static lv_obj_t *g_stars[STAR_COUNT];
static lv_obj_t *g_confetti;

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

static void on_note_detected(const char *note, int octave, float cents)
{
    char tmp[8];

    snprintf(tmp, sizeof(tmp), "%s%d", note, octave);

    k_mutex_lock(&note_mutex, K_FOREVER);
    strncpy(note_buf, tmp, sizeof(note_buf) - 1);
    note_buf[sizeof(note_buf) - 1] = '\0';
    note_pos   = note_to_pos(note);
    note_cents = cents;
    note_dirty = true;
    k_mutex_unlock(&note_mutex);
}

/* =========================================================================
 * Pitch level callback (pitch thread context)
 * ========================================================================= */

static void on_level_detected(float rms)
{
    g_mic_level = rms;
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
 * Asteroid dodge helpers (display thread only)
 * ========================================================================= */

static void space_reset(void)
{
    for (int i = 0; i < MAX_ASTEROIDS; i++) {
        g_asteroids[i].active = false;
        lv_obj_add_flag(g_asteroids[i].img, LV_OBJ_FLAG_HIDDEN);
    }
    g_spawn_interval_ms = SPAWN_INTERVAL_START_MS;
    g_ast_speed         = AST_SPEED_START;
    g_dodged            = 0;
    g_crash_until_ms    = 0;
    g_next_spawn_ms     = k_uptime_get() + SPAWN_INTERVAL_START_MS;
    lv_label_set_text(g_score_label, "0");
    lv_obj_add_flag(g_explosion, LV_OBJ_FLAG_HIDDEN);
}

static void space_spawn(void)
{
    for (int i = 0; i < MAX_ASTEROIDS; i++) {
        struct asteroid *a = &g_asteroids[i];

        if (a->active) {
            continue;
        }
        float ang = (float)(sys_rand32_get() % 3600) * 0.1f * (3.14159265f / 180.0f);
        a->x      = 120.0f;
        a->y      = 120.0f;
        a->vx     = cosf(ang) * g_ast_speed;
        a->vy     = sinf(ang) * g_ast_speed;
        a->active = true;
        lv_obj_set_pos(a->img, (int)a->x - AST_SIZE / 2, (int)a->y - AST_SIZE / 2);
        lv_obj_clear_flag(a->img, LV_OBJ_FLAG_HIDDEN);
        return;
    }
}

/* Advance every asteroid one tick; returns true when one hits the UFO. */static bool space_step(int ufo_x, int ufo_y, bool ufo_visible)
{
    bool hit = false;

    for (int i = 0; i < MAX_ASTEROIDS; i++) {
        struct asteroid *a = &g_asteroids[i];

        if (!a->active) {
            continue;
        }
        a->x += a->vx;
        a->y += a->vy;
        lv_obj_set_pos(a->img, (int)a->x - AST_SIZE / 2, (int)a->y - AST_SIZE / 2);

        float dx = a->x - 120.0f;
        float dy = a->y - 120.0f;
        if (dx * dx + dy * dy > (float)(ARENA_RADIUS * ARENA_RADIUS)) {
            a->active = false;
            lv_obj_add_flag(a->img, LV_OBJ_FLAG_HIDDEN);
            g_dodged++;
            lv_label_set_text_fmt(g_score_label, "%d", g_dodged);
            continue;
        }

        if (ufo_visible) {
            float ox = a->x - (float)ufo_x;
            float oy = a->y - (float)ufo_y;
            if (ox * ox + oy * oy < (float)(COLLISION_DIST * COLLISION_DIST)) {
                hit     = true;
                g_hit_x = (int)a->x;
                g_hit_y = (int)a->y;
            }
        }
    }
    return hit;
}

/* =========================================================================
 * Rocket launch helpers (display thread only)
 * ========================================================================= */

static void rocket_reset(void)
{
    g_rocket_y     = ROCKET_Y_BOTTOM;
    g_altitude     = 0.0f;
    g_win_until_ms = 0;

    for (int i = 0; i < STAR_COUNT; i++) {
        g_star_y[i] = (float)(sys_rand32_get() % 240);
        lv_obj_set_pos(g_stars[i], 10 + (int)(sys_rand32_get() % 220),
                       (int)g_star_y[i]);
        lv_obj_clear_flag(g_stars[i], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_pos(g_moon, 120 - MOON_SIZE / 2, MOON_Y_START - MOON_SIZE / 2);
    lv_obj_clear_flag(g_moon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_flame, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_confetti, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_rocket, LV_OBJ_FLAG_HIDDEN);
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

    /* ---- Screen 1: Note match (dot moves Ab=bottom … G=top) ---- */
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

    /* ---- Screen 2: Asteroid dodge ---- */
    g_screen_space = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_screen_space, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_screen_space, LV_OPA_COVER, 0);
    lv_obj_clear_flag(g_screen_space, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(g_screen_space, 0, 0);

    g_score_label = lv_label_create(g_screen_space);
    lv_obj_set_style_text_color(g_score_label, lv_color_make(120, 120, 120), 0);
    lv_label_set_text(g_score_label, "0");
    lv_obj_align(g_score_label, LV_ALIGN_TOP_MID, 0, 30);

    for (int i = 0; i < MAX_ASTEROIDS; i++) {
        g_asteroids[i].img = lv_image_create(g_screen_space);
        lv_image_set_src(g_asteroids[i].img, &asteroid_img);
        lv_obj_add_flag(g_asteroids[i].img, LV_OBJ_FLAG_HIDDEN);
    }

    /* UFO created after the asteroids so it renders on top. */
    g_ufo = lv_image_create(g_screen_space);
    lv_image_set_src(g_ufo, &ufo_img);
    lv_obj_align(g_ufo, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(g_ufo, LV_OBJ_FLAG_HIDDEN);

    g_explosion = lv_image_create(g_screen_space);
    lv_image_set_src(g_explosion, &explosion_img);
    lv_obj_align(g_explosion, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(g_explosion, LV_OBJ_FLAG_HIDDEN);

    /* ---- Screen 0: Rocket launch ---- */
    g_screen_rocket = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_screen_rocket, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_screen_rocket, LV_OPA_COVER, 0);
    lv_obj_clear_flag(g_screen_rocket, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(g_screen_rocket, 0, 0);

    for (int i = 0; i < STAR_COUNT; i++) {
        g_stars[i] = lv_obj_create(g_screen_rocket);
        lv_obj_set_size(g_stars[i], 4, 4);
        lv_obj_set_style_radius(g_stars[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(g_stars[i], lv_color_white(), 0);
        lv_obj_set_style_bg_opa(g_stars[i], LV_OPA_70, 0);
        lv_obj_set_style_border_width(g_stars[i], 0, 0);
        lv_obj_align(g_stars[i], LV_ALIGN_TOP_LEFT, 0, 0);
    }

    g_moon = lv_obj_create(g_screen_rocket);
    lv_obj_set_size(g_moon, MOON_SIZE, MOON_SIZE);
    lv_obj_set_style_radius(g_moon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_moon, lv_color_make(230, 230, 200), 0);
    lv_obj_set_style_bg_opa(g_moon, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_moon, 0, 0);
    lv_obj_align(g_moon, LV_ALIGN_TOP_LEFT, 0, 0);

    g_flame = lv_obj_create(g_screen_rocket);
    lv_obj_set_style_radius(g_flame, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_flame, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_bg_opa(g_flame, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_flame, 0, 0);
    lv_obj_set_size(g_flame, 14, 10);
    lv_obj_align(g_flame, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(g_flame, LV_OBJ_FLAG_HIDDEN);

    /* No rocket artwork yet – the UFO sprite stands in as the spacecraft. */
    g_rocket = lv_image_create(g_screen_rocket);
    lv_image_set_src(g_rocket, &ufo_img);
    lv_obj_align(g_rocket, LV_ALIGN_TOP_LEFT, 0, 0);

    g_confetti = lv_image_create(g_screen_rocket);
    lv_image_set_src(g_confetti, &confetti_img);
    lv_obj_center(g_confetti);
    lv_obj_add_flag(g_confetti, LV_OBJ_FLAG_HIDDEN);

    /* Start on the rocket launch screen */
    lv_scr_load(g_screen_rocket);
    g_current_screen = 0;

    pitch_set_note_cb(on_note_detected);
    pitch_set_level_cb(on_level_detected);
    imu_set_motion_cb(on_motion_detected);
    pitch_start();

    game_set_target(game_target_pos);
    space_reset();
    rocket_reset();

    int  last_known_pos = -1;
    float last_known_cents = 0.0f;
    int  stale_ticks    = 0;

    while (true) {
        /* ---- Handle motion-triggered screen change ---- */
        if (screen_change_requested) {
            screen_change_requested = false;
            g_current_screen = (g_current_screen + 1) % SCREEN_COUNT;

            lv_obj_t *next = g_screen_hello;
            if (g_current_screen == 0) {
                next = g_screen_rocket;
                rocket_reset();
            } else if (g_current_screen == 2) {
                next = g_screen_space;
                space_reset();
            }
            lv_scr_load(next);

            if (!pitch_is_active()) {
                pitch_start();
            }
        }

        bool is_active      = pitch_is_active();
        bool reward_showing = (reward_show_until_ms > 0 &&
                               k_uptime_get() < reward_show_until_ms);

        /* ---- Push latest detected note to the note-match widgets ---- */
        if (is_active) {
            char tmp[8] = {0};
            int  pos    = -1;
            float cents = 0.0f;
            bool dirty  = false;

            k_mutex_lock(&note_mutex, K_FOREVER);
            if (note_dirty) {
                memcpy(tmp, note_buf, sizeof(tmp));
                pos        = note_pos;
                cents      = note_cents;
                note_dirty = false;
                dirty      = true;
            }
            k_mutex_unlock(&note_mutex);

            if (dirty) {
                stale_ticks      = 0;
                last_known_pos   = pos;
                last_known_cents = cents;

                /* Note match: orbit dot around the edge, note name below centre. */
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
        if (g_current_screen == 1) {
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

        /* ---- Asteroid dodge: UFO follows the sung note, asteroids fly out ---- */
        if (g_current_screen == 2) {
            bool ufo_visible = is_active
                            && stale_ticks < STALE_THRESHOLD
                            && last_known_pos >= 0;
            int  ufo_x = 120, ufo_y = 120;

            if (ufo_visible) {
                float a = (90.0f + (last_known_pos + last_known_cents / 100.0f) * 30.0f)
                          * (3.14159265f / 180.0f);
                ufo_x = 120 + (int)(UFO_RADIUS * cosf(a));
                ufo_y = 120 + (int)(UFO_RADIUS * sinf(a));
                lv_obj_set_pos(g_ufo, ufo_x - UFO_SIZE / 2, ufo_y - UFO_SIZE / 2);
                lv_obj_clear_flag(g_ufo, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(g_ufo, LV_OBJ_FLAG_HIDDEN);
            }

            if (g_crash_until_ms > 0) {
                lv_obj_add_flag(g_ufo, LV_OBJ_FLAG_HIDDEN);
                if (k_uptime_get() >= g_crash_until_ms) {
                    space_reset();
                }
            } else {
                if (k_uptime_get() >= g_next_spawn_ms) {
                    space_spawn();
                    g_next_spawn_ms = k_uptime_get() + g_spawn_interval_ms;
                    if (g_spawn_interval_ms > SPAWN_INTERVAL_MIN_MS) {
                        g_spawn_interval_ms -= SPAWN_INTERVAL_STEP_MS;
                    }
                    if (g_ast_speed < AST_SPEED_MAX) {
                        g_ast_speed += AST_SPEED_STEP;
                    }
                }

                if (space_step(ufo_x, ufo_y, ufo_visible)) {
                    for (int i = 0; i < MAX_ASTEROIDS; i++) {
                        g_asteroids[i].active = false;
                        lv_obj_add_flag(g_asteroids[i].img, LV_OBJ_FLAG_HIDDEN);
                    }
                    lv_obj_add_flag(g_ufo, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_set_pos(g_explosion, g_hit_x - EXPLOSION_SIZE / 2,
                                                g_hit_y - EXPLOSION_SIZE / 2);
                    lv_obj_clear_flag(g_explosion, LV_OBJ_FLAG_HIDDEN);
                    g_crash_until_ms = k_uptime_get() + CRASH_SHOW_MS;
                }
            }
        }

        /* ---- Rocket launch: loudness is thrust, keep singing to reach the moon ---- */
        if (g_current_screen == 0) {
            float thrust = is_active
                         ? (g_mic_level - LEVEL_FLOOR) / (LEVEL_CEIL - LEVEL_FLOOR)
                         : 0.0f;

            if (thrust < 0.0f) { thrust = 0.0f; }
            if (thrust > 1.0f) { thrust = 1.0f; }

            if (g_win_until_ms > 0) {
                if (k_uptime_get() >= g_win_until_ms) {
                    rocket_reset();
                }
            } else {
                float target_y = ROCKET_Y_BOTTOM -
                                 thrust * (ROCKET_Y_BOTTOM - ROCKET_Y_TOP);
                g_rocket_y += (target_y - g_rocket_y) * 0.12f;
                lv_obj_set_pos(g_rocket, 120 - UFO_SIZE / 2,
                               (int)g_rocket_y - UFO_SIZE / 2);

                int flame_h = (int)(thrust * 28.0f);
                if (flame_h > 4) {
                    lv_obj_set_size(g_flame, 14, flame_h);
                    lv_obj_set_pos(g_flame, 120 - 7,
                                   (int)g_rocket_y + UFO_SIZE / 2 - 4);
                    lv_obj_clear_flag(g_flame, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_add_flag(g_flame, LV_OBJ_FLAG_HIDDEN);
                }

                /* Stars stream downwards faster the harder the rocket pushes. */
                float star_speed = 0.3f + thrust * 3.5f;
                for (int i = 0; i < STAR_COUNT; i++) {
                    g_star_y[i] += star_speed;
                    if (g_star_y[i] > 244.0f) {
                        g_star_y[i] = -4.0f;
                        lv_obj_set_x(g_stars[i], 10 + (int)(sys_rand32_get() % 220));
                    }
                    lv_obj_set_y(g_stars[i], (int)g_star_y[i]);
                }

                g_altitude += thrust;
                float progress = g_altitude / ROCKET_GOAL;
                if (progress > 1.0f) { progress = 1.0f; }
                lv_obj_set_y(g_moon,
                    MOON_Y_START + (int)(progress * (MOON_Y_END - MOON_Y_START))
                    - MOON_SIZE / 2);

                if (g_altitude >= ROCKET_GOAL) {
                    lv_obj_add_flag(g_rocket, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_flame,  LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(g_moon,   LV_OBJ_FLAG_HIDDEN);
                    for (int i = 0; i < STAR_COUNT; i++) {
                        lv_obj_add_flag(g_stars[i], LV_OBJ_FLAG_HIDDEN);
                    }
                    lv_obj_clear_flag(g_confetti, LV_OBJ_FLAG_HIDDEN);
                    g_win_until_ms = k_uptime_get() + WIN_SHOW_MS;
                }
            }
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
