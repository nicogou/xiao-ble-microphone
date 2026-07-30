/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Real-time monophonic singing pitch detector.
 *
 * Algorithm: McLeod Pitch Method (MPM) using the Normalized Square
 * Difference Function (NSDF) with parabolic interpolation for sub-sample
 * accuracy.  See:  McLeod & Wyvill, "A Smarter Way to Find Pitch", ICMC 2005.
 *
 * The detector runs in a dedicated thread that:
 *   1. Reads PDM blocks from the microphone
 *   2. Accumulates samples into a 1024-sample (64 ms) analysis frame with a
 *      256-sample (16 ms) hop → 75 % overlap, ≈ 62 detections / second
 *   3. On each hop, computes NSDF for lags covering 80 – 1200 Hz, picks the
 *      first key maximum, and logs the detected frequency + musical note name
 *
 * Shell commands:
 *   pitch start   – start real-time pitch detection
 *   pitch stop    – stop detection
 *   pitch status  – show current state
 */

#include "pitch.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "mic.h"

LOG_MODULE_REGISTER(pitch, CONFIG_APP_LOG_LEVEL);

/* Peer-mode query functions (defined in main.c / kws.cpp) */
extern bool rec_is_active(void);
extern bool kws_is_active(void);

/* =========================================================================
 * Analysis parameters
 * ========================================================================= */

/* Must match the shared mic layer sample rate. */
#define PITCH_SAMPLE_RATE  MIC_SAMPLE_RATE

/* Analysis window: 1024 samples = 64 ms */
#define PITCH_FRAME_SIZE   1024U

/* Hop size: 256 samples = 16 ms (75 % overlap) */
#define PITCH_HOP_SIZE     256U

/* Number of hops needed to fill one frame for the first time */
#define PITCH_HOPS_TO_PRIME (PITCH_FRAME_SIZE / PITCH_HOP_SIZE) /* 4 */

/*
 * Pitch range for human singing voice:
 *   80 Hz  – low bass (MIDI B1)
 *   1200 Hz – high soprano harmonics / whistle register
 */
#define PITCH_MIN_FREQ     80U
#define PITCH_MAX_FREQ     1200U

#define PITCH_MIN_LAG      (PITCH_SAMPLE_RATE / PITCH_MAX_FREQ) /* 13 */
#define PITCH_MAX_LAG      (PITCH_SAMPLE_RATE / PITCH_MIN_FREQ) /* 200 */
#define PITCH_NLAGS        (PITCH_MAX_LAG - PITCH_MIN_LAG + 1)  /* 188 */

/*
 * MPM thresholds.
 *
 * NSDF_MIN_VOICED:  if the global NSDF peak is below this, the frame is
 *                   considered unvoiced (too noisy or no clear periodicity).
 * KEY_FRAC:         the first local NSDF max >= global_max * KEY_FRAC is
 *                   taken as the pitch period (standard MPM value: 0.86).
 * RMS_SILENCE:      frames with RMS below this are skipped entirely.
 */
#define NSDF_MIN_VOICED    0.25f
#define NSDF_KEY_FRAC      0.86f
#define PITCH_RMS_SILENCE  0.005f  /* ≈ −46 dBFS */

/* =========================================================================
 * Static buffers (not on the thread stack)
 * ========================================================================= */

static float pitch_frame[PITCH_FRAME_SIZE]; /* sliding analysis window */
static float pitch_nsdf[PITCH_NLAGS];       /* NSDF values per lag     */
static float pitch_hop[PITCH_HOP_SIZE];     /* current hop accumulator */
static size_t pitch_hop_fill;               /* samples in current hop  */
static int    pitch_hops_seen;              /* hops completed so far   */

/* =========================================================================
 * State
 * ========================================================================= */

static K_MUTEX_DEFINE(pitch_mutex);
static bool      pitch_active;
static atomic_t  pitch_stop_req = ATOMIC_INIT(0);

#define PITCH_STACK_SIZE 2048
static K_THREAD_STACK_DEFINE(pitch_stack, PITCH_STACK_SIZE);
static struct k_thread pitch_thread;

/* =========================================================================
 * MPM pitch detection
 * ========================================================================= */

/*
 * Musical note names (semitones 0–11 starting at C).
 * A4 = 440 Hz = MIDI note 69.
 */
static const char *const NOTE_NAMES[12] = {
    "C", "C#", "D", "Eb", "E", "F", "F#", "G", "Ab", "A", "Bb", "B"
};

/*
 * detect_pitch() – McLeod Pitch Method on a float frame.
 *
 * Returns the detected fundamental frequency in Hz, or 0.0f when the
 * frame is silent / unvoiced.
 */
static float detect_pitch(const float *frame, int N)
{
    /* --- Silence gate: compute RMS energy of the frame --- */
    float energy = 0.0f;

    for (int n = 0; n < N; n++) {
        energy += frame[n] * frame[n];
    }
    if (sqrtf(energy / (float)N) < PITCH_RMS_SILENCE) {
        return 0.0f;
    }

    /* --- Compute NSDF for lags in [PITCH_MIN_LAG, PITCH_MAX_LAG] ---
     *
     * NSDF(τ) = 2 · Σ x[n]·x[n+τ]  /  (Σ x[n]² + Σ x[n+τ]²)
     *           for n = 0 … N-1-τ
     */
    for (int tau = PITCH_MIN_LAG; tau <= PITCH_MAX_LAG; tau++) {
        int          len = N - tau;
        float        acf = 0.0f, e0 = 0.0f, e1 = 0.0f;
        const float *p0  = frame;
        const float *p1  = frame + tau;

        for (int n = 0; n < len; n++) {
            float s0 = p0[n];
            float s1 = p1[n];
            acf += s0 * s1;
            e0  += s0 * s0;
            e1  += s1 * s1;
        }
        float denom = e0 + e1;
        pitch_nsdf[tau - PITCH_MIN_LAG] =
            (denom > 1e-10f) ? 2.0f * acf / denom : 0.0f;
    }

    /* --- Find global NSDF maximum --- */
    float nsdf_max = 0.0f;

    for (int i = 0; i < PITCH_NLAGS; i++) {
        if (pitch_nsdf[i] > nsdf_max) {
            nsdf_max = pitch_nsdf[i];
        }
    }
    if (nsdf_max < NSDF_MIN_VOICED) {
        return 0.0f; /* unvoiced or too noisy */
    }

    float key_thresh = nsdf_max * NSDF_KEY_FRAC;

    /* --- Find the FIRST local maximum above the key threshold (MPM rule) --- */
    int best_idx = -1;

    for (int i = 1; i < PITCH_NLAGS - 1; i++) {
        if (pitch_nsdf[i] >= pitch_nsdf[i - 1] &&
            pitch_nsdf[i] >  pitch_nsdf[i + 1] &&
            pitch_nsdf[i] >= key_thresh) {
            best_idx = i;
            break;
        }
    }
    if (best_idx < 0) {
        return 0.0f;
    }

    /* --- Parabolic interpolation for sub-sample accuracy ---
     *
     * For a concave-down parabola through (k-1, α), (k, β), (k+1, γ):
     *   vertex at k + (α − γ) / (2·(α − 2β + γ))
     */
    float alpha = (best_idx > 0)
                  ? pitch_nsdf[best_idx - 1] : pitch_nsdf[best_idx];
    float beta  = pitch_nsdf[best_idx];
    float gamma = (best_idx < PITCH_NLAGS - 1)
                  ? pitch_nsdf[best_idx + 1] : pitch_nsdf[best_idx];

    float dpar  = alpha - 2.0f * beta + gamma;
    float refine = (fabsf(dpar) > 1e-10f)
                   ? 0.5f * (alpha - gamma) / dpar : 0.0f;

    /* Clamp refinement to ±0.5 to avoid runaway correction */
    if (refine >  0.5f) { refine =  0.5f; }
    if (refine < -0.5f) { refine = -0.5f; }

    float lag = (float)(best_idx + PITCH_MIN_LAG) + refine;
    return (float)PITCH_SAMPLE_RATE / lag;
}

/*
 * log_pitch() – log frequency in Hz and the nearest musical note name.
 *
 * Uses the standard piano tuning: A4 = 440 Hz = MIDI 69.
 * Cent deviation is included when |deviation| > 5 cents.
 */
static void log_pitch(float f0)
{
    /* Map to nearest MIDI note number */
    float midi_f = 12.0f * log2f(f0 / 440.0f) + 69.0f;
    int   midi_n = (int)(midi_f + 0.5f);

    if (midi_n < 12 || midi_n > 120) {
        /* Out of typical vocal MIDI range (C0 – C9) – just print Hz */
        LOG_INF("%.1f Hz", (double)f0);
        return;
    }

    int   octave   = (midi_n / 12) - 1;
    int   semitone = midi_n % 12;
    float cents    = (midi_f - (float)midi_n) * 100.0f;

    if (cents > 5.0f) {
        LOG_INF("%.1f Hz  %s%d  (+%d ct)",
                (double)f0, NOTE_NAMES[semitone], octave, (int)cents);
    } else if (cents < -5.0f) {
        LOG_INF("%.1f Hz  %s%d  (%d ct)",
                (double)f0, NOTE_NAMES[semitone], octave, (int)cents);
    } else {
        LOG_INF("%.1f Hz  %s%d",
                (double)f0, NOTE_NAMES[semitone], octave);
    }
}

/* =========================================================================
 * Pitch detection thread
 * ========================================================================= */

/*
 * PDM block callback: accumulate samples into the hop buffer, update the
 * sliding frame, then run one MPM analysis after freeing the slab block.
 * Releasing the block before the O(N*lags) NSDF loop keeps the driver
 * slab from starving.
 */
static void pitch_block_cb(void *buf, uint32_t size, void *user_data)
{
    ARG_UNUSED(user_data);

    const int16_t *src     = (const int16_t *)buf;
    size_t         samples = size / sizeof(int16_t);

    for (size_t i = 0; i < samples; i++) {
        pitch_hop[pitch_hop_fill++] = (float)src[i] / 32768.0f;

        if (pitch_hop_fill == PITCH_HOP_SIZE) {
            if (pitch_hops_seen >= PITCH_HOPS_TO_PRIME) {
                memmove(pitch_frame,
                        pitch_frame + PITCH_HOP_SIZE,
                        (PITCH_FRAME_SIZE - PITCH_HOP_SIZE) * sizeof(float));
                memcpy(pitch_frame + PITCH_FRAME_SIZE - PITCH_HOP_SIZE,
                       pitch_hop, PITCH_HOP_SIZE * sizeof(float));
            } else {
                memcpy(pitch_frame + pitch_hops_seen * PITCH_HOP_SIZE,
                       pitch_hop, PITCH_HOP_SIZE * sizeof(float));
            }
            pitch_hops_seen++;
            pitch_hop_fill = 0;
        }
    }

    /* Release slab block before the slow NSDF computation. */
    mic_block_free(buf);

    if (pitch_hops_seen >= PITCH_HOPS_TO_PRIME) {
        float f0 = detect_pitch(pitch_frame, PITCH_FRAME_SIZE);

        if (f0 > 0.0f) {
            log_pitch(f0);
        }
    }
}

static void pitch_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (mic_open() < 0) {
        goto done;
    }

    pitch_hop_fill  = 0;
    pitch_hops_seen = 0;
    memset(pitch_frame, 0, sizeof(pitch_frame));

    LOG_INF("started  range=%u-%u Hz  frame=%u ms  hop=%u ms  ~%u det/s",
            PITCH_MIN_FREQ, PITCH_MAX_FREQ,
            (unsigned)(PITCH_FRAME_SIZE * 1000U / PITCH_SAMPLE_RATE),
            (unsigned)(PITCH_HOP_SIZE   * 1000U / PITCH_SAMPLE_RATE),
            (unsigned)(MIC_SAMPLE_RATE / MIC_BLOCK_SAMPLES));

    mic_run(&pitch_stop_req, pitch_block_cb, NULL);
    mic_close();

done:
    k_mutex_lock(&pitch_mutex, K_FOREVER);
    pitch_active = false;
    k_mutex_unlock(&pitch_mutex);
}

/* =========================================================================
 * Public C API
 * ========================================================================= */

bool pitch_is_active(void)
{
    k_mutex_lock(&pitch_mutex, K_FOREVER);
    bool a = pitch_active;
    k_mutex_unlock(&pitch_mutex);
    return a;
}

/* =========================================================================
 * Shell commands
 * ========================================================================= */

static int cmd_pitch_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&pitch_mutex, K_FOREVER);

    if (pitch_active) {
        shell_error(sh, "Pitch detection already running -- use 'pitch stop' first");
        k_mutex_unlock(&pitch_mutex);
        return -EBUSY;
    }
    if (rec_is_active()) {
        shell_error(sh, "WAV recording in progress -- use 'record stop' first");
        k_mutex_unlock(&pitch_mutex);
        return -EBUSY;
    }
    if (kws_is_active()) {
        shell_error(sh, "KWS is running -- use 'kws stop' first");
        k_mutex_unlock(&pitch_mutex);
        return -EBUSY;
    }

    atomic_set(&pitch_stop_req, 0);
    pitch_active = true;
    k_mutex_unlock(&pitch_mutex);

    k_thread_create(&pitch_thread, pitch_stack,
                    K_THREAD_STACK_SIZEOF(pitch_stack),
                    pitch_thread_fn, NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);
    k_thread_name_set(&pitch_thread, "pitch");

    shell_print(sh, "Pitch detection started -- use 'pitch stop' to end");
    return 0;
}

static int cmd_pitch_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&pitch_mutex, K_FOREVER);
    bool active = pitch_active;
    k_mutex_unlock(&pitch_mutex);

    if (!active) {
        shell_error(sh, "Pitch detection is not running");
        return -ENODEV;
    }

    shell_print(sh, "Stopping pitch detection...");
    atomic_set(&pitch_stop_req, 1);

    int ret = k_thread_join(&pitch_thread, K_SECONDS(3));

    if (ret < 0) {
        shell_error(sh, "Pitch thread join timeout: %d", ret);
        return ret;
    }

    shell_print(sh, "Pitch detection stopped");
    return 0;
}

static int cmd_pitch_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (pitch_is_active()) {
        shell_print(sh,
                    "Pitch running  range=%u-%u Hz  "
                    "frame=%u ms  hop=%u ms  (~%u detections/s)",
                    PITCH_MIN_FREQ, PITCH_MAX_FREQ,
                    (unsigned)(PITCH_FRAME_SIZE * 1000U / PITCH_SAMPLE_RATE),
                    (unsigned)(PITCH_HOP_SIZE   * 1000U / PITCH_SAMPLE_RATE),
                    (unsigned)(PITCH_SAMPLE_RATE / PITCH_HOP_SIZE));
    } else {
        shell_print(sh, "Pitch idle");
    }
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pitch,
    SHELL_CMD_ARG(start, NULL,
                  "Start real-time pitch detection",
                  cmd_pitch_start, 1, 0),
    SHELL_CMD_ARG(stop, NULL,
                  "Stop pitch detection",
                  cmd_pitch_stop, 1, 0),
    SHELL_CMD_ARG(status, NULL,
                  "Show pitch detector state",
                  cmd_pitch_status, 1, 0),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(pitch, &sub_pitch, "Real-time monophonic pitch detector", NULL);
