/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Keyword Spotting (KWS) mode using the Edge Impulse SDK.
 *
 * This file is the SINGLE translation unit that includes ei_run_classifier.h
 * (all its extern "C" function bodies – run_classifier_init, run_classifier_continuous,
 * process_impulse_continuous, etc. – are defined there).
 *
 * Shell commands:
 *   kws start   – start continuous keyword detection
 *   kws stop    – stop keyword detection
 *   kws status  – print current state and label list
 *
 * The inference runs in a dedicated thread that reads PDM slices of
 * EI_CLASSIFIER_SLICE_SIZE samples and feeds them to run_classifier_continuous().
 * This implements a sliding-window approach: every slice triggers one DSP + inference
 * step; the SDK keeps the rolling feature matrix internally.
 */

#include "kws.h"

#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

/*
 * Edge Impulse SDK.
 * ei_run_classifier.h includes model-parameters/model_metadata.h and
 * model-parameters/model_variables.h, both found under the model directory
 * that is added to the include path by the model module's CMakeLists.txt.
 */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

LOG_MODULE_REGISTER(kws, CONFIG_APP_LOG_LEVEL);

/*
 * rec_is_active() is implemented in main.c (C linkage).
 * It lets the KWS start command refuse to start when a WAV recording is in
 * progress (both modes share the PDM microphone and mic-power regulator).
 */
extern "C" bool rec_is_active(void);

/* =========================================================================
 * Audio / PDM configuration
 * ========================================================================= */

#define KWS_SAMPLE_RATE      16000U
#define KWS_BITS             16U
#define KWS_CHANNELS         1U

/* 100 ms PDM read blocks (same as in the recording mode). */
#define KWS_PDM_BLOCK_MS     100U
#define KWS_PDM_BLOCK_BYTES  \
    (KWS_SAMPLE_RATE * KWS_PDM_BLOCK_MS / 1000U * KWS_CHANNELS * (KWS_BITS / 8U))

/* Enough slab blocks for hardware double-buffering + a small read queue. */
#define KWS_PDM_SLAB_BLOCKS  4U

K_MEM_SLAB_DEFINE_STATIC(kws_pdm_slab, KWS_PDM_BLOCK_BYTES, KWS_PDM_SLAB_BLOCKS, 4);

/*
 * Inference slice buffer.
 * EI_CLASSIFIER_SLICE_SIZE = EI_CLASSIFIER_RAW_SAMPLE_COUNT / SLICES_PER_MODEL_WINDOW
 *                          = 15488 / 4 = 3872 samples  (≈ 242 ms at 16 kHz).
 * PDM blocks (1600 samples) are accumulated here until a full slice is ready.
 */
#define KWS_SLICE_SAMPLES  EI_CLASSIFIER_SLICE_SIZE

static int16_t kws_slice_buf[KWS_SLICE_SAMPLES];

/* =========================================================================
 * Detection threshold
 * ========================================================================= */

/*
 * Only log a label when its confidence score is >= this value.
 * Initialised from the model metadata default; the user can change it at
 * run-time with "kws threshold <value>".
 */
static float kws_threshold = (float)EI_CLASSIFIER_THRESHOLD;

/* =========================================================================
 * KWS state
 * ========================================================================= */

static K_MUTEX_DEFINE(kws_mutex);
static bool     kws_active;
static atomic_t kws_stop_req = ATOMIC_INIT(0);

#define KWS_STACK_SIZE  8192
static K_THREAD_STACK_DEFINE(kws_stack, KWS_STACK_SIZE);
static struct k_thread kws_thread;

/* =========================================================================
 * EI signal_t callback
 * Normalises int16 PCM samples from kws_slice_buf to float [-1.0, 1.0].
 * ========================================================================= */

static int kws_get_data(size_t offset, size_t length, float *out_ptr)
{
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)kws_slice_buf[offset + i] / 32768.0f;
    }
    return EIDSP_OK;
}

/* =========================================================================
 * KWS inference thread
 * ========================================================================= */

static void kws_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *dmic    = DEVICE_DT_GET(DT_NODELABEL(pdm0));
    const struct device *mic_pwr = DEVICE_DT_GET(DT_NODELABEL(mic_pwr));
    int ret;

    /* Power up the microphone */
    ret = regulator_enable(mic_pwr);
    if (ret < 0) {
        LOG_ERR("kws: regulator_enable: %d", ret);
        goto done;
    }
    k_sleep(K_MSEC(100)); /* wait for mic power to stabilise */

    /* Configure PDM */
    {
        struct pcm_stream_cfg stream = {
            .pcm_rate   = KWS_SAMPLE_RATE,
            .pcm_width  = KWS_BITS,
            .block_size = KWS_PDM_BLOCK_BYTES,
            .mem_slab   = &kws_pdm_slab,
        };
        struct dmic_cfg cfg = {
            .io = {
                .min_pdm_clk_freq = 1000000,
                .max_pdm_clk_freq = 3200000,
                .min_pdm_clk_dc   = 40,
                .max_pdm_clk_dc   = 60,
            },
            .streams = &stream,
            .channel = {
                .req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
                .req_num_chan    = KWS_CHANNELS,
                .req_num_streams = 1,
            },
        };

        ret = dmic_configure(dmic, &cfg);
        if (ret < 0) {
            LOG_ERR("kws: dmic_configure: %d", ret);
            regulator_disable(mic_pwr);
            goto done;
        }
    }

    ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("kws: DMIC_TRIGGER_START: %d", ret);
        regulator_disable(mic_pwr);
        goto done;
    }

    /* Discard first block – PDM filter warm-up */
    {
        void    *buf;
        uint32_t size;

        if (dmic_read(dmic, 0, &buf, &size, KWS_PDM_BLOCK_MS * 2) == 0) {
            k_mem_slab_free(&kws_pdm_slab, buf);
        }
    }

    /* Initialise the EI continuous-inference state */
    run_classifier_init();

    LOG_INF("KWS: running  labels=%d  slice=%d samples  threshold=%.2f",
            EI_CLASSIFIER_LABEL_COUNT, KWS_SLICE_SAMPLES, (double)kws_threshold);

    {
        size_t slice_fill = 0; /* samples accumulated in kws_slice_buf */

        while (!atomic_get(&kws_stop_req)) {
            void    *pdm_buf;
            uint32_t pdm_size;

            ret = dmic_read(dmic, 0, &pdm_buf, &pdm_size,
                            KWS_PDM_BLOCK_MS * 2);
            if (ret < 0) {
                LOG_ERR("kws: dmic_read: %d", ret);
                break;
            }

            const int16_t *src     = (const int16_t *)pdm_buf;
            size_t         samples = pdm_size / sizeof(int16_t);
            size_t         pos     = 0;

            /* Accumulate samples into the slice buffer; run inference
             * whenever a full slice is ready. */
            while (pos < samples) {
                size_t space   = (size_t)KWS_SLICE_SAMPLES - slice_fill;
                size_t to_copy = (samples - pos < space) ? (samples - pos) : space;

                memcpy(&kws_slice_buf[slice_fill], &src[pos],
                       to_copy * sizeof(int16_t));
                slice_fill += to_copy;
                pos        += to_copy;

                if (slice_fill == (size_t)KWS_SLICE_SAMPLES) {
                    /* Run one sliding-window inference step */
                    signal_t signal;
                    signal.total_length = KWS_SLICE_SAMPLES;
                    signal.get_data     = kws_get_data;

                    ei_impulse_result_t result;
                    EI_IMPULSE_ERROR res =
                        run_classifier_continuous(&signal, &result, false);

                    if (res == EI_IMPULSE_OK) {
                        for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                            if (result.classification[i].value >= kws_threshold) {
                                LOG_INF("KWS: '%s' %.0f%%",
                                        result.classification[i].label,
                                        (double)(result.classification[i].value
                                                 * 100.0f));
                            }
                        }
                    } else {
                        LOG_WRN("kws: inference error %d", (int)res);
                    }

                    slice_fill = 0;
                }
            }

            k_mem_slab_free(&kws_pdm_slab, pdm_buf);
        }
    }

    /* Stop PDM and drain any blocks still queued in the driver */
    dmic_trigger(dmic, DMIC_TRIGGER_STOP);
    {
        void    *buf;
        uint32_t size;

        while (dmic_read(dmic, 0, &buf, &size, 0) == 0) {
            k_mem_slab_free(&kws_pdm_slab, buf);
        }
    }

    run_classifier_deinit();
    regulator_disable(mic_pwr);

done:
    k_mutex_lock(&kws_mutex, K_FOREVER);
    kws_active = false;
    k_mutex_unlock(&kws_mutex);
}

/* =========================================================================
 * Public C API
 * ========================================================================= */

extern "C" bool kws_is_active(void)
{
    k_mutex_lock(&kws_mutex, K_FOREVER);
    bool a = kws_active;
    k_mutex_unlock(&kws_mutex);
    return a;
}

/* =========================================================================
 * Shell commands
 * ========================================================================= */

static int cmd_kws_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&kws_mutex, K_FOREVER);

    if (kws_active) {
        shell_error(sh, "KWS already running -- use 'kws stop' first");
        k_mutex_unlock(&kws_mutex);
        return -EBUSY;
    }

    if (rec_is_active()) {
        shell_error(sh, "WAV recording in progress -- use 'record stop' first");
        k_mutex_unlock(&kws_mutex);
        return -EBUSY;
    }

    atomic_set(&kws_stop_req, 0);
    kws_active = true;
    k_mutex_unlock(&kws_mutex);

    k_thread_create(&kws_thread, kws_stack,
                    K_THREAD_STACK_SIZEOF(kws_stack),
                    kws_thread_fn, NULL, NULL, NULL,
                    6, 0, K_NO_WAIT);
    k_thread_name_set(&kws_thread, "kws");

    shell_print(sh, "KWS started (threshold=%.2f) -- use 'kws stop' to end",
                (double)kws_threshold);
    return 0;
}

static int cmd_kws_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    k_mutex_lock(&kws_mutex, K_FOREVER);
    bool active = kws_active;
    k_mutex_unlock(&kws_mutex);

    if (!active) {
        shell_error(sh, "KWS is not running");
        return -ENODEV;
    }

    shell_print(sh, "Stopping KWS...");
    atomic_set(&kws_stop_req, 1);

    int ret = k_thread_join(&kws_thread, K_SECONDS(5));
    if (ret < 0) {
        shell_error(sh, "KWS thread join timeout: %d", ret);
        return ret;
    }

    shell_print(sh, "KWS stopped");
    return 0;
}

static int cmd_kws_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (kws_is_active()) {
        shell_print(sh, "KWS running  threshold=%.2f  labels=%d",
                    (double)kws_threshold, EI_CLASSIFIER_LABEL_COUNT);
        for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            shell_print(sh, "  [%d] %s", i,
                        ei_classifier_inferencing_categories[i]);
        }
    } else {
        shell_print(sh, "KWS idle");
    }
    return 0;
}

static int cmd_kws_threshold(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Current threshold: %.2f", (double)kws_threshold);
        return 0;
    }

    char  *end;
    float  val = strtof(argv[1], &end);

    if (end == argv[1] || val < 0.0f || val > 1.0f) {
        shell_error(sh, "Invalid threshold (expected float 0.0 – 1.0)");
        return -EINVAL;
    }

    kws_threshold = val;
    shell_print(sh, "Threshold set to %.2f", (double)kws_threshold);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_kws,
    SHELL_CMD_ARG(start, NULL,
                  "Start continuous keyword detection",
                  cmd_kws_start, 1, 0),
    SHELL_CMD_ARG(stop, NULL,
                  "Stop keyword detection",
                  cmd_kws_stop, 1, 0),
    SHELL_CMD_ARG(status, NULL,
                  "Show KWS state and detected labels",
                  cmd_kws_status, 1, 0),
    SHELL_CMD_ARG(threshold, NULL,
                  "[value]  Get or set detection threshold (0.0 – 1.0)",
                  cmd_kws_threshold, 1, 1),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(kws, &sub_kws, "Keyword spotting (Edge Impulse)", NULL);
