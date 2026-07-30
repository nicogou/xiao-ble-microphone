/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared PDM microphone acquisition layer.
 *
 * A single 10-block slab and a unified open/run/close API replace the
 * per-mode PDM boilerplate in kws.cpp, pitch.c and main.c.
 *
 * Usage pattern in a mode thread:
 *
 *   int ret = mic_open();
 *   if (ret < 0) { ... }
 *   mic_run(&mode_stop_req, mode_block_cb, user_data);
 *   mic_close();
 *
 * The block callback MUST call mic_block_free(buf) when it is done with
 * the data.  mic_run() does NOT free blocks automatically so that
 * computationally heavy callbacks (pitch NSDF, KWS inference) can release
 * the slab slot early and let the driver keep filling new buffers.
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

/* Audio parameters shared across all modes. */
#define MIC_SAMPLE_RATE    16000U
#define MIC_BITS           16U
#define MIC_CHANNELS       1U
#define MIC_BLOCK_MS       100U
#define MIC_BLOCK_SAMPLES  (MIC_SAMPLE_RATE * MIC_BLOCK_MS / 1000U)  /* 1600 */
#define MIC_BLOCK_BYTES    (MIC_BLOCK_SAMPLES * MIC_CHANNELS * (MIC_BITS / 8U))  /* 3200 */

/*
 * Called by mic_run() for every PDM block.
 * The callee MUST call mic_block_free(buf) when done with the data.
 */
typedef void (*mic_block_fn_t)(void *buf, uint32_t size, void *user_data);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enable the microphone regulator, configure and start PDM.
 *
 * Discards the first block (filter warm-up) before returning.
 *
 * @return 0 on success, negative errno on error.
 */
int mic_open(void);

/**
 * @brief Read PDM blocks and invoke cb until *stop_flag is set.
 *
 * Blocks the calling thread.  The stop flag is sampled at the top of the
 * loop; at most one block after the flag is set will still be delivered.
 *
 * @return 0 on clean stop, negative errno on driver error.
 */
int mic_run(atomic_t *stop_flag, mic_block_fn_t cb, void *user_data);

/**
 * @brief Stop PDM, drain the driver queue, and disable the microphone regulator.
 */
void mic_close(void);

/**
 * @brief Return a slab block received in the callback to the shared pool.
 */
void mic_block_free(void *buf);

#ifdef __cplusplus
}
#endif
