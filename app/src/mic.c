/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mic.h"

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mic, CONFIG_APP_LOG_LEVEL);

/*
 * 8 blocks: hardware double-buffering (2) + driver rx-queue (4) + reader
 * hold (1) + 1 spare.  Only one mode runs at a time so this covers all cases.
 */
#define MIC_SLAB_BLOCKS 8U
K_MEM_SLAB_DEFINE_STATIC(mic_slab, MIC_BLOCK_BYTES, MIC_SLAB_BLOCKS, 4);

static const struct device *mic_dmic;
static const struct device *mic_pwr;

int mic_open(void)
{
	mic_dmic = DEVICE_DT_GET(DT_NODELABEL(pdm0));
	mic_pwr  = DEVICE_DT_GET(DT_NODELABEL(mic_pwr));

	if (!device_is_ready(mic_pwr)) {
		LOG_ERR("mic_pwr not ready");
		return -ENODEV;
	}

	int ret = regulator_enable(mic_pwr);

	if (ret < 0) {
		LOG_ERR("regulator_enable: %d", ret);
		return ret;
	}
	k_sleep(K_MSEC(100));

	struct pcm_stream_cfg stream = {
		.pcm_rate   = MIC_SAMPLE_RATE,
		.pcm_width  = MIC_BITS,
		.block_size = MIC_BLOCK_BYTES,
		.mem_slab   = &mic_slab,
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
			.req_num_chan    = MIC_CHANNELS,
			.req_num_streams = 1,
		},
	};

	ret = dmic_configure(mic_dmic, &cfg);
	if (ret < 0) {
		LOG_ERR("dmic_configure: %d", ret);
		regulator_disable(mic_pwr);
		return ret;
	}

	ret = dmic_trigger(mic_dmic, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("dmic_trigger START: %d", ret);
		regulator_disable(mic_pwr);
		return ret;
	}

	/* Discard the first block: PDM filter warm-up. */
	{
		void    *buf;
		uint32_t size;

		if (dmic_read(mic_dmic, 0, &buf, &size, MIC_BLOCK_MS * 2) == 0) {
			k_mem_slab_free(&mic_slab, buf);
		}
	}

	return 0;
}

int mic_run(atomic_t *stop_flag, mic_block_fn_t cb, void *user_data)
{
	while (!atomic_get(stop_flag)) {
		void    *buf;
		uint32_t size;

		int ret = dmic_read(mic_dmic, 0, &buf, &size, MIC_BLOCK_MS * 2);

		if (ret < 0) {
			LOG_ERR("dmic_read: %d", ret);
			return ret;
		}

		cb(buf, size, user_data);
	}
	return 0;
}

void mic_close(void)
{
	dmic_trigger(mic_dmic, DMIC_TRIGGER_STOP);

	void    *buf;
	uint32_t size;

	while (dmic_read(mic_dmic, 0, &buf, &size, 0) == 0) {
		k_mem_slab_free(&mic_slab, buf);
	}

	regulator_disable(mic_pwr);
}

void mic_block_free(void *buf)
{
	k_mem_slab_free(&mic_slab, buf);
}
