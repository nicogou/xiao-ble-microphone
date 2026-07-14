/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/regulator.h>
#include <ff.h>

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* ===========================================================================
 * Audio configuration
 * =========================================================================== */

#define SAMPLE_RATE      16000U
#define NUM_CHANNELS     1U
#define BITS_PER_SAMPLE  16U
#define BYTES_PER_SAMPLE (BITS_PER_SAMPLE / 8U)
#define RECORD_SECONDS   1U

/* 100 ms PDM blocks; two fit in RAM simultaneously for double-buffering */
#define BLOCK_MS         100U
#define BLOCK_SAMPLES    (SAMPLE_RATE * BLOCK_MS / 1000U)
#define BLOCK_BYTES      (BLOCK_SAMPLES * NUM_CHANNELS * BYTES_PER_SAMPLE)

/* 4 blocks needed: the nRF52840 PDM hardware double-buffers internally, so the
 * driver always requires 2 buffers staged in hardware, plus 1 queued for the
 * application to read, plus 1 spare so the driver can stage the next fill
 * without stalling.  Using only 2 causes ENOMEM in dmic_nrfx_pdm. */
#define SLAB_NUM_BLOCKS  4U
K_MEM_SLAB_DEFINE_STATIC(pdm_mem_slab, BLOCK_BYTES, SLAB_NUM_BLOCKS, 4);

#define AUDIO_BYTES (SAMPLE_RATE * RECORD_SECONDS * NUM_CHANNELS * BYTES_PER_SAMPLE)
/* 32 000 B, kept in BSS so it does not consume stack */
static int16_t audio_buf[SAMPLE_RATE * RECORD_SECONDS];

/* ===========================================================================
 * WAV file header (44 bytes, standard RIFF PCM)
 * =========================================================================== */

struct wav_hdr {
	uint8_t  riff[4];
	uint32_t riff_size;   /* total file size - 8 */
	uint8_t  wave[4];
	uint8_t  fmt_id[4];
	uint32_t fmt_size;    /* 16 for PCM */
	uint16_t audio_fmt;   /* 1 = PCM */
	uint16_t channels;
	uint32_t sample_rate;
	uint32_t byte_rate;
	uint16_t block_align;
	uint16_t bits;
	uint8_t  data_id[4];
	uint32_t data_size;
} __packed;

/* ---------------------------------------------------------------------------
 * USB device descriptors
 *
 * VID 0x2FE3 is the Zephyr project vendor ID – replace with your own for
 * production use.
 * --------------------------------------------------------------------------- */
USBD_DEVICE_DEFINE(usbd_msc_dev,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2FE3, 0x0008);

USBD_DESC_LANG_DEFINE(usbd_lang);
USBD_DESC_MANUFACTURER_DEFINE(usbd_mfr, "Seeed Studio");
USBD_DESC_PRODUCT_DEFINE(usbd_product, "XIAO BLE Flash Disk");

#if defined(CONFIG_HWINFO)
USBD_DESC_SERIAL_NUMBER_DEFINE(usbd_sn);
#endif

USBD_DESC_CONFIG_DEFINE(usbd_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(usbd_fs_config,
			   0,   /* attributes: bus-powered */
			   125, /* bMaxPower: 250 mA (in 2 mA units) */
			   &usbd_fs_cfg_desc);

/* ---------------------------------------------------------------------------
 * MSC Logical Unit
 *
 * "NAND" must match the disk-name property of the msc_disk0 DTS node in the
 * board overlay.
 * --------------------------------------------------------------------------- */
USBD_DEFINE_MSC_LUN(nand, "NAND", "Seeed", "XIAO Flash", "0.01");

/* No class instances are excluded from registration */
static const char *const usbd_blocklist[] = {NULL};

/* ===========================================================================
 * Recording
 * =========================================================================== */

static int record_audio(void)
{
	const struct device *dmic = DEVICE_DT_GET(DT_NODELABEL(pdm0));

	if (!device_is_ready(dmic)) {
		LOG_ERR("PDM device not ready");
		return -ENODEV;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width  = BITS_PER_SAMPLE,
		.pcm_rate   = SAMPLE_RATE,
		.block_size = BLOCK_BYTES,
		.mem_slab   = &pdm_mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* PDM clock range for the MSM261D3526HICPM-C */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3200000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan    = NUM_CHANNELS,
			/*
			 * XIAO BLE Sense SELECT pin is tied low: data is on the
			 * clock falling edge (left channel).
			 * Change to PDM_CHAN_RIGHT if the recording is silent.
			 */
			.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT),
		},
	};

	int ret = dmic_configure(dmic, &cfg);

	if (ret < 0) {
		LOG_ERR("dmic_configure failed: %d", ret);
		return ret;
	}

	ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("DMIC START failed: %d", ret);
		return ret;
	}

	/* Discard the first block – PDM filter warm-up noise */
	{
		void *buf;
		uint32_t size;

		ret = dmic_read(dmic, 0, &buf, &size, BLOCK_MS + 500);
		if (ret < 0) {
			LOG_ERR("DMIC warm-up read failed: %d", ret);
			dmic_trigger(dmic, DMIC_TRIGGER_STOP);
			return ret;
		}
		k_mem_slab_free(&pdm_mem_slab, buf);
	}

	const uint32_t total_blocks = (SAMPLE_RATE * RECORD_SECONDS) / BLOCK_SAMPLES;
	uint32_t offset = 0;

	for (uint32_t i = 0; i < total_blocks; i++) {
		void *buf;
		uint32_t size;

		ret = dmic_read(dmic, 0, &buf, &size, BLOCK_MS + 500);
		if (ret < 0) {
			LOG_ERR("DMIC read block %u failed: %d", i, ret);
			break;
		}

		uint32_t to_copy = MIN(size, AUDIO_BYTES - offset);

		memcpy((uint8_t *)audio_buf + offset, buf, to_copy);
		offset += to_copy;
		k_mem_slab_free(&pdm_mem_slab, buf);
	}

	dmic_trigger(dmic, DMIC_TRIGGER_STOP);
	return (ret < 0) ? ret : 0;
}

/* ===========================================================================
 * WAV file save
 * =========================================================================== */

static int save_wav(const char *path)
{
	struct wav_hdr hdr = {
		.riff        = {'R', 'I', 'F', 'F'},
		.riff_size   = AUDIO_BYTES + sizeof(struct wav_hdr) - 8,
		.wave        = {'W', 'A', 'V', 'E'},
		.fmt_id      = {'f', 'm', 't', ' '},
		.fmt_size    = 16,
		.audio_fmt   = 1,
		.channels    = NUM_CHANNELS,
		.sample_rate = SAMPLE_RATE,
		.byte_rate   = SAMPLE_RATE * NUM_CHANNELS * BYTES_PER_SAMPLE,
		.block_align = NUM_CHANNELS * BYTES_PER_SAMPLE,
		.bits        = BITS_PER_SAMPLE,
		.data_id     = {'d', 'a', 't', 'a'},
		.data_size   = AUDIO_BYTES,
	};

	struct fs_file_t file;

	fs_file_t_init(&file);

	int ret = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);

	if (ret < 0) {
		LOG_ERR("fs_open(%s) failed: %d", path, ret);
		return ret;
	}

	ssize_t written = fs_write(&file, &hdr, sizeof(hdr));

	if (written < 0) {
		LOG_ERR("Failed to write WAV header: %zd", written);
		fs_close(&file);
		return (int)written;
	}

	written = fs_write(&file, audio_buf, AUDIO_BYTES);
	if (written < 0) {
		LOG_ERR("Failed to write audio data: %zd", written);
		fs_close(&file);
		return (int)written;
	}

	return fs_close(&file);
}

/* ===========================================================================
 * main
 * =========================================================================== */

int main(void)
{
	int ret;

	/* 1. Initialise the QSPI flash disk ---------------------------------- */
	ret = disk_access_init("NAND");
	if (ret) {
		LOG_ERR("Failed to initialise NAND disk (%d)", ret);
		return ret;
	}

	/* 2. Power up the PDM microphone ------------------------------------- */
	const struct device *mic_pwr = DEVICE_DT_GET(DT_NODELABEL(mic_pwr));

	if (!device_is_ready(mic_pwr)) {
		LOG_ERR("Mic power regulator not ready");
		return -ENODEV;
	}
	ret = regulator_enable(mic_pwr);
	if (ret < 0) {
		LOG_ERR("Failed to enable mic power: %d", ret);
		return ret;
	}
	k_sleep(K_MSEC(100)); /* wait for rail and mic to stabilise */

	/* 3. Mount FAT -------------------------------------------------------
	 * CONFIG_FS_FATFS_MOUNT_MKFS=y auto-formats on first use.
	 * --------------------------------------------------------------------- */
	static FATFS fat_fs;
	static struct fs_mount_t mnt = {
		.type      = FS_FATFS,
		.fs_data   = &fat_fs,
		.mnt_point = "/NAND:",
	};

	ret = fs_mount(&mnt);
	if (ret < 0) {
		LOG_ERR("fs_mount failed (%d) – connect USB and format drive", ret);
		goto usb_init;
	}
	LOG_INF("FAT mounted on %s", mnt.mnt_point);

	/* 4. Record 1 second ------------------------------------------------- */
	LOG_INF("Recording %u s at %u Hz...", RECORD_SECONDS, SAMPLE_RATE);
	ret = record_audio();
	if (ret < 0) {
		LOG_ERR("Recording failed: %d", ret);
		goto unmount;
	}
	LOG_INF("Recording complete (%u B captured)", AUDIO_BYTES);

	/* 5. Save WAV -------------------------------------------------------- */
	ret = save_wav("/NAND:/record.wav");
	if (ret < 0) {
		LOG_ERR("Failed to save WAV: %d", ret);
	} else {
		LOG_INF("Saved /NAND:/record.wav");
	}

unmount:
	/* 6. Unmount before handing disk to USB MSC -------------------------- */
	fs_unmount(&mnt);
	LOG_INF("FAT unmounted");

	/* 7. Mic power off --------------------------------------------------- */
	regulator_disable(mic_pwr);

usb_init:
	/* 8. Bring up USB (MSC + CDC-ACM console) ---------------------------- */
	ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_lang);
	if (ret) {
		LOG_ERR("Failed to add language descriptor (%d)", ret);
		return ret;
	}

	ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_mfr);
	if (ret) {
		LOG_ERR("Failed to add manufacturer descriptor (%d)", ret);
		return ret;
	}

	ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_product);
	if (ret) {
		LOG_ERR("Failed to add product descriptor (%d)", ret);
		return ret;
	}

#if defined(CONFIG_HWINFO)
	ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_sn);
	if (ret) {
		LOG_ERR("Failed to add serial number descriptor (%d)", ret);
		return ret;
	}
#endif

	ret = usbd_add_configuration(&usbd_msc_dev, USBD_SPEED_FS,
				     &usbd_fs_config);
	if (ret) {
		LOG_ERR("Failed to add FS configuration (%d)", ret);
		return ret;
	}

	ret = usbd_register_all_classes(&usbd_msc_dev, USBD_SPEED_FS, 1,
					usbd_blocklist);
	if (ret) {
		LOG_ERR("Failed to register USB classes (%d)", ret);
		return ret;
	}

#if defined(CONFIG_USBD_CDC_ACM_CLASS)
	ret = usbd_device_set_code_triple(&usbd_msc_dev, USBD_SPEED_FS,
					  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	if (ret) {
		LOG_ERR("Failed to set composite class triple (%d)", ret);
		return ret;
	}
#endif

	ret = usbd_init(&usbd_msc_dev);
	if (ret) {
		LOG_ERR("Failed to initialise USB device (%d)", ret);
		return ret;
	}

	ret = usbd_enable(&usbd_msc_dev);
	if (ret) {
		LOG_ERR("Failed to enable USB device (%d)", ret);
		return ret;
	}

	LOG_INF("XIAO BLE Flash Disk ready (app %s)", APP_VERSION_STRING);

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
