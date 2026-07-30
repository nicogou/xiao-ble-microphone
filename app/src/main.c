/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include "mic.h"

#include <app_version.h>
#include "kws.h"
#include "pitch.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* ===========================================================================
 * Audio / PDM configuration
 * =========================================================================== */

/* Bytes per second of mono 16-bit 16 kHz PCM – used for WAV math. */
#define AUDIO_BYTES_PER_SEC  (MIC_SAMPLE_RATE * MIC_CHANNELS * (MIC_BITS / 8U))

/* Ring buffer between the PDM reader thread and the FAT writer thread.
 * 8 slots × 3200 B = 25 600 B ≈ 800 ms – absorbs flash-write and USB-MSC
 * latency spikes.  Blocks that arrive while the ring is full are dropped
 * with a warning; the recording stays alive. */
#define RING_BLOCKS 8U
static uint8_t ring_buf[RING_BLOCKS][MIC_BLOCK_BYTES];
static int     ring_wr_idx;
static K_SEM_DEFINE(ring_space_sem, RING_BLOCKS, RING_BLOCKS);

struct ring_msg {
	uint8_t  *buf;   /* pointer into ring_buf; NULL = end-of-recording */
	uint32_t  size;
};
K_MSGQ_DEFINE(ring_msgq, sizeof(struct ring_msg), RING_BLOCKS + 1U, 4);

/* ===========================================================================
 * WAV file header (44-byte RIFF PCM)
 * =========================================================================== */

struct wav_hdr {
uint8_t  riff[4];
uint32_t riff_size;   /* total file size - 8             */
uint8_t  wave[4];
uint8_t  fmt_id[4];
uint32_t fmt_size;    /* 16 for PCM                      */
uint16_t audio_fmt;   /* 1 = PCM                         */
uint16_t channels;
uint32_t sample_rate;
uint32_t byte_rate;
uint16_t block_align;
uint16_t bits;
uint8_t  data_id[4];
uint32_t data_size;
} __packed;

/* ===========================================================================
 * USB device setup (composite CDC-ACM + MSC)
 * =========================================================================== */

/* VID 0x2FE3 is the Zephyr project vendor ID -- replace with your own. */
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
USBD_CONFIGURATION_DEFINE(usbd_fs_config, 0, 125, &usbd_fs_cfg_desc);

/* "NAND" must match disk-name in the board overlay */
USBD_DEFINE_MSC_LUN(nand, "NAND", "Seeed", "XIAO Flash", "0.01");

static const char *const usbd_blocklist[] = {NULL};

/* ===========================================================================
 * Recording state
 * =========================================================================== */

static K_MUTEX_DEFINE(rec_mutex);
static bool      rec_active;
static atomic_t  rec_bytes        = ATOMIC_INIT(0); /* bytes written to file */
static atomic_t  rec_bytes_queued = ATOMIC_INIT(0); /* bytes enqueued to ring */
static atomic_t  rec_stop_req     = ATOMIC_INIT(0);
static uint32_t  rec_max_bytes;

/* PDM reader thread (high priority – keeps up with hardware) */
#define REC_STACK_SIZE 2048
static K_THREAD_STACK_DEFINE(rec_stack, REC_STACK_SIZE);
static struct k_thread rec_thread;

/* FAT writer thread (lower priority – may stall on flash/USB without issue) */
#define WRITE_STACK_SIZE 4096
static K_THREAD_STACK_DEFINE(write_stack, WRITE_STACK_SIZE);
static struct k_thread write_thread;

static struct fs_file_t  rec_file;
static FATFS             rec_fat_fs;
static struct fs_mount_t rec_mnt = {
	.type      = FS_FATFS,
	.fs_data   = &rec_fat_fs,
	.mnt_point = "/NAND:",
};

/* ===========================================================================
 * PDM block callback for the recording mode.
 *
 * Clamps the block to the max-duration limit, copies it into the ring buffer,
 * and posts it to the FAT writer.  K_NO_WAIT on the semaphore means a full
 * ring (writer stalled by flash or USB) causes a block drop rather than
 * blocking the PDM acquisition path.
 * =========================================================================== */

static void rec_pdm_cb(void *buf, uint32_t size, void *user_data)
{
	ARG_UNUSED(user_data);

	uint32_t to_copy = size;

	if (rec_max_bytes > 0) {
		uint32_t queued = (uint32_t)atomic_get(&rec_bytes_queued);

		if (queued >= rec_max_bytes) {
			mic_block_free(buf);
			atomic_set(&rec_stop_req, 1);
			return;
		}
		uint32_t remaining = rec_max_bytes - queued;

		if (to_copy > remaining) {
			to_copy = remaining;
		}
	}

	if (k_sem_take(&ring_space_sem, K_NO_WAIT) != 0) {
		LOG_WRN("Ring full - dropping PDM block (flash/USB busy)");
		mic_block_free(buf);
		return;
	}
	memcpy(ring_buf[ring_wr_idx], buf, to_copy);
	mic_block_free(buf);

	struct ring_msg msg = {.buf = ring_buf[ring_wr_idx], .size = to_copy};

	k_msgq_put(&ring_msgq, &msg, K_NO_WAIT);
	ring_wr_idx = (ring_wr_idx + 1) % RING_BLOCKS;
	atomic_add(&rec_bytes_queued, to_copy);

	/* Stop after the last clamped block. */
	if (rec_max_bytes > 0 &&
	    (uint32_t)atomic_get(&rec_bytes_queued) >= rec_max_bytes) {
		atomic_set(&rec_stop_req, 1);
	}
}

/* ===========================================================================
 * PDM reader thread
 * Opens the shared mic layer, runs the read loop, then sends a sentinel to
 * the FAT writer.
 * =========================================================================== */

static void rec_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (mic_open() < 0) {
		goto send_sentinel;
	}

	mic_run(&rec_stop_req, rec_pdm_cb, NULL);
	mic_close();

send_sentinel:
	{
		struct ring_msg sentinel = {.buf = NULL, .size = 0};

		k_msgq_put(&ring_msgq, &sentinel, K_FOREVER);
	}
}

/* ===========================================================================
 * FAT writer thread
 * Drains the ring buffer and writes each block to the open FAT file.
 * Slow flash erases and USB-MSC mutex contention only block this thread;
 * the PDM reader above is completely unaffected.
 * =========================================================================== */

static void write_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Set to false on the first write error so we stop hammering fs_write
	 * while still draining the ring (semaphore slots must be returned so
	 * the ring is reusable for the next recording session). */
	bool write_ok = true;

	while (true) {
		struct ring_msg msg;

		k_msgq_get(&ring_msgq, &msg, K_FOREVER);

		if (!msg.buf) {
			break;  /* NULL sentinel = PDM reader has finished */
		}

		if (write_ok) {
			ssize_t written = fs_write(&rec_file, msg.buf, msg.size);

			if (written == (ssize_t)msg.size) {
				/* Full block written – normal path */
				atomic_add(&rec_bytes, (atomic_val_t)written);
			} else {
				/* Partial write (ELM FAT ran out of clusters
				 * mid-block and returned FR_OK with bw < btw)
				 * OR a hard write error (written < 0).
				 * Either way the disk is full. */
				if (written > 0) {
					atomic_add(&rec_bytes,
						   (atomic_val_t)written);
				}
				LOG_WRN("Storage full (%zd/%u B written) "
					"– stopping recording",
					written, msg.size);
				atomic_set(&rec_stop_req, 1);
				write_ok = false;
			}
		}

		k_sem_give(&ring_space_sem);  /* always return the ring slot */
	}

	/* Drain any leftover messages (e.g. after a write error) */
	{
		struct ring_msg msg;

		while (k_msgq_get(&ring_msgq, &msg, K_NO_WAIT) == 0) {
			if (msg.buf) {
				k_sem_give(&ring_space_sem);
			}
		}
	}

	/* Truncate to the actual recorded size, releasing unused pre-allocated
	 * clusters back to the FAT.  This is done once at the end so the file
	 * on the drive exactly matches the recorded audio. */
	{
		uint32_t data_sz = (uint32_t)atomic_get(&rec_bytes);
		int tret = fs_truncate(&rec_file,
				       (off_t)(sizeof(struct wav_hdr) + data_sz));

		if (tret < 0) {
			LOG_WRN("Truncate to actual size failed (%d)", tret);
		}
	}

	/* Seek back and write the final WAV header */
	{
		uint32_t data_sz = (uint32_t)atomic_get(&rec_bytes);
		struct wav_hdr hdr = {
			.riff        = {'R', 'I', 'F', 'F'},
			.riff_size   = data_sz + sizeof(struct wav_hdr) - 8,
			.wave        = {'W', 'A', 'V', 'E'},
			.fmt_id      = {'f', 'm', 't', ' '},
			.fmt_size    = 16,
			.audio_fmt   = 1,
			.channels    = MIC_CHANNELS,
			.sample_rate = MIC_SAMPLE_RATE,
			.byte_rate   = AUDIO_BYTES_PER_SEC,
			.block_align = MIC_CHANNELS * (MIC_BITS / 8U),
			.bits        = MIC_BITS,
			.data_id     = {'d', 'a', 't', 'a'},
			.data_size   = data_sz,
		};

		fs_seek(&rec_file, 0, FS_SEEK_SET);
		fs_write(&rec_file, &hdr, sizeof(hdr));
	}

	fs_close(&rec_file);
	fs_unmount(&rec_mnt);

	{
		uint32_t total = (uint32_t)atomic_get(&rec_bytes);
		uint32_t ms    = total * 1000U / AUDIO_BYTES_PER_SEC;

		LOG_INF("Recording saved: %u bytes (%u.%03u s)",
			total, ms / 1000U, ms % 1000U);
	}

	k_mutex_lock(&rec_mutex, K_FOREVER);
	rec_active = false;
	k_mutex_unlock(&rec_mutex);
}

/* ===========================================================================
 * rec_is_active() – queried by kws.cpp to prevent simultaneous use of the mic
 * =========================================================================== */

bool rec_is_active(void)
{
	k_mutex_lock(&rec_mutex, K_FOREVER);
	bool a = rec_active;
	k_mutex_unlock(&rec_mutex);
	return a;
}

/* ===========================================================================
 * Shell commands
 * =========================================================================== */

static int cmd_record_start(const struct shell *sh, size_t argc, char **argv)
{
	k_mutex_lock(&rec_mutex, K_FOREVER);

	if (rec_active) {
		shell_error(sh, "Already recording -- use 'record stop' first");
		k_mutex_unlock(&rec_mutex);
		return -EBUSY;
	}

	if (kws_is_active()) {
		shell_error(sh, "KWS is running -- use 'kws stop' first");
		k_mutex_unlock(&rec_mutex);
		return -EBUSY;
	}

	if (pitch_is_active()) {
		shell_error(sh, "Pitch detection is running -- use 'pitch stop' first");
		k_mutex_unlock(&rec_mutex);
		return -EBUSY;
	}

	uint32_t max_sec = 0;

	if (argc >= 2) {
		long v = strtol(argv[1], NULL, 10);

		if (v > 0) {
			max_sec = (uint32_t)v;
		}
	}

	int ret = fs_mount(&rec_mnt);

	if (ret < 0) {
		shell_error(sh, "fs_mount: %d", ret);
		k_mutex_unlock(&rec_mutex);
		return ret;
	}

	/* Find next free recNNNN.wav */
	char path[32];
	int i;

	for (i = 1; i <= 9999; i++) {
		snprintf(path, sizeof(path), "/NAND:/rec%04d.wav", i);
		struct fs_dirent de;

		if (fs_stat(path, &de) == -ENOENT) {
			break;
		}
	}
	if (i > 9999) {
		shell_error(sh, "No free filename (delete old recordings)");
		fs_unmount(&rec_mnt);
		k_mutex_unlock(&rec_mutex);
		return -ENOSPC;
	}

	fs_file_t_init(&rec_file);
	ret = fs_open(&rec_file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret < 0) {
		shell_error(sh, "fs_open: %d", ret);
		fs_unmount(&rec_mnt);
		k_mutex_unlock(&rec_mutex);
		return ret;
	}

	/* Pre-allocate clusters with f_expand BEFORE writing any data.
	 * f_expand requires fp->fsize == 0, so it must run on the empty file.
	 *
	 * Query free space first so we never request more clusters than are
	 * actually available (which causes FR_DENIED on a non-empty disk).
	 * We clamp the pre-allocation to the available space minus one cluster
	 * reserve, and also to the requested max duration if one was given.
	 *
	 * f_expand with opt=1 (contiguous) automatically falls back to
	 * non-contiguous allocation when no single contiguous block exists.
	 * The file pointer is unchanged after the call (stays at 0), so
	 * writing the placeholder header below advances it to sizeof(wav_hdr)
	 * -- exactly where audio data should start. */
	{
		struct fs_statvfs sv;
		FSIZE_t prealloc_bytes = 0;

		if (fs_statvfs(rec_mnt.mnt_point, &sv) == 0 && sv.f_bfree > 1U) {
			/* Leave 1 cluster as a reserve so the FAT never runs
			 * completely full (needed for directory-entry writes). */
			uint64_t avail = (uint64_t)(sv.f_bfree - 1U) * sv.f_frsize;

			/* Desired size: user-requested max + 1 s margin, or all
			 * available space if no limit was given. */
			uint64_t wanted = (max_sec > 0U)
				? (uint64_t)(max_sec + 1U) * AUDIO_BYTES_PER_SEC
				  + sizeof(struct wav_hdr)
				: avail;

			uint64_t alloc = MIN(avail, wanted);

			if (max_sec > 0U && avail < wanted) {
				uint32_t avail_s = (uint32_t)(avail / AUDIO_BYTES_PER_SEC);

				LOG_WRN("Only %u s of free space, clamping "
					"pre-alloc from %u s",
					avail_s, max_sec);
			}

			/* Only bother if we can secure at least 5 s */
			if (alloc >= (uint64_t)5U * AUDIO_BYTES_PER_SEC) {
				prealloc_bytes = (FSIZE_t)alloc;
			}
		}

		if (prealloc_bytes > 0) {
			FRESULT fr = f_expand(
				(FIL *)rec_file.filep, prealloc_bytes, 1);

			if (fr != FR_OK) {
				LOG_WRN("f_expand failed (%u) - recording "
					"without pre-allocation", (unsigned)fr);
			}
		} else {
			LOG_WRN("Insufficient free space for pre-allocation");
		}
	}

	/* Write a zeroed placeholder header at position 0.
	 * After this write the file pointer is at sizeof(wav_hdr), ready for
	 * audio data.  The final WAV header is written by write_thread on stop. */
	static const uint8_t placeholder[sizeof(struct wav_hdr)];

	fs_write(&rec_file, placeholder, sizeof(placeholder));

	/* Reset all recording state */
	atomic_set(&rec_bytes, 0);
	atomic_set(&rec_bytes_queued, 0);
	atomic_set(&rec_stop_req, 0);
	rec_max_bytes = max_sec > 0 ? max_sec * AUDIO_BYTES_PER_SEC : 0;

	/* Reset ring buffer */
	ring_wr_idx = 0;
	k_sem_init(&ring_space_sem, RING_BLOCKS, RING_BLOCKS);
	k_msgq_purge(&ring_msgq);

	rec_active = true;
	k_mutex_unlock(&rec_mutex);

	/* Start PDM reader (priority 5) then FAT writer (priority 7) */
	k_thread_create(&rec_thread, rec_stack,
			K_THREAD_STACK_SIZEOF(rec_stack),
			rec_thread_fn, NULL, NULL, NULL,
			5, 0, K_NO_WAIT);
	k_thread_create(&write_thread, write_stack,
			K_THREAD_STACK_SIZEOF(write_stack),
			write_thread_fn, NULL, NULL, NULL,
			7, 0, K_NO_WAIT);

	if (max_sec > 0) {
		shell_print(sh, "Recording to %s (max %u s) -- "
			    "use 'record stop' to end early", path, max_sec);
	} else {
		shell_print(sh, "Recording to %s -- "
			    "use 'record stop' to finish", path);
	}

	return 0;
}

static int cmd_record_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_mutex_lock(&rec_mutex, K_FOREVER);
	bool active = rec_active;

	k_mutex_unlock(&rec_mutex);

	if (!active) {
		shell_error(sh, "Not recording");
		return -ENODEV;
	}

	shell_print(sh, "Stopping...");
	atomic_set(&rec_stop_req, 1);

	/* Wait for PDM reader to stop and send the ring sentinel (fast) */
	int ret = k_thread_join(&rec_thread, K_SECONDS(5));

	if (ret < 0) {
		shell_error(sh, "PDM reader join timeout: %d", ret);
		return ret;
	}

	/* Wait for FAT writer to drain the ring and close the file (slow) */
	ret = k_thread_join(&write_thread, K_SECONDS(30));
	if (ret < 0) {
		shell_error(sh, "FAT writer join timeout: %d", ret);
		return ret;
	}

	uint32_t total = (uint32_t)atomic_get(&rec_bytes);
	uint32_t ms    = total * 1000U / AUDIO_BYTES_PER_SEC;

	shell_print(sh, "Saved %u bytes (%u.%03u s)", total, ms / 1000U, ms % 1000U);
	return 0;
}

static int cmd_record_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_mutex_lock(&rec_mutex, K_FOREVER);
	bool active = rec_active;

	k_mutex_unlock(&rec_mutex);

	if (active) {
		/* rec_bytes_queued tracks what the PDM reader has captured;
		 * rec_bytes tracks what the FAT writer has flushed (lags by
		 * up to RING_BLOCKS blocks during fast recordings). */
		uint32_t captured = (uint32_t)atomic_get(&rec_bytes_queued);
		uint32_t written  = (uint32_t)atomic_get(&rec_bytes);
		uint32_t ms = captured * 1000U / AUDIO_BYTES_PER_SEC;

		shell_print(sh, "Recording: %u B captured, %u B written (%u.%03u s)",
			    captured, written, ms / 1000U, ms % 1000U);
	} else {
		shell_print(sh, "Idle");
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_record,
	SHELL_CMD_ARG(start, NULL,
		      "[max_seconds]  Start recording to recNNNN.wav",
		      cmd_record_start, 1, 1),
	SHELL_CMD_ARG(stop, NULL,
		      "Stop recording and write WAV header",
		      cmd_record_stop, 1, 0),
	SHELL_CMD_ARG(status, NULL,
		      "Show recording state and elapsed time",
		      cmd_record_status, 1, 0),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(record, &sub_record, "PDM WAV recorder", NULL);

/* ===========================================================================
 * main -- init disk and USB, then idle (shell runs in its own context)
 * =========================================================================== */

int main(void)
{
int ret;

ret = disk_access_init("NAND");
if (ret) {
LOG_ERR("disk_access_init: %d", ret);
return ret;
}

ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_lang);
if (ret) {
LOG_ERR("lang descriptor: %d", ret);
return ret;
}
ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_mfr);
if (ret) {
LOG_ERR("mfr descriptor: %d", ret);
return ret;
}
ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_product);
if (ret) {
LOG_ERR("product descriptor: %d", ret);
return ret;
}
#if defined(CONFIG_HWINFO)
ret = usbd_add_descriptor(&usbd_msc_dev, &usbd_sn);
if (ret) {
LOG_ERR("sn descriptor: %d", ret);
return ret;
}
#endif
ret = usbd_add_configuration(&usbd_msc_dev, USBD_SPEED_FS,
     &usbd_fs_config);
if (ret) {
LOG_ERR("add_configuration: %d", ret);
return ret;
}
ret = usbd_register_all_classes(&usbd_msc_dev, USBD_SPEED_FS, 1,
usbd_blocklist);
if (ret) {
LOG_ERR("register_all_classes: %d", ret);
return ret;
}
#if defined(CONFIG_USBD_CDC_ACM_CLASS)
ret = usbd_device_set_code_triple(&usbd_msc_dev, USBD_SPEED_FS,
  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
if (ret) {
LOG_ERR("set_code_triple: %d", ret);
return ret;
}
#endif
ret = usbd_init(&usbd_msc_dev);
if (ret) {
LOG_ERR("usbd_init: %d", ret);
return ret;
}
ret = usbd_enable(&usbd_msc_dev);
if (ret) {
LOG_ERR("usbd_enable: %d", ret);
return ret;
}

LOG_INF("XIAO BLE Microphone ready (app %s) -- "
"connect USB and open the serial port", APP_VERSION_STRING);

while (true) {
k_sleep(K_SECONDS(1));
}

return 0;
}
