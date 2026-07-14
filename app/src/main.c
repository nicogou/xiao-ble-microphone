/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/storage/disk_access.h>

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

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

int main(void)
{
	int ret;

	/* Initialise the flash disk driver so disk->dev is populated and
	 * disk_access_status("NAND") returns DISK_STATUS_OK.  Without this
	 * call the flashdisk driver's status check returns DISK_STATUS_NOMEDIA
	 * because disk->dev is NULL, causing the USB MSC SCSI layer to report
	 * "no media" to the host for every TEST UNIT READY command. */
	ret = disk_access_init("NAND");
	if (ret) {
		LOG_ERR("Failed to initialise NAND disk (%d)", ret);
		return ret;
	}

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

	/* Register all class instances (MSC + CDC-ACM console) */
	ret = usbd_register_all_classes(&usbd_msc_dev, USBD_SPEED_FS, 1,
					usbd_blocklist);
	if (ret) {
		LOG_ERR("Failed to register USB classes (%d)", ret);
		return ret;
	}

#if defined(CONFIG_USBD_CDC_ACM_CLASS)
	/*
	 * With CDC-ACM present the device is a composite USB device.
	 * Windows requires the device-level class triple to be set to
	 * Miscellaneous / IAD (0xEF/0x02/0x01) so it can locate both the
	 * serial port and the mass-storage drive via Interface Association
	 * Descriptors.  Without this, Windows only enumerates one function.
	 */
	ret = usbd_device_set_code_triple(&usbd_msc_dev, USBD_SPEED_FS,
					  USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	if (ret) {
		LOG_ERR("Failed to set composite device class triple (%d)", ret);
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

	return 0;
}

