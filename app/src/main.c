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
