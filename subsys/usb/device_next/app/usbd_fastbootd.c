/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file usbd_fastbootd.c
 * @brief USB composite device: CDC ACM + Fastboot, auto-init via SYS_INIT
 *
 * Enable with CONFIG_USBD_FASTBOOTD=y.
 * Registers CDC ACM and Fastboot classes to a shared context.
 * Application must provide fastboot_ops via fastboot_register_ops()
 * before usbd_init() is called (use SYS_INIT with lower priority value).
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_fastbootd, CONFIG_USBD_LOG_LEVEL);

/* ============================================================
 * Device Context & Descriptors
 * ============================================================ */

USBD_DEVICE_DEFINE(usbd_fastbootd_ctx,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   CONFIG_USBD_FASTBOOTD_VID,
		   CONFIG_USBD_FASTBOOTD_PID);

USBD_DESC_LANG_DEFINE(usbd_fastbootd_lang);
USBD_DESC_MANUFACTURER_DEFINE(usbd_fastbootd_mfr,
	CONFIG_USBD_FASTBOOTD_MANUFACTURER_STRING);
USBD_DESC_PRODUCT_DEFINE(usbd_fastbootd_product,
	CONFIG_USBD_FASTBOOTD_PRODUCT_STRING);
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(usbd_fastbootd_sn)));

USBD_DESC_CONFIG_DEFINE(usbd_fastbootd_fs_cfg, "Composite FS");
USBD_DESC_CONFIG_DEFINE(usbd_fastbootd_hs_cfg, "Composite HS");

USBD_CONFIGURATION_DEFINE(usbd_fastbootd_fs_config,
	IS_ENABLED(CONFIG_USBD_FASTBOOTD_SELF_POWERED) ?
	USB_SCD_SELF_POWERED : 0,
	CONFIG_USBD_FASTBOOTD_MAX_POWER, &usbd_fastbootd_fs_cfg);

USBD_CONFIGURATION_DEFINE(usbd_fastbootd_hs_config,
	IS_ENABLED(CONFIG_USBD_FASTBOOTD_SELF_POWERED) ?
	USB_SCD_SELF_POWERED : 0,
	CONFIG_USBD_FASTBOOTD_MAX_POWER, &usbd_fastbootd_hs_cfg);

/* ============================================================
 * VBUS callback
 * ============================================================ */

static void bootloader_msg_cb(struct usbd_context *const ctx,
			      const struct usbd_msg *msg)
{
	switch (msg->type) {
	case USBD_MSG_VBUS_READY:
		usbd_enable(ctx);
		break;
	case USBD_MSG_VBUS_REMOVED:
		usbd_disable(ctx);
		break;
	default:
		break;
	}
}

/* ============================================================
 * Init
 * ============================================================ */

static int usbd_fastbootd_init(void)
{
	struct usbd_context *ctx = &usbd_fastbootd_ctx;
	int err;

	/* String descriptors */
	usbd_add_descriptor(ctx, &usbd_fastbootd_lang);
	usbd_add_descriptor(ctx, &usbd_fastbootd_mfr);
	usbd_add_descriptor(ctx, &usbd_fastbootd_product);
	IF_ENABLED(CONFIG_HWINFO, (
		usbd_add_descriptor(ctx, &usbd_fastbootd_sn);
	))

	/* VBUS callback */
	usbd_msg_register_cb(ctx, bootloader_msg_cb);

	/* Add configs and register classes */
	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(ctx) == USBD_SPEED_HS) {
		err = usbd_add_configuration(ctx, USBD_SPEED_HS,
					     &usbd_fastbootd_hs_config);
		if (err) {
			LOG_ERR("Failed to add HS config: %d", err);
			return err;
		}
		err = usbd_register_class(ctx, "cdc_acm_0", USBD_SPEED_HS, 1);
	LOG_DBG("register cdc_acm HS = %d\n", err);
		IF_ENABLED(CONFIG_USBD_FASTBOOT_CLASS, (
			err = usbd_register_class(ctx, "fastboot_class",
						  USBD_SPEED_HS, 1);
	LOG_DBG("register fastboot HS = %d\n",
			       err);
		))
		usbd_device_set_code_triple(ctx, USBD_SPEED_HS,
			USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	}

	err = usbd_add_configuration(ctx, USBD_SPEED_FS,
				     &usbd_fastbootd_fs_config);
	if (err) {
		LOG_ERR("Failed to add FS config: %d", err);
		return err;
	}
	err = usbd_register_class(ctx, "cdc_acm_0", USBD_SPEED_FS, 1);
	LOG_DBG("register cdc_acm FS = %d\n", err);
	IF_ENABLED(CONFIG_USBD_FASTBOOT_CLASS, (
		err = usbd_register_class(ctx, "fastboot_class",
					  USBD_SPEED_FS, 1);
	LOG_DBG("register fastboot FS = %d\n", err);
	))
	usbd_device_set_code_triple(ctx, USBD_SPEED_FS,
		USB_BCC_MISCELLANEOUS, 0x02, 0x01);

	/* Init USB */
	err = usbd_init(ctx);
	if (err) {
		LOG_ERR("Failed to init USB: %d", err);
		return err;
	}

	/* Enable */
	if (!usbd_can_detect_vbus(ctx)) {
		err = usbd_enable(ctx);
		if (err) {
			LOG_ERR("Failed to enable USB: %d", err);
			return err;
		}
	}

	return 0;
}

SYS_INIT(usbd_fastbootd_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
