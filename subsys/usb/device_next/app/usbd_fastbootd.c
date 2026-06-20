/*
 * Copyright (c) 2024 juno <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file usbd_fastbootd.c
 * @brief USB composite device: CDC ACM + Fastboot, auto-init via SYS_INIT
 *
 * Enable with CONFIG_USBD_FASTBOOTD=y.
 * Registers Fastboot classes to a shared context.
 * USB descriptor and configuration data is provided by the application
 * via USBD_FASTBOOT_CONFIG_DEFINE() macro, which places a
 * usbd_fastboot_config structure in a dedicated linker section.
 * Application must also provide fastboot_ops via fastboot_register_ops()
 * before usbd_init() is called (use SYS_INIT with lower priority value).
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_fastboot.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_fastbootd, CONFIG_USBD_LOG_LEVEL);

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

static int usbd_fastbootd_init(void)
{
	const struct usbd_fastboot_config *cfg = usbd_fastboot_config_get();
	struct usbd_context *ctx;
	int err;

	if (!cfg) {
		LOG_ERR("No fastboot config defined");
		return -EINVAL;
	}

	ctx = cfg->ctx;

	/* String descriptors */
	usbd_add_descriptor(ctx, cfg->lang);
	usbd_add_descriptor(ctx, cfg->mfr);
	usbd_add_descriptor(ctx, cfg->product);
	if (cfg->sn) {
		usbd_add_descriptor(ctx, cfg->sn);
	}

	/* VBUS callback */
	usbd_msg_register_cb(ctx, bootloader_msg_cb);

	/* Add configs and register classes */
	if (USBD_SUPPORTS_HIGH_SPEED &&
	    usbd_caps_speed(ctx) == USBD_SPEED_HS) {
		err = usbd_add_configuration(ctx, USBD_SPEED_HS,
					     cfg->hs_config);
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
				     cfg->fs_config);
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
