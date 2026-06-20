/*
 * Copyright (c) 2024 juno <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Fastboot class public API
 *
 * Implements Android Fastboot protocol for firmware download via USB.
 * The class layer handles USB protocol and command parsing.
 * The application layer provides hardware-specific operations via
 * fastboot_ops callbacks.
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBD_FASTBOOT_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USBD_FASTBOOT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/usb/usbd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fastboot partition information
 */
struct fastboot_part {
	const char *name;
	uint32_t addr;
	uint32_t size;
	uint32_t magic;
	bool internal;
};

/**
 * @brief Fastboot USB device configuration
 *
 * Bundles all USB descriptor nodes and configuration nodes needed
 * by the Fastboot device application layer. Defined by the application
 * using USBD_FASTBOOT_CONFIG_DEFINE and automatically placed in a
 * linker section for discovery by usbd_fastbootd.
 */
struct usbd_fastboot_config {
	/** Pointer to USB device context */
	struct usbd_context *ctx;
	/** Language string descriptor */
	struct usbd_desc_node *lang;
	/** Manufacturer string descriptor */
	struct usbd_desc_node *mfr;
	/** Product string descriptor */
	struct usbd_desc_node *product;
	/** Serial number string descriptor (optional, NULL if unused) */
	struct usbd_desc_node *sn;
	/** FS configuration string descriptor */
	struct usbd_desc_node *fs_cfg_str;
	/** HS configuration string descriptor */
	struct usbd_desc_node *hs_cfg_str;
	/** FS configuration node */
	struct usbd_config_node *fs_config;
	/** HS configuration node */
	struct usbd_config_node *hs_config;
};

/**
 * @brief Define a fastboot_part entry from a DTS fixed-partition node
 *
 * @param node_id  DTS node identifier (e.g. DT_NODELABEL(boot_partition))
 * @param _magic   Firmware magic value for verification
 * @param _internal Whether this partition is on internal flash
 */
#define FASTBOOT_PART_DEFINE(node_id, _magic, _internal) \
	{                                                \
		.name = DT_PROP(node_id, label),         \
		.addr = DT_REG_ADDR(node_id),            \
		.size = DT_REG_SIZE(node_id),            \
		.magic = _magic,                         \
		.internal = _internal,                   \
	}

/**
 * @brief Fastboot application operations
 *
 * Application must implement these callbacks to provide
 * hardware-specific flash and partition operations.
 */
struct fastboot_ops {
	/**
	 * @brief Look up partition by name
	 *
	 * @param name  Partition name (e.g. "spl", "tpl", "app")
	 * @param part  Output: filled with partition info on success
	 * @return 0 on success, negative errno on failure
	 */
	int (*get_partition)(const char *name,
			     struct fastboot_part *part);

	/**
	 * @brief Erase flash region
	 *
	 * @param addr  Start address
	 * @param size  Number of bytes to erase
	 * @return 0 on success, negative errno on failure
	 */
	int (*flash_erase)(uint32_t addr, uint32_t size);

	/**
	 * @brief Write data to flash
	 *
	 * @param addr  Start address
	 * @param data  Data to write
	 * @param size  Number of bytes to write
	 * @return 0 on success, negative errno on failure
	 */
	int (*flash_write)(uint32_t addr,
			   const uint8_t *data, uint32_t size);

	/**
	 * @brief Verify firmware header
	 *
	 * @param data            Firmware data buffer
	 * @param size            Firmware data size
	 * @param expected_magic  Expected magic value for this partition
	 * @return 0 on success, negative errno on failure
	 */
	int (*firmware_verify)(const uint8_t *data, uint32_t size,
			       uint32_t expected_magic);

	/**
	 * @brief Called after successful flash of a partition
	 *
	 * Optional. Can be NULL.
	 *
	 * @param partition  Name of the partition that was flashed
	 */
	void (*on_flash_done)(const char *partition);
};

/**
 * @brief Register application callbacks for Fastboot
 *
 * Must be called before usbd_init().
 *
 * @param ops  Pointer to fastboot_ops (must remain valid indefinitely)
 * @return 0 on success, negative errno on failure
 */
int fastboot_register_ops(const struct fastboot_ops *ops);

/**
 * @brief Register a static partition table
 *
 * Convenience helper for applications that have a simple static
 * partition table. Internally implements get_partition() by
 * searching the table by name.
 *
 * If this is used, the application does not need to provide
 * its own get_partition() callback in fastboot_ops.
 *
 * @param parts      Array of fastboot_part entries
 * @param num_parts  Number of entries in the array
 * @return 0 on success, negative errno on failure
 */
int fastboot_register_partitions(const struct fastboot_part *parts,
				 size_t num_parts);

/**
 * @brief Initialize Fastboot download buffer
 *
 * Optional. If not called, buffer is allocated on first download.
 *
 * @return 0 on success, negative errno on failure
 */
int fastboot_init_download_buf(void);

/**
 * @brief Get the Fastboot USB device configuration
 *
 * Retrieves the usbd_fastboot_config structure placed in the
 * dedicated linker section by USBD_FASTBOOT_CONFIG_DEFINE.
 *
 * @return Pointer to usbd_fastboot_config, or NULL if none defined
 */
const struct usbd_fastboot_config *usbd_fastboot_config_get(void);

/**
 * @brief Define a complete Fastboot USB device configuration
 *
 * Convenience macro that invokes all USBD_*_DEFINE macros and
 * populates a usbd_fastboot_config structure with pointers to
 * the generated static variables. The config structure is
 * automatically placed in a linker section so that
 * usbd_fastboot_config_get() can retrieve it without an
 * explicit registration step.
 *
 * @param cfg_name      Configuration name (used as prefix for all symbols)
 * @param udc_dev       UDC device (e.g. DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)))
 * @param vid           USB Vendor ID
 * @param pid           USB Product ID
 * @param mfr_str       Manufacturer string (ASCII7)
 * @param prod_str      Product string (ASCII7)
 * @param sn_enabled    Whether serial number is enabled (1 or 0)
 * @param fs_cfg_str    FS configuration description string
 * @param hs_cfg_str    HS configuration description string
 * @param self_powered  Self-powered flag (1 or 0)
 * @param max_power     Max power in 2mA units
 */
#define USBD_FASTBOOT_CONFIG_DEFINE(cfg_name, udc_dev, vid, pid,       \
				     mfr_str, prod_str, sn_enabled,     \
				     fs_cfg_str, hs_cfg_str,            \
				     self_powered, max_power)            \
	USBD_DEVICE_DEFINE(cfg_name##_ctx, udc_dev, vid, pid);         \
	USBD_DESC_LANG_DEFINE(cfg_name##_lang);                         \
	USBD_DESC_MANUFACTURER_DEFINE(cfg_name##_mfr, mfr_str);        \
	USBD_DESC_PRODUCT_DEFINE(cfg_name##_product, prod_str);        \
	COND_CODE_1(sn_enabled, (                                       \
		USBD_DESC_SERIAL_NUMBER_DEFINE(cfg_name##_sn);          \
	), ())                                                            \
	USBD_DESC_CONFIG_DEFINE(cfg_name##_fs_cfg_str, fs_cfg_str);    \
	USBD_DESC_CONFIG_DEFINE(cfg_name##_hs_cfg_str, hs_cfg_str);    \
	USBD_CONFIGURATION_DEFINE(cfg_name##_fs_config,                  \
		(self_powered) ? USB_SCD_SELF_POWERED : 0,               \
		max_power, &cfg_name##_fs_cfg_str);                       \
	USBD_CONFIGURATION_DEFINE(cfg_name##_hs_config,                  \
		(self_powered) ? USB_SCD_SELF_POWERED : 0,               \
		max_power, &cfg_name##_hs_cfg_str);                       \
	static STRUCT_SECTION_ITERABLE(usbd_fastboot_config, cfg_name) = {\
		.ctx = &cfg_name##_ctx,                                 \
		.lang = &cfg_name##_lang,                               \
		.mfr = &cfg_name##_mfr,                                 \
		.product = &cfg_name##_product,                         \
		.sn = COND_CODE_1(sn_enabled, (&cfg_name##_sn), (NULL)),\
		.fs_cfg_str = &cfg_name##_fs_cfg_str,                   \
		.hs_cfg_str = &cfg_name##_hs_cfg_str,                   \
		.fs_config = &cfg_name##_fs_config,                     \
		.hs_config = &cfg_name##_hs_config,                     \
	}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_FASTBOOT_H_ */
