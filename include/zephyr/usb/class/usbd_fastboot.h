/*
 * Copyright (c) 2024
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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_FASTBOOT_H_ */
