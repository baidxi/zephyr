/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal header for the USB Host Mass Storage Class driver.
 *
 * Defines BOT protocol structures (CBW/CSW), SCSI command opcodes,
 * per-LUN and per-device private data structures, and internal APIs
 * shared between the class driver core, BOT transport layer, and
 * SCSI command builders.
 */

#ifndef ZEPHYR_SUBSYS_USB_HOST_CLASS_USBH_MSC_INTERNAL_H_
#define ZEPHYR_SUBSYS_USB_HOST_CLASS_USBH_MSC_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/drivers/usb/uhc.h>

/* ---- USB MSC class / protocol codes ---- */

#define MSC_SCSI_TRANSPARENT_CMD_SET	0x06U
#define MSC_BULK_ONLY_TRANSPORT		0x50U

/* BOT class-specific control requests */
#define MSC_REQ_GET_MAX_LUN		0xFEU
#define MSC_REQ_BOT_RESET		0xFFU

/* Request type for class requests to interface (D2H | Class | Interface) */
#define MSC_REQTYPE_IN_IFACE		0xA1U
/* (H2D | Class | Interface) */
#define MSC_REQTYPE_OUT_IFACE		0x21U

/* ---- CBW / CSW signatures ---- */

#define CBW_SIGNATURE			0x43425355U	/* "USBC" */
#define CSW_SIGNATURE			0x53425355U	/* "USBS" */

/* CSW status codes */
#define CSW_STATUS_PASSED		0x00U
#define CSW_STATUS_FAILED		0x01U
#define CSW_STATUS_PHASE_ERROR		0x02U

/* CBW direction flag (bit 7 of bmCBWFlags) */
#define CBW_FLAGS_DIR_IN		0x80U

/* CBW / CSW wire sizes */
#define CBW_LEN				31U
#define CSW_LEN				13U

/* ---- SCSI operation codes ---- */

#define SCSI_OP_TEST_UNIT_READY		0x00U
#define SCSI_OP_REQUEST_SENSE		0x03U
#define SCSI_OP_INQUIRY			0x12U
#define SCSI_OP_MODE_SENSE_6		0x1AU
#define SCSI_OP_START_STOP_UNIT		0x1BU
#define SCSI_OP_PREVENT_ALLOW_REMOVAL	0x1EU
#define SCSI_OP_READ_CAPACITY_10	0x25U
#define SCSI_OP_READ_10			0x28U
#define SCSI_OP_WRITE_10		0x2AU

/* ---- BOT protocol structures ---- */

/** Command Block Wrapper (31 bytes, sent via Bulk OUT). */
struct msc_cbw {
	uint32_t dCBWSignature;
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t  bmCBWFlags;
	uint8_t  bCBWLUN;
	uint8_t  bCBWCBLength;
	uint8_t  CBWCB[16];
} __packed;

/** Command Status Wrapper (13 bytes, received via Bulk IN). */
struct msc_csw {
	uint32_t dCSWSignature;
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t  bCSWStatus;
} __packed;

/* ---- SCSI response structures ---- */

/** INQUIRY response (standard 36 bytes). */
struct scsi_inquiry_data {
	uint8_t peripheral_device_type;	/* 0x00 = Direct-access block */
	uint8_t rmb_byte;		/* bit7 = removable medium */
	uint8_t version;
	uint8_t response_data_format;
	uint8_t additional_length;
	uint8_t reserved[3];
	uint8_t vendor_id[8];
	uint8_t product_id[16];
	uint8_t product_revision[4];
} __packed;

/** READ_CAPACITY(10) response (8 bytes, big-endian fields). */
struct scsi_read_capacity_data {
	uint8_t max_lba_be[4];
	uint8_t block_size_be[4];
} __packed;

/** REQUEST_SENSE response (18 bytes). */
struct scsi_request_sense_data {
	uint8_t error_code;
	uint8_t reserved1;
	uint8_t sense_key;
	uint8_t information[4];
	uint8_t additional_sense_length;
	uint8_t reserved2[4];
	uint8_t asc;		/* Additional Sense Code */
	uint8_t ascq;		/* Additional Sense Code Qualifier */
	uint8_t fruc;
	uint8_t sense_key_specific[3];
} __packed;

/* ---- Per-LUN state ---- */

enum msc_lun_state {
	MSC_LUN_UNINITIALIZED,
	MSC_LUN_ENUMERATING,
	MSC_LUN_READY,
	MSC_LUN_ERROR,
};

struct msc_lun {
	/** disk_info for disk_access registration. */
	struct disk_info disk;

	/** Back-pointer to parent device (set at enumeration). */
	struct msc_dev *md;

	/** INQUIRY identification. */
	char vendor[9];
	char product[17];
	char revision[5];
	bool removable;
	bool read_only;

	/** Capacity from READ_CAPACITY(10). */
	uint32_t sector_count;
	uint32_t sector_size;

	/** State. */
	enum msc_lun_state state;
	uint8_t lun_idx;

	/** Disk name storage (disk_info.name points here). */
	char disk_name[16];
};

/* ---- Per-device MSC private data ---- */

struct msc_dev {
	bool active;
	struct usb_device *udev;
	struct usbh_class_data *c_data;
	uint8_t iface;
	uint8_t ep_in;		/* Bulk IN endpoint address */
	uint8_t ep_out;		/* Bulk OUT endpoint address */
	uint16_t mps_in;	/* Bulk IN max packet size */
	uint16_t mps_out;	/* Bulk OUT max packet size */
	uint8_t max_lun;	/* Maximum LUN index from GET_MAX_LUN */

	/**
	 * Persisted USB data-toggle bits for the two bulk endpoints.
	 * The EHCI saves the next-expected toggle into uhc_transfer
	 * .data_toggle after each transfer, but each BOT exchange
	 * allocates a fresh uhc_transfer — so we must stash the toggle
	 * here and restore it on the next transfer.  Reset to 0 by
	 * ClearFeature(ENDPOINT_HALT) / BOT Reset.
	 */
	uint8_t toggle_in;
	uint8_t toggle_out;

	struct msc_lun luns[CONFIG_USBH_MSC_MAX_LUNS];

	/**
	 * Learned CSW signature.  The first valid CSW's signature is
	 * stored here; subsequent CSWs must match.  This tolerates
	 * non-compliant devices that return a bogus signature.
	 * 0 means "not yet learned".
	 */
	uint32_t csw_signature;

	/** CBW tag counter (incremented per command). */
	uint32_t next_tag;

	/**
	 * Synchronization for the synchronous BOT command pipeline.
	 * The transfer completion callback gives this semaphore; the
	 * submitting thread takes it.
	 */
	struct k_sem xfer_done;

	/** Result of the last transfer (set by callback). */
	int xfer_err;
	/** Actual bytes transferred (set by callback for Bulk IN). */
	size_t xfer_actual;
	/** Pending transfer pointer (for cleanup on removal). */
	struct uhc_transfer *pending_xfer;

	/** Enumeration work item. */
	struct k_work_delayable enumerate_work;
	bool enumerating;
};

/* ---- Internal API: CBW / CSW helpers ---- */

/**
 * Build a Command Block Wrapper.
 *
 * @param cbw          Destination CBW (must be zeroed or will be overwritten).
 * @param tag          CBW tag (monotonic counter from msc_dev.next_tag).
 * @param lun          Logical Unit Number (bits 0-3).
 * @param cdb          Command Descriptor Block data (up to 16 bytes).
 * @param cdb_len      Valid length of CDB (1-16).
 * @param transfer_len Expected data-phase length in bytes (0 = no data).
 * @param dir_in       true for device-to-host (IN), false for host-to-device.
 */
void msc_cbw_init(struct msc_cbw *cbw, uint32_t tag, uint8_t lun,
		  const uint8_t *cdb, uint8_t cdb_len,
		  uint32_t transfer_len, bool dir_in);

/**
 * Validate a Command Status Wrapper.
 *
 * @param csw          CSW to validate.
 * @param expected_tag Tag that was sent in the corresponding CBW.
 * @param dev_sig      Pointer to learned-signature (0 = not yet learned,
 *                     will be updated on first valid CSW).
 * @param status       Output: CSW_STATUS_PASSED / FAILED / PHASE_ERROR.
 *
 * @return 0 if CSW is structurally valid (signature/tag match),
 *         -EIO otherwise.
 */
int msc_csw_validate(const struct msc_csw *csw, uint32_t expected_tag,
		     uint32_t *dev_sig, uint8_t *status);

/* ---- Internal API: SCSI CDB builders ---- */

int msc_scsi_inquiry(uint8_t *cdb);
int msc_scsi_read_capacity(uint8_t *cdb);
int msc_scsi_read10(uint8_t *cdb, uint32_t lba, uint16_t num_sectors);
int msc_scsi_write10(uint8_t *cdb, uint32_t lba, uint16_t num_sectors);
int msc_scsi_test_unit_ready(uint8_t *cdb);
int msc_scsi_request_sense(uint8_t *cdb);
int msc_scsi_mode_sense_6(uint8_t *cdb);
int msc_scsi_prevent_allow_removal(uint8_t *cdb, bool prevent);
int msc_scsi_start_stop(uint8_t *cdb, bool start, bool load_eject);

/* ---- Internal API: BOT transport ---- */

/**
 * Execute one BOT command synchronously (CBW → optional data → CSW).
 *
 * @param md         MSC device private data.
 * @param lun        Target LUN.
 * @param cdb        Command Descriptor Block (up to 16 bytes).
 * @param cdb_len    Valid CDB length.
 * @param dir_in     true for IN data phase, false for OUT, irrelevant if
 *                   data_len == 0.
 * @param data_buf   Data buffer for the data phase (may be NULL if
 *                   data_len == 0).  For IN this receives data, for OUT
 *                   it provides data to send.
 * @param data_len   Data-phase length in bytes (0 = no data phase).
 * @param actual_len Output: actual bytes transferred in data phase.
 *
 * @return 0 on success, negative errno on error.
 */
int msc_bot_execute(struct msc_dev *md, uint8_t lun,
		    const uint8_t *cdb, uint8_t cdb_len,
		    bool dir_in,
		    uint8_t *data_buf, uint32_t data_len,
		    uint32_t *actual_len);

/** Perform BOT Reset (class request) + clear-halt on both bulk endpoints. */
int msc_bot_reset(struct msc_dev *md);

/** Get Max LUN via class request.  Returns LUN index (0 on failure). */
uint8_t msc_get_max_lun(struct msc_dev *md);

#endif /* ZEPHYR_SUBSYS_USB_HOST_CLASS_USBH_MSC_INTERNAL_H_ */
