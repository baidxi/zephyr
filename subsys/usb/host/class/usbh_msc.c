/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Host Mass Storage Class (MSC) driver for the Zephyr USB host stack.
 *
 * Supports USB mass-storage devices using the Bulk-Only Transport (BOT)
 * protocol with the SCSI transparent command set (subclass 0x06 /
 * protocol 0x50).  When a compatible device is connected, the driver:
 *
 *   1. Issues a BOT Reset and queries Get Max LUN.
 *   2. For each LUN: performs INQUIRY, waits for TEST_UNIT_READY,
 *      issues READ_CAPACITY(10), and checks write protection via
 *      MODE_SENSE(6).
 *   3. Registers the LUN as a Zephyr disk device through disk_access.
 *
 * File-system layers (FATFS, LittleFS) can then mount the disk and
 * perform read/write operations transparently via disk_operations.
 *
 * The driver uses a synchronous execution model: each BOT command
 * (CBW → data → CSW) is driven by the calling thread, which blocks on
 * a semaphore until the bulk transfer completion callback fires.  This
 * keeps the first implementation simple while remaining functionally
 * equivalent to Linux's blocking URB model.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/disk.h>

#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usbh_msc.h>

#include "usbh_msc_internal.h"
#include "../usbh_device.h"
#include "../usbh_ch9.h"
#include "../usbh_class.h"
#include "../usbh_desc.h"

LOG_MODULE_REGISTER(usbh_msc);

#define USBH_MSC_MAX_DEVICES	CONFIG_USBH_MSC_MAX_DEVICES

/* Transfer timeout for BOT phases (ms). */
#define MSC_XFER_TIMEOUT_MS	5000U

/* Maximum retries for TEST_UNIT_READY during enumeration. */
#define MSC_TUR_RETRIES		20U
#define MSC_TUR_RETRY_DELAY_MS	200U

/* INQUIRY standard response length. */
#define INQUIRY_RESP_LEN	36U
/* REQUEST_SENSE response length. */
#define REQUEST_SENSE_RESP_LEN	18U
/* READ_CAPACITY(10) response length. */
#define READ_CAPACITY_RESP_LEN	8U
/* MODE_SENSE(6) allocation length. */
#define MODE_SENSE_RESP_LEN	64U

/* Maximum single bulk data transfer (bytes).  Chunks larger than this
 * are split into multiple URBs to avoid over-sized net_buf allocations. */
#define MSC_MAX_BULK_XFER	(CONFIG_USBH_MSC_MAX_XFER_SIZE)

/* ---- Static device pool ---- */

static struct msc_dev msc_devs[USBH_MSC_MAX_DEVICES];

/* ---- Application callback ---- */

static usbh_msc_event_cb_t msc_event_cb;
static void *msc_event_cb_user_data;

void usbh_msc_register_event_cb(usbh_msc_event_cb_t cb, void *user_data)
{
	msc_event_cb = cb;
	msc_event_cb_user_data = user_data;
}

static void msc_notify_event(struct usb_device *udev, uint8_t lun,
			     const char *disk_name, bool connected)
{
	if (msc_event_cb != NULL) {
		msc_event_cb(udev, lun, disk_name, connected,
			     msc_event_cb_user_data);
	}
}

/* ---- Device slot management ---- */

static struct msc_dev *msc_dev_alloc(void)
{
	for (int i = 0; i < USBH_MSC_MAX_DEVICES; i++) {
		if (!msc_devs[i].active) {
			memset(&msc_devs[i], 0, sizeof(msc_devs[i]));
			msc_devs[i].active = true;
			k_sem_init(&msc_devs[i].xfer_done, 0, 1);
			return &msc_devs[i];
		}
	}

	return NULL;
}

/* ================================================================
 * CBW / CSW helpers
 * ================================================================ */

void msc_cbw_init(struct msc_cbw *cbw, uint32_t tag, uint8_t lun,
		  const uint8_t *cdb, uint8_t cdb_len,
		  uint32_t transfer_len, bool dir_in)
{
	memset(cbw, 0, sizeof(*cbw));

	cbw->dCBWSignature = CBW_SIGNATURE;
	cbw->dCBWTag = tag;
	cbw->dCBWDataTransferLength = transfer_len;
	cbw->bmCBWFlags = dir_in ? CBW_FLAGS_DIR_IN : 0U;
	cbw->bCBWLUN = lun & 0x0FU;
	cbw->bCBWCBLength = MIN(cdb_len, 16U);

	if (cdb_len > 0U && cdb != NULL) {
		memcpy(cbw->CBWCB, cdb, MIN(cdb_len, 16U));
	}
}

int msc_csw_validate(const struct msc_csw *csw, uint32_t expected_tag,
		     uint32_t *dev_sig, uint8_t *status)
{
	uint32_t sig = csw->dCSWSignature;

	/*
	 * Linux-style signature learning: accept any signature on the
	 * first valid CSW, then enforce consistency on subsequent ones.
	 */
	if (*dev_sig == 0U) {
		*dev_sig = sig;
		LOG_DBG("Learned CSW signature 0x%08x", sig);
	} else if (sig != *dev_sig) {
		LOG_ERR("CSW signature mismatch: got 0x%08x expected 0x%08x",
			sig, *dev_sig);
		return -EIO;
	}

	if (csw->dCSWTag != expected_tag) {
		LOG_ERR("CSW tag mismatch: got %u expected %u",
			csw->dCSWTag, expected_tag);
		return -EIO;
	}

	if (status != NULL) {
		*status = csw->bCSWStatus;
	}

	return 0;
}

/* ================================================================
 * SCSI CDB builders
 * ================================================================
 *
 * Each builder fills a 16-byte CDB buffer (zeroed first) and returns
 * the valid CDB length.  Multi-byte integer fields use SCSI big-endian
 * ordering.
 */

int msc_scsi_inquiry(uint8_t *cdb)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_INQUIRY;
	cdb[4] = INQUIRY_RESP_LEN;	/* allocation length */
	return 6;
}

int msc_scsi_read_capacity(uint8_t *cdb)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_READ_CAPACITY_10;
	return 10;
}

int msc_scsi_read10(uint8_t *cdb, uint32_t lba, uint16_t num_sectors)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_READ_10;
	sys_put_be32(lba, &cdb[2]);		/* LBA (big-endian) */
	sys_put_be16(num_sectors, &cdb[7]);	/* transfer length */
	return 10;
}

int msc_scsi_write10(uint8_t *cdb, uint32_t lba, uint16_t num_sectors)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_WRITE_10;
	sys_put_be32(lba, &cdb[2]);
	sys_put_be16(num_sectors, &cdb[7]);
	return 10;
}

int msc_scsi_test_unit_ready(uint8_t *cdb)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_TEST_UNIT_READY;
	return 6;
}

int msc_scsi_request_sense(uint8_t *cdb)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_REQUEST_SENSE;
	cdb[4] = REQUEST_SENSE_RESP_LEN;
	return 6;
}

int msc_scsi_mode_sense_6(uint8_t *cdb)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_MODE_SENSE_6;
	cdb[2] = 0x3FU;		/* page code: return all pages */
	cdb[4] = MODE_SENSE_RESP_LEN;
	return 6;
}

int msc_scsi_prevent_allow_removal(uint8_t *cdb, bool prevent)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_PREVENT_ALLOW_REMOVAL;
	cdb[4] = prevent ? 0x01U : 0x00U;
	return 6;
}

int msc_scsi_start_stop(uint8_t *cdb, bool start, bool load_eject)
{
	memset(cdb, 0, 16);
	cdb[0] = SCSI_OP_START_STOP_UNIT;
	cdb[4] = (uint8_t)((start ? BIT(0) : 0U) | (load_eject ? BIT(1) : 0U));
	return 6;
}

/* ================================================================
 * Bulk transfer completion callback
 * ================================================================ */

static int msc_bulk_xfer_cb(struct usb_device *const udev,
			    struct uhc_transfer *const xfer)
{
	struct msc_dev *md = xfer->priv;

	if (md == NULL) {
		return 0;
	}

	/* Record the result for the waiting thread. */
	md->xfer_err = xfer->err;

	if (xfer->err == 0 && xfer->buf != NULL) {
		md->xfer_actual = xfer->buf->len;
	} else {
		md->xfer_actual = 0;
	}

	/* Persist the data-toggle bit so the next transfer on this
	 * endpoint continues the correct DATA0/DATA1 sequence.
	 * The EHCI completion handler writes the next-expected toggle
	 * into xfer->data_toggle; we stash it per-endpoint because each
	 * BOT exchange allocates a fresh uhc_transfer. */
	if (xfer->err == 0) {
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			md->toggle_in = xfer->data_toggle ? 1U : 0U;
		} else {
			md->toggle_out = xfer->data_toggle ? 1U : 0U;
		}
	}

	md->pending_xfer = NULL;

	/* Wake up the submitting thread. */
	k_sem_give(&md->xfer_done);

	return 0;
}

/**
 * Submit a single bulk transfer and wait synchronously for completion.
 *
 * @param md      MSC device.
 * @param ep      Endpoint address (IN or OUT).
 * @param buf     net_buf carrying OUT data, or empty buffer for IN.
 * @param is_in   true for IN transfer.
 *
 * @return 0 on success, negative errno on error/timeout.
 */
static int msc_bulk_xfer_sync(struct msc_dev *md, uint8_t ep,
			      struct net_buf *buf, bool is_in)
{
	struct uhc_transfer *xfer;
	int ret;

	/* Pass md directly as the callback private data. */
	xfer = usbh_xfer_alloc(md->udev, ep, msc_bulk_xfer_cb, md);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	xfer->type = USB_EP_TYPE_BULK;
	xfer->mps = is_in ? md->mps_in : md->mps_out;
	/* Restore the persisted data-toggle so this transfer continues
	 * the correct DATA0/DATA1 sequence from the previous transfer
	 * on the same endpoint. */
	xfer->data_toggle = is_in ? md->toggle_in : md->toggle_out;

	ret = usbh_xfer_buf_add(md->udev, xfer, buf);
	if (ret != 0) {
		usbh_xfer_free(md->udev, xfer);
		return ret;
	}

	md->xfer_err = 0;
	md->xfer_actual = 0;
	md->pending_xfer = xfer;

	ret = usbh_xfer_enqueue(md->udev, xfer);
	if (ret != 0) {
		md->pending_xfer = NULL;
		usbh_xfer_free(md->udev, xfer);
		return ret;
	}

	/* Wait for completion. */
	ret = k_sem_take(&md->xfer_done, K_MSEC(MSC_XFER_TIMEOUT_MS));
	if (ret != 0) {
		LOG_ERR("Bulk %s timeout (ep=0x%02x)",
			is_in ? "IN" : "OUT", ep);
		/* Attempt to dequeue the stuck transfer. */
		(void)usbh_xfer_dequeue(md->udev, xfer);
		md->pending_xfer = NULL;
		/*
		 * Cannot free xfer here safely if the HCD still owns it.
		 * It will be cleaned up when the device is removed.
		 */
		return -ETIMEDOUT;
	}

	/* Free the transfer wrapper; the net_buf is managed by the caller. */
	md->pending_xfer = NULL;
	usbh_xfer_free(md->udev, xfer);

	return md->xfer_err;
}

/* ================================================================
 * BOT transport layer
 * ================================================================ */

/**
 * Send CBW over Bulk OUT.
 */
static int msc_send_cbw(struct msc_dev *md, const struct msc_cbw *cbw)
{
	struct net_buf *buf;
	int ret;

	buf = usbh_xfer_buf_alloc(md->udev, CBW_LEN);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_mem(buf, cbw, CBW_LEN);

	ret = msc_bulk_xfer_sync(md, md->ep_out, buf, false);

	usbh_xfer_buf_free(md->udev, buf);
	return ret;
}

/**
 * Receive CSW over Bulk IN, with STALL-retry and ZLP-skip handling.
 */
static int msc_recv_csw(struct msc_dev *md, struct msc_csw *csw)
{
	struct net_buf *buf;
	int ret;
	int retry;

	for (retry = 0; retry < 2; retry++) {
		buf = usbh_xfer_buf_alloc(md->udev, CSW_LEN);
		if (buf == NULL) {
			return -ENOMEM;
		}

		ret = msc_bulk_xfer_sync(md, md->ep_in, buf, true);

		if (ret == 0 && buf->len == CSW_LEN) {
			/* Got a complete CSW. */
			memcpy(csw, buf->data, CSW_LEN);
			usbh_xfer_buf_free(md->udev, buf);
			return 0;
		}

		if (ret == 0 && buf->len == 0U) {
			/*
			 * Zero-length packet: some devices emit a ZLP
			 * before the real CSW.  Retry once.
			 */
			LOG_WRN("CSW: got ZLP, retrying");
			usbh_xfer_buf_free(md->udev, buf);
			continue;
		}

		/*
		 * Transfer error or short read.  If the endpoint stalled,
		 * clear the halt and retry once (Linux pattern).
		 */
		LOG_WRN("CSW recv error %d (len=%u), clearing halt",
			ret, buf != NULL ? buf->len : 0U);
		usbh_xfer_buf_free(md->udev, buf);

		if (retry == 0) {
			(void)usbh_req_clear_sfs_halt(md->udev, md->ep_in);
			md->toggle_in = 0U;
			continue;
		}

		return ret != 0 ? ret : -EIO;
	}

	return -EIO;
}

int msc_bot_execute(struct msc_dev *md, uint8_t lun,
		    const uint8_t *cdb, uint8_t cdb_len,
		    bool dir_in,
		    uint8_t *data_buf, uint32_t data_len,
		    uint32_t *actual_len)
{
	struct msc_cbw cbw;
	struct msc_csw csw;
	uint32_t tag;
	uint32_t transferred = 0U;
	uint8_t status;
	int ret;

	if (!md->active) {
		return -ENODEV;
	}

	tag = md->next_tag++;

	msc_cbw_init(&cbw, tag, lun, cdb, cdb_len, data_len, dir_in);

	/* Phase 1: send CBW. */
	ret = msc_send_cbw(md, &cbw);
	if (ret != 0) {
		LOG_ERR("CBW send failed: %d", ret);
		(void)usbh_req_clear_sfs_halt(md->udev, md->ep_out);
		md->toggle_out = 0U;
		return ret;
	}

	/* Phase 2: optional data transfer. */
	if (data_len > 0U && data_buf != NULL) {
		uint32_t remaining = data_len;
		uint32_t offset = 0U;

		while (remaining > 0U && md->active) {
			struct net_buf *dbuf;
			uint32_t chunk = MIN(remaining, MSC_MAX_BULK_XFER);
			int dret;

			dbuf = usbh_xfer_buf_alloc(md->udev, chunk);
			if (dbuf == NULL) {
				ret = -ENOMEM;
				break;
			}

			if (dir_in) {
				/* IN: receive data. */
				dret = msc_bulk_xfer_sync(md, md->ep_in,
							  dbuf, true);
				if (dret == 0) {
					uint16_t got = dbuf->len;

					memcpy(data_buf + offset,
					       dbuf->data, got);
					transferred += got;
					offset += got;
					remaining -= got;
					if (got < chunk) {
						/* Short read — device
						 * signalled end of data. */
						usbh_xfer_buf_free(
							md->udev, dbuf);
						break;
					}
				}
			} else {
				/* OUT: send data. */
				net_buf_add_mem(dbuf, data_buf + offset,
						chunk);
				dret = msc_bulk_xfer_sync(md, md->ep_out,
							  dbuf, false);
				if (dret == 0) {
					transferred += chunk;
					offset += chunk;
					remaining -= chunk;
				}
			}

			usbh_xfer_buf_free(md->udev, dbuf);

			if (dret != 0) {
				LOG_ERR("Data phase %s error: %d",
					dir_in ? "IN" : "OUT", dret);
				if (dir_in) {
					(void)usbh_req_clear_sfs_halt(
						md->udev, md->ep_in);
					md->toggle_in = 0U;
				} else {
					(void)usbh_req_clear_sfs_halt(
						md->udev, md->ep_out);
					md->toggle_out = 0U;
				}
				/* Still try to read CSW for error recovery. */
				break;
			}
		}
	}

	/* Phase 3: receive CSW. */
	ret = msc_recv_csw(md, &csw);
	if (ret != 0) {
		LOG_ERR("CSW recv failed: %d", ret);
		return ret;
	}

	if (actual_len != NULL) {
		*actual_len = transferred;
	}

	/* Validate CSW. */
	ret = msc_csw_validate(&csw, tag, &md->csw_signature, &status);
	if (ret != 0) {
		return ret;
	}

	switch (status) {
	case CSW_STATUS_PASSED:
		return 0;

	case CSW_STATUS_FAILED:
		LOG_DBG("CSW FAILED (opcode=0x%02x lun=%u)", cdb[0], lun);
		return -EIO;

	case CSW_STATUS_PHASE_ERROR:
		LOG_ERR("CSW PHASE ERROR (opcode=0x%02x)", cdb[0]);
		(void)msc_bot_reset(md);
		return -EIO;

	default:
		LOG_ERR("CSW unknown status 0x%02x", status);
		return -EIO;
	}
}

int msc_bot_reset(struct msc_dev *md)
{
	int ret;

	LOG_INF("BOT Reset (iface=%u)", md->iface);

	ret = usbh_req_setup(md->udev,
			     MSC_REQTYPE_OUT_IFACE, MSC_REQ_BOT_RESET,
			     0, md->iface, 0, NULL);
	if (ret != 0) {
		LOG_WRN("BOT Reset request failed: %d", ret);
	}

	/* Clear halt on both bulk endpoints after reset.
	 * ClearFeature(ENDPOINT_HALT) resets the device-side data toggle
	 * to DATA0; we must also reset our host-side tracking. */
	(void)usbh_req_clear_sfs_halt(md->udev, md->ep_in);
	(void)usbh_req_clear_sfs_halt(md->udev, md->ep_out);
	md->toggle_in = 0U;
	md->toggle_out = 0U;

	return ret;
}

uint8_t msc_get_max_lun(struct msc_dev *md)
{
	struct net_buf *buf;
	uint8_t max_lun = 0U;
	int ret;

	buf = usbh_xfer_buf_alloc(md->udev, 1U);
	if (buf == NULL) {
		return 0U;
	}

	ret = usbh_req_setup(md->udev,
			     MSC_REQTYPE_IN_IFACE, MSC_REQ_GET_MAX_LUN,
			     0, md->iface, 1, buf);
	if (ret != 0 || buf->len < 1U) {
		/*
		 * Many devices STALL this request or return an error;
		 * default to single LUN (0) per Linux behavior.
		 */
		LOG_WRN("Get Max LUN failed: %d, assuming 1 LUN", ret);
		max_lun = 0U;
	} else {
		max_lun = buf->data[0];
		if (max_lun >= CONFIG_USBH_MSC_MAX_LUNS) {
			LOG_WRN("Max LUN %u exceeds configured max %u, clamping",
				max_lun, CONFIG_USBH_MSC_MAX_LUNS - 1U);
			max_lun = CONFIG_USBH_MSC_MAX_LUNS - 1U;
		}
		LOG_INF("Get Max LUN = %u", max_lun);
	}

	usbh_xfer_buf_free(md->udev, buf);
	return max_lun;
}

/* ================================================================
 * Endpoint discovery
 * ================================================================ */

/**
 * Scan the endpoints following an interface descriptor to find the
 * Bulk IN and Bulk OUT endpoints.
 */
static int msc_find_bulk_eps(const struct usb_if_descriptor *if_desc,
			     uint8_t *ep_in, uint16_t *mps_in,
			     uint8_t *ep_out, uint16_t *mps_out)
{
	const struct usb_desc_header *head = (const void *)if_desc;
	uint8_t ep_count = if_desc->bNumEndpoints;
	uint8_t found = 0;

	while (found < ep_count) {
		head = usbh_desc_get_next(head);
		if (head == NULL) {
			break;
		}

		/* Stop at the next interface descriptor. */
		if (head->bDescriptorType == USB_DESC_INTERFACE) {
			break;
		}

		if (usbh_desc_is_valid_endpoint(head)) {
			const struct usb_ep_descriptor *ep = (const void *)head;

			found++;
			if ((ep->bmAttributes & 0x03U) != USB_EP_TYPE_BULK) {
				continue;
			}

			if (USB_EP_DIR_IS_IN(ep->bEndpointAddress)) {
				*ep_in = ep->bEndpointAddress;
				*mps_in = sys_le16_to_cpu(ep->wMaxPacketSize);
			} else {
				*ep_out = ep->bEndpointAddress;
				*mps_out = sys_le16_to_cpu(ep->wMaxPacketSize);
			}
		}
	}

	if (*ep_in == 0U || *ep_out == 0U) {
		return -ENOTSUP;
	}

	return 0;
}

/* ================================================================
 * High-level SCSI command wrappers
 * ================================================================ */

/**
 * Perform INQUIRY and populate LUN identity fields.
 */
static int msc_do_inquiry(struct msc_dev *md, struct msc_lun *lun)
{
	uint8_t cdb[16];
	uint8_t resp[INQUIRY_RESP_LEN];
	uint32_t actual = 0U;
	int ret;
	int len;

	len = msc_scsi_inquiry(cdb);
	ret = msc_bot_execute(md, lun->lun_idx, cdb, len, true,
			      resp, sizeof(resp), &actual);
	if (ret != 0) {
		return ret;
	}

	/* Parse standard inquiry response. */
	lun->removable = (resp[1] & BIT(7)) != 0U;

	memcpy(lun->vendor, &resp[8], 8);
	lun->vendor[8] = '\0';
	memcpy(lun->product, &resp[16], 16);
	lun->product[16] = '\0';
	memcpy(lun->revision, &resp[32], 4);
	lun->revision[4] = '\0';

	LOG_INF("LUN %u: \"%s\" \"%s\" rev %s (type=0x%02x%s)",
		lun->lun_idx, lun->vendor, lun->product,
		lun->revision, resp[0],
		lun->removable ? " removable" : "");

	return 0;
}

/**
 * Send TEST_UNIT_READY, retrying with delays.
 */
static int msc_do_test_unit_ready(struct msc_dev *md, struct msc_lun *lun)
{
	uint8_t cdb[16];
	int len;
	int ret;

	for (int i = 0; i < MSC_TUR_RETRIES; i++) {
		len = msc_scsi_test_unit_ready(cdb);
		ret = msc_bot_execute(md, lun->lun_idx, cdb, len,
				      false, NULL, 0, NULL);
		if (ret == 0) {
			return 0;
		}

		/* Issue REQUEST_SENSE to clear the CHECK_CONDITION. */
		uint8_t sense_cdb[16];
		uint8_t sense[REQUEST_SENSE_RESP_LEN];
		uint32_t sense_actual = 0U;
		int slen = msc_scsi_request_sense(sense_cdb);

		(void)msc_bot_execute(md, lun->lun_idx, sense_cdb, slen,
				      true, sense, sizeof(sense),
				      &sense_actual);

		if (sense_actual >= 3U) {
			LOG_DBG("TUR retry %d: sense key 0x%02x asc 0x%02x ascq 0x%02x",
				i, sense[2] & 0x0FU, sense[12], sense[13]);
		}

		k_sleep(K_MSEC(MSC_TUR_RETRY_DELAY_MS));
	}

	return -EIO;
}

/**
 * Perform READ_CAPACITY(10) and populate sector_count / sector_size.
 */
static int msc_do_read_capacity(struct msc_dev *md, struct msc_lun *lun)
{
	uint8_t cdb[16];
	uint8_t resp[READ_CAPACITY_RESP_LEN];
	uint32_t actual = 0U;
	int ret;
	int len;

	len = msc_scsi_read_capacity(cdb);
	ret = msc_bot_execute(md, lun->lun_idx, cdb, len, true,
			      resp, sizeof(resp), &actual);
	if (ret != 0) {
		return ret;
	}

	if (actual < READ_CAPACITY_RESP_LEN) {
		LOG_ERR("READ_CAPACITY short response: %u", actual);
		return -EIO;
	}

	lun->sector_count = sys_get_be32(resp) + 1U; /* max_lba is 0-based */
	lun->sector_size = sys_get_be32(&resp[4]);

	LOG_INF("LUN %u: %u sectors x %u bytes (%llu MB)",
		lun->lun_idx, lun->sector_count, lun->sector_size,
		((uint64_t)lun->sector_count * lun->sector_size) /
			(1024U * 1024U));

	return 0;
}

/**
 * Check write-protection via MODE_SENSE(6).
 */
static void msc_do_check_wp(struct msc_dev *md, struct msc_lun *lun)
{
	uint8_t cdb[16];
	uint8_t resp[MODE_SENSE_RESP_LEN];
	uint32_t actual = 0U;
	int len;

	len = msc_scsi_mode_sense_6(cdb);
	lun->read_only = false;

	if (msc_bot_execute(md, lun->lun_idx, cdb, len, true,
			    resp, sizeof(resp), &actual) == 0) {
		/*
		 * Mode Parameter Header (6-byte): byte 1, bit 7 is
		 * WP (Write Protect).
		 */
		if (actual >= 2U) {
			lun->read_only = (resp[1] & BIT(7)) != 0U;
		}
	}

	if (lun->read_only) {
		LOG_INF("LUN %u: write-protected", lun->lun_idx);
	}
}

/* ================================================================
 * disk_operations implementation
 * ================================================================ */

static int msc_disk_init(struct disk_info *disk)
{
	/* Nothing extra to do; the LUN is already enumerated. */
	ARG_UNUSED(disk);
	return 0;
}

static int msc_disk_status(struct disk_info *disk)
{
	struct msc_lun *lun = CONTAINER_OF(disk, struct msc_lun, disk);

	if (!lun->md->active || lun->state != MSC_LUN_READY) {
		return DISK_STATUS_UNINIT;
	}

	if (lun->read_only) {
		return DISK_STATUS_WR_PROTECT;
	}

	return DISK_STATUS_OK;
}

static int msc_disk_read(struct disk_info *disk, uint8_t *data_buf,
			 uint32_t start_sector, uint32_t num_sector)
{
	struct msc_lun *lun = CONTAINER_OF(disk, struct msc_lun, disk);
	struct msc_dev *md = lun->md;
	uint8_t cdb[16];
	int len;
	int ret;

	if (!md->active) {
		return -ENODEV;
	}

	if (num_sector == 0U) {
		return 0;
	}

	/*
		* READ(10) supports up to 65535 blocks per CDB.  Split if
		* the request exceeds that.
		*/
	while (num_sector > 0U) {
		uint16_t blocks = (uint16_t)MIN(num_sector, 0xFFFFU);
		uint32_t xfer_bytes = (uint32_t)blocks * lun->sector_size;
		uint32_t got = 0U;

		len = msc_scsi_read10(cdb, start_sector, blocks);
		ret = msc_bot_execute(md, lun->lun_idx, cdb, len, true,
			       data_buf, xfer_bytes, &got);
		if (ret != 0) {
			LOG_ERR("READ(10) lba=%u n=%u failed: %d",
			 start_sector, blocks, ret);
			return ret;
		}

		data_buf += xfer_bytes;
		start_sector += blocks;
		num_sector -= blocks;
	}

	return 0;
}

static int msc_disk_write(struct disk_info *disk, const uint8_t *data_buf,
			  uint32_t start_sector, uint32_t num_sector)
{
	struct msc_lun *lun = CONTAINER_OF(disk, struct msc_lun, disk);
	struct msc_dev *md = lun->md;
	uint8_t cdb[16];
	int len;
	int ret;

	if (!md->active) {
		return -ENODEV;
	}

	if (num_sector == 0U) {
		return 0;
	}

	if (lun->read_only) {
		return -EACCES;
	}

	while (num_sector > 0U) {
		uint16_t blocks = (uint16_t)MIN(num_sector, 0xFFFFU);
		uint32_t xfer_bytes = (uint32_t)blocks * lun->sector_size;
		uint32_t sent = 0U;

		len = msc_scsi_write10(cdb, start_sector, blocks);
		ret = msc_bot_execute(md, lun->lun_idx, cdb, len, false,
				      (uint8_t *)data_buf, xfer_bytes, &sent);
		if (ret != 0) {
			LOG_ERR("WRITE(10) lba=%u n=%u failed: %d",
				start_sector, blocks, ret);
			return ret;
		}

		data_buf += xfer_bytes;
		start_sector += blocks;
		num_sector -= blocks;
	}

	return 0;
}

static int msc_disk_erase(struct disk_info *disk, uint32_t start_sector,
			  uint32_t num_sector)
{
	/* USB mass storage devices handle erase internally; nothing to do. */
	ARG_UNUSED(disk);
	ARG_UNUSED(start_sector);
	ARG_UNUSED(num_sector);
	return 0;
}

static int msc_disk_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	struct msc_lun *lun = CONTAINER_OF(disk, struct msc_lun, disk);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		if (buff != NULL) {
			*(uint32_t *)buff = lun->sector_count;
		}
		return 0;

	case DISK_IOCTL_GET_SECTOR_SIZE:
		if (buff != NULL) {
			*(uint32_t *)buff = lun->sector_size;
		}
		return 0;

	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		if (buff != NULL) {
			*(uint32_t *)buff = 1U;
		}
		return 0;

	case DISK_IOCTL_CTRL_SYNC:
		/* BOT writes are synchronous; no flush needed. */
		return 0;

	case DISK_IOCTL_CTRL_INIT:
		return 0;

	case DISK_IOCTL_CTRL_DEINIT:
		return 0;

	default:
		return -ENOTSUP;
	}
}

static const struct disk_operations msc_disk_ops = {
	.init   = msc_disk_init,
	.status = msc_disk_status,
	.read   = msc_disk_read,
	.write  = msc_disk_write,
	.erase  = msc_disk_erase,
	.ioctl  = msc_disk_ioctl,
};

/* ================================================================
 * Enumeration
 * ================================================================ */

static void msc_enumerate_lun(struct msc_dev *md, uint8_t lun_idx)
{
	struct msc_lun *lun = &md->luns[lun_idx];
	int ret;

	lun->md = md;
	lun->lun_idx = lun_idx;
	lun->state = MSC_LUN_ENUMERATING;

	/* INQUIRY. */
	ret = msc_do_inquiry(md, lun);
	if (ret != 0) {
		LOG_ERR("LUN %u INQUIRY failed: %d", lun_idx, ret);
		lun->state = MSC_LUN_ERROR;
		return;
	}

	/* Wait for unit ready. */
	ret = msc_do_test_unit_ready(md, lun);
	if (ret != 0) {
		LOG_ERR("LUN %u not ready: %d", lun_idx, ret);
		lun->state = MSC_LUN_ERROR;
		return;
	}

	/* Read capacity. */
	ret = msc_do_read_capacity(md, lun);
	if (ret != 0) {
		LOG_ERR("LUN %u READ_CAPACITY failed: %d", lun_idx, ret);
		lun->state = MSC_LUN_ERROR;
		return;
	}

	/* Check write protection. */
	msc_do_check_wp(md, lun);

	/* Register the disk. */
	snprintk(lun->disk_name, sizeof(lun->disk_name),
		 "USB_MSC_%u_%u", md->udev->addr, lun_idx);

	lun->disk.name = lun->disk_name;
	lun->disk.ops = &msc_disk_ops;
	lun->disk.dev = NULL;

	ret = disk_access_register(&lun->disk);
	if (ret != 0) {
		LOG_ERR("LUN %u disk registration failed: %d",
			lun_idx, ret);
		lun->state = MSC_LUN_ERROR;
		return;
	}

	lun->state = MSC_LUN_READY;
	LOG_INF("LUN %u registered as disk \"%s\"",
		lun_idx, lun->disk_name);

	/* Notify application. */
	msc_notify_event(md->udev, lun_idx, lun->disk_name, true);
}

static void msc_enumerate_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct msc_dev *md =
		CONTAINER_OF(dwork, struct msc_dev, enumerate_work);

	if (!md->active) {
		return;
	}

	LOG_INF("Enumerating MSC device addr=%u", md->udev->addr);

	/* BOT Reset to clear any stale device state. */
	(void)msc_bot_reset(md);

	/* Get Max LUN. */
	md->max_lun = msc_get_max_lun(md);

	/* Enumerate each LUN. */
	for (uint8_t i = 0U; i <= md->max_lun; i++) {
		if (md->active) {
			msc_enumerate_lun(md, i);
		}
	}

	md->enumerating = false;
	LOG_INF("MSC enumeration complete (addr=%u, %u LUNs)",
		md->udev->addr, md->max_lun + 1U);
}

/* ================================================================
 * Class API callbacks
 * ================================================================ */

static int msc_class_init(struct usbh_class_data *const c_data)
{
	ARG_UNUSED(c_data);
	LOG_INF("USB MSC class driver init");
	return 0;
}

static int msc_class_probe(struct usbh_class_data *const c_data,
			   struct usb_device *const udev,
			   const uint8_t iface)
{
	struct msc_dev *md;
	struct usb_if_descriptor *if_desc = NULL;
	uint8_t ep_in = 0U, ep_out = 0U;
	uint16_t mps_in = 0U, mps_out = 0U;
	int ret;

	LOG_INF("MSC device probing: addr=%u iface=%u", udev->addr, iface);

	/* Look up the interface descriptor by bInterfaceNumber. */
	for (int i = 0;
	     i < ((struct usb_cfg_descriptor *)udev->cfg_desc)->bNumInterfaces;
	     i++) {
		struct usb_if_descriptor *d =
			(struct usb_if_descriptor *)udev->ifaces[i].dhp;

		if (d != NULL && d->bInterfaceNumber == iface) {
			if_desc = d;
			break;
		}
	}

	if (if_desc == NULL) {
		LOG_ERR("Interface %u descriptor not found", iface);
		return -ENOTSUP;
	}

	/* Verify the class triple (filter already matched, but double-check). */
	if (if_desc->bInterfaceClass != USB_BCC_MASS_STORAGE ||
	    if_desc->bInterfaceSubClass != MSC_SCSI_TRANSPARENT_CMD_SET ||
	    if_desc->bInterfaceProtocol != MSC_BULK_ONLY_TRANSPORT) {
		LOG_DBG("Not BOT/SCSI: 0x%02x/0x%02x/0x%02x, skipping",
			if_desc->bInterfaceClass,
			if_desc->bInterfaceSubClass,
			if_desc->bInterfaceProtocol);
		return -ENOTSUP;
	}

	/* Find bulk endpoints. */
	ret = msc_find_bulk_eps(if_desc, &ep_in, &mps_in,
				&ep_out, &mps_out);
	if (ret != 0) {
		LOG_ERR("Bulk endpoint discovery failed: %d", ret);
		return -ENOTSUP;
	}

	LOG_INF("MSC bulk endpoints: IN=0x%02x(%u) OUT=0x%02x(%u)",
		ep_in, mps_in, ep_out, mps_out);

	/* Allocate device slot. */
	md = msc_dev_alloc();
	if (md == NULL) {
		LOG_ERR("No free MSC device slot");
		return -ENOTSUP;
	}

	md->udev = udev;
	md->c_data = c_data;
	md->iface = iface;
	md->ep_in = ep_in;
	md->ep_out = ep_out;
	md->mps_in = mps_in;
	md->mps_out = mps_out;
	md->enumerating = true;

	c_data->priv = md;

	/* Start enumeration in the system work queue. */
	k_work_init_delayable(&md->enumerate_work, msc_enumerate_work_handler);
	k_work_schedule(&md->enumerate_work, K_NO_WAIT);

	return 0;
}

static int msc_class_removed(struct usbh_class_data *const c_data)
{
	struct msc_dev *md = c_data->priv;

	if (md == NULL) {
		return 0;
	}

	md->active = false;

	/* Cancel any pending enumeration work. */
	(void)k_work_cancel_delayable(&md->enumerate_work);

	/* Unregister all registered LUNs.
	 * IMPORTANT: the disconnect event is sent BEFORE unregistering
	 * the disk so the application can cleanly fs_unmount() while the
	 * disk_access layer is still functional.  If the disk is
	 * unregistered first, fatfs_unmount()'s disk_ioctl(CTRL_POWER,
	 * POWER_OFF) fails and the VFS mount point is never released,
	 * leaving a stale entry that blocks re-mount after replug. */
	for (uint8_t i = 0U; i <= md->max_lun; i++) {
		struct msc_lun *lun = &md->luns[i];

		if (lun->state == MSC_LUN_READY) {
			msc_notify_event(md->udev, i, lun->disk_name, false);
			(void)disk_access_unregister(&lun->disk);
			lun->state = MSC_LUN_UNINITIALIZED;
		}
	}

	/* Cancel any stuck pending transfer. */
	if (md->pending_xfer != NULL) {
		(void)usbh_xfer_dequeue(md->udev, md->pending_xfer);
		md->pending_xfer = NULL;
	}

	LOG_INF("MSC device removed (addr=%u)", md->udev->addr);
	c_data->priv = NULL;
	return 0;
}

/* ================================================================
 * Class registration
 * ================================================================ */

static struct usbh_class_api msc_class_api = {
	.init    = msc_class_init,
	.probe   = msc_class_probe,
	.removed = msc_class_removed,
};

static const struct usbh_class_filter msc_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_MASS_STORAGE,			/* 0x08 */
		.sub   = MSC_SCSI_TRANSPARENT_CMD_SET,		/* 0x06 */
		.proto = MSC_BULK_ONLY_TRANSPORT,		/* 0x50 */
	},
	{ 0 },  /* terminator */
};

/*
 * Register one class instance per supported device slot so that
 * multiple MSC devices can be managed simultaneously.
 */
#define _MSC_CLASS_INSTANCE(n, _) \
	USBH_DEFINE_CLASS(CONCAT(usbh_msc, n), &msc_class_api, NULL, msc_filters)

LISTIFY(CONFIG_USBH_MSC_MAX_DEVICES, _MSC_CLASS_INSTANCE, (), 0)
