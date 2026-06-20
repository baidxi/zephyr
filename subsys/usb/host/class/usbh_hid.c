/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB HID Class driver for the Zephyr USB host stack.
 *
 * Supports USB HID Boot Protocol devices (keyboards and mice) connected
 * either directly to the host controller root port or through a hub.
 *
 * The driver relies on the EHCI Periodic Schedule (interrupt transfers)
 * implemented in the low-level host controller driver.  When a device is
 * enumerated the class auto-probe mechanism (usbh_class_probe_device) is
 * triggered by the hub driver; this class matches on
 *   bInterfaceClass     = 0x03 (HID)
 *   bInterfaceSubClass  = 0x01 (Boot Interface)
 *   bInterfaceProtocol  = 0x01 (Keyboard) or 0x02 (Mouse)
 * and sets the device into Boot Protocol mode, then continuously polls
 * the Interrupt IN endpoint and delivers parsed input reports through a
 * user-registered callback.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>

#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usbh_hid.h>
#include <zephyr/usb/usbh.h>

#include "../usbh_device.h"
#include "../usbh_ch9.h"
#include "../usbh_class.h"
#include "../usbh_desc.h"

LOG_MODULE_REGISTER(usbh_hid);

#define USBH_HID_MAX_DEVICES	CONFIG_USBH_HID_MAX_DEVICES

/* Boot-protocol input report sizes (fixed format) */
#define HID_KBD_BOOT_REPORT_LEN		8
#define HID_MOUSE_BOOT_REPORT_LEN	4

/* HID class-specific request bmRequestType values */
#define HID_REQTYPE_OUT_IFACE		0x21	/* H2D | Class | Interface */
#define HID_REQTYPE_IN_IFACE		0xA1	/* D2H | Class | Interface */

/* ---- Boot-protocol parsed report structures ---- */

struct hid_kbd_report {
	uint8_t modifier;	/* Ctrl/Shift/Alt/etc bitmap */
	uint8_t reserved;
	uint8_t key[6];		/* up to 6 simultaneous key codes, 0 = none */
} __packed;

struct hid_mouse_report {
	uint8_t buttons;	/* bit0=left, bit1=right, bit2=middle */
	int8_t x;		/* relative X movement */
	int8_t y;		/* relative Y movement */
	int8_t wheel;		/* optional wheel movement */
} __packed;

/* ---- Per-device private data ---- */

struct hid_dev {
	bool active;		/* true while polling is desired */
	uint8_t iface;		/* interface number */
	uint8_t proto;		/* HID_BOOT_IFACE_CODE_KEYBOARD / MOUSE */
	uint8_t ep_addr;	/* interrupt IN endpoint address */
	uint16_t mps;		/* max packet size of interrupt endpoint */
	uint16_t err_count;	/* consecutive error completions */
	struct uhc_transfer *xfer;
	struct net_buf *buf;
};

static struct hid_dev hid_devs[USBH_HID_MAX_DEVICES];

/* ---- Application callback registration ---- */

static usbh_hid_report_cb_t report_cb;
static void *report_cb_user_data;

void usbh_hid_register_report_cb(usbh_hid_report_cb_t cb, void *user_data)
{
	report_cb = cb;
	report_cb_user_data = user_data;
}

/* ---- Device slot management ---- */

static struct hid_dev *hid_dev_alloc(void)
{
	for (int i = 0; i < USBH_HID_MAX_DEVICES; i++) {
		if (!hid_devs[i].active) {
			memset(&hid_devs[i], 0, sizeof(hid_devs[i]));
			hid_devs[i].active = true;
			return &hid_devs[i];
		}
	}

	return NULL;
}

/* ---- HID class requests ---- */

static int hid_set_idle(struct usb_device *udev, uint8_t iface)
{
	/* SET_IDLE: wValue=0 (duration 0 = infinite), wIndex=interface */
	return usbh_req_setup(udev,
			      HID_REQTYPE_OUT_IFACE, USB_HID_SET_IDLE,
			      0, iface, 0, NULL);
}

static int hid_set_protocol(struct usb_device *udev, uint8_t iface,
			    uint16_t proto)
{
	/* SET_PROTOCOL: wValue=protocol (0=Boot, 1=Report) */
	return usbh_req_setup(udev,
			      HID_REQTYPE_OUT_IFACE, USB_HID_SET_PROTOCOL,
			      proto, iface, 0, NULL);
}

/* ---- Endpoint discovery ---- */

/*
 * Find the Interrupt IN endpoint belonging to a specific interface by
 * scanning the raw configuration descriptor.  This is necessary because
 * udev->ep_in[] is a global array shared across all interfaces, so for
 * composite (multi-interface) devices we cannot rely on it.
 */
static int hid_find_interrupt_in_ep(const struct usb_if_descriptor *if_desc,
				    uint8_t *ep_addr, uint16_t *mps)
{
	const struct usb_desc_header *head = (const void *)if_desc;
	uint8_t ep_count = if_desc->bNumEndpoints;
	uint8_t found = 0;

	while (found < ep_count) {
		head = usbh_desc_get_next(head);
		if (head == NULL) {
			break;
		}

		/* Stop if we reach the next interface descriptor */
		if (head->bDescriptorType == USB_DESC_INTERFACE) {
			break;
		}

		if (usbh_desc_is_valid_endpoint(head)) {
			const struct usb_ep_descriptor *ep = (const void *)head;

			found++;
			if ((ep->bmAttributes & 0x03) == USB_EP_TYPE_INTERRUPT &&
			    USB_EP_DIR_IS_IN(ep->bEndpointAddress)) {
				*ep_addr = ep->bEndpointAddress;
				*mps = sys_le16_to_cpu(ep->wMaxPacketSize);
				return 0;
			}
		}
	}

	return -ENOTSUP;
}

/* ---- Interrupt transfer completion ---- */

static void hid_log_kbd_report(const struct hid_kbd_report *r)
{
	char keys[32];
	int off = 0;

	for (int i = 0; i < 6 && off < (int)sizeof(keys) - 4; i++) {
		if (r->key[i] != 0) {
			off += snprintk(keys + off, sizeof(keys) - off,
					"%02x ", r->key[i]);
		}
	}

	if (off == 0) {
		keys[0] = '\0';
	}

	LOG_INF("KBD: mod=0x%02x keys=[%s]", r->modifier, keys);
}

static void hid_log_mouse_report(const struct hid_mouse_report *r)
{
	LOG_INF("MOUSE: btn=0x%02x x=%d y=%d w=%d",
		r->buttons, r->x, r->y, r->wheel);
}

static int hid_xfer_cb(struct usb_device *const udev,
		       struct uhc_transfer *const xfer)
{
	struct hid_dev *hd = xfer->priv;

	if (hd == NULL) {
		return 0;
	}

	/*
	 * If the device is being removed, clean up the transfer here
	 * (in the callback thread) to avoid racing with hid_class_removed()
	 * which runs in the hub workqueue thread.
	 */
	if (!hd->active) {
		if (xfer->buf != NULL) {
			usbh_xfer_buf_free(udev, xfer->buf);
		}
		usbh_xfer_free(udev, xfer);
		return 0;
	}

	/*
	 * Transient transfer errors (XACT, etc.) are common on the USB bus,
	 * especially when another device on the same controller is removed.
	 * As long as this HID device is still active, re-arm the transfer to
	 * resume polling instead of permanently killing the endpoint.
	 */
	if (xfer->err != 0) {
		/* Only log the first error in a burst to avoid flooding
		 * the log when a device is physically removed (the error
		 * loop runs at ~1 kHz until the hub detects removal).
		 */
		if (hd->err_count == 0) {
			LOG_WRN("HID xfer error %d, re-arming (addr=%u iface=%u)",
				xfer->err, udev->addr, hd->iface);
		}
		hd->err_count++;
		if (xfer->buf != NULL) {
			net_buf_reset(xfer->buf);
		}
		(void)usbh_xfer_enqueue(udev, xfer);
		return 0;
	}

	/* Successful completion: reset the error counter. */
	hd->err_count = 0;

	if (xfer->buf != NULL && xfer->buf->len > 0) {
		uint8_t *data = xfer->buf->data;
		uint16_t len = xfer->buf->len;

		/* Deliver parsed report to application callback. */
		if (report_cb != NULL) {
			report_cb(udev, data, len, hd->proto,
				  report_cb_user_data);
		} else if (hd->proto == HID_BOOT_IFACE_CODE_KEYBOARD &&
			   len >= sizeof(struct hid_kbd_report)) {
			const struct hid_kbd_report *r =
				(const struct hid_kbd_report *)data;
			/* Only log when there are actual key events — keyboards
			 * in boot protocol send empty reports (all zeros) on
			 * every poll interval when idle, which floods the log. */
			bool has_key = false;
			for (int i = 0; i < 6; i++) {
				if (r->key[i] != 0) {
					has_key = true;
					break;
				}
			}
			if (r->modifier != 0 || has_key) {
				hid_log_kbd_report(r);
			}
		} else if (hd->proto == HID_BOOT_IFACE_CODE_MOUSE &&
			   len >= 3) {
			hid_log_mouse_report(
				(const struct hid_mouse_report *)data);
		} else {
			LOG_HEXDUMP_DBG(data, len, "HID report");
		}

		/* Reset the buffer so the next IN transfer can refill it. */
		net_buf_reset(xfer->buf);
	}

	/* Re-arm the interrupt transfer for continuous polling. */
	(void)usbh_xfer_enqueue(udev, xfer);

	return 0;
}

static int hid_submit_interrupt(struct usb_device *udev, struct hid_dev *hd)
{
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	uint16_t buflen = hd->mps;

	if (buflen == 0 || buflen > 64) {
		buflen = HID_KBD_BOOT_REPORT_LEN;
	}

	xfer = usbh_xfer_alloc(udev, hd->ep_addr, hid_xfer_cb, hd);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	buf = usbh_xfer_buf_alloc(udev, buflen);
	if (buf == NULL) {
		usbh_xfer_free(udev, xfer);
		return -ENOMEM;
	}

	if (usbh_xfer_buf_add(udev, xfer, buf) != 0) {
		usbh_xfer_buf_free(udev, buf);
		usbh_xfer_free(udev, xfer);
		return -ENOMEM;
	}

	xfer->type = USB_EP_TYPE_INTERRUPT;

	hd->xfer = xfer;
	hd->buf = buf;

	return usbh_xfer_enqueue(udev, xfer);
}

/* ---- Class API callbacks ---- */

static int hid_class_init(struct usbh_class_data *const c_data)
{
	ARG_UNUSED(c_data);
	LOG_INF("USB HID class driver init");
	return 0;
}

static int hid_class_probe(struct usbh_class_data *const c_data,
			   struct usb_device *const udev,
			   const uint8_t iface)
{
	struct hid_dev *hd;
	uint8_t ep_addr = 0;
	uint16_t mps = 0;
	int ret;

	LOG_INF("HID device probing: addr=%u iface=%u", udev->addr, iface);

	/*
	 * Look up the interface descriptor by bInterfaceNumber, as the
	 * ifaces[] array may not be indexed by interface number.
	 */
	struct usb_if_descriptor *if_desc = NULL;

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

	uint8_t proto = if_desc->bInterfaceProtocol;

	LOG_INF("HID iface %u: class=0x%02x sub=0x%02x proto=0x%02x",
		iface, if_desc->bInterfaceClass, if_desc->bInterfaceSubClass,
		proto);

	if (proto != HID_BOOT_IFACE_CODE_KEYBOARD &&
	    proto != HID_BOOT_IFACE_CODE_MOUSE) {
		LOG_DBG("HID protocol 0x%02x not keyboard/mouse, skipping", proto);
		return -ENOTSUP;
	}

	/* Locate the Interrupt IN endpoint for this specific interface. */
	ret = hid_find_interrupt_in_ep(if_desc, &ep_addr, &mps);
	if (ret != 0) {
		LOG_ERR("No interrupt IN endpoint found: %d", ret);
		return -ENOTSUP;
	}

	LOG_INF("HID %s: interrupt IN ep=0x%02x mps=%u",
		proto == HID_BOOT_IFACE_CODE_KEYBOARD ? "keyboard" : "mouse",
		ep_addr, mps);

	hd = hid_dev_alloc();
	if (hd == NULL) {
		LOG_ERR("No free HID device slot");
		return -ENOTSUP;
	}

	hd->iface = iface;
	hd->proto = proto;
	hd->ep_addr = ep_addr;
	hd->mps = mps;

	/* Configure the device for Boot Protocol operation. */
	ret = hid_set_idle(udev, iface);
	if (ret != 0) {
		LOG_WRN("SET_IDLE failed: %d (continuing)", ret);
	}

	ret = hid_set_protocol(udev, iface, HID_PROTOCOL_BOOT);
	if (ret != 0) {
		LOG_ERR("SET_PROTOCOL(Boot) failed: %d", ret);
		hd->active = false;
		return -ENOTSUP;
	}

	/* Start polling the interrupt endpoint. */
	ret = hid_submit_interrupt(udev, hd);
	if (ret != 0) {
		LOG_ERR("Failed to submit interrupt transfer: %d", ret);
		hd->active = false;
		return -ENOTSUP;
	}

	c_data->priv = hd;

	LOG_INF("HID %s polling started (addr=%u)",
		proto == HID_BOOT_IFACE_CODE_KEYBOARD ? "keyboard" : "mouse",
		udev->addr);

	return 0;
}

static int hid_class_removed(struct usbh_class_data *const c_data)
{
	struct hid_dev *hd = c_data->priv;

	if (hd == NULL) {
		return 0;
	}

	/*
	 * Mark the device as inactive so that hid_xfer_cb() (running
	 * asynchronously in the msgq thread) stops re-enqueuing and
	 * instead frees the transfer.  Then dequeue the QH from the
	 * periodic schedule — the resulting cancel completion event
	 * will trigger the callback which performs the actual cleanup.
	 *
	 * We must NOT free the transfer here because a completion
	 * event for it may still be pending in the message queue.
	 */
	hd->active = false;

	if (hd->xfer != NULL) {
		(void)usbh_xfer_dequeue(c_data->udev, hd->xfer);
		/* Transfer will be freed by hid_xfer_cb when it sees
		 * hd->active == false.  Clear the pointer so we know
		 * cleanup is in progress.
		 */
		hd->xfer = NULL;
	}

	LOG_INF("HID device removed");
	c_data->priv = NULL;
	return 0;
}

/* ---- Class registration ---- */

static struct usbh_class_api hid_class_api = {
	.init          = hid_class_init,
	.probe         = hid_class_probe,
	.removed       = hid_class_removed,
};

static const struct usbh_class_filter hid_filters[] = {
	/*
	 * Match any HID interface (bInterfaceClass = 0x03).
	 * The probe handler inspects bInterfaceProtocol to decide whether
	 * this is a keyboard (0x01) or mouse (0x02), and issues
	 * SET_PROTOCOL(Boot) to switch Report-Protocol devices.
	 */
	{
		.flags = USBH_CLASS_MATCH_CODE,
		.class = USB_BCC_HID,			/* 0x03 */
	},
	{ 0 },  /* terminator */
};

/*
 * Each USBH_DEFINE_CLASS() creates one class instance that can bind to at
 * most one device/interface at a time.  To support simultaneous keyboard and
 * mouse (or multiple HID devices), register one instance per supported
 * device slot.  The probe handler allocates from the shared hid_devs[] pool.
 */
#define _HID_CLASS_INSTANCE(n, _) \
	USBH_DEFINE_CLASS(CONCAT(usbh_hid, n), &hid_class_api, NULL, hid_filters)

LISTIFY(CONFIG_USBH_HID_MAX_DEVICES, _HID_CLASS_INSTANCE, (), 0)
