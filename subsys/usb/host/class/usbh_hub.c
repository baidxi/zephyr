/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-FileCopyrightText: Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Hub Class driver for the USB host stack
 *
 * This driver handles USB 2.0 hubs (bDeviceClass = 0x09).
 * It powers on downstream ports, polls for connection changes,
 * resets newly connected ports, and enumerates child devices
 * through the existing usbh device management infrastructure.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usb_hub.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>

#include "usbh_device.h"
#include "usbh_ch9.h"
#include "usbh_class.h"
#include "usbh_class_api.h"

LOG_MODULE_REGISTER(usbh_hub, CONFIG_USBH_LOG_LEVEL);

/* ---- Timing constants ---- */
#define HUB_POLL_INTERVAL_MS		250
#define HUB_RESET_TIMEOUT_MS		1000
#define HUB_POWER_SETTLE_MS		100
#define HUB_ENUM_RETRY_DELAY_MS		500

/* ---- Hub Descriptor (USB 2.0 Spec §11.23.2.1) ---- */
#define USB_HUB_DESCRIPTOR_TYPE		0x29

struct usb_hub_descriptor {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bNbrPorts;
	uint16_t wHubCharacteristics;
	uint8_t  bPwrOn2PwrGood;
	uint8_t  bHubContrCurrent;
	/* Variable-length bitmaps follow: DeviceRemovable, PortPwrCtrlMask */
} __packed;

/* wHubCharacteristics bits */
#define HUB_CHAR_LPSM_MASK		0x0003  /* Logical Power Switching Mode */
#define HUB_CHAR_COMPOUND		BIT(2)
#define HUB_CHAR_OCPM_MASK		0x0018  /* Over-Current Protection Mode */
#define HUB_CHAR_TTTT_MASK		0x0060  /* TT Think Time */
#define HUB_CHAR_PORT_IND		BIT(7)

/* Port Status bits (USB 2.0 Spec §11.24.2.7.1, returned in wPortStatus) */
#define PORT_STAT_CONNECTION		BIT(0)
#define PORT_STAT_ENABLE		BIT(1)
#define PORT_STAT_SUSPEND		BIT(2)
#define PORT_STAT_OVERCURRENT		BIT(3)
#define PORT_STAT_RESET			BIT(4)
#define PORT_STAT_POWER			BIT(8)
#define PORT_STAT_LOW_SPEED		BIT(9)
#define PORT_STAT_HIGH_SPEED		BIT(10)
#define PORT_STAT_TEST			BIT(11)
#define PORT_STAT_INDICATOR		BIT(12)

/* Port Status Change bits (USB 2.0 Spec §11.24.2.7.2, in high 16 bits) */
#define PORT_CSC			BIT(0)  /* Connect Status Change */
#define PORT_PESC			BIT(1)  /* Port Enable Status Change */
#define PORT_PSSC			BIT(2)  /* Port Suspend Status Change */
#define PORT_OCIC			BIT(3)  /* Over-Current Indicator Change */
#define PORT_PRSC			BIT(4)  /* Port Reset Status Change */

/* Request recipient/type encodings for hub class requests */
#define HUB_REQ_DIR_IN			0x80
#define HUB_REQ_DIR_OUT			0x00
#define HUB_REQ_TYPE_CLASS		0x20
#define HUB_REQ_RECP_DEVICE		0x00
#define HUB_REQ_RECP_OTHER		0x03

#define MAX_HUB_PORTS			8

/* Per-hub runtime data */
struct usbh_hub_data {
	struct usb_device *hub_udev;
	struct usbh_context *ctx;
	uint8_t  nports;
	uint16_t characteristics;
	uint8_t  pwr_on_2_pwr_good;	/* in 2ms units */
	bool active;
	struct usb_device *children[MAX_HUB_PORTS];
	struct k_work_delayable poll_work;
};

/* Support up to CONFIG_USBH_HUB_MAX hubs */
static struct usbh_hub_data hub_data_pool[CONFIG_USBH_HUB_MAX];

static struct usbh_hub_data *hub_data_alloc(void)
{
	for (int i = 0; i < CONFIG_USBH_HUB_MAX; i++) {
		if (hub_data_pool[i].hub_udev == NULL) {
			return &hub_data_pool[i];
		}
	}
	return NULL;
}

/* ---- Hub class request helpers ----
 *
 * All hub class requests are sent via usbh_req_setup() which is
 * synchronous (waits for completion with a 5-second timeout).
 */

/* GET_DESCRIPTOR(hub) — read the hub descriptor */
static int hub_get_descriptor(struct usb_device *udev,
			      struct usb_hub_descriptor *desc, size_t len)
{
	struct net_buf *buf;
	int ret;

	buf = usbh_xfer_buf_alloc(udev, len);
	if (buf == NULL) {
		return -ENOMEM;
	}

	ret = usbh_req_setup(udev,
		HUB_REQ_DIR_IN | HUB_REQ_TYPE_CLASS | HUB_REQ_RECP_DEVICE,
		USB_HCREQ_GET_DESCRIPTOR,
		(USB_HUB_DESCRIPTOR_TYPE << 8), 0, len, buf);

	if (ret == 0) {
		size_t copy = MIN(buf->len, len);
		memcpy(desc, buf->data, copy);
	}

	usbh_xfer_buf_free(udev, buf);
	return ret;
}

/* GET_STATUS(port) — returns 4 bytes: wPortStatus (LE) + wPortChange (LE) */
static int hub_get_port_status(struct usb_device *udev, uint8_t port,
			       uint16_t *status, uint16_t *change)
{
	uint8_t buf32[4];
	struct net_buf *buf;
	int ret;

	buf = usbh_xfer_buf_alloc(udev, sizeof(buf32));
	if (buf == NULL) {
		return -ENOMEM;
	}

	ret = usbh_req_setup(udev,
		HUB_REQ_DIR_IN | HUB_REQ_TYPE_CLASS | HUB_REQ_RECP_OTHER,
		USB_HCREQ_GET_STATUS,
		0, port, sizeof(buf32), buf);

	if (ret == 0 && buf->len >= 4) {
		*status = sys_get_le16(&buf->data[0]);
		*change = sys_get_le16(&buf->data[2]);
	} else if (ret == 0) {
		ret = -EIO;
	}

	usbh_xfer_buf_free(udev, buf);
	return ret;
}

/* CLEAR_FEATURE(port, feature) */
static int hub_clear_port_feature(struct usb_device *udev, uint8_t port,
				  uint16_t feature)
{
	return usbh_req_setup(udev,
		HUB_REQ_DIR_OUT | HUB_REQ_TYPE_CLASS | HUB_REQ_RECP_OTHER,
		USB_HCREQ_CLEAR_FEATURE,
		feature, port, 0, NULL);
}

/* SET_FEATURE(port, feature) — already wrapped for power/reset in ch9 */

/* ---- Port management ---- */

/* Determine device speed from hub port status bits */
static enum usb_device_speed port_speed_from_status(uint16_t status)
{
	if (status & PORT_STAT_HIGH_SPEED) {
		return USB_SPEED_SPEED_HS;
	}
	if (status & PORT_STAT_LOW_SPEED) {
		return USB_SPEED_SPEED_LS;
	}
	return USB_SPEED_SPEED_FS;
}

/*
 * Reset a hub downstream port and wait for the reset to complete.
 * Returns the port status word or a negative error code.
 */
static int hub_reset_port(struct usb_device *hub, uint8_t port)
{
	int ret;
	uint16_t status = 0, change = 0;

	LOG_INF("Resetting hub port %u", port);

	ret = usbh_req_set_hcfs_prst(hub, port);
	if (ret) {
		LOG_ERR("SET_FEATURE(PORT_RESET) failed: %d", ret);
		return ret;
	}

	/* Poll for C_PORT_RESET (USB 2.0: reset takes ≥10 ms, typ 50–100 ms) */
	for (int i = 0; i < HUB_RESET_TIMEOUT_MS / 20; i++) {
		k_msleep(20);

		ret = hub_get_port_status(hub, port, &status, &change);
		if (ret) {
			LOG_WRN("GET_PORT_STATUS during reset: %d", ret);
			continue;
		}

		if (change & PORT_PRSC) {
			/* Reset complete — clear the change bit */
			hub_clear_port_feature(hub, port,
					       USB_HCFS_C_PORT_RESET);

			LOG_INF("Port %u reset done, status=0x%04x", port, status);
			return status;
		}
	}

	LOG_ERR("Port %u reset timeout", port);
	return -ETIMEDOUT;
}

/*
 * Enumerate a device that just connected behind the hub.
 * Creates a usb_device, sets its speed from the hub port status,
 * and runs the standard enumeration flow (descriptor → address → config).
 */
static void hub_enumerate_child(struct usbh_hub_data *hd, uint8_t port,
				uint16_t port_status)
{
	struct usbh_context *const ctx = hd->ctx;
	struct usb_device *child;
	int ret;

	child = usbh_device_alloc(ctx);
	if (child == NULL) {
		LOG_ERR("Failed to allocate child device for port %u", port);
		return;
	}

	child->speed = port_speed_from_status(port_status);
	child->hub_addr = hd->hub_udev->addr;
	child->hub_port = port;
	LOG_INF("Port %u: child speed=%u", port, child->speed);

	/* usbh_device_init() skips bus reset for non-root devices,
	 * which is correct — the hub port reset has already happened. */
	child->state = USB_STATE_DEFAULT;
	ret = usbh_device_init(child);
	if (ret) {
		LOG_ERR("Child device init failed on port %u: %d", port, ret);
		usbh_device_free(child);
		return;
	}

	LOG_INF("Port %u: child enumerated addr=%u class=0x%02x",
		port, child->addr, child->dev_desc.bDeviceClass);

	hd->children[port - 1] = child;

	/* Probe class drivers for the child */
	usbh_class_probe_device(child);
}

/* ---- Polling work ---- */

static void hub_poll_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct usbh_hub_data *hd =
		CONTAINER_OF(dwork, struct usbh_hub_data, poll_work);
	struct usb_device *hub = hd->hub_udev;
	uint16_t status, change;
	int ret;

	if (!hd->active || hub == NULL) {
		return;
	}

	for (uint8_t p = 1; p <= hd->nports; p++) {
		ret = hub_get_port_status(hub, p, &status, &change);
		if (ret) {
			LOG_DBG("GET_PORT_STATUS(%u): %d", p, ret);
			continue;
		}

		LOG_DBG("port %u: status=%04x change=%04x conn=%u csc=%u",
			p, status, change,
			(status & PORT_STAT_CONNECTION) ? 1 : 0,
			(change & PORT_CSC) ? 1 : 0);

		/* Handle connection status change */
		if (change & PORT_CSC) {
			hub_clear_port_feature(hub, p,
					       USB_HCFS_C_PORT_CONNECTION);

			if (status & PORT_STAT_CONNECTION) {
				/* Device connected — reset and enumerate */
				if (hd->children[p - 1] != NULL) {
					LOG_INF("Port %u already has child, skip", p);
					continue;
				}
				LOG_INF("Port %u: device connected", p);

				int rst = hub_reset_port(hub, p);
				if (rst >= 0) {
					hub_enumerate_child(hd, p,
						(uint16_t)rst);
				}
			} else {
				/* Device disconnected */
				LOG_INF("Port %u: device disconnected", p);
				if (hd->children[p - 1] != NULL) {
					usbh_device_disconnect(hd->ctx,
						hd->children[p - 1]);
					hd->children[p - 1] = NULL;
				}
			}
		}
	}

	/* Re-schedule */
	k_work_reschedule(&hd->poll_work, K_MSEC(HUB_POLL_INTERVAL_MS));
}

/* ---- Class API callbacks ---- */

static int hub_class_init(struct usbh_class_data *const c_data)
{
	LOG_INF("USB Hub class driver init");
	return 0;
}

static int hub_class_probe(struct usbh_class_data *const c_data,
			   struct usb_device *const udev,
			   const uint8_t iface)
{
	struct usb_hub_descriptor hub_desc;
	struct usbh_hub_data *hd;
	int ret;

	LOG_INF("Hub device probing: addr=%u class=0x%02x iface=%u",
		udev->addr, udev->dev_desc.bDeviceClass, iface);

	/* Read the hub descriptor (request at least the fixed part + 2 bitmap bytes) */
	ret = hub_get_descriptor(udev, &hub_desc, sizeof(hub_desc) + 2);
	if (ret) {
		LOG_ERR("Failed to read hub descriptor: %d", ret);
		return -ENOTSUP;
	}

	if (hub_desc.bNbrPorts == 0 || hub_desc.bNbrPorts > MAX_HUB_PORTS) {
		LOG_ERR("Hub has unsupported port count: %u", hub_desc.bNbrPorts);
		return -ENOTSUP;
	}

	LOG_INF("Hub: %u ports, characteristics=0x%04x, pwr_on2pwr_good=%u",
		hub_desc.bNbrPorts,
		sys_le16_to_cpu(hub_desc.wHubCharacteristics),
		hub_desc.bPwrOn2PwrGood);

	hd = hub_data_alloc();
	if (hd == NULL) {
		LOG_ERR("No free hub data slot");
		return -ENOMEM;
	}

	memset(hd, 0, sizeof(*hd));
	hd->hub_udev = udev;
	hd->ctx = udev->ctx;
	hd->nports = hub_desc.bNbrPorts;
	hd->characteristics = sys_le16_to_cpu(hub_desc.wHubCharacteristics);
	hd->pwr_on_2_pwr_good = hub_desc.bPwrOn2PwrGood;

	c_data->priv = hd;

	/* Power on all downstream ports */
	LOG_INF("Powering on %u hub ports", hd->nports);
	for (uint8_t p = 1; p <= hd->nports; p++) {
		ret = usbh_req_set_hcfs_ppwr(udev, p);
		if (ret) {
			LOG_WRN("SET_FEATURE(PORT_POWER, %u) failed: %d",
				p, ret);
		}
	}

	/* Wait for port power to stabilize */
	uint32_t settle_ms = (hd->pwr_on_2_pwr_good + 1) * 2;
	if (settle_ms < HUB_POWER_SETTLE_MS) {
		settle_ms = HUB_POWER_SETTLE_MS;
	}
	LOG_INF("Waiting %u ms for port power settle", settle_ms);
	k_msleep(settle_ms);

	/* Start polling */
	hd->active = true;
	k_work_init_delayable(&hd->poll_work, hub_poll_handler);
	k_work_reschedule(&hd->poll_work, K_MSEC(HUB_POLL_INTERVAL_MS));

	LOG_INF("Hub driver active, polling every %u ms", HUB_POLL_INTERVAL_MS);
	return 0;
}

static int hub_class_removed(struct usbh_class_data *const c_data)
{
	struct usbh_hub_data *hd = c_data->priv;

	if (hd == NULL) {
		return 0;
	}

	hd->active = false;
	k_work_cancel_delayable(&hd->poll_work);

	/* Disconnect all children */
	for (uint8_t p = 0; p < hd->nports && p < MAX_HUB_PORTS; p++) {
		if (hd->children[p] != NULL) {
			usbh_device_disconnect(hd->ctx, hd->children[p]);
			hd->children[p] = NULL;
		}
	}

	LOG_INF("Hub removed: %u ports", hd->nports);
	hd->hub_udev = NULL;
	c_data->priv = NULL;
	return 0;
}

/* ---- Class registration ---- */

static struct usbh_class_api hub_class_api = {
	.init          = hub_class_init,
	.probe         = hub_class_probe,
	.removed       = hub_class_removed,
};

/* Match bDeviceClass = 0x09 (Hub), any subclass/protocol */
static const struct usbh_class_filter hub_filters[] = {
	{
		.class = 0x09,
		.sub   = 0,
		.proto = 0,
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
	},
	{ 0 },  /* terminator */
};

USBH_DEFINE_CLASS(usbh_hub, &hub_class_api, NULL, hub_filters);
