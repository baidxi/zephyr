/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the Zephyr USB Host HID class driver.
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBH_HID_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USBH_HID_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/hid.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration to avoid pulling in the full UHC header. */
struct usb_device;

/**
 * @brief HID device protocol, as reported by the interface descriptor.
 */
enum usbh_hid_proto {
	/** USB HID Boot Protocol keyboard. */
	USBH_HID_PROTO_KEYBOARD = HID_BOOT_IFACE_CODE_KEYBOARD,
	/** USB HID Boot Protocol mouse. */
	USBH_HID_PROTO_MOUSE = HID_BOOT_IFACE_CODE_MOUSE,
};

/**
 * @brief Input report callback type.
 *
 * Called from the USB host workqueue thread whenever an Interrupt IN
 * transfer completes with data.  The buffer @p report is only valid for
 * the duration of the callback; copy it if longer-lived access is needed.
 *
 * @param udev      USB device that produced the report.
 * @param report    Pointer to the raw input report data.
 * @param len       Number of valid bytes in @p report.
 * @param proto     Device protocol (see @ref usbh_hid_proto).
 * @param user_data Caller-provided opaque pointer.
 */
typedef void (*usbh_hid_report_cb_t)(struct usb_device *udev,
				     const uint8_t *report,
				     uint16_t len,
				     uint8_t proto,
				     void *user_data);

/**
 * @brief Register an input report callback.
 *
 * The callback is global and shared by all HID devices handled by this
 * class driver.  Pass NULL to unregister.
 *
 * @param cb        Callback function, or NULL to clear.
 * @param user_data Opaque pointer passed back to the callback.
 */
void usbh_hid_register_report_cb(usbh_hid_report_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBH_HID_H_ */
