/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Host Mass Storage Class (MSC) public API.
 *
 * The MSC host driver automatically detects USB mass-storage devices
 * (USB flash drives, external hard drives, card readers, …) that use
 * the Bulk-Only Transport (BOT) protocol with the SCSI transparent
 * command set, and registers each Logical Unit (LUN) as a Zephyr
 * disk device accessible through the standard disk_access API.
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBH_MSC_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USBH_MSC_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/usb/usbh.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback invoked when a MSC LUN is connected or disconnected.
 *
 * @param udev       USB device that was connected/disconnected.
 * @param lun        Logical Unit Number (0-based).
 * @param disk_name  Name of the registered disk (for disk_access_* APIs),
 *                   or NULL if disk registration failed / device removed.
 * @param connected  true if the LUN was just connected, false if removed.
 * @param user_data  User data pointer passed to usbh_msc_register_event_cb().
 */
typedef void (*usbh_msc_event_cb_t)(struct usb_device *udev,
				    uint8_t lun,
				    const char *disk_name,
				    bool connected,
				    void *user_data);

/**
 * @brief Register a callback for MSC device connection/disconnection events.
 *
 * The callback is invoked from the system workqueue thread whenever a
 * MSC LUN is successfully enumerated and registered as a disk, or when
 * the device is removed.
 *
 * @param cb         Callback function, or NULL to unregister.
 * @param user_data  User data passed to the callback.
 */
void usbh_msc_register_event_cb(usbh_msc_event_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBH_MSC_H_ */
