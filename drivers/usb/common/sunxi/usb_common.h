/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sunxi USB PHY pseudo-device definitions.
 *
 * Pattern reference: STM32 usb_common.h (drivers/usb/common/stm32/)
 *
 * USB controller drivers include this header and use
 * USB_SUNXI_PHY_PSEUDODEV_GET_OR_NULL(usb_node) to obtain a pointer
 * to the PHY pseudo-device, then call phy->enable() / phy->disable().
 */

#ifndef ZEPHYR_DRIVERS_USB_COMMON_SUNXI_USB_COMMON_H_
#define ZEPHYR_DRIVERS_USB_COMMON_SUNXI_USB_COMMON_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

/*
 * DT compatibles for all Allwinner sunxi USB controllers.
 * Phase 1: V3s MUSB (peripheral mode).
 * Phase 2: add other MUSB/EHCI/OHCI compatibles.
 */
#define SUNXI_USB_COMPATIBLES \
	allwinner_sun8i_v3s_musb, \
	allwinner_sun20i_d1_musb

/* Obtain the PHY node for a USB controller from its "phys" property */
#define USB_SUNXI_PHY(usb_node) \
	DT_PHANDLE_BY_IDX(usb_node, phys, 0)

/* Evaluates to 1 if the PHY of usb_node is a sun4i-type PHY */
#define USB_SUNXI_NODE_PHY_IS_SUN4I(usb_node) \
	DT_NODE_HAS_COMPAT(USB_SUNXI_PHY(usb_node), allwinner,sun4i-usb-phy)

/**
 * @brief USB dual-role mode.
 *
 * Matches Linux usb_dr_mode enumeration.
 */
enum sunxi_usb_dr_mode {
	SUNXI_USB_DR_MODE_PERIPHERAL = 0,
	SUNXI_USB_DR_MODE_HOST,
	SUNXI_USB_DR_MODE_OTG,
};

/**
 * @brief PHY state change event types for notification callback.
 */
enum sunxi_usb_phy_event {
	/** ID detection pin changed state */
	SUNXI_USB_PHY_EVENT_ID,
	/** VBUS detection changed state */
	SUNXI_USB_PHY_EVENT_VBUS,
};

/**
 * @brief Allwinner sunxi USB PHY pseudo-device.
 *
 * Each USB controller that references a sun4i-usb-phy in its "phys"
 * property gets a compile-time generated instance of this struct.
 */
struct sunxi_usb_phy {
	/**
	 * @brief Enable PHY: clocks + reset deassert + calibration + pass-through.
	 *
	 * @param phy  PHY pseudo-device pointer
	 * @returns 0 on success, negative error code on failure
	 */
	int (*enable)(const struct sunxi_usb_phy *phy);

	/**
	 * @brief Disable PHY: pass-through off + reset assert + clocks off.
	 *
	 * @param phy  PHY pseudo-device pointer
	 * @returns 0 on success, negative error code on failure
	 */
	int (*disable)(const struct sunxi_usb_phy *phy);

	/**
	 * @brief Set squelch detect (called by USB controller driver).
	 *
	 * @param phy     PHY pseudo-device pointer
	 * @param enabled true = disable squelch, false = enable squelch
	 */
	void (*set_squelch_detect)(const struct sunxi_usb_phy *phy, bool enabled);

	/**
	 * @brief Get current ID detection state.
	 *
	 * Reads the ID GPIO if configured, otherwise returns the default
	 * based on dr_mode.
	 *
	 * @param phy  PHY pseudo-device pointer
	 * @returns 1 = peripheral mode (ID floating/high),
	 *          0 = host mode (ID grounded)
	 */
	int (*get_id_det)(const struct sunxi_usb_phy *phy);

	/**
	 * @brief Get current VBUS detection state.
	 *
	 * Reads the VBUS GPIO if configured, otherwise returns 1 (VBUS present)
	 * as a fallback.
	 *
	 * @param phy  PHY pseudo-device pointer
	 * @returns 1 = VBUS present, 0 = VBUS absent
	 */
	int (*get_vbus_det)(const struct sunxi_usb_phy *phy);

	/**
	 * @brief Register a callback for PHY state changes (ID/VBUS).
	 *
	 * @param phy   PHY pseudo-device pointer
	 * @param cb    Callback function (NULL to unregister)
	 * @param user  User data passed to callback
	 */
	void (*register_notify)(const struct sunxi_usb_phy *phy,
				void (*cb)(const struct sunxi_usb_phy *phy,
					   enum sunxi_usb_phy_event event,
					   int state, void *user),
				void *user);

	/** PHY index (0..3), extracted from DT phys specifier cell */
	uint8_t index;

	/** Pointer to per-instance configuration (SoC-specific) */
	const void *pcfg;
};

/*
 * Name generation for the PHY pseudo-device symbol.
 * The symbol name is derived from the USB controller's DT node label,
 * suffixed with "__sunxi_phy".
 */
#define USB_SUNXI_PHY_PSEUDODEV_NAME(usb_node) \
	CONCAT(DEVICE_DT_NAME_GET(usb_node), __sunxi_phy)

/*
 * Returns a pointer to the PHY pseudo-device for usb_node.
 * In Phase 1 this always returns a valid pointer (no NULL case),
 * but the macro pattern allows future conditional compilation.
 */
#define USB_SUNXI_PHY_PSEUDODEV_GET_OR_NULL(usb_node) \
	(&USB_SUNXI_PHY_PSEUDODEV_NAME(usb_node))

/* Forward-declare all PHY pseudo-devices */
#define _SUNXI_PHY_PSEUDODEV_DECLARE(usb_node) \
	extern const struct sunxi_usb_phy USB_SUNXI_PHY_PSEUDODEV_NAME(usb_node);
#define _SUNXI_DECLARE_ALL_PHYS_OF_COMPAT(compat) \
	DT_FOREACH_STATUS_OKAY(compat, _SUNXI_PHY_PSEUDODEV_DECLARE)
FOR_EACH(_SUNXI_DECLARE_ALL_PHYS_OF_COMPAT, (), SUNXI_USB_COMPATIBLES)

#endif /* ZEPHYR_DRIVERS_USB_COMMON_SUNXI_USB_COMMON_H_ */
