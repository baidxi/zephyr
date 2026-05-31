/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun4i USB PHY pseudo-device driver.
 *
 * Ported from Linux drivers/phy/allwinner/phy-sun4i-usb.c.
 * Supports VBUS/ID GPIO detection, dynamic ISCR setting,
 * PHY route switching for OTG mode.
 */

#define DT_DRV_COMPAT allwinner_sun4i_usb_phy

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

LOG_MODULE_REGISTER(usb_phy_sun4i, CONFIG_MUSB_LOG_LEVEL);

#include "usb_common.h"
#include "usb_phy_sun4i_regs.h"

/* =========================================================================
 * SoC-specific PHY configuration (compile-time constant)
 * ========================================================================= */

struct sunxi_usb_phy_cfg {
	uint8_t num_phys;
	uint8_t disc_thresh;
	uint16_t phyctl_offset;
	bool dedicated_clocks;
	bool phy0_dual_route;
	bool siddq_in_base;
	bool needs_phy2_siddq;
	uint32_t hci_phy_ctl_clear;
	uint16_t missing_phys;
};

static const struct sunxi_usb_phy_cfg sun8i_v3s_cfg = {
	.num_phys = 1,
	.disc_thresh = 3,
	.phyctl_offset = REG_PHYCTL_A33,
	.dedicated_clocks = true,
	.phy0_dual_route = true,
	.hci_phy_ctl_clear = PHY_CTL_H3_SIDDQ,
};

#define SUNXI_USB_PHY_CFG_FOR_COMPAT(phy_node)	(&sun8i_v3s_cfg)

/* =========================================================================
 * Per-instance PHY configuration (GPIO info, clock/reset handles)
 * ========================================================================= */

struct sunxi_usb_phy_config {
	uintptr_t phy_ctrl_base;
	uintptr_t pmu_base;
	const struct device *ccu;
	const struct device *rst;
	clock_control_subsys_t clk_subsys;
	uint32_t rst_id;
	const struct sunxi_usb_phy_cfg *cfg;

#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
	const struct device *id_det_gpio_dev;
	gpio_pin_t id_det_pin;
	gpio_flags_t id_det_flags;
#endif
	bool has_id_det;

#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
	const struct device *vbus_det_gpio_dev;
	gpio_pin_t vbus_det_pin;
	gpio_flags_t vbus_det_flags;
#endif
	bool has_vbus_det;
};

/* =========================================================================
 * Runtime state for PHY0 (ID/VBUS detection)
 * ========================================================================= */

#define DEBOUNCE_MS	50
#define POLL_MS		250

struct sunxi_usb_phy_state {
	struct k_work_delayable detect_work;
	int id_det;		/* -1 = uninitialized, 0 = host, 1 = peripheral */
	int vbus_det;		/* -1 = uninitialized, 0 = offline, 1 = online */
	bool phy0_init;		/* PHY0 has been enabled */
	bool id_irq_available;
	bool vbus_irq_available;

	/* Notification callback */
	void (*notify_cb)(const struct sunxi_usb_phy *phy,
			  enum sunxi_usb_phy_event event,
			  int state, void *user);
	void *notify_user;
};

static struct sunxi_usb_phy_state phy0_state;

#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
static struct gpio_callback id_det_gpio_cb;
#endif
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
static struct gpio_callback vbus_det_gpio_cb;
#endif

/* Global PHY config pointer for work function access */
static const struct sunxi_usb_phy_config *g_phy0_cfg;

/* Forward declarations */
static void sunxi_usb_phy_set_squelch_detect(const struct sunxi_usb_phy *phy,
					     bool enabled);
static void sunxi_usb_phy0_id_vbus_det_scan(struct k_work *work);

/* =========================================================================
 * PHY register access helpers
 * ========================================================================= */

static struct k_spinlock phyctl_lock;

static void sunxi_usb_phy_write(const struct sunxi_usb_phy *phy,
				uint32_t addr, uint32_t data, int len)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;
	const struct sunxi_usb_phy_cfg *cfg = pcfg->cfg;
	uint32_t usbc_bit = BIT(phy->index * 2);
	uintptr_t phyctl = pcfg->phy_ctrl_base + cfg->phyctl_offset;
	k_spinlock_key_t key;
	uint8_t temp;
	int i;

	key = k_spin_lock(&phyctl_lock);

	if (cfg->phyctl_offset == REG_PHYCTL_A33) {
		sys_write32(0, phyctl);
	}

	for (i = 0; i < len; i++) {
		temp = sys_read32(phyctl);
		temp &= ~(0xff << 8);
		temp |= ((addr + i) << 8);
		sys_write32(temp, phyctl);

		temp = sys_read8(phyctl);
		if (data & 0x1) {
			temp |= PHYCTL_DATA;
		} else {
			temp &= ~PHYCTL_DATA;
		}
		temp &= ~usbc_bit;
		sys_write8(temp, phyctl);

		temp = sys_read8(phyctl);
		temp |= usbc_bit;
		sys_write8(temp, phyctl);

		temp = sys_read8(phyctl);
		temp &= ~usbc_bit;
		sys_write8(temp, phyctl);

		data >>= 1;
	}

	k_spin_unlock(&phyctl_lock, key);
}

/* =========================================================================
 * ISCR helpers
 * ========================================================================= */

static void sunxi_usb_phy0_update_iscr(const struct sunxi_usb_phy *phy,
				       uint32_t clr, uint32_t set)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;
	uint32_t iscr;

	iscr = sys_read32(pcfg->phy_ctrl_base + REG_ISCR);
	iscr &= ~clr;
	iscr |= set;
	sys_write32(iscr, pcfg->phy_ctrl_base + REG_ISCR);
}

static void sunxi_usb_phy0_set_id_detect(const struct sunxi_usb_phy *phy,
					  int id_det)
{
	uint32_t val = id_det ? ISCR_FORCE_ID_HIGH : ISCR_FORCE_ID_LOW;

	sunxi_usb_phy0_update_iscr(phy, ISCR_FORCE_ID_MASK, val);
}

static void sunxi_usb_phy0_set_vbus_detect(const struct sunxi_usb_phy *phy,
					    int vbus_det)
{
	uint32_t val = vbus_det ? ISCR_FORCE_VBUS_HIGH : ISCR_FORCE_VBUS_LOW;

	sunxi_usb_phy0_update_iscr(phy, ISCR_FORCE_VBUS_MASK, val);
}

/* =========================================================================
 * Pass-by / PMU configuration
 * ========================================================================= */

static void sunxi_usb_phy_passby(const struct sunxi_usb_phy *phy, bool enable)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;
	uint32_t bits, reg_value;

	if (!pcfg->pmu_base) {
		return;
	}

	bits = SUNXI_AHB_ICHR8_EN | SUNXI_AHB_INCR4_BURST_EN |
	       SUNXI_AHB_INCRX_ALIGN_EN | SUNXI_ULPI_BYPASS_EN;

	reg_value = sys_read32(pcfg->pmu_base);

	if (enable) {
		reg_value |= bits;
	} else {
		reg_value &= ~bits;
	}

	sys_write32(reg_value, pcfg->pmu_base);
}

/* =========================================================================
 * ID / VBUS GPIO read helpers
 *
 * Equivalent to Linux sun4i_usb_phy0_get_id_det/vbus_det().
 * ========================================================================= */

static int sunxi_usb_phy0_get_id_det(const struct sunxi_usb_phy *phy)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;

	if (pcfg->has_id_det) {
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
		int val = gpio_pin_get(pcfg->id_det_gpio_dev, pcfg->id_det_pin);

		if (val < 0) {
			LOG_WRN("ID GPIO read error: %d", val);
			return 1; /* Fallback to peripheral */
		}
		return val;
#endif
	}
	/* No ID GPIO: default peripheral mode */
	return 1;
}

static int sunxi_usb_phy0_get_vbus_det(const struct sunxi_usb_phy *phy)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;

	if (pcfg->has_vbus_det) {
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
		int val = gpio_pin_get(pcfg->vbus_det_gpio_dev, pcfg->vbus_det_pin);

		if (val < 0) {
			LOG_WRN("VBUS GPIO read error: %d", val);
			return 1; /* Fallback to VBUS present */
		}
		return val;
#endif
	}
	/* No VBUS GPIO: fallback to VBUS present */
	return 1;
}

static bool sunxi_usb_phy0_have_vbus_det(const struct sunxi_usb_phy *phy)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;

	return pcfg->has_vbus_det;
}

static bool sunxi_usb_phy0_poll(const struct sunxi_usb_phy *phy)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;

	if ((pcfg->has_id_det && !phy0_state.id_irq_available) ||
	    (pcfg->has_vbus_det && !phy0_state.vbus_irq_available)) {
		return true;
	}
	return false;
}

/* =========================================================================
 * PHY route switching
 *
 * Equivalent to Linux sun4i_usb_phy0_reroute().
 * ========================================================================= */

static void sunxi_usb_phy0_reroute(const struct sunxi_usb_phy *phy,
				    int id_det)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;
	uint32_t regval;

	regval = sys_read32(pcfg->phy_ctrl_base + REG_PHY_OTGCTL);
	if (id_det == 0) {
		/* Host mode: route phy0 to EHCI/OHCI */
		regval &= ~OTGCTL_ROUTE_MUSB;
	} else {
		/* Peripheral mode: route phy0 to MUSB */
		regval |= OTGCTL_ROUTE_MUSB;
	}
	sys_write32(regval, pcfg->phy_ctrl_base + REG_PHY_OTGCTL);
}

/* =========================================================================
 * GPIO ISR — schedules debounced scan work
 * ========================================================================= */

static void sunxi_usb_phy0_gpio_isr(const struct device *dev,
				     struct gpio_callback *cb,
				     uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* ID or VBUS changed — schedule debounced scan */
	(void)k_work_schedule(&phy0_state.detect_work, K_MSEC(DEBOUNCE_MS));
}

/* =========================================================================
 * ID / VBUS detection scan work
 *
 * Ported from Linux sun4i_usb_phy0_id_vbus_det_scan().
 * ========================================================================= */

static void sunxi_usb_phy0_id_vbus_det_scan(struct k_work *work)
{
	const struct sunxi_usb_phy *phy;
	const struct sunxi_usb_phy_config *pcfg;
	const struct sunxi_usb_phy_cfg *cfg;
	int id_det, vbus_det;
	bool force_session_end = false;
	bool id_notify = false, vbus_notify = false;

	/* Obtain PHY pointer from global state */
	if (!g_phy0_cfg) {
		return;
	}

	/*
	 * Build a temporary sunxi_usb_phy struct to pass to helpers.
	 * The PHY pseudo-device instance holds the same pcfg pointer.
	 */
	struct sunxi_usb_phy tmp_phy = {
		.index = 0,
		.pcfg = g_phy0_cfg,
	};
	phy = &tmp_phy;
	pcfg = g_phy0_cfg;
	cfg = pcfg->cfg;

	if (!phy0_state.phy0_init) {
		return;
	}

	id_det = sunxi_usb_phy0_get_id_det(phy);
	vbus_det = sunxi_usb_phy0_get_vbus_det(phy);

	/* ID state changed? */
	if (id_det != phy0_state.id_det) {
		/*
		 * If we have no VBUS detection, force session end on
		 * ID change (matches Linux behaviour for OTG without
		 * vbus_det).
		 */
		if (!sunxi_usb_phy0_have_vbus_det(phy)) {
			force_session_end = true;
		}

		/* When entering host mode (id=0), force session end */
		if (force_session_end && id_det == 0) {
			sunxi_usb_phy0_set_vbus_detect(phy, 0);
			k_msleep(200);
			sunxi_usb_phy0_set_vbus_detect(phy, 1);
		}

		sunxi_usb_phy0_set_id_detect(phy, id_det);
		phy0_state.id_det = id_det;
		id_notify = true;
	}

	/* VBUS state changed? */
	if (vbus_det != phy0_state.vbus_det) {
		sunxi_usb_phy0_set_vbus_detect(phy, vbus_det);
		phy0_state.vbus_det = vbus_det;
		vbus_notify = true;
	}

	/* Process ID change notifications */
	if (id_notify) {
		LOG_INF("USB ID changed: %s mode",
			id_det ? "peripheral" : "host");

		/* When leaving host mode (id=1), force session end */
		if (force_session_end && id_det == 1) {
			sunxi_usb_phy0_set_vbus_detect(phy, 0);
			k_msleep(1000);
			sunxi_usb_phy0_set_vbus_detect(phy, 1);
		}

		/* Enable passby only in host mode */
		sunxi_usb_phy_passby(phy, !id_det);

		/* Re-route PHY0 if the SoC supports dual route */
		if (cfg->phy0_dual_route) {
			sunxi_usb_phy0_reroute(phy, id_det);
		}

		/* Notify upper layer */
		if (phy0_state.notify_cb) {
			phy0_state.notify_cb(phy, SUNXI_USB_PHY_EVENT_ID,
					     id_det, phy0_state.notify_user);
		}
	}

	/* Process VBUS change notifications */
	if (vbus_notify) {
		LOG_INF("USB VBUS changed: %s",
			vbus_det ? "online" : "offline");

		if (phy0_state.notify_cb) {
			phy0_state.notify_cb(phy, SUNXI_USB_PHY_EVENT_VBUS,
					     vbus_det, phy0_state.notify_user);
		}
	}

	/* If polling mode, reschedule */
	if (sunxi_usb_phy0_poll(phy)) {
		(void)k_work_schedule(&phy0_state.detect_work, K_MSEC(POLL_MS));
	}
}

/* =========================================================================
 * PHY enable / disable
 * ========================================================================= */

static int sunxi_usb_phy_enable(const struct sunxi_usb_phy *phy)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;
	const struct sunxi_usb_phy_cfg *cfg = pcfg->cfg;
	uint32_t val;
	int ret;

	/* PHY clock */
	ret = clock_control_on(pcfg->ccu, pcfg->clk_subsys);
	if (ret < 0) {
		return ret;
	}

	/* PHY reset */
	ret = reset_line_deassert(pcfg->rst, pcfg->rst_id);
	if (ret < 0) {
		clock_control_off(pcfg->ccu, pcfg->clk_subsys);
		return ret;
	}

	if (pcfg->pmu_base && cfg->hci_phy_ctl_clear) {
		val = sys_read32(pcfg->pmu_base + REG_HCI_PHY_CTL);
		val &= ~cfg->hci_phy_ctl_clear;
		sys_write32(val, pcfg->pmu_base + REG_HCI_PHY_CTL);
	}

	if (phy->index == 0) {
		sunxi_usb_phy_write(phy, PHY_RES45_CAL_EN, 0x01, 1);
	}

	sunxi_usb_phy_write(phy, PHY_TX_AMPLITUDE_TUNE, 0x14, 5);
	sunxi_usb_phy_write(phy, PHY_DISCON_TH_SEL, cfg->disc_thresh, 2);
	/*
	 * Enable squelch detect explicitly to match the state Linux
	 * leaves the PHY in after post_root_reset_end().
	 */
	sunxi_usb_phy_set_squelch_detect(phy, true);
	sunxi_usb_phy_passby(phy, true);

	if (phy->index == 0) {
		/* Enable pull-ups on DP/DM and ID pins */
		sunxi_usb_phy0_update_iscr(phy, 0,
			ISCR_DPDM_PULLUP_EN | ISCR_ID_PULLUP_EN);

		/*
		 * Configure ID/VBUS GPIO pins as inputs.
		 * The sun8i-v3s GPIO driver does not handle pull-up
		 * flags internally, but the PHY-level pull-up
		 * (ISCR_ID_PULLUP_EN) handles the ID pin.
		 */
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
		if (pcfg->has_id_det) {
			ret = gpio_pin_configure(pcfg->id_det_gpio_dev,
						 pcfg->id_det_pin,
						 GPIO_INPUT | pcfg->id_det_flags);
			if (ret < 0) {
				LOG_WRN("ID GPIO configure failed: %d", ret);
			}
		}
#endif
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
		if (pcfg->has_vbus_det) {
			ret = gpio_pin_configure(pcfg->vbus_det_gpio_dev,
						 pcfg->vbus_det_pin,
						 GPIO_INPUT | pcfg->vbus_det_flags);
			if (ret < 0) {
				LOG_WRN("VBUS GPIO configure failed: %d", ret);
			}
		}
#endif

		/*
		 * Initial synchronous scan: read ID/VBUS GPIOs and set
		 * ISCR before MUSB starts, so the controller begins in
		 * the correct role.  This matches Linux's approach where
		 * the PHY probe synchronously reads the initial state.
		 */
		phy0_state.id_det = sunxi_usb_phy0_get_id_det(phy);
		phy0_state.vbus_det = sunxi_usb_phy0_get_vbus_det(phy);
		sunxi_usb_phy0_set_id_detect(phy, phy0_state.id_det);
		sunxi_usb_phy0_set_vbus_detect(phy, phy0_state.vbus_det);

		LOG_INF("Initial: ID=%d (%s), VBUS=%d (%s)",
			phy0_state.id_det,
			phy0_state.id_det ? "peripheral" : "host",
			phy0_state.vbus_det,
			phy0_state.vbus_det ? "online" : "offline");

		/* Adjust passby based on detected mode (host only) */
		sunxi_usb_phy_passby(phy, !phy0_state.id_det);

		/* Re-route PHY0 if the SoC supports dual route */
		if (cfg->phy0_dual_route) {
			sunxi_usb_phy0_reroute(phy, phy0_state.id_det);
		}

		phy0_state.phy0_init = true;

		/* Store config for work function access */
		g_phy0_cfg = pcfg;

		/*
		 * Try to configure GPIO interrupts for ID and VBUS.
		 * If the GPIO controller doesn't support interrupts
		 * (returns -EOPNOTSUPP), we fall back to polling.
		 * This mirrors Linux's gpiod_to_irq() > 0 check.
		 */
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
		if (pcfg->has_id_det) {
			ret = gpio_pin_interrupt_configure(
				pcfg->id_det_gpio_dev, pcfg->id_det_pin,
				GPIO_INT_EDGE_BOTH);
			if (ret == 0) {
				gpio_init_callback(&id_det_gpio_cb,
						   sunxi_usb_phy0_gpio_isr,
						   BIT(pcfg->id_det_pin));
				(void)gpio_add_callback(pcfg->id_det_gpio_dev,
							&id_det_gpio_cb);
				phy0_state.id_irq_available = true;
				LOG_INF("ID det: interrupt mode (pin %d)",
					pcfg->id_det_pin);
			} else {
				LOG_INF("ID det: polling mode "
					"(irq config failed: %d)", ret);
			}
		}
#endif

#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
		if (pcfg->has_vbus_det) {
			ret = gpio_pin_interrupt_configure(
				pcfg->vbus_det_gpio_dev, pcfg->vbus_det_pin,
				GPIO_INT_EDGE_BOTH);
			if (ret == 0) {
				gpio_init_callback(&vbus_det_gpio_cb,
						   sunxi_usb_phy0_gpio_isr,
						   BIT(pcfg->vbus_det_pin));
				(void)gpio_add_callback(pcfg->vbus_det_gpio_dev,
							&vbus_det_gpio_cb);
				phy0_state.vbus_irq_available = true;
				LOG_INF("VBUS det: interrupt mode (pin %d)",
					pcfg->vbus_det_pin);
			} else {
				LOG_INF("VBUS det: polling mode "
					"(irq config failed: %d)", ret);
			}
		}
#endif

		/*
		 * Schedule ongoing monitoring.  If in polling mode,
		 * the work runs every POLL_MS.  If interrupts are
		 * available, the GPIO ISR triggers the work instead.
		 */
		if (sunxi_usb_phy0_poll(phy)) {
			(void)k_work_schedule(&phy0_state.detect_work,
					      K_MSEC(POLL_MS));
		}
	}

	return 0;
}

static int sunxi_usb_phy_disable(const struct sunxi_usb_phy *phy)
{
	const struct sunxi_usb_phy_config *pcfg = phy->pcfg;

	if (phy->index == 0) {
		sunxi_usb_phy0_update_iscr(phy,
			ISCR_DPDM_PULLUP_EN | ISCR_ID_PULLUP_EN, 0);
		phy0_state.phy0_init = false;

		/* Cancel pending work */
		(void)k_work_cancel_delayable(&phy0_state.detect_work);

		/* Remove GPIO callbacks */
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
		if (pcfg->has_id_det && phy0_state.id_irq_available) {
			(void)gpio_remove_callback(pcfg->id_det_gpio_dev,
						   &id_det_gpio_cb);
			phy0_state.id_irq_available = false;
		}
#endif
#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
		if (pcfg->has_vbus_det && phy0_state.vbus_irq_available) {
			(void)gpio_remove_callback(pcfg->vbus_det_gpio_dev,
						   &vbus_det_gpio_cb);
			phy0_state.vbus_irq_available = false;
		}
#endif
	}

	sunxi_usb_phy_passby(phy, false);
	reset_line_assert(pcfg->rst, pcfg->rst_id);
	clock_control_off(pcfg->ccu, pcfg->clk_subsys);

	return 0;
}

/* =========================================================================
 * Accessors and notification registration
 * ========================================================================= */

static int sunxi_usb_phy_get_id_det_impl(const struct sunxi_usb_phy *phy)
{
	if (phy->index == 0 && phy0_state.id_det >= 0) {
		return phy0_state.id_det;
	}
	return 1; /* Default: peripheral */
}

static int sunxi_usb_phy_get_vbus_det_impl(const struct sunxi_usb_phy *phy)
{
	if (phy->index == 0 && phy0_state.vbus_det >= 0) {
		return phy0_state.vbus_det;
	}
	return 1; /* Default: VBUS present */
}

static void sunxi_usb_phy_register_notify(const struct sunxi_usb_phy *phy,
					   void (*cb)(const struct sunxi_usb_phy *phy,
						      enum sunxi_usb_phy_event event,
						      int state, void *user),
					   void *user)
{
	phy0_state.notify_cb = cb;
	phy0_state.notify_user = user;
}

static void sunxi_usb_phy_set_squelch_detect(const struct sunxi_usb_phy *phy,
					     bool enabled)
{
	sunxi_usb_phy_write(phy, PHY_SQUELCH_DETECT, enabled ? 0 : 2, 2);
}

/* =========================================================================
 * Compile-time instance generation via DEFINE_SUNXI_USB_PHY
 * ========================================================================= */

#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
#define _ID_DET_GPIO_DEV  \
	DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(DT_NODELABEL(usbphy), \
					   usb0_id_det_gpios, 0))
#define _ID_DET_PIN  \
	DT_GPIO_PIN_BY_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
#define _ID_DET_FLAGS \
	DT_GPIO_FLAGS_BY_IDX(DT_NODELABEL(usbphy), usb0_id_det_gpios, 0)
#define _HAS_ID_DET true
#else
#define _ID_DET_GPIO_DEV NULL
#define _ID_DET_PIN 0
#define _ID_DET_FLAGS 0
#define _HAS_ID_DET false
#endif

#if DT_PROP_HAS_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
#define _VBUS_DET_GPIO_DEV \
	DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(DT_NODELABEL(usbphy), \
					   usb0_vbus_det_gpios, 0))
#define _VBUS_DET_PIN \
	DT_GPIO_PIN_BY_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
#define _VBUS_DET_FLAGS \
	DT_GPIO_FLAGS_BY_IDX(DT_NODELABEL(usbphy), usb0_vbus_det_gpios, 0)
#define _HAS_VBUS_DET true
#else
#define _VBUS_DET_GPIO_DEV NULL
#define _VBUS_DET_PIN 0
#define _VBUS_DET_FLAGS 0
#define _HAS_VBUS_DET false
#endif

#define DEFINE_SUNXI_USB_PHY(usb_node, phy_node)                           \
	static const struct sunxi_usb_phy_config                           \
	CONCAT(sunxi_phy, DT_DEP_ORD(phy_node), _cfg) = {                  \
		.phy_ctrl_base = DT_REG_ADDR_BY_IDX(phy_node, 0),          \
		.pmu_base = DT_REG_ADDR_BY_NAME_OR(phy_node, pmu0, 0),     \
		.ccu = DEVICE_DT_GET(DT_NODELABEL(ccu)),                   \
		.rst = DEVICE_DT_GET(DT_NODELABEL(reset)),                 \
		.clk_subsys = (clock_control_subsys_t)(uintptr_t)           \
			      DT_PHA_BY_IDX(phy_node, clocks, 0, clk_id),  \
		.rst_id = DT_PHA_BY_IDX(phy_node, resets, 0, id),         \
		.cfg = SUNXI_USB_PHY_CFG_FOR_COMPAT(phy_node),             \
		IF_ENABLED(                                                 \
			DT_PROP_HAS_IDX(DT_NODELABEL(usbphy),               \
					 usb0_id_det_gpios, 0),             \
			(.id_det_gpio_dev = _ID_DET_GPIO_DEV,))              \
		IF_ENABLED(                                                 \
			DT_PROP_HAS_IDX(DT_NODELABEL(usbphy),               \
					 usb0_id_det_gpios, 0),             \
			(.id_det_pin = _ID_DET_PIN,))                        \
		IF_ENABLED(                                                 \
			DT_PROP_HAS_IDX(DT_NODELABEL(usbphy),               \
					 usb0_id_det_gpios, 0),             \
			(.id_det_flags = _ID_DET_FLAGS,))                    \
		.has_id_det = _HAS_ID_DET,                                  \
		IF_ENABLED(                                                 \
			DT_PROP_HAS_IDX(DT_NODELABEL(usbphy),               \
					 usb0_vbus_det_gpios, 0),           \
			(.vbus_det_gpio_dev = _VBUS_DET_GPIO_DEV,))          \
		IF_ENABLED(                                                 \
			DT_PROP_HAS_IDX(DT_NODELABEL(usbphy),               \
					 usb0_vbus_det_gpios, 0),           \
			(.vbus_det_pin = _VBUS_DET_PIN,))                    \
		IF_ENABLED(                                                 \
			DT_PROP_HAS_IDX(DT_NODELABEL(usbphy),               \
					 usb0_vbus_det_gpios, 0),           \
			(.vbus_det_flags = _VBUS_DET_FLAGS,))                \
		.has_vbus_det = _HAS_VBUS_DET,                              \
	};                                                                  \
	const struct sunxi_usb_phy                                         \
	USB_SUNXI_PHY_PSEUDODEV_NAME(usb_node) = {                         \
		.enable = sunxi_usb_phy_enable,                            \
		.disable = sunxi_usb_phy_disable,                          \
		.set_squelch_detect = sunxi_usb_phy_set_squelch_detect,    \
		.get_id_det = sunxi_usb_phy_get_id_det_impl,              \
		.get_vbus_det = sunxi_usb_phy_get_vbus_det_impl,          \
		.register_notify = sunxi_usb_phy_register_notify,          \
		.index = DT_PHA_BY_IDX(usb_node, phys, 0, index),         \
		.pcfg = &CONCAT(sunxi_phy, DT_DEP_ORD(phy_node), _cfg),   \
	};

/* Early init: set up the delayed work before any PHY enable */
static int sunxi_usb_phy_early_init(void)
{
	k_work_init_delayable(&phy0_state.detect_work,
			      sunxi_usb_phy0_id_vbus_det_scan);
	return 0;
}
SYS_INIT(sunxi_usb_phy_early_init, POST_KERNEL,
	 CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

/*
 * Use the DEFINE_SUNXI_USB_PHY macro to instantiate the PHY pseudo-device.
 */
DEFINE_SUNXI_USB_PHY(DT_NODELABEL(usbotg), DT_NODELABEL(usbphy));
