/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register definitions for Allwinner sun4i USB PHY controller.
 * Ported from Linux drivers/phy/allwinner/phy-sun4i-usb.c
 */

#ifndef ZEPHYR_DRIVERS_USB_COMMON_SUNXI_USB_PHY_SUN4I_REGS_H_
#define ZEPHYR_DRIVERS_USB_COMMON_SUNXI_USB_PHY_SUN4I_REGS_H_

#include <zephyr/sys/util.h>

/* PHY control register offsets (within phy_ctrl base) */
#define REG_ISCR              0x00
#define REG_PHYCTL_A10        0x04
#define REG_PHYBIST           0x08
#define REG_PHYTUNE           0x0c
#define REG_PHYCTL_A33        0x10
#define REG_PHY_OTGCTL        0x20

/* PMU register offsets (within pmu base) */
#define REG_HCI_PHY_CTL       0x10

/* PHYCTL bit */
#define PHYCTL_DATA           BIT(7)

/* OTGCTL bits */
#define OTGCTL_ROUTE_MUSB     BIT(0)

/* PMU pass-through configuration bits */
#define SUNXI_AHB_ICHR8_EN       BIT(10)
#define SUNXI_AHB_INCR4_BURST_EN BIT(9)
#define SUNXI_AHB_INCRX_ALIGN_EN BIT(8)
#define SUNXI_ULPI_BYPASS_EN     BIT(0)

/* ISCR (Interface Status and Control Register) bits */
#define ISCR_ID_PULLUP_EN     BIT(17)
#define ISCR_DPDM_PULLUP_EN   BIT(16)
#define ISCR_FORCE_ID_MASK    GENMASK(15, 14)
#define ISCR_FORCE_ID_LOW     BIT(15)
#define ISCR_FORCE_ID_HIGH    (BIT(15) | BIT(14))
#define ISCR_FORCE_VBUS_MASK  GENMASK(13, 12)
#define ISCR_FORCE_VBUS_LOW   BIT(13)
#define ISCR_FORCE_VBUS_HIGH  (BIT(13) | BIT(12))

/* PHY internal tuning register addresses */
#define PHY_PLL_BW               0x03
#define PHY_RES45_CAL_EN         0x0c
#define PHY_TX_AMPLITUDE_TUNE    0x20
#define PHY_TX_SLEWRATE_TUNE     0x22
#define PHY_VBUSVALID_TH_SEL     0x25
#define PHY_PULLUP_RES_SEL       0x27
#define PHY_OTG_FUNC_EN          0x28
#define PHY_VBUS_DET_EN          0x29
#define PHY_DISCON_TH_SEL        0x2a
#define PHY_SQUELCH_DETECT       0x3c

/* SIDDQ bits (varies by SoC variant) */
#define PHY_CTL_VBUSVLDEXT       BIT(5)
#define PHY_CTL_SIDDQ            BIT(3)
#define PHY_CTL_H3_SIDDQ         BIT(1)

#define PHYCTL_VBUSVLDEXT        BIT(5)

#define MAX_PHYS                 4

#endif /* ZEPHYR_DRIVERS_USB_COMMON_SUNXI_USB_PHY_SUN4I_REGS_H_ */
