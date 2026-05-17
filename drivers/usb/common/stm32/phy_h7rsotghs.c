/*
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * STM32 H7RS OTGHS embedded HS PHY driver
 *
 * The STM32H7RS series includes a USBPHYC (USB PHY Controller)
 * that must be initialized before the USB OTG HS core can operate.
 * Unlike the U5 series, the H7RS USBPHYC is controlled purely
 * through RCC registers (no SYSCFG involvement).
 */

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>

#include <stm32_bitops.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/kernel.h>

#include "stm32_usb_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_h7rsotghs, CONFIG_STM32_USB_COMMON_LOG_LEVEL);

/* Even though we won't use this macro, define it for grep-ability */
#define DT_DRV_COMPAT st_stm32h7rs_otghs_phy

struct stm32_h7rsotghs_phy_config {
	uint32_t num_clocks;
	struct stm32_pclken clocks[];
};

static const struct device *rcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

static int stm32_h7rsotghs_phy_enable(const struct stm32_usb_phy *phy)
{
	const struct stm32_h7rsotghs_phy_config *cfg = phy->pcfg;
	int res;

	LOG_INF("USBPHYC enable start");

	/* Enable USBPHYC bus clock (AHB1 bit 26) */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USBPHYC);

	/*
	 * Force USBPHYC reset before changing the clock source.
	 * The bootrom may have initialized the USBPHYC with a different
	 * clock source (e.g. HSI48). Switching the clock mux while the
	 * internal PLL is running causes frequency glitches and PLL
	 * lock loss. Assert reset first, then configure the clock,
	 * then release reset so the PLL cleanly re-locks.
	 */
	LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_USBPHYC);

	/* Configure USBPHYC input clock source mux (if provided) */
	if (cfg->num_clocks > 1) {
		res = clock_control_configure(rcc,
			(clock_control_subsys_t)&cfg->clocks[1], NULL);
		if (res != 0) {
			LOG_ERR("USBPHYC clock mux config failed: %d", res);
			return res;
		}
	}

	/* Turn on the USBPHYC kernel clock */
	res = clock_control_on(rcc,
		(clock_control_subsys_t)&cfg->clocks[0]);
	if (res != 0) {
		LOG_ERR("USBPHYC kernel clock enable failed: %d", res);
		return res;
	}

	/* Release USBPHYC from reset (clean start with correct clock) */
	LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_USBPHYC);

	/* Disable USBPHYC power-down */
	if (LL_RCC_USBPHYC_IsEnabledPowerDown()) {
		LL_RCC_USBPHYC_DisablePowerDown();
		LOG_INF("USBPHYC power-down disabled");
	} else {
		LOG_INF("USBPHYC already powered up");
	}

	/* Wait for USBPHYC internal PLL to stabilize and lock.
	 * The USBPHYC PLL needs time to generate the 48MHz PHY clock
	 * and 60MHz UTMI clock from the input reference clock.
	 * Give it ample time (5ms) for a clean lock after reset.
	 */
	k_busy_wait(5000);
	LOG_INF("USBPHYC enable done (USBPHYCSEL=0x%08x)",
		LL_RCC_GetUSBPHYCClockSource(LL_RCC_USBPHYC_CLKSOURCE));

	return 0;
}

static int stm32_h7rsotghs_phy_disable(const struct stm32_usb_phy *phy)
{
	const struct stm32_h7rsotghs_phy_config *cfg = phy->pcfg;

	return clock_control_off(rcc,
		(clock_control_subsys_t)&cfg->clocks[0]);
}

#define DEFINE_H7RSOTGHS_PHY(usb_node, phy_node)						\
	static const struct stm32_h7rsotghs_phy_config						\
		CONCAT(phy, DT_DEP_ORD(phy_node), _cfg) = {					\
		.num_clocks = DT_NUM_CLOCKS(phy_node),						\
		.clocks = STM32_DT_CLOCKS(phy_node)						\
	};											\
	const struct stm32_usb_phy USB_STM32_PHY_PSEUDODEV_NAME(usb_node) = {			\
		.enable = stm32_h7rsotghs_phy_enable,						\
		.disable = stm32_h7rsotghs_phy_disable,						\
		.pcfg = &CONCAT(phy, DT_DEP_ORD(phy_node), _cfg),				\
	};

/*
 * Iterate all USB nodes and instantiate PHY when appropriate.
 */
#define _FOREACH_NODE(usb_node)									\
	IF_ENABLED(USB_STM32_NODE_PHY_IS_EMBEDDED_HS(usb_node),				\
		(DEFINE_H7RSOTGHS_PHY(usb_node, USB_STM32_PHY(usb_node))))
#define _FOREACH_COMPAT(compat) DT_FOREACH_STATUS_OKAY(compat, _FOREACH_NODE)
FOR_EACH(_FOREACH_COMPAT, (), STM32_USB_COMPATIBLES)
