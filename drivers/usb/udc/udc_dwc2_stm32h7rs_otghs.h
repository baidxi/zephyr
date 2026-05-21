/*
 * Copyright (c) 2025 STMicroelectronics
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_STM32H7RS_OTGHS_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_STM32H7RS_OTGHS_H

#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <usb_dwc2_hw.h>

struct usb_dw_stm32_clk {
	const struct device *const dev;
	const struct stm32_pclken *const pclken;
	size_t pclken_len;
};

static inline int stm32h7rs_otghs_enable_clk(const struct usb_dw_stm32_clk *const clk)
{
	int ret;

	if (!device_is_ready(clk->dev)) {
		return -ENODEV;
	}

	/*
	 * If a second clock is configured (PLL3_Q for 60 MHz PHY clock),
	 * configure it first, then enable the bus clock (AHB1).
	 */
	if (clk->pclken_len > 1) {
		ret = clock_control_configure(clk->dev,
					      (void *)&clk->pclken[1],
					      NULL);
		if (ret) {
			return ret;
		}
	}

	return clock_control_on(clk->dev, (void *)&clk->pclken[0]);
}

/*
 * On STM32H7RS, register offset 0x038 is OTG_GCCFG (General Core Configuration),
 * not the standard DWC2 GGPIo. The usb_dwc2_reg structure maps ggpio at 0x038,
 * so we operate on ggpio to access GCCFG bits.
 *
 * GCCFG bit definitions (RM0477):
 *   Bit 21: VBDEN  - VBUS detection enable
 *   Bit 16: PWRDWN - PHY power down (active high)
 *   Bit 20: PDEN   - Primary detection enable (BCD)
 *   Bit 22: SDEN   - Secondary detection enable (BCD)
 *   Bit 19: DCDEN  - Data contact detection enable (BCD)
 */
static inline int stm32h7rs_otghs_enable_phy(const struct device *dev)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t ggpio_reg = (mem_addr_t)&base->ggpio;

	/* Enable VBUS detection and take PHY out of power-down */
	sys_set_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_VBDEN);
	sys_clear_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_PWRDWN);

	return 0;
}

static inline int stm32h7rs_otghs_disable_phy(const struct device *dev)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t ggpio_reg = (mem_addr_t)&base->ggpio;

	/* Power down PHY and disable VBUS detection */
	sys_clear_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_VBDEN |
				  USB_DWC2_GGPIO_STM32_PWRDWN);

	return 0;
}

#define QUIRK_STM32H7RS_OTGHS_DEFINE(n)					\
	static const struct stm32_pclken pclken_##n[] = STM32_DT_INST_CLOCKS(n);\
										\
	static const struct usb_dw_stm32_clk stm32h7rs_clk_##n = {		\
		.dev = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),			\
		.pclken = pclken_##n,						\
		.pclken_len = DT_INST_NUM_CLOCKS(n),				\
	};									\
										\
	static int stm32h7rs_otghs_enable_clk_##n(const struct device *dev)	\
	{									\
		return stm32h7rs_otghs_enable_clk(&stm32h7rs_clk_##n);		\
	}									\
										\
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {		\
		.pre_enable = stm32h7rs_otghs_enable_clk_##n,			\
		.post_enable = stm32h7rs_otghs_enable_phy,			\
		.disable = stm32h7rs_otghs_disable_phy,				\
		.irq_clear = NULL,						\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_STM32H7RS_OTGHS_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_STM32H7RS_OTGHS_H */
