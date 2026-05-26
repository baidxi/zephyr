/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control_sun8i_v3s.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT allwinner_sun8i_v3s_pinctrl

LOG_MODULE_REGISTER(pinctrl_sun8i_v3s, LOG_LEVEL_WRN);

/* PIO register layout per port (stride 0x24) */
#define PORT_STRIDE		0x24

/* Offsets within a port block */
#define CFG_REG_OFF(pin)	((pin / 8) * 4)
#define DRV_REG_OFF(pin)	((pin >= 16) ? 0x18 : 0x14)
#define PULL_REG_OFF(pin)	((pin >= 16) ? 0x20 : 0x1C)
#define CFG_PIN_SHIFT(pin)	((pin % 8) * 4)
#define CFG_PIN_MASK(pin)	(0x7 << CFG_PIN_SHIFT(pin))
#define DRV_PIN_SHIFT(pin)	(((pin >= 16) ? (pin - 16) : pin) * 2)
#define DRV_PIN_MASK(pin)	(0x3 << DRV_PIN_SHIFT(pin))
#define PULL_PIN_SHIFT(pin)	(((pin >= 16) ? (pin - 16) : pin) * 2)
#define PULL_PIN_MASK(pin)	(0x3 << PULL_PIN_SHIFT(pin))

/*
 * PIO controller base address is obtained from the pinctrl device itself,
 * NOT from the consumer device's reg parameter (which would be e.g. UART
 * or SPI base).  This mirrors the pattern used by pinctrl_sifive.c and
 * pinctrl_bflb.c which ignore the reg argument and use their own base.
 */
#define PIO_BASE_ADDR	DT_INST_REG_ADDR(0)

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins,
			   uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *pin = &pins[i];
		uint32_t port_base = PIO_BASE_ADDR + PORT_STRIDE * pin->port;
		uint32_t val;

		/* Configure pin function (mux) */
		val = sys_read32(port_base + CFG_REG_OFF(pin->pin));
		val &= ~CFG_PIN_MASK(pin->pin);
		val |= (pin->func << CFG_PIN_SHIFT(pin->pin));
		sys_write32(val, port_base + CFG_REG_OFF(pin->pin));

		/* Configure drive strength */
		val = sys_read32(port_base + DRV_REG_OFF(pin->pin));
		val &= ~DRV_PIN_MASK(pin->pin);
		val |= (pin->driving_level << DRV_PIN_SHIFT(pin->pin));
		sys_write32(val, port_base + DRV_REG_OFF(pin->pin));

		/* Configure pull-up / pull-down */
		val = sys_read32(port_base + PULL_REG_OFF(pin->pin));
		val &= ~PULL_PIN_MASK(pin->pin);
		val |= (pin->pull << PULL_PIN_SHIFT(pin->pin));
		sys_write32(val, port_base + PULL_REG_OFF(pin->pin));
	}

	return 0;
}

struct sun8i_pinctrl_config {
	uintptr_t base;
	const struct device *clock_dev;
	clock_control_subsys_t clk_bus;
};

static int sun8i_pinctrl_init(const struct device *dev)
{
	const struct sun8i_pinctrl_config *cfg = dev->config;
	int ret;

	/* Enable PIO bus clock (CLK_BUS_PIO) so PIO registers are accessible */
	ret = clock_control_on(cfg->clock_dev, (void *)cfg->clk_bus);
	if (ret < 0) {
		LOG_ERR("Failed to enable PIO bus clock (err %d)", ret);
		return ret;
	}

	LOG_DBG("PIO initialized at 0x%lx", (unsigned long)cfg->base);
	return 0;
}

#define SUN8I_PINCTRL_INIT(n)							\
	static const struct sun8i_pinctrl_config sun8i_pinctrl_cfg_##n = {	\
		.base = DT_INST_REG_ADDR(n),					\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
		.clk_bus = (clock_control_subsys_t)				\
			DT_INST_CLOCKS_CELL_BY_NAME(n, bus, clk_id),		\
	};									\
	DEVICE_DT_INST_DEFINE(n, sun8i_pinctrl_init, NULL, NULL,		\
			      &sun8i_pinctrl_cfg_##n, POST_KERNEL,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_PINCTRL_INIT)
