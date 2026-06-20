/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner D1 / T113-S3 pinctrl driver.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT allwinner_sun20i_d1_pinctrl

LOG_MODULE_REGISTER(pinctrl_d1, LOG_LEVEL_WRN);

/* D1 / T113-S3 new register layout (PDF §9.7.4, BANK_MEM_SIZE=0x30) */
#define PORT_STRIDE	0x30

#define CFG_REG(p)	(((p) / 8) * 4)
#define DRV_REG(p)	(((p) >= 16) ? 0x18 : 0x14)
/* PULL at +0x24 (pins 16-31) / +0x28 (pins 0-15) on D1, vs 0x1c/0x20 on old SoCs */
#define PULL_REG(p)	(((p) >= 16) ? 0x28 : 0x24)
#define CFG_SHIFT(p)	(((p) % 8) * 4)
#define CFG_MASK(p)	(0xf << CFG_SHIFT(p))
/* DRV field width = 4 bits/pin (2 data + 2 reserved), step=4, data mask=0x3 */
#define DRV_SHIFT(p)	((((p) >= 16) ? (p) - 16 : (p)) * 4)
#define DRV_MASK(p)	(0x3 << DRV_SHIFT(p))
/* PULL field width = 2 bits/pin, step=2 */
#define PULL_SHIFT(p)	((((p) >= 16) ? (p) - 16 : (p)) * 2)
#define PULL_MASK(p)	(0x3 << PULL_SHIFT(p))

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *p = &pins[i];
		uintptr_t port = DT_INST_REG_ADDR(0) + PORT_STRIDE * p->port;
		uint32_t val;

		val = sys_read32(port + CFG_REG(p->pin));
		val &= ~CFG_MASK(p->pin);
		val |= p->func << CFG_SHIFT(p->pin);
		sys_write32(val, port + CFG_REG(p->pin));

		val = sys_read32(port + DRV_REG(p->pin));
		val &= ~DRV_MASK(p->pin);
		val |= p->driving_level << DRV_SHIFT(p->pin);
		sys_write32(val, port + DRV_REG(p->pin));

		val = sys_read32(port + PULL_REG(p->pin));
		val &= ~PULL_MASK(p->pin);
		val |= p->pull << PULL_SHIFT(p->pin);
		sys_write32(val, port + PULL_REG(p->pin));
	}

	return 0;
}

struct d1_pinctrl_cfg {
	uintptr_t base;
	const struct device *clk_dev;
	clock_control_subsys_t clk_bus;
};

static int d1_pinctrl_init(const struct device *dev)
{
	const struct d1_pinctrl_cfg *cfg = dev->config;
	int ret;

	ret = clock_control_on(cfg->clk_dev, (void *)cfg->clk_bus);
	if (ret < 0) {
		LOG_ERR("PIO clock fail (%d)", ret);
		return ret;
	}
	LOG_DBG("ready @ 0x%lx", (unsigned long)cfg->base);
	return 0;
}

/*
 * Pin controller must run early in PRE_KERNEL_1 so that pinmux
 * (pinctrl_apply_state) is available before the UART driver
 * initializes.  Order: CCU(30) -> PINCTRL(40) -> UART(50).
 */
#define D1_PINCTRL_INIT(n)						\
	static const struct d1_pinctrl_cfg d1_cfg_##n = {		\
		.base = DT_INST_REG_ADDR(n),				\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clk_bus = (clock_control_subsys_t)			\
			DT_INST_CLOCKS_CELL_BY_NAME(n, apb, clk_id),	\
	};								\
	DEVICE_DT_INST_DEFINE(n, d1_pinctrl_init, NULL, NULL,		\
			      &d1_cfg_##n, PRE_KERNEL_1,		\
			      40, NULL);

DT_INST_FOREACH_STATUS_OKAY(D1_PINCTRL_INIT)
