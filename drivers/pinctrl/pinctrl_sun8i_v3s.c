/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>

#define DT_DRV_COMPAT allwinner_sun8i_v3s_pinctrl

#define PORT_STRIDE		0x24

#define CFG_REG_OFF(pin)	((pin / 8) * 4)	/* CFG0..CFG3 */
#define DRV_REG_OFF(pin)	((pin >= 16) ? 0x18 : 0x14) /* DRV0 or DRV1 */
#define PULL_REG_OFF(pin)	((pin >= 16) ? 0x20 : 0x1C) /* PULL0 or PULL1 */
#define CFG_PIN_SHIFT(pin)	((pin % 8) * 4)
#define CFG_PIN_MASK(pin)	(0x7 << CFG_PIN_SHIFT(pin))
#define DRV_PIN_SHIFT(pin)	(((pin >= 16) ? (pin - 16) : pin) * 2)
#define DRV_PIN_MASK(pin)	(0x3 << DRV_PIN_SHIFT(pin))
#define PULL_PIN_SHIFT(pin)	(((pin >= 16) ? (pin - 16) : pin) * 2)
#define PULL_PIN_MASK(pin)	(0x3 << PULL_PIN_SHIFT(pin))

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins,
			   uint8_t pin_cnt,
			   uintptr_t reg)
{
	if (!reg) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *pin = &pins[i];
		uint32_t port_base = reg + PORT_STRIDE * pin->port;
		uint32_t val;

		val = sys_read32(port_base + CFG_REG_OFF(pin->pin));
		val &= ~CFG_PIN_MASK(pin->pin);
		val |= (pin->func << CFG_PIN_SHIFT(pin->pin));
		sys_write32(val, port_base + CFG_REG_OFF(pin->pin));

		val = sys_read32(port_base + DRV_REG_OFF(pin->pin));
		val &= ~DRV_PIN_MASK(pin->pin);
		val |= (pin->driving_level << DRV_PIN_SHIFT(pin->pin));
		sys_write32(val, port_base + DRV_REG_OFF(pin->pin));

		val = sys_read32(port_base + PULL_REG_OFF(pin->pin));
		val &= ~PULL_PIN_MASK(pin->pin);
		val |= (pin->pull << PULL_PIN_SHIFT(pin->pin));
		sys_write32(val, port_base + PULL_REG_OFF(pin->pin));
	}

	return 0;
}
