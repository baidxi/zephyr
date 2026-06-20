/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 / D1 pinctrl soc glue.
 *
 * DTS pin groups carry a flat "pinmux" array of SUNXI_PINMUX(pin, func)
 * cells (see allwinner,sun20i-d1-pinctrl.h).  Each cell encodes bank, pin
 * and the per-(pin,function) SELECT value.  This header defines how one cell
 * is expanded into the pinctrl_soc_pin_t consumed by pinctrl_sun20i_d1.c.
 */

#ifndef ZEPHYR_SOC_ALLWINNER_SUN8I_T113S3_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ALLWINNER_SUN8I_T113S3_PINCTRL_SOC_H_

#include <zephyr/dt-bindings/pinctrl/allwinner,sun20i-d1-pinctrl.h>
#include <zephyr/types.h>

typedef struct pinctrl_soc_pin {
	/* bank: PB=1 ... PG=6 (matches PIO port stride) */
	uint32_t port : 3;
	uint32_t pin  : 5;		/* 0..31 */
	uint32_t func : 4;		/* PX_CFGx PXn_SELECT, 0..15 */
	uint32_t driving_level : 2;
	uint32_t pull : 2;		/* 0=none, 1=up, 2=down */
} pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)			\
	{								\
	 .port = SUNXI_PINMUX_BANK(DT_PROP_BY_IDX(node_id, prop, idx)), \
	 .pin  = SUNXI_PINMUX_PIN(DT_PROP_BY_IDX(node_id, prop, idx)),  \
	 .func = SUNXI_PINMUX_FUNC(DT_PROP_BY_IDX(node_id, prop, idx)), \
	 .driving_level = DT_PROP_OR(node_id, drive_strength, 2),	\
	 .pull = DT_ENUM_IDX_OR(node_id, allwinner_pull, 0),		\
	},

/*
 * Iterate the pinmux array that lives DIRECTLY on the pin-group node
 * (e.g. mmc0_pins { pinmux = <...>, <...>, ...; }).  Matches the DTS style
 * used in sun8i-t113s3.dtsi.  (The previous DT_FOREACH_CHILD_VARGS form
 * produced zero pins for this flat-array style, silently leaving PF0-5 at
 * the boot-ROM default and breaking SD card CMD/CLK.)
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			\
	{DT_FOREACH_PROP_ELEM(DT_PHANDLE(node_id, prop),		\
	 pinmux, Z_PINCTRL_STATE_PIN_INIT)}

#endif /* ZEPHYR_SOC_ALLWINNER_SUN8I_T113S3_PINCTRL_SOC_H_ */
