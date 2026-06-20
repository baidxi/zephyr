/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/pinctrl/allwinner,sun8i-v3s-pinctrl.h>

/**
 * @brief Pull configuration values for SUN8I-V3S PIO.
 */
#define SUN8I_PULL_NONE		0
#define SUN8I_PULL_UP		1
#define SUN8I_PULL_DOWN		2

/**
 * @brief SUN8I-V3S pin configuration structure.
 *
 * Each pin instance contains:
 *   - port:          Port number (1-6, matching dtsi gpio reg property)
 *   - pin:           Pin number within the port (0-21)
 *   - func:          Function select value (0-7)
 *   - pull:          Pull configuration (SUN8I_PULL_*)
 *   - driving_level: Drive strength level (0-3)
 */
typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t func;
	uint8_t pull;
	uint8_t driving_level;
} pinctrl_soc_pin_t;

/** @cond INTERNAL_HIDDEN */

/**
 * @brief Convert DT allwinner,pull enum string to pull value.
 */
#define SUN8I_PULL_DT(node_id)						       \
	COND_CODE_1(DT_ENUM_EQ(node_id, allwinner_pull, pull_up),	       \
		(SUN8I_PULL_UP),					       \
		(COND_CODE_1(DT_ENUM_EQ(node_id, allwinner_pull, pull_down),  \
			(SUN8I_PULL_DOWN),				       \
			(SUN8I_PULL_NONE))))

/**
 * @brief Macro to initialize a single pin from the pinmux array.
 *
 * Called by Z_PINCTRL_STATE_PINS_INIT via DT_FOREACH_PROP_ELEM.
 *
 * @param node_id Node identifier for the pin configuration group.
 * @param prop    Property name (pinmux).
 * @param idx     Index into the pinmux array.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)		\
	{							\
		.port = SUN8I_V3S_PINMUX_PORT(			\
			DT_PROP_BY_IDX(node_id, prop, idx)),	\
		.pin = SUN8I_V3S_PINMUX_PIN(			\
			DT_PROP_BY_IDX(node_id, prop, idx)),	\
		.func = SUN8I_V3S_PINMUX_FUNC(			\
			DT_PROP_BY_IDX(node_id, prop, idx)),	\
		.pull = SUN8I_PULL_DT(node_id),			\
		.driving_level = DT_PROP_OR(node_id,		\
			drive_strength, 2),			\
	},

/**
 * @brief Macro to initialize all pins for a given pinctrl state.
 *
 * @param node_id Node identifier for the device using pinctrl.
 * @param prop    Property name (e.g., pinctrl_0).
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)		       \
	{ DT_FOREACH_PROP_ELEM(DT_PHANDLE(node_id, prop),	       \
	  pinmux, Z_PINCTRL_STATE_PIN_INIT) }

/** @endcond */
