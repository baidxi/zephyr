/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ALLWINNER_SUN8I_V3S_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ALLWINNER_SUN8I_V3S_PINCTRL_H_

/**
 * @brief Pinmux encoding for Allwinner SUN8I-V3S.
 *
 * Encoding layout:
 *   bits [2:0]  = pin number  (0-7, enough for max 22 pins per port when
 *                               combined with cfg register index)
 *   bits [5:3]  = port number (1-6, matching dtsi reg property)
 *   bits [8:6]  = function    (0-7, 3-bit function select)
 */

#define SUN8I_V3S_PINMUX_PIN_SHIFT   0
#define SUN8I_V3S_PINMUX_PORT_SHIFT  3
#define SUN8I_V3S_PINMUX_FUNC_SHIFT  6

#define SUN8I_V3S_PINMUX(port, pin, func) \
	(((port) << SUN8I_V3S_PINMUX_PORT_SHIFT) | \
	 ((pin) << SUN8I_V3S_PINMUX_PIN_SHIFT) | \
	 ((func) << SUN8I_V3S_PINMUX_FUNC_SHIFT))

#define SUN8I_V3S_PINMUX_PORT(val)  (((val) >> SUN8I_V3S_PINMUX_PORT_SHIFT) & 0x7)
#define SUN8I_V3S_PINMUX_PIN(val)   (((val) >> SUN8I_V3S_PINMUX_PIN_SHIFT) & 0x7)
#define SUN8I_V3S_PINMUX_FUNC(val)  (((val) >> SUN8I_V3S_PINMUX_FUNC_SHIFT) & 0x7)

/* Port numbers matching sun8i-v3s.dtsi gpio reg property */
#define PORT_B	1
#define PORT_C	2
#define PORT_D	3
#define PORT_E	4
#define PORT_F	5
#define PORT_G	6

/* Function select values (verify with V3s datasheet RM0440) */
#define FUNC_GPIO	0

/* Port C functions */
#define FUNC_PC_SDC2	2	/* PC0=CLK, PC1=CMD, PC2=RST, PC3=D0 */
#define FUNC_PC_SPI0	3	/* PC0=MISO, PC1=CLK, PC2=CS, PC3=MOSI */

/* Port F functions */
#define FUNC_PF_UART0	2	/* PF2=TX, PF4=RX */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_ALLWINNER_SUN8I_V3S_PINCTRL_H_ */
