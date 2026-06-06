/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Based on Linux kernel include/dt-bindings/clock/sun8i-de2.h
 */

#ifndef ZEPHYR_DT_BINDINGS_CLOCK_SUN8I_DE2_H_
#define ZEPHYR_DT_BINDINGS_CLOCK_SUN8I_DE2_H_

/*
 * Clock and reset IDs for Allwinner DE2 Display Engine Clock Controller.
 *
 * The DE2 CCU sits at 0x01000000 and provides:
 *  - Bus clock gates for each mixer/wb/rot module
 *  - Module clock gates and dividers
 *  - Reset lines
 */

/* Bus clock gates (register 0x04) */
#define CLK_BUS_MIXER0		0
#define CLK_BUS_MIXER1		1
#define CLK_BUS_WB		2

/* Module clock gates (register 0x00) */
#define CLK_MIXER0		6
#define CLK_MIXER1		7
#define CLK_WB			8

/* Rotation engine clocks */
#define CLK_BUS_ROT		9
#define CLK_ROT			10

#endif /* ZEPHYR_DT_BINDINGS_CLOCK_SUN8I_DE2_H_ */
