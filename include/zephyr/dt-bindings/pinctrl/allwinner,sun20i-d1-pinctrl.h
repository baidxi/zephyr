/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner D1 / T113-S3 pinctrl dt-bindings.
 *
 * Usage in DTS -- readable, datasheet-driven, no magic constants:
 *
 *     pinmux = <SUNXI_PINMUX(PF2, SUN8I_FUNC_MMC0)>,    // PF2 -> mmc0
 *              <SUNXI_PINMUX(PG0, SUN8I_FUNC_UART3)>,   // PG0 -> uart3 (SELECT 3)
 *              <SUNXI_PINMUX(PG8, SUN8I_FUNC_UART3)>;   // PG8 -> uart3 (SELECT 5)
 *
 * 1st arg  = pin token (PF2, PB10, ...).
 * 2nd arg  = function token (SUN8I_FUNC_MMC0, SUN8I_FUNC_UART3, ...).
 *
 * SUNXI_PINMUX() resolves the PX_CFGx SELECT for that exact (pin, function)
 * via token pasting (SUNXI_FUNC_SEL__PXn__SUN8I_FUNC_xxx) and packs
 * bank/pin/SELECT into one 32-bit cell consumed by pinctrl_sun20i_d1.c:
 *
 *   bits[4:0]  = pin   (0..31)
 *   bits[7:5]  = bank  (PB=1 .. PG=6)
 *   bits[11:8] = SELECT(0..15)  -- PER-PIN, see note below
 *
 * Why per-(pin,function): the SAME peripheral uses a DIFFERENT SELECT on
 * different pins, e.g. uart3 = SELECT 7 on PB6, 4 on PC6, 3 on PG0, 5 on PG8.
 * So there is no single FUNC_UART3 value; SELECT is looked up per pin.  An
 * invalid (pin,function) combination is a compile error (token undefined).
 *
 * Generated from the authoritative Linux driver pinctrl-sun20i-d1.c via:
 *   .roo/skills/sunxi-pinctrl-gen/scripts/gen_sunxi_pinctrl_macros.py
 */

#ifndef ZEPHYR_DT_BINDINGS_PINCTRL_ALLWINNER_SUN20I_D1_PINCTRL_H_
#define ZEPHYR_DT_BINDINGS_PINCTRL_ALLWINNER_SUN20I_D1_PINCTRL_H_

#ifndef _ASMLANGUAGE

/*
 * === BEGIN AUTO-GENERATED (do not edit by hand) ===
 * Source: linux/drivers/pinctrl/sunxi/pinctrl-sun20i-d1.c
 * Regenerator: .roo/skills/sunxi-pinctrl-gen/scripts/gen_sunxi_pinctrl_macros.py
 *
 * The SELECT values below are the first argument of each
 * SUNXI_FUNCTION(0xN, "name") declaration -- i.e. the raw
 * PX_CFGx PXn_SELECT field value for that (pin, function).
 * The SAME peripheral may use DIFFERENT SELECT on different pins.
 */

/* Pin tokens: PXn encodes (bank<<5)|pin. bank: PB=1 .. PG=6. */
#define PB0   ((1U << 5) | 0U)
#define PB1   ((1U << 5) | 1U)
#define PB2   ((1U << 5) | 2U)
#define PB3   ((1U << 5) | 3U)
#define PB4   ((1U << 5) | 4U)
#define PB5   ((1U << 5) | 5U)
#define PB6   ((1U << 5) | 6U)
#define PB7   ((1U << 5) | 7U)
#define PB8   ((1U << 5) | 8U)
#define PB9   ((1U << 5) | 9U)
#define PB10  ((1U << 5) | 10U)
#define PB11  ((1U << 5) | 11U)
#define PB12  ((1U << 5) | 12U)

#define PC0   ((2U << 5) | 0U)
#define PC1   ((2U << 5) | 1U)
#define PC2   ((2U << 5) | 2U)
#define PC3   ((2U << 5) | 3U)
#define PC4   ((2U << 5) | 4U)
#define PC5   ((2U << 5) | 5U)
#define PC6   ((2U << 5) | 6U)
#define PC7   ((2U << 5) | 7U)

#define PD0   ((3U << 5) | 0U)
#define PD1   ((3U << 5) | 1U)
#define PD2   ((3U << 5) | 2U)
#define PD3   ((3U << 5) | 3U)
#define PD4   ((3U << 5) | 4U)
#define PD5   ((3U << 5) | 5U)
#define PD6   ((3U << 5) | 6U)
#define PD7   ((3U << 5) | 7U)
#define PD8   ((3U << 5) | 8U)
#define PD9   ((3U << 5) | 9U)
#define PD10  ((3U << 5) | 10U)
#define PD11  ((3U << 5) | 11U)
#define PD12  ((3U << 5) | 12U)
#define PD13  ((3U << 5) | 13U)
#define PD14  ((3U << 5) | 14U)
#define PD15  ((3U << 5) | 15U)
#define PD16  ((3U << 5) | 16U)
#define PD17  ((3U << 5) | 17U)
#define PD18  ((3U << 5) | 18U)
#define PD19  ((3U << 5) | 19U)
#define PD20  ((3U << 5) | 20U)
#define PD21  ((3U << 5) | 21U)
#define PD22  ((3U << 5) | 22U)

#define PE0   ((4U << 5) | 0U)
#define PE1   ((4U << 5) | 1U)
#define PE2   ((4U << 5) | 2U)
#define PE3   ((4U << 5) | 3U)
#define PE4   ((4U << 5) | 4U)
#define PE5   ((4U << 5) | 5U)
#define PE6   ((4U << 5) | 6U)
#define PE7   ((4U << 5) | 7U)
#define PE8   ((4U << 5) | 8U)
#define PE9   ((4U << 5) | 9U)
#define PE10  ((4U << 5) | 10U)
#define PE11  ((4U << 5) | 11U)
#define PE12  ((4U << 5) | 12U)
#define PE13  ((4U << 5) | 13U)
#define PE14  ((4U << 5) | 14U)
#define PE15  ((4U << 5) | 15U)
#define PE16  ((4U << 5) | 16U)
#define PE17  ((4U << 5) | 17U)

#define PF0   ((5U << 5) | 0U)
#define PF1   ((5U << 5) | 1U)
#define PF2   ((5U << 5) | 2U)
#define PF3   ((5U << 5) | 3U)
#define PF4   ((5U << 5) | 4U)
#define PF5   ((5U << 5) | 5U)
#define PF6   ((5U << 5) | 6U)

#define PG0   ((6U << 5) | 0U)
#define PG1   ((6U << 5) | 1U)
#define PG2   ((6U << 5) | 2U)
#define PG3   ((6U << 5) | 3U)
#define PG4   ((6U << 5) | 4U)
#define PG5   ((6U << 5) | 5U)
#define PG6   ((6U << 5) | 6U)
#define PG7   ((6U << 5) | 7U)
#define PG8   ((6U << 5) | 8U)
#define PG9   ((6U << 5) | 9U)
#define PG10  ((6U << 5) | 10U)
#define PG11  ((6U << 5) | 11U)
#define PG12  ((6U << 5) | 12U)
#define PG13  ((6U << 5) | 13U)
#define PG14  ((6U << 5) | 14U)
#define PG15  ((6U << 5) | 15U)
#define PG16  ((6U << 5) | 16U)
#define PG17  ((6U << 5) | 17U)
#define PG18  ((6U << 5) | 18U)

/* Function tokens (logical names, NOT SELECT values). */
#define SUN8I_FUNC_BIST0             0
#define SUN8I_FUNC_BIST1             1
#define SUN8I_FUNC_BOOT              2
#define SUN8I_FUNC_CAN0              3
#define SUN8I_FUNC_CAN1              4
#define SUN8I_FUNC_CLK               5
#define SUN8I_FUNC_D_JTAG            6
#define SUN8I_FUNC_DMIC              7
#define SUN8I_FUNC_DSI               8
#define SUN8I_FUNC_EMAC              9
#define SUN8I_FUNC_GPIO_IN           10
#define SUN8I_FUNC_GPIO_OUT          11
#define SUN8I_FUNC_I2C0              12
#define SUN8I_FUNC_I2C1              13
#define SUN8I_FUNC_I2C2              14
#define SUN8I_FUNC_I2C3              15
#define SUN8I_FUNC_I2S0              16
#define SUN8I_FUNC_I2S0_DIN          17
#define SUN8I_FUNC_I2S0_DOUT         18
#define SUN8I_FUNC_I2S1              19
#define SUN8I_FUNC_I2S1_DIN          20
#define SUN8I_FUNC_I2S1_DOUT         21
#define SUN8I_FUNC_I2S2              22
#define SUN8I_FUNC_I2S2_DIN          23
#define SUN8I_FUNC_I2S2_DOUT         24
#define SUN8I_FUNC_IR                25
#define SUN8I_FUNC_JTAG              26
#define SUN8I_FUNC_LCD0              27
#define SUN8I_FUNC_LEDC              28
#define SUN8I_FUNC_LVDS0             29
#define SUN8I_FUNC_LVDS1             30
#define SUN8I_FUNC_MMC0              31
#define SUN8I_FUNC_MMC1              32
#define SUN8I_FUNC_MMC2              33
#define SUN8I_FUNC_NCSI0             34
#define SUN8I_FUNC_PLL               35
#define SUN8I_FUNC_PWM0              36
#define SUN8I_FUNC_PWM1              37
#define SUN8I_FUNC_PWM2              38
#define SUN8I_FUNC_PWM3              39
#define SUN8I_FUNC_PWM4              40
#define SUN8I_FUNC_PWM5              41
#define SUN8I_FUNC_PWM6              42
#define SUN8I_FUNC_PWM7              43
#define SUN8I_FUNC_R_JTAG            44
#define SUN8I_FUNC_SPDIF             45
#define SUN8I_FUNC_SPI0              46
#define SUN8I_FUNC_SPI1              47
#define SUN8I_FUNC_TCON              48
#define SUN8I_FUNC_UART0             49
#define SUN8I_FUNC_UART1             50
#define SUN8I_FUNC_UART2             51
#define SUN8I_FUNC_UART3             52
#define SUN8I_FUNC_UART4             53
#define SUN8I_FUNC_UART5             54

/*
 * (pin, function) -> SELECT table.
 * SUNXI_FUNC_SEL(PF2, SUN8I_FUNC_MMC0) expands to the SELECT value
 * for PF2+mmc0 (here 0x2).  An undefined combination is a compile
 * error, which catches wrong pin/function assignments at build time.
 */
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_I2C2              0x4
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_IR                0x3
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_PWM3              0x2
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_SPDIF             0x8
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_SPI1              0x5
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_UART0             0x6
#define SUNXI_FUNC_SEL__PB0__SUN8I_FUNC_UART2             0x7

#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_I2C2              0x4
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_I2S2_DIN          0x5
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_I2S2_DOUT         0x3
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_IR                0x8
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_PWM4              0x2
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_UART0             0x6
#define SUNXI_FUNC_SEL__PB1__SUN8I_FUNC_UART2             0x7

#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_CAN0              0x8
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_I2S2_DIN          0x5
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_I2S2_DOUT         0x3
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_LCD0              0x6
#define SUNXI_FUNC_SEL__PB2__SUN8I_FUNC_UART4             0x7

#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_CAN0              0x8
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_I2S2_DIN          0x5
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_I2S2_DOUT         0x3
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_LCD0              0x6
#define SUNXI_FUNC_SEL__PB3__SUN8I_FUNC_UART4             0x7

#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_CAN1              0x8
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_I2C1              0x4
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_I2S2_DIN          0x5
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_I2S2_DOUT         0x3
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_LCD0              0x6
#define SUNXI_FUNC_SEL__PB4__SUN8I_FUNC_UART5             0x7

#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_CAN1              0x8
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_I2C1              0x4
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_I2S2              0x3
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_LCD0              0x6
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_PWM0              0x5
#define SUNXI_FUNC_SEL__PB5__SUN8I_FUNC_UART5             0x7

#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_BIST0             0x8
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_I2C3              0x4
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_I2S2              0x3
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_LCD0              0x6
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_PWM1              0x5
#define SUNXI_FUNC_SEL__PB6__SUN8I_FUNC_UART3             0x7

#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_BIST1             0x8
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_I2C3              0x4
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_I2S2              0x3
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_IR                0x5
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_LCD0              0x6
#define SUNXI_FUNC_SEL__PB7__SUN8I_FUNC_UART3             0x7

#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_DMIC              0x2
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_I2C2              0x4
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_PWM5              0x3
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_SPI1              0x5
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_UART0             0x6
#define SUNXI_FUNC_SEL__PB8__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_DMIC              0x2
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_I2C2              0x4
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_PWM6              0x3
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_SPI1              0x5
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_UART0             0x6
#define SUNXI_FUNC_SEL__PB9__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_CLK               0x6
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_DMIC              0x2
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_PWM7              0x3
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_SPI1              0x5
#define SUNXI_FUNC_SEL__PB10__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_CLK               0x6
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_DMIC              0x2
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_PWM2              0x3
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_SPI1              0x5
#define SUNXI_FUNC_SEL__PB11__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_CLK               0x6
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_DMIC              0x2
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_IR                0x7
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_PWM0              0x3
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_SPDIF             0x4
#define SUNXI_FUNC_SEL__PB12__SUN8I_FUNC_SPI1              0x5

#define SUNXI_FUNC_SEL__PC0__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC0__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC0__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PC0__SUN8I_FUNC_LEDC              0x4
#define SUNXI_FUNC_SEL__PC0__SUN8I_FUNC_UART2             0x2

#define SUNXI_FUNC_SEL__PC1__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC1__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC1__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PC1__SUN8I_FUNC_UART2             0x2

#define SUNXI_FUNC_SEL__PC2__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC2__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC2__SUN8I_FUNC_MMC2              0x3
#define SUNXI_FUNC_SEL__PC2__SUN8I_FUNC_SPI0              0x2

#define SUNXI_FUNC_SEL__PC3__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC3__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC3__SUN8I_FUNC_MMC2              0x3
#define SUNXI_FUNC_SEL__PC3__SUN8I_FUNC_SPI0              0x2

#define SUNXI_FUNC_SEL__PC4__SUN8I_FUNC_BOOT              0x4
#define SUNXI_FUNC_SEL__PC4__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC4__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC4__SUN8I_FUNC_MMC2              0x3
#define SUNXI_FUNC_SEL__PC4__SUN8I_FUNC_SPI0              0x2

#define SUNXI_FUNC_SEL__PC5__SUN8I_FUNC_BOOT              0x4
#define SUNXI_FUNC_SEL__PC5__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC5__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC5__SUN8I_FUNC_MMC2              0x3
#define SUNXI_FUNC_SEL__PC5__SUN8I_FUNC_SPI0              0x2

#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_I2C3              0x5
#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_MMC2              0x3
#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_PLL               0x6
#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_SPI0              0x2
#define SUNXI_FUNC_SEL__PC6__SUN8I_FUNC_UART3             0x4

#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_I2C3              0x5
#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_MMC2              0x3
#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_SPI0              0x2
#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_TCON              0x6
#define SUNXI_FUNC_SEL__PC7__SUN8I_FUNC_UART3             0x4

#define SUNXI_FUNC_SEL__PD0__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD0__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD0__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD0__SUN8I_FUNC_I2C0              0x5
#define SUNXI_FUNC_SEL__PD0__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD0__SUN8I_FUNC_LVDS0             0x3

#define SUNXI_FUNC_SEL__PD1__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD1__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD1__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD1__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD1__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD1__SUN8I_FUNC_UART2             0x5

#define SUNXI_FUNC_SEL__PD2__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD2__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD2__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD2__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD2__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD2__SUN8I_FUNC_UART2             0x5

#define SUNXI_FUNC_SEL__PD3__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD3__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD3__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD3__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD3__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD3__SUN8I_FUNC_UART2             0x5

#define SUNXI_FUNC_SEL__PD4__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD4__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD4__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD4__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD4__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD4__SUN8I_FUNC_UART2             0x5

#define SUNXI_FUNC_SEL__PD5__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD5__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD5__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD5__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD5__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD5__SUN8I_FUNC_UART5             0x5

#define SUNXI_FUNC_SEL__PD6__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD6__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD6__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD6__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD6__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD6__SUN8I_FUNC_UART5             0x5

#define SUNXI_FUNC_SEL__PD7__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD7__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD7__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD7__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD7__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD7__SUN8I_FUNC_UART4             0x5

#define SUNXI_FUNC_SEL__PD8__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD8__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD8__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD8__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD8__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD8__SUN8I_FUNC_UART4             0x5

#define SUNXI_FUNC_SEL__PD9__SUN8I_FUNC_DSI               0x4
#define SUNXI_FUNC_SEL__PD9__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD9__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD9__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD9__SUN8I_FUNC_LVDS0             0x3
#define SUNXI_FUNC_SEL__PD9__SUN8I_FUNC_PWM6              0x5

#define SUNXI_FUNC_SEL__PD10__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD10__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD10__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD10__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD10__SUN8I_FUNC_SPI1              0x4
#define SUNXI_FUNC_SEL__PD10__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PD11__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD11__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD11__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD11__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD11__SUN8I_FUNC_SPI1              0x4
#define SUNXI_FUNC_SEL__PD11__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PD12__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD12__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD12__SUN8I_FUNC_I2C0              0x5
#define SUNXI_FUNC_SEL__PD12__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD12__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD12__SUN8I_FUNC_SPI1              0x4

#define SUNXI_FUNC_SEL__PD13__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD13__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD13__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD13__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD13__SUN8I_FUNC_SPI1              0x4
#define SUNXI_FUNC_SEL__PD13__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PD14__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD14__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD14__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD14__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD14__SUN8I_FUNC_SPI1              0x4
#define SUNXI_FUNC_SEL__PD14__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PD15__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD15__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD15__SUN8I_FUNC_IR                0x5
#define SUNXI_FUNC_SEL__PD15__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD15__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD15__SUN8I_FUNC_SPI1              0x4

#define SUNXI_FUNC_SEL__PD16__SUN8I_FUNC_DMIC              0x4
#define SUNXI_FUNC_SEL__PD16__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD16__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD16__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD16__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD16__SUN8I_FUNC_PWM0              0x5

#define SUNXI_FUNC_SEL__PD17__SUN8I_FUNC_DMIC              0x4
#define SUNXI_FUNC_SEL__PD17__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD17__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD17__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD17__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD17__SUN8I_FUNC_PWM1              0x5

#define SUNXI_FUNC_SEL__PD18__SUN8I_FUNC_DMIC              0x4
#define SUNXI_FUNC_SEL__PD18__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD18__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD18__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD18__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD18__SUN8I_FUNC_PWM2              0x5

#define SUNXI_FUNC_SEL__PD19__SUN8I_FUNC_DMIC              0x4
#define SUNXI_FUNC_SEL__PD19__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD19__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD19__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD19__SUN8I_FUNC_LVDS1             0x3
#define SUNXI_FUNC_SEL__PD19__SUN8I_FUNC_PWM3              0x5

#define SUNXI_FUNC_SEL__PD20__SUN8I_FUNC_DMIC              0x4
#define SUNXI_FUNC_SEL__PD20__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD20__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD20__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PD20__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD20__SUN8I_FUNC_PWM4              0x5

#define SUNXI_FUNC_SEL__PD21__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD21__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD21__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PD21__SUN8I_FUNC_LCD0              0x2
#define SUNXI_FUNC_SEL__PD21__SUN8I_FUNC_PWM5              0x5
#define SUNXI_FUNC_SEL__PD21__SUN8I_FUNC_UART1             0x4

#define SUNXI_FUNC_SEL__PD22__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PD22__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PD22__SUN8I_FUNC_IR                0x3
#define SUNXI_FUNC_SEL__PD22__SUN8I_FUNC_PWM7              0x5
#define SUNXI_FUNC_SEL__PD22__SUN8I_FUNC_SPDIF             0x2
#define SUNXI_FUNC_SEL__PD22__SUN8I_FUNC_UART1             0x4

#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_I2C1              0x4
#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_LCD0              0x5
#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE0__SUN8I_FUNC_UART2             0x3

#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_I2C1              0x4
#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_LCD0              0x5
#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE1__SUN8I_FUNC_UART2             0x3

#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_UART0             0x6
#define SUNXI_FUNC_SEL__PE2__SUN8I_FUNC_UART2             0x3

#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_UART0             0x6
#define SUNXI_FUNC_SEL__PE3__SUN8I_FUNC_UART2             0x3

#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_D_JTAG            0x6
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_I2C2              0x4
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_R_JTAG            0x7
#define SUNXI_FUNC_SEL__PE4__SUN8I_FUNC_UART4             0x3

#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_D_JTAG            0x6
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_I2C2              0x4
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_LEDC              0x5
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_R_JTAG            0x7
#define SUNXI_FUNC_SEL__PE5__SUN8I_FUNC_UART4             0x3

#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_D_JTAG            0x6
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_I2C3              0x4
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_R_JTAG            0x7
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_SPDIF             0x5
#define SUNXI_FUNC_SEL__PE6__SUN8I_FUNC_UART5             0x3

#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_D_JTAG            0x6
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_I2C3              0x4
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_R_JTAG            0x7
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_SPDIF             0x5
#define SUNXI_FUNC_SEL__PE7__SUN8I_FUNC_UART5             0x3

#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_JTAG              0x6
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_PWM2              0x4
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_UART1             0x3
#define SUNXI_FUNC_SEL__PE8__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_JTAG              0x6
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_PWM3              0x4
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_UART1             0x3
#define SUNXI_FUNC_SEL__PE9__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_IR                0x5
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_JTAG              0x6
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_PWM4              0x4
#define SUNXI_FUNC_SEL__PE10__SUN8I_FUNC_UART1             0x3

#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_I2S0_DIN          0x5
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_I2S0_DOUT         0x4
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_JTAG              0x6
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_NCSI0             0x2
#define SUNXI_FUNC_SEL__PE11__SUN8I_FUNC_UART1             0x3

#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_I2C2              0x2
#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_I2S0_DIN          0x5
#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_I2S0_DOUT         0x4
#define SUNXI_FUNC_SEL__PE12__SUN8I_FUNC_NCSI0             0x3

#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_DMIC              0x6
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_I2C2              0x2
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_I2S0_DIN          0x5
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_I2S0_DOUT         0x4
#define SUNXI_FUNC_SEL__PE13__SUN8I_FUNC_PWM5              0x3

#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_D_JTAG            0x3
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_DMIC              0x6
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_I2C1              0x2
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_I2S0_DIN          0x5
#define SUNXI_FUNC_SEL__PE14__SUN8I_FUNC_I2S0_DOUT         0x4

#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_D_JTAG            0x3
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_DMIC              0x6
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_EMAC              0x8
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_I2C1              0x2
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_I2S0              0x5
#define SUNXI_FUNC_SEL__PE15__SUN8I_FUNC_PWM6              0x4

#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_D_JTAG            0x3
#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_DMIC              0x6
#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_I2C3              0x2
#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_I2S0              0x5
#define SUNXI_FUNC_SEL__PE16__SUN8I_FUNC_PWM7              0x4

#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_D_JTAG            0x3
#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_DMIC              0x6
#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_I2C3              0x2
#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_I2S0              0x5
#define SUNXI_FUNC_SEL__PE17__SUN8I_FUNC_IR                0x4

#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_I2S2_DIN          0x6
#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_I2S2_DOUT         0x5
#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_JTAG              0x3
#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_MMC0              0x2
#define SUNXI_FUNC_SEL__PF0__SUN8I_FUNC_R_JTAG            0x4

#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_I2S2_DIN          0x6
#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_I2S2_DOUT         0x5
#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_JTAG              0x3
#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_MMC0              0x2
#define SUNXI_FUNC_SEL__PF1__SUN8I_FUNC_R_JTAG            0x4

#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_LEDC              0x5
#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_MMC0              0x2
#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_SPDIF             0x6
#define SUNXI_FUNC_SEL__PF2__SUN8I_FUNC_UART0             0x3

#define SUNXI_FUNC_SEL__PF3__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF3__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF3__SUN8I_FUNC_I2S2              0x5
#define SUNXI_FUNC_SEL__PF3__SUN8I_FUNC_JTAG              0x3
#define SUNXI_FUNC_SEL__PF3__SUN8I_FUNC_MMC0              0x2
#define SUNXI_FUNC_SEL__PF3__SUN8I_FUNC_R_JTAG            0x4

#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_I2C0              0x4
#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_IR                0x6
#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_MMC0              0x2
#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_PWM6              0x5
#define SUNXI_FUNC_SEL__PF4__SUN8I_FUNC_UART0             0x3

#define SUNXI_FUNC_SEL__PF5__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF5__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF5__SUN8I_FUNC_I2S2              0x5
#define SUNXI_FUNC_SEL__PF5__SUN8I_FUNC_JTAG              0x3
#define SUNXI_FUNC_SEL__PF5__SUN8I_FUNC_MMC0              0x2
#define SUNXI_FUNC_SEL__PF5__SUN8I_FUNC_R_JTAG            0x4

#define SUNXI_FUNC_SEL__PF6__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PF6__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PF6__SUN8I_FUNC_I2S2              0x5
#define SUNXI_FUNC_SEL__PF6__SUN8I_FUNC_IR                0x4
#define SUNXI_FUNC_SEL__PF6__SUN8I_FUNC_PWM5              0x6
#define SUNXI_FUNC_SEL__PF6__SUN8I_FUNC_SPDIF             0x3

#define SUNXI_FUNC_SEL__PG0__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG0__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG0__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG0__SUN8I_FUNC_MMC1              0x2
#define SUNXI_FUNC_SEL__PG0__SUN8I_FUNC_PWM7              0x5
#define SUNXI_FUNC_SEL__PG0__SUN8I_FUNC_UART3             0x3

#define SUNXI_FUNC_SEL__PG1__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG1__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG1__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG1__SUN8I_FUNC_MMC1              0x2
#define SUNXI_FUNC_SEL__PG1__SUN8I_FUNC_PWM6              0x5
#define SUNXI_FUNC_SEL__PG1__SUN8I_FUNC_UART3             0x3

#define SUNXI_FUNC_SEL__PG2__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG2__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG2__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG2__SUN8I_FUNC_MMC1              0x2
#define SUNXI_FUNC_SEL__PG2__SUN8I_FUNC_UART3             0x3
#define SUNXI_FUNC_SEL__PG2__SUN8I_FUNC_UART4             0x5

#define SUNXI_FUNC_SEL__PG3__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG3__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG3__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG3__SUN8I_FUNC_MMC1              0x2
#define SUNXI_FUNC_SEL__PG3__SUN8I_FUNC_UART3             0x3
#define SUNXI_FUNC_SEL__PG3__SUN8I_FUNC_UART4             0x5

#define SUNXI_FUNC_SEL__PG4__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG4__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG4__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG4__SUN8I_FUNC_MMC1              0x2
#define SUNXI_FUNC_SEL__PG4__SUN8I_FUNC_PWM5              0x5
#define SUNXI_FUNC_SEL__PG4__SUN8I_FUNC_UART5             0x3

#define SUNXI_FUNC_SEL__PG5__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG5__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG5__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG5__SUN8I_FUNC_MMC1              0x2
#define SUNXI_FUNC_SEL__PG5__SUN8I_FUNC_PWM4              0x5
#define SUNXI_FUNC_SEL__PG5__SUN8I_FUNC_UART5             0x3

#define SUNXI_FUNC_SEL__PG6__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG6__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG6__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG6__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PG6__SUN8I_FUNC_PWM1              0x5
#define SUNXI_FUNC_SEL__PG6__SUN8I_FUNC_UART1             0x2

#define SUNXI_FUNC_SEL__PG7__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG7__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG7__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG7__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PG7__SUN8I_FUNC_SPDIF             0x5
#define SUNXI_FUNC_SEL__PG7__SUN8I_FUNC_UART1             0x2

#define SUNXI_FUNC_SEL__PG8__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG8__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG8__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG8__SUN8I_FUNC_I2C1              0x3
#define SUNXI_FUNC_SEL__PG8__SUN8I_FUNC_UART1             0x2
#define SUNXI_FUNC_SEL__PG8__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PG9__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG9__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG9__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG9__SUN8I_FUNC_I2C1              0x3
#define SUNXI_FUNC_SEL__PG9__SUN8I_FUNC_UART1             0x2
#define SUNXI_FUNC_SEL__PG9__SUN8I_FUNC_UART3             0x5

#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_I2C3              0x3
#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_IR                0x6
#define SUNXI_FUNC_SEL__PG10__SUN8I_FUNC_PWM3              0x2

#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_I2C3              0x3
#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_I2S1              0x2
#define SUNXI_FUNC_SEL__PG11__SUN8I_FUNC_TCON              0x6

#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_I2C0              0x3
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_I2S1              0x2
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_PWM0              0x6
#define SUNXI_FUNC_SEL__PG12__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_I2C0              0x3
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_I2S1              0x2
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_LEDC              0x6
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_PWM2              0x5
#define SUNXI_FUNC_SEL__PG13__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_I2S1_DIN          0x2
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_I2S1_DOUT         0x5
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_SPI0              0x6
#define SUNXI_FUNC_SEL__PG14__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_EMAC              0x4
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_I2C2              0x3
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_I2S1_DIN          0x5
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_I2S1_DOUT         0x2
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_SPI0              0x6
#define SUNXI_FUNC_SEL__PG15__SUN8I_FUNC_UART1             0x7

#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_IR                0x2
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_LEDC              0x7
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_PWM5              0x4
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_SPDIF             0x6
#define SUNXI_FUNC_SEL__PG16__SUN8I_FUNC_TCON              0x3

#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_I2C3              0x3
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_IR                0x6
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_PWM7              0x4
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_UART0             0x7
#define SUNXI_FUNC_SEL__PG17__SUN8I_FUNC_UART2             0x2

#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_CLK               0x5
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_GPIO_IN           0x0
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_GPIO_OUT          0x1
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_I2C3              0x3
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_PWM6              0x4
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_SPDIF             0x6
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_UART0             0x7
#define SUNXI_FUNC_SEL__PG18__SUN8I_FUNC_UART2             0x2

/*
 * PINMUX cell encoding (consumed by pinctrl_sun20i_d1.c):
 *   bits[4:0]  = pin   (0..31)
 *   bits[7:5]  = bank  (0..7)
 *   bits[11:8] = SELECT(0..15)  -- PX_CFGx PXn_SELECT, per-pin
 *
 * pin_tok/func_tok are DIRECT operands of ##, so they are NOT
 * macro-expanded: they paste as bare tokens to form the
 * SUNXI_FUNC_SEL__PXn__SUN8I_FUNC_xxx key.  The standalone (pin_tok)
 * IS expanded to its (bank<<5)|pin value.  An undefined
 * (pin,function) combination yields an undefined token -> compile error.
 */
#define SUNXI_PINMUX(pin_tok, func_tok) \
    ((pin_tok) | (SUNXI_FUNC_SEL__ ## pin_tok ## __ ## func_tok << 8))
#define SUNXI_PINMUX_BANK(v)  (((v) >> 5) & 0x7U)
#define SUNXI_PINMUX_PIN(v)   ((v) & 0x1FU)
#define SUNXI_PINMUX_FUNC(v)  (((v) >> 8) & 0xFU)   /* SELECT, 0..15 */

/* === END AUTO-GENERATED === */

#endif /* !_ASMLANGUAGE */

#endif /* ZEPHYR_DT_BINDINGS_PINCTRL_ALLWINNER_SUN20I_D1_PINCTRL_H_ */
