/*
 * Copyright (c) 2020 huangzhenwei@allwinnertech.com
 * Copyright (C) 2021 Samuel Holland <samuel@sholland.org>
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reset IDs for Allwinner D1 / T113-S3 CCU.
 * Aligned with Linux dt-bindings <dt-bindings/reset/sun20i-d1-ccu.h>.
 * D1_RST_ prefix avoids collision with other SoC headers.
 */

#ifndef ZEPHYR_DT_BINDINGS_RESET_SUN20I_D1_CCU_H_
#define ZEPHYR_DT_BINDINGS_RESET_SUN20I_D1_CCU_H_

#define D1_RST_MBUS		0
#define D1_RST_BUS_DE		1
#define D1_RST_BUS_DI		2
#define D1_RST_BUS_G2D		3
#define D1_RST_BUS_CE		4
#define D1_RST_BUS_VE		5
#define D1_RST_BUS_DMA		6
#define D1_RST_BUS_MSGBOX0	7
#define D1_RST_BUS_MSGBOX1	8
#define D1_RST_BUS_MSGBOX2	9
#define D1_RST_BUS_SPINLOCK	10
#define D1_RST_BUS_HSTIMER	11
#define D1_RST_BUS_DBG		12
#define D1_RST_BUS_PWM		13
#define D1_RST_BUS_DRAM		14
#define D1_RST_BUS_MMC0		15
#define D1_RST_BUS_MMC1		16
#define D1_RST_BUS_MMC2		17
#define D1_RST_BUS_UART0	18
#define D1_RST_BUS_UART1	19
#define D1_RST_BUS_UART2	20
#define D1_RST_BUS_UART3	21
#define D1_RST_BUS_UART4	22
#define D1_RST_BUS_UART5	23
#define D1_RST_BUS_I2C0		24
#define D1_RST_BUS_I2C1		25
#define D1_RST_BUS_I2C2		26
#define D1_RST_BUS_I2C3		27
#define D1_RST_BUS_SPI0		28
#define D1_RST_BUS_SPI1		29
#define D1_RST_BUS_EMAC		30
#define D1_RST_BUS_IR_TX		31
#define D1_RST_BUS_GPADC		32
#define D1_RST_BUS_THS		33
#define D1_RST_BUS_I2S0		34
#define D1_RST_BUS_I2S1		35
#define D1_RST_BUS_I2S2		36
#define D1_RST_BUS_SPDIF		37
#define D1_RST_BUS_DMIC		38
#define D1_RST_BUS_AUDIO		39
#define D1_RST_USB_PHY0		40
#define D1_RST_USB_PHY1		41
#define D1_RST_BUS_OHCI0		42
#define D1_RST_BUS_OHCI1		43
#define D1_RST_BUS_EHCI0		44
#define D1_RST_BUS_EHCI1		45
#define D1_RST_BUS_OTG		46
#define D1_RST_BUS_LRADC		47
#define D1_RST_BUS_DPSS_TOP	48
#define D1_RST_BUS_HDMI_SUB	49
#define D1_RST_BUS_HDMI_MAIN	50
#define D1_RST_BUS_MIPI_DSI	51
#define D1_RST_BUS_TCON_LCD0	52
#define D1_RST_BUS_TCON_TV	53
#define D1_RST_BUS_LVDS0		54
#define D1_RST_BUS_TVE		55
#define D1_RST_BUS_TVE_TOP	56
#define D1_RST_BUS_TVD		57
#define D1_RST_BUS_TVD_TOP	58
#define D1_RST_BUS_LEDC		59
#define D1_RST_BUS_CSI		60
#define D1_RST_BUS_TPADC		61
#define D1_RST_DSP		62
#define D1_RST_BUS_DSP_CFG	63
#define D1_RST_BUS_DSP_DBG	64
#define D1_RST_BUS_RISCV_CFG	65
#define D1_RST_BUS_CAN0		66
#define D1_RST_BUS_CAN1		67

#endif /* ZEPHYR_DT_BINDINGS_RESET_SUN20I_D1_CCU_H_ */
