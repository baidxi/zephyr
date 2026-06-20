/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Clock IDs for Allwinner D1 / T113-S3 CCU.
 * Aligned with Linux dt-bindings <dt-bindings/clock/sun20i-d1-ccu.h>.
 * D1_CLK_ prefix avoids collision with other SoC headers.
 */

#ifndef ZEPHYR_DT_BINDINGS_CLOCK_SUN20I_D1_CCU_H_
#define ZEPHYR_DT_BINDINGS_CLOCK_SUN20I_D1_CCU_H_

/* PLL outputs */
#define D1_CLK_PLL_CPUX		0
#define D1_CLK_PLL_DDR0		1
#define D1_CLK_PLL_PERIPH0_4X	2
#define D1_CLK_PLL_PERIPH0_2X	3
#define D1_CLK_PLL_PERIPH0_800M	4
#define D1_CLK_PLL_PERIPH0	5
#define D1_CLK_PLL_PERIPH0_DIV3	6
#define D1_CLK_PLL_VIDEO0_4X	7
#define D1_CLK_PLL_VIDEO0_2X	8
#define D1_CLK_PLL_VIDEO0	9
#define D1_CLK_PLL_VIDEO1_4X	10
#define D1_CLK_PLL_VIDEO1_2X	11
#define D1_CLK_PLL_VIDEO1	12
#define D1_CLK_PLL_VE		13
#define D1_CLK_PLL_AUDIO0_4X	14
#define D1_CLK_PLL_AUDIO0_2X	15
#define D1_CLK_PLL_AUDIO0	16
#define D1_CLK_PLL_AUDIO1	17
#define D1_CLK_PLL_AUDIO1_DIV2	18
#define D1_CLK_PLL_AUDIO1_DIV5	19

/* CPU clocks */
#define D1_CLK_CPUX		20
#define D1_CLK_CPUX_AXI		21
#define D1_CLK_CPUX_APB		22

/* Bus clocks */
#define D1_CLK_PSI_AHB		23
#define D1_CLK_APB0		24
#define D1_CLK_APB1		25
#define D1_CLK_MBUS		26

/* Module clocks */
#define D1_CLK_DE		27
#define D1_CLK_BUS_DE		28
#define D1_CLK_DI		29
#define D1_CLK_BUS_DI		30
#define D1_CLK_G2D		31
#define D1_CLK_BUS_G2D		32
#define D1_CLK_CE		33
#define D1_CLK_BUS_CE		34
#define D1_CLK_VE		35
#define D1_CLK_BUS_VE		36
#define D1_CLK_BUS_DMA		37
#define D1_CLK_BUS_MSGBOX0	38
#define D1_CLK_BUS_MSGBOX1	39
#define D1_CLK_BUS_MSGBOX2	40
#define D1_CLK_BUS_SPINLOCK	41
#define D1_CLK_BUS_HSTIMER	42
#define D1_CLK_AVS		43
#define D1_CLK_BUS_DBG		44
#define D1_CLK_BUS_PWM		45
#define D1_CLK_BUS_IOMMU		46
#define D1_CLK_DRAM		47
#define D1_CLK_MBUS_DMA		48
#define D1_CLK_MBUS_VE		49
#define D1_CLK_MBUS_CE		50
#define D1_CLK_MBUS_TVIN		51
#define D1_CLK_MBUS_CSI		52
#define D1_CLK_MBUS_G2D		53
#define D1_CLK_MBUS_RISCV	54
#define D1_CLK_BUS_DRAM		55
#define D1_CLK_MMC0		56
#define D1_CLK_MMC1		57
#define D1_CLK_MMC2		58
#define D1_CLK_BUS_MMC0		59
#define D1_CLK_BUS_MMC1		60
#define D1_CLK_BUS_MMC2		61
/*
 * T113/D1 has no separate CCU output/sample phase clocks for MMC: phase
 * tuning is done inside the SMHC via the DRV_DL/SAMP_DL delay registers.
 * Provide aliases to the module clock so the shared sunxi-mmc driver's
 * four-clock (ahb/mmc/output/sample) binding requirement is satisfied.
 */
#define D1_CLK_MMC0_OUTPUT	D1_CLK_MMC0
#define D1_CLK_MMC0_SAMPLE	D1_CLK_MMC0
#define D1_CLK_MMC1_OUTPUT	D1_CLK_MMC1
#define D1_CLK_MMC1_SAMPLE	D1_CLK_MMC1
#define D1_CLK_MMC2_OUTPUT	D1_CLK_MMC2
#define D1_CLK_MMC2_SAMPLE	D1_CLK_MMC2

/* UART */
#define D1_CLK_BUS_UART0	62
#define D1_CLK_BUS_UART1	63
#define D1_CLK_BUS_UART2	64
#define D1_CLK_BUS_UART3	65
#define D1_CLK_BUS_UART4	66
#define D1_CLK_BUS_UART5	67

/* I2C / TWI */
#define D1_CLK_BUS_I2C0		68
#define D1_CLK_BUS_I2C1		69
#define D1_CLK_BUS_I2C2		70
#define D1_CLK_BUS_I2C3		71

/* SPI */
#define D1_CLK_SPI0		72
#define D1_CLK_SPI1		73
#define D1_CLK_BUS_SPI0		74
#define D1_CLK_BUS_SPI1		75

/* EMAC */
#define D1_CLK_EMAC_25M		76
#define D1_CLK_BUS_EMAC		77

/* IR */
#define D1_CLK_IR_TX		78
#define D1_CLK_BUS_IR_TX		79

/* GPADC / THS */
#define D1_CLK_BUS_GPADC		80
#define D1_CLK_BUS_THS		81

/* I2S */
#define D1_CLK_I2S0		82
#define D1_CLK_I2S1		83
#define D1_CLK_I2S2		84
#define D1_CLK_I2S2_ASRC		85
#define D1_CLK_BUS_I2S0		86
#define D1_CLK_BUS_I2S1		87
#define D1_CLK_BUS_I2S2		88

/* SPDIF */
#define D1_CLK_SPDIF_TX		89
#define D1_CLK_SPDIF_RX		90
#define D1_CLK_BUS_SPDIF		91

/* DMIC / Audio */
#define D1_CLK_DMIC		92
#define D1_CLK_BUS_DMIC		93
#define D1_CLK_AUDIO_DAC		94
#define D1_CLK_AUDIO_ADC		95
#define D1_CLK_BUS_AUDIO		96

/* USB */
#define D1_CLK_USB_OHCI0		97
#define D1_CLK_USB_OHCI1		98
#define D1_CLK_BUS_OHCI0		99
#define D1_CLK_BUS_OHCI1		100
#define D1_CLK_BUS_EHCI0		101
#define D1_CLK_BUS_EHCI1		102
#define D1_CLK_BUS_OTG		103

/* Misc */
#define D1_CLK_BUS_LRADC		104
#define D1_CLK_BUS_DPSS_TOP	105

/* HDMI */
#define D1_CLK_HDMI_24M		106
#define D1_CLK_HDMI_CEC_32K	107
#define D1_CLK_HDMI_CEC		108
#define D1_CLK_BUS_HDMI		109

/* MIPI DSI */
#define D1_CLK_MIPI_DSI		110
#define D1_CLK_BUS_MIPI_DSI	111

/* TCON */
#define D1_CLK_TCON_LCD0		112
#define D1_CLK_BUS_TCON_LCD0	113
#define D1_CLK_TCON_TV		114
#define D1_CLK_BUS_TCON_TV		115

/* TV Encoder / Decoder */
#define D1_CLK_TVE		116
#define D1_CLK_BUS_TVE_TOP	117
#define D1_CLK_BUS_TVE		118
#define D1_CLK_TVD		119
#define D1_CLK_BUS_TVD_TOP	120
#define D1_CLK_BUS_TVD		121

/* LEDC */
#define D1_CLK_LEDC		122
#define D1_CLK_BUS_LEDC		123

/* CSI */
#define D1_CLK_CSI_TOP		124
#define D1_CLK_CSI_MCLK		125
#define D1_CLK_BUS_CSI		126

/* TPADC */
#define D1_CLK_TPADC		127
#define D1_CLK_BUS_TPADC		128

/* TZMA */
#define D1_CLK_BUS_TZMA		129

/* DSP */
#define D1_CLK_DSP		130
#define D1_CLK_BUS_DSP_CFG	131

/* RISC-V (D1 only) */
#define D1_CLK_RISCV		132
#define D1_CLK_RISCV_AXI		133
#define D1_CLK_BUS_RISCV_CFG	134

/* Fanout */
#define D1_CLK_FANOUT_24M	135
#define D1_CLK_FANOUT_12M	136
#define D1_CLK_FANOUT_16M	137
#define D1_CLK_FANOUT_25M	138
#define D1_CLK_FANOUT_32K	139
#define D1_CLK_FANOUT_27M	140
#define D1_CLK_FANOUT_PCLK	141
#define D1_CLK_FANOUT0		142
#define D1_CLK_FANOUT1		143
#define D1_CLK_FANOUT2		144

/* CAN */
#define D1_CLK_BUS_CAN0		145
#define D1_CLK_BUS_CAN1		146

#endif /* ZEPHYR_DT_BINDINGS_CLOCK_SUN20I_D1_CCU_H_ */
