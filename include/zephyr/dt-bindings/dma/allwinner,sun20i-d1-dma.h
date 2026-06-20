/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun20i D1 / T113 DMA controller DRQ port numbers.
 *
 * Values come from the T113 DMAC user manual (Table 3-13 DMA DRQ Type).
 * Note that for most peripherals the RX and TX share the same port
 * number, so a single macro is provided per port.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_ALLWINNER_SUN20I_D1_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_ALLWINNER_SUN20I_D1_DMA_H_

/* Memory ports */
#define SUN20I_D1_DMA_DRQ_SRAM		0
#define SUN20I_D1_DMA_DRQ_SDRAM		1

/* Audio */
#define SUN20I_D1_DMA_DRQ_OWA		2	/* OWA-RX / OWA-TX */
#define SUN20I_D1_DMA_DRQ_I2S_PCM1	4	/* I2S/PCM1-RX / I2S/PCM1-TX */
#define SUN20I_D1_DMA_DRQ_I2S_PCM2	5	/* I2S/PCM2-RX / I2S/PCM2-TX */
#define SUN20I_D1_DMA_DRQ_AUDIO_CODEC	7
#define SUN20I_D1_DMA_DRQ_DMIC		8

/* ADC */
#define SUN20I_D1_DMA_DRQ_GPADC		12
#define SUN20I_D1_DMA_DRQ_TPADC		13	/* TPADC (src) / IR-TX (dst) */

/* UART (RX and TX share the same port number) */
#define SUN20I_D1_DMA_DRQ_UART0		14
#define SUN20I_D1_DMA_DRQ_UART1		15
#define SUN20I_D1_DMA_DRQ_UART2		16
#define SUN20I_D1_DMA_DRQ_UART3		17
#define SUN20I_D1_DMA_DRQ_UART4		18
#define SUN20I_D1_DMA_DRQ_UART5		19

/* SPI (RX and TX share the same port number) */
#define SUN20I_D1_DMA_DRQ_SPI0		22
#define SUN20I_D1_DMA_DRQ_SPI1		23

/* USB0 endpoints */
#define SUN20I_D1_DMA_DRQ_USB0_EP1	30
#define SUN20I_D1_DMA_DRQ_USB0_EP2	31
#define SUN20I_D1_DMA_DRQ_USB0_EP3	32
#define SUN20I_D1_DMA_DRQ_USB0_EP4	33
#define SUN20I_D1_DMA_DRQ_USB0_EP5	34

/* LEDC (destination only) */
#define SUN20I_D1_DMA_DRQ_LEDC		42

/* TWI (RX and TX share the same port number) */
#define SUN20I_D1_DMA_DRQ_TWI0		43
#define SUN20I_D1_DMA_DRQ_TWI1		44
#define SUN20I_D1_DMA_DRQ_TWI2		45
#define SUN20I_D1_DMA_DRQ_TWI3		46

/* SD/MMC (SDMMC-RX and SDMMC-TX share the same port number) */
#define SUN20I_D1_DMA_DRQ_SDMMC0	47
#define SUN20I_D1_DMA_DRQ_SDMMC1	48
#define SUN20I_D1_DMA_DRQ_SDMMC2	49

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_ALLWINNER_SUN20I_D1_DMA_H_ */
