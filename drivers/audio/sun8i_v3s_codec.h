/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register definitions for the Allwinner V3s internal audio codec.
 * Based on V3s Datasheet (Revision 1.0) Chapter 4.13 Audio Codec.
 *
 * The codec consists of:
 * - Digital registers: MMIO at base 0x01c22c00
 * - Analog registers: Indirectly accessed via AC_PR_CFG bridge at offset 0x400
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_SUN8I_V3S_CODEC_H_
#define ZEPHYR_DRIVERS_AUDIO_SUN8I_V3S_CODEC_H_

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DAC Digital Part Control (Default: 0x00000000) */
#define AC_DAC_DPC		0x000
/* Bit fields for AC_DAC_DPC */
#define AC_DAC_DPC_EN_DAC		BIT(31)
#define AC_DAC_DPC_MODQU_SHIFT		25
#define AC_DAC_DPC_MODQU_MASK		GENMASK(28, 25)
#define AC_DAC_DPC_HPF_EN		BIT(18)
#define AC_DAC_DPC_DVOL_SHIFT		12
#define AC_DAC_DPC_DVOL_MASK		GENMASK(17, 12)

/* DAC FIFO Control (Default: 0x00000F00) */
#define AC_DAC_FIFOC		0x004
/* Bit fields for AC_DAC_FIFOC */
#define AC_DAC_FIFOC_DAC_FS_SHIFT	29
#define AC_DAC_FIFOC_DAC_FS_MASK	GENMASK(31, 29)
#define AC_DAC_FIFOC_FIR_VER		BIT(28)
#define AC_DAC_FIFOC_SEND_LASAT		BIT(26)
#define AC_DAC_FIFOC_FIFO_MODE_SHIFT	24
#define AC_DAC_FIFOC_FIFO_MODE_MASK	GENMASK(25, 24)
#define AC_DAC_FIFOC_DRQ_CLR_CNT_SHIFT	21
#define AC_DAC_FIFOC_DRQ_CLR_CNT_MASK	GENMASK(22, 21)
#define AC_DAC_FIFOC_TXTL_SHIFT	8
#define AC_DAC_FIFOC_TXTL_MASK		GENMASK(14, 8)
#define AC_DAC_FIFOC_ADDA_LOOP_EN	BIT(7)
#define AC_DAC_FIFOC_DAC_MONO_EN	BIT(6)
#define AC_DAC_FIFOC_TX_SAMPLE_BITS	BIT(5)
#define AC_DAC_FIFOC_DAC_DRQ_EN	BIT(4)
#define AC_DAC_FIFOC_DAC_IRQ_EN		BIT(3)
#define AC_DAC_FIFOC_FIFO_UNDERRUN_IRQ_EN	BIT(2)
#define AC_DAC_FIFOC_FIFO_OVERRUN_IRQ_EN	BIT(1)
#define AC_DAC_FIFOC_FIFO_FLUSH		BIT(0)

/* DAC_FS sample rate encoding */
#define DAC_FS_48KHZ		0
#define DAC_FS_32KHZ		1
#define DAC_FS_24KHZ		2
#define DAC_FS_16KHZ		3
#define DAC_FS_12KHZ		4
#define DAC_FS_8KHZ		5
#define DAC_FS_192KHZ		6
#define DAC_FS_96KHZ		7

/* DAC FIFO Status (Default: 0x00800088) */
#define AC_DAC_FIFOS		0x008
#define AC_DAC_FIFOS_TX_EMPTY		BIT(23)
#define AC_DAC_FIFOS_TXE_CNT_SHIFT	8
#define AC_DAC_FIFOS_TXE_CNT_MASK	GENMASK(22, 8)
#define AC_DAC_FIFOS_TXE_INT		BIT(3)
#define AC_DAC_FIFOS_TXU_INT		BIT(2)
#define AC_DAC_FIFOS_TXO_INT		BIT(1)

/* ADC FIFO Control (Default: 0x00000F00) */
#define AC_ADC_FIFOC		0x010
/* Bit fields for AC_ADC_FIFOC */
#define AC_ADC_FIFOC_ADFS_SHIFT		29
#define AC_ADC_FIFOC_ADFS_MASK		GENMASK(31, 29)
#define AC_ADC_FIFOC_EN_AD		BIT(28)
#define AC_ADC_FIFOC_RX_FIFO_MODE	BIT(24)
#define AC_ADC_FIFOC_ADCFDT_SHIFT	17
#define AC_ADC_FIFOC_ADCFDT_MASK	GENMASK(18, 17)
#define AC_ADC_FIFOC_ADCDFEN		BIT(16)
#define AC_ADC_FIFOC_RXTL_SHIFT	8
#define AC_ADC_FIFOC_RXTL_MASK		GENMASK(12, 8)
#define AC_ADC_FIFOC_ADC_MONO_EN	BIT(7)
#define AC_ADC_FIFOC_RX_SAMPLE_BITS	BIT(6)
#define AC_ADC_FIFOC_ADC_DRQ_EN	BIT(4)
#define AC_ADC_FIFOC_ADC_IRQ_EN		BIT(3)
#define AC_ADC_FIFOC_ADC_OVERRUN_IRQ_EN	BIT(1)
#define AC_ADC_FIFOC_ADC_FIFO_FLUSH	BIT(0)

/* ADC FIFO Status (Default: 0x00000000) */
#define AC_ADC_FIFOS		0x014
#define AC_ADC_FIFOS_RXA		BIT(23)
#define AC_ADC_FIFOS_RXA_CNT_SHIFT	8
#define AC_ADC_FIFOS_RXA_CNT_MASK	GENMASK(13, 8)
#define AC_ADC_FIFOS_RXA_INT		BIT(3)
#define AC_ADC_FIFOS_RXO_INT		BIT(1)

/* Data registers */
#define AC_ADC_RXDATA		0x018
#define AC_DAC_TXDATA		0x020

/* Counter registers */
#define AC_DAC_CNT		0x040
#define AC_ADC_CNT		0x044

/* Debug registers */
#define AC_DAC_DG		0x048
#define AC_ADC_DG		0x04C

/* DAC DAP Control (Default: 0x00000000) */
#define AC_DAC_DAP_CTR		0x060
#define AC_DAC_DAP_CTR_DDAP_EN		BIT(31)
#define AC_DAC_DAP_CTR_DDAP_DRC_EN	BIT(15)
#define AC_DAC_DAP_CTR_DDAP_HPF_EN	BIT(14)

/* ADC DAP (AGC) registers */
#define AC_ADC_DAP_CTR		0x070
#define AC_ADC_DAP_LCTR		0x074
#define AC_ADC_DAP_RCTR		0x078
#define AC_ADC_DAP_PARA		0x07C
#define AC_ADC_DAP_LAC		0x080
#define AC_ADC_DAP_LDAT		0x084
#define AC_ADC_DAP_RAC		0x088
#define AC_ADC_DAP_RDAT		0x08C
#define ADC_DAP_HPFC		0x090
#define ADC_DAP_LINAC		0x094
#define ADC_DAP_RINAC		0x098
#define ADC_DAP_OPT		0x09C

/* DAC DRC registers (0x100-0x1BC) */
#define AC_DAC_DRC_HHPFC		0x100
#define AC_DAC_DRC_LHPFC		0x104
#define AC_DAC_DRC_CTRL		0x108
/* ... remaining DAC DRC registers omitted for brevity ... */

/* ADC DRC registers (0x200-0x2BC) */
#define AC_ADC_DRC_HHPFC		0x200
#define AC_ADC_DRC_CTRL		0x208
/* ... remaining ADC DRC registers omitted for brevity ... */

#define AC_PR_CFG		0x400

#define AC_PR_CFG_RST			BIT(28)  /* De-assert analog reset */
#define AC_PR_CFG_RW			BIT(24)  /* 1=write, 0=read */
#define AC_PR_CFG_ADDR_SHIFT		16
#define AC_PR_CFG_ADDR_MASK		GENMASK(20, 16)
#define AC_PR_CFG_WDAT_SHIFT		8
#define AC_PR_CFG_WDAT_MASK		GENMASK(15, 8)
#define AC_PR_CFG_RDAT_MASK		GENMASK(7, 0)

/* Headphone Volume (Default: 0x00) */
#define AC_ANALOG_HPVOL			0x00
#define AC_ANALOG_HPVOL_MASK		GENMASK(5, 0)

/* Left/Right Output Mixer Source Select (Default: 0x00) */
#define AC_ANALOG_LOMIXSC		0x01
#define AC_ANALOG_ROMIXSC		0x02
/* Mixer mute control bits */
#define ANALOG_MIXER_MIC1		BIT(6)
#define ANALOG_MIXER_DAC_L		BIT(1)
#define ANALOG_MIXER_DAC_R		BIT(0)

/* DAC Analog Enable and PA Source (Default: 0x00) */
#define AC_ANALOG_DAC_PA_SRC		0x03
#define ANALOG_DACAREN			BIT(7)  /* Right DAC analog enable */
#define ANALOG_DACALEN			BIT(6)  /* Left DAC analog enable */
#define ANALOG_RMIXEN			BIT(5)  /* Right mixer enable */
#define ANALOG_LMIXEN			BIT(4)  /* Left mixer enable */
#define ANALOG_RHPPAMUTE		BIT(3)  /* Right HP PA unmute */
#define ANALOG_LHPPAMUTE		BIT(2)  /* Left HP PA unmute */
#define ANALOG_RHPIS			BIT(1)  /* Right HP input: 0=DAC, 1=Mixer */
#define ANALOG_LHPIS			BIT(0)  /* Left HP input: 0=DAC, 1=Mixer */

/* MIC1 Gain Control (Default: 0x33) */
#define AC_ANALOG_MIC_GCTR		0x06
#define ANALOG_MIC1_GAIN_SHIFT		4
#define ANALOG_MIC1_GAIN_MASK		GENMASK(6, 4)

/* HP Control (Default: 0x14) */
#define AC_ANALOG_HP_CTRL		0x07
#define ANALOG_HPPAEN			BIT(7)  /* HP PA enable */
#define ANALOG_HPCOM_FC_SHIFT		5
#define ANALOG_HPCOM_FC_MASK		GENMASK(6, 5)
#define ANALOG_COMPTEN			BIT(4)  /* HPCOM protection */
#define ANALOG_COS_SLOPE_SHIFT		2
#define ANALOG_COS_SLOPE_MASK		GENMASK(3, 2)

/* BIAS and MIC1 boost (Default: 0x04) */
#define AC_ANALOG_BIAS_MIC_CTRL		0x0B
#define ANALOG_HMICBIASEN		BIT(7)
#define ANALOG_MMICBIASEN		BIT(6)  /* Main MIC bias - used by Linux V3s */
#define ANALOG_MIC1AMPEN		BIT(3)
#define ANALOG_MIC1BOOST_SHIFT		0
#define ANALOG_MIC1BOOST_MASK		GENMASK(2, 0)

/* Left/Right ADC Mixer Mute (Default: 0x00) */
#define AC_ANALOG_LADC_MIX_MUTE		0x0C
#define AC_ANALOG_RADC_MIX_MUTE		0x0D
/* ADC mixer bits - same layout as output mixer */
#define ANALOG_ADC_MIX_MIC1		BIT(6)
#define ANALOG_ADC_MIX_OUTMIX_L		BIT(1)
#define ANALOG_ADC_MIX_OUTMIX_R		BIT(0)

/* PA Anti-pop time Control (Default: 0x04) */
#define AC_ANALOG_PA_ANTI_POP		0x0E
#define ANALOG_PA_ANTI_POP_MASK		GENMASK(2, 0)

/* ADC Analog Control (Default: 0x03) */
#define AC_ANALOG_ADC_CTRL		0x0F
#define ANALOG_ADCREN			BIT(7)
#define ANALOG_ADCLEN			BIT(6)
#define ANALOG_ADCG_SHIFT		0
#define ANALOG_ADCG_MASK		GENMASK(2, 0)

/* Op-amp bias controls (Default: 0x55) */
#define AC_ANALOG_OPADC_CTRL		0x10
#define AC_ANALOG_OPMIC_CTRL		0x11

/* Zero Cross Control (Default: 0x42) */
#define AC_ANALOG_ZERO_CROSS		0x12

/* ADC Function Control (Default: 0xD6) */
#define AC_ANALOG_ADC_FUN_CTRL		0x13

/* Calibration registers */
#define AC_ANALOG_CALIBRATION_CTRL	0x14
#define AC_ANALOG_DA16CALI_DATA		0x15
#define AC_ANALOG_BIAS16CALI_DATA	0x17
#define AC_ANALOG_BIAS16CALI_SET	0x18

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_AUDIO_SUN8I_V3S_CODEC_H_ */
