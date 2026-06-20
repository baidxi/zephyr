/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SDHC_SUNXI_MMC_REGS_H_
#define ZEPHYR_DRIVERS_SDHC_SUNXI_MMC_REGS_H_

#include <zephyr/types.h>

/*
 * Allwinner Sunxi MMC register map
 * Based on V3s User Manual V1.0 Chapter 5 and Linux sunxi-mmc driver.
 * Register bit definitions aligned with Linux drivers/mmc/host/sunxi-mmc.c.
 */

struct sunxi_mmc_regs {
	uint32_t gctrl;		/* 0x00 Global Control */
	uint32_t clkcr;		/* 0x04 Clock Control */
	uint32_t timeout;	/* 0x08 Timeout */
	uint32_t width;		/* 0x0C Bus Width */
	uint32_t blksz;		/* 0x10 Block Size */
	uint32_t bytecnt;	/* 0x14 Byte Count */
	uint32_t cmd;		/* 0x18 Command */
	uint32_t arg;		/* 0x1C Argument */
	uint32_t resp0;		/* 0x20 Response 0 */
	uint32_t resp1;		/* 0x24 Response 1 */
	uint32_t resp2;		/* 0x28 Response 2 */
	uint32_t resp3;		/* 0x2C Response 3 */
	uint32_t imask;		/* 0x30 Interrupt Mask */
	uint32_t mint;		/* 0x34 Masked Interrupt Status */
	uint32_t rint;		/* 0x38 Raw Interrupt Status */
	uint32_t status;	/* 0x3C Status */
	uint32_t ftrglevel;	/* 0x40 FIFO Threshold Level */
	uint32_t funcsel;	/* 0x44 Function Select */
	uint32_t cbcr;		/* 0x48 CIU Byte Count */
	uint32_t bbcr;		/* 0x4C BIU Byte Count */
	uint32_t dbgc;		/* 0x50 Debug Control */
	uint32_t res0;		/* 0x54 Reserved */
	uint32_t a12a;		/* 0x58 Auto CMD12 Argument */
	uint32_t ntsr;		/* 0x5C New Timing Set Register */
	uint32_t sdbg;		/* 0x60 SD NewTiming Set Debug Register */
	uint32_t res1[5];	/* 0x64-0x77 Reserved */
	uint32_t hwrst;		/* 0x78 Hardware Reset Register */
	uint32_t res1b;		/* 0x7C Reserved */
	uint32_t dmac;		/* 0x80 IDMAC Control */
	uint32_t dlba;		/* 0x84 IDMAC Descriptor List Base Address */
	uint32_t idst;		/* 0x88 IDMAC Status */
	uint32_t idie;		/* 0x8C IDMAC Interrupt Enable */
	uint32_t chda;		/* 0x90 Current Host Descriptor Address */
	uint32_t cbda;		/* 0x94 Current IDMAC Buffer Descriptor Address */
#ifdef CONFIG_SDHC_SUNXI_T113
	/* T113-S3 / D1 SMHC layout: res2[90] expanded into named registers */
	uint32_t res2[26];		/* 0x98-0xFF Reserved */
	uint32_t thld;			/* 0x100 Card Threshold Control */
	uint32_t sfc;			/* 0x104 Sample FIFO Control */
	uint32_t a23a;			/* 0x108 Auto CMD23 Argument */
	uint32_t ddr_sbit_det;		/* 0x10C DDR Start Bit Detect */
	uint32_t res3[10];		/* 0x110-0x137 Reserved */
	uint32_t ext_cmd;		/* 0x138 Extend Command */
	uint32_t ext_resp;		/* 0x13C Extend Response */
	uint32_t drv_dl;		/* 0x140 Drive Delay */
	uint32_t samp_dl;		/* 0x144 Sample Delay */
	uint32_t ds_dl;			/* 0x148 DS Delay */
	uint32_t hs400_dl;		/* 0x14C HS400 Delay */
	uint32_t res4[44];		/* 0x150-0x1FF Reserved */
#else
	uint32_t res2[90];		/* 0x98-0x1FC Reserved — V3s layout */
#endif
	uint32_t fifo;			/* 0x200 FIFO Data Port */
};

/* GCTRL (0x00) bit definitions - matches Linux SDXC_REG_GCTRL */
#define SUNXI_MMC_GCTRL_SOFT_RST		BIT(0)
#define SUNXI_MMC_GCTRL_FIFO_RST		BIT(1)
#define SUNXI_MMC_GCTRL_DMA_RST		BIT(2)
#define SUNXI_MMC_GCTRL_INT_ENB		BIT(4)
#define SUNXI_MMC_GCTRL_DMA_ENB		BIT(5)
#define SUNXI_MMC_GCTRL_DEBOUNCE_ENB		BIT(8)
#define SUNXI_MMC_GCTRL_POSEDGE_LATCH		BIT(9)
#define SUNXI_MMC_GCTRL_DDR_MODE		BIT(10)
#define SUNXI_MMC_GCTRL_MEM_ACCESS_DONE	BIT(29)
#define SUNXI_MMC_GCTRL_ACCESS_DONE_DIRECT	BIT(30)
#define SUNXI_MMC_GCTRL_FIFO_AC_MOD		BIT(31)
#define SUNXI_MMC_GCTRL_ACCESS_BY_AHB		BIT(31)
#define SUNXI_MMC_GCTRL_HARDWARE_RESET \
	(SUNXI_MMC_GCTRL_SOFT_RST | SUNXI_MMC_GCTRL_FIFO_RST | \
	 SUNXI_MMC_GCTRL_DMA_RST)

/*
 * T113-S3 / D1 additional GCTRL (0x00) bit definitions.
 * Guarded with CONFIG_SDHC_SUNXI_T113 so the V3s build is not polluted;
 * these bits are absent on V3s.
 */
#ifdef CONFIG_SDHC_SUNXI_T113
#define SUNXI_MMC_GCTRL_TIME_UNIT_DAT		BIT(11)  /* Time unit of DAT */
#define SUNXI_MMC_GCTRL_TIME_UNIT_CMD		BIT(12)  /* Time unit of CMD */
#define SUNXI_MMC_GCTRL_CD_DBC_ENB		BIT(15)  /* Card detect debounce */
#endif

/* CLKCR (0x04) bit definitions - matches Linux SDXC_REG_CLKCR */
#define SUNXI_MMC_CLKCR_CCLK_DIV_MASK		GENMASK(7, 0)
#define SUNXI_MMC_CLKCR_CCLK_DIV_SHIFT		0
#define SUNXI_MMC_CLKCR_CLK_ENB			BIT(16)
#define SUNXI_MMC_CLKCR_CLK_LOW_PWR		BIT(17)
#define SUNXI_MMC_CLKCR_CLK_MASK_DATA0		BIT(31)

/* WIDTH (0x0C) bit definitions */
#define SUNXI_MMC_WIDTH_1BIT	0
#define SUNXI_MMC_WIDTH_4BIT	1
#define SUNXI_MMC_WIDTH_8BIT	2

/* CMD (0x18) bit definitions - matches Linux SDXC_REG_CMDR */
#define SUNXI_MMC_CMD_RESP_EXP			BIT(6)
#define SUNXI_MMC_CMD_LONG_RESP		BIT(7)
#define SUNXI_MMC_CMD_CHK_RESP_CRC		BIT(8)
#define SUNXI_MMC_CMD_DATA_EXP			BIT(9)
#define SUNXI_MMC_CMD_WRITE			BIT(10)
#define SUNXI_MMC_CMD_SEQUENCE_MODE		BIT(11)
#define SUNXI_MMC_CMD_AUTO_STOP		BIT(12)
#define SUNXI_MMC_CMD_WAIT_PRE_OVER		BIT(13)
#define SUNXI_MMC_CMD_STOP_ABORT_CMD		BIT(14)
#define SUNXI_MMC_CMD_SEND_INIT_SEQ		BIT(15)
#define SUNXI_MMC_CMD_UPCLK_ONLY		BIT(21)
#define SUNXI_MMC_CMD_START			BIT(31)

/*
 * RINT (0x38) / IMASK (0x30) / MINT (0x34) bit definitions
 * Matches Linux sunxi-mmc.c SDXC_REG_RINTR bits and V3s User Manual.
 * IMPORTANT: Previous definitions were from a different SoC and were WRONG for V3s.
 */
#define SUNXI_MMC_RINT_RESP_ERROR		BIT(1)
#define SUNXI_MMC_RINT_COMMAND_DONE		BIT(2)
#define SUNXI_MMC_RINT_DATA_OVER		BIT(3)
#define SUNXI_MMC_RINT_TX_DATA_REQUEST	BIT(4)
#define SUNXI_MMC_RINT_RX_DATA_REQUEST	BIT(5)
#define SUNXI_MMC_RINT_RESP_CRC_ERROR		BIT(6)
#define SUNXI_MMC_RINT_DATA_CRC_ERROR		BIT(7)
#define SUNXI_MMC_RINT_RESP_TIMEOUT		BIT(8)
#define SUNXI_MMC_RINT_DATA_TIMEOUT		BIT(9)
#define SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE	BIT(10)
#define SUNXI_MMC_RINT_FIFO_RUN_ERROR		BIT(11)
#define SUNXI_MMC_RINT_HARDWARE_LOCKED		BIT(12)
#define SUNXI_MMC_RINT_START_BIT_ERROR		BIT(13)
#define SUNXI_MMC_RINT_AUTO_COMMAND_DONE	BIT(14)
#define SUNXI_MMC_RINT_END_BIT_ERROR		BIT(15)
#define SUNXI_MMC_RINT_SDIO_INTERRUPT		BIT(16)
#define SUNXI_MMC_RINT_CARD_INSERT		BIT(30)
#define SUNXI_MMC_RINT_CARD_REMOVE		BIT(31)

/* Error interrupts mask - matches Linux SDXC_INTERRUPT_ERROR_BIT */
#define SUNXI_MMC_RINT_ERR_MASK \
	(SUNXI_MMC_RINT_RESP_ERROR | \
	 SUNXI_MMC_RINT_RESP_CRC_ERROR | \
	 SUNXI_MMC_RINT_DATA_CRC_ERROR | \
	 SUNXI_MMC_RINT_RESP_TIMEOUT | \
	 SUNXI_MMC_RINT_DATA_TIMEOUT | \
	 SUNXI_MMC_RINT_FIFO_RUN_ERROR | \
	 SUNXI_MMC_RINT_HARDWARE_LOCKED | \
	 SUNXI_MMC_RINT_START_BIT_ERROR | \
	 SUNXI_MMC_RINT_END_BIT_ERROR)

/* Done interrupts mask - matches Linux SDXC_INTERRUPT_DONE_BIT */
#define SUNXI_MMC_RINT_DONE_MASK \
	(SUNXI_MMC_RINT_AUTO_COMMAND_DONE | \
	 SUNXI_MMC_RINT_DATA_OVER | \
	 SUNXI_MMC_RINT_COMMAND_DONE | \
	 SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE)

/* Command completion interrupts */
#define SUNXI_MMC_RINT_CMD_COMPLETE_MASK \
	(SUNXI_MMC_RINT_COMMAND_DONE | \
	 SUNXI_MMC_RINT_RESP_ERROR | \
	 SUNXI_MMC_RINT_RESP_CRC_ERROR | \
	 SUNXI_MMC_RINT_RESP_TIMEOUT)

/* Data transfer completion interrupts */
#define SUNXI_MMC_RINT_DATA_COMPLETE_MASK \
	(SUNXI_MMC_RINT_DATA_OVER | \
	 SUNXI_MMC_RINT_DATA_CRC_ERROR | \
	 SUNXI_MMC_RINT_DATA_TIMEOUT | \
	 SUNXI_MMC_RINT_FIFO_RUN_ERROR | \
	 SUNXI_MMC_RINT_AUTO_COMMAND_DONE)

/*
 * STATUS (0x3C) bit definitions
 * Matches Linux sunxi-mmc.c SDXC_REG_STAS bits and V3s User Manual.
 * IMPORTANT: Previous definitions were from a different SoC and were WRONG for V3s.
 */
#define SUNXI_MMC_STATUS_RXWL_FLAG		BIT(0)
#define SUNXI_MMC_STATUS_TXWL_FLAG		BIT(1)
#define SUNXI_MMC_STATUS_FIFO_EMPTY		BIT(2)
#define SUNXI_MMC_STATUS_FIFO_FULL		BIT(3)
#define SUNXI_MMC_STATUS_CARD_PRESENT		BIT(8)
#define SUNXI_MMC_STATUS_CARD_DATA_BUSY		BIT(9)
#define SUNXI_MMC_STATUS_DATA_FSM_BUSY		BIT(10)
#define SUNXI_MMC_STATUS_DMA_REQUEST		BIT(31)
#define SUNXI_MMC_STATUS_FIFO_SIZE		16
#define SUNXI_MMC_STATUS_FIFO_SIZE_T113		16

/*
 * FIFO Level mask — verified against V3s User Manual (v3s_sdhc.pdf):
 *   Bits [21:17] = FIFO_LEVEL (5 bits, max 31, FIFO has 32 entries)
 *   Bits [16:11] = RESP_IDX (response index)
 *   Bit  [10]    = FSM_BUSY (data FSM busy)
 *   Bit  [9]     = CARD_BUSY (inverted DATA[0])
 *   Bit  [8]     = CARD_PRESENT (DATA[3] level)
 *
 * A20/A13 used separate RX/TX counts at different bit positions.
 * V3s uses a single FIFO_LEVEL field at bits [21:17] for both directions.
 */
/*
 * FIFO_LEVEL field width differs across SoCs:
 *   V3s    : bits [21:17]  (5 bits, max 31, FIFO = 32 entries)
 *   T113/D1: bits [25:17]  (9 bits, max 511, FIFO = 256 entries)
 * The default *_MASK / *_SHIFT macros keep the V3s value (backward compat);
 * the driver variant struct selects the appropriate value at runtime.
 */
#define SUNXI_MMC_STATUS_FIFO_LEVEL_MASK	GENMASK(21, 17)
#define SUNXI_MMC_STATUS_FIFO_LEVEL_SHIFT	17
#define SUNXI_MMC_STATUS_FIFO_LEVEL_MASK_T113	GENMASK(25, 17)
#define SUNXI_MMC_STATUS_FIFO_LEVEL_SHIFT_T113	17

/* Legacy compat — used by sunxi_mmc_fifo_read/write */
#define SUNXI_MMC_STATUS_RX_FIFO_CNT_MASK	SUNXI_MMC_STATUS_FIFO_LEVEL_MASK
#define SUNXI_MMC_STATUS_RX_FIFO_CNT_SHIFT	SUNXI_MMC_STATUS_FIFO_LEVEL_SHIFT
#define SUNXI_MMC_STATUS_TX_FIFO_CNT_MASK	SUNXI_MMC_STATUS_FIFO_LEVEL_MASK
#define SUNXI_MMC_STATUS_TX_FIFO_CNT_SHIFT	SUNXI_MMC_STATUS_FIFO_LEVEL_SHIFT

/* FTRGLEVEL (0x40) bit definitions */
#define SUNXI_MMC_FTRGLEVEL_RX_THRESHOLD_MASK	GENMASK(6, 0)
#define SUNXI_MMC_FTRGLEVEL_TX_THRESHOLD_MASK	GENMASK(22, 16)
#define SUNXI_MMC_FTRGLEVEL_TX_THRESHOLD_SHIFT	16

/* Function Select (FUNS) (0x44) bit definitions - matches Linux SDXC_REG_FUNS */
#define SUNXI_MMC_FUNS_CEATA_ON			(0xceaa << 16)
#define SUNXI_MMC_FUNS_SEND_IRQ_RESPONSE		BIT(0)
#define SUNXI_MMC_FUNS_SDIO_READ_WAIT			BIT(1)
#define SUNXI_MMC_FUNS_ABORT_READ_DATA			BIT(2)

/* IDST (0x88) / IDIE (0x8C) bit definitions */
#define SUNXI_MMC_IDST_TRAN_INT		BIT(0)
#define SUNXI_MMC_IDST_RECV_INT		BIT(1)
#define SUNXI_MMC_IDST_FATAL_INT	BIT(2)
#define SUNXI_MMC_IDST_DMA_DONE		BIT(3)
#define SUNXI_MMC_IDST_NORMAL_INT_SUM	BIT(4)
#define SUNXI_MMC_IDST_ABNORMAL_INT_SUM	BIT(5)
#define SUNXI_MMC_IDST_OWNERSHIP	BIT(6)
#define SUNXI_MMC_IDST_DES_ERR		BIT(7)
#define SUNXI_MMC_IDST_DATA_ERR		BIT(8)

/* IDMAC Control (DMAC) (0x80) bit definitions */
#define SUNXI_MMC_DMAC_SOFT_RST		BIT(0)
#define SUNXI_MMC_DMAC_FIX_BURST	BIT(1)
#define SUNXI_MMC_DMAC_IDMA_ON		BIT(7)
#define SUNXI_MMC_DMAC_DES_LOAD		BIT(31)  /* DES_LOAD_CTRL: refetch desc */

/* DMAC burst length definitions */
#define SUNXI_MMC_DMAC_BURST_LEN_INCR1	0
#define SUNXI_MMC_DMAC_BURST_LEN_INCR4	1
#define SUNXI_MMC_DMAC_BURST_LEN_INCR8	2
#define SUNXI_MMC_DMAC_BURST_LEN_SHIFT	2

/*
 * FIFO size:
 *   V3s    : 128 bytes (32 entries of 32-bit)
 *   T113/D1: 1024 bytes (256 entries of 32-bit)
 * The driver variant struct selects the appropriate value at runtime.
 */
#define SUNXI_MMC_FIFO_SIZE		128
#define SUNXI_MMC_FIFO_ENTRIES		32
#define SUNXI_MMC_FIFO_SIZE_T113		1024
#define SUNXI_MMC_FIFO_ENTRIES_T113		256

/* Hardware reset register (0x78) */
#define SUNXI_MMC_HWRST_EN		BIT(0)

/*
 * T113-S3 / D1 SMHC extended register bit definitions.
 * Guarded with CONFIG_SDHC_SUNXI_T113 so the V3s build is not polluted;
 * these registers/bits are absent on V3s.
 */
#ifdef CONFIG_SDHC_SUNXI_T113

/* NTSR (0x5C) — New Timing Set Register, T113 adds mode select bit */
#define SUNXI_MMC_NTSR_MODE_SEL_NEW		BIT(31)  /* 1 = new timing mode */

/* THLD (0x100) — Card Threshold Control */
#define SUNXI_MMC_THLD_CARD_WR_BLK_SIZE_EN	BIT(2)
#define SUNXI_MMC_THLD_CARD_RD_BLK_SIZE_EN	BIT(0)

/* SFC (0x104) — Sample FIFO Control */
#define SUNXI_MMC_SFC_RX_WMARK_SHIFT		16
#define SUNXI_MMC_SFC_RX_WMARK_MASK		GENMASK(28, 16)
#define SUNXI_MMC_SFC_TX_WMARK_MASK		GENMASK(7, 0)

/* DRV_DL (0x140) — Drive Delay */
#define SUNXI_MMC_DRV_DL_CMD_PHASE_SEL		BIT(16)
#define SUNXI_MMC_DRV_DL_DAT0_PHASE_SEL		BIT(0)

/* SAMP_DL (0x144) — Sample Delay */
#define SUNXI_MMC_SAMP_DL_CMD_PHASE_SEL		BIT(16)
#define SUNXI_MMC_SAMP_DL_DAT0_PHASE_SEL	BIT(0)

#endif /* CONFIG_SDHC_SUNXI_T113 */

/*
 * IDMAC descriptor config flags — matches Linux SDXC_IDMAC_DES0_*
 * If idma_des_size_bits is 16, buf_size bits are:
 *   Bits  0-15: buf1 size
 *   Bits 16-31: reserved / status
 * Since we only ever set buf1 size, we store it directly.
 */
#define SUNXI_IDMAC_DES0_OWN		BIT(31)  /* 1 = IDMAC owns, 0 = host */
#define SUNXI_IDMAC_DES0_CES		BIT(30)  /* Card error summary (RO) */
#define SUNXI_IDMAC_DES0_ER		BIT(5)   /* End of ring */
#define SUNXI_IDMAC_DES0_CH		BIT(4)   /* Chain mode */
#define SUNXI_IDMAC_DES0_FD		BIT(3)   /* First descriptor */
#define SUNXI_IDMAC_DES0_LD		BIT(2)   /* Last descriptor */
#define SUNXI_IDMAC_DES0_DIC		BIT(1)   /* Disable interrupt on completion */

/*
 * IDMAC descriptor structure — matches Linux struct sunxi_idma_des.
 * Each descriptor is 16 bytes.
 */
struct sunxi_idma_des {
	uint32_t config;		/* Descriptor control / status */
	uint32_t buf_size;		/* Buffer size (0 = max, i.e. 65536) */
	uint32_t buf_addr_ptr1;	/* Data buffer physical address */
	uint32_t buf_addr_ptr2;	/* Next descriptor physical address */
};

/* Max data per single descriptor: 64KB (buf_size=0 means maximum) */
#define SUNXI_IDMAC_MAX_BUF_SIZE	65536

/* Descriptor pool size: one page (4096 bytes / 16 bytes per desc = 256 descs) */
#define SUNXI_IDMAC_DESC_POOL_SIZE	4096
#define SUNXI_IDMAC_MAX_DESC_COUNT	(SUNXI_IDMAC_DESC_POOL_SIZE / \
					 sizeof(struct sunxi_idma_des))

#endif /* ZEPHYR_DRIVERS_SDHC_SUNXI_MMC_REGS_H_ */
