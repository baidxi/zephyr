/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun8i V3s DMA driver internal header
 */

#ifndef ZEPHYR_DRIVERS_DMA_DMA_SUN8I_V3S_H_
#define ZEPHYR_DRIVERS_DMA_DMA_SUN8I_V3S_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>

/*
 * Common / global registers
 */
#define DMA_IRQ_EN_REG0		0x00
#define DMA_IRQ_PEND_REG0	0x10
#define DMA_AUTO_GATE_REG	0x20
#define DMA_STA_REG		0x30

/* Disable auto-gating for all circuits during init (MCLK + common + channel) */
#define DMA_AUTO_GATE_ENABLE	(BIT(2) | BIT(1) | BIT(0))

/*
 * Per-channel registers (offset = 0x100 + N * 0x40)
 */
#define DMA_CHAN_ENABLE		0x00
#define DMA_CHAN_ENABLE_START	BIT(0)

#define DMA_CHAN_PAUSE		0x04
#define DMA_CHAN_PAUSE_STOP	BIT(0)

#define DMA_CHAN_LLI_ADDR	0x08

#define DMA_CHAN_CUR_CFG	0x0c
#define DMA_CHAN_CUR_SRC	0x10
#define DMA_CHAN_CUR_DST	0x14
#define DMA_CHAN_CUR_CNT	0x18
#define DMA_CHAN_CUR_PARA	0x1c

/* Channel base offset */
#define DMA_CHAN_OFFSET(n)	(0x100 + (n) * 0x40)

/*
 * Configuration register bit fields (A31 layout, used by V3s)
 */
#define DMA_CFG_SRC_DRQ(x)	((x) & 0x1f)
#define DMA_CFG_SRC_MODE(x)	(((x) & 0x1) << 5)
#define DMA_CFG_SRC_BURST(x)	(((x) & 0x3) << 7)
#define DMA_CFG_SRC_WIDTH(x)	(((x) & 0x3) << 9)
#define DMA_CFG_DST_DRQ(x)	(DMA_CFG_SRC_DRQ(x) << 16)
#define DMA_CFG_DST_MODE(x)	(DMA_CFG_SRC_MODE(x) << 16)
#define DMA_CFG_DST_BURST(x)	(DMA_CFG_SRC_BURST(x) << 16)
#define DMA_CFG_DST_WIDTH(x)	(DMA_CFG_SRC_WIDTH(x) << 16)

/*
 * Constants
 */
#define ADDR_MODE_LINEAR	0
#define ADDR_MODE_IO		1
#define DRQ_SDRAM		1
#define LLI_LAST_ITEM		0xfffff800
#define NORMAL_WAIT		8
#define DMA_IRQ_WIDTH		4
#define DMA_IRQ_CHAN_PER_REG	8
#define DMA_IRQ_PKG		BIT(1)	/* Package end */
#define DMA_IRQ_QUEUE		BIT(2)	/* Queue end */

/*
 * Hardware descriptor (LLI) — 6 words, stored in DRAM/SRAM
 */
struct sun8i_v3s_dma_lli {
	uint32_t cfg;
	uint32_t src;
	uint32_t dst;
	uint32_t len;
	uint32_t para;
	uint32_t p_lli_next;
	struct sun8i_v3s_dma_lli *v_lli_next;
};

/*
 * Software transfer descriptor — wraps a chain of LLIs
 */
struct sun8i_v3s_dma_desc {
	struct sun8i_v3s_dma_lli *v_lli;
	uintptr_t p_lli;
};

/* Channel states */
enum sun8i_v3s_dma_state {
	SUN8I_V3S_DMA_IDLE,
	SUN8I_V3S_DMA_PREPARED,
	SUN8I_V3S_DMA_ACTIVE,
	SUN8I_V3S_DMA_SUSPENDED,
};

/*
 * Per-channel data (one per virtual channel)
 */
struct sun8i_v3s_dma_stream {
	enum sun8i_v3s_dma_state state;
	uint8_t port;			/* peripheral DRQ port number */
	bool cyclic;
	uint32_t irq_type;		/* DMA_IRQ_PKG or DMA_IRQ_QUEUE */
	struct sun8i_v3s_dma_desc *desc;
	dma_callback_t dma_callback;
	void *user_data;
	sys_snode_t node;		/* pending vchan list node */

	/* Cached configuration for building descriptors */
	uint32_t direction;
	uint8_t src_width;
	uint8_t dst_width;
	uint8_t src_burst;
	uint8_t dst_burst;
};

/*
 * Device driver data
 */
struct sun8i_v3s_dma_data {
	struct dma_context ctx;
	uint32_t reg_base;
	struct sun8i_v3s_dma_stream streams[CONFIG_DMA_SUN8I_V3S_VCHAN_COUNT];
	/* descriptors for pchan currently executing */
	struct sun8i_v3s_dma_desc *active_desc[8];
	uint8_t pchan_vchan[8];		/* pchan -> vchan index mapping */
	struct k_spinlock lock;
	struct k_work schedule_work;
	sys_slist_t pending_streams;
};

#endif /* ZEPHYR_DRIVERS_DMA_DMA_SUN8I_V3S_H_ */
