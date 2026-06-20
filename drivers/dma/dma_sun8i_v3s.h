/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun6i-family DMA driver internal header.
 *
 * Supports the sun8i V3s (A31-style CFG layout, 8 channels) and the
 * sun20i D1 / T113 (H6-style CFG layout, 16 channels, MBUS clock).
 * The differences between SoCs are captured in a variant descriptor.
 */

#ifndef ZEPHYR_DRIVERS_DMA_DMA_SUN8I_V3S_H_
#define ZEPHYR_DRIVERS_DMA_DMA_SUN8I_V3S_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>

/*
 * Common / global registers
 *
 * The IRQ enable/status registers are replicated per group of 8 channels:
 *   group 0 -> channels [7:0]
 *   group 1 -> channels [15:8]   (only on D1/T113 and newer)
 */
#define DMA_IRQ_EN_OFFSET(g)	((g) * 0x04)		/* g=0 -> 0x00, g=1 -> 0x04 */
#define DMA_IRQ_PEND_OFFSET(g)	((g) * 0x04 + 0x10)	/* g=0 -> 0x10, g=1 -> 0x14 */

/* Legacy single-register aliases (group 0) */
#define DMA_IRQ_EN_REG0		DMA_IRQ_EN_OFFSET(0)
#define DMA_IRQ_PEND_REG0	DMA_IRQ_PEND_OFFSET(0)

/* Auto-gating register: V3s at 0x20, D1/T113 at 0x28 */
#define DMA_AUTO_GATE_REG_A31	0x20
#define DMA_AUTO_GATE_REG_H6	0x28

/* Disable auto-gating for all circuits during init (MCLK + common + channel).
 * Writing 1 to a bit disables that circuit's auto-gating so the clock stays
 * on while software programs the controller.
 */
#define DMA_AUTO_GATE_ENABLE	(BIT(2) | BIT(1) | BIT(0))

#define DMA_STA_REG		0x30

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
 * IRQ layout — each IRQ register group packs 8 channels, 4 bits each
 * (bit0 = half-package, bit1 = package-end, bit2 = queue-end).
 */
#define DMA_IRQ_WIDTH		4
#define DMA_IRQ_CHAN_PER_REG	8
#define DMA_IRQ_PKG		BIT(1)	/* Package end */
#define DMA_IRQ_QUEUE		BIT(2)	/* Queue end */
#define DMA_IRQ_ALL		0x7U

/*
 * CFG register field macros
 *
 * The data-width field is at the same bit position for all supported
 * SoCs (SRC width bits 10:9, DST width bits 26:25), so it is shared.
 * The DRQ-type, address-mode and burst-length fields differ between
 * the A31 layout (V3s) and the H6 layout (D1/T113) and are therefore
 * applied through per-variant setter callbacks.
 */
#define DMA_CFG_SRC_WIDTH(x)	(((x) & 0x3) << 9)
#define DMA_CFG_DST_WIDTH(x)	(((x) & 0x3) << 25)

/* A31-style layout (V3s): 5-bit DRQ, mode at bit 5/21, burst at bit 8:7/24:23 */
#define DMA_CFG_A31_SRC_DRQ(x)		((x) & 0x1f)
#define DMA_CFG_A31_SRC_MODE(x)		(((x) & 0x1) << 5)
#define DMA_CFG_A31_SRC_BURST(x)	(((x) & 0x3) << 7)
#define DMA_CFG_A31_DST_DRQ(x)		(DMA_CFG_A31_SRC_DRQ(x) << 16)
#define DMA_CFG_A31_DST_MODE(x)		(DMA_CFG_A31_SRC_MODE(x) << 16)
#define DMA_CFG_A31_DST_BURST(x)	(DMA_CFG_A31_SRC_BURST(x) << 16)

/* H6-style layout (D1/T113): 6-bit DRQ, mode at bit 8/24, burst at bit 7:6/23:22 */
#define DMA_CFG_H6_SRC_DRQ(x)		((x) & 0x3f)
#define DMA_CFG_H6_SRC_MODE(x)		(((x) & 0x1) << 8)
#define DMA_CFG_H6_SRC_BURST(x)		(((x) & 0x3) << 6)
#define DMA_CFG_H6_DST_DRQ(x)		(DMA_CFG_H6_SRC_DRQ(x) << 16)
#define DMA_CFG_H6_DST_MODE(x)		(DMA_CFG_H6_SRC_MODE(x) << 16)
#define DMA_CFG_H6_DST_BURST(x)		(DMA_CFG_H6_SRC_BURST(x) << 16)

/*
 * Constants
 */
#define ADDR_MODE_LINEAR	0
#define ADDR_MODE_IO		1
#define DRQ_SDRAM		1
#define LLI_LAST_ITEM		0xfffff800
#define NORMAL_WAIT		8

/* Worst-case number of physical channels across all supported SoCs */
#define SUN6I_DMA_MAX_PCHANS	16

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
 * SoC variant descriptor — mirrors Linux's sun6i_dma_config.
 *
 * The CFG register DRQ/mode/burst fields are applied through the
 * setter callbacks because their bit positions differ between the
 * A31 layout (V3s) and the H6 layout (D1/T113).
 */
struct sun6i_dma_variant {
	uint8_t nr_pchans;		/* number of physical channels */
	uint8_t nr_max_drq;		/* highest DRQ port id + 1 */
	uint32_t auto_gate_reg;		/* auto-gating register offset */
	bool has_high_addr;		/* 34-bit src/dst addressing */
	bool has_mbus_clk;		/* needs a dedicated MBUS clock */
	bool has_second_irq_reg;	/* channels [15:8] use a 2nd IRQ group */

	void (*set_drq)(uint32_t *cfg, uint8_t src_drq, uint8_t dst_drq);
	void (*set_mode)(uint32_t *cfg, uint8_t src_mode, uint8_t dst_mode);
	void (*set_burst)(uint32_t *cfg, uint8_t src_burst, uint8_t dst_burst);

	uint32_t src_burst_lengths;	/* bitmask of supported src bursts */
	uint32_t dst_burst_lengths;	/* bitmask of supported dst bursts */
};

/*
 * Device driver data
 */
struct sun8i_v3s_dma_data {
	struct dma_context ctx;
	uint32_t reg_base;
	struct sun8i_v3s_dma_stream streams[CONFIG_DMA_SUN8I_V3S_VCHAN_COUNT];
	/* descriptors for pchan currently executing */
	struct sun8i_v3s_dma_desc *active_desc[SUN6I_DMA_MAX_PCHANS];
	uint8_t pchan_vchan[SUN6I_DMA_MAX_PCHANS];	/* pchan -> vchan mapping */
	const struct sun6i_dma_variant *variant;
	struct k_spinlock lock;
	struct k_work schedule_work;
	sys_slist_t pending_streams;
};

#endif /* ZEPHYR_DRIVERS_DMA_DMA_SUN8I_V3S_H_ */
