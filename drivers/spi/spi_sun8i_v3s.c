/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun8i V3s SPI driver (SPI0 only)
 *
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_spi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>
#include <zephyr/linker/sections.h>

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_sun8i_v3s);

#include <zephyr/drivers/clock_control_sun8i_v3s.h>
#include <zephyr/drivers/dma.h>
#include "spi_context.h"

/*
 * Register offsets (ref: v3s_spi.pdf §8.2.4)
 */
#define SPI_GCR			0x04
#define SPI_TCR			0x08
#define SPI_IER			0x10
#define SPI_ISR			0x14
#define SPI_FCR			0x18
#define SPI_FSR			0x1C
#define SPI_CCR			0x24
#define SPI_MBC			0x30
#define SPI_MTC			0x34
#define SPI_BCC			0x38
#define SPI_TXD			0x200
#define SPI_RXD			0x300

/* GCR bits (§8.2.5.1, default 0x00000080) */
#define SPI_GCR_EN		BIT(0)
#define SPI_GCR_MASTER		BIT(1)
#define SPI_GCR_TP_EN		BIT(7)
#define SPI_GCR_SRST		BIT(31)

/* TCR bits (§8.2.5.2, default 0x00000087) */
#define SPI_TCR_CPHA		BIT(0)
#define SPI_TCR_CPOL		BIT(1)
#define SPI_TCR_SPOL		BIT(2)
#define SPI_TCR_SS_SEL_SHIFT	4
#define SPI_TCR_SS_SEL_MASK	GENMASK(5, 4)
#define SPI_TCR_SS_SEL(x)	(((x) & 0x3) << SPI_TCR_SS_SEL_SHIFT)
#define SPI_TCR_SS_OWNER	BIT(6)   /* SS_OWNER: 0=controller, 1=software */
#define SPI_TCR_SS_LEVEL	BIT(7)   /* SS_LEVEL: manual CS level (when SS_OWNER=1) */
#define SPI_TCR_DHB		BIT(8)   /* Discard Hash Burst (1=discard unused RX) */
#define SPI_TCR_DDB		BIT(9)   /* Dummy Burst Type (1=dummy=ONE, 0=ZERO) */
#define SPI_TCR_SDM		BIT(13)  /* Normal Sample Mode */
#define SPI_TCR_XCH		BIT(31)  /* Exchange burst trigger (auto-clear) */

/* IER bits (§8.2.5.3) */
#define SPI_IER_RF_RDY_INT_EN	BIT(0)
#define SPI_IER_TX_ERQ_INT_EN	BIT(4)
#define SPI_IER_TF_OVF_INT_EN	BIT(9)
#define SPI_IER_RF_OVF_INT_EN	BIT(10)
#define SPI_IER_TC_INT_EN	BIT(12)

/* ISR bits (§8.2.5.4, default 0x00000022) */
#define SPI_ISR_RX_RDY		BIT(0)
#define SPI_ISR_TX_ERQ		BIT(4)
#define SPI_ISR_TX_EMP		BIT(5)
#define SPI_ISR_TF_OVF		BIT(9)
#define SPI_ISR_RF_OVF		BIT(10)
#define SPI_ISR_TC		BIT(12)

/* FCR bits (§8.2.5.5, default 0x00400001) */
#define SPI_FCR_RF_RST		BIT(15)
#define SPI_FCR_TF_RST		BIT(31)
#define SPI_FCR_RF_DRQ_EN	BIT(8)    /* RX FIFO DMA request enable */
#define SPI_FCR_TF_DRQ_EN	BIT(24)   /* TX FIFO DMA request enable */
#define SPI_FCR_RX_TRIG_MASK	0xFF
#define SPI_FCR_TX_TRIG_SHIFT	16
#define SPI_FCR_TX_TRIG(x)	(((x) & 0xFF) << SPI_FCR_TX_TRIG_SHIFT)

/* FSR bits (§8.2.5.6) */
#define SPI_FSR_RF_CNT_MASK	0xFF
#define SPI_FSR_TF_CNT_SHIFT	16
#define SPI_FSR_TF_CNT_MASK	GENMASK(23, 16)

/* CCR bits (§8.2.5.8, default 0x00000002) */
#define SPI_CCR_CDR2_MASK	0xFF
#define SPI_CCR_CDR1_SHIFT	8
#define SPI_CCR_CDR1(x)	(((x) & 0xF) << SPI_CCR_CDR1_SHIFT)
#define SPI_CCR_DRS		BIT(12)

#define SPI_FIFO_DEPTH		64
#define SPI_FIFO_TRIG_LEVEL	(SPI_FIFO_DEPTH / 4)

/*
 * FIFO access function pointer types (unified signature)
 *
 * sys_write8/16/32 have (val, addr) parameter order, so we wrap them
 * to a uniform (addr, val) signature for runtime polymorphism.
 */
typedef void (*v3s_fifo_write_t)(mm_reg_t addr, uint32_t val);
typedef uint32_t (*v3s_fifo_read_t)(mm_reg_t addr);

/* FIFO access wrappers */
static void fifo_write8(mm_reg_t addr, uint32_t val)  { sys_write8(val, addr); }
static void fifo_write16(mm_reg_t addr, uint32_t val) { sys_write16(val, addr); }
static void fifo_write32(mm_reg_t addr, uint32_t val) { sys_write32(val, addr); }
static uint32_t fifo_read8(mm_reg_t addr)   { return sys_read8(addr); }
static uint32_t fifo_read16(mm_reg_t addr)  { return sys_read16(addr); }
static uint32_t fifo_read32(mm_reg_t addr)  { return sys_read32(addr); }

#ifdef CONFIG_SPI_SUN8I_V3S_DMA
enum spi_sun8i_v3s_dma_dir {
	V3S_DMA_TX = 0,
	V3S_DMA_RX,
	V3S_DMA_NUM
};

struct spi_sun8i_v3s_dma_data {
	struct dma_config cfg;
	struct dma_block_config blk;
	uint32_t channel;
};
#endif

struct spi_sun8i_v3s_request {
	sys_snode_t node;
	const struct spi_config *config;
	const struct spi_buf_set *tx_bufs;
	const struct spi_buf_set *rx_bufs;
	spi_callback_t cb;
	void *userdata;
};


struct spi_sun8i_v3s_data {
	struct spi_context ctx;
	mm_reg_t base;
	uint8_t bytes_per_access;	/* 1, 2, or 4 (from DT allwinner,data-width) */
	v3s_fifo_write_t fifo_write;	/* fifo_write8/16/32 */
	v3s_fifo_read_t fifo_read;	/* fifo_read8/16/32 */
#ifdef CONFIG_SPI_SUN8I_V3S_DMA
	struct spi_sun8i_v3s_dma_data dma[V3S_DMA_NUM];
	struct k_sem dma_sem;
	int dma_status;
	uint8_t dma_pending;
#endif
	/* Async ring queue (power-of-2, counter + AND for index) */
#define SPI_SUN8I_V3S_QUEUE_BITS 3
#define SPI_SUN8I_V3S_QUEUE_SIZE (1 << SPI_SUN8I_V3S_QUEUE_BITS)
#define SPI_SUN8I_V3S_QUEUE_MASK (SPI_SUN8I_V3S_QUEUE_SIZE - 1)
	struct spi_sun8i_v3s_request queue[SPI_SUN8I_V3S_QUEUE_SIZE];
	uint32_t enq_count;	/* # items ever enqueued (producer counter) */
	uint32_t deq_count;	/* # items ever dequeued  (consumer counter) */
	struct k_sem queue_sem;
	struct k_thread worker;
	bool worker_running;
};
static K_THREAD_STACK_DEFINE(worker_stack, 1024);


struct spi_sun8i_v3s_config {
	uintptr_t phys_base;
	size_t reg_size;
	const struct device *clock_dev;
	clock_control_subsys_t clk_bus;
	clock_control_subsys_t clk_mod;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config)(const struct device *dev);
	uint32_t clock_frequency;
	enum sun8i_spi_clk_src clk_src;
	bool gpio_cs;			/* true if gpio-cs property present */
	uint8_t data_width;		/* 8, 16, or 32 (from DT allwinner,data-width) */
#ifdef CONFIG_SPI_SUN8I_V3S_DMA
	const struct device *dma_dev;
	uint8_t dma_drq_port;
#endif
};

/* Control register access (always 32-bit) */
static inline uint32_t v3s_spi_rd(const struct spi_sun8i_v3s_data *data,
				  uint32_t offset)
{
	return sys_read32(data->base + offset);
}

static inline void v3s_spi_wr(const struct spi_sun8i_v3s_data *data,
			      uint32_t offset, uint32_t val)
{
	sys_write32(val, data->base + offset);
}

/*
 * Clock divider calculation (§8.2.5.8 CCR register)
 *
 * CDR1 (n=0..15): SPI_CLK = Source_CLK / 2^n
 * CDR2 (n=0..255): SPI_CLK = Source_CLK / (2*(n+1)), with DRS=1
 *
 * Scans all CDR1 and CDR2 values, picks the closest to req_freq.
 */

/*
 * Queued async request.  transceive_async enqueues it; the worker
 * thread dequeues and calls spi_sun8i_v3s_transceive.
 */

static int spi_calc_divider(uint32_t src_freq, uint32_t req_freq,
			    uint32_t *ccr_val)
{
	uint32_t best_diff = UINT32_MAX, best_val = 0;
	int n;

	if (req_freq == 0 || src_freq == 0) {
		return -EINVAL;
	}
	if (req_freq >= src_freq) {
		/* Fastest possible: CDR1=0 → div by 1 */
		*ccr_val = SPI_CCR_CDR1(0);
		return 0;
	}

	/* Scan CDR1 (n=0..15): div = 2^n */
	for (n = 0; n <= 15; n++) {
		uint32_t div = 1U << n;
		uint32_t actual = src_freq / div;
		uint32_t diff = (actual > req_freq) ? (actual - req_freq)
						    : (req_freq - actual);
		if (diff < best_diff) {
			best_diff = diff;
			best_val = SPI_CCR_CDR1(n);
		}
		if (actual <= req_freq) {
			break;
		}
	}

	/* Scan CDR2 (n=0..255): div = 2*(n+1) */
	for (n = 0; n <= 255; n++) {
		uint32_t div = 2 * (n + 1);
		uint32_t actual = src_freq / div;
		if (actual == 0) {
			break;
		}
		uint32_t diff = (actual > req_freq) ? (actual - req_freq)
						    : (req_freq - actual);
		if (diff < best_diff) {
			best_diff = diff;
			best_val = SPI_CCR_DRS | (n & SPI_CCR_CDR2_MASK);
		}
		if (actual <= req_freq) {
			break;
		}
	}

	*ccr_val = best_val;
	return 0;
}

/*
 * Fill TX FIFO with 'count' bytes using variable-width access.
 */
static void spi_fill_tx_fifo(struct spi_sun8i_v3s_data *data,
			      struct spi_context *ctx, uint32_t count)
{
	uint32_t bpa = data->bytes_per_access;
	uint32_t aligned = count / bpa;

	for (uint32_t i = 0; i < aligned; i++) {
		uint32_t val = 0;
		for (int j = 0; j < bpa; j++) {
			uint8_t byte = spi_context_tx_buf_on(ctx)
				       ? *ctx->tx_buf : 0;
			val |= (uint32_t)byte << (j * 8);
			spi_context_update_tx(ctx, 1, 1);
		}
		data->fifo_write(data->base + SPI_TXD, val);
	}

	uint32_t remainder = count % bpa;
	for (uint32_t i = 0; i < remainder; i++) {
		uint8_t byte = spi_context_tx_buf_on(ctx)
			       ? *ctx->tx_buf : 0;
		sys_write8(byte, data->base + SPI_TXD);
		spi_context_update_tx(ctx, 1, 1);
	}
}

/*
 * Drain RX FIFO for 'count' bytes using variable-width access.
 */
static void spi_drain_rx_fifo(struct spi_sun8i_v3s_data *data,
			       struct spi_context *ctx, uint32_t count)
{
	uint32_t bpa = data->bytes_per_access;
	uint32_t aligned = count / bpa;

	for (uint32_t i = 0; i < aligned; i++) {
		uint32_t val = data->fifo_read(data->base + SPI_RXD);
		for (int j = 0; j < bpa; j++) {
			uint8_t byte = (val >> (j * 8)) & 0xFF;
			if (spi_context_rx_buf_on(ctx)) {
				*ctx->rx_buf = byte;
			}
			spi_context_update_rx(ctx, 1, 1);
		}
	}

	uint32_t remainder = count % bpa;
	for (uint32_t i = 0; i < remainder; i++) {
		uint8_t byte = sys_read8(data->base + SPI_RXD);
		if (spi_context_rx_buf_on(ctx)) {
			*ctx->rx_buf = byte;
		}
		spi_context_update_rx(ctx, 1, 1);
	}
}

/*
 * CS deassert helper.
 */
static void v3s_spi_cs_deassert(struct spi_sun8i_v3s_data *data,
				 struct spi_context *ctx,
				 const struct spi_config *config)
{
	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(ctx, false);
	} else {
		uint32_t tcr = v3s_spi_rd(data, SPI_TCR);
		if (tcr & SPI_TCR_SPOL) {
			tcr |= SPI_TCR_SS_LEVEL;
		} else {
			tcr &= ~SPI_TCR_SS_LEVEL;
		}
		v3s_spi_wr(data, SPI_TCR, tcr);
	}
}

#ifdef CONFIG_SPI_SUN8I_V3S_DMA

static void spi_sun8i_v3s_dma_callback(const struct device *dma_dev, void *arg,
				       uint32_t channel, int status)
{
	const struct device *dev = (const struct device *)arg;
	struct spi_sun8i_v3s_data *data = dev->data;

	if (status < 0) {
		data->dma_status = status;
	}
	k_sem_give(&data->dma_sem);
}

static int spi_sun8i_v3s_dma_setup(const struct device *dev, int dir,
				    const uint8_t *buf, uint32_t *dummy,
				    uint32_t dma_blk_size)
{
	const struct spi_sun8i_v3s_config *cfg = dev->config;
	struct spi_sun8i_v3s_data *data = dev->data;
	struct spi_sun8i_v3s_dma_data *dma = &data->dma[dir];
	bool is_tx = (dir == V3S_DMA_TX);
	int ret;

	memset(&dma->cfg, 0, sizeof(dma->cfg));
	memset(&dma->blk, 0, sizeof(dma->blk));

	dma->cfg.channel_direction = is_tx ? MEMORY_TO_PERIPHERAL
					    : PERIPHERAL_TO_MEMORY;
	dma->cfg.dma_slot = cfg->dma_drq_port;
	/*
	 * Source and destination data sizes must match: the V3s DMA
	 * controller does not support width packing (e.g. 4 x 1-byte
	 * reads → 1 x 4-byte write).  Mismatched widths cause the
	 * channel to stall without ever completing.
	 */
	if (is_tx) {
		dma->cfg.source_data_size = data->bytes_per_access;
		dma->cfg.dest_data_size = data->bytes_per_access;
	} else {
		dma->cfg.source_data_size = data->bytes_per_access;
		dma->cfg.dest_data_size = data->bytes_per_access;
	}
	dma->cfg.source_burst_length = 8;
	dma->cfg.dest_burst_length = 8;
	dma->cfg.block_count = 1;
	dma->cfg.head_block = &dma->blk;
	dma->cfg.dma_callback = spi_sun8i_v3s_dma_callback;
	dma->cfg.user_data = (void *)dev;

	dma->blk.block_size = dma_blk_size;

	if (is_tx) {
		dma->blk.dest_address = cfg->phys_base + SPI_TXD;
		dma->blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (buf) {
			dma->blk.source_address = (uint32_t)(uintptr_t)buf;
			dma->blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			dma->blk.source_address = (uint32_t)(uintptr_t)dummy;
			dma->blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	} else {
		dma->blk.source_address = cfg->phys_base + SPI_RXD;
		dma->blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (buf) {
			dma->blk.dest_address = (uint32_t)(uintptr_t)buf;
			dma->blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			dma->blk.dest_address = (uint32_t)(uintptr_t)dummy;
			dma->blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	ret = dma_config(cfg->dma_dev, dma->channel, &dma->cfg);
	if (ret < 0) {
		LOG_ERR("dma_config %s failed %d", is_tx ? "tx" : "rx", ret);
		return ret;
	}

	ret = dma_start(cfg->dma_dev, dma->channel);
	if (ret < 0) {
		LOG_ERR("dma_start %s failed %d", is_tx ? "tx" : "rx", ret);
		return ret;
	}

	return 0;
}
#endif /* CONFIG_SPI_SUN8I_V3S_DMA */

static void spi_sun8i_v3s_isr(const struct device *dev)
{
	struct spi_sun8i_v3s_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t isr = v3s_spi_rd(data, SPI_ISR);

	v3s_spi_wr(data, SPI_ISR, isr);

	if (isr & (SPI_ISR_TF_OVF | SPI_ISR_RF_OVF)) {
		v3s_spi_wr(data, SPI_IER, 0);
		spi_context_complete(ctx, dev, -EIO);
		return;
	}

if (isr & SPI_ISR_TC) {
		spi_drain_rx_fifo(data, ctx,
				  v3s_spi_rd(data, SPI_FSR) & SPI_FSR_RF_CNT_MASK);
		v3s_spi_wr(data, SPI_IER, 0);
		v3s_spi_cs_deassert(data, ctx, ctx->config);
		spi_context_complete(ctx, dev, 0);
		return;
	}

	if (isr & SPI_ISR_RX_RDY) {
		spi_drain_rx_fifo(data, ctx,
				  v3s_spi_rd(data, SPI_FSR) & SPI_FSR_RF_CNT_MASK);
	}
	if (isr & SPI_ISR_TX_ERQ) {
		spi_fill_tx_fifo(data, ctx, SPI_FIFO_DEPTH);
	}
}

/*
 * Configure SPI hardware. Called only when ctx->config != config
 */
static int spi_sun8i_v3s_configure(const struct device *dev,
				   const struct spi_config *config)
{
	const struct spi_sun8i_v3s_config *cfg = dev->config;
	struct spi_sun8i_v3s_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (spi_context_is_slave(ctx)) {
		return -ENOTSUP;
	}
	if (config->operation & SPI_MODE_LOOP) {
		return -ENOTSUP;
	}

	uint32_t tcr = v3s_spi_rd(data, SPI_TCR);

	/*
	 * Only update mode/CS-polarity bits here. SS_LEVEL is managed
	 * exclusively by the transceive path (assert on transfer start,
	 * deassert on completion).  Writing SS_LEVEL=0 here would glitch
	 * CS before the transfer even begins.
	 */
	tcr &= ~(SPI_TCR_CPHA | SPI_TCR_CPOL | SPI_TCR_SPOL |
		 SPI_TCR_SS_SEL_MASK);

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		tcr |= SPI_TCR_CPOL;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		tcr |= SPI_TCR_CPHA;
	}

	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		tcr &= ~SPI_TCR_SPOL;
	} else {
		tcr |= SPI_TCR_SPOL;
	}

	if (!cfg->gpio_cs && !spi_cs_is_gpio(config)) {
		tcr |= SPI_TCR_SS_SEL(config->slave);
	}

	/*
	 * Keep CS deasserted while changing configuration:
	 *   SPOL=0 (active-high)  → SS_LEVEL=0 keeps CS deasserted
	 *   SPOL=1 (active-low)   → SS_LEVEL=1 keeps CS deasserted
	 */
	if (tcr & SPI_TCR_SPOL) {
		tcr |= SPI_TCR_SS_LEVEL;
	} else {
		tcr &= ~SPI_TCR_SS_LEVEL;
	}

	v3s_spi_wr(data, SPI_TCR, tcr);

	uint32_t ccr_val;
	int ret = spi_calc_divider(cfg->clock_frequency,
				   config->frequency, &ccr_val);
	if (ret) {
		return ret;
	}
	v3s_spi_wr(data, SPI_CCR, ccr_val);

	v3s_spi_wr(data, SPI_FCR,
		   SPI_FCR_TX_TRIG(SPI_FIFO_TRIG_LEVEL) |
		   (SPI_FIFO_TRIG_LEVEL & SPI_FCR_RX_TRIG_MASK));

	ctx->config = config;
	return 0;
}

static int spi_sun8i_v3s_transceive(const struct device *dev,
				    const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_SUN8I_V3S_DMA
	const struct spi_sun8i_v3s_config *cfg = dev->config;
#endif
	struct spi_sun8i_v3s_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	spi_context_lock(ctx, false, NULL, NULL, config);

	if (ctx->config != config) {
		ret = spi_sun8i_v3s_configure(dev, config);
		if (ret) {
			goto out_release;
		}
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	uint32_t total = (uint32_t)ctx->max_count;
	LOG_DBG("transceive: total=%u freq=%u", total, config->frequency);
	if (total == 0) {
		ret = 0;
		goto out_release;
	}

	/* Assert CS */
	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(ctx, true);
	} else {
		uint32_t tcr = v3s_spi_rd(data, SPI_TCR);
		if (tcr & SPI_TCR_SPOL) {
			tcr &= ~SPI_TCR_SS_LEVEL;
		} else {
			tcr |= SPI_TCR_SS_LEVEL;
		}
		v3s_spi_wr(data, SPI_TCR, tcr);
	}

	/* Reset FIFOs (Linux-style: assert reset, then set trigger levels) */
	v3s_spi_wr(data, SPI_FCR, SPI_FCR_RF_RST | SPI_FCR_TF_RST);
	v3s_spi_wr(data, SPI_FCR,
		   SPI_FCR_TX_TRIG(SPI_FIFO_TRIG_LEVEL) |
		   (SPI_FIFO_TRIG_LEVEL & SPI_FCR_RX_TRIG_MASK));

#ifdef CONFIG_SPI_SUN8I_V3S_DMA
	/* Use PIO for transfers that fit within the FIFO (<= 64 bytes).
	 * DMA requires DDB=1 for correct MTC-based bursting, but DDB=1
	 * (a.k.a. DHB) discards RX data.  DDB=0 keeps RX but the SPI
	 * controller uses FIFO count instead of MTC, causing a race
	 * between DMA-fill and SPI-start for transfers > a few bytes.
	 * PIO is used for FIFO-sized transfers to avoid both issues.
	 */
	if (cfg->dma_dev != NULL && total > SPI_FIFO_DEPTH) {
		static uint32_t dma_dummy_tx, dma_dummy_rx;
		bool has_rx = spi_context_rx_buf_on(ctx);

		data->dma_pending = 1; /* TX always needed */
		data->dma_status = 0;
		k_sem_reset(&data->dma_sem);

		/*
		 * FCR: set DMA trigger level to the transfer size (or
		 * FIFO_DEPTH/2 for large transfers).  A fixed threshold
		 * of 32 would miss the RX trigger on small transfers
		 * (e.g. 4 bytes), causing RX DMA to stall forever.
		 */
		/*
		 * TX trigger = min(total, FIFO_DEPTH/2): DMA refills when the
		 * TX FIFO runs low.  RX trigger = 1: fires as soon as the first
		 * byte enters the RX FIFO, avoiding a stall when total < burst.
		 */
		uint32_t tx_trig = total;
		if (tx_trig > SPI_FIFO_DEPTH / 2) {
			tx_trig = SPI_FIFO_DEPTH / 2;
		}
		uint32_t rx_trig = total / 4;
		if (rx_trig < 1) rx_trig = 1;
		if (rx_trig > SPI_FIFO_DEPTH / 2) rx_trig = SPI_FIFO_DEPTH / 2;
		uint32_t fcr = SPI_FCR_TF_DRQ_EN |
			       ((tx_trig & SPI_FCR_RX_TRIG_MASK) << SPI_FCR_TX_TRIG_SHIFT) |
			       (rx_trig & SPI_FCR_RX_TRIG_MASK);
		if (has_rx) {
			fcr |= SPI_FCR_RF_DRQ_EN;
		}
		v3s_spi_wr(data, SPI_FCR, fcr);

		/* SDK-style read-modify-write: the initial read
		 * unlocks the counter registers after interrupt-
		 * driven PIO transfers.
		 */
		v3s_spi_wr(data, SPI_MBC,
			   (v3s_spi_rd(data, SPI_MBC) & ~0xFFFFFFU) | total);
		v3s_spi_wr(data, SPI_MTC,
			   (v3s_spi_rd(data, SPI_MTC) & ~0xFFFFFFU) | total);
		v3s_spi_wr(data, SPI_BCC,
			   (v3s_spi_rd(data, SPI_BCC) & ~0xFFFFFFU) | total);
		LOG_DBG("DMA cnt: MBC=%u MTC=%u BCC=%u",
			v3s_spi_rd(data, SPI_MBC),
			v3s_spi_rd(data, SPI_MTC),
			v3s_spi_rd(data, SPI_BCC));

		/* DHB = Discard Hash Burst.  SDK sets DHB=1 for
		 * TX-only (discard RX), DHB=0 for full-duplex.
		 */
		if (has_rx) {
			v3s_spi_wr(data, SPI_TCR,
				   v3s_spi_rd(data, SPI_TCR) & ~SPI_TCR_DHB);
		} else {
			v3s_spi_wr(data, SPI_TCR,
				   v3s_spi_rd(data, SPI_TCR) | SPI_TCR_DHB);
		}

		LOG_DBG("DMA xfer: total=%u tx_drq=1 rx_drq=%d",
			total, has_rx);

		/* Trigger transfer BEFORE configuring DMA (SDK order:
		 * XCH first so the SPI controller is running when
		 * DMA starts, matching sunxi_spi_dma_transfer).
		 */
		v3s_spi_wr(data, SPI_TCR,
			   v3s_spi_rd(data, SPI_TCR) | SPI_TCR_XCH);

		/* Now configure and start DMA channels (TX always) */
		ret = spi_sun8i_v3s_dma_setup(dev, V3S_DMA_TX,
		    spi_context_tx_buf_on(ctx) ? ctx->tx_buf : NULL,
		    &dma_dummy_tx, total);
		if (ret) {
			goto dma_cleanup;
		}

		if (has_rx) {
			ret = spi_sun8i_v3s_dma_setup(dev, V3S_DMA_RX,
			    ctx->rx_buf, &dma_dummy_rx, total);
			if (ret) {
				goto dma_cleanup;
			}
			data->dma_pending++;
		}

		/* Wait for DMA channels */
		for (int i = 0; i < data->dma_pending; i++) {
			ret = k_sem_take(&data->dma_sem, K_MSEC(5000));
			if (ret < 0) {
				LOG_ERR("DMA timeout: FSR=0x%08x TCR=0x%08x ISR=0x%08x GCR=0x%08x",
					v3s_spi_rd(data, SPI_FSR),
					v3s_spi_rd(data, SPI_TCR),
					v3s_spi_rd(data, SPI_ISR),
					v3s_spi_rd(data, SPI_GCR));
				data->dma_status = -ETIMEDOUT;
				data->dma_pending = 0;
			}
		}

		if (data->dma_status < 0) {
			ret = data->dma_status;
			goto dma_cleanup;
		}

		spi_context_update_tx(ctx, 1, total);
		spi_context_update_rx(ctx, 1, total);

		spi_context_complete(ctx, dev, 0);
		ret = spi_context_wait_for_completion(ctx);
		goto cs_deassert;

	dma_cleanup:
		dma_stop(cfg->dma_dev,
			 data->dma[V3S_DMA_TX].channel);
		if (has_rx) {
			dma_stop(cfg->dma_dev,
				 data->dma[V3S_DMA_RX].channel);
		}
		LOG_ERR("DMA transfer failed (err %d)", ret);
		goto cs_deassert;
	}
#endif

	while (total > 0) {
		uint32_t chunk = (total > SPI_FIFO_DEPTH)
				 ? SPI_FIFO_DEPTH : total;

		v3s_spi_wr(data, SPI_MBC, chunk);
		v3s_spi_wr(data, SPI_MTC, chunk);
		v3s_spi_wr(data, SPI_BCC, chunk);
		spi_fill_tx_fifo(data, ctx, chunk);

		/* Flush writes before triggering transfer */
		v3s_spi_rd(data, SPI_FSR);
		v3s_spi_wr(data, SPI_TCR,
			   v3s_spi_rd(data, SPI_TCR) | SPI_TCR_XCH);

		uint32_t timeout = 100000;
		while (timeout-- && (v3s_spi_rd(data, SPI_TCR) & SPI_TCR_XCH)) {
			;
		}
		if (timeout == 0) {
			LOG_ERR("XCH timeout: GCR=0x%08x TCR=0x%08x FSR=0x%08x",
				v3s_spi_rd(data, SPI_GCR),
				v3s_spi_rd(data, SPI_TCR),
				v3s_spi_rd(data, SPI_FSR));
			ret = -ETIMEDOUT;
			goto cs_deassert;
		}

		spi_drain_rx_fifo(data, ctx, chunk);
		total -= chunk;
	}

	spi_context_complete(ctx, dev, 0);
	ret = spi_context_wait_for_completion(ctx);
#ifdef CONFIG_SPI_SUN8I_V3S_DMA
cs_deassert:
#endif
	v3s_spi_cs_deassert(data, ctx, config);

out_release:
	spi_context_release(ctx, ret);
	return ret;
}

/*
 * Worker thread: dequeues async requests and executes them
 * by calling the existing sync transceive function.
 */
static void spi_sun8i_v3s_worker(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = (const struct device *)arg1;
	struct spi_sun8i_v3s_data *data = dev->data;
	while (data->worker_running) {
		if (k_sem_take(&data->queue_sem, K_MSEC(100)) < 0) {
			continue;
		}

		while (__atomic_load_n(&data->deq_count, __ATOMIC_RELAXED) !=
		       __atomic_load_n(&data->enq_count, __ATOMIC_ACQUIRE)) {
			uint32_t slot = __atomic_fetch_add(&data->deq_count, 1,
							 __ATOMIC_RELAXED) &
				       SPI_SUN8I_V3S_QUEUE_MASK;
			struct spi_sun8i_v3s_request *req =
				&data->queue[slot];

			int ret = spi_sun8i_v3s_transceive(dev, req->config,
					       req->tx_bufs,
					       req->rx_bufs);

			if (req->cb) {
				req->cb(dev, ret, req->userdata);
			}
		}
	}}

#ifdef CONFIG_SPI_ASYNC
static int spi_sun8i_v3s_transceive_async(const struct device *dev,
					  const struct spi_config *config,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs,
					  spi_callback_t cb,
					  void *userdata)
{
	struct spi_sun8i_v3s_data *data = dev->data;
	uint32_t enq, deq, slot;
	struct spi_sun8i_v3s_request *req;

	/* CAS loop: atomically claim a slot.  Retry if another
	 * producer races us on enq_count between the read and CAS.
	 */
	do {
		enq = __atomic_load_n(&data->enq_count, __ATOMIC_RELAXED);
		deq = __atomic_load_n(&data->deq_count, __ATOMIC_ACQUIRE);

		if (enq - deq >= SPI_SUN8I_V3S_QUEUE_SIZE) {
			return -ENOBUFS;
		}
	} while (!__atomic_compare_exchange_n(&data->enq_count, &enq,
					      enq + 1, false,
					      __ATOMIC_ACQ_REL,
					      __ATOMIC_RELAXED));

	slot = enq & SPI_SUN8I_V3S_QUEUE_MASK;
	req = &data->queue[slot];
	req->config = config;
	req->tx_bufs = tx_bufs;
	req->rx_bufs = rx_bufs;
	req->cb = cb;
	req->userdata = userdata;

	k_sem_give(&data->queue_sem);

	return 0;
}
#endif

static int spi_sun8i_v3s_release(const struct device *dev,
				 const struct spi_config *config)
{
	struct spi_sun8i_v3s_data *data = dev->data;
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, spi_sun8i_v3s_api) = {
	.transceive = spi_sun8i_v3s_transceive,
	.release = spi_sun8i_v3s_release,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_sun8i_v3s_transceive_async,
#endif
};

static int spi_sun8i_v3s_init(const struct device *dev)
{
	const struct spi_sun8i_v3s_config *cfg = dev->config;
	struct spi_sun8i_v3s_data *data = dev->data;
	int ret;

#if defined(CONFIG_PINCTRL)
	if (cfg->pincfg != NULL) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl (err %d)", ret);
			return ret;
		}
	}
#endif

	device_map(&data->base, cfg->phys_base, cfg->reg_size,
		   K_MEM_CACHE_NONE);

	switch (cfg->data_width) {
	case 8:
		data->bytes_per_access = 1;
		data->fifo_write = fifo_write8;
		data->fifo_read = fifo_read8;
		break;
	case 16:
		data->bytes_per_access = 2;
		data->fifo_write = fifo_write16;
		data->fifo_read = fifo_read16;
		break;
	case 32:
		data->bytes_per_access = 4;
		data->fifo_write = fifo_write32;
		data->fifo_read = fifo_read32;
		break;
	default:
		LOG_ERR("Invalid data-width %u", cfg->data_width);
		return -EINVAL;
	}

	/*
	 * Init sequence must match Linux sun6i_spi_runtime_resume():
	 *  1. Enable clocks first
	 *  2. Deassert reset second
	 *  3. GCR write (no SRST — SoC reset already handled it)
	 */
	ret = clock_control_on(cfg->clock_dev, (void *)cfg->clk_bus);
	if (ret) {
		LOG_ERR("bus clk fail");
		return ret;
	}

	struct sun8i_spi_clk_config clk_cfg = {
		.src = cfg->clk_src,
		.output_freq = cfg->clock_frequency,
	};
	ret = clock_control_set_rate(cfg->clock_dev, (void *)cfg->clk_mod,
				     &clk_cfg);
	if (ret) {
		LOG_ERR("mod clk fail");
		return ret;
	}

	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("reset fail");
		return ret;
	}

	/* Enable controller (no SRST — SoC reset line already reset hardware) */
	v3s_spi_wr(data, SPI_GCR,
		   SPI_GCR_TP_EN | SPI_GCR_MASTER | SPI_GCR_EN);

	v3s_spi_wr(data, SPI_TCR, SPI_TCR_SDM | SPI_TCR_SS_OWNER);
	v3s_spi_wr(data, SPI_IER, 0);

	/* Reset FIFOs to clear power-on state */
	v3s_spi_wr(data, SPI_FCR, SPI_FCR_RF_RST | SPI_FCR_TF_RST);
	v3s_spi_wr(data, SPI_FCR,
		   SPI_FCR_TX_TRIG(SPI_FIFO_TRIG_LEVEL) |
		   (SPI_FIFO_TRIG_LEVEL & SPI_FCR_RX_TRIG_MASK));

	cfg->irq_config(dev);

#ifdef CONFIG_SPI_SUN8I_V3S_DMA
	if (cfg->dma_dev != NULL) {
		if (!device_is_ready(cfg->dma_dev)) {
			LOG_ERR("DMA device not ready");
			return -ENODEV;
		}

		for (int i = 0; i < V3S_DMA_NUM; i++) {
			ret = dma_request_channel(cfg->dma_dev, NULL);
			if (ret < 0) {
				LOG_ERR("dma_request_channel failed %d", ret);
				return ret;
			}
			data->dma[i].channel = (uint32_t)ret;
		}

		k_sem_init(&data->dma_sem, 0, 2);

		LOG_DBG("DMA channels: tx=%u rx=%u",
			data->dma[V3S_DMA_TX].channel,
			data->dma[V3S_DMA_RX].channel);
	}
#endif

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	/* Init async request queue and start worker thread */
	k_sem_init(&data->queue_sem, 0, 1);
	data->worker_running = true;
	k_thread_create(&data->worker, worker_stack,
			K_THREAD_STACK_SIZEOF(worker_stack),
			spi_sun8i_v3s_worker,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(1), 0, K_NO_WAIT);

	spi_context_unlock_unconditionally(&data->ctx);

	LOG_DBG("SPI0 ready (data-width=%u)", cfg->data_width);
	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int spi_sun8i_v3s_pm_action(const struct device *dev,
				   enum pm_device_action action)
{
	const struct spi_sun8i_v3s_config *cfg = dev->config;
	struct spi_sun8i_v3s_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		data->worker_running = false;
		k_sem_give(&data->queue_sem);
		k_sleep(K_MSEC(200));
		v3s_spi_wr(data, SPI_GCR,
			   v3s_spi_rd(data, SPI_GCR) & ~SPI_GCR_EN);
		return clock_control_off(cfg->clock_dev, (void *)cfg->clk_bus);
	case PM_DEVICE_ACTION_RESUME:
		if (clock_control_on(cfg->clock_dev, (void *)cfg->clk_bus)) {
			return -EIO;
		}
		reset_line_deassert_dt(&cfg->reset);
		v3s_spi_wr(data, SPI_GCR,
			   SPI_GCR_TP_EN | SPI_GCR_MASTER | SPI_GCR_EN);
		data->worker_running = true;
		k_thread_create(&data->worker, worker_stack,
				K_THREAD_STACK_SIZEOF(worker_stack),
				spi_sun8i_v3s_worker,
				(void *)dev, NULL, NULL,
				K_PRIO_COOP(1), 0, K_NO_WAIT);
		return 0;
	default:
		return -ENOTSUP;
	}
}
#endif

#define SPI_SUN8I_V3S_IRQ_CONFIG(inst)						\
	static void spi_sun8i_v3s_irq_config_##inst(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),	\
			    spi_sun8i_v3s_isr, DEVICE_DT_INST_GET(inst),		\
			    DT_INST_IRQ(inst, flags));				\
		irq_enable(DT_INST_IRQN(inst));					\
	}

#define SPI_SUN8I_V3S_INIT(inst)						\
	SPI_SUN8I_V3S_IRQ_CONFIG(inst);						\
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst)));		\
	static struct spi_sun8i_v3s_data spi_sun8i_v3s_data_##inst = {		\
		SPI_CONTEXT_INIT_LOCK(spi_sun8i_v3s_data_##inst, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_sun8i_v3s_data_##inst, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)		\
	};									\
	static const struct spi_sun8i_v3s_config spi_sun8i_v3s_config_##inst = {\
		.phys_base = DT_INST_REG_ADDR(inst),				\
		.reg_size = DT_INST_REG_SIZE(inst),				\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),		\
		.clk_bus = (clock_control_subsys_t)				\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, bus, clk_id),		\
		.clk_mod = (clock_control_subsys_t)				\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, mod, clk_id),		\
		.reset = RESET_DT_SPEC_INST_GET(inst),				\
		IF_ENABLED(CONFIG_PINCTRL,					\
			(.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),))	\
		.irq_config = spi_sun8i_v3s_irq_config_##inst,			\
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),		\
		.clk_src = COND_CODE_1(						\
			DT_SAME_NODE(						\
				DT_INST_PHANDLE_BY_IDX(inst, clock_source, 0),	\
				DT_NODELABEL(osc24M)),				\
			(SUN8I_SPI_CLK_SRC_HOSC),				\
			(SUN8I_SPI_CLK_SRC_HOSC)),				\
		.gpio_cs = DT_INST_NODE_HAS_PROP(inst, gpio_cs),		\
		.data_width = DT_INST_PROP(inst, allwinner_data_width),		\
		IF_ENABLED(CONFIG_SPI_SUN8I_V3S_DMA,				\
			(.dma_dev = COND_CODE_1(					\
				DT_INST_DMAS_HAS_NAME(inst, tx),		\
				(DEVICE_DT_GET(					\
					DT_INST_DMAS_CTLR_BY_NAME(inst, tx))),	\
				(NULL)),					\
			 .dma_drq_port = COND_CODE_1(				\
				DT_INST_DMAS_HAS_NAME(inst, tx),		\
				(DT_INST_DMAS_CELL_BY_NAME(inst, tx, request)),	\
				(0)),))						\
	};									\
	PM_DEVICE_DT_INST_DEFINE(inst, spi_sun8i_v3s_pm_action);		\
	SPI_DEVICE_DT_INST_DEFINE(inst, spi_sun8i_v3s_init,			\
			    PM_DEVICE_DT_INST_GET(inst),			\
			    &spi_sun8i_v3s_data_##inst,				\
			    &spi_sun8i_v3s_config_##inst,			\
			    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
			    &spi_sun8i_v3s_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SUN8I_V3S_INIT)
