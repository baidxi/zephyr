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
#define SPI_TCR_DDB		BIT(8)
#define SPI_TCR_DHB		BIT(9)
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

struct spi_sun8i_v3s_data {
	struct spi_context ctx;
	mm_reg_t base;
	uint8_t bytes_per_access;	/* 1, 2, or 4 (from DT allwinner,data-width) */
	v3s_fifo_write_t fifo_write;	/* fifo_write8/16/32 */
	v3s_fifo_read_t fifo_read;	/* fifo_read8/16/32 */
#ifdef CONFIG_SPI_SUN8I_V3S_INTERRUPT
	uint32_t total;		/* remaining bytes for interrupt-driven transfer */
	const struct spi_config *current_config; /* config for CS deassert in ISR */
#endif
};

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
 *
 * Wide access: pack 'bytes_per_access' bytes into one word, write via
 * data->fifo_write. Remainder bytes fall back to sys_write8.
 *
 * When TX buffer is exhausted (spi_context_tx_buf_on returns false),
 * dummy 0x00 bytes are written to keep the clock running for RX.
 */
static void spi_fill_tx_fifo(struct spi_sun8i_v3s_data *data,
			      struct spi_context *ctx, uint32_t count)
{
	uint32_t bpa = data->bytes_per_access;
	uint32_t aligned = count / bpa;

	/* Wide access portion */
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

	/* Remainder bytes: fallback to byte access */
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
 *
 * Wide access: read one word via data->fifo_read, unpack into
 * 'bytes_per_access' bytes. Remainder bytes fall back to sys_read8.
 *
 * When RX buffer is exhausted, bytes are discarded.
 */
static void spi_drain_rx_fifo(struct spi_sun8i_v3s_data *data,
			       struct spi_context *ctx, uint32_t count)
{
	uint32_t bpa = data->bytes_per_access;
	uint32_t aligned = count / bpa;

	/* Wide access portion */
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

	/* Remainder bytes: fallback to byte access */
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
 * Start a single batch: set MBC/MTC/BCC, fill TX FIFO, trigger XCH.
 * Used by both polling and interrupt-driven paths.
 *
 * BCC.STC must equal the burst count; otherwise the V3s controller
 * produces only half the expected SPI clock cycles per byte.
 * (Confirmed by U-Boot spi-sunxi.c: sun8i variants write BCTL = nbytes.)
 */
static void v3s_spi_start_batch(struct spi_sun8i_v3s_data *data,
				 struct spi_context *ctx, uint32_t chunk)
{
	v3s_spi_wr(data, SPI_MBC, chunk);
	v3s_spi_wr(data, SPI_MTC, chunk);
	v3s_spi_wr(data, SPI_BCC, chunk);
	spi_fill_tx_fifo(data, ctx, chunk);

	LOG_DBG("batch start: MBC=%u MTC=%u BCC=%u TCR=0x%08x",
		v3s_spi_rd(data, SPI_MBC),
		v3s_spi_rd(data, SPI_MTC),
		v3s_spi_rd(data, SPI_BCC),
		v3s_spi_rd(data, SPI_TCR));

	v3s_spi_wr(data, SPI_TCR, v3s_spi_rd(data, SPI_TCR) | SPI_TCR_XCH);
}

/*
 * CS deassert helper (shared by ISR and transceive error path).
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

static void spi_sun8i_v3s_isr(const struct device *dev)
{
	struct spi_sun8i_v3s_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t isr = v3s_spi_rd(data, SPI_ISR);

	/* Clear all pending interrupt flags (W1C) */
	v3s_spi_wr(data, SPI_ISR, isr);

	if (isr & (SPI_ISR_TF_OVF | SPI_ISR_RF_OVF)) {
		v3s_spi_wr(data, SPI_IER, 0);
		spi_context_complete(ctx, dev, -EIO);
		return;
	}

#ifdef CONFIG_SPI_SUN8I_V3S_INTERRUPT
	if (isr & SPI_ISR_TC) {
		/* Current batch complete: drain RX */
		uint32_t chunk = (data->total > SPI_FIFO_DEPTH)
				 ? SPI_FIFO_DEPTH : data->total;

		/* Drain RX FIFO for the completed batch (FSR-based count) */
		uint32_t rx_count = v3s_spi_rd(data, SPI_FSR) & SPI_FSR_RF_CNT_MASK;
		spi_drain_rx_fifo(data, ctx, rx_count);

		data->total -= chunk;

		if (data->total > 0) {
			/* Start next batch */
			uint32_t next = (data->total > SPI_FIFO_DEPTH)
					? SPI_FIFO_DEPTH : data->total;
			v3s_spi_start_batch(data, ctx, next);
		} else {
			/* All batches done: deassert CS, complete */
			v3s_spi_wr(data, SPI_IER, 0);
			v3s_spi_cs_deassert(data, ctx, data->current_config);
			spi_context_complete(ctx, dev, 0);
		}
		return;
	}
#else
	if (isr & SPI_ISR_TC) {
		spi_drain_rx_fifo(data, ctx,
				  v3s_spi_rd(data, SPI_FSR) & SPI_FSR_RF_CNT_MASK);
		v3s_spi_wr(data, SPI_IER, 0);
		spi_context_complete(ctx, dev, 0);
		return;
	}
#endif

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
 * (fast path check is inlined in transceive).
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

	/* Build TCR: preserve SDM | SS_OWNER, clear mode/CS bits */
	uint32_t tcr = v3s_spi_rd(data, SPI_TCR);
	tcr &= ~(SPI_TCR_CPHA | SPI_TCR_CPOL | SPI_TCR_SPOL |
		 SPI_TCR_SS_LEVEL | SPI_TCR_SS_SEL_MASK);

	/* CPOL/CPHA from Zephyr SPI mode */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		tcr |= SPI_TCR_CPOL;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		tcr |= SPI_TCR_CPHA;
	}

	/* SPOL: default active-low (SPOL=1). Set SPOL=0 for active-high. */
	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		tcr &= ~SPI_TCR_SPOL;  /* SPOL=0: active high */
	} else {
		tcr |= SPI_TCR_SPOL;   /* SPOL=1: active low (default) */
	}

	/* CS selection: hardware SS pin or GPIO */
	if (!cfg->gpio_cs && !spi_cs_is_gpio(config)) {
		tcr |= SPI_TCR_SS_SEL(config->slave);
	}

	v3s_spi_wr(data, SPI_TCR, tcr);

	/* Clock divider */
	uint32_t ccr_val;
	int ret = spi_calc_divider(cfg->clock_frequency,
				   config->frequency, &ccr_val);
	if (ret) {
		return ret;
	}
	v3s_spi_wr(data, SPI_CCR, ccr_val);

	/* FIFO trigger levels */
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
	struct spi_sun8i_v3s_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	spi_context_lock(ctx, false, NULL, NULL, config);

	/* Fast path: skip configure when the same config pointer is reused */
	if (ctx->config != config) {
		ret = spi_sun8i_v3s_configure(dev, config);
		if (ret) {
			goto out_release;
		}
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);  /* dfs=1 */

	/* Total transfer = MAX(tx, rx), already computed in ctx->max_count */
	uint32_t total = (uint32_t)ctx->max_count;
	LOG_DBG("transceive: total=%u freq=%u", total, config->frequency);
	if (total == 0) {
		ret = 0;
		goto out_release;
	}

	/* Assert CS (held across all chunks) */
	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(ctx, true);
	} else {
		/* Hardware SS pin, software control: assert based on SPOL */
		uint32_t tcr = v3s_spi_rd(data, SPI_TCR);
		if (tcr & SPI_TCR_SPOL) {
			tcr &= ~SPI_TCR_SS_LEVEL;  /* SPOL=1: active low → assert=low */
		} else {
			tcr |= SPI_TCR_SS_LEVEL;   /* SPOL=0: active high → assert=high */
		}
		v3s_spi_wr(data, SPI_TCR, tcr);
	}

	/* Reset FIFOs once (not needed between chunks) */
	v3s_spi_wr(data, SPI_FCR,
		   v3s_spi_rd(data, SPI_FCR) | SPI_FCR_RF_RST | SPI_FCR_TF_RST);

#ifdef CONFIG_SPI_SUN8I_V3S_INTERRUPT
	/* Interrupt-driven multi-batch transfer */
	data->total = total;
	data->current_config = config;

	uint32_t first = (total > SPI_FIFO_DEPTH) ? SPI_FIFO_DEPTH : total;
	v3s_spi_start_batch(data, ctx, first);

	/* Enable Transfer Complete interrupt; ISR handles remaining batches */
	v3s_spi_wr(data, SPI_IER, SPI_IER_TC_INT_EN);

	ret = spi_context_wait_for_completion(ctx);
	if (ret) {
		/* ISR reported error or timeout: deassert CS manually */
		v3s_spi_cs_deassert(data, ctx, config);
	}
	/* else: ISR already deasserted CS on success */
#else
	/* Polling batched transfer loop */
	while (total > 0) {
		uint32_t chunk = (total > SPI_FIFO_DEPTH)
				 ? SPI_FIFO_DEPTH : total;

		/* ①② Write burst counters */
		v3s_spi_wr(data, SPI_MBC, chunk);
		v3s_spi_wr(data, SPI_MTC, chunk);

		/* ③ Fill TX FIFO (TX exhausted → dummy 0x00) */
		spi_fill_tx_fifo(data, ctx, chunk);

		/* ④ Trigger transfer */
		v3s_spi_wr(data, SPI_TCR,
			   v3s_spi_rd(data, SPI_TCR) | SPI_TCR_XCH);

		/* ⑤ Poll XCH auto-clear */
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

		/* ⑥ Drain RX FIFO */
		spi_drain_rx_fifo(data, ctx, chunk);

		total -= chunk;
	}

	spi_context_complete(ctx, dev, 0);
	ret = spi_context_wait_for_completion(ctx);

cs_deassert:
	v3s_spi_cs_deassert(data, ctx, config);
#endif

out_release:
	spi_context_release(ctx, ret);
	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_sun8i_v3s_transceive_async(const struct device *dev,
					  const struct spi_config *config,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs,
					  struct k_poll_signal *async)
{
	return -ENOTSUP;
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

	/* 1. Pin control */
#if defined(CONFIG_PINCTRL)
	if (cfg->pincfg != NULL) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl (err %d)", ret);
			return ret;
		}
	}
#endif

	/* 2. Map register address */
	device_map(&data->base, cfg->phys_base, cfg->reg_size,
		   K_MEM_CACHE_NONE);

	/* 3. Set FIFO access function pointers based on data-width */
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

	/* 4. Release reset */
	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("reset fail");
		return ret;
	}

	/* 5. Enable bus clock */
	ret = clock_control_on(cfg->clock_dev, (void *)cfg->clk_bus);
	if (ret) {
		LOG_ERR("bus clk fail");
		return ret;
	}

	/* 6. Configure module clock source and frequency */
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

	/* 7. GCR: TP_EN | MASTER | EN | SRST, wait for SRST auto-clear */
	v3s_spi_wr(data, SPI_GCR,
		   SPI_GCR_TP_EN | SPI_GCR_MASTER | SPI_GCR_EN | SPI_GCR_SRST);
	for (int timeout = 1000; timeout > 0; timeout--) {
		if (!(v3s_spi_rd(data, SPI_GCR) & SPI_GCR_SRST)) {
			break;
		}
	}

	/* 8. TCR: SDM (Normal Sample Mode) | SS_OWNER (software CS control) */
	v3s_spi_wr(data, SPI_TCR, SPI_TCR_SDM | SPI_TCR_SS_OWNER);

	/* 9. Disable all interrupts (enabled per-transfer in interrupt mode) */
	v3s_spi_wr(data, SPI_IER, 0);

	/* 10. FIFO trigger levels */
	v3s_spi_wr(data, SPI_FCR,
		   SPI_FCR_TX_TRIG(SPI_FIFO_TRIG_LEVEL) |
		   (SPI_FIFO_TRIG_LEVEL & SPI_FCR_RX_TRIG_MASK));

	/* 11. IRQ config (reserved for P1 interrupt mode) */
	cfg->irq_config(dev);

	/* 12. Initialize GPIO CS (if any) */
	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	/* 13. Release initial lock */
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
	};									\
	PM_DEVICE_DT_INST_DEFINE(inst, spi_sun8i_v3s_pm_action);		\
	SPI_DEVICE_DT_INST_DEFINE(inst, spi_sun8i_v3s_init,			\
			    PM_DEVICE_DT_INST_GET(inst),			\
			    &spi_sun8i_v3s_data_##inst,				\
			    &spi_sun8i_v3s_config_##inst,			\
			    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
			    &spi_sun8i_v3s_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SUN8I_V3S_INIT)
