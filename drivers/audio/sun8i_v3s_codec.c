/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr audio codec driver for the Allwinner V3s internal audio codec.
 * Based on:
 * - V3s Datasheet Rev 1.0 Chapter 4.13 (v3s_codec.pdf)
 * - Linux kernel sun4i-codec driver (sun8i-v3s variant)
 *
 * Features implemented:
 * - DAC playback via DMA (TX FIFO → DAC → Headphone)
 * - ADC recording via DMA (MIC → ADC → RX FIFO → Memory)
 * - 16-bit / 24-bit sample format
 * - 8kHz – 48kHz sample rates (via hardware divisor from PLL_AUDIO)
 * - Digital volume control
 * - Headphone output with analog mixer routing
 * - Analog register access via AC_PR_CFG bridge
 * - Cyclic DMA with double-buffering for continuous audio streaming
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_codec

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>

#include "sun8i_v3s_codec.h"

LOG_MODULE_REGISTER(sun8i_v3s_codec, CONFIG_AUDIO_CODEC_LOG_LEVEL);

/* DMA buffer size per direction (total, split into 2 halves for double-buffering) */
#define SUN8I_CODEC_DMA_BUF_SIZE	8192

/* DC blocking high-pass filter coefficient (Q0.8 fixed-point).
 * R = 1 - 2*pi*fc/fs ≈ 0.96 for fc≈100Hz, fs=16kHz.
 * Removes ADC DC offset without affecting speech (100Hz+).
 */
#define DC_BLOCKER_R	246	/* 0.96 * 256 = 245.76 ≈ 246 */

struct sun8i_codec_config {
	uintptr_t mmio;
	uintptr_t analog_base;
	const struct device *clk_dev;
	clock_control_subsys_t clk_bus_id;
	clock_control_subsys_t clk_mod_id;
	const struct device *rst_dev;
	uint32_t rst_id;

	/* DMA configuration from DTS */
	const struct device *dma_dev;
	uint8_t rx_dma_slot;	/* DRQ port for RX */
	uint8_t tx_dma_slot;	/* DRQ port for TX */
};

struct sun8i_codec_data {
	uint32_t sample_rate;
	uint8_t sample_bits;
	uint8_t channels;
	uint8_t digital_vol;
	uint8_t hp_vol;
	uint8_t adc_gain;
	uint8_t mic1_boost;
	bool output_started;
	bool input_started;

	/* DMA RX */
	int rx_dma_chan;		/* DMA channel index, <0 = not allocated */
	uint8_t *rx_buf;
	size_t rx_buf_size;
	uint8_t rx_block_idx;		/* toggles 0/1 for double-buffering */

	/* DMA TX */
	int tx_dma_chan;
	uint8_t *tx_buf;
	size_t tx_buf_size;
	uint8_t tx_block_idx;

	/* DC blocking filter state (per-sample IIR high-pass) */
	int16_t dc_prev_input;
	int32_t dc_prev_output;

	/* Callbacks registered via audio_codec_api */
	audio_codec_tx_done_callback_t tx_done;
	void *tx_cb_user_data;
	audio_codec_rx_done_callback_t rx_done;
	void *rx_cb_user_data;
};

static inline void reg_write(const struct device *dev, uint32_t offset, uint32_t val)
{
	const struct sun8i_codec_config *cfg = dev->config;

	sys_write32(val, cfg->mmio + offset);
}

static inline uint32_t reg_read(const struct device *dev, uint32_t offset)
{
	const struct sun8i_codec_config *cfg = dev->config;

	return sys_read32(cfg->mmio + offset);
}

static inline void reg_update(const struct device *dev, uint32_t offset,
			      uint32_t mask, uint32_t val)
{
	uint32_t r = reg_read(dev, offset);

	r = (r & ~mask) | (val & mask);
	reg_write(dev, offset, r);
}

static inline uint32_t analog_reg_read(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;

	return sys_read32(cfg->analog_base);
}

static inline void analog_reg_write(const struct device *dev, uint32_t val)
{
	const struct sun8i_codec_config *cfg = dev->config;

	sys_write32(val, cfg->analog_base);
}

static void analog_write(const struct device *dev, uint8_t addr, uint8_t data)
{
	uint32_t tmp;

	/* De-assert analog reset */
	tmp = analog_reg_read(dev) | AC_PR_CFG_RST;
	analog_reg_write(dev, tmp);

	/* Set register address */
	tmp = analog_reg_read(dev);
	tmp &= ~AC_PR_CFG_ADDR_MASK;
	tmp |= ((uint32_t)addr << AC_PR_CFG_ADDR_SHIFT) & AC_PR_CFG_ADDR_MASK;
	analog_reg_write(dev, tmp);

	/* Set data to write */
	tmp = analog_reg_read(dev);
	tmp &= ~AC_PR_CFG_WDAT_MASK;
	tmp |= ((uint32_t)data << AC_PR_CFG_WDAT_SHIFT) & AC_PR_CFG_WDAT_MASK;
	analog_reg_write(dev, tmp);

	/* Set write bit to trigger the write */
	tmp = analog_reg_read(dev) | AC_PR_CFG_RW;
	analog_reg_write(dev, tmp);

	/* Clear write bit manually (RW does NOT auto-clear on V3s) */
	tmp = analog_reg_read(dev) & ~AC_PR_CFG_RW;
	analog_reg_write(dev, tmp);
}

static uint8_t analog_read(const struct device *dev, uint8_t addr)
{
	uint32_t tmp;

	/* De-assert analog reset */
	tmp = analog_reg_read(dev) | AC_PR_CFG_RST;
	analog_reg_write(dev, tmp);

	/* Clear write bit to select read mode */
	tmp = analog_reg_read(dev) & ~AC_PR_CFG_RW;
	analog_reg_write(dev, tmp);

	/* Set register address */
	tmp = analog_reg_read(dev);
	tmp &= ~AC_PR_CFG_ADDR_MASK;
	tmp |= ((uint32_t)addr << AC_PR_CFG_ADDR_SHIFT) & AC_PR_CFG_ADDR_MASK;
	analog_reg_write(dev, tmp);

	/* Read back value from RDAT field */
	return (uint8_t)(analog_reg_read(dev) & AC_PR_CFG_RDAT_MASK);
}

static void analog_update(const struct device *dev, uint8_t addr,
			  uint8_t mask, uint8_t val)
{
	uint8_t r = analog_read(dev, addr);

	r = (r & ~mask) | (val & mask);
	analog_write(dev, addr, r);
}

struct sample_rate_entry {
	uint32_t rate;
	uint32_t dac_fs;
};

static const struct sample_rate_entry rate_table[] = {
	{ 48000,  DAC_FS_48KHZ  },
	{ 32000,  DAC_FS_32KHZ  },
	{ 24000,  DAC_FS_24KHZ  },
	{ 16000,  DAC_FS_16KHZ  },
	{ 12000,  DAC_FS_12KHZ  },
	{ 8000,   DAC_FS_8KHZ   },
	{ 96000,  DAC_FS_96KHZ  },
	{ 192000, DAC_FS_192KHZ },
};

static int find_dac_fs(uint32_t rate)
{
	for (int i = 0; i < ARRAY_SIZE(rate_table); i++) {
		if (rate_table[i].rate == rate) {
			return rate_table[i].dac_fs;
		}
	}
	return -EINVAL;
}

static void dma_rx_callback(const struct device *dma_dev, void *user_data,
			    uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct sun8i_codec_data *data = dev->data;

	if (status != DMA_STATUS_BLOCK) {
		LOG_WRN("RX DMA unexpected status: %d", status);
		return;
	}

	if (data->rx_done == NULL) {
		return;
	}

	size_t half = data->rx_buf_size / 2;
	uint8_t *buf;

	if (data->rx_block_idx == 0) {
		buf = data->rx_buf;
	} else {
		buf = data->rx_buf + half;
	}

	/* Convert 32-bit FIFO data to 16-bit PCM in-place.
	 *
	 * With RX_FIFO_MODE=1, each 32-bit FIFO word has the 16-bit ADC
	 * sample in bits[15:0] and sign extension in bits[31:16].
	 * We compact to 16-bit samples and apply a DC blocking high-pass
	 * filter to remove ADC DC offset.
	 *
	 * In-place compaction is safe: dst[i] at offset i*2 never
	 * overwrites src[i] at offset i*4 before it is read.
	 */
	if (data->sample_bits == 16) {
		size_t n_samples = half / 4; /* 32-bit words → samples */
		uint32_t *src = (uint32_t *)(uintptr_t)buf;
		int16_t *dst = (int16_t *)(uintptr_t)buf;

		for (size_t i = 0; i < n_samples; i++) {
			int16_t sample = (int16_t)(src[i] & 0xFFFF);

			/* DC blocking: y[n] = x[n] - x[n-1] + R*y[n-1] */
			int32_t out = (int32_t)sample - data->dc_prev_input
				    + ((data->dc_prev_output * DC_BLOCKER_R) >> 8);
			data->dc_prev_input = sample;
			data->dc_prev_output = out;

			if (out > 32767) {
				out = 32767;
			} else if (out < -32768) {
				out = -32768;
			}
			dst[i] = (int16_t)out;
		}

		/* Report converted size (16-bit PCM, half the original bytes) */
		half = n_samples * 2;
	}

	data->rx_done(dev, buf, half, data->rx_cb_user_data);
	data->rx_block_idx ^= 1;
}

static void dma_tx_callback(const struct device *dma_dev, void *user_data,
			    uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct sun8i_codec_data *data = dev->data;

	if (status != DMA_STATUS_BLOCK) {
		LOG_WRN("TX DMA unexpected status: %d", status);
		return;
	}

	if (data->tx_done == NULL) {
		return;
	}

	/* Notify application: the half that was just consumed is now free.
	 * The application can call audio_codec_write() to refill it.
	 */
	data->tx_done(dev, data->tx_cb_user_data);
	data->tx_block_idx ^= 1;
}

static int setup_rx_dma(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;
	int ret;

	if (cfg->dma_dev == NULL) {
		LOG_WRN("No DMA device, RX DMA disabled");
		return -ENODEV;
	}

	/* Request a DMA channel */
	data->rx_dma_chan = dma_request_channel(cfg->dma_dev, NULL);
	if (data->rx_dma_chan < 0) {
		LOG_ERR("Failed to request RX DMA channel: %d", data->rx_dma_chan);
		return data->rx_dma_chan;
	}

	/* Allocate RX buffer (cache-line aligned) */
	data->rx_buf_size = SUN8I_CODEC_DMA_BUF_SIZE;
	data->rx_buf = k_aligned_alloc(32, data->rx_buf_size);
	if (data->rx_buf == NULL) {
		LOG_ERR("Failed to allocate RX DMA buffer (%zu bytes)", data->rx_buf_size);
		dma_release_channel(cfg->dma_dev, data->rx_dma_chan);
		data->rx_dma_chan = -1;
		return -ENOMEM;
	}
	memset(data->rx_buf, 0, data->rx_buf_size);

	/* Build 2-block cyclic DMA config: PERIPHERAL → MEMORY */
	size_t half = data->rx_buf_size / 2;
	struct dma_block_config rx_blk[2];

	memset(rx_blk, 0, sizeof(rx_blk));

	/* Block 0: first half of buffer */
	rx_blk[0].source_address = cfg->mmio + AC_ADC_RXDATA;
	rx_blk[0].dest_address = (uint32_t)(uintptr_t)data->rx_buf;
	rx_blk[0].block_size = half;
	rx_blk[0].source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	rx_blk[0].dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	rx_blk[0].next_block = &rx_blk[1];

	/* Block 1: second half of buffer */
	rx_blk[1].source_address = cfg->mmio + AC_ADC_RXDATA;
	rx_blk[1].dest_address = (uint32_t)(uintptr_t)(data->rx_buf + half);
	rx_blk[1].block_size = half;
	rx_blk[1].source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	rx_blk[1].dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	rx_blk[1].next_block = NULL;

	struct dma_config dma_cfg;

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.dma_slot = cfg->rx_dma_slot;
	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.cyclic = 1;
	dma_cfg.source_data_size = 4;	/* 32-bit FIFO read */
	dma_cfg.dest_data_size = 4;	/* 32-bit memory write */
	dma_cfg.source_burst_length = 4;
	dma_cfg.dest_burst_length = 4;
	dma_cfg.block_count = 2;
	dma_cfg.head_block = rx_blk;
	dma_cfg.dma_callback = dma_rx_callback;
	dma_cfg.user_data = (void *)dev;

	ret = dma_config(cfg->dma_dev, data->rx_dma_chan, &dma_cfg);
	if (ret < 0) {
		LOG_ERR("dma_config RX failed: %d", ret);
		k_free(data->rx_buf);
		data->rx_buf = NULL;
		dma_release_channel(cfg->dma_dev, data->rx_dma_chan);
		data->rx_dma_chan = -1;
		return ret;
	}

	data->rx_block_idx = 0;
	LOG_INF("RX DMA configured: chan=%d buf=%p size=%zu",
		data->rx_dma_chan, data->rx_buf, data->rx_buf_size);
	return 0;
}

static int setup_tx_dma(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;
	int ret;

	if (cfg->dma_dev == NULL) {
		return -ENODEV;
	}

	data->tx_dma_chan = dma_request_channel(cfg->dma_dev, NULL);
	if (data->tx_dma_chan < 0) {
		LOG_ERR("Failed to request TX DMA channel: %d", data->tx_dma_chan);
		return data->tx_dma_chan;
	}

	data->tx_buf_size = SUN8I_CODEC_DMA_BUF_SIZE;
	data->tx_buf = k_aligned_alloc(32, data->tx_buf_size);
	if (data->tx_buf == NULL) {
		dma_release_channel(cfg->dma_dev, data->tx_dma_chan);
		data->tx_dma_chan = -1;
		return -ENOMEM;
	}
	memset(data->tx_buf, 0, data->tx_buf_size);

	/* Build 2-block cyclic DMA config: MEMORY → PERIPHERAL */
	size_t half = data->tx_buf_size / 2;
	struct dma_block_config tx_blk[2];

	memset(tx_blk, 0, sizeof(tx_blk));

	/* Block 0: first half */
	tx_blk[0].source_address = (uint32_t)(uintptr_t)data->tx_buf;
	tx_blk[0].dest_address = cfg->mmio + AC_DAC_TXDATA;
	tx_blk[0].block_size = half;
	tx_blk[0].source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	tx_blk[0].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	tx_blk[0].next_block = &tx_blk[1];

	/* Block 1: second half */
	tx_blk[1].source_address = (uint32_t)(uintptr_t)(data->tx_buf + half);
	tx_blk[1].dest_address = cfg->mmio + AC_DAC_TXDATA;
	tx_blk[1].block_size = half;
	tx_blk[1].source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	tx_blk[1].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	tx_blk[1].next_block = NULL;

	struct dma_config dma_cfg;

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.dma_slot = cfg->tx_dma_slot;
	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.cyclic = 1;
	dma_cfg.source_data_size = 4;
	dma_cfg.dest_data_size = 4;
	dma_cfg.source_burst_length = 4;
	dma_cfg.dest_burst_length = 4;
	dma_cfg.block_count = 2;
	dma_cfg.head_block = tx_blk;
	dma_cfg.dma_callback = dma_tx_callback;
	dma_cfg.user_data = (void *)dev;

	ret = dma_config(cfg->dma_dev, data->tx_dma_chan, &dma_cfg);
	if (ret < 0) {
		LOG_ERR("dma_config TX failed: %d", ret);
		k_free(data->tx_buf);
		data->tx_buf = NULL;
		dma_release_channel(cfg->dma_dev, data->tx_dma_chan);
		data->tx_dma_chan = -1;
		return ret;
	}

	data->tx_block_idx = 0;
	LOG_INF("TX DMA configured: chan=%d buf=%p size=%zu",
		data->tx_dma_chan, data->tx_buf, data->tx_buf_size);
	return 0;
}

static void teardown_rx_dma(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;

	if (data->rx_dma_chan >= 0) {
		dma_stop(cfg->dma_dev, data->rx_dma_chan);
		dma_release_channel(cfg->dma_dev, data->rx_dma_chan);
		data->rx_dma_chan = -1;
	}
	if (data->rx_buf) {
		k_free(data->rx_buf);
		data->rx_buf = NULL;
	}
}

static void teardown_tx_dma(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;

	if (data->tx_dma_chan >= 0) {
		dma_stop(cfg->dma_dev, data->tx_dma_chan);
		dma_release_channel(cfg->dma_dev, data->tx_dma_chan);
		data->tx_dma_chan = -1;
	}
	if (data->tx_buf) {
		k_free(data->tx_buf);
		data->tx_buf = NULL;
	}
}

static int codec_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	struct sun8i_codec_data *data = dev->data;
	int dac_fs;
	uint32_t fifoc_val;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("Unsupported DAI type: %d", cfg->dai_type);
		return -EINVAL;
	}

	/* Map sample rate */
	dac_fs = find_dac_fs(cfg->dai_cfg.i2s.frame_clk_freq);
	if (dac_fs < 0) {
		LOG_ERR("Unsupported sample rate: %u", cfg->dai_cfg.i2s.frame_clk_freq);
		return dac_fs;
	}
	data->sample_rate = cfg->dai_cfg.i2s.frame_clk_freq;

	/* Map sample bits */
	switch (cfg->dai_cfg.i2s.word_size) {
	case 16:
		data->sample_bits = 16;
		break;
	case 24:
	case 32:
		data->sample_bits = 24;
		break;
	default:
		LOG_ERR("Unsupported sample bits: %u", cfg->dai_cfg.i2s.word_size);
		return -EINVAL;
	}

	/* Map channels */
	data->channels = (uint8_t)cfg->dai_cfg.i2s.channels;

	LOG_DBG("Configure: rate=%u bits=%u channels=%u dac_fs=%d",
		data->sample_rate, data->sample_bits, data->channels, dac_fs);

	/* Flush TX FIFO */
	reg_write(dev, AC_DAC_FIFOC, AC_DAC_FIFOC_FIFO_FLUSH);

	/* Build DAC_FIFOC value */
	fifoc_val = ((uint32_t)dac_fs << AC_DAC_FIFOC_DAC_FS_SHIFT);

	if (data->sample_bits == 24) {
		fifoc_val |= AC_DAC_FIFOC_TX_SAMPLE_BITS;
	}

	if (data->channels == 1) {
		fifoc_val |= AC_DAC_FIFOC_DAC_MONO_EN;
		fifoc_val |= (63U << AC_DAC_FIFOC_TXTL_SHIFT);
	} else {
		fifoc_val |= (31U << AC_DAC_FIFOC_TXTL_SHIFT);
	}

	reg_write(dev, AC_DAC_FIFOC, fifoc_val);

	/* Configure DAC DPC: enable DAC digital part, set digital volume */
	reg_update(dev, AC_DAC_DPC,
		   AC_DAC_DPC_EN_DAC | AC_DAC_DPC_DVOL_MASK,
		   AC_DAC_DPC_EN_DAC |
		   ((uint32_t)data->digital_vol << AC_DAC_DPC_DVOL_SHIFT));

	return 0;
}

static void codec_start_output(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;

	LOG_DBG("Start output");

	/* Enable DAC FIFO DMA requests */
	reg_update(dev, AC_DAC_FIFOC,
		   AC_DAC_FIFOC_DAC_DRQ_EN,
		   AC_DAC_FIFOC_DAC_DRQ_EN);

	/* Configure analog path */
	analog_write(dev, AC_ANALOG_DAC_PA_SRC,
		     ANALOG_DACALEN | ANALOG_DACAREN |
		     ANALOG_LMIXEN | ANALOG_RMIXEN |
		     ANALOG_LHPIS | ANALOG_RHPIS |
		     ANALOG_LHPPAMUTE | ANALOG_RHPPAMUTE);

	analog_write(dev, AC_ANALOG_LOMIXSC, ANALOG_MIXER_DAC_L);
	analog_write(dev, AC_ANALOG_ROMIXSC, ANALOG_MIXER_DAC_R);
	analog_write(dev, AC_ANALOG_HPVOL, data->hp_vol);

	analog_write(dev, AC_ANALOG_HP_CTRL,
		     ANALOG_HPPAEN |
		     (0x01 << ANALOG_HPCOM_FC_SHIFT) |
		     ANALOG_COMPTEN |
		     (0x01 << ANALOG_COS_SLOPE_SHIFT));

	/* Start TX DMA if available */
	if (data->tx_dma_chan >= 0 && data->tx_buf != NULL) {
		/* Flush TX buffer to memory for DMA */
		sys_cache_data_flush_range(data->tx_buf, data->tx_buf_size);
		dma_start(cfg->dma_dev, data->tx_dma_chan);
	}

	data->output_started = true;
}

static void codec_stop_output(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;

	LOG_DBG("Stop output");

	/* Stop TX DMA and release resources */
	if (data->tx_dma_chan >= 0) {
		dma_stop(cfg->dma_dev, data->tx_dma_chan);
		teardown_tx_dma(dev);
	}

	reg_update(dev, AC_DAC_FIFOC, AC_DAC_FIFOC_DAC_DRQ_EN, 0);

	analog_write(dev, AC_ANALOG_HP_CTRL, 0);
	analog_write(dev, AC_ANALOG_DAC_PA_SRC, 0);
	analog_write(dev, AC_ANALOG_LOMIXSC, 0);
	analog_write(dev, AC_ANALOG_ROMIXSC, 0);
	reg_update(dev, AC_DAC_FIFOC, AC_DAC_FIFOC_FIFO_FLUSH,
		   AC_DAC_FIFOC_FIFO_FLUSH);

	data->output_started = false;
}

static void codec_start_input(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;
	int adfs;
	uint32_t fifoc_val;

	LOG_INF("Start input (ADC)");

	/* Debug: dump CCU clock state for audio.
	 * CCU base = 0x01c20000
	 * PLL_AUDIO_BASE = CCU + 0x008
	 * CLK_AC_DIG    = CCU + 0x140
	 */
	uint32_t pll_audio = sys_read32(0x01c20008);
	uint32_t ac_dig_clk = sys_read32(0x01c20140);
	uint32_t bus_codec_gate = sys_read32(0x01c20068);

	LOG_INF("CCU: PLL_AUDIO=0x%08x AC_DIG_CLK=0x%08x BUS_CODEC_GATE=0x%08x",
		pll_audio, ac_dig_clk, bus_codec_gate);

	/* Map sample rate to ADFS */
	adfs = find_dac_fs(data->sample_rate);
	if (adfs < 0) {
		adfs = DAC_FS_48KHZ;
	}

	/* Enable MIC1 main bias (MMICBIASEN) and amplifier.
	 * Linux V3s uses MMICBIASEN (bit6) for the main electret mic bias,
	 * not HMICBIASEN (bit7) which is for headset mic detection.
	 */
	analog_write(dev, AC_ANALOG_BIAS_MIC_CTRL,
		     ANALOG_MMICBIASEN | ANALOG_MIC1AMPEN |
		     (data->mic1_boost << ANALOG_MIC1BOOST_SHIFT));

	/* Route MIC1 to both left and right ADC mixers */
	analog_write(dev, AC_ANALOG_LADC_MIX_MUTE, ANALOG_ADC_MIX_MIC1);
	analog_write(dev, AC_ANALOG_RADC_MIX_MUTE, ANALOG_ADC_MIX_MIC1);

	/* Enable ADC left & right analog, set gain */
	analog_write(dev, AC_ANALOG_ADC_CTRL,
		     ANALOG_ADCREN | ANALOG_ADCLEN |
		     (data->adc_gain << ANALOG_ADCG_SHIFT));

	/* Diagnostic: dump analog registers after configuration */
	LOG_INF("Analog regs: BIAS_MIC=0x%02x LADC_MIX=0x%02x RADC_MIX=0x%02x ADC_CTRL=0x%02x",
		analog_read(dev, AC_ANALOG_BIAS_MIC_CTRL),
		analog_read(dev, AC_ANALOG_LADC_MIX_MUTE),
		analog_read(dev, AC_ANALOG_RADC_MIX_MUTE),
		analog_read(dev, AC_ANALOG_ADC_CTRL));
	LOG_INF("Analog regs: MIC_GCTR=0x%02x ADC_FUN=0x%02x OPADC=0x%02x OPMIC=0x%02x",
		analog_read(dev, AC_ANALOG_MIC_GCTR),
		analog_read(dev, 0x13),
		analog_read(dev, AC_ANALOG_OPADC_CTRL),
		analog_read(dev, AC_ANALOG_OPMIC_CTRL));

	/* Flush ADC RX FIFO */
	reg_write(dev, AC_ADC_FIFOC, AC_ADC_FIFOC_ADC_FIFO_FLUSH);

	/* Configure ADC FIFO.
	 *
	 * For 16-bit mode (our default), the Linux sun4i-codec.c driver sets:
	 *   - RX_SAMPLE_BITS = 0 (16-bit samples)
	 *   - RX_FIFO_MODE   = 1
	 * With RX_FIFO_MODE=1, the 16-bit sample is in bits[15:0] of the
	 * 32-bit FIFO word, with sign extension in bits[31:16].
	 * The driver extracts samples via (raw & 0xFFFF) and converts
	 * in-place to 16-bit PCM in dma_rx_callback().
	 *
	 * For 24-bit mode:
	 *   - RX_SAMPLE_BITS = 1
	 *   - RX_FIFO_MODE   = 0
	 */
	fifoc_val = AC_ADC_FIFOC_EN_AD;
	fifoc_val |= ((uint32_t)adfs << AC_ADC_FIFOC_ADFS_SHIFT);

	if (data->sample_bits == 24) {
		fifoc_val |= AC_ADC_FIFOC_RX_SAMPLE_BITS;
		/* RX_FIFO_MODE stays 0 for 24-bit */
	} else {
		/* 16-bit mode: set RX_FIFO_MODE to match Linux */
		fifoc_val |= AC_ADC_FIFOC_RX_FIFO_MODE;
	}

	if (data->channels == 1) {
		fifoc_val |= AC_ADC_FIFOC_ADC_MONO_EN;
	}
	fifoc_val |= (16U << AC_ADC_FIFOC_RXTL_SHIFT);

	fifoc_val |= AC_ADC_FIFOC_ADC_DRQ_EN;

	reg_write(dev, AC_ADC_FIFOC, fifoc_val);

	/* Debug: dump ADC FIFO registers */
	LOG_INF("ADC FIFOC=0x%08x FIFOS=0x%08x",
		reg_read(dev, AC_ADC_FIFOC), reg_read(dev, AC_ADC_FIFOS));

	/* Diagnostic: poll ADC samples directly from FIFO (before DMA).
	 * This bypasses DMA to verify what the ADC is actually producing.
	 */
	LOG_INF("ADC polling diagnostic (raw FIFO reads):");
	for (int i = 0; i < 20; i++) {
		uint32_t timeout = 100000; /* ~100ms busy-wait timeout */

		while (!(reg_read(dev, AC_ADC_FIFOS) & AC_ADC_FIFOS_RXA)) {
			if (--timeout == 0) {
				LOG_INF("  [%d] FIFO timeout - no data!", i);
				break;
			}
		}
		if (timeout > 0) {
			uint32_t raw = reg_read(dev, AC_ADC_RXDATA);
			LOG_INF("  [%d] raw=0x%08x  hi16=%d  lo16=%u",
				i, raw, (int16_t)(raw >> 16), raw & 0xFFFF);
		}
	}

	/* Flush FIFO again before DMA to discard diagnostic samples */
	reg_update(dev, AC_ADC_FIFOC, AC_ADC_FIFOC_ADC_FIFO_FLUSH,
		   AC_ADC_FIFOC_ADC_FIFO_FLUSH);

	/* Start RX DMA if available */
	if (data->rx_dma_chan >= 0 && data->rx_buf != NULL) {
		data->rx_block_idx = 0;
		dma_start(cfg->dma_dev, data->rx_dma_chan);
		LOG_INF("RX DMA started");
	}

	data->input_started = true;
}

static void codec_stop_input(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;

	LOG_DBG("Stop input (ADC)");

	/* Stop RX DMA and release resources */
	if (data->rx_dma_chan >= 0) {
		dma_stop(cfg->dma_dev, data->rx_dma_chan);
		teardown_rx_dma(dev);
	}

	reg_write(dev, AC_ADC_FIFOC, 0);
	analog_write(dev, AC_ANALOG_ADC_CTRL, 0);
	analog_write(dev, AC_ANALOG_LADC_MIX_MUTE, 0);
	analog_write(dev, AC_ANALOG_RADC_MIX_MUTE, 0);
	analog_write(dev, AC_ANALOG_BIAS_MIC_CTRL, 0);

	data->input_started = false;
}

static int codec_set_property(const struct device *dev, audio_property_t property,
			      audio_channel_t channel, audio_property_value_t val)
{
	struct sun8i_codec_data *data = dev->data;

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		data->digital_vol = (uint8_t)((val.vol * 63) / 100);
		reg_update(dev, AC_DAC_DPC,
			   AC_DAC_DPC_DVOL_MASK,
			   ((uint32_t)data->digital_vol << AC_DAC_DPC_DVOL_SHIFT));
		break;

	case AUDIO_PROPERTY_OUTPUT_MUTE:
		if (val.mute) {
			reg_update(dev, AC_DAC_DPC, AC_DAC_DPC_DVOL_MASK, 0);
			analog_write(dev, AC_ANALOG_HPVOL, 0x00);
		} else {
			reg_update(dev, AC_DAC_DPC, AC_DAC_DPC_DVOL_MASK,
				   ((uint32_t)data->digital_vol << AC_DAC_DPC_DVOL_SHIFT));
			analog_write(dev, AC_ANALOG_HPVOL, data->hp_vol);
		}
		break;

	case AUDIO_PROPERTY_INPUT_VOLUME:
		data->adc_gain = (uint8_t)((val.vol * 7) / 100);
		analog_update(dev, AC_ANALOG_ADC_CTRL,
			      ANALOG_ADCG_MASK,
			      (uint8_t)(data->adc_gain << ANALOG_ADCG_SHIFT));
		break;

	case AUDIO_PROPERTY_INPUT_MUTE:
		if (val.mute) {
			analog_update(dev, AC_ANALOG_ADC_CTRL,
				      ANALOG_ADCREN | ANALOG_ADCLEN, 0);
		} else {
			analog_update(dev, AC_ANALOG_ADC_CTRL,
				      ANALOG_ADCREN | ANALOG_ADCLEN,
				      ANALOG_ADCREN | ANALOG_ADCLEN);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int codec_apply_properties(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int codec_start(const struct device *dev, audio_dai_dir_t dir)
{
	if (dir == AUDIO_DAI_DIR_TX) {
		codec_start_output(dev);
		return 0;
	}
	if (dir == AUDIO_DAI_DIR_RX) {
		codec_start_input(dev);
		return 0;
	}
	return -ENOTSUP;
}

static int codec_stop(const struct device *dev, audio_dai_dir_t dir)
{
	if (dir == AUDIO_DAI_DIR_TX) {
		codec_stop_output(dev);
		return 0;
	}
	if (dir == AUDIO_DAI_DIR_RX) {
		codec_stop_input(dev);
		return 0;
	}
	return -ENOTSUP;
}

static int codec_api_write(const struct device *dev, uint8_t *buf, size_t size)
{
	struct sun8i_codec_data *data = dev->data;

	if (data->tx_buf == NULL) {
		return -ENODEV;
	}

	size_t half = data->tx_buf_size / 2;
	size_t to_copy = MIN(size, half);

	/* Write to the current free half */
	uint8_t *dst = (data->tx_block_idx == 0)
		       ? data->tx_buf
		       : data->tx_buf + half;

	memcpy(dst, buf, to_copy);

	/* Flush to memory for DMA to read */
	sys_cache_data_flush_range(dst, to_copy);

	return 0;
}

static int codec_register_done_callback(const struct device *dev,
					audio_codec_tx_done_callback_t tx_cb,
					void *tx_cb_user_data,
					audio_codec_rx_done_callback_t rx_cb,
					void *rx_cb_user_data)
{
	struct sun8i_codec_data *data = dev->data;

	data->tx_done = tx_cb;
	data->tx_cb_user_data = tx_cb_user_data;
	data->rx_done = rx_cb;
	data->rx_cb_user_data = rx_cb_user_data;

	return 0;
}

static int sun8i_codec_init(const struct device *dev)
{
	const struct sun8i_codec_config *cfg = dev->config;
	struct sun8i_codec_data *data = dev->data;
	int ret;

	LOG_DBG("Initializing V3S audio codec");

	/* Enable bus clock */
	ret = clock_control_on(cfg->clk_dev, cfg->clk_bus_id);
	if (ret < 0) {
		LOG_ERR("Failed to enable bus clock: %d", ret);
		return ret;
	}

	/* Configure and enable PLL_AUDIO (CCU register 0x008).
	 *
	 * The audio codec requires a running PLL_AUDIO to clock the ADC/DAC
	 * digital blocks.  On the V3s this PLL is off by default.
	 *
	 * The ac_dig_clk (CCU + 0x140) is a GATE ONLY (no divider),
	 * so PLL_AUDIO must output the exact rate the codec needs (~25 MHz).
	 *
	 * PLL formula:  rate = 24 MHz × (N+1) / (M+1)
	 * With N=24, M=23: rate = 24 MHz × 25 / 24 = 25 MHz
	 *  - VCO = 24 MHz × 25 = 600 MHz (within PLL operating range)
	 *  - 25 MHz / 1536 = 16276 Hz (≈16 kHz, ~1.7 % off)
	 *
	 * Register layout (CCU + 0x008, from Linux ccu-sun8i-v3s.c):
	 *   bit31       = PLL enable
	 *   bit28       = PLL lock (read-only, 1 = locked)
	 *   bits[14:8]  = N multiplier (offset=8, width=7, max=127)
	 *   bits[4:0]   = M divider     (offset=0, width=5, max=31)
	 */
#define CCU_BASE		0x01c20000U
#define PLL_AUDIO_REG		(CCU_BASE + 0x008U)
#define PLL_AUDIO_ENABLE	BIT(31)
#define PLL_AUDIO_LOCK		BIT(28)
#define PLL_AUDIO_N_SHIFT	8	/* bits[14:8], NOT bit7! */
#define PLL_AUDIO_M_SHIFT	0

	/* N=24, M=23 → 24MHz × 25/24 = 25MHz */
	uint32_t pll_val = PLL_AUDIO_ENABLE
			 | ((uint32_t)24 << PLL_AUDIO_N_SHIFT)
			 | ((uint32_t)23 << PLL_AUDIO_M_SHIFT);

	sys_write32(pll_val, PLL_AUDIO_REG);
	LOG_INF("PLL_AUDIO configured: 0x%08x", sys_read32(PLL_AUDIO_REG));

	/* Wait for PLL lock (bit28) */
	for (int i = 0; i < 1000; i++) {
		if (sys_read32(PLL_AUDIO_REG) & PLL_AUDIO_LOCK) {
			break;
		}
		k_busy_wait(10);
	}

	if (!(sys_read32(PLL_AUDIO_REG) & PLL_AUDIO_LOCK)) {
		LOG_ERR("PLL_AUDIO failed to lock: 0x%08x",
			sys_read32(PLL_AUDIO_REG));
		return -EIO;
	}
	LOG_INF("PLL_AUDIO locked: 0x%08x", sys_read32(PLL_AUDIO_REG));

	/* Enable module clock (CLK_AC_DIG) – gates the audio digital clock
	 * from PLL_AUDIO to the ADC/DAC digital blocks.  Without this the
	 * ADC FIFO never fills and DMA will not transfer any data.
	 */
	ret = clock_control_on(cfg->clk_dev, cfg->clk_mod_id);
	if (ret < 0) {
		LOG_ERR("Failed to enable module clock: %d", ret);
		return ret;
	}
	LOG_INF("Module clock enabled");

	/* Deassert reset */
	ret = reset_line_deassert(cfg->rst_dev, cfg->rst_id);
	if (ret < 0) {
		LOG_ERR("Failed to deassert reset: %d", ret);
		return ret;
	}

	/* Wait for reset to complete */
	k_busy_wait(100);

	/* Set defaults */
	data->sample_rate = 0;
	data->sample_bits = 16;
	data->channels = 2;
	data->digital_vol = 0;		/* Start muted */
	data->hp_vol = 40;		/* ~-22dB safe level */
	data->adc_gain = 3;		/* ADC gain: 0dB */
	data->mic1_boost = 1;		/* MIC1 boost: 6dB */
	data->output_started = false;
	data->input_started = false;
	data->rx_dma_chan = -1;
	data->tx_dma_chan = -1;
	data->rx_buf = NULL;
	data->tx_buf = NULL;
	data->rx_block_idx = 0;
	data->tx_block_idx = 0;
	data->tx_done = NULL;
	data->tx_cb_user_data = NULL;
	data->rx_done = NULL;
	data->rx_cb_user_data = NULL;

	/* De-assert analog reset via AC_PR_CFG bridge */
	analog_reg_write(dev, AC_PR_CFG_RST);

	/* Flush both FIFOs */
	reg_write(dev, AC_DAC_FIFOC, AC_DAC_FIFOC_FIFO_FLUSH);
	reg_write(dev, AC_ADC_FIFOC, AC_ADC_FIFOC_ADC_FIFO_FLUSH);

	/* Configure op-amp bias (default values from datasheet) */
	analog_write(dev, AC_ANALOG_OPADC_CTRL, 0x55);
	analog_write(dev, AC_ANALOG_OPMIC_CTRL, 0x55);

	/* Verify analog register read-back (sanity check) */
	uint8_t val = analog_read(dev, AC_ANALOG_OPADC_CTRL);

	LOG_DBG("V3S audio codec initialized at MMIO %p, analog base %p, OPADC_CTRL=0x%02x",
		(void *)cfg->mmio, (void *)cfg->analog_base, val);

	/* Setup DMA channels */
	if (cfg->dma_dev != NULL) {
		if (!device_is_ready(cfg->dma_dev)) {
			LOG_ERR("DMA device not ready");
			return -ENODEV;
		}

		ret = setup_rx_dma(dev);
		if (ret < 0) {
			LOG_WRN("RX DMA setup failed: %d (polling fallback)", ret);
		}

		ret = setup_tx_dma(dev);
		if (ret < 0) {
			LOG_WRN("TX DMA setup failed: %d (polling fallback)", ret);
		}
	} else {
		LOG_INF("No DMA device configured, using polling mode");
	}

	return 0;
}

static const struct audio_codec_api sun8i_codec_driver_api = {
	.configure = codec_configure,
	.start_output = codec_start_output,
	.stop_output = codec_stop_output,
	.set_property = codec_set_property,
	.apply_properties = codec_apply_properties,
	.start = codec_start,
	.stop = codec_stop,
	.write = codec_api_write,
	.register_done_callback = codec_register_done_callback,
};

#define SUN8I_CODEC_INIT(n)							      \
	static const struct sun8i_codec_config codec_config_##n = {		      \
		.mmio = DT_INST_REG_ADDR(n),					      \
		.analog_base = DT_REG_ADDR(DT_INST_PHANDLE(n,			      \
						allwinner_codec_analog_controls)),    \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		      \
		.clk_bus_id = (clock_control_subsys_t)			      \
				DT_INST_CLOCKS_CELL_BY_IDX(n, 0, clk_id),	      \
		.clk_mod_id = (clock_control_subsys_t)			      \
				DT_INST_CLOCKS_CELL_BY_IDX(n, 1, clk_id),	      \
		.rst_dev = DEVICE_DT_GET(DT_INST_RESET_CTLR(n)),		      \
		.rst_id = DT_INST_RESET_CELL(n, id),				      \
		.dma_dev = COND_CODE_1(					      \
			DT_INST_DMAS_HAS_NAME(n, rx),				      \
			(DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx))),	      \
			(NULL)),						      \
		.rx_dma_slot = COND_CODE_1(					      \
			DT_INST_DMAS_HAS_NAME(n, rx),				      \
			(DT_INST_DMAS_CELL_BY_NAME(n, rx, channel)),		      \
			(0)),							      \
		.tx_dma_slot = COND_CODE_1(					      \
			DT_INST_DMAS_HAS_NAME(n, tx),				      \
			(DT_INST_DMAS_CELL_BY_NAME(n, tx, channel)),		      \
			(0)),							      \
	};									      \
										      \
	static struct sun8i_codec_data codec_data_##n;			      \
										      \
	DEVICE_DT_INST_DEFINE(n,						      \
			      sun8i_codec_init,				      \
			      NULL,						      \
			      &codec_data_##n,				      \
			      &codec_config_##n,				      \
			      POST_KERNEL,					      \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY,		      \
			      &sun8i_codec_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_CODEC_INIT)
