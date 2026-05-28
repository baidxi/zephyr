/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun8i V3s DMA driver.
 *
 * 8 physical channels, linked-list descriptor architecture.
 * Reference: Linux drivers/dma/sun6i-dma.c
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include "dma_sun8i_v3s.h"

LOG_MODULE_REGISTER(dma_sun8i_v3s, CONFIG_DMA_LOG_LEVEL);

#define DT_DRV_COMPAT allwinner_sun8i_v3s_dma

#define LLI_POOL_SIZE CONFIG_DMA_SUN8I_V3S_LLI_POOL_SIZE
#define VCHAN_COUNT   CONFIG_DMA_SUN8I_V3S_VCHAN_COUNT

/*
 * Per-virtual-channel descriptor pool.
 * Since channels are used one transfer at a time, a per-vchan pool
 * is simpler than a shared heap.
 */
struct lli_pool {
	struct sun8i_v3s_dma_lli lli[LLI_POOL_SIZE];
	struct sun8i_v3s_dma_desc desc;
	uint32_t count;		/* number of lli used in current transfer */
};

static struct lli_pool lli_pools[VCHAN_COUNT] __aligned(4);

static void dma_cache_maintain(struct sun8i_v3s_dma_stream *stream,
			       bool after_xfer);

/* --------------------------------------------------------------------------
 * Helper: convert Zephyr burst/width to hardware encoding
 * -------------------------------------------------------------------------- */
static int convert_burst(uint32_t bytes)
{
	switch (bytes) {
	case 1:  return 0;
	case 8:  return 2;
	default: return -EINVAL;
	}
}

static int convert_buswidth(uint32_t bytes)
{
	switch (bytes) {
	case 1:  return 0;
	case 2:  return 1;
	case 4:  return 2;
	default: return -EINVAL;
	}
}

/* --------------------------------------------------------------------------
 * Descriptor linked-list management
 * -------------------------------------------------------------------------- */
static void lli_chain(struct sun8i_v3s_dma_lli *prev,
		      struct sun8i_v3s_dma_lli *next,
		      uintptr_t next_phy,
		      struct sun8i_v3s_dma_desc *desc)
{
	if (prev != NULL) {
		prev->p_lli_next = next_phy;
		prev->v_lli_next = next;
	} else {
		desc->v_lli = next;
		desc->p_lli = next_phy;
	}

	next->p_lli_next = LLI_LAST_ITEM;
	next->v_lli_next = NULL;
}

/* --------------------------------------------------------------------------
 * Build configuration word from dma_config + direction
 * -------------------------------------------------------------------------- */
static int build_cfg(const struct device *dev, struct dma_config *cfg, uint32_t *lli_cfg)
{
	uint32_t src_width, dst_width, src_burst, dst_burst;
	uint32_t src_mode = ADDR_MODE_LINEAR;
	uint32_t dst_mode = ADDR_MODE_LINEAR;
	int ret;

	src_width = cfg->source_data_size;
	dst_width = cfg->dest_data_size;
	src_burst = cfg->source_burst_length;
	dst_burst = cfg->dest_burst_length;

	switch (cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		if (src_burst == 0) {
			src_burst = 8;
		}
		if (dst_burst == 0) {
			dst_burst = 8;
		}
		if (src_width == 0) {
			src_width = 4;
		}
		if (dst_width == 0) {
			dst_width = 4;
		}
		break;
	case MEMORY_TO_PERIPHERAL:
		if (src_burst == 0) {
			src_burst = 8;
		}
		if (src_width == 0) {
			src_width = 4;
		}
		dst_mode = ADDR_MODE_IO;
		break;
	case PERIPHERAL_TO_MEMORY:
		if (dst_burst == 0) {
			dst_burst = 8;
		}
		if (dst_width == 0) {
			dst_width = 4;
		}
		src_mode = ADDR_MODE_IO;
		break;
	default:
		LOG_ERR("unsupported direction %u", cfg->channel_direction);
		return -EINVAL;
	}

	if (src_width == 0 || dst_width == 0) {
		LOG_ERR("data width not configured");
		return -EINVAL;
	}

	ret = convert_buswidth(src_width);
	if (ret < 0) {
		LOG_ERR("invalid src_width %u", src_width);
		return -EINVAL;
	}
	src_width = (uint32_t)ret;

	ret = convert_buswidth(dst_width);
	if (ret < 0) {
		LOG_ERR("invalid dst_width %u", dst_width);
		return -EINVAL;
	}
	dst_width = (uint32_t)ret;

	ret = convert_burst(src_burst);
	if (ret < 0) {
		LOG_ERR("invalid src_burst %u (only 1,8 supported)", src_burst);
		return -EINVAL;
	}
	src_burst = (uint32_t)ret;

	ret = convert_burst(dst_burst);
	if (ret < 0) {
		LOG_ERR("invalid dst_burst %u (only 1,8 supported)", dst_burst);
		return -EINVAL;
	}
	dst_burst = (uint32_t)ret;

	*lli_cfg = DMA_CFG_SRC_WIDTH(src_width) |
		   DMA_CFG_DST_WIDTH(dst_width) |
		   DMA_CFG_SRC_BURST(src_burst) |
		   DMA_CFG_DST_BURST(dst_burst) |
		   DMA_CFG_SRC_MODE(src_mode) |
		   DMA_CFG_DST_MODE(dst_mode);

	return 0;
}

/* --------------------------------------------------------------------------
 * DMA driver API
 * -------------------------------------------------------------------------- */

static int sun8i_v3s_dma_config(const struct device *dev, uint32_t channel,
				struct dma_config *cfg)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	struct sun8i_v3s_dma_stream *stream;
	struct lli_pool *pool;
	struct dma_block_config *blk;
	struct sun8i_v3s_dma_lli *prev = NULL;
	uint32_t lli_base_cfg;
	int ret;

	if (channel >= VCHAN_COUNT) {
		return -EINVAL;
	}

	stream = &data->streams[channel];
	if (stream->state != SUN8I_V3S_DMA_IDLE) {
		return -EBUSY;
	}

	if (cfg->head_block == NULL || cfg->block_count == 0) {
		return -EINVAL;
	}

	if (cfg->block_count > LLI_POOL_SIZE) {
		LOG_ERR("block_count %u exceeds pool size %u",
			cfg->block_count, LLI_POOL_SIZE);
		return -EINVAL;
	}

	ret = build_cfg(dev, cfg, &lli_base_cfg);
	if (ret < 0) {
		return ret;
	}

	pool = &lli_pools[channel];
	pool->count = 0;

	stream->desc = &pool->desc;
	stream->desc->v_lli = NULL;
	stream->desc->p_lli = 0;

	blk = cfg->head_block;
	for (uint32_t i = 0; i < cfg->block_count; i++) {
		struct sun8i_v3s_dma_lli *lli = &pool->lli[i];
		uint32_t lli_cfg = lli_base_cfg;

		if (blk == NULL) {
			return -EINVAL;
		}

		lli->cfg = lli_cfg;
		lli->src = blk->source_address;
		lli->dst = blk->dest_address;
		lli->len = blk->block_size;
		lli->para = (NORMAL_WAIT << 0);

		switch (cfg->channel_direction) {
		case MEMORY_TO_MEMORY:
			lli->cfg |= DMA_CFG_SRC_DRQ(DRQ_SDRAM) |
				    DMA_CFG_DST_DRQ(DRQ_SDRAM);
			break;
		case MEMORY_TO_PERIPHERAL:
			lli->cfg |= DMA_CFG_SRC_DRQ(DRQ_SDRAM) |
				    DMA_CFG_DST_DRQ(cfg->dma_slot);
			break;
		case PERIPHERAL_TO_MEMORY:
			lli->cfg |= DMA_CFG_SRC_DRQ(cfg->dma_slot) |
				    DMA_CFG_DST_DRQ(DRQ_SDRAM);
			break;
		default:
			return -EINVAL;
		}

		lli_chain(prev, lli, (uintptr_t)lli, stream->desc);
		prev = lli;
		pool->count++;
		blk = blk->next_block;
	}

	stream->dma_callback = cfg->dma_callback;
	stream->user_data = cfg->user_data;
	stream->port = (uint8_t)cfg->dma_slot;
	stream->cyclic = cfg->cyclic;
	stream->direction = cfg->channel_direction;
	stream->src_width = convert_buswidth(cfg->source_data_size);
	stream->dst_width = convert_buswidth(cfg->dest_data_size);
	stream->src_burst = convert_burst(cfg->source_burst_length);
	stream->dst_burst = convert_burst(cfg->dest_burst_length);
	/* Enable both PKG and QUEUE: the hardware may fire either
	 * for the last block in a chain, depending on transfer size.
	 */
	stream->irq_type = cfg->cyclic ? DMA_IRQ_PKG
				       : (DMA_IRQ_PKG | DMA_IRQ_QUEUE);
	stream->state = SUN8I_V3S_DMA_PREPARED;

	/* Flush LLI descriptors to memory — DMA reads from physical memory
	 * bypassing CPU cache on Cortex-A7.
	 */
	sys_cache_data_flush_range(&lli_pools[channel],
				   sizeof(struct lli_pool));

	LOG_DBG("config chan %u: slot=%u dir=%u blocks=%u cyclic=%d",
		channel, cfg->dma_slot, cfg->channel_direction,
		cfg->block_count, cfg->cyclic);

	return 0;
}

static int sun8i_v3s_dma_start(const struct device *dev, uint32_t channel)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	struct sun8i_v3s_dma_stream *stream;
	uint32_t reg_base = data->reg_base;
	int pchan = -1;

	if (channel >= VCHAN_COUNT) {
		return -EINVAL;
	}

	stream = &data->streams[channel];
	if (stream->state != SUN8I_V3S_DMA_PREPARED) {
		return -EINVAL;
	}

	/* Find a free physical channel */
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	for (int i = 0; i < 8; i++) {
		if (data->pchan_vchan[i] == VCHAN_COUNT) {
			pchan = i;
			data->pchan_vchan[i] = (uint8_t)channel;
			data->active_desc[i] = stream->desc;
			break;
		}
	}
	k_spin_unlock(&data->lock, key);

	if (pchan < 0) {
		/* No free physical channel — queue for later */
		LOG_DBG("no free pchan, queuing vchan %u", channel);
		key = k_spin_lock(&data->lock);
		sys_slist_append(&data->pending_streams, &stream->node);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	uintptr_t chan_base = reg_base + DMA_CHAN_OFFSET(pchan);

	/* Set up interrupts: clear old enables, set the type for this channel */
	uint32_t irq_reg = 0;
	for (int i = 0; i < 8; i++) {
		uint32_t shift = i * DMA_IRQ_WIDTH;
		uint32_t type;

		if (data->pchan_vchan[i] < VCHAN_COUNT) {
			type = data->streams[data->pchan_vchan[i]].irq_type;
		} else {
			type = 0;
		}
		irq_reg |= type << shift;
	}
	sys_write32(irq_reg, reg_base + DMA_IRQ_EN_REG0);

	/* Flush data buffers to memory before DMA accesses them */
	dma_cache_maintain(stream, false);

	sys_write32(stream->desc->p_lli, chan_base + DMA_CHAN_LLI_ADDR);
	sys_write32(DMA_CHAN_ENABLE_START, chan_base + DMA_CHAN_ENABLE);

	stream->state = SUN8I_V3S_DMA_ACTIVE;

	LOG_DBG("start vchan %u on pchan %d, lli=0x%lx",
		channel, pchan, (unsigned long)stream->desc->p_lli);

	return 0;
}

static int sun8i_v3s_dma_stop(const struct device *dev, uint32_t channel)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	struct sun8i_v3s_dma_stream *stream;
	uint32_t reg_base = data->reg_base;

	if (channel >= VCHAN_COUNT) {
		return -EINVAL;
	}

	stream = &data->streams[channel];

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Find which physical channel is running this stream */
	for (int i = 0; i < 8; i++) {
		if (data->pchan_vchan[i] == channel) {
			uintptr_t chan_base = reg_base + DMA_CHAN_OFFSET(i);

			uint32_t cur_cnt = sys_read32(chan_base + DMA_CHAN_CUR_CNT);
			uint32_t cur_src = sys_read32(chan_base + DMA_CHAN_CUR_SRC);
			uint32_t cur_dst = sys_read32(chan_base + DMA_CHAN_CUR_DST);
			uint32_t cur_cfg = sys_read32(chan_base + DMA_CHAN_CUR_CFG);
			LOG_WRN("stop chan %u pchan %d: cur_cnt=%u cur_src=0x%08x"
				" cur_dst=0x%08x cur_cfg=0x%08x",
				channel, i, cur_cnt, cur_src, cur_dst, cur_cfg);

			sys_write32(DMA_CHAN_PAUSE_STOP, chan_base + DMA_CHAN_PAUSE);
			sys_write32(0, chan_base + DMA_CHAN_ENABLE);

			/* Clear IRQ enable for this channel */
			uint32_t irq_en = sys_read32(reg_base + DMA_IRQ_EN_REG0);
			irq_en &= ~(0x7U << (i * DMA_IRQ_WIDTH));
			sys_write32(irq_en, reg_base + DMA_IRQ_EN_REG0);

			/* Clear any pending IRQs */
			sys_write32(0x7U << (i * DMA_IRQ_WIDTH),
				    reg_base + DMA_IRQ_PEND_REG0);

			data->pchan_vchan[i] = VCHAN_COUNT;
			data->active_desc[i] = NULL;
			break;
		}
	}

	/* Remove from pending list if queued */
	sys_slist_find_and_remove(&data->pending_streams, &stream->node);
	k_spin_unlock(&data->lock, key);

	stream->state = SUN8I_V3S_DMA_IDLE;

	LOG_DBG("stop vchan %u", channel);

	return 0;
}

static int sun8i_v3s_dma_suspend(const struct device *dev, uint32_t channel)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	uint32_t reg_base = data->reg_base;

	if (channel >= VCHAN_COUNT) {
		return -EINVAL;
	}

	for (int i = 0; i < 8; i++) {
		if (data->pchan_vchan[i] == channel) {
			sys_write32(DMA_CHAN_PAUSE_STOP,
				    reg_base + DMA_CHAN_OFFSET(i) + DMA_CHAN_PAUSE);
			data->streams[channel].state = SUN8I_V3S_DMA_SUSPENDED;
			return 0;
		}
	}

	return -EINVAL;
}

static int sun8i_v3s_dma_resume(const struct device *dev, uint32_t channel)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	uint32_t reg_base = data->reg_base;

	if (channel >= VCHAN_COUNT) {
		return -EINVAL;
	}

	if (data->streams[channel].state != SUN8I_V3S_DMA_SUSPENDED) {
		return -EINVAL;
	}

	for (int i = 0; i < 8; i++) {
		if (data->pchan_vchan[i] == channel) {
			sys_write32(0, reg_base + DMA_CHAN_OFFSET(i) + DMA_CHAN_PAUSE);
			data->streams[channel].state = SUN8I_V3S_DMA_ACTIVE;
			return 0;
		}
	}

	return -EINVAL;
}

static int sun8i_v3s_dma_get_status(const struct device *dev, uint32_t channel,
				    struct dma_status *stat)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	uint32_t reg_base = data->reg_base;

	if (channel >= VCHAN_COUNT || stat == NULL) {
		return -EINVAL;
	}

	stat->busy = false;
	stat->pending_length = 0;

	for (int i = 0; i < 8; i++) {
		if (data->pchan_vchan[i] == channel) {
			uintptr_t cb = reg_base + DMA_CHAN_OFFSET(i);

			stat->busy = !!(sys_read32(reg_base + DMA_STA_REG) & BIT(i));
			stat->pending_length = sys_read32(cb + DMA_CHAN_CUR_CNT);
			stat->dir = (enum dma_channel_direction)
				    data->streams[channel].direction;
			return 0;
		}
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * Cache maintenance helper — walk LLI chain and flush/invalidate buffers
 * -------------------------------------------------------------------------- */
static void dma_cache_maintain(struct sun8i_v3s_dma_stream *stream, bool after_xfer)
{
	const struct sun8i_v3s_dma_lli *lli = stream->desc->v_lli;

	while (lli != NULL) {
		if (stream->direction == MEMORY_TO_MEMORY) {
			if (!after_xfer) {
				/* Before DMA: flush src to memory */
				sys_cache_data_flush_range((void *)(uintptr_t)lli->src,
							   lli->len);
				sys_cache_data_flush_range((void *)(uintptr_t)lli->dst,
							   lli->len);
			} else {
				/* After DMA: invalidate dst to see DMA writes */
				sys_cache_data_invd_range((void *)(uintptr_t)lli->dst,
							  lli->len);
			}
		} else if (stream->direction == MEMORY_TO_PERIPHERAL) {
			if (!after_xfer) {
				sys_cache_data_flush_range((void *)(uintptr_t)lli->src,
							   lli->len);
			}
		} else if (stream->direction == PERIPHERAL_TO_MEMORY) {
			if (!after_xfer) {
				sys_cache_data_flush_range((void *)(uintptr_t)lli->dst,
							   lli->len);
			} else {
				sys_cache_data_invd_range((void *)(uintptr_t)lli->dst,
							  lli->len);
			}
		}

		lli = lli->v_lli_next;
	}
}

/* --------------------------------------------------------------------------
 * Interrupt handler
 * -------------------------------------------------------------------------- */
static void sun8i_v3s_dma_isr(const void *dev)
{
	struct sun8i_v3s_dma_data *data = ((const struct device *)dev)->data;
	struct sun8i_v3s_dma_stream *stream;
	uint32_t reg_base = data->reg_base;
	uint32_t pend;
	bool need_schedule = false;

	pend = sys_read32(reg_base + DMA_IRQ_PEND_REG0);
	if (!pend) {
		return;
	}

	/* Write 1 to clear pending bits */
	sys_write32(pend, reg_base + DMA_IRQ_PEND_REG0);

	for (int i = 0; i < 8; i++) {
		uint32_t mask = 0x7U << (i * DMA_IRQ_WIDTH);
		uint32_t chan_pend = pend & mask;

		if (!chan_pend) {
			continue;
		}

		uint8_t vchan_idx = data->pchan_vchan[i];
		if (vchan_idx >= VCHAN_COUNT) {
			continue;
		}

		stream = &data->streams[vchan_idx];

		if (stream->cyclic) {
			if (chan_pend & (DMA_IRQ_PKG << (i * DMA_IRQ_WIDTH))) {
				if (stream->dma_callback) {
					stream->dma_callback(dev,
						stream->user_data,
						vchan_idx,
						DMA_STATUS_BLOCK);
				}
			}
		} else {
			if (chan_pend & (stream->irq_type << (i * DMA_IRQ_WIDTH))) {
				dma_cache_maintain(stream, true);
				if (stream->dma_callback) {
					stream->dma_callback(dev,
						stream->user_data,
						vchan_idx,
						DMA_STATUS_COMPLETE);
				}
				/* Disable channel */
				sys_write32(0, reg_base + DMA_CHAN_OFFSET(i) + DMA_CHAN_ENABLE);
				/* Clear IRQ enable */
				uint32_t irq_en = sys_read32(reg_base + DMA_IRQ_EN_REG0);
				irq_en &= ~(0x7U << (i * DMA_IRQ_WIDTH));
				sys_write32(irq_en, reg_base + DMA_IRQ_EN_REG0);
				/* Mark pchan free */
				data->pchan_vchan[i] = VCHAN_COUNT;
				data->active_desc[i] = NULL;
				stream->state = SUN8I_V3S_DMA_IDLE;
				need_schedule = true;
			}
		}
	}

	if (need_schedule) {
		k_work_submit(&data->schedule_work);
	}
}

/* --------------------------------------------------------------------------
 * Work handler: start any queued transfers on freed physical channels
 * -------------------------------------------------------------------------- */
static void sun8i_v3s_dma_work(struct k_work *work)
{
	struct sun8i_v3s_dma_data *data =
		CONTAINER_OF(work, struct sun8i_v3s_dma_data, schedule_work);
	uint32_t reg_base = data->reg_base;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);

	sys_snode_t *prev = NULL;
	sys_snode_t *node;

	SYS_SLIST_FOR_EACH_NODE_SAFE(&data->pending_streams, prev, node) {
		struct sun8i_v3s_dma_stream *stream =
			CONTAINER_OF(node, struct sun8i_v3s_dma_stream, node);

		/* Find free physical channel */
		int pchan = -1;
		for (int i = 0; i < 8; i++) {
			if (data->pchan_vchan[i] == VCHAN_COUNT) {
				pchan = i;
				break;
			}
		}
		if (pchan < 0) {
			break;	/* no free channels */
		}

		uint8_t vchan_idx = (uint8_t)(stream - data->streams);
		sys_slist_remove(&data->pending_streams, prev, node);

		data->pchan_vchan[pchan] = vchan_idx;
		data->active_desc[pchan] = stream->desc;

		uintptr_t chan_base = reg_base + DMA_CHAN_OFFSET(pchan);

		/* Set IRQ enable for this channel */
		uint32_t irq_en = sys_read32(reg_base + DMA_IRQ_EN_REG0);
		uint32_t shift = pchan * DMA_IRQ_WIDTH;

		irq_en &= ~(0x7U << shift);
		irq_en |= stream->irq_type << shift;
		sys_write32(irq_en, reg_base + DMA_IRQ_EN_REG0);

		sys_write32(stream->desc->p_lli, chan_base + DMA_CHAN_LLI_ADDR);
		sys_write32(DMA_CHAN_ENABLE_START, chan_base + DMA_CHAN_ENABLE);

		stream->state = SUN8I_V3S_DMA_ACTIVE;

		LOG_DBG("work: started vchan %u on pchan %d", vchan_idx, pchan);
	}

	k_spin_unlock(&data->lock, key);
}

/* --------------------------------------------------------------------------
 * Initialization
 * -------------------------------------------------------------------------- */

struct sun8i_v3s_dma_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	struct reset_dt_spec reset;
	void (*irq_config_func)(const struct device *dev);
};

static int sun8i_v3s_dma_init(const struct device *dev)
{
	struct sun8i_v3s_dma_data *data = dev->data;
	const struct sun8i_v3s_dma_config *config = dev->config;
	int ret;

	/* Mark all pchans free */
	for (int i = 0; i < 8; i++) {
		data->pchan_vchan[i] = VCHAN_COUNT;
	}

	sys_slist_init(&data->pending_streams);
	k_work_init(&data->schedule_work, sun8i_v3s_dma_work);

	data->ctx.magic = DMA_MAGIC;
	data->ctx.dma_channels = VCHAN_COUNT;
	static atomic_t chan_atomic;
	data->ctx.atomic = &chan_atomic;
	atomic_clear(&chan_atomic);

	if (config->reset.dev != NULL) {
		if (!device_is_ready(config->reset.dev)) {
			LOG_ERR("reset controller not ready");
			return -ENODEV;
		}

		ret = reset_line_deassert_dt(&config->reset);
		if (ret < 0) {
			LOG_ERR("failed to de-assert DMA reset: %d", ret);
			return ret;
		}
	}

	if (config->clock_dev != NULL) {
		if (!device_is_ready(config->clock_dev)) {
			LOG_ERR("clock controller not ready");
			return -ENODEV;
		}

		ret = clock_control_on(config->clock_dev, config->clock_subsys);
		if (ret < 0) {
			LOG_ERR("failed to enable DMA bus clock: %d", ret);
			return ret;
		}
	}

	/* Must set bit 2 (MCLK circuit auto-gate disable) during init per datasheet */
	sys_write32(DMA_AUTO_GATE_ENABLE, data->reg_base + DMA_AUTO_GATE_REG);

	config->irq_config_func(dev);

	return 0;
}

/* --------------------------------------------------------------------------
 * API table and driver instantiation
 * -------------------------------------------------------------------------- */

static const struct dma_driver_api sun8i_v3s_dma_api = {
	.config = sun8i_v3s_dma_config,
	.start  = sun8i_v3s_dma_start,
	.stop   = sun8i_v3s_dma_stop,
	.suspend = sun8i_v3s_dma_suspend,
	.resume = sun8i_v3s_dma_resume,
	.get_status = sun8i_v3s_dma_get_status,
	/* Optional: get_attribute not implemented */
	/* Optional: chan_filter not implemented */
	/* Optional: chan_release not implemented */
	/* Optional: reload not implemented */
};

#define SUN8I_V3S_DMA_IRQ_CONFIG(n)								\
	static void dma_sun8i_v3s_irq_config_##n(const struct device *dev)			\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),				\
			    sun8i_v3s_dma_isr, DEVICE_DT_INST_GET(n),				\
			    DT_INST_IRQ(n, flags));						\
		irq_enable(DT_INST_IRQN(n));							\
	}

#define SUN8I_V3S_DMA_INIT(n)						\
	SUN8I_V3S_DMA_IRQ_CONFIG(n);					\
									\
	static const struct sun8i_v3s_dma_config			\
		dma_sun8i_v3s_config_##n = {				\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clock_subsys = (clock_control_subsys_t)		\
			DT_INST_CLOCKS_CELL(n, clk_id),			\
		.reset = RESET_DT_SPEC_INST_GET_OR(n, {0}),		\
		.irq_config_func = dma_sun8i_v3s_irq_config_##n,	\
	};								\
									\
	static struct sun8i_v3s_dma_data dma_sun8i_v3s_data_##n = {	\
		.reg_base = DT_INST_REG_ADDR(n),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n,					\
		sun8i_v3s_dma_init,					\
		NULL,							\
		&dma_sun8i_v3s_data_##n,				\
		&dma_sun8i_v3s_config_##n,				\
		POST_KERNEL,						\
		CONFIG_DMA_INIT_PRIORITY,				\
		&sun8i_v3s_dma_api);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_V3S_DMA_INIT)
