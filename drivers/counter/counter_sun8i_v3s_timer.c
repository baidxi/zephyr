/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_counter

#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(counter_sun8i_v3s, CONFIG_COUNTER_LOG_LEVEL);

#define TMR_IRQ_EN_REG		0x00
#define TMR_IRQ_STA_REG		0x04
#define TMR_CTRL_REG(n)		(0x10 + (n) * 0x10)
#define TMR_INTV_REG(n)		(0x14 + (n) * 0x10)
#define TMR_CUR_REG(n)		(0x18 + (n) * 0x10)

#define TMR_MODE_SINGLE		BIT(7)
#define TMR_CLK_PRES_SHIFT	4
#define TMR_CLK_SRC_OSC24M	(1 << 2)
#define TMR_RELOAD		BIT(1)
#define TMR_EN			BIT(0)

#define CLK_PRES_TO_DIV(p)	(1u << (p))
#define INTOSC_FREQ		32000

struct sun8i_counter_data {
	counter_top_callback_t top_cb;
	void *top_user_data;
	uint32_t guard_period;
};

struct sun8i_counter_config {
	struct counter_config_info info;
	uintptr_t base;
	uint8_t chan_id;
	uint8_t clock_source;
	uint8_t prescaler;
	void (*irq_config_func)(const struct device *dev);
};

#define DEV_CFG(dev) ((const struct sun8i_counter_config *)(dev)->config)
#define DEV_DAT(dev) ((struct sun8i_counter_data *)(dev)->data)

static uint32_t sun8i_read(const struct device *dev)
{
	return sys_read32(DEV_CFG(dev)->base +
			  TMR_CUR_REG(DEV_CFG(dev)->chan_id));
}

static int sun8i_start(const struct device *dev)
{
	const struct sun8i_counter_config *cfg = DEV_CFG(dev);
	uint32_t ctrl = TMR_EN;

	if (cfg->clock_source) {
		ctrl |= TMR_CLK_SRC_OSC24M;
	}
	ctrl |= (cfg->prescaler << TMR_CLK_PRES_SHIFT) | TMR_RELOAD;
	sys_write32(ctrl, cfg->base + TMR_CTRL_REG(cfg->chan_id));

	/* Enable interrupt for top callback */
	sys_set_bit(cfg->base + TMR_IRQ_EN_REG, cfg->chan_id);

	return 0;
}

static int sun8i_stop(const struct device *dev)
{
	sys_write32(0, DEV_CFG(dev)->base + TMR_CTRL_REG(DEV_CFG(dev)->chan_id));
	return 0;
}

static int sun8i_get_value(const struct device *dev, uint32_t *ticks)
{
	*ticks = sun8i_read(dev);
	return 0;
}

static uint32_t sun8i_get_top_value(const struct device *dev)
{
	return sys_read32(DEV_CFG(dev)->base + TMR_INTV_REG(DEV_CFG(dev)->chan_id));
}

static uint32_t sun8i_get_freq(const struct device *dev)
{
	return DEV_CFG(dev)->info.freq;
}

static int sun8i_set_top_value(const struct device *dev,
			       const struct counter_top_cfg *top_cfg)
{
	const struct sun8i_counter_config *cfg = DEV_CFG(dev);
	struct sun8i_counter_data *data = DEV_DAT(dev);

	if (!top_cfg->ticks) {
		return -EINVAL;
	}
	sys_write32(top_cfg->ticks, cfg->base + TMR_INTV_REG(cfg->chan_id));
	if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		sys_set_bit(cfg->base + TMR_CTRL_REG(cfg->chan_id),
			    __builtin_ctz(TMR_RELOAD));
	}
	data->top_cb = top_cfg->callback;
	data->top_user_data = top_cfg->user_data;
	return 0;
}

static uint32_t sun8i_get_pending_int(const struct device *dev)
{
	return sys_read32(DEV_CFG(dev)->base + TMR_IRQ_STA_REG) &
	       BIT(DEV_CFG(dev)->chan_id);
}

static int sun8i_set_alarm(const struct device *dev, uint8_t chan_id,
			   const struct counter_alarm_cfg *alarm_cfg)
{
	const struct sun8i_counter_config *cfg = DEV_CFG(dev);
	struct sun8i_counter_data *data = DEV_DAT(dev);
	uint32_t ticks = alarm_cfg->ticks;
	bool absolute = (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE);

	ARG_UNUSED(chan_id);

	if (absolute && ticks > sun8i_read(dev)) {
		return -ETIME;
	}

	uint32_t ctrl = TMR_MODE_SINGLE | TMR_EN;

	if (cfg->clock_source) {
		ctrl |= TMR_CLK_SRC_OSC24M;
	}
	ctrl |= (cfg->prescaler << TMR_CLK_PRES_SHIFT);

	if (!absolute) {
		ticks = sun8i_read(dev) - ticks;
	}
	sys_write32(ticks, cfg->base + TMR_INTV_REG(cfg->chan_id));
	sys_write32(ctrl | TMR_RELOAD, cfg->base + TMR_CTRL_REG(cfg->chan_id));
	sys_set_bit(cfg->base + TMR_IRQ_EN_REG, cfg->chan_id);

	/* Store alarm callback as top callback for channels=1 device */
	data->top_cb = (counter_top_callback_t)alarm_cfg->callback;
	data->top_user_data = alarm_cfg->user_data;
	return 0;
}

static int sun8i_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	const struct sun8i_counter_config *cfg = DEV_CFG(dev);

	ARG_UNUSED(chan_id);
	sys_write32(0, cfg->base + TMR_CTRL_REG(cfg->chan_id));
	sys_clear_bit(cfg->base + TMR_IRQ_EN_REG, cfg->chan_id);
	sys_write32(BIT(cfg->chan_id), cfg->base + TMR_IRQ_STA_REG);
	return 0;
}

static int sun8i_set_guard_period(const struct device *dev, uint32_t ticks,
				  uint32_t flags)
{
	ARG_UNUSED(flags);
	DEV_DAT(dev)->guard_period = ticks;
	return 0;
}

static uint32_t sun8i_get_guard_period(const struct device *dev, uint32_t flags)
{
	ARG_UNUSED(flags);
	return DEV_DAT(dev)->guard_period;
}

static void sun8i_isr(const struct device *dev)
{
	const struct sun8i_counter_config *cfg = DEV_CFG(dev);
	struct sun8i_counter_data *data = DEV_DAT(dev);

	/* Clear pending */
	sys_write32(BIT(cfg->chan_id), cfg->base + TMR_IRQ_STA_REG);

	/* Single-shot mode: hardware auto-stops at 0, disable IRQ */
	if (sys_read32(cfg->base + TMR_CTRL_REG(cfg->chan_id)) & TMR_MODE_SINGLE) {
		sys_write32(0, cfg->base + TMR_CTRL_REG(cfg->chan_id));
		sys_clear_bit(cfg->base + TMR_IRQ_EN_REG, cfg->chan_id);
	}

	if (data->top_cb) {
		data->top_cb(dev, data->top_user_data);
	}
}

static DEVICE_API(counter, sun8i_api) = {
	.start = sun8i_start,
	.stop = sun8i_stop,
	.get_value = sun8i_get_value,
	.get_top_value = sun8i_get_top_value,
	.get_pending_int = sun8i_get_pending_int,
	.set_alarm = sun8i_set_alarm,
	.cancel_alarm = sun8i_cancel_alarm,
	.set_top_value = sun8i_set_top_value,
	.set_guard_period = sun8i_set_guard_period,
	.get_guard_period = sun8i_get_guard_period,
	.get_freq = sun8i_get_freq,
};

#define PARENT(n) DT_PARENT(DT_DRV_INST(n))

#define SUN8I_INIT(n)								\
	static void sun8i_irq_config_##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_IRQ_BY_IDX(PARENT(n), 0, irq),			\
			    DT_IRQ_BY_IDX(PARENT(n), 0, priority),		\
			    sun8i_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_IRQ_BY_IDX(PARENT(n), 0, irq));			\
	}									\
										\
	static int sun8i_init_##n(const struct device *dev)			\
	{									\
		const struct sun8i_counter_config *cfg = dev->config;		\
		cfg->irq_config_func(dev);					\
		return 0;							\
	}									\
										\
	static const struct sun8i_counter_config config_##n = {			\
		.info = {							\
			.max_top_value = UINT32_MAX,				\
			.channels = 1,						\
			.flags = 0,						\
			.freq = (DT_INST_PROP(n, clock_source)			\
				? DT_PROP(DT_PHANDLE_BY_IDX(DT_DRV_INST(n),	\
					  clocks, 0), clock_frequency)		\
				: INTOSC_FREQ)					\
				/ CLK_PRES_TO_DIV(DT_INST_PROP(n, prescaler)),\
		},								\
		.base = DT_REG_ADDR(PARENT(n)),				\
		.chan_id = DT_INST_REG_ADDR(n),				\
		.clock_source = DT_INST_PROP(n, clock_source),			\
		.prescaler = DT_INST_PROP(n, prescaler),			\
		.irq_config_func = sun8i_irq_config_##n,			\
	};									\
										\
	static struct sun8i_counter_data data_##n;				\
										\
	DEVICE_DT_INST_DEFINE(n, sun8i_init_##n, NULL,				\
		&data_##n, &config_##n,						\
		POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY, &sun8i_api);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_INIT)
