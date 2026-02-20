/*
 * Copyright (c) 2022, Valerio Setti <vsetti@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_qdec

/** @file
 * @brief STM32 family Quadrature Decoder (QDEC) driver.
 */

#include <errno.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>

LOG_MODULE_REGISTER(qdec_stm32, CONFIG_SENSOR_LOG_LEVEL);

/* Device constant configuration parameters */
struct qdec_stm32_dev_cfg {
	const struct pinctrl_dev_config *pin_config;
	struct stm32_pclken pclken;
	TIM_TypeDef *timer_inst;
	uint32_t encoder_mode;
	bool is_input_polarity_inverted;
	uint8_t input_filtering_level;
	uint32_t counts_per_revolution;
#if IS_ENABLED(CONFIG_QDEC_STM32_TRIGGER)
	void (*irq_config)(const struct device *dev);
	void (*irq_connect)(void);
	int irq;
#endif
};

/* Device run time data */
struct qdec_stm32_dev_data {
	uint32_t position;
	uint32_t counts;
#if IS_ENABLED(CONFIG_QDEC_STM32_TRIGGER)
	const struct device *dev;
	const struct sensor_trigger *trig;
	sensor_trigger_handler_t handler;
	struct k_work work;
#endif
};

#if IS_ENABLED(CONFIG_QDEC_STM32_TRIGGER)

static void qdec_stm32_work(struct k_work *work)
{
	struct qdec_stm32_dev_data *data = CONTAINER_OF(work, struct qdec_stm32_dev_data, work);

	if (data->handler)
		data->handler(data->dev, data->trig);
}

static void qdec_stm32_isr(const struct device *dev)
{
	const struct qdec_stm32_dev_cfg *config = dev->config;
	struct qdec_stm32_dev_data *data = dev->data;
	TIM_TypeDef *timer = config->timer_inst;
	bool work_submitted = false;

	/* Main interrupt sources: Capture/Compare 1 and 2 */
	if (LL_TIM_IsActiveFlag_CC1(timer) && LL_TIM_IsEnabledIT_CC1(timer)) {
		LL_TIM_ClearFlag_CC1(timer);
		if (!work_submitted) {
			k_work_submit(&data->work);
			work_submitted = true;
		}
	}

	if (LL_TIM_IsActiveFlag_CC2(timer) && LL_TIM_IsEnabledIT_CC2(timer)) {
		LL_TIM_ClearFlag_CC2(timer);
		if (!work_submitted) {
			k_work_submit(&data->work);
			work_submitted = true;
		}
	}

	/*
	 * Defensively clear ALL other flags that share the same IRQ vector
	 * to prevent the interrupt line from getting stuck.
	 */
	if (LL_TIM_IsActiveFlag_UPDATE(timer)) {
		LL_TIM_ClearFlag_UPDATE(timer);
	}
	if (LL_TIM_IsActiveFlag_CC1OVR(timer)) {
		LL_TIM_ClearFlag_CC1OVR(timer);
	}
	if (LL_TIM_IsActiveFlag_CC2OVR(timer)) {
		LL_TIM_ClearFlag_CC2OVR(timer);
	}
	if (LL_TIM_IsActiveFlag_TRIG(timer)) {
		LL_TIM_ClearFlag_TRIG(timer);
	}
	if (LL_TIM_IsActiveFlag_COM(timer)) {
		LL_TIM_ClearFlag_COM(timer);
	}
}

static int qdec_stm32_trigger_set(const struct device *dev, const struct sensor_trigger *trig, sensor_trigger_handler_t handler)
{
	const struct qdec_stm32_dev_cfg *config = dev->config;
	struct qdec_stm32_dev_data *data = dev->data;
	TIM_TypeDef *timer = config->timer_inst;

	if (trig->type != SENSOR_TRIG_DATA_READY)
		return -ENOTSUP;

	LL_TIM_DisableIT_UPDATE(timer);

	data->trig = trig;
	data->handler = handler;

	if (handler == NULL)
		return 0;

	LL_TIM_EnableIT_CC1(timer);
	LL_TIM_EnableIT_CC2(timer);

	return 0;
}

#endif

static int qdec_stm32_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qdec_stm32_dev_data *dev_data = dev->data;
	const struct qdec_stm32_dev_cfg *dev_cfg = dev->config;
	uint32_t total_counter_value;
	uint32_t counter_value;

	if ((chan != SENSOR_CHAN_ALL) &&
	    (chan != SENSOR_CHAN_ROTATION) && (chan != SENSOR_CHAN_ENCODER_COUNT)) {
		return -ENOTSUP;
	}

	total_counter_value = LL_TIM_GetCounter(dev_cfg->timer_inst);
	dev_data->counts = total_counter_value;

	/* We're only interested in the remainder between the current counter value and
	 * counts_per_revolution. The integer part represents an entire rotation so it
	 * can be ignored
	 */
	counter_value = total_counter_value % dev_cfg->counts_per_revolution;

	/* The angle calculated in the fixed-point format (Q26.6 format) */
	dev_data->position = (counter_value * 23040) / dev_cfg->counts_per_revolution;

	return 0;
}

static int qdec_stm32_get(const struct device *dev, enum sensor_channel chan,
			  struct sensor_value *val)
{
	struct qdec_stm32_dev_data *const dev_data = dev->data;

	if (chan == SENSOR_CHAN_ROTATION) {
		val->val1 = dev_data->position >> 6;
		val->val2 = (dev_data->position & 0x3F) * 15625;
	} else if (chan == SENSOR_CHAN_ENCODER_COUNT) {
		val->val1 = dev_data->counts;
		val->val2 = 0;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static void qdec_stm32_initialize_channel(const struct device *dev, uint32_t ll_channel)
{
	const struct qdec_stm32_dev_cfg *const dev_cfg = dev->config;

	LL_TIM_IC_SetActiveInput(dev_cfg->timer_inst, ll_channel, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetFilter(dev_cfg->timer_inst, ll_channel,
			    dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2);
	LL_TIM_IC_SetPrescaler(dev_cfg->timer_inst, ll_channel, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetPolarity(dev_cfg->timer_inst, ll_channel,
			      dev_cfg->is_input_polarity_inverted ? LL_TIM_IC_POLARITY_FALLING :
								    LL_TIM_IC_POLARITY_RISING);
}

static int qdec_stm32_initialize(const struct device *dev)
{
	const struct qdec_stm32_dev_cfg *const dev_cfg = dev->config;
	struct qdec_stm32_dev_data *data = dev->data;
	int retval;
	uint32_t max_counter_value;

	retval = pinctrl_apply_state(dev_cfg->pin_config, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	retval = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				  (clock_control_subsys_t)&dev_cfg->pclken);
	if (retval < 0) {
		LOG_ERR("Could not initialize clock");
		return retval;
	}

	/* Ensure that the counter will always count up to a multiple of counts_per_revolution */
	if (IS_TIM_32B_COUNTER_INSTANCE(dev_cfg->timer_inst)) {
		max_counter_value = UINT32_MAX - (UINT32_MAX % dev_cfg->counts_per_revolution) - 1;
	} else {
		max_counter_value = UINT16_MAX - (UINT16_MAX % dev_cfg->counts_per_revolution) - 1;
	}
	LL_TIM_SetAutoReload(dev_cfg->timer_inst, max_counter_value);

	LL_TIM_SetClockSource(dev_cfg->timer_inst, dev_cfg->encoder_mode);

	qdec_stm32_initialize_channel(dev, LL_TIM_CHANNEL_CH1);
	qdec_stm32_initialize_channel(dev, LL_TIM_CHANNEL_CH2);

	LL_TIM_CC_EnableChannel(dev_cfg->timer_inst, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);

	LL_TIM_EnableCounter(dev_cfg->timer_inst);

#if IS_ENABLED(CONFIG_QDEC_STM32_TRIGGER)
	data->dev = dev;
	k_work_init(&data->work, qdec_stm32_work);
	dev_cfg->irq_config(dev);
#endif

	/* Clear any flags that may have been set during configuration */
	LL_TIM_ClearFlag_UPDATE(dev_cfg->timer_inst);

	return 0;
}

static DEVICE_API(sensor, qdec_stm32_driver_api) = {
	.sample_fetch = qdec_stm32_fetch,
	.channel_get = qdec_stm32_get,
#if IS_ENABLED(CONFIG_QDEC_STM32_TRIGGER)
	.trigger_set = qdec_stm32_trigger_set,
#endif
};

#if IS_ENABLED(CONFIG_QDEC_STM32_TRIGGER)
static void qdec_stm32_irq_config(const struct device *dev)
{
	const struct qdec_stm32_dev_cfg *config = dev->config;

	config->irq_connect();
	irq_enable(config->irq);
}
#endif

#define QDEC_STM32_INIT(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP(n, st_counts_per_revolution) > 0,                                \
		     "Counts per revolution must be above 0");                                     \
                                                                                                   \
	BUILD_ASSERT(!(DT_INST_PROP(n, st_encoder_mode) & ~TIM_SMCR_SMS),                          \
		     "Encoder mode is not supported by this MCU");                                 \
                                                                                            \
	IF_ENABLED(CONFIG_QDEC_STM32_TRIGGER,	(												\
		static void qdec_stm32_irq_connect_##n(void)										\
		{																					\
			IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ_BY_IDX(DT_INST_PARENT(n), 0, priority), qdec_stm32_isr, DEVICE_DT_INST_GET(n), 0); \
		}																					\
	));																						\
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct qdec_stm32_dev_cfg qdec##n##_stm32_config = {                          \
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                   \
		.timer_inst = ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n))),                     \
		.pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),				   \
		.encoder_mode = DT_INST_PROP(n, st_encoder_mode),                                  \
		.is_input_polarity_inverted = DT_INST_PROP(n, st_input_polarity_inverted),         \
		.input_filtering_level = DT_INST_PROP(n, st_input_filter_level),                   \
		.counts_per_revolution = DT_INST_PROP(n, st_counts_per_revolution),                \
		IF_ENABLED(CONFIG_QDEC_STM32_TRIGGER,	(											\
			.irq_config = qdec_stm32_irq_config,										\
			.irq_connect = qdec_stm32_irq_connect_##n,  									\
			.irq = DT_IRQN(DT_INST_PARENT(n)),												\
		))																				\
	};                                                                                         \
                                                                                                   \
	static struct qdec_stm32_dev_data qdec##n##_stm32_data;                                    \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, qdec_stm32_initialize, NULL, &qdec##n##_stm32_data,        \
				     &qdec##n##_stm32_config, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &qdec_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QDEC_STM32_INIT)
