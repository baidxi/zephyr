#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>

#define DT_DRV_COMPAT  zephyr_gpio_pwm

#define PWM_TICK_RESOLUTION_US 10
#define GPIO_PWM_THREAD_STACK_SIZE 256

struct gpio_pwm_config {
	uint64_t freq_hz;
	const struct gpio_dt_spec *gpios;
	uint32_t nb_gpios;
};

struct gpio_pwm_channel_data {
	uint32_t period_cycles;
	uint32_t pulse_cycles;
	uint32_t period_ticks;
	uint32_t pulse_ticks;
	uint32_t counter;
};

struct gpio_pwm_data {
	k_tid_t tid;
	atomic_t active_channels;
	struct k_thread thread;
	struct gpio_pwm_channel_data *channel_data;
	K_KERNEL_STACK_MEMBER(stack, GPIO_PWM_THREAD_STACK_SIZE);
};

static void gpio_pwm_thread_entry_func(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct gpio_pwm_config *config = dev->config;
	struct gpio_pwm_data *data = dev->data;
	struct gpio_pwm_channel_data *channel;
	int i;

	while (1) {
		for (i = 0; i < config->nb_gpios; i++) {
			channel = &data->channel_data[i];

			if (channel->period_ticks == 0) {
				continue;
			}

			if (channel->counter == 0 && channel->pulse_ticks > 0) {
				gpio_pin_set_dt(&config->gpios[i], 1);
			} else if (channel->counter == channel->pulse_ticks) {
				gpio_pin_set_dt(&config->gpios[i], 0);
			}

			channel->counter++;

			if (channel->counter >= channel->period_ticks) {
				channel->counter = 0;
			}
		}
		k_busy_wait(PWM_TICK_RESOLUTION_US);
	}
}

static int gpio_pwm_set_cycles(const struct device *dev, uint32_t channel,
			       uint32_t period_cycles, uint32_t pulse_cycles,
			       pwm_flags_t flags)
{
	const struct gpio_pwm_config *config = dev->config;
	struct gpio_pwm_data *data = dev->data;
	struct gpio_pwm_channel_data *channel_p;
	bool old_is_active, new_is_active;

	if (channel >= config->nb_gpios) {
		return -EINVAL;
	}
	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}

	channel_p = &data->channel_data[channel];

	old_is_active = (channel_p->pulse_cycles > 0 &&
			 channel_p->pulse_cycles < channel_p->period_cycles);

	if (pulse_cycles == 0) {
		/* 0% duty cycle */
		new_is_active = false;
		if (channel_p->pulse_cycles != 0) {
			gpio_pin_set_dt(&config->gpios[channel], 0);
		}
	} else if (pulse_cycles == period_cycles) {
		/* 100% duty cycle */
		new_is_active = false;
		if (channel_p->pulse_cycles != channel_p->period_cycles) {
			gpio_pin_set_dt(&config->gpios[channel], 1);
		}
	} else {
		/* 1-99% duty cycle */
		new_is_active = true;
	}

	if (old_is_active && !new_is_active) {
		atomic_dec(&data->active_channels);
	} else if (!old_is_active && new_is_active) {
		atomic_inc(&data->active_channels);
	}

	channel_p->period_cycles = period_cycles;
	channel_p->pulse_cycles = pulse_cycles;

	if (new_is_active) {
		uint64_t period_us = USEC_PER_SEC * (uint64_t)period_cycles / config->freq_hz;
		uint64_t pulse_us = USEC_PER_SEC * (uint64_t)pulse_cycles / config->freq_hz;

		channel_p->period_ticks = period_us / PWM_TICK_RESOLUTION_US;
		channel_p->pulse_ticks = pulse_us / PWM_TICK_RESOLUTION_US;
		channel_p->counter = 0;
	} else {
		channel_p->period_ticks = 0;
		channel_p->pulse_ticks = 0;
	}

	if (atomic_get(&data->active_channels) > 0 && data->tid == NULL) {
		data->tid = k_thread_create(&data->thread, data->stack,
					    GPIO_PWM_THREAD_STACK_SIZE,
					    gpio_pwm_thread_entry_func,
					    (void *)dev, NULL, NULL,
					    CONFIG_PWM_INIT_PRIORITY, 0, K_NO_WAIT);
	} else if (atomic_get(&data->active_channels) == 0 && data->tid != NULL) {
		k_thread_abort(data->tid);
		data->tid = NULL;
	}

	return 0;
}

static int gpio_pwm_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	const struct gpio_pwm_config *config = dev->config;

	*cycles = config->freq_hz;

	return 0;
}

static int gpio_pwm_init(const struct device *dev)
{
	const struct gpio_pwm_config *config = dev->config;
	struct gpio_pwm_data *data = dev->data;
	const struct gpio_dt_spec *spec;
	int i;

	if (!config->nb_gpios) {
		return -ENODEV;
	}

	data->tid = NULL;
	atomic_set(&data->active_channels, 0);

	data->channel_data = k_malloc(config->nb_gpios * sizeof(*data->channel_data));
	if (!data->channel_data) {
		return -ENOMEM;
	}
	memset(data->channel_data, 0, config->nb_gpios * sizeof(*data->channel_data));

	for (i = 0, spec = config->gpios; i < config->nb_gpios; i++, spec++) {
		if (!device_is_ready(spec->port)) {
			return -ENODEV;
		}
		gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
	}

	return 0;
}

static const struct pwm_driver_api gpio_pwm_driver_api = {
	.set_cycles = gpio_pwm_set_cycles,
	.get_cycles_per_sec = gpio_pwm_get_cycles_per_sec,
};

#define GPIO_PWM_INST_INIT(inst)                                               \
	static struct gpio_pwm_data gpio_pwm_data_##inst;                       \
									       \
	static const struct gpio_dt_spec gpios_##inst[] = {                    \
		DT_INST_FOREACH_PROP_ELEM_SEP(inst, gpios,                     \
					      GPIO_DT_SPEC_GET_BY_IDX,         \
					      (,))                              \
	};                                                                     \
									       \
	static const struct gpio_pwm_config gpio_pwm_config_##inst = {          \
		.freq_hz = DT_INST_PROP(inst, frequency),                      \
		.gpios = gpios_##inst,                                         \
		.nb_gpios = ARRAY_SIZE(gpios_##inst),                          \
	};                                                                     \
									       \
	DEVICE_DT_INST_DEFINE(                                                 \
		inst, &gpio_pwm_init, NULL, &gpio_pwm_data_##inst,             \
		&gpio_pwm_config_##inst, POST_KERNEL,                          \
		CONFIG_PWM_INIT_PRIORITY, &gpio_pwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PWM_INST_INIT)