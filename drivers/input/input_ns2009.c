/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for NSIWAY NS2009 4-wire resistive touchscreen controller.
 * Uses polling-based touch detection without PENIRQ interrupt pin.
 *
 * Touch/release detection strategy (baseline comparison):
 *
 *   The NS2009 returns fixed, identical X/Y coordinates on every poll
 *   when no finger is present.  When a finger presses the resistive
 *   film, the resistance changes and the ADC readings shift
 *   significantly from the no-finger baseline.
 *
 *   We exploit this by storing the no-finger baseline at init time,
 *   then comparing every poll's raw coordinates against it:
 *
 *   - If either axis differs from baseline by more than a threshold
 *     → finger is pressing.
 *   - If both axes stay near the baseline for N consecutive polls
 *     → finger was lifted.
 *
 *   BTN_TOUCH is sent ONLY on state transitions (press / release),
 *   never during a sustained press.  This prevents flooding the
 *   LVGL input message queue (8 slots), which was causing release
 *   events to be silently dropped.
 *
 * Datasheet: NS2009 Data Sheet V1.1 (September 2011)
 */

#define DT_DRV_COMPAT nsiway_ns2009

#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/minmax.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ns2009, LOG_LEVEL_INF);

/*
 * NS2009 command byte definitions (12-bit, differential, power-down).
 *
 * On this touchscreen, CH5 (0xD0) measures the horizontal axis and
 * CH4 (0xC0) measures the vertical axis.  The physical X (horizontal)
 * is therefore on CH5 and physical Y (vertical) on CH4.
 */
#define NS2009_CMD_X_POS	0xD0	/* Physical X = horizontal (CH5) */
#define NS2009_CMD_Y_POS	0xC0	/* Physical Y = vertical (CH4) */
#define NS2009_CMD_Z1_POS	0xE0	/* Measure Z1 pressure (CH6) */

/* ADC conversion time: typical 200us at 12-bit resolution */
#define NS2009_ADC_WAIT_US	200

/* Number of samples for median filter */
#ifdef CONFIG_INPUT_NS2009_SAMPLES
#define NS2009_NUM_SAMPLES	CONFIG_INPUT_NS2009_SAMPLES
#else
#define NS2009_NUM_SAMPLES	1
#endif

/*
 * Raw ADC threshold for detecting a finger press.
 *
 * When the raw coordinate in either axis differs from the stored
 * no-finger baseline by more than this value, a finger is
 * considered to be pressing the screen.
 *
 * The no-finger baseline is very stable (always the same value),
 * and even a light touch causes a shift of hundreds of ADC units,
 * so 30 is a very conservative threshold.
 */
#define NS2009_TOUCH_THRESHOLD	30

/*
 * Number of consecutive polls with coordinates near the baseline
 * before confirming the finger was lifted.
 *
 * At 15ms poll period, N=5 gives 75ms release latency.
 * This must be large enough to avoid false releases from brief
 * coordinate stability during a sustained press, but small enough
 * for responsive release detection.
 */
#define NS2009_RELEASE_COUNT	5

/*
 * ADC saturation threshold.  Readings at or near 4096 (12-bit max)
 * indicate a floating input with no finger contact.
 */
#define NS2009_ADC_SATURATION	4000

struct ns2009_config {
	/* Must be first field for input_touchscreen_common_config */
	struct input_touchscreen_common_config common;
	/* I2C bus specification */
	struct i2c_dt_spec bus;
	/* Polling period in milliseconds */
	uint32_t poll_period_ms;
	/* Calibration: raw ADC min/max */
	int min_x;
	int min_y;
	uint16_t max_x;
	uint16_t max_y;
};

struct ns2009_data {
	const struct device *dev;
	struct k_timer poll_timer;
	struct k_work poll_work;
	/* No-finger baseline coordinates (recorded at init) */
	uint16_t base_x;
	uint16_t base_y;
	/* Touch state */
	bool pressed;
	/*
	 * Number of consecutive polls where coordinates were near
	 * the baseline (no finger detected).
	 */
	uint32_t stable_count;
};

/* Compile-time check: common must be first in config struct */
INPUT_TOUCH_STRUCT_CHECK(struct ns2009_config);

static int ns2009_read_adc(const struct device *dev, uint8_t cmd, uint16_t *value)
{
	const struct ns2009_config *config = dev->config;
	uint8_t buf[2];
	int ret;

	/*
	 * Send channel command, wait for ADC conversion, then read.
	 * Separate write+read with delay is needed because the
	 * combined I2C transaction (repeated start) completes in
	 * ~100us at 100kHz, but the NS2009 needs ~200us for 12-bit
	 * conversion.  The STOP between write and read does not
	 * cause issues because the chip stores the result internally.
	 */
	ret = i2c_write_dt(&config->bus, &cmd, 1);
	if (ret < 0) {
		return ret;
	}

	k_usleep(NS2009_ADC_WAIT_US);

	ret = i2c_read_dt(&config->bus, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	*value = (buf[0] << 4) | (buf[1] >> 4);
	return 0;
}

static int ns2009_read_adc_filtered(const struct device *dev, uint8_t cmd, uint16_t *value)
{
	uint16_t samples[NS2009_NUM_SAMPLES];
	int ret;

	for (int i = 0; i < NS2009_NUM_SAMPLES; i++) {
		ret = ns2009_read_adc(dev, cmd, &samples[i]);
		if (ret < 0) {
			return ret;
		}
	}

	if (NS2009_NUM_SAMPLES == 1) {
		*value = samples[0];
	} else {
		for (int i = 0; i < NS2009_NUM_SAMPLES - 1; i++) {
			for (int j = i + 1; j < NS2009_NUM_SAMPLES; j++) {
				if (samples[i] > samples[j]) {
					uint16_t tmp = samples[i];
					samples[i] = samples[j];
					samples[j] = tmp;
				}
			}
		}
		*value = samples[NS2009_NUM_SAMPLES / 2];
	}

	return 0;
}

static void ns2009_report_touch(const struct device *dev, uint32_t raw_x, uint32_t raw_y)
{
	const struct ns2009_config *config = dev->config;
	const struct input_touchscreen_common_config *common = &config->common;
	int32_t x = raw_x;
	int32_t y = raw_y;

	if (common->screen_width > 0 && common->screen_height > 0) {
		x = ((int32_t)(raw_x - config->min_x) * common->screen_width)
		    / (config->max_x - config->min_x);
		y = ((int32_t)(raw_y - config->min_y) * common->screen_height)
		    / (config->max_y - config->min_y);

		x = clamp(x, 0, (int32_t)common->screen_width);
		y = clamp(y, 0, (int32_t)common->screen_height);
	}

	/* Log final coordinates (after inversion) that LVGL receives */
	int32_t fx = common->inverted_x ? common->screen_width - x : x;
	int32_t fy = common->inverted_y ? common->screen_height - y : y;

	LOG_INF("report: raw=[%u,%u] lvgl=[%d,%d]", raw_x, raw_y, fx, fy);
	input_touchscreen_report_pos(dev, x, y, K_NO_WAIT);
}

static void ns2009_poll_work_handler(struct k_work *work)
{
	struct ns2009_data *data = CONTAINER_OF(work, struct ns2009_data, poll_work);
	const struct device *dev = data->dev;
	uint16_t x, y;
	int ret;

	/* Read filtered X/Y coordinates */
	ret = ns2009_read_adc_filtered(dev, NS2009_CMD_X_POS, &x);
	if (ret < 0) {
		return;
	}

	ret = ns2009_read_adc_filtered(dev, NS2009_CMD_Y_POS, &y);
	if (ret < 0) {
		return;
	}

	/*
	 * Determine whether a finger is present by comparing raw
	 * coordinates to the no-finger baseline.
	 *
	 * Two conditions must BOTH be true for a touch:
	 *   1. Coordinates are not in saturation range (>=4000 or <50)
	 *   2. At least one axis differs from baseline by > threshold
	 *
	 * The saturation check filters out floating-input readings.
	 * The baseline comparison detects the resistance change from
	 * finger pressure.
	 */
	bool touch = false;

	if (x < NS2009_ADC_SATURATION && y < NS2009_ADC_SATURATION &&
	    x >= 50 && y >= 50) {
		int32_t dx = (int32_t)x - (int32_t)data->base_x;
		int32_t dy = (int32_t)y - (int32_t)data->base_y;

		if (dx < 0) {
			dx = -dx;
		}
		if (dy < 0) {
			dy = -dy;
		}

		touch = (dx > NS2009_TOUCH_THRESHOLD ||
			 dy > NS2009_TOUCH_THRESHOLD);
	}

	if (touch) {
		/* Coordinates differ from baseline → finger is present */
		data->stable_count = 0;

		if (!data->pressed) {
			/* NOT_PRESSED → PRESSED: first detection */
			data->pressed = true;
			ns2009_report_touch(dev, x, y);
			input_report_key(dev, INPUT_BTN_TOUCH, 1, true,
					 K_NO_WAIT);
			LOG_INF("touch down raw=[%u,%u] base=[%u,%u]",
				x, y, data->base_x, data->base_y);
		}
		/*
		 * While pressed, we do NOT send further BTN_TOUCH events.
		 * LVGL's lvgl_input_read_cb reuses previous_event when
		 * the msgq is empty, keeping the PRESSED state alive.
		 * This prevents msgq flooding (8 slots at 15ms = 120ms).
		 */
	} else {
		/* Coordinates near baseline → no finger */
		data->stable_count++;

		if (data->pressed &&
		    data->stable_count >= NS2009_RELEASE_COUNT) {
			/* PRESSED → RELEASED */
			data->pressed = false;
			data->stable_count = 0;
			input_report_key(dev, INPUT_BTN_TOUCH, 0, true,
					 K_NO_WAIT);
			LOG_INF("touch up (stable %u polls)",
				NS2009_RELEASE_COUNT);
		}
	}
}

static void ns2009_timer_handler(struct k_timer *timer)
{
	struct ns2009_data *data = CONTAINER_OF(timer, struct ns2009_data, poll_timer);

	k_work_submit(&data->poll_work);
}

static int ns2009_init(const struct device *dev)
{
	const struct ns2009_config *config = dev->config;
	struct ns2009_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR("I2C bus not ready!");
		return -ENODEV;
	}

	data->dev = dev;
	data->pressed = false;
	data->stable_count = 0;

	/*
	 * Establish the no-finger baseline coordinates.
	 * Use filtered reads (median of N samples) for reliability.
	 * The NS2009 must be untouched during driver init.
	 */
	int ret = ns2009_read_adc_filtered(dev, NS2009_CMD_X_POS,
					    &data->base_x);
	if (ret < 0) {
		LOG_ERR("baseline X read failed");
		return -EIO;
	}

	ret = ns2009_read_adc_filtered(dev, NS2009_CMD_Y_POS,
				       &data->base_y);
	if (ret < 0) {
		LOG_ERR("baseline Y read failed");
		return -EIO;
	}

	LOG_INF("baseline: X=%u Y=%u (no finger)", data->base_x, data->base_y);
	LOG_INF("started: poll=%ums release_n=%u threshold=%u screen=%ux%u",
		config->poll_period_ms, NS2009_RELEASE_COUNT,
		NS2009_TOUCH_THRESHOLD,
		config->common.screen_width, config->common.screen_height);

	k_work_init(&data->poll_work, ns2009_poll_work_handler);
	k_timer_init(&data->poll_timer, ns2009_timer_handler, NULL);
	k_timer_start(&data->poll_timer,
		      K_MSEC(config->poll_period_ms),
		      K_MSEC(config->poll_period_ms));

	return 0;
}

#define NS2009_INIT(inst)								\
	BUILD_ASSERT(DT_INST_PROP(inst, max_x) > DT_INST_PROP(inst, min_x),		\
		     "max-x must be greater than min-x");				\
	BUILD_ASSERT(DT_INST_PROP(inst, max_y) > DT_INST_PROP(inst, min_y),		\
		     "max-y must be greater than min-y");				\
											\
	static const struct ns2009_config ns2009_config_##inst = {			\
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(inst),			\
		.bus = I2C_DT_SPEC_INST_GET(inst),					\
		.poll_period_ms = DT_INST_PROP(inst, poll_period_ms),			\
		.min_x = DT_INST_PROP(inst, min_x),					\
		.min_y = DT_INST_PROP(inst, min_y),					\
		.max_x = DT_INST_PROP(inst, max_x),					\
		.max_y = DT_INST_PROP(inst, max_y),					\
	};										\
											\
	static struct ns2009_data ns2009_data_##inst;					\
											\
	DEVICE_DT_INST_DEFINE(inst, ns2009_init, NULL,					\
			      &ns2009_data_##inst, &ns2009_config_##inst,		\
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NS2009_INIT)
