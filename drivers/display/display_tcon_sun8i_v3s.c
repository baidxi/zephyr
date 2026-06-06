/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner V3s TCON (Timing Controller) driver.
 *
 * Generates HSYNC/VSYNC/DE/DCLK for RGB parallel LCD panels.
 * Based on Linux sun4i_tcon.c sun4i_tcon0_mode_set_rgb().
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_tcon

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include "tcon_sun8i_v3s.h"

LOG_MODULE_REGISTER(tcon_v3s, CONFIG_DISPLAY_LOG_LEVEL);

#define TCON_GCTL_REG            0x00
#define   TCON_GCTL_TCON_ENABLE  BIT(31)
#define   TCON_GCTL_IOMAP_MASK   BIT(0)
#define   TCON_GCTL_IOMAP_TCON0  (0 << 0)

#define TCON0_DCLK_REG           0x44
#define   TCON0_DCLK_GATE        BIT(31)
#define   TCON0_DCLK_DIV_SHIFT   0
#define   TCON0_DCLK_DIV_MASK    GENMASK(6, 0)

#define TCON0_CTL_REG            0x40
#define   TCON0_CTL_TCON_ENABLE  BIT(31)
#define   TCON0_CTL_CLK_DELAY_MASK  GENMASK(8, 4)
#define   TCON0_CTL_CLK_DELAY(d)    ((d) << 4)

#define TCON0_BASIC0_REG         0x48
#define   TCON0_BASIC0_X(w)      (((w) - 1) << 16)
#define   TCON0_BASIC0_Y(h)      ((h) - 1)

#define TCON0_BASIC1_REG         0x4C
#define   TCON0_BASIC1_H_TOTAL(t)   (((t) - 1) << 16)
#define   TCON0_BASIC1_H_BACKPORCH(b) ((b) - 1)

#define TCON0_BASIC2_REG         0x50
#define   TCON0_BASIC2_V_TOTAL(t)   ((t) << 16)
#define   TCON0_BASIC2_V_BACKPORCH(b) ((b) - 1)

#define TCON0_BASIC3_REG         0x54
#define   TCON0_BASIC3_H_SYNC(w)    (((w) - 1) << 16)
#define   TCON0_BASIC3_V_SYNC(h)    ((h) - 1)

#define TCON0_IO_POL_REG         0x88
#define   TCON0_IO_POL_DCLK_PHASE(p)   ((p & 3) << 28)
#define   TCON0_IO_POL_DE_NEGATIVE     BIT(27)
#define   TCON0_IO_POL_DCLK_DRIVE_NEGEDGE BIT(26)
#define   TCON0_IO_POL_HSYNC_POSITIVE  BIT(25)
#define   TCON0_IO_POL_VSYNC_POSITIVE  BIT(24)

#define TCON0_IO_TRI_REG         0x8C
#define   TCON0_IO_TRI_ALL_ENABLE  0x00000000

#define TCON_FRM_CTL_REG         0x10
#define   TCON0_FRM_CTL_EN       BIT(31)
#define   TCON0_FRM_CTL_MODE_R   BIT(6)
#define   TCON0_FRM_CTL_MODE_G   BIT(5)
#define   TCON0_FRM_CTL_MODE_B   BIT(4)

#define TCON0_FRM_SEED_PR_REG    0x14
#define TCON0_FRM_SEED_PG_REG    0x18
#define TCON0_FRM_SEED_PB_REG    0x1C
#define TCON0_FRM_SEED_LR_REG    0x20
#define TCON0_FRM_SEED_LG_REG    0x24
#define TCON0_FRM_SEED_LB_REG    0x28
#define TCON0_FRM_TBL0_REG       0x2C
#define TCON0_FRM_TBL1_REG       0x30
#define TCON0_FRM_TBL2_REG       0x34
#define TCON0_FRM_TBL3_REG       0x38

/* Maximum CLK_DELAY value (TCON0_CTL bits[8:4] = 5 bits, max 31) */
#define CLK_DELAY_MAX             30

struct tcon_v3s_data {
	uintptr_t base;
	bool output_enabled;
	uint8_t brightness;		/* Current brightness (0-255) */
};

struct tcon_v3s_config {
	uintptr_t base;
	const struct device *clock_dev;
	clock_control_subsys_t ahb_clk;
	clock_control_subsys_t ch0_clk;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	struct tcon_timings timings;
	struct gpio_dt_spec backlight_gpio;
	const struct device *backlight_pwm_dev;
	uint32_t backlight_pwm_channel;
	uint32_t backlight_pwm_freq;	/* Frequency in Hz */
};

static void tcon0_set_dithering(uintptr_t base)
{
	sys_write32(0x11111111, base + TCON0_FRM_SEED_PR_REG);
	sys_write32(0x11111111, base + TCON0_FRM_SEED_PG_REG);
	sys_write32(0x11111111, base + TCON0_FRM_SEED_PB_REG);
	sys_write32(0x11111111, base + TCON0_FRM_SEED_LR_REG);
	sys_write32(0x11111111, base + TCON0_FRM_SEED_LG_REG);
	sys_write32(0x11111111, base + TCON0_FRM_SEED_LB_REG);

	sys_write32(0x01010000, base + TCON0_FRM_TBL0_REG);
	sys_write32(0x06060606, base + TCON0_FRM_TBL1_REG);
	sys_write32(0x10101010, base + TCON0_FRM_TBL2_REG);
	sys_write32(0x16161616, base + TCON0_FRM_TBL3_REG);
	sys_write32(0, base + TCON_FRM_CTL_REG);
}

static uint8_t tcon_get_clk_delay(uint16_t vtotal, uint16_t vdisplay)
{
	int delay = vtotal - vdisplay;

	if (delay > CLK_DELAY_MAX) {
		delay = CLK_DELAY_MAX;
	}
	if (delay < 1) {
		delay = 1;
	}
	return (uint8_t)delay;
}

static void tcon_read_timings_from_dt(const struct device *dev)
{
	struct tcon_v3s_config *cfg = (struct tcon_v3s_config *)dev->config;

#define TCON_TIMING(node_id, prop) DT_PROP(DT_CHILD(DT_DRV_INST(0), display_timings), prop)

	cfg->timings.hsync_len    = TCON_TIMING(DT_DRV_INST(0), hsync_len);
	cfg->timings.vsync_len    = TCON_TIMING(DT_DRV_INST(0), vsync_len);
	cfg->timings.hback_porch  = TCON_TIMING(DT_DRV_INST(0), hback_porch);
	cfg->timings.vback_porch  = TCON_TIMING(DT_DRV_INST(0), vback_porch);
	cfg->timings.hfront_porch = TCON_TIMING(DT_DRV_INST(0), hfront_porch);
	cfg->timings.vfront_porch = TCON_TIMING(DT_DRV_INST(0), vfront_porch);

	cfg->timings.pixel_clock_hz = TCON_TIMING(DT_DRV_INST(0), clock_frequency);

	cfg->timings.hsync_active_low  = !TCON_TIMING(DT_DRV_INST(0), hsync_active);
	cfg->timings.vsync_active_low  = !TCON_TIMING(DT_DRV_INST(0), vsync_active);
	cfg->timings.de_active_high    =  TCON_TIMING(DT_DRV_INST(0), de_active);
	cfg->timings.pixelclk_active_low = !TCON_TIMING(DT_DRV_INST(0), pixelclk_active);

#undef TCON_TIMING

	LOG_DBG("Timings from DT: %ux%u H=%u/%u/%u V=%u/%u/%u %uHz",
		cfg->timings.width, cfg->timings.height,
		cfg->timings.hsync_len,
		cfg->timings.hback_porch,
		cfg->timings.hfront_porch,
		cfg->timings.vsync_len,
		cfg->timings.vback_porch,
		cfg->timings.vfront_porch,
		cfg->timings.pixel_clock_hz);
}

const struct tcon_timings *tcon_get_timings(const struct device *dev)
{
	const struct tcon_v3s_config *cfg = dev->config;
	return &cfg->timings;
}

int tcon_apply_timings(const struct device *dev, const struct tcon_timings *t)
{
	const struct tcon_v3s_config *cfg = dev->config;
	struct tcon_v3s_data *data = dev->data;
	uintptr_t base = data->base;
	uint32_t h_total, v_total;
	uint32_t h_bp, v_bp;
	uint32_t pol = 0;
	uint8_t clk_delay;
	int ret;

	/* ---- Configure CLK_TCON0 source/divider ---- */
	/* Bug #9 fix: clock_control_on() only sets the gate bit.
	 * We must also configure the source mux and M-divider via
	 * clock_control_set_rate() so that CLK_TCON0 produces a
	 * usable clock even if PLL_VIDEO was not set up by bootloader.
	 */
	if (t->pixel_clock_hz != 0) {
		ret = clock_control_set_rate(cfg->clock_dev, cfg->ch0_clk,
					     (clock_control_subsys_rate_t)
					     (uintptr_t)t->pixel_clock_hz);
		if (ret != 0) {
			LOG_WRN("CLK_TCON0 set_rate(%u) failed: %d",
				t->pixel_clock_hz, ret);
		} else {
			LOG_DBG("CLK_TCON0 set_rate(%u Hz) OK",
				t->pixel_clock_hz);
		}
	}

	/* ---- Set DCLK divider ---- */
	/* TCON0_DCLK_REG bits[6:0] = DCLKDIV (V3s TCON datasheet §7.2.5.8):
	 *   Tdclk = Tsclk * DCLKDIV  →  fdclk = fsclk / DCLKDIV
	 *   Constraint: "if dclk only, DCLKDIV >= 1"
	 *
	 * Linux sun4i_dclk_set_rate() writes div = parent_rate / rate
	 * directly (no +1 / -1 offset).  Previously we subtracted 1,
	 * which produced DCLKDIV=0 when parent ≈ target (violating the
	 * DCLKDIV>=1 constraint and likely disabling the DCLK output).
	 */
	uint32_t parent_rate = 0;
	ret = clock_control_get_rate(cfg->clock_dev, cfg->ch0_clk,
				     &parent_rate);
	if (ret != 0 || parent_rate == 0) {
		LOG_WRN("CLK_TCON0 rate unavailable, assuming 297 MHz");
		parent_rate = 297000000U;
	}

	uint32_t div;
	if (t->pixel_clock_hz == 0 || t->pixel_clock_hz >= parent_rate) {
		div = 1; /* minimum: DCLKDIV >= 1, /1 = no division */
	} else {
		/* Round to nearest, matching Linux sun4i_dclk_round_rate */
		uint32_t best_div = 1;
		uint32_t best_err = UINT32_MAX;

		for (uint32_t d = 1; d <= TCON0_DCLK_DIV_MASK; d++) {
			uint32_t actual = parent_rate / d;
			uint32_t err = (actual > t->pixel_clock_hz)
				? (actual - t->pixel_clock_hz)
				: (t->pixel_clock_hz - actual);
			if (err < best_err) {
				best_err = err;
				best_div = d;
			}
		}
		div = best_div;
	}

	uint32_t actual_dclk = parent_rate / div;
	uint32_t dclk_val = TCON0_DCLK_GATE | (div << TCON0_DCLK_DIV_SHIFT);
	sys_write32(dclk_val, base + TCON0_DCLK_REG);

	LOG_DBG("DCLK: parent=%u DCLKDIV=%u actual=%u Hz target=%u Hz reg=0x%08x",
		parent_rate, div, actual_dclk, t->pixel_clock_hz, dclk_val);

	/* ---- Compute total timings (needed for CLK_DELAY) ---- */
	h_total = t->width + t->hsync_len + t->hback_porch + t->hfront_porch;
	v_total = t->height + t->vsync_len + t->vback_porch + t->vfront_porch;
	h_bp    = t->hsync_len + t->hback_porch;
	v_bp    = t->vsync_len + t->vback_porch;

	/* ---- Resolution ---- */
	sys_write32(TCON0_BASIC0_X(t->width) | TCON0_BASIC0_Y(t->height),
		    base + TCON0_BASIC0_REG);

	/* ---- Dithering seeds/tables (Bug #21a: match Linux) ---- */
	tcon0_set_dithering(base);

	/* ---- Clock delay (Bug #20 fix: use vtotal-vdisplay, not pixel clock) ---- */
	clk_delay = tcon_get_clk_delay(v_total, t->height);
	uint32_t ctl = sys_read32(base + TCON0_CTL_REG);
	ctl &= ~TCON0_CTL_CLK_DELAY_MASK;
	ctl |= TCON0_CTL_CLK_DELAY(clk_delay);
	sys_write32(ctl, base + TCON0_CTL_REG);
	LOG_DBG("CLK_DELAY: vtotal=%u vdisplay=%u delay=%u",
		v_total, t->height, clk_delay);

	/* ---- Horizontal timings ---- */
	sys_write32(TCON0_BASIC1_H_TOTAL(h_total) |
		    TCON0_BASIC1_H_BACKPORCH(h_bp),
		    base + TCON0_BASIC1_REG);

	/* ---- Vertical timings ---- */
	sys_write32(TCON0_BASIC2_V_TOTAL(v_total * 2) |
		    TCON0_BASIC2_V_BACKPORCH(v_bp),
		    base + TCON0_BASIC2_REG);

	/* ---- Sync pulse widths ---- */
	sys_write32(TCON0_BASIC3_H_SYNC(t->hsync_len) |
		    TCON0_BASIC3_V_SYNC(t->vsync_len),
		    base + TCON0_BASIC3_REG);

	/* ---- Signal polarities ---- */
	if (!t->hsync_active_low) {
		pol |= TCON0_IO_POL_HSYNC_POSITIVE;
	}
	if (!t->vsync_active_low) {
		pol |= TCON0_IO_POL_VSYNC_POSITIVE;
	}
	if (!t->de_active_high) {
		pol |= TCON0_IO_POL_DE_NEGATIVE;
	}
	if (t->pixelclk_active_low) {
		pol |= TCON0_IO_POL_DCLK_DRIVE_NEGEDGE;
	}
	sys_write32(pol, base + TCON0_IO_POL_REG);

	/* ---- IOMAP: select TCON0 ---- */
	sys_write32(TCON_GCTL_IOMAP_TCON0, base + TCON_GCTL_REG);

	LOG_DBG("TCON timings applied: %ux%u @ %u Hz", t->width, t->height,
		t->pixel_clock_hz);
	LOG_DBG("  BASIC0=0x%08x BASIC1=0x%08x",
		sys_read32(base + 0x48), sys_read32(base + 0x4C));
	LOG_DBG("  BASIC2=0x%08x BASIC3=0x%08x",
		sys_read32(base + 0x50), sys_read32(base + 0x54));
	LOG_DBG("  DCLK  =0x%08x IO_POL=0x%08x",
		sys_read32(base + 0x44), sys_read32(base + 0x88));
	return 0;
}

int tcon_set_brightness(const struct device *dev, uint8_t brightness)
{
	const struct tcon_v3s_config *cfg = dev->config;
	struct tcon_v3s_data *data = dev->data;

	if (cfg->backlight_pwm_dev == NULL) {
		LOG_WRN("No PWM backlight device configured");
		return -ENOTSUP;
	}

	if (!device_is_ready(cfg->backlight_pwm_dev)) {
		LOG_ERR("PWM device %s not ready!",
			cfg->backlight_pwm_dev->name);
		return -ENODEV;
	}

	data->brightness = brightness;

	/*
	 * backlight_pwm_freq stores the value from the DTS backlight property's
	 * period cell.  The TCON binding documents this as frequency in Hz.
	 * Example: backlight = <&pwm 0 1000> → backlight_pwm_freq = 1000 (Hz).
	 */
	uint32_t period_ns = NSEC_PER_SEC / cfg->backlight_pwm_freq;
	uint32_t pulse_ns = (uint64_t)period_ns * brightness / 255;

	LOG_DBG("set_brightness: freq_val=%u period_ns=%u pulse_ns=%u ch=%u",
		cfg->backlight_pwm_freq, period_ns, pulse_ns,
		cfg->backlight_pwm_channel);

	int ret = pwm_set(cfg->backlight_pwm_dev, cfg->backlight_pwm_channel,
			  period_ns, pulse_ns, 0);
	if (ret) {
		LOG_ERR("PWM set failed: %d", ret);
		return ret;
	}

	LOG_DBG("Backlight brightness: %u/255 OK", brightness);
	return 0;
}

int tcon_enable_output(const struct device *dev, bool enable)
{
	const struct tcon_v3s_config *cfg = dev->config;
	struct tcon_v3s_data *data = dev->data;
	uintptr_t base = data->base;
	uint32_t reg;

	if (enable) {
		/* Enable order: global → channel → output pins */
		sys_write32(TCON_GCTL_IOMAP_TCON0 | TCON_GCTL_TCON_ENABLE,
			    base + TCON_GCTL_REG);
		reg = sys_read32(base + TCON0_CTL_REG);
		reg |= TCON0_CTL_TCON_ENABLE;
		sys_write32(reg, base + TCON0_CTL_REG);
		sys_write32(TCON0_IO_TRI_ALL_ENABLE, base + TCON0_IO_TRI_REG);
		LOG_DBG("TCON output enabled: GCTL=0x%08x CTL=0x%08x TRI=0x%08x",
			sys_read32(base + TCON_GCTL_REG),
			sys_read32(base + TCON0_CTL_REG),
			sys_read32(base + TCON0_IO_TRI_REG));

		/* Enable LCD backlight */
		if (cfg->backlight_pwm_dev != NULL) {
			/* PWM backlight: restore brightness */
			tcon_set_brightness(dev, data->brightness);
		} else if (cfg->backlight_gpio.port != NULL) {
			/* GPIO backlight: set HIGH */
			gpio_pin_set_dt(&cfg->backlight_gpio, 1);
		}
	} else {
		/* Disable LCD backlight before shutting down TCON signals */
		if (cfg->backlight_pwm_dev != NULL) {
			/* PWM backlight: 0% duty cycle */
			uint32_t period_ns = NSEC_PER_SEC / cfg->backlight_pwm_freq;
			pwm_set(cfg->backlight_pwm_dev, cfg->backlight_pwm_channel,
				period_ns, 0, 0);
		} else if (cfg->backlight_gpio.port != NULL) {
			gpio_pin_set_dt(&cfg->backlight_gpio, 0);
		}

		reg = sys_read32(base + TCON0_CTL_REG);
		reg &= ~TCON0_CTL_TCON_ENABLE;
		sys_write32(reg, base + TCON0_CTL_REG);
		sys_write32(0, base + TCON_GCTL_REG);
		LOG_DBG("TCON output disabled");
	}

	data->output_enabled = enable;
	return 0;
}

static int tcon_v3s_init(const struct device *dev)
{
	const struct tcon_v3s_config *cfg = dev->config;
	struct tcon_v3s_data *data = dev->data;
	int ret;

	data->base = cfg->base;

	/* ---- Apply pin multiplexing for LCD output ---- */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_WRN("TCON pinctrl apply failed: %d (LCD pins may not be muxed)", ret);
	}

	/* Enable AHB bus clock */
	ret = clock_control_on(cfg->clock_dev, cfg->ahb_clk);
	if (ret) {
		LOG_ERR("Failed to enable AHB clock: %d", ret);
		return ret;
	}

	/* Enable channel 0 pixel clock */
	ret = clock_control_on(cfg->clock_dev, cfg->ch0_clk);
	if (ret) {
		LOG_ERR("Failed to enable ch0 clock: %d", ret);
		goto err_ahb_off;
	}

	/* Deassert reset */
	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("Failed to deassert reset: %d", ret);
		goto err_ch0_off;
	}

	/* Read panel timings from DTS */
	tcon_read_timings_from_dt(dev);

	/* Configure PWM backlight (optional) */
	if (cfg->backlight_pwm_dev != NULL) {
		if (device_is_ready(cfg->backlight_pwm_dev)) {
			/* Start with backlight at full brightness (255).
			 * The PWM will actually be enabled when
			 * tcon_enable_output(true) is called by the DE2 mixer.
			 */
			data->brightness = 255;
			LOG_DBG("PWM backlight: ch=%u freq=%u Hz",
				cfg->backlight_pwm_channel,
				cfg->backlight_pwm_freq);
		} else {
			LOG_WRN("PWM backlight device not ready");
		}
	} else if (cfg->backlight_gpio.port != NULL) {
		/* Fallback: configure GPIO backlight (e.g. PB4 = LED_EN) */
		if (gpio_is_ready_dt(&cfg->backlight_gpio)) {
			ret = gpio_pin_configure_dt(&cfg->backlight_gpio, GPIO_OUTPUT);
			if (ret < 0) {
				LOG_WRN("Failed to configure backlight GPIO: %d", ret);
			} else {
				gpio_pin_set_dt(&cfg->backlight_gpio, 0);
				LOG_DBG("Backlight GPIO configured (active-high)");
			}
		} else {
			LOG_WRN("Backlight GPIO device not ready");
		}
	}

	LOG_DBG("TCON V3s initialized (base=0x%08lx)", (unsigned long)cfg->base);
	return 0;

err_ch0_off:
	clock_control_off(cfg->clock_dev, cfg->ch0_clk);
err_ahb_off:
	clock_control_off(cfg->clock_dev, cfg->ahb_clk);
	return ret;
}

#define TCON_V3S_INIT(inst)                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                          \
                                                                                \
	static struct tcon_v3s_data tcon_v3s_data_##inst;                      \
                                                                                \
	static struct tcon_v3s_config tcon_v3s_config_##inst = {               \
		.base      = DT_INST_REG_ADDR(inst),                            \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),          \
		.ahb_clk   = (clock_control_subsys_t)(uintptr_t)                \
			     DT_INST_CLOCKS_CELL_BY_NAME(inst, ahb, clk_id),    \
		.ch0_clk   = (clock_control_subsys_t)(uintptr_t)                \
			     DT_INST_CLOCKS_CELL_BY_NAME(inst, tcon_ch0, clk_id), \
		.reset     = RESET_DT_SPEC_INST_GET(inst),                      \
		.pcfg      = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),              \
		.backlight_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
		.backlight_pwm_dev = \
			COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, backlight), \
				(DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, backlight, 0))), \
				(NULL)), \
		.backlight_pwm_channel = \
			COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, backlight), \
				(DT_INST_PHA_BY_IDX(inst, backlight, 0, channel)), \
				(0)), \
		.backlight_pwm_freq = \
			COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, backlight), \
				(DT_INST_PHA_BY_IDX(inst, backlight, 0, period)), \
				(0)), \
	};                                                                      \
                                                                                \
	DEVICE_DT_INST_DEFINE(inst,                                             \
			      tcon_v3s_init,                                    \
			      NULL,                                             \
			      &tcon_v3s_data_##inst,                            \
			      &tcon_v3s_config_##inst,                          \
			      POST_KERNEL,                                      \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,               \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TCON_V3S_INIT)
