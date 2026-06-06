/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal API for Allwinner V3s TCON (Timing Controller).
 *
 * The TCON generates HSYNC/VSYNC/DE/DCLK signals for RGB LCD panels.
 * It is referenced by the DE2 Mixer driver via the "tcon" phandle.
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_TCON_SUN8I_V3S_H_
#define ZEPHYR_DRIVERS_DISPLAY_TCON_SUN8I_V3S_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Panel timing parameters.
 *
 * Read by TCON during init from the display-timings child node.
 * Mixer driver obtains these via tcon_get_timings().
 */
struct tcon_timings {
	uint16_t width;
	uint16_t height;
	uint16_t hsync_len;
	uint16_t vsync_len;
	uint16_t hback_porch;
	uint16_t vback_porch;
	uint16_t hfront_porch;
	uint16_t vfront_porch;
	uint32_t pixel_clock_hz;
	bool hsync_active_low;
	bool vsync_active_low;
	bool de_active_high;
	bool pixelclk_active_low;
};

/**
 * @brief Get timings read from DTS display-timings child node.
 *
 * @param tcon TCON device pointer.
 * @return Pointer to read-only timings, valid for TCON's lifetime.
 */
const struct tcon_timings *tcon_get_timings(const struct device *tcon);

/**
 * @brief Write timing parameters to TCON hardware registers.
 *
 * Calculates and writes BASIC0~3, IO_POL, GCTL, and DCLK registers.
 *
 * @param tcon TCON device pointer.
 * @param timings Timing parameters.
 * @return 0 on success, negative errno on failure.
 */
int tcon_apply_timings(const struct device *tcon,
		       const struct tcon_timings *timings);

/**
 * @brief Enable or disable TCON output signals.
 *
 * Controls HSYNC/VSYNC/DCLK/DE output to the LCD panel.
 *
 * @param tcon TCON device pointer.
 * @param enable true to enable output, false to disable.
 * @return 0 on success, negative errno on failure.
 */
int tcon_enable_output(const struct device *tcon, bool enable);

/**
 * @brief Set LCD backlight brightness via PWM.
 *
 * Controls the PWM duty cycle to adjust backlight brightness.
 * Only works if the "backlight" PWM property is defined in devicetree.
 *
 * @param tcon TCON device pointer.
 * @param brightness Brightness level (0-255, where 255 is maximum).
 * @return 0 on success, negative errno on failure or if PWM not configured.
 */
int tcon_set_brightness(const struct device *tcon, uint8_t brightness);

#endif /* ZEPHYR_DRIVERS_DISPLAY_TCON_SUN8I_V3S_H_ */
