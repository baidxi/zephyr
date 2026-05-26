/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_SUN8I_V3S_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_SUN8I_V3S_H_

#include <zephyr/kernel.h>

struct sun8i_clock_data {
  uint8_t type;
  uint32_t freq;
};

/**
 * @brief SPI module clock source selection.
 *
 * The V3s SPI0_CLK register supports three clock sources.
 */
enum sun8i_spi_clk_src {
	SUN8I_SPI_CLK_SRC_HOSC         = 0, /* OSC24M (24 MHz) */
	SUN8I_SPI_CLK_SRC_PLL_PERIPH0  = 1, /* PLL_PERIPH0 (600 MHz) */
	SUN8I_SPI_CLK_SRC_PLL_PERIPH1  = 2, /* PLL_PERIPH1 (600 MHz) */
};

/**
 * @brief Configuration structure for SPI module clock (CLK_SPI0).
 *
 * Passed as @p data argument to clock_control_set_rate() for CLK_SPI0.
 * The CCU driver calculates the M/N dividers to achieve @p output_freq
 * from the selected @p src parent clock.
 */
struct sun8i_spi_clk_config {
	enum sun8i_spi_clk_src src; /* Parent clock source */
	uint32_t output_freq;       /* Desired SPI module clock output (Hz) */
};

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_SUN8I_V3S_H_ */
