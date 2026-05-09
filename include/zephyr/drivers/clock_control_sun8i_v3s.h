/*
 * Copyright (c) 2025
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

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_SUN8I_V3S_H_ */
