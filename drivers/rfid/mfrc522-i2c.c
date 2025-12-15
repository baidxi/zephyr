/*
 * Copyright (c) 2025, juno
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT nxp_mfrc522_i2c

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "mfrc522.h"

LOG_MODULE_DECLARE(mfrc522, CONFIG_RFID_LOG_LEVEL);

static int mfrc522_i2c_read(const struct device *dev, uint8_t reg, uint8_t *val, size_t len)
{
	const struct mfrc522_config *config = dev->config;

	return i2c_write_read_dt(&config->bus_cfg.i2c, &reg, 1, val, len);
}

static int mfrc522_i2c_write(const struct device *dev, uint8_t reg, uint8_t *val, size_t len)
{
	const struct mfrc522_config *config = dev->config;
	uint8_t buf[MFRC522_FIFO_SIZE + 1];

	if (len > MFRC522_FIFO_SIZE) {
		return -EINVAL;
	}

	buf[0] = reg;
	memcpy(&buf[1], val, len);

	return i2c_write_dt(&config->bus_cfg.i2c, buf, len + 1);
}

static const struct mfrc522_bus_io mfrc522_i2c_bus_io = {
	.read = mfrc522_i2c_read,
	.write = mfrc522_i2c_write,
};

#define MFRC522_INIT_I2C(n)						\
	static struct mfrc522_data mfrc522_data_##n;			\
	static const struct mfrc522_config mfrc522_config_##n = {	\
		.bus_io = &mfrc522_i2c_bus_io,				\
		.bus_cfg = {						\
			.i2c = I2C_DT_SPEC_INST_GET(n)			\
		},							\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),\
	};								\
	DEVICE_DT_INST_DEFINE(n, mfrc522_init, NULL,			\
			    &mfrc522_data_##n, &mfrc522_config_##n,	\
			    POST_KERNEL, CONFIG_RFID_INIT_PRIORITY,	\
			    &mfrc522_api);

DT_INST_FOREACH_STATUS_OKAY(MFRC522_INIT_I2C)