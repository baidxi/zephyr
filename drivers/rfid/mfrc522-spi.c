/*
 * Copyright (c) 2025, juno
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mfrc522_spi

#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "mfrc522.h"

LOG_MODULE_DECLARE(mfrc522, CONFIG_RFID_LOG_LEVEL);

static int mfrc522_spi_read(const struct device *dev, uint8_t reg, uint8_t *val, size_t len)
{
	const struct mfrc522_config *config = dev->config;
	uint8_t addr = reg | 0x80;
	struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
	struct spi_buf rx_buf = { .buf = val, .len = len };
	struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

	return spi_transceive_dt(&config->bus_cfg.spi, &tx_set, &rx_set);
}

static int mfrc522_spi_write(const struct device *dev, uint8_t reg, uint8_t *val, size_t len)
{
	const struct mfrc522_config *config = dev->config;
	uint8_t addr = reg & ~0x80;
	struct spi_buf tx_bufs[] = {
		{ .buf = &addr, .len = 1 },
		{ .buf = val, .len = len }
	};
	struct spi_buf_set tx_set = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };

	return spi_write_dt(&config->bus_cfg.spi, &tx_set);
}

static const struct mfrc522_bus_io mfrc522_spi_bus_io = {
	.read = mfrc522_spi_read,
	.write = mfrc522_spi_write,
};

#define MFRC522_INIT_SPI(n)						\
	static struct mfrc522_data mfrc522_data_##n;			\
	static const struct mfrc522_config mfrc522_config_##n = {	\
		.bus_io = &mfrc522_spi_bus_io,				\
		.bus_cfg = {						\
			.spi = SPI_DT_SPEC_INST_GET(n,		\
				SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0)	\
		},							\
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),\
	};								\
	DEVICE_DT_INST_DEFINE(n, mfrc522_init, NULL,			\
			    &mfrc522_data_##n, &mfrc522_config_##n,	\
			    POST_KERNEL, CONFIG_RFID_INIT_PRIORITY,	\
			    &mfrc522_api);

DT_INST_FOREACH_STATUS_OKAY(MFRC522_INIT_SPI)