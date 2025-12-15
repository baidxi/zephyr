/*
 * Copyright (c) 2025, juno
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rfid.h>
#include <zephyr/logging/log.h>

#include "mfrc522.h"

LOG_MODULE_REGISTER(mfrc522, CONFIG_RFID_LOG_LEVEL);

/* Static function prototypes */
static int mfrc522_read(const struct device *dev, uint8_t reg, uint8_t *val, size_t len);
static int mfrc522_write(const struct device *dev, uint8_t reg, const uint8_t *val, size_t len);
static int mfrc522_read_reg(const struct device *dev, uint8_t reg, uint8_t *val);
static int mfrc522_write_reg(const struct device *dev, uint8_t reg, uint8_t val);
static int mfrc522_set_bit_mask(const struct device *dev, uint8_t reg, uint8_t mask);
static int mfrc522_clear_bit_mask(const struct device *dev, uint8_t reg, uint8_t mask);
static int mfrc522_transceive(const struct device *dev, const uint8_t *tx_buf, size_t tx_len,
				uint8_t *rx_buf, size_t *rx_len);
static int mfrc522_wait_for_reg_set(const struct device *dev, uint8_t reg, uint8_t mask,
				    int timeout_ms);


#if IS_ENABLED(CONFIG_RFID_MFRC522_TRIGGER)
static void mfrc522_irq_work_handler(struct k_work *work)
{
	struct mfrc522_data *data = CONTAINER_OF(work, struct mfrc522_data, work);
	const struct device *dev = data->dev;
	uint8_t irq_val;

	mfrc522_read_reg(dev, MFRC522_REG_COMIRQ, &irq_val);
	if (irq_val & MFRC522_IRQ_IDLE) {
		if (data->handler) {
			data->handler(dev, RFID_TRIGGER_CARD_DETECTED);
		}
	}
	mfrc522_write_reg(dev, MFRC522_REG_COMIRQ, MFRC522_IRQ_SET1);
}

static void mfrc522_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct mfrc522_data *data = CONTAINER_OF(cb, struct mfrc522_data, irq_cb);

	k_work_submit(&data->work);
}
#endif /* IS_ENABLED(CONFIG_RFID_MFRC522_TRIGGER) */


static int mfrc522_read(const struct device *dev, uint8_t reg, uint8_t *val, size_t len)
{
	const struct mfrc522_config *config = dev->config;

	return config->bus_io->read(dev, reg, val, len);
}

static int mfrc522_write(const struct device *dev, uint8_t reg, const uint8_t *val, size_t len)
{
	const struct mfrc522_config *config = dev->config;

	return config->bus_io->write(dev, reg, (uint8_t *)val, len);
}

static int mfrc522_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	return mfrc522_read(dev, reg, val, 1);
}

static int mfrc522_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	return mfrc522_write(dev, reg, &val, 1);
}


static int mfrc522_set_bit_mask(const struct device *dev, uint8_t reg, uint8_t mask)
{
	uint8_t tmp;
	int ret;

	ret = mfrc522_read_reg(dev, reg, &tmp);
	if (ret < 0) {
		return ret;
	}

	return mfrc522_write_reg(dev, reg, tmp | mask);
}

static int mfrc522_clear_bit_mask(const struct device *dev, uint8_t reg, uint8_t mask)
{
	uint8_t tmp;
	int ret;

	ret = mfrc522_read_reg(dev, reg, &tmp);
	if (ret < 0) {
		return ret;
	}

	return mfrc522_write_reg(dev, reg, tmp & ~mask);
}

static int mfrc522_is_present(const struct device *dev, uint8_t *atqa)
{
	int ret;
	uint8_t cmd = PICC_CMD_REQA;
	size_t rx_len = 2;

	ret = mfrc522_write_reg(dev, MFRC522_REG_BITFRAMING, 0x07);
	if (ret < 0) {
		return ret;
	}

	return mfrc522_transceive(dev, &cmd, 1, atqa, &rx_len);
}

static int mfrc522_select(const struct device *dev, uint8_t *uid, size_t *uid_len, uint8_t *sak)
{
	int ret;
	uint8_t cmd[] = { PICC_CMD_SEL_CL1, 0x20 };
	uint8_t rx_buf[5];
	size_t rx_len = sizeof(rx_buf);
	uint8_t bcc = 0;

	ret = mfrc522_transceive(dev, cmd, sizeof(cmd), rx_buf, &rx_len);
	if (ret < 0) {
		return ret;
	}

	if (rx_len != 5) {
		return -EIO;
	}

	for (int i = 0; i < 4; i++) {
		uid[i] = rx_buf[i];
		bcc ^= uid[i];
	}

	if (bcc != rx_buf[4]) {
		return -EIO;
	}

	*uid_len = 4;
	*sak = rx_buf[0];

	return 0;
}

static int mfrc522_halt(const struct device *dev)
{
	int ret;
	uint8_t cmd[] = { PICC_CMD_HLTA, 0 };
	uint16_t crc;

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_CMD_CALC_CRC);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write(dev, MFRC522_REG_FIFODATA, cmd, sizeof(cmd));
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_wait_for_reg_set(dev, MFRC522_REG_DIVIRQ, MFRC522_IRQ_CRC, 100);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_CRCRESULT_LSB, (uint8_t *)&crc);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_CRCRESULT_MSB, ((uint8_t *)&crc) + 1);
	if (ret < 0) {
		return ret;
	}

	cmd[1] = crc & 0xFF;
	cmd[0] = (crc >> 8) & 0xFF;

	return mfrc522_transceive(dev, cmd, sizeof(cmd), NULL, 0);
}

static int mfrc522_rats(const struct device *dev, uint8_t *ats, size_t *ats_len)
{
	int ret;
	uint8_t cmd[] = { PICC_CMD_RATS, 0x50 };
	uint16_t crc;

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_CMD_CALC_CRC);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write(dev, MFRC522_REG_FIFODATA, cmd, sizeof(cmd));
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_wait_for_reg_set(dev, MFRC522_REG_DIVIRQ, MFRC522_IRQ_CRC, 100);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_CRCRESULT_LSB, (uint8_t *)&crc);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_CRCRESULT_MSB, ((uint8_t *)&crc) + 1);
	if (ret < 0) {
		return ret;
	}

	cmd[1] = crc & 0xFF;
	cmd[0] = (crc >> 8) & 0xFF;

	return mfrc522_transceive(dev, cmd, sizeof(cmd), ats, ats_len);
}

static int mfrc522_pps(const struct device *dev, uint8_t dsi, uint8_t dri)
{
	int ret;
	uint8_t cmd[] = { 0xD0 | (dsi & 0x0F) | ((dri & 0x0F) << 4), 0x00 };
	uint16_t crc;

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_CMD_CALC_CRC);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write(dev, MFRC522_REG_FIFODATA, cmd, sizeof(cmd));
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_wait_for_reg_set(dev, MFRC522_REG_DIVIRQ, MFRC522_IRQ_CRC, 100);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_CRCRESULT_LSB, (uint8_t *)&crc);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_CRCRESULT_MSB, ((uint8_t *)&crc) + 1);
	if (ret < 0) {
		return ret;
	}

	cmd[1] = crc & 0xFF;
	cmd[0] = (crc >> 8) & 0xFF;

	return mfrc522_transceive(dev, cmd, sizeof(cmd), NULL, 0);
}

static int mfrc522_wait_for_reg_set(const struct device *dev, uint8_t reg, uint8_t mask, int timeout_ms)
{
	uint8_t val;
	int ret;

	while (timeout_ms > 0) {
		ret = mfrc522_read_reg(dev, reg, &val);
		if (ret < 0) {
			return ret;
		}
		if (val & mask) {
			return 0;
		}
		k_msleep(1);
		timeout_ms--;
	}

	return -ETIMEDOUT;
}

static int mfrc522_transceive(const struct device *dev, const uint8_t *tx_buf, size_t tx_len,
				uint8_t *rx_buf, size_t *rx_len)
{
	int ret;
	uint8_t irq_val;
	size_t rx_read_len = 0;

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_CMD_IDLE);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMIRQ, MFRC522_IRQ_SET1);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_FIFOLEVEL, 0x80);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write(dev, MFRC522_REG_FIFODATA, (uint8_t *)tx_buf, tx_len);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_CMD_TRANSCEIVE);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_set_bit_mask(dev, MFRC522_REG_BITFRAMING, 0x80);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_wait_for_reg_set(dev, MFRC522_REG_COMIRQ, MFRC522_IRQ_RX | MFRC522_IRQ_IDLE, 100);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_ERROR, &irq_val);
	if (ret < 0) {
		return ret;
	}

	if (irq_val & 0x1B) {
		return -EIO;
	}

	ret = mfrc522_read_reg(dev, MFRC522_REG_FIFOLEVEL, &irq_val);
	if (ret < 0) {
		return ret;
	}

	rx_read_len = irq_val;
	if (rx_read_len > *rx_len) {
		return -ENOMEM;
	}

	*rx_len = rx_read_len;

	return mfrc522_read(dev, MFRC522_REG_FIFODATA, rx_buf, *rx_len);
}

static int mfrc522_wtx(const struct device *dev, uint8_t wtxm)
{
	uint8_t cmd[] = { 0xF2, wtxm };
	size_t rx_len = 0;

	return mfrc522_transceive(dev, cmd, sizeof(cmd), NULL, &rx_len);
}

static int mfrc522_deselect(const struct device *dev)
{
	uint8_t cmd[] = { 0xC2 };
	size_t rx_len = 0;

	return mfrc522_transceive(dev, cmd, sizeof(cmd), NULL, &rx_len);
}

#if IS_ENABLED(CONFIG_RFID_MFRC522_TRIGGER)
static int mfrc522_trigger_set(const struct device *dev, enum rfid_trigger trig,
				    rfid_trigger_handler_t handler)
{
	struct mfrc522_data *data = dev->data;
	const struct mfrc522_config *config = dev->config;

	if (!config->irq_gpio.port) {
		return -ENOTSUP;
	}

	if (trig != RFID_TRIGGER_CARD_DETECTED) {
		return -ENOTSUP;
	}

	data->handler = handler;

	if (handler) {
		mfrc522_write_reg(dev, MFRC522_REG_COMIEN, MFRC522_IRQ_IDLE | MFRC522_IRQ_SET1);
	} else {
		mfrc522_write_reg(dev, MFRC522_REG_COMIEN, MFRC522_IRQ_SET1);
	}

	return 0;
}
#else
static int mfrc522_trigger_set(const struct device *dev, enum rfid_trigger trig,
				    rfid_trigger_handler_t handler)
{
	return -ENOSYS;
}
#endif /* IS_ENABLED(CONFIG_RFID_MFRC522_TRIGGER) */

const struct rfid_driver_api mfrc522_api = {
	.is_present = mfrc522_is_present,
	.select = mfrc522_select,
	.halt = mfrc522_halt,
	.rats = mfrc522_rats,
	.pps = mfrc522_pps,
	.transceive = mfrc522_transceive,
	.wtx = mfrc522_wtx,
	.deselect = mfrc522_deselect,
	.trigger_set = mfrc522_trigger_set,
};

int mfrc522_init(const struct device *dev)
{
	const struct mfrc522_config *config = dev->config;
	struct mfrc522_data *data = dev->data;
	int ret;

	data->dev = dev;

	if (config->reset_gpio.port && !gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("Reset GPIO is not ready");
		return -ENODEV;
	}

	if (config->reset_gpio.port) {
		gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		k_sleep(K_MSEC(1));
		gpio_pin_set_dt(&config->reset_gpio, 0);
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_CMD_SOFT_RESET);
	if (ret < 0) {
		return ret;
	}

	k_msleep(50);

	ret = mfrc522_write_reg(dev, MFRC522_REG_TMODE, 0x8D);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_TPRESCALER, 0x3E);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_TRELOAD_LSB, 30);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_TRELOAD_MSB, 0);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_TXASK, 0x40);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_write_reg(dev, MFRC522_REG_MODE, 0x3D);
	if (ret < 0) {
		return ret;
	}

	ret = mfrc522_set_bit_mask(dev, MFRC522_REG_TXCONTROL, 0x03);
	if (ret < 0) {
		return ret;
	}

#if IS_ENABLED(CONFIG_RFID_MFRC522_TRIGGER)
	k_work_init(&data->work, mfrc522_irq_work_handler);

	if (config->irq_gpio.port) {
		if (!gpio_is_ready_dt(&config->irq_gpio)) {
			LOG_ERR("IRQ GPIO is not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT | GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure IRQ GPIO");
			return ret;
		}

		gpio_init_callback(&data->irq_cb, mfrc522_irq_handler, BIT(config->irq_gpio.pin));
		ret = gpio_add_callback(config->irq_gpio.port, &data->irq_cb);
		if (ret < 0) {
			LOG_ERR("Failed to add IRQ callback");
			return ret;
		}
	}
#endif /* IS_ENABLED(CONFIG_RFID_MFRC522_TRIGGER) */

	return 0;
}
