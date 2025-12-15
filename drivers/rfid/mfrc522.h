/*
 * Copyright (c) 2025, juno
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_RFID_MFRC522_H_
#define ZEPHYR_DRIVERS_RFID_MFRC522_H_

#include <zephyr/drivers/rfid.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* MFRC522 registers */
#define MFRC522_REG_COMMAND		0x01
#define MFRC522_REG_COMIEN		0x02
#define MFRC522_REG_DIVIEN		0x03
#define MFRC522_REG_COMIRQ		0x04
#define MFRC522_REG_DIVIRQ		0x05
#define MFRC522_REG_ERROR		0x06
#define MFRC522_REG_STATUS1		0x07
#define MFRC522_REG_STATUS2		0x08
#define MFRC522_REG_FIFODATA		0x09
#define MFRC522_REG_FIFOLEVEL		0x0A
#define MFRC522_REG_WATERLEVEL		0x0B
#define MFRC522_REG_CONTROL		0x0C
#define MFRC522_REG_BITFRAMING		0x0D
#define MFRC522_REG_COLL		0x0E
#define MFRC522_REG_MODE		0x11
#define MFRC522_REG_TXMODE		0x12
#define MFRC522_REG_RXMODE		0x13
#define MFRC522_REG_TXCONTROL		0x14
#define MFRC522_REG_TXASK		0x15
#define MFRC522_REG_TXSEL		0x16
#define MFRC522_REG_RXSEL		0x17
#define MFRC522_REG_RXTHRESHOLD		0x18
#define MFRC522_REG_DEMOD		0x19
#define MFRC522_REG_MFTX		0x1C
#define MFRC522_REG_MFRX		0x1D
#define MFRC522_REG_SERIALSPEED		0x1F
#define MFRC522_REG_CRCRESULT_MSB	0x21
#define MFRC522_REG_CRCRESULT_LSB	0x22
#define MFRC522_REG_MODWIDTH		0x24
#define MFRC522_REG_RFCFG		0x26
#define MFRC522_REG_GSN			0x27
#define MFRC522_REG_CWGSP		0x28
int mfrc522_init(const struct device *dev);
#define MFRC522_REG_MODGSP		0x29
#define MFRC522_REG_TMODE		0x2A
#define MFRC522_REG_TPRESCALER		0x2B
#define MFRC522_REG_TRELOAD_MSB		0x2C
#define MFRC522_REG_TRELOAD_LSB		0x2D
#define MFRC522_REG_TCOUNTERVAL_MSB	0x2E
#define MFRC522_REG_TCOUNTERVAL_LSB	0x2F
#define MFRC522_REG_TESTSEL1		0x31
#define MFRC522_REG_TESTSEL2		0x32
#define MFRC522_REG_TESTPINEN		0x33
#define MFRC522_REG_TESTPINVALUE	0x34
#define MFRC522_REG_TESTBUS		0x35
#define MFRC522_REG_AUTOTEST		0x36
#define MFRC522_REG_VERSION		0x37
#define MFRC522_REG_ANALOGTEST		0x38
#define MFRC522_REG_TESTDAC1		0x39
#define MFRC522_REG_TESTDAC2		0x3A
#define MFRC522_REG_TESTADC		0x3B

/* MFRC522 commands */
#define MFRC522_CMD_IDLE		0x00
#define MFRC522_CMD_MEM			0x01
#define MFRC522_CMD_GENERATE_RANDOM_ID	0x02
#define MFRC522_CMD_CALC_CRC		0x03
#define MFRC522_CMD_TRANSMIT		0x04
#define MFRC522_CMD_NO_CMD_CHANGE	0x07
#define MFRC522_CMD_RECEIVE		0x08
#define MFRC522_CMD_TRANSCEIVE		0x0C
#define MFRC522_CMD_MF_AUTHENT		0x0E
#define MFRC522_CMD_SOFT_RESET		0x0F

/* MFRC522 PICC commands */
#define PICC_CMD_REQA			0x26
#define PICC_CMD_WUPA			0x52
#define PICC_CMD_CT			0x88
#define PICC_CMD_SEL_CL1		0x93
#define PICC_CMD_SEL_CL2		0x95
#define PICC_CMD_SEL_CL3		0x97
#define PICC_CMD_HLTA			0x50
#define PICC_CMD_RATS           0xE0

/* MFRC522 Mifare Classic commands */
#define PICC_CMD_MF_AUTH_KEY_A		0x60
#define PICC_CMD_MF_AUTH_KEY_B		0x61
#define PICC_CMD_MF_READ		0x30
#define PICC_CMD_MF_WRITE		0xA0
#define PICC_CMD_MF_DECREMENT		0xC0
#define PICC_CMD_MF_INCREMENT		0xC1
#define PICC_CMD_MF_RESTORE		0xC2
#define PICC_CMD_MF_TRANSFER		0xB0

/* MFRC522 masks */
#define MFRC522_FIFO_SIZE		64
#define MFRC522_IRQ_TIMER		BIT(0)
#define MFRC522_IRQ_ERROR		BIT(1)
#define MFRC522_IRQ_LOALERT		BIT(2)
#define MFRC522_IRQ_HIALERT		BIT(3)
#define MFRC522_IRQ_IDLE		BIT(4)
#define MFRC522_IRQ_RX			BIT(5)
#define MFRC522_IRQ_TX			BIT(6)
#define MFRC522_IRQ_SET1		BIT(7)

#define MFRC522_IRQ_CRC			BIT(2)
#define MFRC522_IRQ_MFINACT		BIT(4)
#define MFRC522_IRQ_SET2		BIT(7)

typedef int (*mfrc522_bus_io_fn)(const struct device *dev, uint8_t reg, uint8_t *val, size_t len);

struct mfrc522_bus_io {
	mfrc522_bus_io_fn read;
	mfrc522_bus_io_fn write;
};

struct mfrc522_config {
	const struct mfrc522_bus_io *bus_io;
	union {
		struct spi_dt_spec spi;
		struct i2c_dt_spec i2c;
	} bus_cfg;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec irq_gpio;
};

struct mfrc522_data {
	const struct device *dev;
	rfid_trigger_handler_t handler;
	struct k_work work;
	struct gpio_callback irq_cb;
};

extern const struct rfid_driver_api mfrc522_api;

int mfrc522_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_RFID_MFRC522_H_ */