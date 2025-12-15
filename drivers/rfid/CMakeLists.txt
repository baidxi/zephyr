# Copyright (c) 2025, juno
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_RFID_MFRC522 mfrc522-core.c)
zephyr_library_sources_ifdef(CONFIG_RFID_MFRC522_SPI mfrc522-spi.c)
zephyr_library_sources_ifdef(CONFIG_RFID_MFRC522_I2C mfrc522-i2c.c)