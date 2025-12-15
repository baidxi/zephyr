/*
 * Copyright (c) 2024, Your Name
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rfid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mfrc522_sample, LOG_LEVEL_DBG);

const struct device *const rfid_dev = DEVICE_DT_GET(DT_NODELABEL(mfrc522));

static void card_detected_handler(const struct device *dev, enum rfid_trigger trigger)
{
	uint8_t uid[10];
	size_t uid_len = sizeof(uid);
	uint8_t sak;

	if (rfid_select(dev, uid, &uid_len, &sak) == 0) {
		LOG_HEXDUMP_INF(uid, uid_len, "UID found:");
	}
}

int main(void)
{
	if (!device_is_ready(rfid_dev)) {
		LOG_ERR("RFID device not ready");
		return 0;
	}

	LOG_INF("RFID device ready");

	rfid_trigger_set(rfid_dev, RFID_TRIGGER_CARD_DETECTED, card_detected_handler);

	while (1) {
		k_sleep(K_MSEC(1000));
	}
	return 0;
}