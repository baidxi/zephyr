/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner Sun8i V3s Entropy driver.
 *
 * Uses the 160-bit hardware PRNG inside the Crypto Engine (Security System)
 * to provide entropy for the Zephyr random subsystem.
 *
 * The PRNG is a sub-function of the Crypto Engine at 0x01C15000.
 * This driver is instantiated as a child node of the crypto device.
 *
 * Based on the Linux kernel sun4i-ss-prng.c driver:
 * uses continue mode and reads random data directly from TXFIFO
 * without checking FIFO status (PRNG generates data on-the-fly).
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_rng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control_sun8i_v3s.h>
#include <zephyr/dt-bindings/clock/allwinner,sun8i-v3s-ccu.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "../crypto/crypto_sun8i_v3s_priv.h"

LOG_MODULE_REGISTER(entropy_sun8i_v3s, CONFIG_ENTROPY_LOG_LEVEL);

/* Parent crypto node provides the register base */
#define CRYPTO_NODE  DT_PARENT(DT_DRV_INST(0))
#define CE_BASE      DT_REG_ADDR(CRYPTO_NODE)

/* PRNG constants matching Linux sun4i-ss */
#define SS_SEED_LEN_WORDS	6   /* 175 bits / 32 */
#define SS_DATA_LEN_BYTES	20  /* 160 bits / 8 */

struct entropy_sun8i_v3s_data {
	struct k_mutex lock;
	uint32_t seed[SS_SEED_LEN_WORDS];
	bool seeded;
};

static struct entropy_sun8i_v3s_data entropy_sun8i_v3s_dev_data;

/**
 * @brief Internal PRNG read function (no locking).
 *
 * Follows the exact sequence of Linux sun4i-ss-prng.c:
 * 1. Enable CE in PRNG continue mode
 * 2. For each 20-byte block:
 *    a. Write seed to KEY registers
 *    b. Read random words directly from TXFIFO
 *    c. Update seed from KEY registers
 * 3. Disable CE
 *
 * In continue mode the PRNG generates data on-the-fly,
 * no FIFO status polling is needed (per Linux driver).
 */
static int prng_read(struct entropy_sun8i_v3s_data *data,
		     uint8_t *buffer, uint16_t length)
{
	uintptr_t base = CE_BASE;
	uint32_t mode = SS_OP_PRNG | SS_PRNG_MODE | SS_ENABLED;
	unsigned int offset = 0;
	int i;

	__ASSERT(data->seeded, "PRNG seed not initialized");

	/* Enable PRNG in continue mode */
	sys_write32(mode, base + SS_CTL);

	while (offset < length) {
		unsigned int chunk = MIN(SS_DATA_LEN_BYTES, length - offset);
		/* Round up to number of 32-bit words to read from TXFIFO */
		unsigned int words = (chunk + 3U) / 4U;
		uint32_t tmp[5]; /* max 5 words = 160 bits */

		/* Write seed to KEY registers */
		for (i = 0; i < SS_SEED_LEN_WORDS; i++) {
			sys_write32(data->seed[i], base + SS_KEY0 + i * 4);
		}

		/* Read random data directly from TXFIFO */
		for (i = 0; i < (int)words; i++) {
			tmp[i] = sys_read32(base + SS_TXFIFO);
		}

		/* Copy exactly 'chunk' bytes to output buffer */
		memcpy(buffer + offset, tmp, chunk);
		offset += chunk;

		/* Update seed by reading back from KEY registers */
		for (i = 0; i < SS_SEED_LEN_WORDS; i++) {
			data->seed[i] = sys_read32(base + SS_KEY0 + i * 4);
		}
	}

	/* Disable CE */
	sys_write32(0, base + SS_CTL);
	return 0;
}

/**
 * @brief Generate random data using PRNG continue mode + TXFIFO.
 *
 * Thread-safe via mutex. Safe to call from any thread context.
 */
static int entropy_sun8i_v3s_get_entropy(const struct device *dev,
					 uint8_t *buffer,
					 uint16_t length)
{
	struct entropy_sun8i_v3s_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = prng_read(data, buffer, length);
	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Generate random data from ISR context.
 *
 * Uses non-blocking mutex. If mutex is contended, proceeds without
 * locking (acceptable for PRNG).
 */
static int entropy_sun8i_v3s_get_entropy_isr(const struct device *dev,
					     uint8_t *buf,
					     uint16_t len,
					     uint32_t flags)
{
	struct entropy_sun8i_v3s_data *data = dev->data;
	bool locked = false;
	int ret;

	ARG_UNUSED(flags);

	if (!data->seeded) {
		return -EAGAIN;
	}

	/* Try non-blocking mutex lock */
	if (k_mutex_lock(&data->lock, K_NO_WAIT) == 0) {
		locked = true;
	}

	ret = prng_read(data, buf, len);

	if (locked) {
		k_mutex_unlock(&data->lock);
	}

	return ret;
}

static int entropy_sun8i_v3s_init(const struct device *dev)
{
	int ret;
	struct entropy_sun8i_v3s_data *data = dev->data;

	/* Enable AHB bus clock for CE */
	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(ccu)),
			       (clock_control_subsys_t)(uintptr_t)CLK_BUS_CE);
	if (ret) {
		LOG_ERR("Failed to enable CLK_BUS_CE: %d", ret);
		return ret;
	}

	/* Enable module clock for CE */
	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(ccu)),
			       (clock_control_subsys_t)(uintptr_t)CLK_CE);
	if (ret) {
		LOG_ERR("Failed to enable CLK_CE: %d", ret);
		return ret;
	}

	/* Deassert reset */
	const struct reset_dt_spec reset_spec =
		RESET_DT_SPEC_GET(CRYPTO_NODE);

	ret = reset_line_deassert_dt(&reset_spec);
	if (ret) {
		LOG_ERR("Failed to deassert reset: %d", ret);
		return ret;
	}

	/* Initialize mutex */
	k_mutex_init(&data->lock);

	/*
	 * Initialize PRNG seed from multiple entropy sources:
	 * - ARM Generic Timer physical counter (hardware entropy)
	 * - Device data address (varies with build/linker layout)
	 * - Compile-time golden ratio constants
	 */
	uint32_t now = k_cycle_get_32();
	uintptr_t addr = (uintptr_t)data;

	for (int i = 0; i < SS_SEED_LEN_WORDS; i++) {
		data->seed[i] = now ^ (now << (i + 1)) ^
				 (now >> (i + 3)) ^ (i * 0x9E3779B9) ^
				 (addr >> (i * 4));
	}
	data->seeded = true;

	LOG_INF("Sun8i V3s Entropy (PRNG) initialized");

	return 0;
}

static DEVICE_API(entropy, entropy_sun8i_v3s_api) = {
	.get_entropy = entropy_sun8i_v3s_get_entropy,
	.get_entropy_isr = entropy_sun8i_v3s_get_entropy_isr,
};

/*
 * Initialize at priority 56, which is after the parent crypto driver (55)
 * but before the Ethernet driver (60), so that sys_rand_get() is available
 * during network initialization.
 */
DEVICE_DT_INST_DEFINE(0, entropy_sun8i_v3s_init, NULL,
		      &entropy_sun8i_v3s_dev_data, NULL,
		      POST_KERNEL, CONFIG_ENTROPY_INIT_PRIORITY,
		      &entropy_sun8i_v3s_api);
