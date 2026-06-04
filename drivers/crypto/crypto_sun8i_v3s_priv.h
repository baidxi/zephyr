/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner Sun8i V3s Crypto Engine (Security System) register definitions.
 *
 */

#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_SUN8I_V3S_PRIV_H_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_SUN8I_V3S_PRIV_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <stdbool.h>

/* Register offsets */
#define SS_CTL            0x00
#define SS_KEY0           0x04
#define SS_KEY1           0x08
#define SS_KEY2           0x0C
#define SS_KEY3           0x10
#define SS_KEY4           0x14
#define SS_KEY5           0x18
#define SS_KEY6           0x1C
#define SS_KEY7           0x20

#define SS_IV0            0x24
#define SS_IV1            0x28
#define SS_IV2            0x2C
#define SS_IV3            0x30

#define SS_FCSR           0x44

#define SS_MD0            0x4C
#define SS_MD1            0x50
#define SS_MD2            0x54
#define SS_MD3            0x58
#define SS_MD4            0x5C

#define SS_RXFIFO         0x200
#define SS_TXFIFO         0x204

/* SS_CTL configuration bits */

/* Operation mode - bits 12-13 */
#define SS_ECB            (0 << 12)
#define SS_CBC            (1 << 12)
#define SS_CTS            (3 << 12)

/* CE_METHOD field values - bits 4-6 */
#define SS_OP_AES         (0 << 4)
#define SS_OP_DES         (1 << 4)
#define SS_OP_3DES        (2 << 4)
#define SS_OP_SHA1        (3 << 4)
#define SS_OP_MD5         (4 << 4)
#define SS_OP_PRNG        (5 << 4)

/* Key size for AES - bits 8-9 */
#define SS_AES_128BITS    (0 << 8)
#define SS_AES_192BITS    (1 << 8)
#define SS_AES_256BITS    (2 << 8)

/* Operation direction - bit 7 */
#define SS_ENCRYPTION     (0 << 7)
#define SS_DECRYPTION     (1 << 7)

/* Data end - bit 2 */
#define SS_DATA_END       (1 << 2)

/* PRNG control bits */
#define SS_PRNG_START     (1 << 1)   /* Write 1 to start, auto-clears */
#define SS_PRNG_MODE      (1 << 15)  /* 0 = one-shot, 1 = continue */

/* Enable - bit 0 */
#define SS_ENABLED        (1 << 0)

/* SS_FCSR FIFO status bits */
#define SS_RXFIFO_FREE    (1 << 30)
#define SS_TXFIFO_AVAILABLE (1 << 22)

#define SS_RXFIFO_SPACES(val)  (((val) >> 24) & 0x3f)
#define SS_TXFIFO_SPACES(val)  (((val) >> 16) & 0x3f)

#define SS_RX_DEFAULT      32
#define SS_TX_MAX          33

#define SS_TIMEOUT         100

/* AES block size in bytes */
#define AES_BLOCK_SIZE     16

/* Maximum AES key size (256 bits / 32 bytes) */
#define AES_MAX_KEY_SIZE   32

/**
 * @brief Per-session state for the crypto engine.
 */
struct sun8i_v3s_crypto_session {
	bool in_use;
	uint8_t key[AES_MAX_KEY_SIZE];
	uint16_t keylen;
	uint32_t ctl_mode;        /* Pre-computed SS_CTL value (without ENABLE bit) */
};

/**
 * @brief Device runtime data.
 */
struct sun8i_v3s_crypto_data {
	struct k_mutex lock;
	struct sun8i_v3s_crypto_session sessions[CONFIG_CRYPTO_SUN8I_V3S_MAX_SESSION];
};

/**
 * @brief Device static configuration.
 */
struct sun8i_v3s_crypto_config {
	uintptr_t base;
};

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_SUN8I_V3S_PRIV_H_ */
