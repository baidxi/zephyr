/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner Sun8i V3s Crypto Engine (Security System) driver.
 *
 * Supported algorithms:
 *   - AES-128/192/256 in ECB mode
 *   - AES-128/192/256 in CBC mode
 *
 * The V3s crypto engine is register-compatible with the A33 variant.
 *
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_crypto

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control_sun8i_v3s.h>
#include <zephyr/dt-bindings/clock/allwinner,sun8i-v3s-ccu.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>

#include "crypto_sun8i_v3s_priv.h"

LOG_MODULE_REGISTER(crypto_sun8i_v3s, CONFIG_CRYPTO_LOG_LEVEL);

#define CRYP_SUPPORT (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)

static inline uintptr_t crypto_base(const struct device *dev)
{
	const struct sun8i_v3s_crypto_config *cfg = dev->config;

	return cfg->base;
}

static inline struct sun8i_v3s_crypto_session *crypto_session(struct cipher_ctx *ctx)
{
	return (struct sun8i_v3s_crypto_session *)ctx->drv_sessn_state;
}

static inline struct sun8i_v3s_crypto_data *crypto_data(const struct device *dev)
{
	return (struct sun8i_v3s_crypto_data *)dev->data;
}

static int crypto_wait_tx_fifo(uintptr_t base, uint32_t words_needed)
{
	uint32_t timeout = SS_TIMEOUT * 1000;
	uint32_t fcsr;

	while (timeout--) {
		fcsr = sys_read32(base + SS_FCSR);
		if (SS_TXFIFO_SPACES(fcsr) >= words_needed) {
			return 0;
		}
		k_busy_wait(1);
	}

	LOG_ERR("TX FIFO timeout: fcsr=0x%08x", fcsr);
	return -ETIMEDOUT;
}

static void crypto_write_key(uintptr_t base, const uint8_t *key, uint16_t keylen)
{
	uint32_t kwords = keylen / 4;
	uint32_t val;

	for (uint32_t i = 0; i < kwords; i++) {
		val = sys_get_le32(key + i * 4);
		sys_write32(val, base + SS_KEY0 + i * 4);
	}
}

static void crypto_write_iv(uintptr_t base, const uint8_t *iv)
{
	uint32_t val;

	for (int i = 0; i < 4; i++) {
		val = sys_get_le32(iv + i * 4);
		sys_write32(val, base + SS_IV0 + i * 4);
	}
}

static int crypto_do_op(uintptr_t base, uint32_t ctl, const uint8_t *in,
			uint32_t in_len, uint8_t *out)
{
	int ret;
	uint32_t fcsr;
	uint32_t words;
	uint32_t avail;
	uint32_t in_off = 0; 
	uint32_t out_off = 0; 

	/* Enable the engine */
	sys_write32(ctl, base + SS_CTL);

	while (out_off < in_len) {
		/* Feed input to RXFIFO if there's room and data remaining */
		if (in_off < in_len) {
			fcsr = sys_read32(base + SS_FCSR);
			avail = SS_RXFIFO_SPACES(fcsr);
			words = MIN(avail, (in_len - in_off) / 4);

			for (uint32_t i = 0; i < words; i++) {
				uint32_t val = sys_get_le32(in + in_off);

				sys_write32(val, base + SS_RXFIFO);
				in_off += 4;
			}
		}

		/* Drain output from TXFIFO */
		fcsr = sys_read32(base + SS_FCSR);
		avail = SS_TXFIFO_SPACES(fcsr);
		words = MIN(avail, (in_len - out_off) / 4);

		for (uint32_t i = 0; i < words; i++) {
			uint32_t val = sys_read32(base + SS_TXFIFO);

			sys_put_le32(val, out + out_off);
			out_off += 4;
		}

		/* If output not complete, wait for the engine */
		if (out_off < in_len) {
			ret = crypto_wait_tx_fifo(base, 1);
			if (ret) {
				goto disable;
			}
		}
	}

	ret = 0;

disable:
	sys_write32(0, base + SS_CTL);
	return ret;
}

/* ============== ECB Cipher Operations ============== */

static int sun8i_v3s_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct sun8i_v3s_crypto_session *sess = crypto_session(ctx);
	const struct device *dev = ctx->device;
	uintptr_t base = crypto_base(dev);
	struct sun8i_v3s_crypto_data *data = crypto_data(dev);
	int ret;

	if (pkt->in_len > AES_BLOCK_SIZE) {
		LOG_ERR("ECB: input must not exceed one block");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	crypto_write_key(base, sess->key, sess->keylen);
	sys_write32(sess->ctl_mode | SS_ENABLED | SS_ECB | SS_ENCRYPTION, base + SS_CTL);

	/* Write one block */
	for (int i = 0; i < AES_BLOCK_SIZE / 4; i++) {
		uint32_t val = sys_get_le32(pkt->in_buf + i * 4);

		sys_write32(val, base + SS_RXFIFO);
	}

	/* Wait for TX FIFO to produce output */
	ret = crypto_wait_tx_fifo(base, 4);
	if (ret) {
		goto out;
	}

	/* Read one block */
	for (int i = 0; i < AES_BLOCK_SIZE / 4; i++) {
		uint32_t val = sys_read32(base + SS_TXFIFO);

		sys_put_le32(val, pkt->out_buf + i * 4);
	}

	pkt->out_len = AES_BLOCK_SIZE;
	ret = 0;

out:
	sys_write32(0, base + SS_CTL);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int sun8i_v3s_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct sun8i_v3s_crypto_session *sess = crypto_session(ctx);
	const struct device *dev = ctx->device;
	uintptr_t base = crypto_base(dev);
	struct sun8i_v3s_crypto_data *data = crypto_data(dev);
	int ret;

	if (pkt->in_len > AES_BLOCK_SIZE) {
		LOG_ERR("ECB: input must not exceed one block");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	crypto_write_key(base, sess->key, sess->keylen);
	sys_write32(sess->ctl_mode | SS_ENABLED | SS_ECB | SS_DECRYPTION, base + SS_CTL);

	/* Write one block */
	for (int i = 0; i < AES_BLOCK_SIZE / 4; i++) {
		uint32_t val = sys_get_le32(pkt->in_buf + i * 4);

		sys_write32(val, base + SS_RXFIFO);
	}

	/* Wait for TX FIFO to produce output */
	ret = crypto_wait_tx_fifo(base, 4);
	if (ret) {
		goto out;
	}

	/* Read one block */
	for (int i = 0; i < AES_BLOCK_SIZE / 4; i++) {
		uint32_t val = sys_read32(base + SS_TXFIFO);

		sys_put_le32(val, pkt->out_buf + i * 4);
	}

	pkt->out_len = AES_BLOCK_SIZE;
	ret = 0;

out:
	sys_write32(0, base + SS_CTL);
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ============== CBC Cipher Operations ============== */

static int sun8i_v3s_cbc_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				 uint8_t *iv)
{
	struct sun8i_v3s_crypto_session *sess = crypto_session(ctx);
	const struct device *dev = ctx->device;
	uintptr_t base = crypto_base(dev);
	struct sun8i_v3s_crypto_data *data = crypto_data(dev);
	uint32_t ctl;
	int ret;
	int out_offset = 0;

	if (pkt->in_len % AES_BLOCK_SIZE != 0) {
		LOG_ERR("CBC: input length must be multiple of 16");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	crypto_write_key(base, sess->key, sess->keylen);
	crypto_write_iv(base, iv);

	ctl = sess->ctl_mode | SS_ENABLED | SS_CBC | SS_ENCRYPTION;

	/* If CAP_NO_IV_PREFIX is set, output does not include IV prefix */
	if ((ctx->flags & CAP_NO_IV_PREFIX) == 0U) {
		memcpy(pkt->out_buf, iv, AES_BLOCK_SIZE);
		out_offset = AES_BLOCK_SIZE;
	}

	ret = crypto_do_op(base, ctl, pkt->in_buf, pkt->in_len,
			   pkt->out_buf + out_offset);
	if (ret == 0) {
		pkt->out_len = pkt->in_len + out_offset;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int sun8i_v3s_cbc_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
				 uint8_t *iv)
{
	struct sun8i_v3s_crypto_session *sess = crypto_session(ctx);
	const struct device *dev = ctx->device;
	uintptr_t base = crypto_base(dev);
	struct sun8i_v3s_crypto_data *data = crypto_data(dev);
	uint32_t ctl;
	int ret;
	int in_offset = 0;
	uint8_t *in_ptr = pkt->in_buf;

	if (pkt->in_len % AES_BLOCK_SIZE != 0) {
		LOG_ERR("CBC: input length must be multiple of 16");
		return -EINVAL;
	}

	/* If CAP_NO_IV_PREFIX is set, IV is not prefixed in input */
	if ((ctx->flags & CAP_NO_IV_PREFIX) == 0U) {
		in_offset = AES_BLOCK_SIZE;
		in_ptr = pkt->in_buf + AES_BLOCK_SIZE;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	crypto_write_key(base, sess->key, sess->keylen);
	crypto_write_iv(base, iv);

	ctl = sess->ctl_mode | SS_ENABLED | SS_CBC | SS_DECRYPTION;

	ret = crypto_do_op(base, ctl, in_ptr, pkt->in_len - in_offset,
			   pkt->out_buf);
	if (ret == 0) {
		pkt->out_len = pkt->in_len - in_offset;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/* ============== Session Management ============== */

static int sun8i_v3s_get_unused_session(const struct device *dev)
{
	struct sun8i_v3s_crypto_data *data = crypto_data(dev);

	for (int i = 0; i < CONFIG_CRYPTO_SUN8I_V3S_MAX_SESSION; i++) {
		if (!data->sessions[i].in_use) {
			data->sessions[i].in_use = true;
			return i;
		}
	}

	return -1;
}

static int sun8i_v3s_cipher_begin_session(const struct device *dev,
					  struct cipher_ctx *ctx,
					  enum cipher_algo algo,
					  enum cipher_mode mode,
					  enum cipher_op op_type)
{
	struct sun8i_v3s_crypto_session *sess;
	int idx;

	if (ctx->flags & ~CRYP_SUPPORT) {
		LOG_ERR("Unsupported flags: 0x%04x", ctx->flags);
		return -ENOTSUP;
	}

	if (algo != CRYPTO_CIPHER_ALGO_AES) {
		LOG_ERR("Unsupported algo: %d", algo);
		return -ENOTSUP;
	}

	if (mode != CRYPTO_CIPHER_MODE_ECB && mode != CRYPTO_CIPHER_MODE_CBC) {
		LOG_ERR("Unsupported mode: %d", mode);
		return -ENOTSUP;
	}

	if (ctx->keylen != 16 && ctx->keylen != 24 && ctx->keylen != 32) {
		LOG_ERR("Invalid key size: %u", ctx->keylen);
		return -EINVAL;
	}

	idx = sun8i_v3s_get_unused_session(dev);
	if (idx < 0) {
		LOG_ERR("No free session");
		return -ENOSPC;
	}

	sess = &crypto_data(dev)->sessions[idx];

	/* Copy key */
	memcpy(sess->key, ctx->key.bit_stream, ctx->keylen);
	sess->keylen = ctx->keylen;

	/* Pre-compute key size bits for SS_CTL */
	switch (ctx->keylen) {
	case 16:
		sess->ctl_mode = SS_OP_AES | SS_AES_128BITS;
		break;
	case 24:
		sess->ctl_mode = SS_OP_AES | SS_AES_192BITS;
		break;
	case 32:
		sess->ctl_mode = SS_OP_AES | SS_AES_256BITS;
		break;
	}

	/* Set cipher operation handlers */
	if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = sun8i_v3s_ecb_encrypt;
			break;
		case CRYPTO_CIPHER_MODE_CBC:
			ctx->ops.cbc_crypt_hndlr = sun8i_v3s_cbc_encrypt;
			break;
		default:
			break;
		}
	} else {
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = sun8i_v3s_ecb_decrypt;
			break;
		case CRYPTO_CIPHER_MODE_CBC:
			ctx->ops.cbc_crypt_hndlr = sun8i_v3s_cbc_decrypt;
			break;
		default:
			break;
		}
	}

	ctx->drv_sessn_state = sess;
	ctx->device = dev;

	return 0;
}

static int sun8i_v3s_cipher_free_session(const struct device *dev,
					 struct cipher_ctx *ctx)
{
	struct sun8i_v3s_crypto_session *sess = crypto_session(ctx);

	sess->in_use = false;
	return 0;
}

/* ============== Hash operations (not supported by hardware) ============== */

static int sun8i_v3s_hash_begin_session(const struct device *dev,
					struct hash_ctx *ctx,
					enum hash_algo algo)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);
	ARG_UNUSED(algo);

	/* V3s crypto engine only supports MD5 and SHA1, which are not
	 * exposed by the Zephyr crypto hash API (SHA-224/256/384/512).
	 */
	return -ENOTSUP;
}

static int sun8i_v3s_hash_free_session(const struct device *dev,
				       struct hash_ctx *ctx)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);

	return 0;
}

static int sun8i_v3s_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return CRYP_SUPPORT;
}

static int sun8i_v3s_crypto_init(const struct device *dev)
{
	const struct sun8i_v3s_crypto_config *cfg = dev->config;
	struct sun8i_v3s_crypto_data *data = crypto_data(dev);
	int ret;

	/* Enable AHB bus clock */
	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(ccu)),
			       (clock_control_subsys_t)(uintptr_t)CLK_BUS_CE);
	if (ret) {
		LOG_ERR("Failed to enable CLK_BUS_CE: %d", ret);
		return ret;
	}

	/* Enable module clock */
	ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(ccu)),
			       (clock_control_subsys_t)(uintptr_t)CLK_CE);
	if (ret) {
		LOG_ERR("Failed to enable CLK_CE: %d", ret);
		return ret;
	}

	/* Set CE module clock to 150 MHz (max per datasheet) */
	uint32_t ce_rate = 150000000;

	ret = clock_control_set_rate(DEVICE_DT_GET(DT_NODELABEL(ccu)),
				     (clock_control_subsys_t)(uintptr_t)CLK_CE,
				     &ce_rate);
	if (ret && ret != -EOPNOTSUPP) {
		LOG_WRN("Failed to set CLK_CE rate: %d", ret);
	}

	/* Deassert reset */
	const struct reset_dt_spec reset_spec = RESET_DT_SPEC_GET(DT_NODELABEL(crypto));

	ret = reset_line_deassert_dt(&reset_spec);
	if (ret) {
		LOG_ERR("Failed to deassert reset: %d", ret);
		return ret;
	}

	/* Initialize mutex */
	k_mutex_init(&data->lock);

	LOG_INF("Sun8i V3s Crypto Engine initialized at 0x%08lx",
		(unsigned long)cfg->base);

	return 0;
}

static DEVICE_API(crypto, sun8i_v3s_crypto_api) = {
	.query_hw_caps = sun8i_v3s_query_caps,
	.cipher_begin_session = sun8i_v3s_cipher_begin_session,
	.cipher_free_session = sun8i_v3s_cipher_free_session,
	.cipher_async_callback_set = NULL,
	.hash_begin_session = sun8i_v3s_hash_begin_session,
	.hash_free_session = sun8i_v3s_hash_free_session,
	.hash_async_callback_set = NULL,
};

static struct sun8i_v3s_crypto_data sun8i_v3s_crypto_dev_data;

static const struct sun8i_v3s_crypto_config sun8i_v3s_crypto_dev_config = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, sun8i_v3s_crypto_init, NULL,
		      &sun8i_v3s_crypto_dev_data,
		      &sun8i_v3s_crypto_dev_config,
		      POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
		      &sun8i_v3s_crypto_api);
