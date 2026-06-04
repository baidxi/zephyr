/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MDIO bus driver for Allwinner Sun8i V3s EMAC.
 *
 * The sun8i variant of the Synopsys DesignWare MAC has its MDIO
 * registers at different offsets compared to the standard DWMAC:
 *   - EMAC_MDIO_CMD  (0x48) instead of MAC_MDIO_ADDRESS
 *   - EMAC_MDIO_DATA (0x4C) instead of MAC_MDIO_DATA
 *
 * This driver accesses the parent EMAC device's MMIO region to
 * perform Clause 22 MDIO read/write operations.
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_mdio

#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(mdio_sun8i_v3s, CONFIG_MDIO_LOG_LEVEL);

/* EMAC MDIO register offsets (sun8i-specific) */
#define SUN8I_MDIO_CMD		0x48
#define SUN8I_MDIO_DATA		0x4C

/* MDIO_CMD register bits */
#define SUN8I_MII_BUSY		BIT(0)
#define SUN8I_MII_WRITE		BIT(1)
#define SUN8I_MII_CLK_SEL_SHIFT	20
#define SUN8I_MII_PHY_ADDR_SHIFT	12
#define SUN8I_MII_REG_ADDR_SHIFT	4

/* Default clock selection: divider 16 → MDC = 24MHz/16 = 1.5MHz */
#define SUN8I_MII_CLK_SEL_DEFAULT	0x01

/* MDIO busy-wait timeout */
#define SUN8I_MDIO_TIMEOUT_US	10000
#define SUN8I_MDIO_POLL_INTERVAL_US	10

struct sun8i_mdio_config {
	/** Parent EMAC device whose MMIO region we share */
	const struct device *mac_dev;
};

struct sun8i_mdio_data {
	struct k_mutex lock;
};

static int sun8i_mdio_wait_idle(mm_reg_t base)
{
	int retries = SUN8I_MDIO_TIMEOUT_US / SUN8I_MDIO_POLL_INTERVAL_US;

	while (sys_read32(base + SUN8I_MDIO_CMD) & SUN8I_MII_BUSY) {
		if (retries-- <= 0) {
			return -ETIMEDOUT;
		}
		k_usleep(SUN8I_MDIO_POLL_INTERVAL_US);
	}
	return 0;
}

static int sun8i_mdio_transfer(const struct device *dev, uint8_t prtad,
			       uint8_t regad, bool write, uint16_t write_val,
			       uint16_t *read_val)
{
	const struct sun8i_mdio_config *cfg = dev->config;
	struct sun8i_mdio_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(cfg->mac_dev);
	uint32_t cmd;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = sun8i_mdio_wait_idle(base);
	if (ret < 0) {
		goto out;
	}

	if (write) {
		sys_write32(write_val, base + SUN8I_MDIO_DATA);
	}

	/* Build command: always set BUSY to trigger the MDIO frame.
	 * See Linux stmmac_mdio.c: value always starts with MII_BUSY.
	 */
	cmd = SUN8I_MII_BUSY |
	      (SUN8I_MII_CLK_SEL_DEFAULT << SUN8I_MII_CLK_SEL_SHIFT) |
	      ((prtad & 0x1F) << SUN8I_MII_PHY_ADDR_SHIFT) |
	      ((regad & 0x1F) << SUN8I_MII_REG_ADDR_SHIFT);

	if (write) {
		cmd |= SUN8I_MII_WRITE;
	}

	sys_write32(cmd, base + SUN8I_MDIO_CMD);

	ret = sun8i_mdio_wait_idle(base);
	if (ret < 0) {
		goto out;
	}

	if (!write && read_val != NULL) {
		*read_val = (uint16_t)(sys_read32(base + SUN8I_MDIO_DATA) & 0xFFFF);
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int sun8i_mdio_read(const struct device *dev, uint8_t prtad,
			   uint8_t regad, uint16_t *data)
{
	return sun8i_mdio_transfer(dev, prtad, regad, false, 0, data);
}

static int sun8i_mdio_write(const struct device *dev, uint8_t prtad,
			    uint8_t regad, uint16_t data)
{
	return sun8i_mdio_transfer(dev, prtad, regad, true, data, NULL);
}

static int sun8i_mdio_init(const struct device *dev)
{
	const struct sun8i_mdio_config *cfg = dev->config;
	struct sun8i_mdio_data *data = dev->data;

	if (!device_is_ready(cfg->mac_dev)) {
		LOG_ERR("Parent MAC device not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);

	LOG_INF("Sun8i V3s MDIO initialized (parent MAC: %s)",
		cfg->mac_dev->name);

	return 0;
}

static DEVICE_API(mdio, sun8i_mdio_api) = {
	.read = sun8i_mdio_read,
	.write = sun8i_mdio_write,
	/* Clause 45 not supported by sun8i EMAC MDIO */
	.read_c45 = NULL,
	.write_c45 = NULL,
};

#define SUN8I_MDIO_INIT(inst)								\
	static const struct sun8i_mdio_config sun8i_mdio_config_##inst = {		\
		.mac_dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),			\
	};										\
											\
	static struct sun8i_mdio_data sun8i_mdio_data_##inst;				\
											\
	DEVICE_DT_INST_DEFINE(inst, sun8i_mdio_init, NULL,				\
			      &sun8i_mdio_data_##inst, &sun8i_mdio_config_##inst,	\
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,			\
			      &sun8i_mdio_api);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_MDIO_INIT)
