/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/reset/allwinner,sun8i-v3s-ccu.h>

#define DT_DRV_COMPAT allwinner_sun8i_v3s_ccu_reset

#define CCU_BUS_RST_REG0         (0x2C0)
#define CCU_BUS_RST_REG1         (0x2C4)
#define CCU_BUS_RST_REG2         (0x2C8)

#define RST_BIT(rst_id)          ((rst_id) & 0x1F)
#define RST_REG(rst_id)          (((rst_id) >> 5) & 0x3)

struct sun8i_rst_info {
	uint8_t id;
	uint16_t offset;
	uint32_t bit;
};

struct sun8i_reset_config {
	uintptr_t base;
};

struct sun8i_reset_data {
	struct k_spinlock lock;
	const struct sun8i_rst_info *rst_info;
	uint32_t nb_rst;
};

#define RST_INFO(_id, _offset, _bit)	\
{	\
	.id = _id,	\
	.offset = _offset,	\
	.bit = _bit,	\
}

static struct sun8i_rst_info rst_infos[] = {
	RST_INFO(RST_BUS_CE, 0x2c0, 5),
	RST_INFO(RST_BUS_DMA, 0x2c0, 6),
	RST_INFO(RST_BUS_MMC0, 0x2c0, 8),
	RST_INFO(RST_BUS_MMC1, 0x2c0, 9),
	RST_INFO(RST_BUS_MMC2, 0x2c0, 10),
	RST_INFO(RST_BUS_DRAM, 0x2c0, 14),
	RST_INFO(RST_BUS_EMAC, 0x2c0, 17),
	RST_INFO(RST_BUS_HSTIMER, 0x2c0, 19),
	RST_INFO(RST_BUS_SPI0, 0x2c0, 20),
	RST_INFO(RST_BUS_OTG, 0x2c0, 24),
	RST_INFO(RST_BUS_EHCI0, 0x2c0, 26),
	RST_INFO(RST_BUS_OHCI0, 0x2c0, 29),
	RST_INFO(RST_BUS_VE, 0x2c4, 0),
	RST_INFO(RST_BUS_TCON0, 0x2c4, 4),
	RST_INFO(RST_BUS_CSI, 0x2c4, 8),
	RST_INFO(RST_BUS_DE, 0x2c4, 12),
	RST_INFO(RST_BUS_DBG, 0x2c4, 31),
	RST_INFO(RST_BUS_EPHY, 0x2c8, 2),
	RST_INFO(RST_BUS_CODEC, 0x2d0, 0),
	RST_INFO(RST_BUS_I2C0, 0x2d8, 0),
	RST_INFO(RST_BUS_I2C1, 0x2d8, 1),
	RST_INFO(RST_BUS_UART0, 0x2d8, 16),
	RST_INFO(RST_BUS_UART1, 0x2d8, 17),
	RST_INFO(RST_BUS_UART2, 0x2d8, 18),
};

static int sun8i_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct sun8i_reset_config *config = dev->config;
	struct sun8i_reset_data *data = dev->data;
	const struct sun8i_rst_info *info = data->rst_info;
	bool found = false;
	int i;
	uint32_t regval;
	k_spinlock_key_t key;
	for (i = 0; i < data->nb_rst; i++, info++)
	{
		if (info->id == id)
		{
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + info->offset);
	/*
	 * V3s reset bits are active-LOW (matching U-Boot's clrbits_le32
	 * for assert).  CLEAR the bit to assert.
	 */
	regval &= ~(BIT(info->bit));
	sys_write32(regval, config->base + info->offset);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int sun8i_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct sun8i_reset_config *config = dev->config;
	struct sun8i_reset_data *data = dev->data;
	const struct sun8i_rst_info *info = data->rst_info;
	bool found = false;
	int i;
	uint32_t regval;
	k_spinlock_key_t key;
	for (i = 0; i < data->nb_rst; i++, info++)
	{
		if (info->id == id)
		{
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + info->offset);
	/*
	 * V3s reset bits are active-LOW (matching U-Boot's setbits_le32
	 * for deassert).  SET the bit to deassert, CLEAR to assert.
	 */
	regval |= BIT(info->bit);
	sys_write32(regval, config->base + info->offset);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int sun8i_reset_line_toggle(const struct device *dev, uint32_t id)
{
	int ret;

	ret = sun8i_reset_line_assert(dev, id);
	if (ret) {
		return ret;
	}

	k_sleep(K_USEC(10));

	ret = sun8i_reset_line_deassert(dev, id);
	if (ret) {
		return ret;
	}

	return 0;
}

DEVICE_API(reset, sun8i_reset_driver_api) = {
	.line_assert = sun8i_reset_line_assert,
	.line_deassert = sun8i_reset_line_deassert,
	.line_toggle = sun8i_reset_line_toggle,
};

static int sun8i_reset_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define SUN8I_RESET_INIT(n) \
	static const struct sun8i_reset_config sun8i_reset_config_##n = { \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)), \
	}; \
	static struct sun8i_reset_data sun8i_reset_data_##n = { \
		.rst_info = rst_infos,	\
		.nb_rst = ARRAY_SIZE(rst_infos),	\
	};	\
	DEVICE_DT_INST_DEFINE(n, sun8i_reset_init, NULL, \
				&sun8i_reset_data_##n, &sun8i_reset_config_##n, \
				PRE_KERNEL_1, CONFIG_RESET_INIT_PRIORITY, \
				&sun8i_reset_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_RESET_INIT)
