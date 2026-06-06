/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner DE2 CCU Reset Controller driver.
 *
 * Provides reset control for DE2 Mixer, Write-Back, and Rotation modules.
 * The reset register is at offset 0x08 within the DE2 CCU register space.
 * Reset bits are active-low: bit=0 means assert, bit=1 means deassert.
 */

#define DT_DRV_COMPAT allwinner_sun8i_de2_reset

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/reset/sun8i-de2.h>

#define DE2_RST_REG   0x08

struct de2_reset_config {
	uintptr_t base;
	const struct device *parent;  /* display_clocks: for init ordering */
};

/*
 * Reset bit mapping for DE2 CCU (register 0x08).
 * Bits are active-low: clear bit to assert, set bit to deassert.
 */
static int de2_rst_id_to_bit(uint32_t id)
{
	switch (id) {
	case RST_MIXER0: return 0;
	case RST_MIXER1: return 1;
	case RST_WB:     return 2;
	case RST_ROT:    return 3;
	default:         return -1;
	}
}

static int de2_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct de2_reset_config *cfg = dev->config;
	int bit = de2_rst_id_to_bit(id);

	if (bit < 0) {
		return -EINVAL;
	}

	/* Active-low: clear bit to assert reset */
	uint32_t reg = sys_read32(cfg->base + DE2_RST_REG);
	reg &= ~BIT(bit);
	sys_write32(reg, cfg->base + DE2_RST_REG);

	return 0;
}

static int de2_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct de2_reset_config *cfg = dev->config;
	int bit = de2_rst_id_to_bit(id);

	if (bit < 0) {
		return -EINVAL;
	}

	/* Active-low: set bit to deassert reset */
	uint32_t reg = sys_read32(cfg->base + DE2_RST_REG);
	reg |= BIT(bit);
	sys_write32(reg, cfg->base + DE2_RST_REG);

	return 0;
}

static int de2_reset_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct de2_reset_config *cfg = dev->config;
	int bit = de2_rst_id_to_bit(id);

	if (bit < 0) {
		return -EINVAL;
	}

	uint32_t reg = sys_read32(cfg->base + DE2_RST_REG);
	/* Active-low: bit=0 means asserted (in reset), bit=1 means deasserted */
	*status = (reg & BIT(bit)) ? 0 : 1;
	return 0;
}

static int de2_reset_line_toggle(const struct device *dev, uint32_t id)
{
	int ret;

	ret = de2_reset_line_assert(dev, id);
	if (ret != 0) {
		return ret;
	}
	k_busy_wait(1);
	ret = de2_reset_line_deassert(dev, id);
	return ret;
}

static DEVICE_API(reset, de2_reset_api) = {
	.status        = de2_reset_status,
	.line_assert   = de2_reset_line_assert,
	.line_deassert = de2_reset_line_deassert,
	.line_toggle   = de2_reset_line_toggle,
};

static int de2_reset_init(const struct device *dev)
{
	/* Deassert all resets by default (set all bits) */
	const struct de2_reset_config *cfg = dev->config;

	sys_write32(0x0000000F, cfg->base + DE2_RST_REG);
	return 0;
}

#define DE2_RESET_INIT(inst)                                                  \
	static const struct de2_reset_config de2_reset_config_##inst = {     \
		.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(inst))),           \
		.parent = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),       \
	};                                                                      \
	DEVICE_DT_INST_DEFINE(inst,                                            \
			      de2_reset_init,                                   \
			      NULL,                                             \
			      NULL,                                             \
			      &de2_reset_config_##inst,                         \
			      PRE_KERNEL_1,                                     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,               \
			      &de2_reset_api);

DT_INST_FOREACH_STATUS_OKAY(DE2_RESET_INIT)
