/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner DE2 Display Engine Clock Controller driver.
 *
 * The DE2 CCU sits at 0x01000000 and provides:
 *  - Bus clock gates (0x04): CLK_BUS_MIXER0/1, CLK_BUS_WB, CLK_BUS_ROT
 *  - Module clock gates (0x00): CLK_MIXER0/1, CLK_WB, CLK_ROT
 *  - Reset lines (0x08): RST_MIXER0/1, RST_WB, RST_ROT
 *  - Clock dividers (0x0C): MIXER0/1_DIV, WB_DIV, ROT_DIV
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_de2_clk

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/clock/sun8i-de2.h>
#include <zephyr/dt-bindings/reset/sun8i-de2.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(de2_ccu, CONFIG_LOG_DEFAULT_LEVEL);

/* DE2 CCU registers */
#define DE2_MOD_CLK_REG          0x00  /* Module clock gates */
#define DE2_BUS_CLK_REG          0x04  /* Bus clock gates */
#define DE2_RST_REG              0x08  /* Reset control */
#define DE2_DIV_REG              0x0C  /* Clock dividers */

/* Divider shift: 4 bits per module */
#define DE2_DIV_SHIFT(id)        ((id) * 4)

struct de2_ccu_config {
	uintptr_t base;
	const struct device *parent_clock;  /* Main CCU for CLK_BUS_DE/CLK_DE */
	uint32_t bus_clk_id;                /* CLK_BUS_DE from main CCU */
	uint32_t mod_clk_id;                /* CLK_DE from main CCU */
	struct reset_dt_spec reset;          /* RST_BUS_DE from main CCU */
};

struct de2_ccu_data {
	struct k_spinlock lock;
};

struct de2_bus_gate {
	uint32_t clk_id;
	uint32_t bit;
};

static const struct de2_bus_gate de2_bus_gates[] = {
	{ CLK_BUS_MIXER0, 0 },
	{ CLK_BUS_MIXER1, 1 },
	{ CLK_BUS_WB,     2 },
	{ CLK_BUS_ROT,    3 },
};

static const struct de2_bus_gate de2_mod_gates[] = {
	{ CLK_MIXER0, 0 },
	{ CLK_MIXER1, 1 },
	{ CLK_WB,     2 },
	{ CLK_ROT,    3 },
};

struct de2_reset {
	uint32_t rst_id;
	uint32_t bit;
};

static const struct de2_reset de2_resets[] = {
	{ RST_MIXER0, 0 },
	{ RST_MIXER1, 1 },
	{ RST_WB,     2 },
	{ RST_ROT,    3 },
};

static int de2_find_bus_gate(uint32_t clk_id, uint32_t *bit)
{
	for (int i = 0; i < ARRAY_SIZE(de2_bus_gates); i++) {
		if (de2_bus_gates[i].clk_id == clk_id) {
			*bit = de2_bus_gates[i].bit;
			return 0;
		}
	}
	return -EINVAL;
}

static int de2_find_mod_gate(uint32_t clk_id, uint32_t *bit)
{
	for (int i = 0; i < ARRAY_SIZE(de2_mod_gates); i++) {
		if (de2_mod_gates[i].clk_id == clk_id) {
			*bit = de2_mod_gates[i].bit;
			return 0;
		}
	}
	return -EINVAL;
}

static int de2_find_reset(uint32_t rst_id, uint32_t *bit)
{
	for (int i = 0; i < ARRAY_SIZE(de2_resets); i++) {
		if (de2_resets[i].rst_id == rst_id) {
			*bit = de2_resets[i].bit;
			return 0;
		}
	}
	return -EINVAL;
}

static int de2_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct de2_ccu_config *cfg = dev->config;
	struct de2_ccu_data *data = dev->data;
	uint32_t clk_id = (uint32_t)sys;
	uint32_t bit;
	k_spinlock_key_t key;
	uint32_t reg;

	/* Try bus clock gate first */
	if (de2_find_bus_gate(clk_id, &bit) == 0) {
		key = k_spin_lock(&data->lock);
		reg = sys_read32(cfg->base + DE2_BUS_CLK_REG);
		reg |= BIT(bit);
		sys_write32(reg, cfg->base + DE2_BUS_CLK_REG);
		k_spin_unlock(&data->lock, key);
		LOG_DBG("DE2 bus clock %u on (bit %u)", clk_id, bit);
		return 0;
	}

	/* Try module clock gate */
	if (de2_find_mod_gate(clk_id, &bit) == 0) {
		key = k_spin_lock(&data->lock);
		reg = sys_read32(cfg->base + DE2_MOD_CLK_REG);
		reg |= BIT(bit);
		sys_write32(reg, cfg->base + DE2_MOD_CLK_REG);
		k_spin_unlock(&data->lock, key);
		LOG_DBG("DE2 mod clock %u on (bit %u)", clk_id, bit);
		return 0;
	}

	return -EINVAL;
}

static int de2_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct de2_ccu_config *cfg = dev->config;
	struct de2_ccu_data *data = dev->data;
	uint32_t clk_id = (uint32_t)sys;
	uint32_t bit;
	k_spinlock_key_t key;
	uint32_t reg;

	if (de2_find_bus_gate(clk_id, &bit) == 0) {
		key = k_spin_lock(&data->lock);
		reg = sys_read32(cfg->base + DE2_BUS_CLK_REG);
		reg &= ~BIT(bit);
		sys_write32(reg, cfg->base + DE2_BUS_CLK_REG);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	if (de2_find_mod_gate(clk_id, &bit) == 0) {
		key = k_spin_lock(&data->lock);
		reg = sys_read32(cfg->base + DE2_MOD_CLK_REG);
		reg &= ~BIT(bit);
		sys_write32(reg, cfg->base + DE2_MOD_CLK_REG);
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	return -EINVAL;
}

static int de2_clock_get_rate(const struct device *dev,
			       clock_control_subsys_t sys,
			       uint32_t *rate)
{
	const struct de2_ccu_config *cfg = dev->config;
	uint32_t clk_id = (uint32_t)sys;
	uint32_t parent_rate = 0;
	int ret;

	/* Get parent rate (CLK_DE) */
	if (cfg->parent_clock) {
		ret = clock_control_get_rate(cfg->parent_clock,
					     (clock_control_subsys_t)(uintptr_t)cfg->mod_clk_id,
					     &parent_rate);
		if (ret != 0) {
			/* fallback: assume 300MHz */
			parent_rate = 300000000U;
		}
	} else {
		parent_rate = 300000000U;
	}

	/* Apply divider */
	uint32_t div_reg = sys_read32(cfg->base + DE2_DIV_REG);
	uint32_t div_shift = DE2_DIV_SHIFT(clk_id);
	uint32_t div = (div_reg >> div_shift) & 0xf;

	*rate = parent_rate / (div + 1);
	return 0;
}

static DEVICE_API(clock_control, de2_clock_api) = {
	.on       = de2_clock_on,
	.off      = de2_clock_off,
	.get_rate = de2_clock_get_rate,
};

static int de2_reset_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct de2_ccu_config *cfg = dev->config;
	uint32_t bit;

	if (de2_find_reset(id, &bit) != 0) {
		return -EINVAL;
	}

	uint32_t reg = sys_read32(cfg->base + DE2_RST_REG);

	*status = (reg & BIT(bit)) ? 1 : 0;
	return 0;
}

static int de2_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct de2_ccu_config *cfg = dev->config;
	uint32_t bit;

	if (de2_find_reset(id, &bit) != 0) {
		return -EINVAL;
	}

	uint32_t reg = sys_read32(cfg->base + DE2_RST_REG);

	reg |= BIT(bit);
	sys_write32(reg, cfg->base + DE2_RST_REG);
	return 0;
}

static int de2_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct de2_ccu_config *cfg = dev->config;
	uint32_t bit;

	if (de2_find_reset(id, &bit) != 0) {
		return -EINVAL;
	}

	uint32_t reg = sys_read32(cfg->base + DE2_RST_REG);

	reg &= ~BIT(bit);
	sys_write32(reg, cfg->base + DE2_RST_REG);
	return 0;
}

static int de2_reset_line_toggle(const struct device *dev, uint32_t id)
{
	int ret;

	ret = de2_reset_line_assert(dev, id);
	if (ret != 0) {
		return ret;
	}
	/* Minimal delay for reset to take effect */
	k_busy_wait(1);
	ret = de2_reset_line_deassert(dev, id);
	return ret;
}

static DEVICE_API(reset, de2_reset_api) = {
	.status   = de2_reset_status,
	.line_assert = de2_reset_line_assert,
	.line_deassert = de2_reset_line_deassert,
	.line_toggle = de2_reset_line_toggle,
};

static int de2_ccu_init(const struct device *dev)
{
	const struct de2_ccu_config *cfg = dev->config;
	int ret;

	/*
	 * Bug #11 fix: Enable parent clocks from main CCU.
	 * Without CLK_BUS_DE, the entire DE2 register space (0x01000000+)
	 * is inaccessible — all reads return 0, all writes are ignored.
	 */

	/* 1. Enable bus clock (CLK_BUS_DE) for AHB register access */
	ret = clock_control_on(cfg->parent_clock,
			       (clock_control_subsys_t)(uintptr_t)cfg->bus_clk_id);
	if (ret) {
		LOG_ERR("Failed to enable DE bus clock: %d", ret);
		return ret;
	}
	LOG_INF("CLK_BUS_DE enabled");

	/* 2. Enable module clock (CLK_DE) for DE2 processing pipeline */
	ret = clock_control_on(cfg->parent_clock,
			       (clock_control_subsys_t)(uintptr_t)cfg->mod_clk_id);
	if (ret) {
		LOG_ERR("Failed to enable DE module clock: %d", ret);
		return ret;
	}
	LOG_INF("CLK_DE enabled");

	/* 3. Deassert bus reset (RST_BUS_DE) */
	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("Failed to deassert DE bus reset: %d", ret);
		return ret;
	}
	LOG_INF("RST_BUS_DE deasserted");

	/* 4. Deassert all DE2 submodule resets (MIXER0/1, WB, ROT) */
	sys_write32(0x0000000F, cfg->base + DE2_RST_REG);
	LOG_INF("DE2 submodule resets deasserted (RST=0x%08x)",
		sys_read32(cfg->base + DE2_RST_REG));

	/* 5. Enable all bus clock gates and module clock gates */
	sys_write32(0x0000000F, cfg->base + DE2_BUS_CLK_REG);
	sys_write32(0x0000000F, cfg->base + DE2_MOD_CLK_REG);
	LOG_INF("DE2 bus/mod gates enabled (BUS=0x%08x MOD=0x%08x)",
		sys_read32(cfg->base + DE2_BUS_CLK_REG),
		sys_read32(cfg->base + DE2_MOD_CLK_REG));

	return 0;
}

#define DE2_CCU_INIT(inst)                                                     \
	static struct de2_ccu_data de2_ccu_data_##inst;                        \
                                                                                \
	static const struct de2_ccu_config de2_ccu_config_##inst = {           \
		.base = DT_INST_REG_ADDR(inst),                                 \
		.parent_clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),      \
		.bus_clk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, bus, clk_id),  \
		.mod_clk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, mod, clk_id),  \
		.reset = RESET_DT_SPEC_INST_GET(inst),                         \
	};                                                                      \
                                                                                \
	DEVICE_DT_INST_DEFINE(inst,                                             \
			      de2_ccu_init,                                     \
			      NULL,                                             \
			      &de2_ccu_data_##inst,                             \
			      &de2_ccu_config_##inst,                           \
			      PRE_KERNEL_1,                                     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,               \
			      &de2_clock_api);

DT_INST_FOREACH_STATUS_OKAY(DE2_CCU_INIT)
