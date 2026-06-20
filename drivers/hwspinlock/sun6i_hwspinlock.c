/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware spinlock driver for Allwinner SUN6I-compatible SoCs
 * (A31, A64, H3, H5, H6, D1, D1s, T113-S3).
 *
 * Hardware reference:
 *   T113-S3 User Manual §3.13 Spinlock
 *   Linux drivers/hwspinlock/sun6i_hwspinlock.c
 */

#define DT_DRV_COMPAT allwinner_sun6i_a31_hwspinlock

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwspinlock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(sun6i_hwspinlock, CONFIG_HWSPINLOCK_LOG_LEVEL);

/* T113-S3 §3.13.5 Register List */
#define SPINLOCK_SYSSTATUS_REG  0x0000  /* bits[29:28]: number of locks */
#define SPINLOCK_LOCK_REGN      0x0100  /* LOCK_REG[n] at 0x0100 + n*4 */

struct sun6i_hwspinlock_data {
	DEVICE_MMIO_RAM;
};

struct sun6i_hwspinlock_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	struct reset_dt_spec reset;
};

static int sun6i_hwspinlock_trylock(const struct device *dev, uint32_t id)
{
	mem_addr_t lock_addr = DEVICE_MMIO_GET(dev) + SPINLOCK_LOCK_REGN + (id << 2);

	/*
	 * T113 hardware (§3.13.3.3):
	 *   read 0 → lock was free, hardware atomically sets it to 1 (locked)
	 *   read 1 → lock was already held, caller must retry
	 */
	if (sys_read32(lock_addr) == 0) {
		return 0;
	}

	return -EBUSY;
}

static void sun6i_hwspinlock_lock(const struct device *dev, uint32_t id)
{
	mem_addr_t lock_addr = DEVICE_MMIO_GET(dev) + SPINLOCK_LOCK_REGN + (id << 2);

	while (sys_read32(lock_addr) != 0) {
		arch_spin_relax();
	}
}

static void sun6i_hwspinlock_unlock(const struct device *dev, uint32_t id)
{
	mem_addr_t lock_addr = DEVICE_MMIO_GET(dev) + SPINLOCK_LOCK_REGN + (id << 2);

	sys_write32(0, lock_addr);
}

static uint32_t sun6i_hwspinlock_get_max_id(const struct device *dev)
{
	uint32_t sysstatus = sys_read32(DEVICE_MMIO_GET(dev) + SPINLOCK_SYSSTATUS_REG);
	uint32_t num_banks = (sysstatus >> 28) & 0x3U;

	return (1U << (4U + num_banks)) - 1U;
}

static int sun6i_hwspinlock_init(const struct device *dev)
{
	const struct sun6i_hwspinlock_config *cfg = dev->config;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (cfg->clock_dev != NULL) {
		if (!device_is_ready(cfg->clock_dev)) {
			return -ENODEV;
		}
		ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
		if (ret != 0) {
			return ret;
		}
	}

	if (cfg->reset.dev != NULL) {
		if (device_is_ready(cfg->reset.dev)) {
			(void)reset_line_deassert_dt(&cfg->reset);
		}
	}

	return 0;
}

static DEVICE_API(hwspinlock, sun6i_hwspinlock_api) = {
	.trylock    = sun6i_hwspinlock_trylock,
	.lock       = sun6i_hwspinlock_lock,
	.unlock     = sun6i_hwspinlock_unlock,
	.get_max_id = sun6i_hwspinlock_get_max_id,
};

#define SUN6I_HWSPINLOCK_INIT(inst)                                                        \
	static struct sun6i_hwspinlock_data sun6i_hwspinlock_data##inst;                    \
	static const struct sun6i_hwspinlock_config sun6i_hwspinlock_config##inst = {       \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                   \
		.clock_dev = COND_CODE_1(DT_INST_CLOCKS_HAS_IDX(inst, 0),                  \
				(DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst))), (NULL)),        \
		.clock_subsys = COND_CODE_1(DT_INST_CLOCKS_HAS_IDX(inst, 0),               \
				((clock_control_subsys_t)                                 \
				 DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, clk_id)),            \
				((clock_control_subsys_t)0U)),                              \
		.reset = RESET_DT_SPEC_INST_GET_OR(inst, {0}),                             \
	};                                                                                 \
	DEVICE_DT_INST_DEFINE(inst, sun6i_hwspinlock_init, NULL,                            \
			      &sun6i_hwspinlock_data##inst,                                 \
			      &sun6i_hwspinlock_config##inst, PRE_KERNEL_1,                  \
			      CONFIG_HWSPINLOCK_INIT_PRIORITY, &sun6i_hwspinlock_api)

DT_INST_FOREACH_STATUS_OKAY(SUN6I_HWSPINLOCK_INIT);
