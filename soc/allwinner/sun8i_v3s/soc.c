/*
 * Copyright (c) 2026 juno <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include <zephyr/drivers/clock_control_sun8i_v3s.h>
#include <zephyr/dt-bindings/clock/allwinner,sun8i-v3s-ccu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("vectors", 0, 0x1000, MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
	MMU_REGION_FLAT_ENTRY("gic", 0x01c81000, 0x7000, MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart", 0x01c28000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ccu", 0x01c20000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pio", 0x01c20800, 0x1000,  MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("timer", 0x01c20c00, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};

void soc_early_init_hook(void)
{

}

void soc_reset_hook(void)
{
	uint32_t cpu_freq = DT_PROP(DT_NODELABEL(cpu0), clock_frequency);
	const struct device *ccu = DEVICE_DT_GET(DT_NODELABEL(ccu));

	clock_control_set_rate(ccu, (void *)CLK_CPU, &cpu_freq);
}