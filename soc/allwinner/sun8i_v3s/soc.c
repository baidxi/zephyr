/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include <zephyr/dt-bindings/clock/allwinner,sun8i-v3s-ccu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/kernel.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("vectors", 0, 0x1000, MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
	MMU_REGION_FLAT_ENTRY("display_clocks", 0x01000000, 0x100000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("mixer0", 0x01100000, 0x100000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("gic", 0x01c81000, 0x7000, MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart", 0x01c28000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ccu", 0x01c20000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pio", 0x01c20800, 0x1000,  MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("timer", 0x01c20c00, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("dma",   0x01c02000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("spi0", 0x01c68000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("twi0", 0x01c2ac00, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("twi1", 0x01c2b000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("usbotg", 0x01c19000, 0x0400, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("phy_ctl", 0x01c19400, 0x2c, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pmu0", 0x01c1a800, 0x04, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("usb_hci0", 0x01c1a000, 0x100, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("sdhc0", 0x01c0f000, 0x400, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("sdhc1", 0x01C10000, 0x400, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("sdhc2", 0x01C11000, 0x400, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ce", 0x01c15000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("tcon0", 0x01c0c000, 0x1000, MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pwm", 0x01c21400, 0x400,  MT_DEVICE | MPERM_R | MPERM_W)
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