/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("vectors", 0, 0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W | MPERM_X),
	/*
	 * DDR identity mapping (128 MB at 0x40000000).
	 * Required for boot: the kernel image executes from physical
	 * 0x40000000.
	 *
	 * z_arm_mmu_init() maps SoC regions (this table) with 4 kB L2
	 * pages, never 1 MB sections.  Therefore k_mem_map_phys_bare() +
	 * K_MEM_DIRECT_MAP can remap any DDR page to a different memory
	 * type (e.g. the SDHC IDMAC DMA buffer to Normal Non-cacheable
	 * via K_MEM_ARM_NORMAL_NC) by simply updating the corresponding
	 * L2 PTE attribute — no section splitting required.  This replaces
	 * the previous hardcoded MT_DEVICE dma_nc region which used Device
	 * memory (strict ordering) and corrupted IDMAC burst transfers.
	 */
	MMU_REGION_FLAT_ENTRY("ddr", 0x40000000, 0x08000000,
			      MT_NORMAL | MPERM_R | MPERM_W | MPERM_X),
	MMU_REGION_FLAT_ENTRY("gic", 0x03021000, 0x7000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart0", 0x02500000, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart1", 0x02500400, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart2", 0x02500800, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart3", 0x02500C00, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart4", 0x02501000, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("uart5", 0x02501400, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("twi", 0x02502000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ccu", 0x02001000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pio", 0x02000000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("timer", 0x02050000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("dma", 0x03002000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("syscon", 0x03000000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("hwspinlock", 0x03005000, 0x1000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ehci0", 0x04101000, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ohci0", 0x04101400, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("usb", 0x04100000, 0x400,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("phy_ctrl", 0x04100400, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pmu0", 0x04101800, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ehci1", 0x04200000, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("ohci1", 0x04200400, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("pmu1", 0x04200800, 0x100,
			      MT_DEVICE | MPERM_R | MPERM_W),
	/* SMHC (MMC/SD) — 0x04020000 (SMHC0), 0x04021000 (SMHC1),
	 * 0x04022000 (SMHC2), each 0x1000.
	 */
	MMU_REGION_FLAT_ENTRY("smhc", 0x04020000, 0x3000,
			      MT_DEVICE | MPERM_R | MPERM_W),
	MMU_REGION_FLAT_ENTRY("rtc", 0x7090000, 0x0310, MT_DEVICE | MPERM_R | MPERM_W)
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};

void relocate_vector_table(void)
{
	write_sctlr(read_sctlr() & ~HIVECS);
	__set_VBAR((uint32_t)(uintptr_t)_vector_start);
	__ISB();
}

#ifdef CONFIG_T113_EARLY_PRINTK
/*
 * Early UART access (ns16550, reg-shift=2).
 * UART base selected via Kconfig choice T113_EARLY_PRINTK_UART.
 */
#if defined(CONFIG_T113_EARLY_PRINTK_UART0)
#define EARLY_UART_BASE 0x02500000UL
#elif defined(CONFIG_T113_EARLY_PRINTK_UART1)
#define EARLY_UART_BASE 0x02500400UL
#elif defined(CONFIG_T113_EARLY_PRINTK_UART2)
#define EARLY_UART_BASE 0x02500800UL
#elif defined(CONFIG_T113_EARLY_PRINTK_UART3)
#define EARLY_UART_BASE 0x02500C00UL
#elif defined(CONFIG_T113_EARLY_PRINTK_UART4)
#define EARLY_UART_BASE 0x02501000UL
#elif defined(CONFIG_T113_EARLY_PRINTK_UART5)
#define EARLY_UART_BASE 0x02501400UL
#else
#define EARLY_UART_BASE 0x02500C00UL
#endif

/* ns16550 register offsets, reg-shift=2 (multiply by 4) */
#define UART_THR  (EARLY_UART_BASE + 0x00)  /* TX holding (DLAB=0) / DLL (DLAB=1) */
#define UART_DLH  (EARLY_UART_BASE + 0x04)  /* Divisor Latch MSB */
#define UART_FCR  (EARLY_UART_BASE + 0x08)  /* FIFO Control */
#define UART_LCR  (EARLY_UART_BASE + 0x0C)  /* Line Control */
#define UART_LSR  (EARLY_UART_BASE + 0x14)  /* Line Status */

static void early_uart_putc(int c)
{
	while (!(sys_read32(UART_LSR) & BIT(5))) { }
	sys_write32(c, UART_THR);
}

#endif

void soc_reset_hook(void)
{
	uint32_t sctlr;

	/*
	 * Bootloader (U-Boot SPL) may have left the MMU enabled with an
	 * address mapping that Zephyr does not expect. Disable MMU,
	 * caches, and invalidate TLBs so that z_arm_mmu_init() later
	 * starts from a clean state.
	 */
	sctlr = __get_SCTLR();
	sctlr &= ~(SCTLR_M_Msk | SCTLR_C_Msk | SCTLR_I_Msk);
	__set_SCTLR(sctlr);
	__DSB();
	__ISB();

	/* Invalidate entire TLB (ARMv7-A: TLBIALL) */
	__asm__ volatile("mcr p15, 0, %0, c8, c7, 0" : : "r"(0) : "memory");
	__DSB();
	__ISB();

	/* Invalidate branch predictor */
	__asm__ volatile("mcr p15, 0, %0, c7, c5, 6" : : "r"(0) : "memory");
	__DSB();
	__ISB();
}

void soc_early_init_hook(void)
{
	/* pinctrl driver now handles pinmux; nothing to do here */
}

void soc_prep_hook()
{
	// printk("early hook\n");
}

#ifdef CONFIG_T113_EARLY_PRINTK

int arch_printk_char_out(int c)
{
	if (c == '\n') {
		early_uart_putc('\r');
	}
	early_uart_putc(c);
	return c;
}
#endif /* CONFIG_T113_EARLY_PRINTK */
