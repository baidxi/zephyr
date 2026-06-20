/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner D1 / T113-S3 CCU driver.
 */

#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control_sun20i_d1.h>
#include <zephyr/dt-bindings/clock/sun20i-d1-ccu.h>
#include <zephyr/dt-bindings/reset/sun20i-d1-ccu.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT allwinner_sun20i_d1_ccu

LOG_MODULE_REGISTER(ccu_d1, CONFIG_LOG_DEFAULT_LEVEL);

struct d1_gate {
	uint32_t offset;
	uint32_t bit;
	uint32_t id;
};

#define G(id, off, bit) { off, bit, id }
#define R(id, off, bit) { off, bit, id }

static const struct d1_gate d1_gates[] = {
	/*
	 * APB0 bus clock — APB0_CLK_REG (0x0520).
	 * Bit 31 is reserved → no-op write; included so that
	 * D1_CLK_APB0 is a valid ID (pinctrl references it).
	 */
	G(D1_CLK_APB0,       0x520, 31),
	/* UART — APB1 bus, UART_BGR_REG (0x090c) */
	G(D1_CLK_BUS_UART0,  0x90c,  0), G(D1_CLK_BUS_UART1,  0x90c,  1),
	G(D1_CLK_BUS_UART2,  0x90c,  2), G(D1_CLK_BUS_UART3,  0x90c,  3),
	G(D1_CLK_BUS_UART4,  0x90c,  4), G(D1_CLK_BUS_UART5,  0x90c,  5),
	/* TWI (I2C) — APB1 bus, TWI_BGR_REG (0x091c) */
	G(D1_CLK_BUS_I2C0,   0x91c,  0), G(D1_CLK_BUS_I2C1,   0x91c,  1),
	G(D1_CLK_BUS_I2C2,   0x91c,  2), G(D1_CLK_BUS_I2C3,   0x91c,  3),
	/* SPI — PSI/AHB bus, SPI_BGR_REG (0x096c) */
	G(D1_CLK_BUS_SPI0,   0x96c,  0), G(D1_CLK_BUS_SPI1,   0x96c,  1),
	/* SMHC (MMC) — PSI/AHB bus, SMHC_BGR_REG (0x084c) */
	G(D1_CLK_BUS_MMC0,   0x84c,  0), G(D1_CLK_BUS_MMC1,   0x84c,  1),
	G(D1_CLK_BUS_MMC2,   0x84c,  2),
	/* SMHC (MMC) module clocks — SMHCx_CLK_REG (0x0830/4/8), bit 31 = gate.
	 * Default reset state: source=HOSC24M, M=0(÷1), N=0(÷1) → 24 MHz.
	 */
	G(D1_CLK_MMC0,       0x830, 31), G(D1_CLK_MMC1,       0x834, 31),
	G(D1_CLK_MMC2,       0x838, 31),
	/* DMA — PSI/AHB bus, DMA_BGR_REG (0x070c) */
	G(D1_CLK_BUS_DMA,    0x70c,  0),
	/* EMAC — PSI/AHB bus, EMAC_BGR_REG (0x097c) */
	G(D1_CLK_BUS_EMAC,   0x97c,  0),
	/* CAN — APB1 bus, CAN_BGR_REG (0x092c) */
	G(D1_CLK_BUS_CAN0,   0x92c,  0), G(D1_CLK_BUS_CAN1,   0x92c,  1),
	/* PWM — APB0 bus, PWM_BGR_REG (0x07ac) */
	G(D1_CLK_BUS_PWM,    0x7ac,  0),
	/* DRAM — PSI/AHB bus, DRAM_BGR_REG (0x080c) */
	G(D1_CLK_BUS_DRAM,   0x80c,  0),
	/* HSTIMER — PSI/AHB bus, HSTIMER_BGR_REG (0x073c) */
	G(D1_CLK_BUS_HSTIMER, 0x73c, 0),
	/* MSGBOX — PSI/AHB bus, MSGBOX_BGR_REG (0x071c / 0x072c) */
	G(D1_CLK_BUS_MSGBOX0, 0x71c, 0),
	G(D1_CLK_BUS_MSGBOX1, 0x71c, 1),
	G(D1_CLK_BUS_MSGBOX2, 0x71c, 2),
	/* SPINLOCK — PSI/AHB bus, SPINLOCK_BGR_REG (0x072c) */
	G(D1_CLK_BUS_SPINLOCK, 0x72c, 0),
	/* MBUS master clock gates — MBUS_CLK_GATING (0x0804).
	 * Each master (DMA, VE, CE, …) gets its own enable bit;
	 * the parent is the fixed-factor MBUS clock from DRAM.
	 */
	G(D1_CLK_MBUS_DMA,   0x804,  0), G(D1_CLK_MBUS_VE,   0x804,  1),
	G(D1_CLK_MBUS_CE,    0x804,  2), G(D1_CLK_MBUS_TVIN, 0x804,  7),
	G(D1_CLK_MBUS_CSI,   0x804,  8), G(D1_CLK_MBUS_G2D,  0x804, 10),
	G(D1_CLK_MBUS_RISCV, 0x804, 11),
	/* USB — USB_BGR_REG (0x0a8c)
	 * Bus clock gate bits (active-high): OHCI0=0, OHCI1=1,
	 * EHCI0=4, EHCI1=5, OTG=8.
	 *
	 * USB OHCI 48 MHz reference clock gates (active-high bit 31):
	 *   0xa70 bit 31 = OHCI0,  0xa74 bit 31 = OHCI1.
	 * These are in the SAME register as the PHY reset (bit 30)
	 * and clock-source mux (bits[25:24]).  sys_set_bit() does a
	 * read-modify-write so only bit 31 is touched; the PHY reset
	 * (bit 30) is left for the PHY driver's on-demand management.
	 * The mux is configured separately in d1_ccu_init().
	 */
	G(D1_CLK_BUS_OHCI0,  0xa8c,  0), G(D1_CLK_BUS_OHCI1,  0xa8c,  1),
	G(D1_CLK_BUS_EHCI0,  0xa8c,  4), G(D1_CLK_BUS_EHCI1,  0xa8c,  5),
	G(D1_CLK_BUS_OTG,    0xa8c,  8),
	G(D1_CLK_USB_OHCI0,  0xa70, 31), G(D1_CLK_USB_OHCI1,  0xa74, 31),
};

static const struct d1_gate d1_resets[] = {
	/* UART — UART_BGR_REG (0x090c), reset bits 16-21 */
	R(D1_RST_BUS_UART0,  0x90c, 16), R(D1_RST_BUS_UART1,  0x90c, 17),
	R(D1_RST_BUS_UART2,  0x90c, 18), R(D1_RST_BUS_UART3,  0x90c, 19),
	R(D1_RST_BUS_UART4,  0x90c, 20), R(D1_RST_BUS_UART5,  0x90c, 21),
	/* TWI (I2C) — TWI_BGR_REG (0x091c), reset bits 16-19 */
	R(D1_RST_BUS_I2C0,   0x91c, 16), R(D1_RST_BUS_I2C1,   0x91c, 17),
	R(D1_RST_BUS_I2C2,   0x91c, 18), R(D1_RST_BUS_I2C3,   0x91c, 19),
	/* SPI — SPI_BGR_REG (0x096c), reset bits 16-17 */
	R(D1_RST_BUS_SPI0,   0x96c, 16), R(D1_RST_BUS_SPI1,   0x96c, 17),
	/* SMHC (MMC) — SMHC_BGR_REG (0x084c), reset bits 16-18 */
	R(D1_RST_BUS_MMC0,   0x84c, 16), R(D1_RST_BUS_MMC1,   0x84c, 17),
	R(D1_RST_BUS_MMC2,   0x84c, 18),
	/* DMA — DMA_BGR_REG (0x070c), reset bit 16 */
	R(D1_RST_BUS_DMA,    0x70c, 16),
	/* EMAC — EMAC_BGR_REG (0x097c), reset bit 16 */
	R(D1_RST_BUS_EMAC,   0x97c, 16),
	/* CAN — CAN_BGR_REG (0x092c), reset bits 16-17 */
	R(D1_RST_BUS_CAN0,   0x92c, 16), R(D1_RST_BUS_CAN1,   0x92c, 17),
	/* MSGBOX — MSGBOX_BGR_REG (0x071c), reset bits 16-18 */
	R(D1_RST_BUS_MSGBOX0, 0x71c, 16), R(D1_RST_BUS_MSGBOX1, 0x71c, 17),
	R(D1_RST_BUS_MSGBOX2, 0x71c, 18),
	/* SPINLOCK — SPINLOCK_BGR_REG (0x072c), reset bit 16 */
	R(D1_RST_BUS_SPINLOCK, 0x72c, 16),
};

#undef G
#undef R

static const struct d1_gate *find_gate(uint32_t id)
{
	for (int i = 0; i < ARRAY_SIZE(d1_gates); i++)
		if (d1_gates[i].id == id) return &d1_gates[i];
	return NULL;
}

static int d1_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t id = (uint32_t)(uintptr_t)sys;
	const struct d1_gate *g = find_gate(id);

	if (!g) return -ENOTSUP;
	sys_set_bit(DT_INST_REG_ADDR(0) + g->offset, g->bit);
	return 0;
}

static int d1_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t id = (uint32_t)(uintptr_t)sys;
	const struct d1_gate *g = find_gate(id);

	if (!g) return -ENOTSUP;
	sys_clear_bit(DT_INST_REG_ADDR(0) + g->offset, g->bit);
	return 0;
}

/* PLL register offsets (from T113-S3 CCU manual §3.3.5) */
#define PLL_CPUX_REG    0x000
#define PLL_DDR0_REG    0x010
#define PLL_PERIPH0_REG 0x020
#define PLL_VIDEO0_REG  0x040
#define PLL_VIDEO1_REG  0x048
#define PLL_VE_REG      0x058
#define PLL_AUDIO0_REG  0x078
#define PLL_AUDIO1_REG  0x080
#define PSI_CLK_REG     0x510
#define APB0_CLK_REG    0x520
#define APB1_CLK_REG    0x524
#define CPUX_CLK_REG    0x500

#define HOSC_HZ         (24U * 1000U * 1000U)
#define CPUX_SRC_SHIFT  24
#define CPUX_SRC_WIDTH  3

/*
 * CPU clock configuration helpers (T113-S3 manual §3.3.4 / §3.3.6).
 *
 * CPUX_CLK_REG (0x500) CLK_SRC_SEL[26:24] selects the CPU clock source:
 *   0 = HOSC 24 MHz       1 = LOSC 32.768 kHz   2 = IOSC (int. RC, ~16 MHz)
 *   3 = PLL_CPUX          4 = PLL_PERIPH0(1X)   5 = PLL_PERIPH0(2X)
 *   6 = PLL_PERIPH0(800M)
 *
 * The CPU core runs at the selected source frequency; the cpux-axi
 * (bits[1:0]) and cpux-apb (bits[9:8]) dividers feed the AXI/APB buses,
 * not the core itself.
 */

/* PLL_CPUX (0x000): per bit[31] description, "PLL_CPU = InputFreq * N" where the
 * effective factor N = PLL_N[15:8] + 1, and the output divider M = PLL_M[1:0] + 1.
 * The driver forces M to 0 (=> 1) in d1_ccu_init() step 2.
 */
static uint32_t d1_pll_cpux_rate(uintptr_t base)
{
	uint32_t reg = sys_read32(base + PLL_CPUX_REG);
	uint32_t n = (reg >> 8) & 0xff;		/* PLL_N: effective factor = N + 1 */
	uint32_t m = reg & 0x3;			/* PLL_M: effective divisor = M + 1 */

	return (uint32_t)((uint64_t)HOSC_HZ * (n + 1) / (m + 1));
}

/* PLL_PERIPH0 (0x020): f4x = HOSC * (PLL_N[15:8] + 1) / (M[1] + 1).  The 4X VCO
 * feeds post-dividers 2X (bits[18:16], +1) and 800M (bits[22:20], +1); the 1X
 * clock is the 2X output divided by 2.
 */
static uint32_t d1_pll_periph0_rate(uintptr_t base, uint32_t src)
{
	uint32_t reg = sys_read32(base + PLL_PERIPH0_REG);
	uint32_t f4x = (uint32_t)((uint64_t)HOSC_HZ *
				  (((reg >> 8) & 0xff) + 1) /
				  (((reg >> 1) & 0x1) + 1));
	uint32_t div_2x = ((reg >> 16) & 0x7) + 1;
	uint32_t div_800m = ((reg >> 20) & 0x7) + 1;

	switch (src) {
	case 4:				/* PLL_PERIPH0 (1X) = 2X / 2 */
		return f4x / div_2x / 2;
	case 5:				/* PLL_PERIPH0 (2X) */
		return f4x / div_2x;
	case 6:				/* PLL_PERIPH0 (800M) */
		return f4x / div_800m;
	default:
		return 0;
	}
}

/* Return the current CPU core frequency in Hz based on the CPUX source mux. */
static uint32_t d1_cpu_rate(uintptr_t base)
{
	uint32_t src = (sys_read32(base + CPUX_CLK_REG) >> CPUX_SRC_SHIFT) &
		       (BIT(CPUX_SRC_WIDTH) - 1);

	switch (src) {
	case 0:				/* HOSC 24 MHz */
		return HOSC_HZ;
	case 1:				/* LOSC 32.768 kHz */
		return 32768U;
	case 2:				/* IOSC internal RC, ~16 MHz (approx.) */
		return 16U * 1000U * 1000U;
	case 3:				/* PLL_CPUX */
		return d1_pll_cpux_rate(base);
	case 4:				/* PLL_PERIPH0 (1X) */
	case 5:				/* PLL_PERIPH0 (2X) */
	case 6:				/* PLL_PERIPH0 (800M) */
		return d1_pll_periph0_rate(base, src);
	default:
		return 0;
	}
}

static int d1_ccu_init(const struct device *dev)
{
	uintptr_t base = DT_INST_REG_ADDR(0);
	uint32_t val;
	int i;

	/*
	 * 0. Restore APB1 (UART bus clock) before anything else.
	 *    z_prep_c may have reset these registers.  Keep HOSC
	 *    24 MHz to match the 115200 baud divisor used below.
	 */
	val = sys_read32(base + APB1_CLK_REG);
	val &= ~(GENMASK(25, 24) | GENMASK(9, 8) | GENMASK(4, 0));
	/* CLK_SRC_SEL = 00 (HOSC), M=1, N=1 → 24 MHz */
	sys_write32(val, base + APB1_CLK_REG);

	/*
	 * 1. Enable all PLLs per manual §3.3.4.4:
	 *    a) Write PLL_EN(31), LDO_EN(30) and LOCK_ENABLE(29).
	 *    b) Wait for LOCK(28) to become 1.
	 *    c) Delay 20 μs.
	 *
	 *    PLLs that are already running (boot ROM) are re-enabled
	 *    idempotently; the lock-wait re-validates their stability.
	 */
	static const uint32_t pll_regs[] = {
		PLL_CPUX_REG, PLL_DDR0_REG, PLL_PERIPH0_REG,
		PLL_VIDEO0_REG, PLL_VIDEO1_REG, PLL_VE_REG,
		PLL_AUDIO0_REG, PLL_AUDIO1_REG,
	};
	static const uint32_t pll_lock_timeout = 200000; /* ~10ms @ 24MHz */

	for (i = 0; i < ARRAY_SIZE(pll_regs); i++) {
		uint32_t timeout;

		val = sys_read32(base + pll_regs[i]);
		val |= BIT(31) | BIT(30) | BIT(29);
		sys_write32(val, base + pll_regs[i]);

		/* Wait for PLL to lock */
		timeout = pll_lock_timeout;
		while (!(sys_read32(base + pll_regs[i]) & BIT(28))) {
			if (--timeout == 0) {
				LOG_WRN("PLL 0x%03x lock timeout", pll_regs[i]);
				break;
			}
		}

		/* Per §3.3.4.4 step 5: delay 20 μs after lock */
		for (volatile int d = 0; d < 500; d++) {
			__asm__ volatile("nop");
		}
	}


	/* 2. Force PLL_CPUX output divider M (bits 1:0) to 0.
	 *    Per manual §3.3.6.1 the M factor is "only for testing".
	 */
	val = sys_read32(base + PLL_CPUX_REG);
	val &= ~GENMASK(1, 0);
	sys_write32(val, base + PLL_CPUX_REG);

	/* 3. Force the output divider of video PLLs (bit 0) to 0.
	 *    Per manual §3.3.6.4/§3.3.6.5 the D factor is "only for testing".
	 */
	static const uint32_t pll_video_regs[] = {
		PLL_VIDEO0_REG, PLL_VIDEO1_REG,
	};

	for (i = 0; i < ARRAY_SIZE(pll_video_regs); i++) {
		val = sys_read32(base + pll_video_regs[i]);
		val &= ~BIT(0);
		sys_write32(val, base + pll_video_regs[i]);
	}

	/* 4. Enforce m1 = 0 (bit 1), m0 = 0 (bit 0) for PLL_AUDIO0.
	 *    Per manual §3.3.6.7 these are pre/post dividers; the Linux
	 *    driver ignores them and forces to 0 for the standard
	 *    M=1 (m0+1=1), M1=1 (m1+1=1) behaviour.
	 */
	val = sys_read32(base + PLL_AUDIO0_REG);
	val &= ~(BIT(1) | BIT(0));
	sys_write32(val, base + PLL_AUDIO0_REG);

	/*
	 * 5. PSI_CLK / APB0_CLK / APB1_CLK are left at their reset
	 *    defaults (HOSC 24 MHz, div-by-1).  This matches the
	 *    clock-frequency values in the DTS.  To increase bus
	 *    speeds, switch the clock source to PLL_PERI(1X) or
	 *    PSI_CLK and adjust the dividers here.
	 */

	/* 5b. USB OHCI 48 MHz reference clock source selection.
	 *     Registers 0xa70 (OHCI0) and 0xa74 (OHCI1) share their
	 *     address space with the PHY reset (bit 30) and the OHCI
	 *     clock gate (bit 31).  Here we only program the source
	 *     mux bits[25:24] = 01 → HOSC/2 = 12 MHz (standard OHCI
	 *     reference).  Bit 31 (gate) is enabled by the gate loop
	 *     below via sys_set_bit(); bit 30 (PHY reset) is left
	 *     untouched for the PHY driver's on-demand management.
	 */
	{
		static const uint32_t ohci_clk_regs[] = { 0xa70, 0xa74 };
		for (i = 0; i < ARRAY_SIZE(ohci_clk_regs); i++) {
			val = sys_read32(base + ohci_clk_regs[i]);
			val &= ~GENMASK(25, 24);   /* clear mux */
			val |= (1U << 24);          /* HOSC/2 = 12 MHz */
			sys_write32(val, base + ohci_clk_regs[i]);
		}
	}

	/* 6. Enable all bus clocks and de-assert all resets.
	 *    Reset bits are active-low: 1 = de-assert, 0 = assert.
	 *    Gate bits  are active-high: 1 = pass,    0 = mask.
	 *    Both use sys_set_bit to write 1.
	 */
	for (i = 0; i < ARRAY_SIZE(d1_gates); i++)
		sys_set_bit(base + d1_gates[i].offset, d1_gates[i].bit);
	for (i = 0; i < ARRAY_SIZE(d1_resets); i++)
		sys_set_bit(base + d1_resets[i].offset, d1_resets[i].bit);

	/* 7. Read back the CPU clock configuration and report the frequency. */
	{
		static const char *const src_name[] = {
			"HOSC(24M)", "LOSC(32k)", "IOSC(~16M)", "PLL_CPUX",
			"PLL_PERIPH0(1X)", "PLL_PERIPH0(2X)", "PLL_PERIPH0(800M)",
		};
		uint32_t src = (sys_read32(base + CPUX_CLK_REG) >> CPUX_SRC_SHIFT) &
			       (BIT(CPUX_SRC_WIDTH) - 1);
		uint32_t cpux_rate = d1_pll_cpux_rate(base);
		uint32_t cpu_rate = d1_cpu_rate(base);
		const char *name = (src < ARRAY_SIZE(src_name)) ? src_name[src]
								: "unknown";

		LOG_INF("CPU clk: src=%s(%u) PLL_CPUX=%u Hz, CPU=%u Hz (%u.%03u MHz)",
			name, src, cpux_rate, cpu_rate,
			cpu_rate / 1000000U, (cpu_rate / 1000U) % 1000U);
	}

	LOG_DBG("D1/T113-S3 CCU ready");
	return 0;
}

/*
 * MP-style module clocks (source mux + linear M divider + power-of-2 P
 * divider).  Register layout (T113 manual §SMHCx_CLK_REG / §SPIx_CLK_REG):
 *
 *   [26:24] CLK_SRC_SEL — source mux
 *   [ 9: 8] FACTOR_N    — power-of-2 divider, div = 1 << N  (Linux "P")
 *   [ 3: 0] FACTOR_M    — linear divider,    div = M + 1
 *   [    31] CLK_GATING — managed by the .on/.off gate API
 *
 * Output rate = source / (M + 1) / (1 << N).
 *
 * NOTE: Linux models the MMC clocks with an extra CCU fixed /2 post-divider
 * (CCU_FEATURE_FIXED_POSTDIV) because its MMC driver clocks the controller
 * directly from this node.  This Zephyr SDHC driver instead feeds the module
 * clock into the controller's CLKCR divider and expects the CCU to output the
 * configured frequency verbatim (manual formula, no /2).  The DTS requests
 * 24 MHz and the default reset state (HOSC, M=0, N=0) must yield 24 MHz, so
 * the /2 is intentionally NOT applied here.
 */

/* PLL_PERIPH0 standard output frequencies (T113 manual §3.3.6.2). */
#define PLL_PERIPH0_1X_HZ	600000000U	/* PLL_PERI(1X)  */
#define PLL_PERIPH0_2X_HZ	1200000000U	/* PLL_PERI(2X)  */

struct d1_mp_clock {
	uint32_t id;			/* clock-control subsystem id */
	uint32_t offset;		/* CCU register offset */
	uint8_t m_shift;		/* FACTOR_M shift (linear, div = M + 1) */
	uint8_t m_width;
	uint8_t p_shift;		/* FACTOR_N shift (power-of-2, div = 1<<N) */
	uint8_t p_width;
	uint8_t mux_shift;		/* CLK_SRC_SEL shift */
	uint8_t mux_width;
	const uint32_t *sources;	/* parent rate per mux value */
};

/*
 * MP-clock mux source frequency tables, indexed by CLK_SRC_SEL.
 * A 0 entry marks a reserved or variable source that the rate-search
 * loop skips.
 */

/* SMHC (MMC): 0=HOSC, 1=PLL_PERI(1X), 2=PLL_PERI(2X), 3=PLL_AUDIO1/2. */
static const uint32_t d1_mmc_sources[] = {
	[0] = HOSC_HZ,
	[1] = PLL_PERIPH0_1X_HZ,
	[2] = PLL_PERIPH0_2X_HZ,
	[3] = 0,			/* PLL_AUDIO1_DIV2 — variable, unused */
};

/* SPI: 0=HOSC, 1=PLL_PERI(1X), 2=PLL_PERI(2X), 3=PLL_AUDIO1/2, 4=PLL_AUDIO1/5. */
static const uint32_t d1_spi_sources[] = {
	[0] = HOSC_HZ,
	[1] = PLL_PERIPH0_1X_HZ,
	[2] = PLL_PERIPH0_2X_HZ,
	[3] = 0,			/* PLL_AUDIO1_DIV2 */
	[4] = 0,			/* PLL_AUDIO1_DIV5 */
};

static const struct d1_mp_clock d1_mp_clocks[] = {
	/* SMHC module clocks — SMHCx_CLK_REG (0x0830/4/8) */
	{ D1_CLK_MMC0, 0x830, 0, 4, 8, 2, 24, 3, d1_mmc_sources },
	{ D1_CLK_MMC1, 0x834, 0, 4, 8, 2, 24, 3, d1_mmc_sources },
	{ D1_CLK_MMC2, 0x838, 0, 4, 8, 2, 24, 3, d1_mmc_sources },
	/* SPI module clocks — SPIx_CLK_REG (0x0940/4) */
	{ D1_CLK_SPI0, 0x940, 0, 4, 8, 2, 24, 3, d1_spi_sources },
	{ D1_CLK_SPI1, 0x944, 0, 4, 8, 2, 24, 3, d1_spi_sources },
};

static const struct d1_mp_clock *find_mp_clock(uint32_t id)
{
	for (int i = 0; i < ARRAY_SIZE(d1_mp_clocks); i++) {
		if (d1_mp_clocks[i].id == id) {
			return &d1_mp_clocks[i];
		}
	}
	return NULL;
}

static int d1_clock_get_rate(const struct device *dev,
			     clock_control_subsys_t sys,
			     uint32_t *rate)
{
	const struct d1_mp_clock *mp = find_mp_clock((uint32_t)(uintptr_t)sys);
	uintptr_t base = DT_INST_REG_ADDR(0);
	uint32_t reg, src_idx, m, p, parent, div;

	if (mp == NULL || rate == NULL) {
		return -ENOTSUP;
	}

	reg = sys_read32(base + mp->offset);
	src_idx = (reg >> mp->mux_shift) & ((1U << mp->mux_width) - 1);
	m = (reg >> mp->m_shift) & ((1U << mp->m_width) - 1);
	p = (reg >> mp->p_shift) & ((1U << mp->p_width) - 1);

	parent = mp->sources[src_idx];
	if (src_idx >= (1U << mp->mux_width) || parent == 0) {
		/* Reserved/variable source — cannot report a rate. */
		*rate = 0;
		return -EINVAL;
	}

	div = (m + 1U) * (1U << p);
	*rate = (uint32_t)((uint64_t)parent / div);
	return 0;
}

static int d1_clock_set_rate(const struct device *dev,
			     clock_control_subsys_t sys,
			     clock_control_subsys_rate_t rate)
{
	uint32_t target = (uint32_t)(uintptr_t)rate;
	const struct d1_mp_clock *mp = find_mp_clock((uint32_t)(uintptr_t)sys);
	uintptr_t base = DT_INST_REG_ADDR(0);
	uint32_t max_src, max_m, max_p;
	uint32_t best_diff = UINT32_MAX;
	uint32_t best_src = 0, best_m = 0, best_p = 0;

	if (mp == NULL || target == 0) {
		return -ENOTSUP;
	}

	max_src = (1U << mp->mux_width);
	max_m = (1U << mp->m_width);	/* M field: 0..max_m-1, div = M + 1 */
	max_p = (1U << mp->p_width);	/* N field: 0..max_p-1, div = 1<<N */

	/*
	 * Brute-force search for the (source, M, N) producing the highest
	 * rate that does not exceed the target — matches Linux
	 * ccu_mp_find_best "best ≤ rate" selection.  Sources below the
	 * target (cannot divide up) and reserved (0 Hz) sources are skipped.
	 */
	for (uint32_t s = 0; s < max_src; s++) {
		uint32_t parent = mp->sources[s];

		if (parent == 0 || parent < target) {
			continue;
		}
		for (uint32_t p = 0; p < max_p; p++) {
			for (uint32_t m = 0; m < max_m; m++) {
				uint32_t div = (m + 1U) * (1U << p);
				uint32_t actual = parent / div;

				if (actual > target) {
					continue;
				}
				if (target - actual < best_diff) {
					best_diff = target - actual;
					best_src = s;
					best_m = m;
					best_p = p;
					if (best_diff == 0) {
						goto found;
					}
				}
			}
		}
	}

	if (best_diff == UINT32_MAX) {
		return -EINVAL;
	}

found:
	{
		uint32_t reg = sys_read32(base + mp->offset);
		uint32_t m_mask = GENMASK(mp->m_shift + mp->m_width - 1,
					  mp->m_shift);
		uint32_t p_mask = GENMASK(mp->p_shift + mp->p_width - 1,
					  mp->p_shift);
		uint32_t mux_mask = GENMASK(mp->mux_shift + mp->mux_width - 1,
					    mp->mux_shift);

		/* Clear M/N/mux, preserve all other bits (incl. gate bit 31). */
		reg &= ~(m_mask | p_mask | mux_mask);
		reg |= (best_m << mp->m_shift);
		reg |= (best_p << mp->p_shift);
		reg |= (best_src << mp->mux_shift);
		sys_write32(reg, base + mp->offset);
	}

	LOG_DBG("set_rate id=%u: src=%u M=%u N=%u -> %u Hz (target %u)",
		(uint32_t)(uintptr_t)sys, best_src, best_m, best_p,
		mp->sources[best_src] / ((best_m + 1U) * (1U << best_p)),
		target);

	return 0;
}

static const struct clock_control_driver_api d1_ccu_api = {
	.on       = d1_clock_on,
	.off      = d1_clock_off,
	.get_rate = d1_clock_get_rate,
	.set_rate = d1_clock_set_rate,
};

DEVICE_DT_INST_DEFINE(0, d1_ccu_init, NULL,
		      NULL, NULL,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &d1_ccu_api);
