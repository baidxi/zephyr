/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file sdhc_sunxi.c
 * @brief Allwinner Sunxi MMC/SDHC driver for Zephyr RTOS
 *
 * Supports MMC/SD/SDIO cards with 1/4-bit bus width and PIO data transfer.
 * Based on Allwinner V3s User Manual V1.0 Chapter 5 and
 */

#define DT_DRV_COMPAT allwinner_sunxi_mmc

#include <zephyr/devicetree.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/cache.h>
#include <zephyr/sys/barrier.h>

#include "sunxi_mmc_regs.h"

LOG_MODULE_REGISTER(sdhc_sunxi, CONFIG_SDHC_LOG_LEVEL);

/* Maximum number of retries for soft reset */
#define SUNXI_MMC_RESET_RETRIES		1000
/* Timeout for polling operations in microseconds */
#define SUNXI_MMC_POLL_TIMEOUT_US	100000

struct sdhc_sunxi_config {
	DEVICE_MMIO_ROM;
	uint32_t bus_width;
	uint32_t max_bus_freq;
	uint32_t min_bus_freq;
	uint32_t src_clock_freq;
	const struct device *clk_dev;
	clock_control_subsys_t clk_id_bus;
	clock_control_subsys_t clk_id_mmc;
	clock_control_subsys_t clk_id_output;
	clock_control_subsys_t clk_id_sample;
	struct reset_dt_spec reset;
	struct gpio_dt_spec cd_gpio;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
};

struct sdhc_sunxi_data {
	DEVICE_MMIO_RAM;
	struct sdhc_host_props props;
	struct sdhc_io host_io;
	sdhc_interrupt_cb_t irq_cb;
	void *irq_user_data;
	uint32_t irq_sources;
	struct k_sem cmd_sem;
	struct k_sem xfer_sem;
	uint32_t src_clk_hz;
	volatile uint32_t last_rint;	/* Accumulated RINT from ISR */
	volatile uint32_t last_idst;	/* Accumulated IDST from ISR */

#ifdef CONFIG_SDHC_SUNXI_DMA
	/* IDMAC DMA state */
	bool use_dma;
	struct sunxi_idma_des *des_pool;    /* Descriptor pool (cache-aligned) */
	uint8_t *dma_buf;                    /* DMA bounce buffer */
	size_t dma_buf_size;                 /* DMA buffer size in bytes */
#endif
};

static inline uint32_t sunxi_mmc_read(const struct device *dev, uint32_t offset)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + offset);
}

static inline void sunxi_mmc_write(const struct device *dev, uint32_t offset,
				   uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + offset);
}

/*
 * Wait for specific bits in a register with timeout
 */
static int sunxi_mmc_wait_reg(const struct device *dev, uint32_t offset,
			      uint32_t mask, uint32_t expected, int timeout_us)
{
	while (timeout_us > 0) {
		uint32_t val = sunxi_mmc_read(dev, offset);

		if ((val & mask) == expected) {
			return 0;
		}
		k_busy_wait(1);
		timeout_us--;
	}
	return -ETIMEDOUT;
}

static int sunxi_mmc_check_errors(const struct device *dev, uint32_t rint)
{
	if (rint & SUNXI_MMC_RINT_RESP_TIMEOUT) {
		return -ETIMEDOUT;
	}
	if (rint & SUNXI_MMC_RINT_DATA_TIMEOUT) {
		return -ETIMEDOUT;
	}
	if (rint & SUNXI_MMC_RINT_RESP_CRC_ERROR) {
		return -EILSEQ;
	}
	if (rint & SUNXI_MMC_RINT_DATA_CRC_ERROR) {
		return -EILSEQ;
	}
	if (rint & SUNXI_MMC_RINT_RESP_ERROR) {
		return -EIO;
	}
	if (rint & SUNXI_MMC_RINT_FIFO_RUN_ERROR) {
		return -EIO;
	}
	if (rint & SUNXI_MMC_RINT_HARDWARE_LOCKED) {
		return -EIO;
	}
	if (rint & SUNXI_MMC_RINT_START_BIT_ERROR) {
		return -EIO;
	}
	if (rint & SUNXI_MMC_RINT_END_BIT_ERROR) {
		return -EIO;
	}
	return 0;
}

static void sdhc_sunxi_isr(const struct device *dev)
{
	struct sdhc_sunxi_data *data = dev->data;
	uint32_t rint;
	uint32_t idst;

	rint = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, rint));
	idst = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, idst));

	data->last_rint |= rint;
	data->last_idst |= idst;

	/* Clear all interrupts by writing back 1s */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, rint), rint);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, idst), idst);

	if (rint & SUNXI_MMC_RINT_COMMAND_DONE) {
		k_sem_give(&data->cmd_sem);
	}

	if (rint & SUNXI_MMC_RINT_DATA_OVER) {
		k_sem_give(&data->xfer_sem);
	}

	if (rint & SUNXI_MMC_RINT_AUTO_COMMAND_DONE) {
		k_sem_give(&data->xfer_sem);
	}

	if (rint & (SUNXI_MMC_RINT_RESP_ERROR | SUNXI_MMC_RINT_RESP_CRC_ERROR |
		    SUNXI_MMC_RINT_RESP_TIMEOUT)) {
		k_sem_give(&data->cmd_sem);
	}

	if (rint & (SUNXI_MMC_RINT_DATA_CRC_ERROR | SUNXI_MMC_RINT_DATA_TIMEOUT |
		    SUNXI_MMC_RINT_FIFO_RUN_ERROR)) {
		k_sem_give(&data->xfer_sem);
	}

	/* DMA receive complete — also signals transfer done */
	if (idst & SUNXI_MMC_IDST_RECV_INT) {
		k_sem_give(&data->xfer_sem);
	}
}

static uint32_t sunxi_mmc_response_flags(uint32_t response_type)
{
	uint32_t flags = 0;

	switch (response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		flags = SUNXI_MMC_CMD_RESP_EXP | SUNXI_MMC_CMD_CHK_RESP_CRC;
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		flags = SUNXI_MMC_CMD_RESP_EXP | SUNXI_MMC_CMD_CHK_RESP_CRC |
			SUNXI_MMC_CMD_WAIT_PRE_OVER;
		break;
	case SD_RSP_TYPE_R2:
		flags = SUNXI_MMC_CMD_RESP_EXP | SUNXI_MMC_CMD_LONG_RESP |
			SUNXI_MMC_CMD_CHK_RESP_CRC;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		flags = SUNXI_MMC_CMD_RESP_EXP;
		break;
	default:
		break;
	}

	return flags;
}

static int sdhc_sunxi_reset(const struct device *dev)
{
	uint32_t gctrl_val;
	uint32_t clkcr_before, clkcr_after;
	int ret;

	/* Diagnostic: log CLKCR before soft reset */
	clkcr_before = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, clkcr));
	LOG_WRN("sdhc_sunxi_reset() called: CLKCR before=0x%08x", clkcr_before);

	/* Set soft reset, FIFO reset, and DMA reset bits */
	gctrl_val = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, gctrl));
	gctrl_val |= SUNXI_MMC_GCTRL_HARDWARE_RESET;
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, gctrl), gctrl_val);

	/* Wait for reset to complete (bits self-clear) */
	ret = sunxi_mmc_wait_reg(dev, offsetof(struct sunxi_mmc_regs, gctrl),
				 SUNXI_MMC_GCTRL_HARDWARE_RESET,
				 0, SUNXI_MMC_POLL_TIMEOUT_US);
	if (ret) {
		LOG_ERR("Soft reset timed out");
		return -ETIMEDOUT;
	}

	/* Diagnostic: log CLKCR after soft reset */
	clkcr_after = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, clkcr));
	LOG_WRN("sdhc_sunxi_reset() done: CLKCR after=0x%08x (clock %s)",
		clkcr_after, (clkcr_after & BIT(16)) ? "enabled" : "DISABLED");

	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, ftrglevel),
			0x20070008);

	/* Maximum timeout value (data and response) */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, timeout),
			0xFFFFFFFF);

	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, rint), 0xFFFFFFFF);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, dbgc), 0xdeb);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, funcsel),
			SUNXI_MMC_FUNS_CEATA_ON);

	/*
	 * Enable interrupts and configure FIFO access mode.
	 *
	 * V3s GCTRL bit31 (FIFO_AC_MOD):
	 *   0 = FIFO connected to DMA bus (data goes to IDMAC)
	 *   1 = FIFO connected to AHB bus (data accessible via FIFO register)
	 *
	 * For PIO mode: must set FIFO_AC_MOD=1 so CPU can read FIFO register.
	 * For DMA mode: FIFO_AC_MOD=0 (default), DMA_ENB set per-transfer.
	 */
	gctrl_val = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, gctrl));
	gctrl_val |= SUNXI_MMC_GCTRL_INT_ENB;
	gctrl_val &= ~SUNXI_MMC_GCTRL_ACCESS_DONE_DIRECT;

#ifdef CONFIG_SDHC_SUNXI_DMA
	/* DMA mode: leave FIFO_AC_MOD=0 (DMA bus), DMA_ENB per-transfer */
	gctrl_val &= ~SUNXI_MMC_GCTRL_DMA_ENB;
#else
	/* PIO mode: FIFO_AC_MOD=1 for AHB bus access */
	gctrl_val |= SUNXI_MMC_GCTRL_FIFO_AC_MOD;
	gctrl_val &= ~SUNXI_MMC_GCTRL_DMA_ENB;
#endif
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, gctrl), gctrl_val);

	LOG_INF("GCTRL after setup: 0x%08x (%s mode)", gctrl_val,
		IS_ENABLED(CONFIG_SDHC_SUNXI_DMA) ? "DMA" : "PIO");

	/* Enable all interrupts we care about */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, imask),
			SUNXI_MMC_RINT_COMMAND_DONE |
			SUNXI_MMC_RINT_DATA_OVER |
			SUNXI_MMC_RINT_RESP_ERROR |
			SUNXI_MMC_RINT_RESP_CRC_ERROR |
			SUNXI_MMC_RINT_RESP_TIMEOUT |
			SUNXI_MMC_RINT_DATA_CRC_ERROR |
			SUNXI_MMC_RINT_DATA_TIMEOUT |
			SUNXI_MMC_RINT_FIFO_RUN_ERROR |
			SUNXI_MMC_RINT_HARDWARE_LOCKED |
			SUNXI_MMC_RINT_START_BIT_ERROR |
			SUNXI_MMC_RINT_END_BIT_ERROR);

	/* Clear any pending interrupts */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, rint), 0xFFFFFFFF);

	/* Disable and reset IDMAC.
	 * Unconditionally clear IDMAC interrupts to prevent
	 * stale IDMAC state from blocking MMC interrupts.
	 */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, idie), 0);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, idst), 0xFFFFFFFF);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, dmac),
			SUNXI_MMC_DMAC_SOFT_RST);

	return 0;
}

/*
 * Send clock update command.
 * After any write to CLKCR, the MMC controller must be told to latch the
 * new settings via UPCLK_ONLY.
 *
 * NOTE: On some Allwinner SoCs (including V3s), the CIU may not process
 * the first UPCLK_ONLY after init.  The recovery path does a CCU reset
 * + soft reset to re-initialize the CIU, which resolves this issue.
 */
static int sunxi_mmc_update_clk(const struct device *dev)
{
	uint32_t cmd;
	uint32_t rint_val;
	int ret;

	cmd = SUNXI_MMC_CMD_START | SUNXI_MMC_CMD_UPCLK_ONLY;

	LOG_DBG("update_clk: cmd=0x%08x", cmd);

	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, cmd), cmd);

	/*
	 * Wait for START bit to clear (hardware processes the clock update).
	 * Without WAIT_PRE_OVER, the CIU processes UPCLK_ONLY almost
	 * instantly (< 1ms) when functional.  Use 50ms timeout; if the
	 * CIU doesn't respond, it needs a CCU reset to recover.
	 */
	ret = sunxi_mmc_wait_reg(dev,
		offsetof(struct sunxi_mmc_regs, cmd),
		SUNXI_MMC_CMD_START, 0, 50000);
	if (ret) {
		/*
		 * CIU is stuck.  Recovery: CCU hardware reset + soft reset
		 * to fully reinitialize the CIU, then restore CLKCR and
		 * retry the UPCLK_ONLY.
		 */
		const struct sdhc_sunxi_config *cfg = dev->config;
		uint32_t saved_clkcr = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, clkcr));

		LOG_WRN("update_clk: START stuck (clkcr=0x%08x), "
			"recovering via CCU + soft reset", saved_clkcr);

		/* Hardware reset via CCU to fully clear CIU state */
		if (cfg->reset.dev) {
			reset_line_toggle(cfg->reset.dev, cfg->reset.id);
		}

		/* Re-enable module clocks after CCU reset */
		if (cfg->clk_dev && device_is_ready(cfg->clk_dev)) {
			clock_control_on(cfg->clk_dev, cfg->clk_id_bus);
			clock_control_on(cfg->clk_dev, cfg->clk_id_mmc);
			clock_control_on(cfg->clk_dev, cfg->clk_id_output);
			clock_control_on(cfg->clk_dev, cfg->clk_id_sample);
		}

		/* Wait for clocks to stabilize */
		k_busy_wait(100);

		/* Soft reset + re-initialize */
		ret = sdhc_sunxi_reset(dev);
		if (ret) {
			LOG_ERR("update_clk: reset failed: %d", ret);
			return -ETIMEDOUT;
		}

		/* Restore CLKCR (hardware reset clears it to 0) */
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, clkcr),
			saved_clkcr);

		/* Retry UPCLK_ONLY */
		cmd = SUNXI_MMC_CMD_START | SUNXI_MMC_CMD_UPCLK_ONLY;
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, cmd), cmd);

		ret = sunxi_mmc_wait_reg(dev,
			offsetof(struct sunxi_mmc_regs, cmd),
			SUNXI_MMC_CMD_START, 0, 50000);
		if (ret) {
			LOG_ERR("update_clk: still stuck after recovery");
			return -ETIMEDOUT;
		}
		LOG_DBG("update_clk: recovered successfully");
	}

	/*
	 * Clear IRQ status bits set by the clock update command.
	 * Preserve SDIO interrupt per Linux behavior.
	 */
	rint_val = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, rint));
	rint_val &= ~SUNXI_MMC_RINT_SDIO_INTERRUPT;
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, rint), rint_val);

	return 0;
}

static int sunxi_mmc_set_clock(const struct device *dev, uint32_t target_hz)
{
	struct sdhc_sunxi_data *data = dev->data;
	uint32_t clkcr;
	uint32_t div;
	int ret;

	if (target_hz == 0) {
		/* Disable card clock - matches Linux oclk_onoff(0) */
		clkcr = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, clkcr));
		clkcr &= ~(SUNXI_MMC_CLKCR_CLK_ENB | SUNXI_MMC_CLKCR_CLK_LOW_PWR);
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
		ret = sunxi_mmc_update_clk(dev);
		LOG_DBG("set_clock: disable -> clkcr=0x%08x ret=%d", clkcr, ret);
		return ret;
	}

	/* Calculate divider: card_clock = source_clock / (2 * (div + 1)) */
	if (target_hz >= data->src_clk_hz) {
		div = 0;
	} else {
		div = data->src_clk_hz / (2 * target_hz);
		if (div > 0) {
			div -= 1;
		}
		if (div > 0xFF) {
			div = 0xFF;
		}
	}

	LOG_DBG("set_clock: target=%u src=%u div=%u (actual=%u)",
		target_hz, data->src_clk_hz, div,
		data->src_clk_hz / (2 * (div + 1)));

	/* Step 1: Disable card clock and clear low-power mode.
	 * Matches Linux sunxi_mmc_oclk_onoff(0) — ALWAYS performed,
	 * even if clock is already disabled. This "primes" the CIU:
	 * the first update_clk after reset initializes internal CIU
	 * state so that subsequent clock-enable update_clk works.
	 * Skipping this (when clock appears already off) causes the
	 * next enable update_clk to hang (START bit never clears).
	 */
	clkcr = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, clkcr));
	clkcr &= ~(SUNXI_MMC_CLKCR_CLK_ENB | SUNXI_MMC_CLKCR_CLK_LOW_PWR);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
	LOG_DBG("set_clock: step1 disable clkcr=0x%08x", clkcr);
	ret = sunxi_mmc_update_clk(dev);
	if (ret) {
		LOG_ERR("set_clock: step1 (disable) FAILED: %d", ret);
		return ret;
	}

	/* Step 2: Set new divider (clock still disabled, no update_clk).
	 * Matches Linux: rval &= ~0xff; rval |= div - 1;
	 */
	clkcr = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, clkcr));
	clkcr &= ~SUNXI_MMC_CLKCR_CCLK_DIV_MASK;
	clkcr |= (div & SUNXI_MMC_CLKCR_CCLK_DIV_MASK);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
	LOG_DBG("set_clock: step2 set_div clkcr=0x%08x", clkcr);

	/* Step 3: Enable card clock.
	 * Matches Linux sunxi_mmc_oclk_onoff(1).
	 */
	clkcr = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, clkcr));
	clkcr |= SUNXI_MMC_CLKCR_CLK_ENB;
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
	LOG_DBG("set_clock: step3 enable clkcr=0x%08x", clkcr);

	/*
	 * Give the card a short period to stabilize after clock enable.
	 * SD spec requires 74 clock cycles minimum for power-up.
	 * At 400kHz (worst case), that's ~185µs.  We use 1ms margin.
	 * Note: WAIT_PRE_OVER is no longer used during initial setup
	 * (data FSM is idle), so DAT0 state doesn't block update_clk.
	 */
	k_busy_wait(1000);

	ret = sunxi_mmc_update_clk(dev);
	if (ret) {
		LOG_ERR("set_clock: step3 (enable) FAILED: %d", ret);
		return ret;
	}

	/* Verify CLKCR was applied by reading it back */
	#if CONFIG_SDHC_LOG_LEVEL >= LOG_LEVEL_DBG
	{
		uint32_t clkcr_rb = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, clkcr));
		LOG_DBG("set_clock: CLKCR read-back=0x%08x (CLK_ENB=%d div=%ld)",
			clkcr_rb,
			!!(clkcr_rb & SUNXI_MMC_CLKCR_CLK_ENB),
			clkcr_rb & SUNXI_MMC_CLKCR_CCLK_DIV_MASK);
	}
	#endif
	/*
	 * Diagnostic (Fix6): Poll STATUS register after clock enable to
	 * determine whether the card successfully exits power-up busy.
	 * After receiving clock, SD cards drive DAT0 LOW (busy) during
	 * internal initialization, then release it HIGH when ready.
	 * We poll every 10ms for up to 2 seconds. This tells us:
	 *  - If DAT0 never went LOW: clock is not reaching the card
	 *  - If DAT0 goes LOW then HIGH: card init completed (time logged)
	 *  - If DAT0 stays LOW forever: card or clock problem
	 * Only done for initial identification clock (400kHz) to avoid
	 * slowing down subsequent clock changes.
	 */
	if (target_hz <= 400000) {
		uint32_t poll_status;
		int poll_ms;
		bool dat0_seen_high = false;
		uintptr_t pio_base =
			DT_REG_ADDR(DT_NODELABEL(pinctrl));
		uint32_t port_f_off = 5 * 0x24; /* Port F offset */

		/* Read initial STATUS right after update_clk */
		poll_status = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, status));
		LOG_DBG("set_clock: STATUS right after clk enable: "
			"0x%08x CP=%d DB=%d",
			poll_status,
			!!(poll_status & SUNXI_MMC_STATUS_CARD_PRESENT),
			!!(poll_status & SUNXI_MMC_STATUS_CARD_DATA_BUSY));

		/*
		 * Read PIO Port F pin state directly to verify actual
		 * pin levels. Port F data register at offset 0x10.
		 * PF0=DAT0, PF1=CLK, PF2=CMD, PF3-5=DAT1-3.
		 */
		{
			uint32_t pf_data = sys_read32(pio_base + port_f_off + 0x10);
			uint32_t pf_cfg0 = sys_read32(pio_base + port_f_off);
			uint32_t pf_pud = sys_read32(pio_base + port_f_off + 0x1C);

			LOG_DBG("set_clock: PIO PortF pins: "
				"DATA=0x%08x (PF0/CLK/CMDDAT=%c%c%c%c%c%c) "
				"CFG0=0x%08x PUD=0x%08x",
				pf_data,
				(pf_data & BIT(0)) ? 'H' : 'L', /* DAT0 */
				(pf_data & BIT(1)) ? 'H' : 'L', /* CLK */
				(pf_data & BIT(2)) ? 'H' : 'L', /* CMD */
				(pf_data & BIT(3)) ? 'H' : 'L', /* DAT1 */
				(pf_data & BIT(4)) ? 'H' : 'L', /* DAT2 */
				(pf_data & BIT(5)) ? 'H' : 'L', /* DAT3 */
				pf_cfg0, pf_pud);
		}

		for (poll_ms = 0; poll_ms < 2000; poll_ms += 10) {
			k_busy_wait(10000); /* 10ms */

			poll_status = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, status));
			bool cp = !!(poll_status & SUNXI_MMC_STATUS_CARD_PRESENT);
			bool db = !!(poll_status & SUNXI_MMC_STATUS_CARD_DATA_BUSY);

			if (cp && !db) {
				/* DAT0 is HIGH — card released busy */
				if (!dat0_seen_high) {
					LOG_DBG("set_clock: DAT0 HIGH after %d ms "
						"(STATUS=0x%08x) — card ready!",
						poll_ms + 10, poll_status);
					dat0_seen_high = true;
				}
				break;
			}

			/* Log every 200ms to track state changes */
			if ((poll_ms % 200) == 0) {
				/* Also read PIO pin state for comparison */
				uint32_t pf_data = sys_read32(
					pio_base + port_f_off + 0x10);
				LOG_DBG("set_clock: poll +%dms: STATUS=0x%08x "
					"CP=%d DB=%d PIO=0x%08x (DAT0=%c CLK=%c)",
					poll_ms, poll_status, cp, db, pf_data,
					(pf_data & BIT(0)) ? 'H' : 'L',
					(pf_data & BIT(1)) ? 'H' : 'L');
			}
		}

		if (!dat0_seen_high) {
			/* Final PIO diagnostic */
			uint32_t pf_data = sys_read32(pio_base + port_f_off + 0x10);
			LOG_WRN("set_clock: DAT0 stayed LOW for 2 seconds! "
				"PIO PF_DATA=0x%08x (DAT0=%c CLK=%c CMD=%c)",
				pf_data,
				(pf_data & BIT(0)) ? 'H' : 'L',
				(pf_data & BIT(1)) ? 'H' : 'L',
				(pf_data & BIT(2)) ? 'H' : 'L');

			/*
			 * Disable clock and check if DAT0 goes back HIGH.
			 * If DAT0 goes HIGH after disabling clock → card WAS
			 * driving it LOW (card busy, clock IS reaching card).
			 * If DAT0 stays LOW → something else is driving DAT0
			 * LOW (pin conflict or controller issue).
			 */
			LOG_DBG("set_clock: Disabling clock to test DAT0...");
			clkcr = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, clkcr));
			clkcr &= ~(SUNXI_MMC_CLKCR_CLK_ENB |
				    SUNXI_MMC_CLKCR_CLK_LOW_PWR);
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
			sunxi_mmc_update_clk(dev);
			k_busy_wait(1000); /* 1ms settling */

			poll_status = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, status));
			if (poll_status & SUNXI_MMC_STATUS_CARD_PRESENT) {
				LOG_WRN("set_clock: DAT0 went HIGH after clock "
					"disable! (STATUS=0x%08x) → Card IS "
					"receiving clock (busy too long)",
					poll_status);
			} else {
				LOG_WRN("set_clock: DAT0 still LOW after clock "
					"disable! (STATUS=0x%08x) → DAT0 pin "
					"conflict or pull-up not working",
					poll_status);
			}

			/* Re-enable clock to continue */
			clkcr = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, clkcr));
			clkcr |= SUNXI_MMC_CLKCR_CLK_ENB;
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
			k_busy_wait(1000);
			sunxi_mmc_update_clk(dev);
		}
	}

	LOG_DBG("Set clock: target=%u Hz, source=%u Hz, div=%u",
		target_hz, data->src_clk_hz, div);

	return 0;
}

static int sunxi_mmc_set_bus_width(const struct device *dev,
				   enum sdhc_bus_width bus_width)
{
	switch (bus_width) {
	case SDHC_BUS_WIDTH1BIT:
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, width),
				SUNXI_MMC_WIDTH_1BIT);
		break;
	case SDHC_BUS_WIDTH4BIT:
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, width),
				SUNXI_MMC_WIDTH_4BIT);
		break;
	case SDHC_BUS_WIDTH8BIT:
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, width),
				SUNXI_MMC_WIDTH_8BIT);
		break;
	default:
		return -ENOTSUP;
	}

	LOG_DBG("Set bus width: %d", bus_width);
	return 0;
}

static int sdhc_sunxi_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhc_sunxi_data *data = dev->data;
	int ret;

	if (!ios) {
		return -EINVAL;
	}

	/* Configure clock rate */
	if (ios->clock != data->host_io.clock) {
		ret = sunxi_mmc_set_clock(dev, ios->clock);
		if (ret) {
			LOG_ERR("Failed to set clock to %u Hz", ios->clock);
			return ret;
		}
		data->host_io.clock = ios->clock;
	}

	/* Configure bus width */
	if (ios->bus_width && ios->bus_width != data->host_io.bus_width) {
		ret = sunxi_mmc_set_bus_width(dev, ios->bus_width);
		if (ret) {
			LOG_ERR("Failed to set bus width to %d", ios->bus_width);
			return ret;
		}
		data->host_io.bus_width = ios->bus_width;
	}

	/* Store bus mode */
	if (ios->bus_mode) {
		if (ios->bus_mode != SDHC_BUSMODE_PUSHPULL) {
			LOG_ERR("Only push-pull bus mode supported");
			return -ENOTSUP;
		}
		data->host_io.bus_mode = ios->bus_mode;
	}

	/* Store power mode */
	if (ios->power_mode) {
		data->host_io.power_mode = ios->power_mode;
	}

	/* Store timing */
	if (ios->timing) {
		data->host_io.timing = ios->timing;
	}

	/* Validate and apply signal voltage */
	if (ios->signal_voltage) {
		if (ios->signal_voltage != SD_VOL_3_3_V) {
			LOG_ERR("Only 3.3V signaling supported");
			return -ENOTSUP;
		}
		data->host_io.signal_voltage = ios->signal_voltage;
	}

	return 0;
}

static int sunxi_mmc_fifo_read(const struct device *dev, uint8_t *buf,
			       unsigned int total_bytes, int timeout_ms)
{
	unsigned int bytes_read = 0;
	int64_t end_time = k_uptime_get() + timeout_ms;

	while (bytes_read < total_bytes && k_uptime_get() < end_time) {
		uint32_t status = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, status));
		uint32_t rx_count;

		rx_count = (status & SUNXI_MMC_STATUS_RX_FIFO_CNT_MASK)
			   >> SUNXI_MMC_STATUS_RX_FIFO_CNT_SHIFT;

		while (rx_count > 0 && bytes_read < total_bytes) {
			uint32_t fifo_word = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, fifo));
			unsigned int copy = MIN(4, total_bytes - bytes_read);

			memcpy(buf + bytes_read, &fifo_word, copy);
			bytes_read += copy;

			status = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, status));
			rx_count = (status & SUNXI_MMC_STATUS_RX_FIFO_CNT_MASK)
				   >> SUNXI_MMC_STATUS_RX_FIFO_CNT_SHIFT;
		}

		/* Check for errors */
		uint32_t rint = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, rint));

		if (rint & SUNXI_MMC_RINT_ERR_MASK) {
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, rint), rint);
			return sunxi_mmc_check_errors(dev, rint);
		}

		/* Check for data over (transfer complete) */
		if (rint & SUNXI_MMC_RINT_DATA_OVER) {
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, rint),
				SUNXI_MMC_RINT_DATA_OVER);
			/* Read remaining FIFO data */
			status = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, status));
			rx_count = (status & SUNXI_MMC_STATUS_RX_FIFO_CNT_MASK)
				   >> SUNXI_MMC_STATUS_RX_FIFO_CNT_SHIFT;
			while (rx_count > 0 && bytes_read < total_bytes) {
				uint32_t fifo_word = sunxi_mmc_read(dev,
					offsetof(struct sunxi_mmc_regs, fifo));
				unsigned int copy = MIN(4,
					total_bytes - bytes_read);

				memcpy(buf + bytes_read, &fifo_word, copy);
				bytes_read += copy;
				status = sunxi_mmc_read(dev,
					offsetof(struct sunxi_mmc_regs, status));
				rx_count = (status &
					SUNXI_MMC_STATUS_RX_FIFO_CNT_MASK)
					>> SUNXI_MMC_STATUS_RX_FIFO_CNT_SHIFT;
			}
			break;
		}
	}

	return bytes_read;
}

static int sunxi_mmc_fifo_write(const struct device *dev, const uint8_t *buf,
				unsigned int total_bytes, int timeout_ms)
{
	unsigned int bytes_written = 0;
	int64_t end_time = k_uptime_get() + timeout_ms;

	while (bytes_written < total_bytes && k_uptime_get() < end_time) {
		uint32_t status = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, status));

		/* Write if FIFO is not full */
		if (!(status & SUNXI_MMC_STATUS_FIFO_FULL)) {
			uint32_t fifo_word = 0;
			unsigned int copy = MIN(4, total_bytes - bytes_written);

			memcpy(&fifo_word, buf + bytes_written, copy);
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, fifo), fifo_word);
			bytes_written += copy;
		}

		/* Check for errors */
		uint32_t rint = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, rint));

		if (rint & SUNXI_MMC_RINT_ERR_MASK) {
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, rint), rint);
			return sunxi_mmc_check_errors(dev, rint);
		}
	}

	return bytes_written;
}

#ifdef CONFIG_SDHC_SUNXI_DMA
static int sunxi_mmc_dma_init(const struct device *dev)
{
	struct sdhc_sunxi_data *data = dev->data;

	/* Allocate descriptor pool (cache-line aligned, one page = 256 descs) */
	data->des_pool = k_aligned_alloc(32, SUNXI_IDMAC_DESC_POOL_SIZE);
	if (!data->des_pool) {
		LOG_ERR("Failed to allocate IDMAC descriptor pool");
		return -ENOMEM;
	}
	memset(data->des_pool, 0, SUNXI_IDMAC_DESC_POOL_SIZE);

	/* Allocate DMA bounce buffer */
	data->dma_buf_size = CONFIG_SDHC_SUNXI_DMA_BUF_SIZE;
	data->dma_buf = k_aligned_alloc(32, data->dma_buf_size);
	if (!data->dma_buf) {
		LOG_ERR("Failed to allocate DMA bounce buffer (%zu bytes)",
			data->dma_buf_size);
		return -ENOMEM;
	}

	data->use_dma = true;

	LOG_INF("DMA initialized: des_pool=%p dma_buf=%p (%zu bytes)",
		data->des_pool, data->dma_buf, data->dma_buf_size);
	return 0;
}

static void sunxi_mmc_dma_prepare(const struct device *dev,
				   struct sdhc_data *data_req,
				   bool is_read)
{
	struct sdhc_sunxi_data *sdata = dev->data;
	struct sunxi_idma_des *des = sdata->des_pool;
	uint32_t total = data_req->blocks * data_req->block_size;
	uint32_t offset = 0;
	int i = 0;

	__ASSERT_NO_MSG(total <= sdata->dma_buf_size);

	/* Build descriptor chain — each descriptor covers up to 64KB */
	while (offset < total) {
		uint32_t chunk = MIN(total - offset, SUNXI_IDMAC_MAX_BUF_SIZE);

		des[i].config = SUNXI_IDMAC_DES0_CH |
				SUNXI_IDMAC_DES0_OWN |
				SUNXI_IDMAC_DES0_DIC;
		des[i].buf_size = (chunk == SUNXI_IDMAC_MAX_BUF_SIZE)
				  ? 0 : chunk;
		des[i].buf_addr_ptr1 = (uint32_t)(uintptr_t)(sdata->dma_buf + offset);
		des[i].buf_addr_ptr2 = (uint32_t)(uintptr_t)(
			(uint8_t *)sdata->des_pool +
			(i + 1) * sizeof(struct sunxi_idma_des));

		offset += chunk;
		i++;
	}

	/* First descriptor: set FD (First Descriptor) flag */
	des[0].config |= SUNXI_IDMAC_DES0_FD;

	/* Last descriptor: set LD + ER, clear DIC, null next pointer */
	des[i - 1].config |= SUNXI_IDMAC_DES0_LD | SUNXI_IDMAC_DES0_ER;
	des[i - 1].config &= ~SUNXI_IDMAC_DES0_DIC;
	des[i - 1].buf_addr_ptr2 = 0;

	/* Flush descriptors from D-cache to RAM so IDMAC can read them */
	sys_cache_data_flush_range(sdata->des_pool,
				   i * sizeof(struct sunxi_idma_des));
	barrier_dmem_fence_full();

	LOG_DBG("DMA prepare: %u bytes, %d descs, %s",
		total, i, is_read ? "read" : "write");
}

static void sunxi_mmc_dma_start(const struct device *dev, bool is_read)
{
	struct sdhc_sunxi_data *sdata = dev->data;
	uint32_t gctrl;

	/* Set descriptor list base address */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, dlba),
			(uint32_t)(uintptr_t)sdata->des_pool);

	/* Reset IDMAC */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, dmac),
			SUNXI_MMC_DMAC_SOFT_RST);

	/* Enable DMA in GCTRL */
	gctrl = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, gctrl));
	gctrl |= SUNXI_MMC_GCTRL_DMA_ENB;
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, gctrl), gctrl);

	/* Enable IDMAC receive interrupt for reads */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, idie),
			is_read ? SUNXI_MMC_IDST_RECV_INT : 0);

	/* Start IDMAC: fix burst + IDMA on */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, dmac),
			SUNXI_MMC_DMAC_FIX_BURST | SUNXI_MMC_DMAC_IDMA_ON);
}

static void sunxi_mmc_dma_stop(const struct device *dev)
{
	uint32_t gctrl;

	/* Stop IDMAC */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, dmac), 0);
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, idie), 0);

	/* Clear IDMAC status */
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, idst),
			0xFFFFFFFF);

	/* Disable DMA in GCTRL */
	gctrl = sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, gctrl));
	gctrl &= ~SUNXI_MMC_GCTRL_DMA_ENB;
	sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, gctrl), gctrl);
}

#endif /* CONFIG_SDHC_SUNXI_DMA */

static int sdhc_sunxi_request(const struct device *dev, struct sdhc_command *cmd,
			      struct sdhc_data *data_req)
{
	struct sdhc_sunxi_data *sdata = dev->data;
	uint32_t cmd_val;
	uint32_t response_type;
	int ret = 0;
	int attempts;

	if (!cmd) {
		return -EINVAL;
	}

	response_type = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;
	attempts = cmd->retries + 1;

	while (attempts-- > 0) {
		/* Clear pending interrupts FIRST to prevent ISR race.
		 * If we reset last_rint before clearing hardware RINT,
		 * the ISR could fire between the two operations and
		 * accumulate stale RINT bits into the freshly-zeroed
		 * last_rint.
		 */
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, rint),
				0xFFFFFFFF);

		/* Reset semaphores and saved RINT state */
		k_sem_reset(&sdata->cmd_sem);
		k_sem_reset(&sdata->xfer_sem);
		sdata->last_rint = 0;
		sdata->last_idst = 0;

		/* Configure block size and byte count for data transfers */
		if (data_req) {
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, blksz),
				data_req->block_size);
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, bytecnt),
				data_req->blocks * data_req->block_size);
		} else {
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, blksz), 0);
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, bytecnt), 0);
		}

		/* Check if controller is busy */
		if (sunxi_mmc_read(dev, offsetof(struct sunxi_mmc_regs, status))
		    & SUNXI_MMC_STATUS_DATA_FSM_BUSY) {
			ret = sunxi_mmc_wait_reg(dev,
				offsetof(struct sunxi_mmc_regs, status),
				SUNXI_MMC_STATUS_DATA_FSM_BUSY, 0,
				SUNXI_MMC_POLL_TIMEOUT_US);
			if (ret) {
				LOG_ERR("Controller busy timeout");
				continue;
			}
		}

		/*
		 * Diagnostic: log STATUS for CMD0/CMD8 to track DAT0 state.
		 * STATUS bit8 = CARD_PRESENT, bit9 = DATA_BUSY.
		 */
		if (cmd->opcode == SD_GO_IDLE_STATE || cmd->opcode == 8) {
			uint32_t pre_st = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, status));
			LOG_DBG("CMD%u pre: STATUS=0x%08x CP=%d DB=%d",
				cmd->opcode, pre_st,
				!!(pre_st & BIT(8)),
				!!(pre_st & BIT(9)));
		}

		/* Set command argument */
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, arg),
				cmd->arg);

		/* Build command value */
		cmd_val = cmd->opcode;
		cmd_val |= sunxi_mmc_response_flags(cmd->response_type);

		/*
		 * CMD0 (GO_IDLE_STATE) requires SEND_INIT_SEQUENCE to send
		 * 80 clock cycles before the command, per SD spec.
		 * Matches Linux: if (cmd->opcode == MMC_GO_IDLE_STATE)
		 *   cmd_val |= SDXC_SEND_INIT_SEQUENCE;
		 */
		if (cmd->opcode == SD_GO_IDLE_STATE) {
			cmd_val |= SUNXI_MMC_CMD_SEND_INIT_SEQ;
		}

		if (data_req) {
			cmd_val |= SUNXI_MMC_CMD_DATA_EXP;

			/* Determine direction */
			switch (cmd->opcode) {
			case SD_WRITE_SINGLE_BLOCK:
			case SD_WRITE_MULTIPLE_BLOCK:
				cmd_val |= SUNXI_MMC_CMD_WRITE;
				break;
			case SD_READ_SINGLE_BLOCK:
			case SD_READ_MULTIPLE_BLOCK:
			case SD_APP_SEND_SCR:
			case SD_APP_SEND_NUM_WRITTEN_BLK:
			case MMC_SEND_EXT_CSD:
				/* Read - WRITE bit clear */
				break;
			default:
				break;
			}

			/* Auto stop for multi-block transfers */
			if (data_req->blocks > 1) {
				cmd_val |= SUNXI_MMC_CMD_AUTO_STOP;
			}
		}

		/*
		 * Start DMA BEFORE sending command.
		 * Matches Linux: sunxi_mmc_start_dma() is called before
		 * writing CMD register.  IDMAC will begin transferring data
		 * once the SDHC command engine starts the data phase.
		 */
#ifdef CONFIG_SDHC_SUNXI_DMA
		if (data_req && sdata->use_dma) {
			bool is_read = !(cmd_val & SUNXI_MMC_CMD_WRITE);
			int total = data_req->blocks * data_req->block_size;

			if ((size_t)total <= sdata->dma_buf_size) {
				/* Write: copy data to DMA buffer and flush */
				if (!is_read) {
					memcpy(sdata->dma_buf,
					       data_req->data, total);
					sys_cache_data_flush_range(
						sdata->dma_buf, total);
				} else {
					/*
					 * Read: pre-invalidate DMA buffer
					 * to purge stale cache lines before
					 * IDMAC writes new data.  Without
					 * this, clean cache lines may not be
					 * properly invalidated after DMA.
					 */
					sys_cache_data_invd_range(
						sdata->dma_buf, total);
					barrier_dmem_fence_full();
				}

				sunxi_mmc_dma_prepare(dev, data_req, is_read);
				sunxi_mmc_dma_start(dev, is_read);
			} else {
				LOG_WRN("Data too large for DMA buffer "
					"(%d > %zu), falling back to PIO",
					total, sdata->dma_buf_size);
			}
		}
#endif

		/* Load command (set START bit to start) */
		cmd_val |= SUNXI_MMC_CMD_START;

		LOG_DBG("CMD%u: cmd_val=0x%08x arg=0x%08x",
			cmd->opcode, cmd_val, cmd->arg);

		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, cmd),
				cmd_val);

		/* Wait for command completion via interrupt */
		ret = k_sem_take(&sdata->cmd_sem, K_MSEC(cmd->timeout_ms));
		if (ret) {
			/*
			 * Interrupt not delivered. Fall back to polling RINT.
			 * Wait for COMMAND_DONE or ANY error bit to be set.
			 * Previous code used sunxi_mmc_wait_reg() with
			 * expected=COMMAND_DONE which would NEVER match
			 * if only error bits were set (RESP_TIMEOUT etc).
			 */
			bool poll_done = false;
			int poll_us = 0;
			const int poll_max_us = 50000;
			uint32_t rint_val = 0;

			while (poll_us < poll_max_us) {
				rint_val = sunxi_mmc_read(dev,
					offsetof(struct sunxi_mmc_regs, rint));
				/* Also check ISR-accumulated RINT, in case
				 * ISR fired and cleared hardware RINT.
				 */
				rint_val |= sdata->last_rint;
				if (rint_val & (SUNXI_MMC_RINT_COMMAND_DONE |
						SUNXI_MMC_RINT_ERR_MASK)) {
					poll_done = true;
					break;
				}
				k_busy_wait(10);
				poll_us += 10;
			}

			if (!poll_done) {
				rint_val = sunxi_mmc_read(dev,
					offsetof(struct sunxi_mmc_regs, rint));
				LOG_ERR("CMD%u poll timeout: rint=0x%08x",
					cmd->opcode, rint_val);
				ret = -ETIMEDOUT;
				continue;
			}

			/* Accumulate polled RINT (use |= to preserve any
			 * ISR-accumulated bits).
			 */
			sdata->last_rint |= rint_val;
			sunxi_mmc_write(dev,
				offsetof(struct sunxi_mmc_regs, rint),
				rint_val);
			LOG_DBG("CMD%u completed via polling: rint=0x%08x",
				cmd->opcode, rint_val);
			ret = 0;
		}

		/*
		 * Check for command errors using saved RINT.
		 * IMPORTANT: Do NOT re-read RINT from hardware here!
		 * The ISR has already cleared the hardware register.
		 * Use sdata->last_rint which accumulates the RINT value(s)
		 * from either ISR or polling path.
		 */
		{
			uint32_t rint = sdata->last_rint;

			if (rint & (SUNXI_MMC_RINT_RESP_ERROR |
				    SUNXI_MMC_RINT_RESP_CRC_ERROR |
				    SUNXI_MMC_RINT_RESP_TIMEOUT)) {
				ret = sunxi_mmc_check_errors(dev, rint);
				uint32_t err_status = sunxi_mmc_read(dev,
					offsetof(struct sunxi_mmc_regs, status));
				uint32_t err_resp0 = sunxi_mmc_read(dev,
					offsetof(struct sunxi_mmc_regs, resp0));
				LOG_ERR("CMD%u error: rint=0x%08x ret=%d "
					"status=0x%08x resp0=0x%08x",
					cmd->opcode, rint, ret,
					err_status, err_resp0);
				/* Clear any remaining RINT bits */
				sunxi_mmc_write(dev,
					offsetof(struct sunxi_mmc_regs, rint),
					rint);
				continue;
			}
		}

		LOG_DBG("CMD%u OK: rint=0x%08x resp0=0x%08x", cmd->opcode,
			sdata->last_rint,
			sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, resp0)));

		/* Extract response */
		switch (response_type) {
		case SD_RSP_TYPE_NONE:
			cmd->response[0] = 0;
			cmd->response[1] = 0;
			cmd->response[2] = 0;
			cmd->response[3] = 0;
			break;
		case SD_RSP_TYPE_R2:
			cmd->response[0] = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, resp0));
			cmd->response[1] = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, resp1));
			cmd->response[2] = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, resp2));
			cmd->response[3] = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, resp3));
			break;
		default:
			cmd->response[0] = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, resp0));
			cmd->response[1] = 0;
			cmd->response[2] = 0;
			cmd->response[3] = 0;
			break;
		}

		/* Handle data transfer if present */
		if (data_req) {
			bool is_read = !(cmd_val & SUNXI_MMC_CMD_WRITE);
			int total = data_req->blocks * data_req->block_size;

#ifdef CONFIG_SDHC_SUNXI_DMA
			if (sdata->use_dma && (size_t)total <= sdata->dma_buf_size) {
				/*
				 * DMA transfer path.
				 * DMA was already started before CMD send.
				 * Wait for transfer completion via xfer_sem
				 * (signaled by DATA_OVER ISR or IDST RECV_INT).
				 */
				ret = k_sem_take(&sdata->xfer_sem,
						K_MSEC(data_req->timeout_ms));

				/* Stop IDMAC engine */
				sunxi_mmc_dma_stop(dev);
	
				if (ret) {
					LOG_ERR("DMA transfer timeout");
					ret = -ETIMEDOUT;
					continue;
				}

				/* Check for data errors */
				if (sdata->last_rint & SUNXI_MMC_RINT_ERR_MASK) {
					ret = sunxi_mmc_check_errors(
						dev, sdata->last_rint);
					LOG_ERR("DMA data error: rint=0x%08x "
						"idst=0x%08x ret=%d",
						sdata->last_rint,
						sdata->last_idst, ret);
					continue;
				}

				/* Read: invalidate cache and copy from bounce buf */
				if (is_read) {
					sys_cache_data_invd_range(
						sdata->dma_buf, total);
					/*
						* Ensure cache invalidation
						* completes before CPU reads
						* the DMA buffer.
						*/
					barrier_dmem_fence_full();
					memcpy(data_req->data,
									sdata->dma_buf, total);
				}

				data_req->bytes_xfered = total;

				if (total <= 8) {
					uint8_t *d = data_req->data;
					LOG_DBG("DMA CMD%u data %d bytes: "
						"%02x %02x %02x %02x "
						"%02x %02x %02x %02x",
						cmd->opcode, total,
						d[0], d[1], d[2], d[3],
						d[4], d[5], d[6], d[7]);
				}
			} else
#endif
			{
				/* PIO transfer path (fallback or no DMA) */
				if (is_read) {
					int bytes = sunxi_mmc_fifo_read(dev,
						data_req->data, total,
						data_req->timeout_ms);
					data_req->bytes_xfered =
						(bytes > 0) ? bytes : 0;
					if (bytes < 0) {
						ret = bytes;
					} else if (total <= 8) {
						uint8_t *d = data_req->data;
						LOG_DBG("PIO CMD%u data read "
							"%d bytes: "
							"%02x %02x %02x %02x "
							"%02x %02x %02x %02x",
							cmd->opcode, total,
							d[0], d[1],
							d[2], d[3],
							d[4], d[5],
							d[6], d[7]);
					}
				} else {
					int bytes = sunxi_mmc_fifo_write(dev,
						data_req->data, total,
						data_req->timeout_ms);
					data_req->bytes_xfered =
						(bytes > 0) ? bytes : 0;
					if (bytes < 0) {
						ret = bytes;
					} else {
						ret = k_sem_take(
							&sdata->xfer_sem,
							K_MSEC(data_req->timeout_ms));
						if (ret) {
							LOG_ERR("Data write "
								"timeout");
							ret = -ETIMEDOUT;
							continue;
						}
						uint32_t rint = sunxi_mmc_read(
							dev,
							offsetof(struct sunxi_mmc_regs, rint));
						if (rint &
						    SUNXI_MMC_RINT_ERR_MASK) {
							sunxi_mmc_write(dev,
								offsetof(struct sunxi_mmc_regs, rint),
								rint);
							ret = sunxi_mmc_check_errors(
								dev, rint);
						}
					}
				}
			}
		}

		if (ret == 0) {
			break;
		}
	}

	if (ret) {
		uint32_t gctrl;

		LOG_WRN("request error path: ret=%d, lightweight recovery", ret);

		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, idst),
			0xFFFFFFFF);
		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, dmac),
			SUNXI_MMC_DMAC_SOFT_RST);

		gctrl = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, gctrl));
		gctrl |= SUNXI_MMC_GCTRL_FIFO_RST;
		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, gctrl),
			gctrl);

		/* Wait for FIFO reset to self-clear */
		sunxi_mmc_wait_reg(dev,
			offsetof(struct sunxi_mmc_regs, gctrl),
			SUNXI_MMC_GCTRL_FIFO_RST, 0,
			SUNXI_MMC_POLL_TIMEOUT_US);

		/* Disable IDMAC interrupt enable */
		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, idie), 0);
		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, rint), 0xFFFFFFFF);

		k_busy_wait(100);
	}

	return ret;
}

static int sdhc_sunxi_get_card_present(const struct device *dev)
{
	const struct sdhc_sunxi_config *cfg = dev->config;

	/* Prefer GPIO-based card detection if configured */
	if (cfg->cd_gpio.port) {
		return gpio_pin_get_dt(&cfg->cd_gpio) ? 0 : 1;
	}

	/* Fall back to hardware STATUS register.
	 * V3s: CARD_PRESENT = BIT(8)
	 */
	uint32_t status = sunxi_mmc_read(dev,
		offsetof(struct sunxi_mmc_regs, status));

	return (status & SUNXI_MMC_STATUS_CARD_PRESENT) ? 1 : 0;
}

static int sdhc_sunxi_card_busy(const struct device *dev)
{
	uint32_t status = sunxi_mmc_read(dev,
		offsetof(struct sunxi_mmc_regs, status));

	return (status & SUNXI_MMC_STATUS_CARD_DATA_BUSY) ? 1 : 0;
}

static int sdhc_sunxi_enable_interrupt(const struct device *dev,
				       sdhc_interrupt_cb_t callback,
				       int sources, void *user_data)
{
	struct sdhc_sunxi_data *data = dev->data;

	data->irq_cb = callback;
	data->irq_user_data = user_data;
	data->irq_sources = sources;

	/* Enable SDIO interrupt in the controller if requested */
	if (sources & SDHC_INT_SDIO) {
		uint32_t imask = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, imask));
		imask |= SUNXI_MMC_RINT_SDIO_INTERRUPT;
		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, imask), imask);
	}
	return 0;
}

static int sdhc_sunxi_disable_interrupt(const struct device *dev, int sources)
{
	struct sdhc_sunxi_data *data = dev->data;

	data->irq_sources &= ~sources;

	if ((sources & SDHC_INT_SDIO) &&
	    !(data->irq_sources & SDHC_INT_SDIO)) {
		uint32_t imask = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, imask));
		imask &= ~SUNXI_MMC_RINT_SDIO_INTERRUPT;
		sunxi_mmc_write(dev,
			offsetof(struct sunxi_mmc_regs, imask), imask);
	}
	return 0;
}

static int sdhc_sunxi_get_host_props(const struct device *dev,
				     struct sdhc_host_props *props)
{
	struct sdhc_sunxi_data *data = dev->data;

	memcpy(props, &data->props, sizeof(*props));
	return 0;
}

static DEVICE_API(sdhc, sdhc_sunxi_api) = {
	.reset           = sdhc_sunxi_reset,
	.request         = sdhc_sunxi_request,
	.set_io          = sdhc_sunxi_set_io,
	.get_card_present = sdhc_sunxi_get_card_present,
	.card_busy       = sdhc_sunxi_card_busy,
	.enable_interrupt = sdhc_sunxi_enable_interrupt,
	.disable_interrupt = sdhc_sunxi_disable_interrupt,
	.get_host_props  = sdhc_sunxi_get_host_props,
};

static void sdhc_sunxi_init_host_props(struct sdhc_sunxi_data *data,
				       const struct sdhc_sunxi_config *cfg)
{
	struct sdhc_host_props *props = &data->props;

	memset(props, 0, sizeof(*props));

	props->f_max = cfg->max_bus_freq;
	props->f_min = cfg->min_bus_freq;
	props->power_delay = 100;
	props->is_spi = false;
	props->bus_4_bit_support = (cfg->bus_width >= 4);

	props->host_caps.vol_330_support = 1;
	props->host_caps.high_spd_support = 1;
#ifdef CONFIG_SDHC_SUNXI_DMA
	props->host_caps.sdma_support = 1;
#else
	props->host_caps.sdma_support = 0;
#endif

	if (cfg->bus_width >= 8) {
		props->host_caps.bus_8_bit_support = 1;
	}

	props->max_current_330 = 0;
	props->max_current_300 = 0;
	props->max_current_180 = 0;
}

static int sdhc_sunxi_init(const struct device *dev)
{
	struct sdhc_sunxi_data *data = dev->data;
	const struct sdhc_sunxi_config *cfg = dev->config;
	int ret;

	/* Map MMIO registers */
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* Apply pinctrl configuration for MMC pins (CLK, CMD, D0-D3) */
	if (cfg->pcfg) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret) {
			LOG_ERR("Failed to apply pinctrl: %d", ret);
			return ret;
		}
		LOG_DBG("MMC pinctrl applied");

		/*
		 * Diagnostic: verify PF0-PF5 pin configuration by reading
		 * PIO registers directly. Port F (port 5), stride 0x24.
		 * FUNC_PF_SDC0=2, each pin uses 4 bits in CFG0 register.
		 * Expected: CFG0[23:0] = 0x222222 (PF0-PF5 all func 2).
		 * Pull-up = 1, each pin uses 2 bits in PUD register.
		 * Expected: PUD[11:0] = 0x555 (all pull-up).
		 */
		#if CONFIG_SDHC_LOG_LEVEL >= LOG_LEVEL_DBG
		{
			uintptr_t pio_base = DT_REG_ADDR(DT_NODELABEL(pinctrl));
			uint32_t port_off = 5 * 0x24; /* Port F */
			uint32_t pf_cfg0 = sys_read32(pio_base + port_off);
			uint32_t pf_pud = sys_read32(pio_base + port_off + 0x1C);
			uint32_t pf_drv = sys_read32(pio_base + port_off + 0x14);

			LOG_WRN("PIO PortF (after pinctrl): "
				"CFG0=0x%08x PUD=0x%08x DRV=0x%08x",
				pf_cfg0, pf_pud, pf_drv);
			if ((pf_cfg0 & 0x00FFFFFF) != 0x00222222) {
				LOG_WRN("PF0-PF5 mux NOT SDC0! "
					"Expected 0x222222 got 0x%06x",
					pf_cfg0 & 0x00FFFFFF);
			}
		}
		#endif
	}

	/*
	 * Default source clock from devicetree src-clock-freq property.
	 * Will be updated with actual CCU output after clock configuration.
	 */
	data->src_clk_hz = cfg->src_clock_freq;

	if (cfg->reset.dev) {
		if (!device_is_ready(cfg->reset.dev)) {
			LOG_ERR("Reset device not ready");
			return -ENODEV;
		}
		ret = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
		if (ret) {
			LOG_ERR("Reset toggle failed");
			return ret;
		}
		LOG_DBG("Hardware reset toggled");
	}

	if (cfg->clk_dev && device_is_ready(cfg->clk_dev)) {
		ret = clock_control_on(cfg->clk_dev, cfg->clk_id_bus);
		if (ret) {
			LOG_ERR("Failed to enable bus clock (err %d)", ret);
			return ret;
		}

		ret = clock_control_on(cfg->clk_dev, cfg->clk_id_mmc);
		if (ret) {
			LOG_ERR("Failed to enable mmc clock (err %d)", ret);
			return ret;
		}

		ret = clock_control_on(cfg->clk_dev, cfg->clk_id_output);
		if (ret) {
			LOG_ERR("Failed to enable output clock (err %d)", ret);
			return ret;
		}

		ret = clock_control_on(cfg->clk_dev, cfg->clk_id_sample);
		if (ret) {
			LOG_ERR("Failed to enable sample clock (err %d)", ret);
			return ret;
		}

		/* Request desired source clock frequency from CCU */
		if (cfg->src_clock_freq > 0) {
			ret = clock_control_set_rate(
				cfg->clk_dev, cfg->clk_id_mmc,
				(clock_control_subsys_rate_t)
					(uintptr_t)cfg->src_clock_freq);
			if (ret) {
				LOG_WRN("set_rate(%u) failed (err %d), "
					"using default clock",
					cfg->src_clock_freq, ret);
			}
		}

		/* Get actual source rate from CCU (may differ from
		 * requested due to divider constraints).
		 */
		ret = clock_control_get_rate(cfg->clk_dev, cfg->clk_id_mmc,
					     &data->src_clk_hz);
		if (ret) {
			LOG_WRN("get_rate() failed (ret=%d), "
				"using frequency=%u",
				ret, cfg->src_clock_freq);
			data->src_clk_hz = cfg->src_clock_freq;
		}
		LOG_DBG("All 4 MMC clocks enabled, src: %u Hz "
			"(requested %u)", data->src_clk_hz,
			cfg->src_clock_freq);

		/*
		 * Diagnostic: dump key registers and CCU clock register
		 * to verify clocks are actually enabled. Now that hw reset
		 * is done first, registers should show non-zero values.
		 */
		#if CONFIG_SDHC_LOG_LEVEL >= LOG_LEVEL_DBG
		{
			uint32_t gctrl = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, gctrl));
			uint32_t clkcr = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, clkcr));
			uint32_t status = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, status));
			uint32_t cmd_reg = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, cmd));
			uint32_t rint_reg = sunxi_mmc_read(dev,
				offsetof(struct sunxi_mmc_regs, rint));

			LOG_DBG("After hw_reset+clk_on: GCTRL=0x%08x CLKCR=0x%08x "
				"STATUS=0x%08x CMD=0x%08x RINT=0x%08x",
				gctrl, clkcr, status, cmd_reg, rint_reg);

			uintptr_t ccu_base = DT_REG_ADDR(DT_NODELABEL(ccu));
			uint32_t mmc0_clk = sys_read32(ccu_base + 0x88);
			uint32_t bus_gating0 = sys_read32(ccu_base + 0x60);
			LOG_DBG("CCU MMC0_CLK(0x88)=0x%08x BUS_GATING0(0x60)=0x%08x "
				"(expect bit8=1, bit31=1)",
				mmc0_clk, bus_gating0);
		}

		/*
		 * Diagnostic: check PIO PortF pull-up state AFTER hardware
		 * reset and clock enable. If PUD=0, the pinctrl was either
		 * not applied or was cleared by the reset sequence.
		 * This helps diagnose why CMD8 RESP_ERROR occurs.
		 */
		{
			uintptr_t pio_base =
				DT_REG_ADDR(DT_NODELABEL(pinctrl));
			uint32_t port_off = 5 * 0x24; /* Port F */
			uint32_t pf_pud = sys_read32(pio_base + port_off + 0x1C);

			if (pf_pud == 0) {
				LOG_WRN("PIO PF PUD=0 after hw_reset+clk! "
					"Forcing pull-up on PF0-PF5 (SDC0)");
				/*
				 * Explicitly configure pull-up on PF0-PF5.
				 * Each pin uses 2 bits in PULL0 register:
				 * 00=none, 01=pull-up, 10=pull-down.
				 * PF0-PF5 = bits [11:0], pull-up = 0x555.
				 */
				sys_write32(0x00000555,
					pio_base + port_off + 0x1C);
				pf_pud = sys_read32(pio_base + port_off + 0x1C);
				LOG_DBG("PIO PF PUD after fix: 0x%08x",
					pf_pud);
			} else {
				LOG_DBG("PIO PF PUD OK after hw_reset+clk: "
					"0x%08x", pf_pud);
			}
		}
		#endif
	} else {
		LOG_WRN("No clock device, defaulting to 24MHz");
	}

	/* Configure card detect GPIO if present */
	if (cfg->cd_gpio.port) {
		if (!device_is_ready(cfg->cd_gpio.port)) {
			LOG_ERR("Card detect GPIO device not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->cd_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("Failed to configure card detect GPIO: %d", ret);
			return ret;
		}
		LOG_DBG("Card detect GPIO configured on %s pin %u",
			cfg->cd_gpio.port->name, cfg->cd_gpio.pin);
	}

	/* Initialize semaphores */
	k_sem_init(&data->cmd_sem, 0, 1);
	k_sem_init(&data->xfer_sem, 0, 1);

	/* Initialize host properties */
	sdhc_sunxi_init_host_props(data, cfg);

	/* Initialize host I/O state */
	data->host_io.clock = 0;
	data->host_io.bus_mode = SDHC_BUSMODE_PUSHPULL;
	data->host_io.power_mode = SDHC_POWER_OFF;
	data->host_io.bus_width = SDHC_BUS_WIDTH1BIT;
	data->host_io.timing = SDHC_TIMING_LEGACY;
	data->host_io.signal_voltage = SD_VOL_3_3_V;

	/* Configure interrupts */
	cfg->irq_config_func(dev);

	/*
	 * Initialize IDMAC DMA if enabled.
	 * Must be done before controller reset so descriptors are ready.
	 */
#ifdef CONFIG_SDHC_SUNXI_DMA
	ret = sunxi_mmc_dma_init(dev);
	if (ret) {
		LOG_WRN("DMA init failed (%d), falling back to PIO", ret);
		data->use_dma = false;
	}
#endif

	ret = sdhc_sunxi_reset(dev);
	if (ret) {
		LOG_ERR("Controller reset failed");
		return ret;
	}

	{
		uint32_t clkcr = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, clkcr));
		clkcr &= ~(SUNXI_MMC_CLKCR_CLK_ENB | SUNXI_MMC_CLKCR_CLK_LOW_PWR);
		sunxi_mmc_write(dev, offsetof(struct sunxi_mmc_regs, clkcr), clkcr);
		ret = sunxi_mmc_update_clk(dev);
		if (ret) {
			LOG_WRN("CIU priming update_clk failed: %d "
				"(non-fatal, recovery path will handle)", ret);
		} else {
			LOG_DBG("CIU primed: update_clk OK (clock off)");
		}
	}

	/* Diagnostic: dump STATUS register to check card presence */
	#if CONFIG_SDHC_LOG_LEVEL >= LOG_LEVEL_DBG
	{
		uint32_t status = sunxi_mmc_read(dev,
			offsetof(struct sunxi_mmc_regs, status));

		LOG_DBG("STATUS=0x%08x: CARD_PRESENT=%d DATA_BUSY=%d FSM_BUSY=%d",
			status,
			!!(status & SUNXI_MMC_STATUS_CARD_PRESENT),
			!!(status & SUNXI_MMC_STATUS_CARD_DATA_BUSY),
			!!(status & SUNXI_MMC_STATUS_DATA_FSM_BUSY));
	}
	#endif
	LOG_DBG("Sunxi MMC initialized: base clock=%u Hz, bus_width=%u",
		data->src_clk_hz, cfg->bus_width);

	return 0;
}

/*
 * IRQ configuration helper macro
 */
#define SDHC_SUNXI_IRQ_CONFIG(inst)						\
	static void irq_config_func_##inst(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),				\
			    DT_INST_IRQ(inst, priority),			\
			    sdhc_sunxi_isr,					\
			    DEVICE_DT_INST_GET(inst),				\
			    DT_INST_IRQ(inst, flags));				\
		irq_enable(DT_INST_IRQN(inst));				\
	}

/*
 * Driver instantiation macro
 */
#define SDHC_SUNXI_INIT(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
	SDHC_SUNXI_IRQ_CONFIG(inst);						\
										\
	static const struct sdhc_sunxi_config sdhc_sunxi_config_##inst = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),			\
		.bus_width = DT_INST_PROP(inst, bus_width),			\
		.max_bus_freq = DT_INST_PROP(inst, max_bus_freq),		\
		.min_bus_freq = DT_INST_PROP(inst, min_bus_freq),		\
		.src_clock_freq = DT_INST_PROP(inst, frequency),		\
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),		\
		.clk_id_bus = (clock_control_subsys_t)(uintptr_t)		\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, ahb, clk_id),	\
		.clk_id_mmc = (clock_control_subsys_t)(uintptr_t)		\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, mmc, clk_id),	\
		.clk_id_output = (clock_control_subsys_t)(uintptr_t)		\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, output, clk_id),	\
		.clk_id_sample = (clock_control_subsys_t)(uintptr_t)		\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, sample, clk_id),	\
		.reset = RESET_DT_SPEC_INST_GET(inst),				\
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, cd_gpios, {0}),	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),			\
		.irq_config_func = irq_config_func_##inst,			\
	};									\
										\
	static struct sdhc_sunxi_data sdhc_sunxi_data_##inst;			\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			      sdhc_sunxi_init,					\
			      NULL,						\
			      &sdhc_sunxi_data_##inst,				\
			      &sdhc_sunxi_config_##inst,			\
			      POST_KERNEL,					\
			      CONFIG_SDHC_INIT_PRIORITY,			\
			      &sdhc_sunxi_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_SUNXI_INIT)
