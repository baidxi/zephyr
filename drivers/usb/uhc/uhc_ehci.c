 /*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic EHCI USB Host Controller Driver for Zephyr.
 *
 * Supports any standard EHCI-compliant controller via
 * compatible = "generic-ehci" in device tree.
 *
 * Platform-specific initialization (clocks, resets, PHY) is
 * handled through Zephyr DT infrastructure.
 */

#define DT_DRV_COMPAT generic_ehci

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>	
#include <zephyr/drivers/usb/uhc.h>

#include "uhc_common.h"
#include "uhc_ehci.h"

/*
 * DMA cache coherency helpers.
 *
 * The EHCI controller is a DMA bus master that reads/writes system
 * memory directly (bypassing the CPU data cache).  We use the portable
 * Zephyr cache API to ensure cache coherency between CPU and controller.
 * When CONFIG_CACHE_MANAGEMENT / CONFIG_DCACHE are not set (e.g. on
 * Cortex-M without D-cache) these functions are no-ops, which is correct
 * because there is no coherency issue.
 */
#include <zephyr/cache.h>

static inline void ehci_dcache_flush_range(void *addr, size_t size)
{
	(void)sys_cache_data_flush_range(addr, size);
}

static inline void ehci_dcache_invd_range(void *addr, size_t size)
{
	(void)sys_cache_data_invd_range(addr, size);
}

/*
 * Sunxi USB PHY pseudo-device support (optional).  When a DT instance
 * has a "phys" property referencing an allwinner,sun4i-usb-phy, the
 * config stores a pointer to the per-controller PHY pseudo-device so
 * that ehci_platform_init() can power up the PHY before touching the
 * EHCI registers.
 */
#include "../common/sunxi/usb_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_ehci, CONFIG_UHC_DRIVER_LOG_LEVEL);

struct uhc_ehci_priv {
	uintptr_t reg_base;
	uintptr_t op_base;
	uint32_t hcs_params;
	uint32_t hcc_params;
	uint8_t  n_ports;

	/* Schedule.
	 * The periodic frame list must be 4096-byte aligned (EHCI spec).
	 * Use a static buffer instead of k_aligned_alloc() so the driver
	 * does not depend on the system heap (CONFIG_HEAP_MEM_POOL_SIZE)
	 * being sized, and allocation can never fail at init time.
	 */
	uint32_t *periodic_list;
	uint32_t __aligned(4096) periodic_list_buf[EHCI_FRAME_LIST_SIZE];
	struct ehci_qh *async_qh;
	struct ehci_qh *periodic_qh;

	/* Deferred-free list for async schedule QHs.
	 * Per EHCI §4.8.2, after removing a QH from the async schedule
	 * software must wait for the IAAD (Interrupt on Async Advance)
	 * doorbell handshake before freeing the QH memory, because the
	 * host controller may still be referencing the removed QH during
	 * its current async schedule traversal.  QHs removed in the ISR
	 * completion handler are added to this singly-linked list (via
	 * the software `next` pointer), the IAAD doorbell is rung, and
	 * the actual k_mem_slab_free happens in the ISR when USBSTS_IAA
	 * fires. */
	struct ehci_qh *async_pending_free;

	/* Descriptor pools.
	 * The EHCI spec requires queue heads and queue element transfer
	 * descriptors to be aligned on a 32-byte boundary (the controller
	 * uses the low 5 bits of the address for type/flag metadata).
	 */
	struct k_mem_slab qh_pool;
	struct k_mem_slab qtd_pool;
	char __aligned(32) qh_pool_buf[EHCI_MAX_QH * sizeof(struct ehci_qh)];
	char __aligned(32) qtd_pool_buf[EHCI_MAX_QTD * sizeof(struct ehci_qtd)];
};

#define UHC_EHCI_MAX_CLOCKS	4
#define UHC_EHCI_MAX_RESETS	4

/*
	* DT element initializers used to populate the per-instance clock and
	* reset arrays from the "clocks" / "resets" devicetree properties.
	* Each expands to a single array element followed by a trailing comma.
	*/
#define EHCI_CLK_ELEM(node_id, prop, idx) \
	(clock_control_subsys_t)(uintptr_t)DT_PHA_BY_IDX(node_id, clocks, idx, clk_id),
#define EHCI_RST_ELEM(node_id, prop, idx) \
	(uint32_t)DT_PHA_BY_IDX(node_id, resets, idx, id),

struct uhc_ehci_config {
	uintptr_t reg_base;
	void (*irq_enable_fn)(const struct device *dev);
	void (*irq_disable_fn)(const struct device *dev);

	/* Platform clocks (bus + functional), enabled before register access */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys[UHC_EHCI_MAX_CLOCKS];
	uint8_t num_clocks;

	/* Platform resets, deasserted before register access */
	const struct device *reset_dev;
	uint32_t reset_ids[UHC_EHCI_MAX_RESETS];
	uint8_t num_resets;

	/* Optional USB PHY pseudo-device (sunxi) */
	const struct sunxi_usb_phy *phy;
};

static inline uint32_t ehci_read32(uintptr_t base, uint32_t offset)
{
	return sys_read32(base + offset);
}

static inline void ehci_write32(uintptr_t base, uint32_t offset, uint32_t val)
{
	sys_write32(val, base + offset);
}

/* 16-bit capability register read (e.g. HCIVERSION at offset 0x02).
 * MUST use sys_read16: a sys_read32() at an offset that is not
 * 4-byte-aligned (such as 0x02) is an unaligned 32-bit access and
 * triggers a data-abort (alignment fault) on Cortex-A with SCTLR.A set.
 */
static inline uint16_t ehci_read16(uintptr_t base, uint32_t offset)
{
	return sys_read16(base + offset);
}

static int ehci_mem_init(struct uhc_ehci_priv *priv)
{
	priv->periodic_list = priv->periodic_list_buf;
	memset(priv->periodic_list, 0, EHCI_FRAME_LIST_SIZE * sizeof(uint32_t));

	k_mem_slab_init(&priv->qh_pool, priv->qh_pool_buf,
			sizeof(struct ehci_qh), EHCI_MAX_QH);
	k_mem_slab_init(&priv->qtd_pool, priv->qtd_pool_buf,
			sizeof(struct ehci_qtd), EHCI_MAX_QTD);

	if (k_mem_slab_alloc(&priv->qh_pool, (void **)&priv->async_qh, K_NO_WAIT)) {
		return -ENOMEM;
	}
	memset(priv->async_qh, 0, sizeof(struct ehci_qh));

	if (k_mem_slab_alloc(&priv->qh_pool, (void **)&priv->periodic_qh, K_NO_WAIT)) {
		return -ENOMEM;
	}
	memset(priv->periodic_qh, 0, sizeof(struct ehci_qh));

	/* Flush all DMA descriptor buffers to RAM so the controller
	 * (which bypasses the CPU L1 cache) sees the correct data. */
	ehci_dcache_flush_range(priv->periodic_list,
				EHCI_FRAME_LIST_SIZE * sizeof(uint32_t));
	ehci_dcache_flush_range(priv->async_qh, sizeof(struct ehci_qh));
	ehci_dcache_flush_range(priv->periodic_qh, sizeof(struct ehci_qh));

	return 0;
}

static int ehci_platform_init(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	const struct uhc_ehci_config *cfg = dev->config;
	uint32_t cap_length, hciversion;
	int ret;

	priv->reg_base = cfg->reg_base;

	/* ---- Platform bring-up: clocks, resets, PHY ----
	 * On SoCs where the EHCI block is held in reset or has its bus
	 * clock gated at power-on (e.g. Allwinner T113-S3), the EHCI
	 * capability registers read back as 0x0000/0xFFFF until the
	 * platform layer explicitly enables the clock, releases reset
	 * and powers up the PHY.  Do all three before touching the regs.
	 */

	LOG_INF("platform init: %u clock(s), %u reset(s), phy=%s",
		cfg->num_clocks, cfg->num_resets, cfg->phy ? "yes" : "no");

	/* Enable all bus / functional clocks declared in DT. */
	for (uint8_t i = 0; i < cfg->num_clocks; i++) {
		ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys[i]);
		if (ret < 0 && ret != -ENOTSUP) {
			LOG_ERR("clock_control_on[%u] (subsys=0x%lx) failed: %d",
				i, (uintptr_t)cfg->clock_subsys[i], ret);
			return ret;
		}
		LOG_INF("  clock[%u] subsys=0x%lx -> %s", i,
			(uintptr_t)cfg->clock_subsys[i],
			(ret == -ENOTSUP) ? "always-on" : "ok");
	}

	/* Deassert all resets declared in DT. */
	for (uint8_t i = 0; i < cfg->num_resets; i++) {
		ret = reset_line_deassert(cfg->reset_dev, cfg->reset_ids[i]);
		if (ret < 0) {
			LOG_ERR("reset_line_deassert[%u] (id=0x%x) failed: %d",
				i, cfg->reset_ids[i], ret);
			return ret;
		}
		LOG_INF("  reset[%u] id=0x%x deasserted", i, cfg->reset_ids[i]);
	}

	/* Power up the PHY (SIDDQ clear, ULPI/UTMI passby enable). */
	if (cfg->phy != NULL) {
		LOG_INF("  enabling PHY (index=%u)", cfg->phy->index);
		ret = cfg->phy->enable(cfg->phy);
		if (ret) {
			LOG_ERR("PHY enable failed: %d", ret);
			return ret;
		}
		LOG_INF("  PHY enabled");

	/* DIAGNOSTIC: dump PHY1 PMU registers to verify ULPI
		* passby and SIDDQ are correctly configured.
		* PHY1 PMU base = 0x04200800 (from sunxi-d1s-t113.dtsi).
		* PMU[0x00] = passby control (ULPI_BYPASS bit etc.)
		* PMU[0x10] = HCI PHY control (SIDDQ bit) */
	{
		uint32_t pmu1 = 0x04200800;
		LOG_DBG("PHY1 PMU[0x00]=%08x PMU[0x10]=%08x",
			sys_read32(pmu1), sys_read32(pmu1 + 0x10));
	}
	}

	/* Give the controller a moment to settle after clock/reset/PHY. */
	k_busy_wait(10);
	LOG_INF("  raw CAPLENGTH dword=0x%08x",
		ehci_read32(priv->reg_base, EHCI_CAPLENGTH));

	cap_length = ehci_read32(priv->reg_base, EHCI_CAPLENGTH) & 0xFF;
	priv->op_base = priv->reg_base + cap_length;

	/* Synchronous printk so we see this even if the CPU hangs before
	 */
	hciversion = ehci_read16(priv->reg_base, EHCI_HCIVERSION);
	LOG_DBG("reg_base=%p cap_length=%u op_base=%p hciversion=%04x",
	 (void *)priv->reg_base, cap_length,
	 (void *)priv->op_base, hciversion);

	/* CAPLENGTH must be a small multiple of 4 (typically 0x10).  If the
	 * controller is unclocked/held-in-reset the bus returns 0x00 or
	 * 0xFF; computing op_base from that and then touching the op regs
	 * (USBSTS/USBCMD at op_base+0x14/0x10) goes OUTSIDE the 0x100 MMU
	 * region mapped for this controller -> data abort / bus lock.
	 * Bail out safely instead. */
	if (cap_length == 0 || cap_length > 0x40 || (cap_length & 0x3)) {
		LOG_ERR("bogus CAPLENGTH=0x%02x — controller not "
			"clocked/reset? Aborting to avoid MMU fault.",
		       cap_length);
		return -EIO;
	}
	LOG_INF("EHCI HCIVERSION %04x, CAPLENGTH=%u", hciversion, cap_length);

	/* DIAGNOSTIC: detect an unclocked / held-in-reset controller.
	 * If capability registers read as all-zero or all-ones, the bus
	 * clock is gated or reset is asserted -> no device will ever be
	 * detected. Expected HCIVERSION == 0x0100.
	 */
	if (hciversion == 0x0000 || hciversion == 0xFFFF) {
		LOG_ERR("EHCI capability regs look invalid (HCIVERSION=%04x, "
			"raw CAPLENGTH dword=%08x). Bus clock gated or reset "
			"asserted? Check clock_control_on()/reset_control_"
			"deassert() and PHY power-up (SIDDQ) in platform init!",
			hciversion, ehci_read32(priv->reg_base, EHCI_CAPLENGTH));
	}

	priv->hcs_params = ehci_read32(priv->reg_base, EHCI_HCSPARAMS);
	priv->hcc_params = ehci_read32(priv->reg_base, EHCI_HCCPARAMS);
	priv->n_ports = HCSPARAMS_N_PORTS(priv->hcs_params);
	LOG_INF("EHCI HCSPARAMS=%08x HCCPARAMS=%08x -> %u port(s)",
		priv->hcs_params, priv->hcc_params, priv->n_ports);

	return 0;
}

static int ehci_hw_reset(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t cmd;
	int timeout;

	/* Per the EHCI spec the controller is Halted after reset until a
	 * CONFIGFLAG + run is issued.  On a freshly-powered controller the
	 * HCH bit may read as 0 while USBCMD.RS is also 0 (no schedule is
	 * armed), so waiting for a halt transition here never completes.
	 * Only attempt a graceful stop when the controller is actually
	 * running (USBCMD.RS set).  HCRESET below will halt it regardless.
	 */
	cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
	if ((cmd & USBCMD_RS) &&
	    !(ehci_read32(priv->op_base, EHCI_USBSTS) & USBSTS_HCH)) {
		cmd &= ~USBCMD_RS;
		ehci_write32(priv->op_base, EHCI_USBCMD, cmd);
		timeout = EHCI_RUN_STOP_TIMEOUT_US;
		while (!(ehci_read32(priv->op_base, EHCI_USBSTS) & USBSTS_HCH)) {
			if (--timeout == 0) {
				LOG_ERR("Timeout waiting for HC halt");
				return -ETIMEDOUT;
			}
			k_busy_wait(1);
		}
	}

	cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
	cmd |= USBCMD_HCRESET;
	ehci_write32(priv->op_base, EHCI_USBCMD, cmd);

	timeout = EHCI_RESET_TIMEOUT_US;
	while (ehci_read32(priv->op_base, EHCI_USBCMD) & USBCMD_HCRESET) {
		if (--timeout == 0) {
			LOG_ERR("Timeout waiting for HCRESET");
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}

	/* After HCRESET self-clears the controller is in the Halted state
	 * per the EHCI spec.  Some Allwinner EHCI implementations do not
	 * assert USBSTS.HCH immediately after reset, so we do not poll it
	 * here (mirrors Linux ehci_reset() behaviour).
	 */
	return 0;
}

static int ehci_hw_setup(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t cmd;

	ehci_write32(priv->op_base, EHCI_CTRLDSSEGMENT, 0);

	cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
	cmd &= ~USBCMD_FLS_MASK;
	cmd |= USBCMD_FLS_1024;
	cmd &= ~USBCMD_ITC_MASK;
	cmd |= USBCMD_ITC_8;
	/* Disable Async Schedule Park Mode.  The Allwinner EHCI comes
	 * out of reset with Park Mode enabled (USBCMD bit 11).  Park
	 * mode is known to cause the controller to get stuck
	 * re-visiting the same QH without advancing the frame counter,
	 * so no SOF packets are generated and the device never sees
	 * any traffic.  Linux ehci-hcd does not enable park mode. */
	cmd &= ~(USBCMD_ASPE | USBCMD_ASPC_MASK);
	ehci_write32(priv->op_base, EHCI_USBCMD, cmd);

	ehci_write32(priv->op_base, EHCI_PERIODICLISTBASE,
		     (uint32_t)(uintptr_t)priv->periodic_list);

	/* The async schedule MUST be a circular ring.  The head QH (H=1)
	 * links back to itself when idle.  When a transfer QH is enqueued
	 * it is inserted right after the head and inherits the head's old
	 * horiz_link, keeping the ring closed:
	 *   idle:  head -> head
	 *   busy:  head -> qh -> head   */
	priv->async_qh->horiz_link =
		(uint32_t)(uintptr_t)priv->async_qh | EHCI_PTR_TYPE_QH;
	priv->async_qh->ep_char = QH_EP_CHAR_H;
	priv->async_qh->current_qtd = 0;
	priv->async_qh->overlay_next = QTD_NEXT_TERMINATE;
	priv->async_qh->overlay_token = QTD_STS_HALT;
	priv->async_qh->qh_dma = (uint32_t)(uintptr_t)priv->async_qh;
	ehci_write32(priv->op_base, EHCI_ASYNCLISTADDR,
		     (uint32_t)(uintptr_t)priv->async_qh | EHCI_PTR_TYPE_QH);

	for (int i = 0; i < EHCI_FRAME_LIST_SIZE; i++) {
		priv->periodic_list[i] = (uint32_t)(uintptr_t)priv->periodic_qh
					 | EHCI_PTR_TYPE_QH;
	}
	priv->periodic_qh->horiz_link = EHCI_PTR_TERMINATE;
	priv->periodic_qh->ep_char = 0;
	priv->periodic_qh->overlay_next = QTD_NEXT_TERMINATE;
	priv->periodic_qh->overlay_token = QTD_STS_HALT;
	priv->periodic_qh->qh_dma = (uint32_t)(uintptr_t)priv->periodic_qh;

	for (int i = 0; i < priv->n_ports; i++) {
		uint32_t portsc;

		portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE + (i * 4));
		portsc &= ~PORTSC_PO;
		portsc |= PORTSC_PP;
		ehci_write32(priv->op_base, EHCI_PORTSC_BASE + (i * 4), portsc);
	}

	ehci_write32(priv->op_base, EHCI_CONFIGFLAG, CONFIGFLAG_CF);

	ehci_write32(priv->op_base, EHCI_USBSTS,
		     ehci_read32(priv->op_base, EHCI_USBSTS));

	ehci_write32(priv->op_base, EHCI_USBINTR,
		     USBINTR_USBINTE | USBINTR_USBERRE |
		     USBINTR_PCDE | USBINTR_IAAE | USBINTR_HSEE);

	/* Flush all DMA descriptors to RAM so the controller sees the
	 * correct periodic list entries and queue heads when it starts. */
	ehci_dcache_flush_range(priv->periodic_list,
				EHCI_FRAME_LIST_SIZE * sizeof(uint32_t));
	ehci_dcache_flush_range(priv->async_qh, sizeof(struct ehci_qh));
	ehci_dcache_flush_range(priv->periodic_qh, sizeof(struct ehci_qh));

	return 0;
}

static int ehci_hw_run(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t cmd;

	/* Ensure all prior DMA descriptor writes are visible to the
	 * controller before we set RS=1 and start the schedule. */
	__asm__ volatile("mcr p15, 0, %0, c7, c10, 4" : : "r"(0) : "memory");

	cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
	cmd |= USBCMD_ASE | USBCMD_PSE | USBCMD_RS;
	ehci_write32(priv->op_base, EHCI_USBCMD, cmd);

	return 0;
}

static int ehci_hw_stop(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t cmd;
	int timeout;

	cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
	cmd &= ~(USBCMD_RS | USBCMD_ASE | USBCMD_PSE);
	ehci_write32(priv->op_base, EHCI_USBCMD, cmd);

	timeout = EHCI_RUN_STOP_TIMEOUT_US;
	while (!(ehci_read32(priv->op_base, EHCI_USBSTS) & USBSTS_HCH)) {
		if (--timeout == 0) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}

	return 0;
}

static void ehci_qtd_fill(struct ehci_qtd *qtd, const uint8_t *data,
			  uint32_t len, uint32_t pid, bool ioc)
{
	uint32_t token;

	memset(qtd, 0, sizeof(*qtd));
	qtd->next = QTD_NEXT_TERMINATE;
	qtd->alt_next = QTD_NEXT_TERMINATE;

	if (data && len > 0) {
		uint32_t addr = (uint32_t)(uintptr_t)data;
		uint32_t remaining = len;

		/* For SETUP and OUT the controller reads the payload from
		 * RAM via DMA, so flush the CPU cache to make the data
		 * written by the CPU visible to the controller.  For IN the
		 * controller writes the buffer and we invalidate it on
		 * completion, so no flush is needed here. */
		if (pid == QTD_PID_SETUP || pid == QTD_PID_OUT) {
			ehci_dcache_flush_range((void *)data, len);
		}

		for (int i = 0; i < 5 && remaining > 0; i++) {
			uint32_t page_size = 0x1000 - (addr & 0xFFF);

			if (page_size > remaining) {
				page_size = remaining;
			}
			qtd->buf[i] = addr;
			addr += page_size;
			remaining -= page_size;
		}
	}

	token = (len & 0x7FFF) << QTD_LENGTH_SHIFT;
	token |= ioc ? QTD_IOC : 0;
	token |= pid;
	token |= QTD_CERR_MASK;
	token |= QTD_STS_ACTIVE;
	qtd->token = token;

	/* Flush qTD to RAM so the controller sees the correct fields. */
	ehci_dcache_flush_range(qtd, sizeof(*qtd));
}

static void ehci_qtd_fill_setup(struct ehci_qtd *qtd, const uint8_t *setup_data)
{
	/* IOC=false: only the LAST qTD (status phase) should have IOC=1.
	 * If setup_qtd has IOC=1, the HC fires an interrupt immediately
	 * after the SETUP phase, and the completion handler may unlink
	 * the QH before the DATA phase executes — resulting in 0 bytes
	 * received for IN transfers. */
	ehci_qtd_fill(qtd, setup_data, 8, QTD_PID_SETUP, false);
}

/* Forward declarations — defined later in the file. */
static void ehci_drain_pending_free(struct uhc_ehci_priv *priv);
static void ehci_free_qtd_chain(struct uhc_ehci_priv *priv,
				struct ehci_qtd *head);

static int ehci_enqueue_control(struct uhc_ehci_priv *priv,
				struct uhc_transfer *const xfer)
{
	struct ehci_qh *qh;
	struct ehci_qtd *setup_qtd, *data_qtd = NULL, *status_qtd;
	uint32_t device_addr = xfer->udev->addr;
	int ret;

	ret = k_mem_slab_alloc(&priv->qh_pool, (void **)&qh, K_NO_WAIT);
	if (ret && priv->async_pending_free) {
		ehci_drain_pending_free(priv);
		ret = k_mem_slab_alloc(&priv->qh_pool, (void **)&qh, K_NO_WAIT);
	}
	if (ret) {
		return -ENOMEM;
	}
	memset(qh, 0, sizeof(*qh));

	ret = k_mem_slab_alloc(&priv->qtd_pool, (void **)&setup_qtd, K_NO_WAIT);
	if (ret) {
		goto free_qh;
	}

	ret = k_mem_slab_alloc(&priv->qtd_pool, (void **)&status_qtd, K_NO_WAIT);
	if (ret) {
		goto free_setup;
	}

	{
		uint32_t eps = QH_EP_CHAR_EPS_HIGH;

		if (xfer->udev->speed == USB_SPEED_SPEED_FS) {
			eps = QH_EP_CHAR_EPS_FULL;
		}

		/* Match Linux ehci-q.c qh_make() for HS control:
		 *   info1 = QH_HIGH_SPEED | (RL=4 << 28) | (64<<16) | DTC
		 *   info2 = (MULT=1 << 30)
		 * Note: QH_CONTROL_EP (C bit, bit 27) is ONLY for FS/LS
		 * control endpoints through a TT.  Setting it for HS is
		 * undefined per EHCI spec and can prevent SETUP execution
		 * on some controllers (e.g. Allwinner EHCI). */
		qh->ep_char = (device_addr << QH_EP_CHAR_DEVADDR_SHIFT) |
			      (0 << QH_EP_CHAR_ENDPT_SHIFT) |
			      eps |
			      QH_EP_CHAR_DTC |
			      (xfer->mps << QH_EP_CHAR_MAX_PKT_LEN_SHIFT);
		if (eps == QH_EP_CHAR_EPS_HIGH) {
			qh->ep_char |= (4U << QH_EP_CHAR_RL_SHIFT);
			qh->ep_caps = (1U << QH_EP_CAPS_MULT_SHIFT);
		} else {
			/* FS/LS device behind a HS hub: route through the
			 * hub's Transaction Translator (TT).  ep_caps must
			 * carry the parent hub address and port number so
			 * the controller knows which TT to use for split
			 * transactions.  Matches Linux qh_make() info2:
			 *   (hub_addr << 16) | (ttport << 23) | (1 << 30). */
			qh->ep_char |= QH_EP_CHAR_C;
			qh->ep_caps = (xfer->udev->hub_addr <<
				       QH_EP_CAPS_HUB_ADDR_SHIFT) |
				      (xfer->udev->hub_port <<
				       QH_EP_CAPS_PORT_SHIFT) |
				      (1U << QH_EP_CAPS_MULT_SHIFT);
		}
	}
	qh->current_qtd = 0;
	qh->qh_dma = (uint32_t)(uintptr_t)qh;
	qh->xfer = xfer;
	qh->qtd_head = setup_qtd;

/* DIAGNOSTIC: dump controller state immediately before SETUP. */
{
	uint32_t psc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
	uint32_t sts = ehci_read32(priv->op_base, EHCI_USBSTS);
	uint32_t cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
	uint32_t asy = ehci_read32(priv->op_base, EHCI_ASYNCLISTADDR);
	LOG_DBG("ctrl: PORTSC=%08x USBSTS=%08x USBCMD=%08x ASYNCLST=%08x",
		psc, sts, cmd, asy);
}

	LOG_DBG("ctrl: addr=%u speed=%u mps=%u ep_char=%08x "
		"setup=%02x %02x %02x %02x %02x %02x %02x %02x",
		device_addr, xfer->udev->speed, xfer->mps, qh->ep_char,
		xfer->setup_pkt[0], xfer->setup_pkt[1],
		xfer->setup_pkt[2], xfer->setup_pkt[3],
		xfer->setup_pkt[4], xfer->setup_pkt[5],
		xfer->setup_pkt[6], xfer->setup_pkt[7]);

	ehci_qtd_fill_setup(setup_qtd, xfer->setup_pkt);

	/* For control transfers with DTC=1 the EHCI loads the initial
	 * data toggle from each qTD's DT bit (bit 31).  SETUP is always
	 * DATA1 (hardware-forced) so setup_qtd can leave DT=0.
	 * But the DATA and STATUS phases MUST have DT=1 (DATA1).
	 *
	 * This mirrors Linux ehci-q.c qh_urb_transaction() where:
	 *   token ^= QTD_TOGGLE after SETUP  (DATA phase: DATA1)
	 *   token |= QTD_TOGGLE for STATUS   (STATUS phase: DATA1)
	 *
	 * Without QTD_TOGGLE on the DATA qTD the HC sends DATA0 when
	 * the device expects DATA1 → toggle mismatch → device STALLs →
	 * qTD halts with HALT=1, CERR unchanged, no error bits, 0 bytes. */
	{
		uint16_t wlength = ((uint16_t)xfer->setup_pkt[7] << 8) |
				   xfer->setup_pkt[6];
		uint32_t status_pid;

		/* Status phase direction is opposite to the data phase:
		 * IN request (device-to-host) -> status OUT
		 * OUT request (host-to-device) -> status IN */
		status_pid = (xfer->setup_pkt[0] & 0x80) ? QTD_PID_OUT : QTD_PID_IN;

		/* Data phase exists when wLength > 0 and a buffer is
		 * provided.  Note: for IN transfers the buffer starts
		 * empty (buf->len == 0) — the controller fills it — so
		 * we must check wLength, NOT buf->len. */
		if (wlength > 0 && xfer->buf) {
			ret = k_mem_slab_alloc(&priv->qtd_pool,
					       (void **)&data_qtd, K_NO_WAIT);
			if (ret) {
				goto free_status;
			}

			if (USB_EP_DIR_IS_OUT(xfer->ep)) {
				ehci_qtd_fill(data_qtd, xfer->buf->data,
					      xfer->buf->len, QTD_PID_OUT, false);
			} else {
				ehci_qtd_fill(data_qtd, net_buf_tail(xfer->buf),
					      net_buf_tailroom(xfer->buf),
					      QTD_PID_IN, false);
			}
			setup_qtd->next = (uint32_t)(uintptr_t)data_qtd;
		}

		if (data_qtd) {
			ehci_qtd_fill(status_qtd, NULL, 0, status_pid, true);
			/* DATA1 toggle for DATA and STATUS phases. */
			data_qtd->token |= QTD_TOGGLE;
			status_qtd->token |= QTD_TOGGLE;
			data_qtd->next = (uint32_t)(uintptr_t)status_qtd;
			/* alt_next = terminate (matches Linux: short packet
			 * on control IN halts the qTD, handled by software). */
			data_qtd->alt_next = QTD_NEXT_TERMINATE;
		} else {
			ehci_qtd_fill(status_qtd, NULL, 0, status_pid, true);
			/* DATA1 toggle for STATUS phase. */
			status_qtd->token |= QTD_TOGGLE;
			setup_qtd->next = (uint32_t)(uintptr_t)status_qtd;
		}

		/* Store data-phase info for completion byte-counting. */
		qh->data_qtd_sw = data_qtd;
		if (data_qtd) {
			qh->data_initial_len =
				(data_qtd->token >> QTD_LENGTH_SHIFT) & 0x7FFF;
		} else {
			qh->data_initial_len = 0;
		}
	}

	/* Flush ALL qTDs AFTER chaining AND token modification — their
	 * next/alt_next pointers and QTD_TOGGLE bits were modified after
	 * ehci_qtd_fill's initial dcache flush.  Without this, the HC may
	 * read stale values from RAM on non-coherent Cortex-A7. */
	ehci_dcache_flush_range(setup_qtd, sizeof(*setup_qtd));
	ehci_dcache_flush_range(status_qtd, sizeof(*status_qtd));
	if (data_qtd) {
		ehci_dcache_flush_range(data_qtd, sizeof(*data_qtd));
	}

	qh->overlay_next = (uint32_t)(uintptr_t)setup_qtd;
	qh->overlay_alt_next = EHCI_PTR_TERMINATE;
	/* overlay_token must be 0 for a freshly-linked QH.
	 * Linux's qh_update() does: hw_token &= (QTD_TOGGLE | QTD_STS_PING),
	 * which clears it to 0 for a new QH.  Setting HALT here was tested
	 * and caused the Allwinner EHCI to never load the first qTD at all
	 * (qh->current_qtd stayed 0).  With token=0 the HC follows
	 * overlay_next to the first qTD, copies it into the overlay, and
	 * executes it. */
	qh->overlay_token = 0;
	ehci_dcache_flush_range(qh, sizeof(*qh));

	/* Atomically insert QH into the async schedule ring.
	 * irq_lock is required because the UHC layer uses a k_mutex
	 * (not irq_lock), so the EHCI ISR can fire between the read of
	 * async_qh->horiz_link and the write-back.  If the ISR's
	 * ehci_process_async_completions unlinks a completed QH in that
	 * window, our write would overwrite the ISR's link update,
	 * re-inserting a freed QH into the schedule → hardware follows
	 * stale pointers → async schedule corruption. */
	{
		unsigned int key = irq_lock();

		qh->horiz_link = priv->async_qh->horiz_link;
		priv->async_qh->horiz_link =
			(uint32_t)(uintptr_t)qh | EHCI_PTR_TYPE_QH;
		/* Flush BOTH QHs: qh->horiz_link was just written (the
		 * earlier flush at the top of this function wrote a
		 * zeroed/stale horiz_link to RAM).  Without this, the
		 * DMA bus master reads a stale horiz_link from RAM and
		 * follows a bogus pointer when traversing the async
		 * schedule ring past this QH — causing transfers to
		 * stall silently. */
		ehci_dcache_flush_range(qh, sizeof(struct ehci_qh));
		ehci_dcache_flush_range(priv->async_qh,
					sizeof(struct ehci_qh));
		irq_unlock(key);
	}

	/* Ring the Async Schedule Doorbell (IAAD).  Per EHCI §4.8.2,
	 * after modifying the asynchronous schedule software should
	 * write USBCMD.IAAD so the host controller reprocesses the
	 * list.  Some Allwinner EHCI implementations appear to need
	 * this to commit their internal async-schedule state. */
	{
		uint32_t cmd2 = ehci_read32(priv->op_base, EHCI_USBCMD);
		cmd2 |= USBCMD_IAAD;
		ehci_write32(priv->op_base, EHCI_USBCMD, cmd2);
	}


	return 0;

free_status:
	k_mem_slab_free(&priv->qtd_pool, status_qtd);
	if (data_qtd) {
		k_mem_slab_free(&priv->qtd_pool, data_qtd);
	}
free_setup:
	k_mem_slab_free(&priv->qtd_pool, setup_qtd);
free_qh:
	k_mem_slab_free(&priv->qh_pool, qh);
	return -ENOMEM;
}

/* Synchronously drain the async pending-free list.
 * Called from thread context (e.g. enqueue) when the QH slab is
 * exhausted.  Ensures the IAAD doorbell has completed before freeing. */
static void ehci_drain_pending_free(struct uhc_ehci_priv *priv)
{
	struct ehci_qh *qh;

	if (priv->async_pending_free == NULL) {
		return;
	}

	/* If the IAAD interrupt hasn't fired yet, poll for the async
	 * advance completion.  Maximum wait is 2 microframes (250 µs),
	 * but typically the advance has already completed by the time
	 * we reach here (thread context runs after the ISR). */
	if (!(ehci_read32(priv->op_base, EHCI_USBSTS) & USBSTS_IAA)) {
		uint32_t timeout = 500; /* microseconds */

		while (!(ehci_read32(priv->op_base, EHCI_USBSTS) &
			 USBSTS_IAA)) {
			if (--timeout == 0) {
				LOG_WRN("IAAD timeout, freeing QHs anyway");
				break;
			}
			k_busy_wait(1);
		}
	}

	/* Atomically take the list so the ISR can't race with us. */
	unsigned int key = irq_lock();

	qh = priv->async_pending_free;
	priv->async_pending_free = NULL;
	irq_unlock(key);

	while (qh) {
		struct ehci_qh *next = qh->next;

		ehci_free_qtd_chain(priv, qh->qtd_head);
		k_mem_slab_free(&priv->qh_pool, qh);
		qh = next;
	}
}

static int ehci_enqueue_bulk(struct uhc_ehci_priv *priv,
			     struct uhc_transfer *const xfer)
{
	struct ehci_qh *qh;
	struct ehci_qtd *qtd;
	uint32_t device_addr = xfer->udev->addr;
	uint8_t ep_num = USB_EP_GET_IDX(xfer->ep);
	uint32_t pid;
	int ret;

	pid = USB_EP_DIR_IS_IN(xfer->ep) ? QTD_PID_IN : QTD_PID_OUT;

	ret = k_mem_slab_alloc(&priv->qh_pool, (void **)&qh, K_NO_WAIT);
	if (ret && priv->async_pending_free) {
		/* QH slab exhausted but there are deferred-free QHs.
		 * Drain them and retry. */
		ehci_drain_pending_free(priv);
		ret = k_mem_slab_alloc(&priv->qh_pool, (void **)&qh, K_NO_WAIT);
	}
	if (ret) {
		return -ENOMEM;
	}
	memset(qh, 0, sizeof(*qh));

	ret = k_mem_slab_alloc(&priv->qtd_pool, (void **)&qtd, K_NO_WAIT);
	if (ret) {
		k_mem_slab_free(&priv->qh_pool, qh);
		return -ENOMEM;
	}

	{
		uint32_t eps = QH_EP_CHAR_EPS_HIGH;

		if (xfer->udev->speed == USB_SPEED_SPEED_FS) {
			eps = QH_EP_CHAR_EPS_FULL;
		}
		/* Per EHCI §3.6, DTC (Data Toggle Control) is only
		 * applicable to control and interrupt QHs.  For bulk QHs
		 * DTC must be 0 so the HC manages the data toggle
		 * autonomously, starting from the overlay_token's DT bit. */
		qh->ep_char = (device_addr << QH_EP_CHAR_DEVADDR_SHIFT) |
			      (ep_num << QH_EP_CHAR_ENDPT_SHIFT) |
			      eps |
			      (xfer->mps << QH_EP_CHAR_MAX_PKT_LEN_SHIFT);

		if (eps == QH_EP_CHAR_EPS_HIGH) {
			/* RL=4 (NAK count reload) for async bulk, same as
			 * the control path.  Mult=1 (one transaction per
			 * microframe).  Mult=0 is RESERVED per EHCI spec
			 * and causes Allwinner EHCI to silently ignore
			 * the QH — the transfer never executes. */
			qh->ep_char |= (4U << QH_EP_CHAR_RL_SHIFT);
			qh->ep_caps = (1U << QH_EP_CAPS_MULT_SHIFT);
		} else {
			/* FS/LS device behind a HS hub: route through the
			 * hub's Transaction Translator.  Matches the control
			 * path's ep_caps setup. */
			qh->ep_caps = (xfer->udev->hub_addr <<
				       QH_EP_CAPS_HUB_ADDR_SHIFT) |
				      (xfer->udev->hub_port <<
				       QH_EP_CAPS_PORT_SHIFT) |
				      (1U << QH_EP_CAPS_MULT_SHIFT);
		}
	}

	qh->current_qtd = 0;
	qh->qh_dma = (uint32_t)(uintptr_t)qh;
	qh->xfer = xfer;
	qh->qtd_head = qtd;

	if (xfer->buf) {
		if (USB_EP_DIR_IS_OUT(xfer->ep)) {
			ehci_qtd_fill(qtd, xfer->buf->data,
				      xfer->buf->len, pid, true);
			qh->data_qtd_sw = NULL;
			qh->data_initial_len = 0;
		} else {
			uint16_t in_len = net_buf_tailroom(xfer->buf);

			ehci_qtd_fill(qtd, net_buf_tail(xfer->buf),
				      in_len, pid, true);
			/* Track data qTD for IN byte-counting in the
			 * completion handler.  Without this, buf->len is
			 * never updated and the USBH layer sees 0 bytes. */
			qh->data_qtd_sw = qtd;
			qh->data_initial_len = in_len;
		}
	} else {
		ehci_qtd_fill(qtd, NULL, 0, pid, true);
		qh->data_qtd_sw = NULL;
		qh->data_initial_len = 0;
	}

	LOG_DBG("BULK %s: ep=0x%02x mps=%u speed=%u dev=%u dt=%u "
		"buf=%p len=%u ep_char=%08x ep_caps=%08x pid=0x%03x",
		USB_EP_DIR_IS_IN(xfer->ep) ? "IN" : "OUT",
		xfer->ep, xfer->mps, xfer->udev->speed,
		xfer->udev->addr, xfer->data_toggle,
		xfer->buf ? (USB_EP_DIR_IS_IN(xfer->ep)
			      ? net_buf_tail(xfer->buf) : xfer->buf->data)
			  : NULL,
		xfer->buf ? (USB_EP_DIR_IS_IN(xfer->ep)
			      ? net_buf_tailroom(xfer->buf) : xfer->buf->len)
			  : 0,
		qh->ep_char, qh->ep_caps, pid);

	qh->overlay_next = (uint32_t)(uintptr_t)qtd;
	/* overlay_alt_next must be TERMINATE so the controller does not
	 * follow address 0 as an alternate qTD when processing the overlay.
	 * This matches the control-transfer enqueue path. */
	qh->overlay_alt_next = EHCI_PTR_TERMINATE;
	/* overlay_token: ACTIVE must be 0 so the HC follows overlay_next
	 * to the first qTD.  The DT bit (bit 31) seeds the initial data
	 * toggle.  With DTC=0 (hardware-managed toggle for bulk), the HC
	 * starts from this DT value and toggles automatically after each
	 * successful transaction.  xfer->data_toggle is saved by the
	 * completion handler after each transfer so the next transfer on
	 * this endpoint continues the correct toggle sequence. */
	qh->overlay_token = xfer->data_toggle ? QTD_TOGGLE : 0;
	ehci_dcache_flush_range(qh, sizeof(*qh));

	/* Atomically insert QH into the async schedule ring.
	 * See ehci_enqueue_control() for the irq_lock rationale. */
	{
		unsigned int key = irq_lock();

		qh->horiz_link = priv->async_qh->horiz_link;
		priv->async_qh->horiz_link =
			(uint32_t)(uintptr_t)qh | EHCI_PTR_TYPE_QH;
		/* Flush BOTH QHs: qh->horiz_link was just written (the
		 * earlier flush wrote a zeroed/stale horiz_link to RAM).
		 * Without this, the DMA master reads stale horiz_link from
		 * RAM and the async schedule traversal breaks. */
		ehci_dcache_flush_range(qh, sizeof(struct ehci_qh));
		ehci_dcache_flush_range(priv->async_qh,
					sizeof(struct ehci_qh));
		irq_unlock(key);
	}

	/* Ring the Async Schedule Doorbell (IAAD).  Per EHCI §4.8.2,
	 * after modifying the asynchronous schedule software should
	 * write USBCMD.IAAD so the host controller reprocesses the
	 * list.  Allwinner EHCI implementations need this to commit
	 * their internal async-schedule state (same as control path). */
	{
		uint32_t cmd2 = ehci_read32(priv->op_base, EHCI_USBCMD);
		cmd2 |= USBCMD_IAAD;
		ehci_write32(priv->op_base, EHCI_USBCMD, cmd2);
	}

	return 0;
}

/* 
* Interrupt transfers are linked into the periodic schedule instead
* of the async schedule.  The periodic frame list (1024 entries) all
* point to periodic_qh (a dummy QH with HALT overlay).  Interrupt
* QHs are chained after periodic_qh via horiz_link.
*
* For high-speed interrupt endpoints, ep_caps.SMASK determines which
* microframes the controller services the QH.  SMASK=0x01 means
* microframe 0 of every frame (1 ms interval), which is sufficient
* for HID keyboard/mouse polling.
*/

static int ehci_enqueue_interrupt(struct uhc_ehci_priv *priv,
				  struct uhc_transfer *const xfer)
{
	struct ehci_qh *qh;
	struct ehci_qtd *qtd;
	uint32_t device_addr = xfer->udev->addr;
	uint8_t ep_num = USB_EP_GET_IDX(xfer->ep);
	uint32_t pid;
	int ret;

	pid = USB_EP_DIR_IS_IN(xfer->ep) ? QTD_PID_IN : QTD_PID_OUT;

	ret = k_mem_slab_alloc(&priv->qh_pool, (void **)&qh, K_NO_WAIT);
	if (ret) {
		return -ENOMEM;
	}
	memset(qh, 0, sizeof(*qh));

	ret = k_mem_slab_alloc(&priv->qtd_pool, (void **)&qtd, K_NO_WAIT);
	if (ret) {
		k_mem_slab_free(&priv->qh_pool, qh);
		return -ENOMEM;
	}

	{
		uint32_t eps = QH_EP_CHAR_EPS_HIGH;

		if (xfer->udev->speed == USB_SPEED_SPEED_FS) {
			eps = QH_EP_CHAR_EPS_FULL;
		}

		/* Interrupt QH: no RL (that's async only), no C bit
		 * (that's control only).  DTC=1 so hardware manages the
		 * data toggle for each transaction. */
		qh->ep_char = (device_addr << QH_EP_CHAR_DEVADDR_SHIFT) |
			      (ep_num << QH_EP_CHAR_ENDPT_SHIFT) |
			      eps |
			      QH_EP_CHAR_DTC |
			      (xfer->mps << QH_EP_CHAR_MAX_PKT_LEN_SHIFT);
	}

	/* ep_caps: for HS, SMASK=0x01 (service in microframe 0 of each
	 * frame, i.e. 1 ms interval), MULT=1.
	 * For FS/LS behind a HS hub, set up split-transaction scheduling:
	 * S-mask (start-split) in microframe 0, C-mask (complete-split)
	 * in microframes 2/3/4, plus hub address and port for TT routing. */
	if (xfer->udev->speed != USB_SPEED_SPEED_HS) {
		qh->ep_caps = (0x01 << QH_EP_CAPS_SSMASK_SHIFT) |
			      (0x1C << QH_EP_CAPS_SCMASK_SHIFT) |
			      (xfer->udev->hub_addr <<
			       QH_EP_CAPS_HUB_ADDR_SHIFT) |
			      (xfer->udev->hub_port <<
			       QH_EP_CAPS_PORT_SHIFT) |
			      (1U << QH_EP_CAPS_MULT_SHIFT);
	} else {
		qh->ep_caps = (0x01 << QH_EP_CAPS_SSMASK_SHIFT) |
			      (1U << QH_EP_CAPS_MULT_SHIFT);
	}

	qh->current_qtd = 0;
	qh->qh_dma = (uint32_t)(uintptr_t)qh;
	qh->xfer = xfer;
	qh->qtd_head = qtd;
	/* Track data qTD for byte-counting in completion handler. */
	qh->data_qtd_sw = qtd;

	if (xfer->buf) {
		if (USB_EP_DIR_IS_OUT(xfer->ep)) {
			ehci_qtd_fill(qtd, xfer->buf->data,
				      xfer->buf->len, pid, true);
		} else {
			ehci_qtd_fill(qtd, net_buf_tail(xfer->buf),
				      net_buf_tailroom(xfer->buf), pid, true);
			qh->data_initial_len =
				(qtd->token >> QTD_LENGTH_SHIFT) & 0x7FFF;
		}
	} else {
		ehci_qtd_fill(qtd, NULL, 0, pid, true);
		qh->data_initial_len = 0;
	}

	qh->overlay_next = (uint32_t)(uintptr_t)qtd;
	/* Preserve the data toggle across re-enqueues.  With DTC=1 the
	 * hardware reads the initial toggle from the QH overlay.  Setting
	 * it to DATA0 unconditionally (overlay_token=0) would cause ~50%
	 * of interrupt-IN reports to be silently dropped because the device
	 * alternates DATA0/DATA1 while the host always starts at DATA0. */
	qh->overlay_token = xfer->data_toggle ? QTD_TOGGLE : 0;

	/* Also set the toggle on the qTD itself as a belt-and-suspenders
	 * measure: some EHCI implementations may copy the qTD toggle into
	 * the overlay despite DTC=1. */
	if (xfer->data_toggle) {
		qtd->token |= QTD_TOGGLE;
	}

	ehci_dcache_flush_range(qh, sizeof(*qh));
	ehci_dcache_flush_range(qtd, sizeof(*qtd));

	/* Atomically insert QH into the periodic schedule ring.
	 * See ehci_enqueue_control() for the irq_lock rationale. */
	{
		unsigned int key = irq_lock();

		qh->horiz_link = priv->periodic_qh->horiz_link;
		priv->periodic_qh->horiz_link =
			(uint32_t)(uintptr_t)qh | EHCI_PTR_TYPE_QH;
		/* Flush BOTH QHs: qh->horiz_link was just written (the
		 * earlier flush wrote a zeroed/stale horiz_link to RAM). */
		ehci_dcache_flush_range(qh, sizeof(struct ehci_qh));
		ehci_dcache_flush_range(priv->periodic_qh,
					sizeof(struct ehci_qh));
		irq_unlock(key);
	}

	LOG_DBG("intr enqueue: addr=%u ep=%u mps=%u interval=%u",
		device_addr, ep_num, xfer->mps, xfer->interval);

	return 0;
}

/*
 *
 * When the controller finishes processing a QH's qTD chain (the last
 * qTD has IOC=1), it clears the overlay_token ACTIVE bit and asserts
 * USBSTS.USBINT.  This function walks the async schedule, finds
 * completed QHs, invalidates the CPU cache so we read the controller's
 * updated overlay area, removes the QH from the schedule, frees its
 * qTD chain, and returns the transfer to the USBH layer.
 */

static void ehci_free_qtd_chain(struct uhc_ehci_priv *priv,
				struct ehci_qtd *head)
{
	struct ehci_qtd *qtd = head;

	while (qtd && (uint32_t)(uintptr_t)qtd != QTD_NEXT_TERMINATE) {
		struct ehci_qtd *next;

		/* Invalidate so we read the controller's (possibly
		 * untouched) next pointer from RAM. */
		ehci_dcache_invd_range(qtd, sizeof(*qtd));
		next = (struct ehci_qtd *)(uintptr_t)(qtd->next & ~0x1F);
		k_mem_slab_free(&priv->qtd_pool, qtd);
		qtd = next;
	}
}

static void ehci_process_async_completions(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	struct ehci_qh *head = priv->async_qh;
	struct ehci_qh *prev = head;

	/* Walk the async schedule ring (head -> ... -> head).
	 * The head QH has H=1 and a HALT overlay, so it must never be
	 * mistaken for a completed transfer. */
	while (1) {
		/* Invalidate prev so we read its current horiz_link from RAM. */
		ehci_dcache_invd_range(prev, sizeof(*prev));
		uint32_t link = prev->horiz_link;

		struct ehci_qh *qh =
			(struct ehci_qh *)(uintptr_t)(link & ~0x1F);

		/* Back to the head → full lap completed. */
		if (qh == head) {
			break;
		}

		/* Invalidate QH so we read the controller's overlay area. */
		ehci_dcache_invd_range(qh, sizeof(*qh));

		if (!(qh->overlay_token & QTD_STS_ACTIVE)) {
			/* Transfer complete (ACTIVE cleared). */
			struct uhc_transfer *xfer = qh->xfer;
			int err = 0;
			uint32_t token = qh->overlay_token;

			LOG_DBG("async done: ep=0x%02x token=%08x",
				xfer ? xfer->ep : 0, token);

			/* Check for error conditions.
			 * Note: HALT alone is NORMAL — the HC sets it when a
			 * qTD chain finishes and overlay_next is terminate.
			 * Only treat specific error bits as failures. */
			if (token & (QTD_STS_DBE | QTD_STS_BABBLE |
				     QTD_STS_XACT | QTD_STS_MMF)) {
				LOG_ERR("Transfer error: token=%08x", token);
				err = -EIO;
			}

			/* For IN data transfers, compute actual bytes received
			 * from the data qTD's token and update buf->len so the
			 * USBH layer knows how much data is valid. */
			if (xfer && xfer->buf && err == 0 && qh->data_qtd_sw) {
				struct ehci_qtd *dqtd = qh->data_qtd_sw;

				ehci_dcache_invd_range(dqtd, sizeof(*dqtd));
				uint32_t remaining =
					(dqtd->token >> QTD_LENGTH_SHIFT) & 0x7FFF;
				uint32_t transferred =
					qh->data_initial_len - remaining;

				if (USB_EP_DIR_IS_IN(xfer->ep) &&
				    transferred <= xfer->buf->size) {
					/* DMA wrote data starting at buf->data
					 * (buffer was empty for IN).  Set len so
					 * the USBH layer can read the data. */
					xfer->buf->len = transferred;
					ehci_dcache_invd_range(xfer->buf->data,
							       transferred);
					LOG_DBG("IN: %u bytes (init=%u rem=%u)",
						transferred, qh->data_initial_len,
						remaining);
				}
			}

			/* Save the data toggle so the next enqueue on this
			 * endpoint starts with the correct sequence bit.
			 * After a successful transaction the hardware updates
			 * the overlay to the NEXT expected toggle; after an
			 * error the overlay still holds the pre-transaction
			 * value.  Without this, every bulk transfer starts at
			 * DATA0 and the device drops packets after the first
			 * successful exchange (toggle mismatch → NAK forever). */
			if (xfer) {
				xfer->data_toggle =
					(token & QTD_TOGGLE) ? 1 : 0;
			}

			/* Unlink QH from the schedule (keep ring closed). */
			prev->horiz_link = qh->horiz_link;
			ehci_dcache_flush_range(prev, sizeof(*prev));

			LOG_DBG("xfer done: token=%08x err=%d", token, err);

			/* Defer QH/qTD freeing until the IAAD doorbell
			 * handshake completes.  Per EHCI §4.8.2, after
			 * unlinking a QH the host controller may still
			 * be referencing its memory during the current
			 * async schedule traversal.  Freeing immediately
			 * causes a use-after-free race that corrupts
			 * subsequent transfers.  The QH is added to a
			 * singly-linked pending-free list (via the
			 * software `next` pointer); the IAAD doorbell
			 * is rung after the walk, and the ISR frees the
			 * list when USBSTS_IAA fires. */
			qh->next = priv->async_pending_free;
			priv->async_pending_free = qh;

			/* Return the transfer to the USBH layer.
			 * Do NOT advance prev — it now links to the
			 * next QH (or back to head). */
			if (xfer) {
				uhc_xfer_return(dev, xfer, err);
			}
		} else {
			/* Still active, advance to next QH. */
			prev = qh;
		}
	}

	/* If any QHs were removed during this walk, ring the IAAD
		* doorbell.  The host controller will complete one async
		* schedule advance and assert USBSTS_IAA, at which point
		* the ISR safely frees the deferred QHs. */
	if (priv->async_pending_free) {
		uint32_t cmd = ehci_read32(priv->op_base, EHCI_USBCMD);
		cmd |= USBCMD_IAAD;
		ehci_write32(priv->op_base, EHCI_USBCMD, cmd);
	}
}

/*
 * Walk the periodic QH chain (linked after periodic_qh) and process
 * completed interrupt transfers.  A QH is "done" when the
 * overlay_token ACTIVE bit is cleared.  The QH is unlinked from the
 * periodic chain and the transfer is returned to the upper layer.
 * The upper layer (HID class driver) resubmits to continue polling.
 */
static void ehci_process_periodic_completions(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	struct ehci_qh *head = priv->periodic_qh;
	struct ehci_qh *prev = head;

	ehci_dcache_invd_range(head, sizeof(*head));

	while (1) {
		uint32_t link = prev->horiz_link;
		struct ehci_qh *qh =
			(struct ehci_qh *)(uintptr_t)(link & ~0x1F);

		if (qh == NULL || (link & ~0x1F) == 0) {
			break;
		}

		ehci_dcache_invd_range(qh, sizeof(*qh));

		if (!(qh->overlay_token & QTD_STS_ACTIVE)) {
			struct uhc_transfer *xfer = qh->xfer;
			int err = 0;
			uint32_t token = qh->overlay_token;
			bool retry_in_place = false;

			/*
			 * Retry XACT errors in-place by resetting the overlay
			 * token, avoiding unlink/re-enqueue churn on every
			 * transient error.  This mirrors Linux's qh_completions()
			 * which retries up to QH_XACTERR_MAX (32) times before
			 * giving up and returning the URB with -EPROTO.
			 *
			 * Each software retry lets the hardware attempt 3
			 * transactions (CERR=3), so total attempts are
			 * EHCI_XACTERR_MAX * 3 ≈ 96 before declaring failure.
			 */
			if ((token & QTD_STS_XACT) &&
			    qh->xacterrs < EHCI_XACTERR_MAX) {
				qh->xacterrs++;
				token &= ~(QTD_STS_HALT | QTD_STS_XACT |
					   QTD_STS_DBE | QTD_STS_BABBLE |
					   QTD_STS_MMF);
				token |= QTD_STS_ACTIVE | QTD_CERR_MASK;
				qh->overlay_token = token;
				if (qh->data_qtd_sw != NULL) {
					qh->data_qtd_sw->token = token;
					ehci_dcache_flush_range(
						qh->data_qtd_sw,
						sizeof(struct ehci_qtd));
				}
				ehci_dcache_flush_range(qh, sizeof(*qh));
				retry_in_place = true;
			}

			if (retry_in_place) {
				/* Leave QH in the periodic schedule; the
				 * hardware retries on the next service
				 * interval. */
				prev = qh;
			} else {
				if (token & (QTD_STS_DBE | QTD_STS_BABBLE |
					     QTD_STS_XACT | QTD_STS_MMF)) {
					LOG_DBG("Intr xfer error: token=%08x",
						token);
					err = -EIO;
				}

				/* Reset retry counter on completion. */
				qh->xacterrs = 0;

				if (xfer && xfer->buf && err == 0 &&
				    qh->data_qtd_sw &&
				    qh->data_initial_len > 0) {
					struct ehci_qtd *dqtd = qh->data_qtd_sw;

					ehci_dcache_invd_range(
						dqtd, sizeof(*dqtd));
					uint32_t remaining =
						(dqtd->token >>
						 QTD_LENGTH_SHIFT) & 0x7FFF;
					uint32_t transferred =
						qh->data_initial_len - remaining;

					if (USB_EP_DIR_IS_IN(xfer->ep) &&
					    transferred <= xfer->buf->size) {
						xfer->buf->len = transferred;
						ehci_dcache_invd_range(
							xfer->buf->data,
							transferred);
						LOG_DBG("intr IN: %u bytes",
							transferred);
					}
				}
	
					/* Save the data toggle so the next enqueue
					 * starts with the correct sequence bit.
					 * After a successful transaction the hardware
					 * updates the overlay to the NEXT expected
					 * toggle; after an error the overlay still
					 * holds the pre-transaction value, which is
					 * also correct because no data was exchanged. */
					if (xfer) {
						xfer->data_toggle =
							(token & QTD_TOGGLE) ? 1 : 0;
					}
	
					prev->horiz_link = qh->horiz_link;
					ehci_dcache_flush_range(prev, sizeof(*prev));
	
					ehci_free_qtd_chain(priv, qh->qtd_head);
					k_mem_slab_free(&priv->qh_pool, qh);
	
					if (xfer) {
						uhc_xfer_return(dev, xfer, err);
					}
			}
		} else {
			prev = qh;
		}

		ehci_dcache_invd_range(prev, sizeof(*prev));
		if ((prev->horiz_link & ~0x1F) == 0) {
			break;
		}
	}
}

volatile uint32_t ehci_isr_count;

static void ehci_isr_handler(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t sts;

	sts = ehci_read32(priv->op_base, EHCI_USBSTS);
	if (!sts) {
		return;
	}

	ehci_isr_count++;

	/* Clear interrupt status bits. */
	ehci_write32(priv->op_base, EHCI_USBSTS, sts);

	/* Process IAA BEFORE USBINT/USBERRINT.  If both are set in the
	 * same ISR, we must free QHs from the PREVIOUS doorbell before
	 * the completion handler adds new QHs to the pending-free list.
	 * Otherwise the new QHs (whose doorbell hasn't been rung yet)
	 * would be incorrectly freed by the stale IAA. */
	if (sts & USBSTS_IAA) {
		/* Interrupt on Async Advance — the host controller has
		 * completed a full traversal of the async schedule after
		 * the IAAD doorbell was rung.  Per EHCI §4.8.2, it is
		 * now safe to free the memory of QHs that were unlinked
		 * by the completion handler. */
		struct ehci_qh *qh = priv->async_pending_free;

		priv->async_pending_free = NULL;
		while (qh) {
			struct ehci_qh *next = qh->next;

			ehci_free_qtd_chain(priv, qh->qtd_head);
			k_mem_slab_free(&priv->qh_pool, qh);
			qh = next;
		}
	}

	if (sts & USBSTS_USBINT) {
		ehci_process_async_completions(dev);
		ehci_process_periodic_completions(dev);
	}
	if (sts & USBSTS_USBERRINT) {
		ehci_process_async_completions(dev);
		ehci_process_periodic_completions(dev);
	}

	if (sts & USBSTS_PCD) {
		LOG_DBG("PCD (port change)");
		for (int i = 0; i < priv->n_ports; i++) {
			uint32_t portsc;

			portsc = ehci_read32(priv->op_base,
					     EHCI_PORTSC_BASE + (i * 4));

			if (portsc & PORTSC_CSC) {
				/* Write only CSC (bit 1) to clear it.
				 * Writing the full PORTSC value would also
				 * clear PE (bit 2, write-1-to-clear) and
				 * could corrupt other writable bits. */
				ehci_write32(priv->op_base,
					     EHCI_PORTSC_BASE + (i * 4),
					     PORTSC_CSC);

				if (portsc & PORTSC_CCS) {
					uint32_t ls;

					ls = (portsc & PORTSC_LS_MASK) >> 10;
					LOG_INF("Device connected port %u LS=%u",
						i, ls);

					if (ls == 1) {
						uhc_submit_event(dev,
							UHC_EVT_DEV_CONNECTED_LS, 0);
					} else {
						/* On EHCI, a device that is not
						 * low-speed will be high-speed if it
						 * remains on this controller after
						 * reset (PE=1).  Full/low-speed
						 * devices are released to the
						 * companion controller (OHCI), which
						 * is not present on this board.
						 * Report HS so the QH uses EPS_HIGH. */
						uhc_submit_event(dev,
							UHC_EVT_DEV_CONNECTED_HS, 0);
					}
				} else {
					LOG_INF("Device removed port %u", i);
					uhc_submit_event(dev,
						UHC_EVT_DEV_REMOVED, 0);
				}
			}
		}
	}

	if (sts & USBSTS_HSE) {
		LOG_ERR("Host System Error");
		uhc_submit_event(dev, UHC_EVT_ERROR, -EIO);
	}

	if (sts & USBSTS_FLR) {
		LOG_DBG("FLR");
	}
}

static int uhc_ehci_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_ehci_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

static int uhc_ehci_init(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	int ret;

	LOG_INF("EHCI init");

	ret = ehci_platform_init(dev);
	if (ret) {
		return ret;
	}

	ret = ehci_mem_init(uhc_get_private(dev));
	if (ret) {
		return ret;
	}

	ret = ehci_hw_reset(dev);
	if (ret) {
		return ret;
	}

	ret = ehci_hw_setup(dev);
	if (ret) {
		return ret;
	}

	/* Dump controller state after setup. At this point the
	 * controller is still halted (RS=0); hw_run is called later from
	 * uhc_ehci_enable(). Port power (PP) should already be 1.
	 */
	LOG_INF("EHCI configured. Post-setup register dump:");
	LOG_INF("  USBCMD    =%08x", ehci_read32(priv->op_base, EHCI_USBCMD));
	LOG_INF("  USBSTS    =%08x", ehci_read32(priv->op_base, EHCI_USBSTS));
	LOG_INF("  USBINTR   =%08x", ehci_read32(priv->op_base, EHCI_USBINTR));
	LOG_INF("  CONFIGFLAG=%08x", ehci_read32(priv->op_base, EHCI_CONFIGFLAG));
	for (int i = 0; i < priv->n_ports; i++) {
		uint32_t psc = ehci_read32(priv->op_base,
					    EHCI_PORTSC_BASE + (i * 4));
		LOG_INF("  PORTSC[%d] =%08x (CCS=%u PP=%u PE=%u CSC=%u)",
			i, psc,
			(psc & PORTSC_CCS) ? 1 : 0,
			(psc & PORTSC_PP) ? 1 : 0,
			(psc & PORTSC_PE) ? 1 : 0,
			(psc & PORTSC_CSC) ? 1 : 0);
	}

	return 0;
}

static int uhc_ehci_enable(const struct device *dev)
{
	const struct uhc_ehci_config *cfg = dev->config;
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	int ret;

	/* Start the controller (RS=1, ASE=1, PSE=1).  This was deferred
	 * from .init so that the controller only begins scanning ports
	 * after the USBH event threads and callback are fully set up. */
	ret = ehci_hw_run(dev);
	if (ret) {
		return ret;
	}

	/* Give the PHY a moment to detect an already-connected device. */
	k_busy_wait(100);

	cfg->irq_enable_fn(dev);

	/* Dump post-enable state. */
	for (int i = 0; i < priv->n_ports; i++) {
		uint32_t psc = ehci_read32(priv->op_base,
					    EHCI_PORTSC_BASE + (i * 4));
		LOG_INF("PORTSC[%d]=%08x (CCS=%u PP=%u PE=%u CSC=%u)",
			i, psc,
			(psc & PORTSC_CCS) ? 1 : 0,
			(psc & PORTSC_PP) ? 1 : 0,
			(psc & PORTSC_PE) ? 1 : 0,
			(psc & PORTSC_CSC) ? 1 : 0);
	}
	return 0;
}

static int uhc_ehci_disable(const struct device *dev)
{
	const struct uhc_ehci_config *cfg = dev->config;

	cfg->irq_disable_fn(dev);
	LOG_DBG("EHCI disabled");
	return 0;
}

static int uhc_ehci_shutdown(const struct device *dev)
{
	const struct uhc_ehci_config *cfg = dev->config;

	cfg->irq_disable_fn(dev);
	ehci_hw_stop(dev);
	LOG_DBG("EHCI shutdown");
	return 0;
}

static int uhc_ehci_bus_reset(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t portsc;

	LOG_DBG("bus_reset: asserting port reset (PR)");

	/* Read current PORTSC.  Preserve port power (PP), assert reset (PR),
	 * and clear PE.  Mask out write-1-to-clear status bits (CSC/PEC/OCC)
	 * so we don't inadvertently clear them. */
	portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
	portsc &= ~(PORTSC_CSC | PORTSC_PEC | PORTSC_OCC | PORTSC_PE);
	portsc |= (PORTSC_PR | PORTSC_PP);
	ehci_write32(priv->op_base, EHCI_PORTSC_BASE, portsc);

	/* Hold the reset for the standard SE0 duration. */
	k_msleep(EHCI_PORT_RESET_MS);

	LOG_DBG("bus_reset: PORTSC during PR=%08x",
		ehci_read32(priv->op_base, EHCI_PORTSC_BASE));

	/* On this controller PR does NOT self-clear — software must
	 * de-assert it after the reset duration (same as the Linux hub
	 * thread).  The HC sets PE=1 for a high/full-speed device when PR
	 * transitions 1->0.  Mask out w1c status bits to avoid clearing
	 * them, and keep PP on. */
	portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
	portsc &= ~(PORTSC_PR | PORTSC_CSC | PORTSC_PEC | PORTSC_OCC);
	portsc |= PORTSC_PP;
	ehci_write32(priv->op_base, EHCI_PORTSC_BASE, portsc);

	k_usleep(100); /* small delay for chirp to begin */

	/* Poll briefly for the HC to set PE after reset de-assertion. */
	for (int i = 0; i < 10; i++) {
		portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
		if (portsc & PORTSC_PE) {
			LOG_DBG("bus_reset: PE set after %d polls", i + 1);
			break;
		}
		k_msleep(2);
	}

	if (!(portsc & PORTSC_PE)) {
		LOG_WRN("bus_reset: PE NOT SET after reset! "
			"PORTSC=%08x (device may be full/low-speed)",
			portsc);
	}

	LOG_INF("bus_reset done: PE=%u CCS=%u",
		(portsc & PORTSC_PE) ? 1 : 0,
		(portsc & PORTSC_CCS) ? 1 : 0);

	/* USB 2.0 spec requires the device to be given recovery time
	 * (tDRSTR ≥ 3 ms) before the first transaction after reset.
	 * Allwinner PHY/HC needs more generous recovery — use 100ms. */
	k_msleep(100);

	LOG_DBG("Bus reset completed");
	uhc_submit_event(dev, UHC_EVT_RESETED, 0);
	return 0;
}

static int uhc_ehci_sof_enable(const struct device *dev)
{
	return 0;
}

static int uhc_ehci_bus_suspend(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t portsc;

	portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
	portsc |= PORTSC_SUSPEND;
	ehci_write32(priv->op_base, EHCI_PORTSC_BASE, portsc);

	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);
	return 0;
}

static int uhc_ehci_bus_resume(const struct device *dev)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	uint32_t portsc;

	portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
	portsc |= PORTSC_FPR;
	ehci_write32(priv->op_base, EHCI_PORTSC_BASE, portsc);

	k_msleep(20);

	portsc = ehci_read32(priv->op_base, EHCI_PORTSC_BASE);
	portsc &= ~PORTSC_FPR;
	ehci_write32(priv->op_base, EHCI_PORTSC_BASE, portsc);

	uhc_submit_event(dev, UHC_EVT_RESUMED, 0);
	return 0;
}

static int uhc_ehci_ep_enqueue(const struct device *dev,
			       struct uhc_transfer *const xfer)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);

	uhc_xfer_append(dev, xfer);

	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		return ehci_enqueue_control(priv, xfer);
	}

	if (xfer->type == USB_EP_TYPE_INTERRUPT) {
		return ehci_enqueue_interrupt(priv, xfer);
	}

	return ehci_enqueue_bulk(priv, xfer);
}

static int uhc_ehci_ep_dequeue(const struct device *dev,
			       struct uhc_transfer *const xfer)
{
	struct uhc_ehci_priv *priv = uhc_get_private(dev);
	struct ehci_qh *head = priv->async_qh;
	struct ehci_qh *prev = head;
	bool found = false;

	/* Disable interrupts to prevent the ISR's completion handler
	 * from modifying the async/periodic schedule linked list while
	 * we walk and unlink.  The UHC layer uses a k_mutex (not
	 * irq_lock), so without this protection the ISR can corrupt
	 * the schedule by unlinking a QH between our read and write of
	 * a horiz_link pointer. */
	unsigned int key = irq_lock();

	/* Walk the circular async schedule to find and remove the QH
	 * that matches this transfer.  The head (H=1) is never a
	 * transfer QH. */
	while (1) {
		ehci_dcache_invd_range(prev, sizeof(*prev));
		uint32_t link = prev->horiz_link;

		struct ehci_qh *qh =
			(struct ehci_qh *)(uintptr_t)(link & ~0x1F);

		/* Back to head → full lap, transfer not found. */
		if (qh == head) {
			break;
		}

		ehci_dcache_invd_range(qh, sizeof(*qh));

		if (qh->xfer == xfer) {
			/* Found — unlink from the schedule. */
			prev->horiz_link = qh->horiz_link;
			ehci_dcache_flush_range(prev, sizeof(*prev));

			/* Defer QH freeing via the IAAD handshake (same
			 * mechanism as ehci_process_async_completions).
			 * Per EHCI §4.8.2, the host controller may still
			 * be referencing the unlinked QH during its
			 * current async schedule traversal.  Freeing
			 * immediately causes a use-after-free race. */
			qh->next = priv->async_pending_free;
			priv->async_pending_free = qh;
			found = true;
			break;
		}
		prev = qh;
	}

	if (!found) {
		/* Check periodic schedule. */
		struct ehci_qh *phead = priv->periodic_qh;
		struct ehci_qh *pprev = phead;

		ehci_dcache_invd_range(phead, sizeof(*phead));

		while (1) {
			uint32_t link = pprev->horiz_link;
			struct ehci_qh *qh =
				(struct ehci_qh *)(uintptr_t)(link & ~0x1F);

			if (qh == NULL || (link & ~0x1F) == 0) {
				break;
			}

			ehci_dcache_invd_range(qh, sizeof(*qh));

			if (qh->xfer == xfer) {
				pprev->horiz_link = qh->horiz_link;
				ehci_dcache_flush_range(pprev,
							sizeof(*pprev));

				/* Periodic schedule: safe to free
				 * immediately since the QH is only
				 * serviced at its SMASK interval and we
				 * hold irq_lock. */
				ehci_free_qtd_chain(priv, qh->qtd_head);
				k_mem_slab_free(&priv->qh_pool, qh);
				found = true;
				break;
			}
			pprev = qh;
		}
	}

	irq_unlock(key);

	if (found) {
		/* If we deferred an async QH, ring the IAAD doorbell so
		 * the ISR can safely free it after the next async
		 * schedule advance. */
		if (priv->async_pending_free) {
			uint32_t cmd = ehci_read32(priv->op_base,
						    EHCI_USBCMD);
			cmd |= USBCMD_IAAD;
			ehci_write32(priv->op_base, EHCI_USBCMD, cmd);
		}

		uhc_xfer_return(dev, xfer, -ECANCELED);
		return 0;
	}

	return -EINVAL;
}

static const struct uhc_api ehci_uhc_api = {
	.lock		= uhc_ehci_lock,
	.unlock		= uhc_ehci_unlock,
	.init		= uhc_ehci_init,
	.enable		= uhc_ehci_enable,
	.disable	= uhc_ehci_disable,
	.shutdown	= uhc_ehci_shutdown,
	.bus_reset	= uhc_ehci_bus_reset,
	.sof_enable	= uhc_ehci_sof_enable,
	.bus_suspend	= uhc_ehci_bus_suspend,
	.bus_resume	= uhc_ehci_bus_resume,
	.ep_enqueue	= uhc_ehci_ep_enqueue,
	.ep_dequeue	= uhc_ehci_ep_dequeue,
};

static void uhc_ehci_isr(const struct device *dev)
{
	ehci_isr_handler(dev);
}

#define UHC_EHCI_DEVICE_DEFINE(n)						\
	/* Declare the sunxi PHY pseudo-device extern when this instance	\
	 * references a PHY, so the config initializer can take its addr.	\
	 */									\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, phys),				\
		(_SUNXI_PHY_PSEUDODEV_DECLARE(DT_DRV_INST(n));), ())		\
	static void ehci_irq_enable_##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    uhc_ehci_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void ehci_irq_disable_##n(const struct device *dev)		\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static struct uhc_ehci_config ehci_config_##n = {			\
		.reg_base = DT_INST_REG_ADDR(n),				\
		.irq_enable_fn = ehci_irq_enable_##n,				\
		.irq_disable_fn = ehci_irq_disable_##n,				\
		.clock_dev =							\
			DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),			\
		.clock_subsys = {						\
			DT_INST_FOREACH_PROP_ELEM(n, clocks, EHCI_CLK_ELEM)	\
		},								\
		.num_clocks = DT_INST_PROP_LEN(n, clocks),			\
		.reset_dev =							\
			DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(n, resets, 0)),	\
		.reset_ids = {							\
			DT_INST_FOREACH_PROP_ELEM(n, resets, EHCI_RST_ELEM)	\
		},								\
		.num_resets = DT_INST_PROP_LEN(n, resets),			\
		.phy = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, phys),		\
			(USB_SUNXI_PHY_PSEUDODEV_GET_OR_NULL(			\
				DT_DRV_INST(n))),				\
			(NULL)),						\
	};									\
										\
	static struct uhc_ehci_priv ehci_priv_##n;				\
										\
	static struct uhc_data uhc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(uhc_data_##n.mutex),		\
		.priv = &ehci_priv_##n,						\
	};									\
										\
	static int ehci_init_##n(const struct device *dev)			\
	{									\
		/* Boot-time init: enable clocks, release resets, power up	\
		 * PHY and read capability registers (CAPLENGTH, HCSPARAMS).	\
		 * Full controller setup (hw_reset, hw_setup) is deferred to	\
		 * the .init API callback invoked by uhc_init(), and hw_run	\
		 * to the .enable API callback (uhc_enable).  This follows	\
		 * the standard Zephyr UHC driver lifecycle and ensures the	\
		 * USBH event callback is registered before the controller	\
		 * starts generating port-change interrupts. */			\
		LOG_INF("EHCI instance " #n " boot init (platform only)");	\
		return ehci_platform_init(dev);					\
	}									\
	DEVICE_DT_INST_DEFINE(n, ehci_init_##n, NULL,				\
			      &uhc_data_##n, &ehci_config_##n,			\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &ehci_uhc_api);

DT_INST_FOREACH_STATUS_OKAY(UHC_EHCI_DEVICE_DEFINE)
