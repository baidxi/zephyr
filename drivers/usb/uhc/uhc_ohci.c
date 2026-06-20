/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Generic OHCI USB Host Controller Driver for Zephyr.
 */

#define DT_DRV_COMPAT generic_ohci

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/usb/uhc.h>

#include "uhc_common.h"
#include "uhc_ohci.h"

/* Sunxi USB PHY pseudo-device support (optional, see ehci driver). */
#include "../common/sunxi/usb_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_ohci, CONFIG_UHC_DRIVER_LOG_LEVEL);

/* ================================================================
 * Private Data
 * ================================================================ */

struct uhc_ohci_priv {
	uintptr_t reg_base;

	/* HCCA (256-byte aligned).  Use static buffers instead of
	 * k_aligned_alloc() so the driver does not depend on the system
	 * heap (CONFIG_HEAP_MEM_POOL_SIZE) being configured and the
	 * allocation can never fail at boot init time.
	 */
	struct ohci_hcca __aligned(256) hcca_buf;
	struct ohci_hcca *hcca;

	/* Endpoint Descriptor pools (16-byte aligned EDs, 32-byte TDs) */
	struct ohci_ed __aligned(16) ed_pool_buf[OHCI_MAX_ED];
	struct ohci_td __aligned(32) td_pool_buf[OHCI_MAX_TD];
	struct ohci_ed *ed_pool;
	struct ohci_td *td_pool;
	uint8_t ed_count;
	uint8_t td_count;

	/* Lists */
	struct ohci_ed *ctrl_head;
	struct ohci_ed *bulk_head;

	/* Current state */
	uint8_t n_ports;
};

#define UHC_OHCI_MAX_CLOCKS	4
#define UHC_OHCI_MAX_RESETS	4

#define OHCI_CLK_ELEM(node_id, prop, idx) \
	(clock_control_subsys_t)(uintptr_t)DT_PHA_BY_IDX(node_id, clocks, idx, clk_id),
#define OHCI_RST_ELEM(node_id, prop, idx) \
	(uint32_t)DT_PHA_BY_IDX(node_id, resets, idx, id),

struct uhc_ohci_config {
	uintptr_t reg_base;
	void (*irq_enable_fn)(const struct device *dev);
	void (*irq_disable_fn)(const struct device *dev);

	/* Platform clocks / resets / PHY (mirrors the EHCI driver). */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys[UHC_OHCI_MAX_CLOCKS];
	uint8_t num_clocks;
	const struct device *reset_dev;
	uint32_t reset_ids[UHC_OHCI_MAX_RESETS];
	uint8_t num_resets;
	const struct sunxi_usb_phy *phy;
};

/* ================================================================
 * Register I/O
 * ================================================================ */

static inline uint32_t ohci_read32(uintptr_t base, uint32_t offset)
{
	return sys_read32(base + offset);
}

static inline void ohci_write32(uintptr_t base, uint32_t offset, uint32_t val)
{
	sys_write32(val, base + offset);
}

/* ================================================================
 * Controller Lifecycle
 * ================================================================ */

static int ohci_hw_reset(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	uint32_t val;
	int timeout;

	/* Software reset via HcCommandStatus.HCR */
	val = ohci_read32(priv->reg_base, OHCI_HC_CMDSTATUS);
	val |= OHCI_CMDS_HCR;
	ohci_write32(priv->reg_base, OHCI_HC_CMDSTATUS, val);

	/* Wait up to 10ms for reset to complete */
	timeout = 10000;
	while (ohci_read32(priv->reg_base, OHCI_HC_CMDSTATUS) & OHCI_CMDS_HCR) {
		if (--timeout == 0) {
			LOG_ERR("OHCI reset timeout");
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}

	return 0;
}

static int ohci_hw_setup(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	uint32_t rh_desc_a;

	/* Program HCCA base address */
	ohci_write32(priv->reg_base, OHCI_HC_HCCA,
		     (uint32_t)(uintptr_t)priv->hcca);
	memset(priv->hcca, 0, sizeof(*priv->hcca));

	/* Set frame interval (nominal: 11999 bit times) */
	ohci_write32(priv->reg_base, OHCI_HC_FMINTERVAL,
		     (1U << 31) | ((OHCI_FM_INTERVAL - 210) * 6 / 7 << 16) |
		     OHCI_FM_INTERVAL);

	/* Set periodic start (~90% of frame) */
	ohci_write32(priv->reg_base, OHCI_HC_PERIODICSTART,
		     OHCI_PERIODIC_START);

	/* Set LS threshold */
	ohci_write32(priv->reg_base, OHCI_HC_LSTHRESHOLD,
		     OHCI_LS_THRESHOLD);

	/* Read number of ports from Root Hub Descriptor A */
	rh_desc_a = ohci_read32(priv->reg_base, OHCI_HC_RHDESCA);
	priv->n_ports = rh_desc_a & 0xFF;
	LOG_INF("OHCI %u port(s), RHDESCA=%08x", priv->n_ports, rh_desc_a);

	/* Set up control list */
	priv->ctrl_head = &priv->ed_pool[0];
	memset(priv->ctrl_head, 0, sizeof(*priv->ctrl_head));
	priv->ctrl_head->hw_headp = ED_H; /* Halted */
	priv->ctrl_head->hw_tailp = 0;
	ohci_write32(priv->reg_base, OHCI_HC_CTRL_HEADED,
		     (uint32_t)(uintptr_t)priv->ctrl_head);

	/* Set up bulk list */
	priv->bulk_head = &priv->ed_pool[1];
	memset(priv->bulk_head, 0, sizeof(*priv->bulk_head));
	priv->bulk_head->hw_headp = ED_H;
	priv->bulk_head->hw_tailp = 0;
	ohci_write32(priv->reg_base, OHCI_HC_BULK_HEADED,
		     (uint32_t)(uintptr_t)priv->bulk_head);

	/* Set up periodic list (all entries point to HCCA int_table[0]) */
	for (int i = 0; i < 32; i++) {
		priv->hcca->int_table[i] = 0;
	}

	/* Enable interrupts */
	ohci_write32(priv->reg_base, OHCI_HC_INTSTATUS,
		     ohci_read32(priv->reg_base, OHCI_HC_INTSTATUS));
	ohci_write32(priv->reg_base, OHCI_HC_INTENABLE,
		     OHCI_INTR_MIE | OHCI_INTR_WDH |
		     OHCI_INTR_RHSC | OHCI_INTR_UE);

	/* Set host controller functional state: USBOperational */
	ohci_write32(priv->reg_base, OHCI_HC_CONTROL,
		     OHCI_CTRL_HCFS_OPER |
		     OHCI_CTRL_CBSR_MASK | /* 4:1 control/bulk ratio */
		     OHCI_CTRL_CLE |       /* Control list enable */
		     OHCI_CTRL_BLE |       /* Bulk list enable */
		     OHCI_CTRL_PLE);       /* Periodic list enable */

	/* Set port power on all ports */
	for (int i = 0; i < priv->n_ports; i++) {
		uint32_t port_status;

		port_status = ohci_read32(priv->reg_base,
					  OHCI_HC_RHPORTSTATUS + (i * 4));
		port_status |= OHCI_RHPS_PPS; /* Set port power */
		ohci_write32(priv->reg_base, OHCI_HC_RHPORTSTATUS + (i * 4),
			     port_status);
	}

	return 0;
}

/* ================================================================
 * Interrupt Handler
 * ================================================================ */

static void ohci_isr_handler(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	uint32_t int_status, int_enable;

	int_status = ohci_read32(priv->reg_base, OHCI_HC_INTSTATUS);

	if (!int_status) {
		return;
	}

	int_enable = ohci_read32(priv->reg_base, OHCI_HC_INTENABLE);

	/* Acknowledge only enabled interrupts */
	ohci_write32(priv->reg_base, OHCI_HC_INTSTATUS,
		     int_status & int_enable);

	/* Writeback Done Head — transfer completion */
	if (int_status & OHCI_INTR_WDH) {
		uint32_t done_head;

		done_head = priv->hcca->done_head;
		LOG_DBG("WDH done_head=%08x", done_head);

		/* Process done queue */
		while (done_head) {
			struct ohci_td *td;

			td = (struct ohci_td *)(uintptr_t)(done_head & ~0x1F);
			done_head = td->hw_nexttd;
		}

		/* Update done head */
		priv->hcca->done_head = 0;
		ohci_write32(priv->reg_base, OHCI_HC_DONEHEAD, 0);
	}

	/* Root Hub Status Change */
	if (int_status & OHCI_INTR_RHSC) {
		LOG_DBG("RHSC");

		/* Check each port */
		for (int i = 0; i < priv->n_ports; i++) {
			uint32_t port_status;

			port_status = ohci_read32(priv->reg_base,
						  OHCI_HC_RHPORTSTATUS + (i * 4));

			if (port_status & OHCI_RHPS_CSC) {
				/* Clear CSC */
				ohci_write32(priv->reg_base,
					     OHCI_HC_RHPORTSTATUS + (i * 4),
					     port_status);

				if (port_status & OHCI_RHPS_CCS) {
					LOG_INF("Device connected port %u", i);

					if (port_status & OHCI_RHPS_LSDA) {
						uhc_submit_event(dev,
							UHC_EVT_DEV_CONNECTED_LS, 0);
					} else {
						uhc_submit_event(dev,
							UHC_EVT_DEV_CONNECTED_FS, 0);
					}
				} else {
					LOG_INF("Device removed port %u", i);
					uhc_submit_event(dev,
						UHC_EVT_DEV_REMOVED, 0);
				}
			}
		}
	}

	/* Unrecoverable Error */
	if (int_status & OHCI_INTR_UE) {
		LOG_ERR("Unrecoverable Error");
		uhc_submit_event(dev, UHC_EVT_ERROR, -EIO);
	}
}

/* ================================================================
 * UHC API
 * ================================================================ */

static int uhc_ohci_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_ohci_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

static int ohci_platform_init(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	const struct uhc_ohci_config *cfg = dev->config;
	int ret;

	priv->reg_base = cfg->reg_base;

	/* ---- Platform bring-up: clocks, resets, PHY ----
	 * Idempotent: safe to call from both boot init and .init API. */
	LOG_INF("OHCI platform init: %u clock(s), %u reset(s), phy=%s",
		cfg->num_clocks, cfg->num_resets, cfg->phy ? "yes" : "no");

	for (uint8_t i = 0; i < cfg->num_clocks; i++) {
		ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys[i]);
		if (ret < 0 && ret != -ENOTSUP) {
			LOG_ERR("clock_control_on[%u] failed: %d", i, ret);
			return ret;
		}
		LOG_INF("  clock[%u] subsys=0x%lx -> %s", i,
			(uintptr_t)cfg->clock_subsys[i],
			(ret == -ENOTSUP) ? "always-on" : "ok");
	}

	for (uint8_t i = 0; i < cfg->num_resets; i++) {
		ret = reset_line_deassert(cfg->reset_dev, cfg->reset_ids[i]);
		if (ret < 0) {
			LOG_ERR("reset_line_deassert[%u] failed: %d", i, ret);
			return ret;
		}
		LOG_INF("  reset[%u] id=0x%x deasserted", i, cfg->reset_ids[i]);
	}

	if (cfg->phy != NULL) {
		LOG_INF("  enabling PHY (index=%u)", cfg->phy->index);
		ret = cfg->phy->enable(cfg->phy);
		if (ret) {
			LOG_ERR("PHY enable failed: %d", ret);
			return ret;
		}
	}

	k_busy_wait(10);
	return 0;
}

static int uhc_ohci_init(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	int ret;

	printk("[ohci] init start (dev=%p reg_base=%p)\n",
		(void *)dev, (void *)priv->reg_base);

	ret = ohci_platform_init(dev);
	if (ret) {
		return ret;
	}

	/* Use static, properly-aligned buffers (no heap dependency). */
	priv->hcca = &priv->hcca_buf;
	memset(priv->hcca, 0, sizeof(struct ohci_hcca));

	priv->ed_count = OHCI_MAX_ED;
	priv->ed_pool = priv->ed_pool_buf;
	memset(priv->ed_pool, 0, OHCI_MAX_ED * sizeof(struct ohci_ed));

	priv->td_count = OHCI_MAX_TD;
	priv->td_pool = priv->td_pool_buf;
	memset(priv->td_pool, 0, OHCI_MAX_TD * sizeof(struct ohci_td));

	printk("[ohci] step 1/2: hw_reset\n");
	ret = ohci_hw_reset(dev);
	printk("[ohci] step 1/2: hw_reset -> %d\n", ret);
	if (ret) {
		return ret;
	}

	printk("[ohci] step 2/2: hw_setup\n");
	ret = ohci_hw_setup(dev);
	printk("[ohci] step 2/2: hw_setup -> %d\n", ret);
	if (ret) {
		return ret;
	}

	printk("[ohci] init done OK\n");
	LOG_INF("OHCI controller initialized");
	return 0;
}

static int uhc_ohci_enable(const struct device *dev)
{
	const struct uhc_ohci_config *cfg = dev->config;

	cfg->irq_enable_fn(dev);
	LOG_DBG("OHCI enabled");
	return 0;
}

static int uhc_ohci_disable(const struct device *dev)
{
	const struct uhc_ohci_config *cfg = dev->config;

	cfg->irq_disable_fn(dev);
	LOG_DBG("OHCI disabled");
	return 0;
}

static int uhc_ohci_shutdown(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	const struct uhc_ohci_config *cfg = dev->config;

	cfg->irq_disable_fn(dev);

	/* Stop HC */
	ohci_write32(priv->reg_base, OHCI_HC_CONTROL,
		     OHCI_CTRL_HCFS_RESET);

	LOG_DBG("OHCI shutdown");
	return 0;
}

static int uhc_ohci_bus_reset(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	uint32_t port_status;

	/* Issue port reset on port 0 (SetPortReset) */
	port_status = ohci_read32(priv->reg_base, OHCI_HC_RHPORTSTATUS);
	port_status |= OHCI_RHPS_PRS;
	ohci_write32(priv->reg_base, OHCI_HC_RHPORTSTATUS, port_status);

	/* Wait 50ms */
	k_msleep(50);

	/* Clear port reset */
	port_status = ohci_read32(priv->reg_base, OHCI_HC_RHPORTSTATUS);
	port_status &= ~OHCI_RHPS_PRS;
	ohci_write32(priv->reg_base, OHCI_HC_RHPORTSTATUS, port_status);

	LOG_DBG("Bus reset completed");
	uhc_submit_event(dev, UHC_EVT_RESETED, 0);
	return 0;
}

static int uhc_ohci_sof_enable(const struct device *dev)
{
	return 0;
}

static int uhc_ohci_bus_suspend(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);

	ohci_write32(priv->reg_base, OHCI_HC_CONTROL,
		     (ohci_read32(priv->reg_base, OHCI_HC_CONTROL) &
		      ~OHCI_CTRL_HCFS_MASK) |
		     OHCI_CTRL_HCFS_SUSPEND);

	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);
	return 0;
}

static int uhc_ohci_bus_resume(const struct device *dev)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);

	ohci_write32(priv->reg_base, OHCI_HC_CONTROL,
		     (ohci_read32(priv->reg_base, OHCI_HC_CONTROL) &
		      ~OHCI_CTRL_HCFS_MASK) |
		     OHCI_CTRL_HCFS_RESUME);

	k_msleep(20);

	ohci_write32(priv->reg_base, OHCI_HC_CONTROL,
		     (ohci_read32(priv->reg_base, OHCI_HC_CONTROL) &
		      ~OHCI_CTRL_HCFS_MASK) |
		     OHCI_CTRL_HCFS_OPER);

	uhc_submit_event(dev, UHC_EVT_RESUMED, 0);
	return 0;
}

static int uhc_ohci_ep_enqueue(const struct device *dev,
			       struct uhc_transfer *const xfer)
{
	struct uhc_ohci_priv *priv = uhc_get_private(dev);
	struct ohci_ed *ed;
	struct ohci_td *td;
	uint32_t device_addr = xfer->udev->addr;
	uint8_t ep_num = USB_EP_GET_IDX(xfer->ep);
	uint32_t dp;
	int ed_idx, td_idx;

	uhc_xfer_append(dev, xfer);

	/* Find free ED */
	for (ed_idx = 2; ed_idx < priv->ed_count; ed_idx++) {
		if (priv->ed_pool[ed_idx].hw_info == 0) {
			break;
		}
	}
	if (ed_idx >= priv->ed_count) {
		return -ENOMEM;
	}
	ed = &priv->ed_pool[ed_idx];

	/* Find free TD */
	for (td_idx = 0; td_idx < priv->td_count; td_idx++) {
		if (priv->td_pool[td_idx].hw_info == 0) {
			break;
		}
	}
	if (td_idx >= priv->td_count) {
		return -ENOMEM;
	}
	td = &priv->td_pool[td_idx];

	/* Determine direction/pid */
	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		dp = TD_DP_SETUP;
	} else if (USB_EP_DIR_IS_IN(xfer->ep)) {
		dp = TD_DP_IN;
	} else {
		dp = TD_DP_OUT;
	}

	/* Set up ED */
	ed->hw_info = (device_addr & 0x7F) |
		      (ep_num << 7) |
		      (xfer->mps << 16);
	if (xfer->udev->speed == USB_SPEED_SPEED_LS) {
		ed->hw_info |= ED_LOWSPEED;
	}
	ed->hw_tailp = (uint32_t)(uintptr_t)td;
	ed->hw_headp = (uint32_t)(uintptr_t)td;
	ed->hw_nexted = 0;

	/* Set up TD */
	memset(td, 0, sizeof(*td));
	if (xfer->buf) {
		td->hw_cbp = (uint32_t)(uintptr_t)xfer->buf->data;
		td->hw_be = td->hw_cbp + xfer->buf->len - 1;
	}
	td->hw_info = (TD_CC_NOERROR << TD_CC_SHIFT) |
		      dp |
		      TD_R |      /* Short packets OK */
		      (0x7 << TD_DI_SHIFT); /* Interrupt delay */
	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		/* Control: toggle from DATA0 */
		td->hw_info |= TD_T_DATA0;
	}
	td->hw_nexttd = 0;

	/* Link ED to appropriate list */
	if (USB_EP_GET_IDX(xfer->ep) == 0) {
		ed->hw_nexted = ohci_read32(priv->reg_base,
					    OHCI_HC_CTRL_HEADED);
		ohci_write32(priv->reg_base, OHCI_HC_CTRL_HEADED,
			     (uint32_t)(uintptr_t)ed);
	} else {
		ed->hw_nexted = ohci_read32(priv->reg_base,
					    OHCI_HC_BULK_HEADED);
		ohci_write32(priv->reg_base, OHCI_HC_BULK_HEADED,
			     (uint32_t)(uintptr_t)ed);
	}

	/* Signal list filled */
	ohci_write32(priv->reg_base, OHCI_HC_CMDSTATUS,
		     (USB_EP_GET_IDX(xfer->ep) == 0) ?
		     OHCI_CMDS_CLF : OHCI_CMDS_BLF);

	LOG_DBG("Transfer queued: dev=%u ep=%u", device_addr, ep_num);
	return 0;
}

static int uhc_ohci_ep_dequeue(const struct device *dev,
			       struct uhc_transfer *const xfer)
{
	return -ENOTSUP;
}

static const struct uhc_api ohci_uhc_api = {
	.lock		= uhc_ohci_lock,
	.unlock		= uhc_ohci_unlock,
	.init		= uhc_ohci_init,
	.enable		= uhc_ohci_enable,
	.disable	= uhc_ohci_disable,
	.shutdown	= uhc_ohci_shutdown,
	.bus_reset	= uhc_ohci_bus_reset,
	.sof_enable	= uhc_ohci_sof_enable,
	.bus_suspend	= uhc_ohci_bus_suspend,
	.bus_resume	= uhc_ohci_bus_resume,
	.ep_enqueue	= uhc_ohci_ep_enqueue,
	.ep_dequeue	= uhc_ohci_ep_dequeue,
};

/* ================================================================
 * ISR + DT Instantiation
 * ================================================================ */

static void ohci_isr(const struct device *dev)
{
	ohci_isr_handler(dev);
}

#define UHC_OHCI_DEVICE_DEFINE(n)						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, phys),				\
		(_SUNXI_PHY_PSEUDODEV_DECLARE(DT_DRV_INST(n));), ())		\
	static void ohci_irq_enable_##n(const struct device *dev)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    ohci_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
	static void ohci_irq_disable_##n(const struct device *dev)		\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
	static struct uhc_ohci_config ohci_config_##n = {			\
		.reg_base = DT_INST_REG_ADDR(n),				\
		.irq_enable_fn = ohci_irq_enable_##n,				\
		.irq_disable_fn = ohci_irq_disable_##n,				\
		.clock_dev =							\
			DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),			\
		.clock_subsys = {						\
			DT_INST_FOREACH_PROP_ELEM(n, clocks, OHCI_CLK_ELEM)	\
		},								\
		.num_clocks = DT_INST_PROP_LEN(n, clocks),			\
		.reset_dev =							\
			DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(n, resets, 0)),	\
		.reset_ids = {							\
			DT_INST_FOREACH_PROP_ELEM(n, resets, OHCI_RST_ELEM)	\
		},								\
		.num_resets = DT_INST_PROP_LEN(n, resets),			\
		.phy = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, phys),		\
			(USB_SUNXI_PHY_PSEUDODEV_GET_OR_NULL(			\
				DT_DRV_INST(n))),				\
			(NULL)),						\
	};									\
	static struct uhc_ohci_priv ohci_priv_##n;				\
	static struct uhc_data uhc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(uhc_data_##n.mutex),		\
		.priv = &ohci_priv_##n,						\
	};									\
	static int ohci_init_##n(const struct device *dev)			\
	{									\
		/* Boot-time init: platform only (clocks, resets, PHY).		\
		 * Full controller setup is deferred to .init API. This		\
		 * prevents the OHCI from entering operational state at		\
		 * boot and interfering with the EHCI on the shared port. */	\
		LOG_INF("OHCI instance " #n " boot init (platform only)");	\
		return ohci_platform_init(dev);					\
	}									\
	DEVICE_DT_INST_DEFINE(n, ohci_init_##n, NULL,				\
			      &uhc_data_##n, &ohci_config_##n,			\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &ohci_uhc_api);

DT_INST_FOREACH_STATUS_OKAY(UHC_OHCI_DEVICE_DEFINE)
