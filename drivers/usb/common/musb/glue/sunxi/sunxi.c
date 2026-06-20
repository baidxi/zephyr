/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner Sunxi MUSB glue layer.
 *
 * Provides:
 *  - Non-standard register offset mapping (Sunxi uses a different layout
 *    from the Mentor MUSB reference)
 *  - PHY enable/disable via the sunxi_usb_phy pseudo-device
 *  - PIO-only FIFO access
 *
 */

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

#include "../../musb_core.h"
#include "../../../sunxi/usb_common.h"

LOG_MODULE_REGISTER(musb_glue_sunxi, CONFIG_MUSB_LOG_LEVEL);

#define SUNXI_MUSB_POWER      0x0040
#define SUNXI_MUSB_DEVCTL     0x0041
#define SUNXI_MUSB_INDEX      0x0042
#define SUNXI_MUSB_VEND0      0x0043
#define SUNXI_MUSB_INTRTX     0x0044
#define SUNXI_MUSB_INTRRX     0x0046
#define SUNXI_MUSB_INTRTXE    0x0048
#define SUNXI_MUSB_INTRRXE    0x004a
#define SUNXI_MUSB_INTRUSB    0x004c
#define SUNXI_MUSB_INTRUSBE   0x0050
#define SUNXI_MUSB_FRAME      0x0054
#define SUNXI_MUSB_TXFIFOSZ   0x0090
#define SUNXI_MUSB_TXFIFOADD  0x0092
#define SUNXI_MUSB_RXFIFOSZ   0x0094
#define SUNXI_MUSB_RXFIFOADD  0x0096
#define SUNXI_MUSB_TXFUNCADDR 0x0098
#define SUNXI_MUSB_CONFIGDATA 0x00c0

/*
 * Indexed EP mode: all EP CSRs use the same base (0x80).
 * The MUSB_INDEX register selects the active endpoint.
 */
#define SUNXI_MUSB_EP_CSR_BASE 0x0080

static mm_reg_t sunxi_mregs;

static uint8_t sunxi_readb(mm_reg_t addr, uint32_t offset)
{
	if (addr == sunxi_mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_POWER:
			return sys_read8(addr + SUNXI_MUSB_POWER);
		case MUSB_DEVCTL:
			return sys_read8(addr + SUNXI_MUSB_DEVCTL);
		case MUSB_INTRUSB:
			return sys_read8(addr + SUNXI_MUSB_INTRUSB);
		case MUSB_INTRUSBE:
			return sys_read8(addr + SUNXI_MUSB_INTRUSBE);
		case MUSB_INDEX:
			return sys_read8(addr + SUNXI_MUSB_INDEX);
		case MUSB_TESTMODE:
			return 0; /* No testmode on sunxi */
		case MUSB_TXFIFOSZ:
			return sys_read8(addr + SUNXI_MUSB_TXFIFOSZ);
		case MUSB_RXFIFOSZ:
			return sys_read8(addr + SUNXI_MUSB_RXFIFOSZ);
		/* busctl registers: offset already includes SUNXI_MUSB_TXFUNCADDR base */
		case SUNXI_MUSB_TXFUNCADDR:
			return sys_read8(addr + offset);
		default:
			LOG_ERR("unknown readb offset %u", offset);
			return 0;
		}
	} else if (addr == (sunxi_mregs + SUNXI_MUSB_EP_CSR_BASE)) {
		/* ep control reg access */
		/* sunxi has a 2 byte hole before the txtype register */
		if (offset >= MUSB_TXTYPE) {
			offset += 2;
		}
		return sys_read8(addr + offset);
	}

	LOG_ERR("unknown readb addr %p (mregs=%p, ep_base=%p)",
		(void *)addr, (void *)sunxi_mregs,
		(void *)(sunxi_mregs + SUNXI_MUSB_EP_CSR_BASE));
	return 0;
}

static void sunxi_writeb(mm_reg_t addr, uint32_t offset, uint8_t data)
{
	if (addr == sunxi_mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_POWER:
			sys_write8(data, addr + SUNXI_MUSB_POWER);
			return;
		case MUSB_DEVCTL:
			sys_write8(data, addr + SUNXI_MUSB_DEVCTL);
			return;
		case MUSB_INTRUSB:
			sys_write8(data, addr + SUNXI_MUSB_INTRUSB);
			return;
		case MUSB_INTRUSBE:
			sys_write8(data, addr + SUNXI_MUSB_INTRUSBE);
			return;
		case MUSB_INDEX:
			sys_write8(data, addr + SUNXI_MUSB_INDEX);
			return;
		case MUSB_TESTMODE:
			if (data) {
				LOG_WRN("sunxi: TESTMODE not supported, write ignored");
			}
			return;
		case MUSB_TXFIFOSZ:
			sys_write8(data, addr + SUNXI_MUSB_TXFIFOSZ);
			return;
		case MUSB_RXFIFOSZ:
			sys_write8(data, addr + SUNXI_MUSB_RXFIFOSZ);
			return;
		/* busctl registers: offset already includes SUNXI_MUSB_TXFUNCADDR base */
		case SUNXI_MUSB_TXFUNCADDR:
			sys_write8(data, addr + offset);
			return;
		default:
			LOG_ERR("unknown writeb offset %u", offset);
			return;
		}
	} else if (addr == (sunxi_mregs + SUNXI_MUSB_EP_CSR_BASE)) {
		/* ep control reg access */
		if (offset >= MUSB_TXTYPE) {
			offset += 2;
		}
		sys_write8(data, addr + offset);
		return;
	}

	LOG_ERR("unknown writeb addr %p (mregs=%p, ep_base=%p)",
		(void *)addr, (void *)sunxi_mregs,
		(void *)(sunxi_mregs + SUNXI_MUSB_EP_CSR_BASE));
}

static uint16_t sunxi_readw(mm_reg_t addr, uint32_t offset)
{
	if (addr == sunxi_mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_INTRTX:
			return sys_read16(addr + SUNXI_MUSB_INTRTX);
		case MUSB_INTRRX:
			return sys_read16(addr + SUNXI_MUSB_INTRRX);
		case MUSB_INTRTXE:
			return sys_read16(addr + SUNXI_MUSB_INTRTXE);
		case MUSB_INTRRXE:
			return sys_read16(addr + SUNXI_MUSB_INTRRXE);
		case MUSB_FRAME:
			return sys_read16(addr + SUNXI_MUSB_FRAME);
		case MUSB_TXFIFOADD:
			return sys_read16(addr + SUNXI_MUSB_TXFIFOADD);
		case MUSB_RXFIFOADD:
			return sys_read16(addr + SUNXI_MUSB_RXFIFOADD);
		default:
			LOG_ERR("unknown readw offset %u", offset);
			return 0;
		}
	} else if (addr == (sunxi_mregs + SUNXI_MUSB_EP_CSR_BASE)) {
		/* ep control reg access */
		return sys_read16(addr + offset);
	}

	LOG_ERR("unknown readw addr %p", (void *)addr);
	return 0;
}

static void sunxi_writew(mm_reg_t addr, uint32_t offset, uint16_t data)
{
	if (addr == sunxi_mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_INTRTX:
			sys_write16(data, addr + SUNXI_MUSB_INTRTX);
			return;
		case MUSB_INTRRX:
			sys_write16(data, addr + SUNXI_MUSB_INTRRX);
			return;
		case MUSB_INTRTXE:
			sys_write16(data, addr + SUNXI_MUSB_INTRTXE);
			return;
		case MUSB_INTRRXE:
			sys_write16(data, addr + SUNXI_MUSB_INTRRXE);
			return;
		case MUSB_FRAME:
			sys_write16(data, addr + SUNXI_MUSB_FRAME);
			return;
		case MUSB_TXFIFOADD:
			sys_write16(data, addr + SUNXI_MUSB_TXFIFOADD);
			return;
		case MUSB_RXFIFOADD:
			sys_write16(data, addr + SUNXI_MUSB_RXFIFOADD);
			return;
		default:
			LOG_ERR("unknown writew offset %u", offset);
			return;
		}
	} else if (addr == (sunxi_mregs + SUNXI_MUSB_EP_CSR_BASE)) {
		/* ep control reg access */
		sys_write16(data, addr + offset);
		return;
	}

	LOG_ERR("unknown writew addr %p", (void *)addr);
}

/*
 * FIFO read — matches Linux musb_default_read_fifo() exactly.
 *
 * Linux sunxi does NOT provide custom read_fifo in platform_ops;
 * it falls back to musb_default_read_fifo() in musb_core.c.
 *
 * Alignment-based dispatch:
 *   - 32-bit aligned buf: ioread32_rep → sys_read32 loop
 *     + readw tail → sys_read16 + readb tail → sys_read8
 *   - 16-bit aligned buf: ioread16_rep → sys_read16 loop
 *     + readb tail → sys_read8
 *   - byte aligned buf: ioread8_rep → sys_read8 loop
 */
static void sunxi_read_fifo(mm_reg_t fifo, uint16_t len, uint8_t *dst)
{
	if (unlikely(len == 0)) {
		return;
	}

	/* we can't assume unaligned writes work */
	if (likely((0x01 & (uintptr_t)dst) == 0)) {
		uint16_t index = 0;

		/* best case is 32bit-aligned destination address */
		if ((0x02 & (uintptr_t)dst) == 0) {
			if (len >= 4) {
				/* ioread32_rep equivalent */
				uint32_t *dst32 = (uint32_t *)dst;

				for (uint16_t i = 0; i < (len >> 2); i++) {
					*dst32++ = sys_read32(fifo);
				}
				index = len & ~0x03;
			}
			if (len & 0x02) {
				*(uint16_t *)&dst[index] = sys_read16(fifo);
				index += 2;
			}
		} else {
			if (len >= 2) {
				/* ioread16_rep equivalent */
				uint16_t *dst16 = (uint16_t *)dst;

				for (uint16_t i = 0; i < (len >> 1); i++) {
					*dst16++ = sys_read16(fifo);
				}
				index = len & ~0x01;
			}
		}
		if (len & 0x01) {
			dst[index] = sys_read8(fifo);
		}
	} else {
		/* byte aligned */
		for (uint16_t i = 0; i < len; i++) {
			dst[i] = sys_read8(fifo);
		}
	}
}

/*
 * FIFO write — matches Linux musb_default_write_fifo() exactly.
 *
 * Alignment-based dispatch mirrors musb_default_write_fifo():
 *   - 32-bit aligned src: iowrite32_rep → sys_write32 loop
 *     + writew tail → sys_write16 + writeb tail → sys_write8
 *   - 16-bit aligned src: iowrite16_rep → sys_write16 loop
 *     + writeb tail → sys_write8
 *   - byte aligned src: iowrite8_rep → sys_write8 loop
 */
static void sunxi_write_fifo(mm_reg_t fifo, uint16_t len, const uint8_t *src)
{
	if (unlikely(len == 0)) {
		return;
	}

	/* we can't assume unaligned reads work */
	if (likely((0x01 & (uintptr_t)src) == 0)) {
		uint16_t index = 0;

		/* best case is 32bit-aligned source address */
		if ((0x02 & (uintptr_t)src) == 0) {
			if (len >= 4) {
				/* iowrite32_rep equivalent */
				const uint32_t *src32 = (const uint32_t *)src;

				for (uint16_t i = 0; i < (len >> 2); i++) {
					sys_write32(*src32++, fifo);
				}
				index = len & ~0x03;
			}
			if (len & 0x02) {
				sys_write16(*(const uint16_t *)&src[index], fifo);
				index += 2;
			}
		} else {
			if (len >= 2) {
				/* iowrite16_rep equivalent */
				const uint16_t *src16 = (const uint16_t *)src;

				for (uint16_t i = 0; i < (len >> 1); i++) {
					sys_write16(*src16++, fifo);
				}
				index = len & ~0x01;
			}
		}
		if (len & 0x01) {
			sys_write8(src[index], fifo);
		}
	} else {
		/* byte aligned */
		for (uint16_t i = 0; i < len; i++) {
			sys_write8(src[i], fifo);
		}
	}
}

/*
 * EP offset: returns the EP CSR base address (0x80).
 * Matches Linux sunxi_musb_ep_offset() exactly — always returns 0x80
 * for indexed EP mode, ignoring epnum and offset.
 */
static uint32_t sunxi_ep_offset(uint8_t epnum, uint16_t offset)
{
	ARG_UNUSED(epnum);

	if (offset != 0) {
		LOG_WRN("sunxi_ep_offset called with non-0 offset 0x%x", offset);
	}

	return SUNXI_MUSB_EP_CSR_BASE;
}

static uint32_t sunxi_fifo_offset(uint8_t epnum)
{
	/*
	 * FIFO data ports are at 4-byte intervals.  Manual:
	 * USB_EPFIFOn at 0x0000 + N*0x0004.  Matches Linux sunxi_musb_fifo_offset().
	 */
	return epnum * 4;
}

static uint32_t sunxi_busctl_offset(uint8_t epnum, uint16_t offset)
{
	ARG_UNUSED(epnum);
	return SUNXI_MUSB_TXFUNCADDR + offset;
}

/* =========================================================================
 * Platform Lifecycle — PHY Integration
 *
 * The PHY pseudo-device (sunxi_usb_phy) was instantiated at compile time
 * by the PHY driver.  We obtain it via the USB controller's DT node.
 * ========================================================================= */

static void sunxi_reg_dump(const struct musb_dev *musb)
{
	uintptr_t musb_base = musb->base;

	LOG_DBG("=== CCU clocks ===");
	LOG_DBG("  PLL_PERIPH0 (0x028):   0x%08x",
		sys_read32(DT_REG_ADDR(DT_NODELABEL(ccu)) + 0x28));
	LOG_DBG("  USB_CLK (0x0cc):       0x%08x",
		sys_read32(DT_REG_ADDR(DT_NODELABEL(ccu)) + 0x0cc));
	LOG_DBG("  BUS_CLK_GATING0 (0x060): 0x%08x",
		sys_read32(DT_REG_ADDR(DT_NODELABEL(ccu)) + 0x60));
	LOG_DBG("  BUS_CLK_GATING1:       0x%08x",
		sys_read32(DT_REG_ADDR(DT_NODELABEL(ccu)) + 0x64));
	LOG_DBG("  BUS_SOFT_RST0:         0x%08x",
		sys_read32(DT_REG_ADDR(DT_NODELABEL(ccu)) + 0x2c0));

	LOG_DBG("=== PHY ===");
	LOG_DBG("  ISCR:                  0x%08x",
		sys_read32(DT_REG_ADDR_BY_IDX(DT_NODELABEL(usbphy), 0) + 0x00));
	LOG_DBG("  PHYCTL_A33:            0x%08x",
		sys_read32(DT_REG_ADDR_BY_IDX(DT_NODELABEL(usbphy), 0) + 0x10));
	LOG_DBG("  PHY_OTGCTL:            0x%08x",
		sys_read32(DT_REG_ADDR_BY_IDX(DT_NODELABEL(usbphy), 0) + 0x20));
	LOG_DBG("  PMU_PASSBY (0x00):     0x%08x",
		sys_read32(DT_REG_ADDR_BY_NAME(DT_NODELABEL(usbphy), pmu0) + 0x00));
	LOG_DBG("  PMU_HCI_PHY_CTL(0x10): 0x%08x",
		sys_read32(DT_REG_ADDR_BY_NAME(DT_NODELABEL(usbphy), pmu0) + 0x10));

	LOG_DBG("=== MUSB ===");
	LOG_DBG("  POWER:                 0x%02x", sys_read8(musb_base + 0x40));
	LOG_DBG("  DEVCTL:                0x%02x", sys_read8(musb_base + 0x41));
	LOG_DBG("  INDEX:                 0x%02x", sys_read8(musb_base + 0x42));
	LOG_DBG("  VEND0:                 0x%02x", sys_read8(musb_base + 0x43));
	LOG_DBG("  INTRUSB:               0x%02x", sys_read8(musb_base + 0x4c));
	LOG_DBG("  INTRUSBE:              0x%02x", sys_read8(musb_base + 0x50));
	LOG_DBG("  INTRTX:                0x%04x", sys_read16(musb_base + 0x44));
	LOG_DBG("  INTRTXE:               0x%04x", sys_read16(musb_base + 0x48));
	LOG_DBG("  INTRRX:                0x%04x", sys_read16(musb_base + 0x46));
	LOG_DBG("  INTRRXE:               0x%04x", sys_read16(musb_base + 0x4a));
	sys_write8(0, musb_base + 0x42); /* INDEX=0 */
	LOG_DBG("  CSR0:                  0x%04x", sys_read16(musb_base + 0x82));
}

static void sunxi_phy_notify(const struct sunxi_usb_phy *phy,
			      enum sunxi_usb_phy_event event,
			      int state, void *user)
{
	ARG_UNUSED(phy);
	ARG_UNUSED(user);

	switch (event) {
	case SUNXI_USB_PHY_EVENT_ID:
		LOG_DBG("PHY notify: ID changed -> %s mode",
			state ? "peripheral" : "host");
		break;
	case SUNXI_USB_PHY_EVENT_VBUS:
		LOG_DBG("PHY notify: VBUS changed -> %s",
			state ? "online" : "offline");
		break;
	}
}

static int sunxi_init(const struct device *dev)
{
	const struct musb_dev *musb = to_musb(dev);
	const struct sunxi_usb_phy *phy_dev = musb->phy_dev;
	const struct musb_glue_config *cfg = musb->config;
	int ret;

	if (!phy_dev) {
		LOG_ERR("No PHY pseudo-device for MUSB");
		return -ENODEV;
	}

	/*
	 * Save mregs base address for addr comparison in readb/writeb.
	 * Matches Linux's assignment of sunxi_musb->mregs.
	 */
	sunxi_mregs = musb->base;

	LOG_DBG("sunxi_init: clk_subsys=0x%lx rst_id=0x%x",
		(uintptr_t)cfg->clk_subsys, cfg->rst_id);

	ret = clock_control_on(musb->clk_dev, cfg->clk_subsys);
	LOG_DBG("clock_control_on(OTG bus) = %d", ret);
	if (ret < 0) {
		LOG_ERR("MUSB bus clock enable failed: %d "
			"(CCU d1_gates[] likely lacks D1_CLK_BUS_OTG=103)", ret);
		return ret;
	}

	ret = reset_line_deassert(musb->rst_dev, cfg->rst_id);
	LOG_DBG("reset_line_deassert(OTG bus) = %d", ret);
	if (ret < 0) {
		LOG_ERR("MUSB bus reset deassert failed: %d "
			"(reset d1_rst_infos[] likely lacks D1_RST_BUS_OTG=46)", ret);
		return ret;
	}

	/* Set PIO mode via VEND0 */
	sys_write8(0, musb->base + SUNXI_MUSB_VEND0);
	LOG_DBG("sunxi init: VEND0=0x%02x (PIO mode)",
		sys_read8(musb->base + SUNXI_MUSB_VEND0));

	ret = phy_dev->enable(phy_dev);
	if (ret) {
		LOG_ERR("PHY enable failed: %d", ret);
		return ret;
	}

	/* Register for PHY state change notifications (ID/VBUS) */
	if (phy_dev->register_notify) {
		phy_dev->register_notify(phy_dev, sunxi_phy_notify, NULL);
	}

	/*
	 * Diagnostic register dump — compare with Linux for debugging.
	 * Only compiled when LOG_DBG is enabled.
	 */
	if (IS_ENABLED(CONFIG_MUSB_LOG_LEVEL_DBG)) {
		sunxi_reg_dump(musb);
	}
	return 0;
}

static void sunxi_enable(const struct device *dev)
{
	/* PHY is already enabled from init.  Nothing extra needed. */
}

static void sunxi_disable(const struct device *dev)
{
	struct musb_dev *musb = to_musb(dev);
	const struct sunxi_usb_phy *phy = musb->phy_dev;

	if (phy) {
		phy->disable(phy);
	}
}

static const struct musb_glue_ops sunxi_ops = {
	.readb = sunxi_readb,
	.writeb = sunxi_writeb,
	.readw = sunxi_readw,
	.writew = sunxi_writew,
	.read_fifo = sunxi_read_fifo,
	.write_fifo = sunxi_write_fifo,
	.ep_offset = sunxi_ep_offset,
	.fifo_offset = sunxi_fifo_offset,
	.busctl_offset = sunxi_busctl_offset,
	.init = sunxi_init,
	.enable = sunxi_enable,
	.disable = sunxi_disable,
};

/* SoC-specific FIFO configuration compiled conditionally */
#if DT_HAS_COMPAT_STATUS_OKAY(allwinner_sun20i_d1_musb)
/* D1/T113-S3: EP1-5 bidirectional (TX+RX), 2KB FIFO (ram_bits=11).
 * Matches Linux sun8i-a33 fallback config (sunxi_musb_hdrc_config_5eps).
 */
static const struct musb_fifo_cfg sunxi_fifo_cfg[] = {
	{.hw_ep_num = 1, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 1, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 2, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 2, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 3, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 3, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 4, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 4, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 5, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 5, .style = FIFO_RX, .maxpacket = 512},
};
#else
/* V3s: EP1-4 bidirectional (TX+RX), 2KB FIFO */
static const struct musb_fifo_cfg sunxi_fifo_cfg[] = {
	{.hw_ep_num = 1, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 1, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 2, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 2, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 3, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 3, .style = FIFO_RX, .maxpacket = 512},
	{.hw_ep_num = 4, .style = FIFO_TX, .maxpacket = 512},
	{.hw_ep_num = 4, .style = FIFO_RX, .maxpacket = 512},
};
#endif

 /* Obtain the PHY pseudo-device for the usbotg node */
#define MUSB_PHY_NODE DT_NODELABEL(usbphy)
#define MUSB_NODE     DT_NODELABEL(usbotg)

/*
 * V3s:  5 endpoints (EP0 + 4 bidir pairs), 2KB FIFO (ram_bits=11)
 * D1:   6 endpoints (EP0 + 5 bidir pairs), 2KB FIFO (ram_bits=11)
 *       (D1 falls back to sun8i-a33 config in Linux, which uses
 *       SUNXI_MUSB_RAM_BITS=11 for all sunxi variants.)
 */
#define SUNXI_MUSB_NUM_EPS						\
	COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(allwinner_sun20i_d1_musb), \
		    (6), (5))

#define SUNXI_MUSB_RAM_BITS						\
	COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(allwinner_sun20i_d1_musb), \
		    (11), (11))

#define SUNXI_MUSB_EPMASK						\
	COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(allwinner_sun20i_d1_musb), \
		    (GENMASK(5, 0)), (GENMASK(4, 0)))

static const struct musb_glue_config sunxi_config = {
	.fifo_cfg = sunxi_fifo_cfg,
	.fifo_cfg_size = ARRAY_SIZE(sunxi_fifo_cfg),
	.num_eps = SUNXI_MUSB_NUM_EPS,
	.ram_bits = SUNXI_MUSB_RAM_BITS,
	.quirks = MUSB_INDEXED_EP | MUSB_NO_CONFIGDATA
#if IS_ENABLED(CONFIG_MUSB_SUNXI_FORCE_FULLSPEED)
		  | MUSB_DISABLE_HS
#endif
		  ,
	.configdata_val = 0xde,
	.epmask = SUNXI_MUSB_EPMASK,
	.clk_subsys =
		(clock_control_subsys_t)(uintptr_t)DT_PHA_BY_IDX(MUSB_NODE, clocks, 0, clk_id),
	.rst_id = DT_PHA_BY_IDX(MUSB_NODE, resets, 0, id),
	/* PIO-only; DMA DRQ slots are SoC-specific and unused */
	.dma_tx_drq =
		{
			[1] = 17, [2] = 18, [3] = 19, [4] = 20,
		},
	.dma_rx_drq =
		{
			[1] = 17, [2] = 18, [3] = 19, [4] = 20,
		},
};

/* Register glue ops + config, overriding core's weak function */
MUSB_REGISTER_GLUE(&sunxi_ops, &sunxi_config);
