/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner V3s EMAC (Ethernet MAC) driver for Zephyr
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_emac


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/cache.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/irq.h>
#include <zephyr/dt-bindings/clock/allwinner,sun8i-v3s-ccu.h>
#include <zephyr/dt-bindings/reset/allwinner,sun8i-v3s-ccu.h>

#include "eth_sun8i_v3s_emac_priv.h"
#include "eth.h"

#define LOG_MODULE_NAME eth_sun8i_v3s_emac
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/*
 * Compile-time sanity: descriptor must be exactly one cache line
 * (32 bytes on Cortex-A7) so that sys_cache_data_invd_range() takes
 * the pure-invalidate fast path in arch_dcache_invd_range().
 */
BUILD_ASSERT(sizeof(struct emac_dma_desc) == 32,
	     "DMA descriptor must be exactly 32 bytes (one cache line)");

/* Helper macros for MMIO access */
#define DEV_CFG(dev)	((const struct emac_v3s_config *)(dev)->config)
#define DEV_DATA(dev)	((struct emac_v3s_data *)(dev)->data)
#define DEV_BASE(dev)	DEVICE_MMIO_GET(dev)

static inline uint32_t emac_read(const struct device *dev, uint32_t offset)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + offset);
}

static inline void emac_write(const struct device *dev, uint32_t offset,
			      uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_GET(dev) + offset);
}

static void emac_setup_tx_descs(const struct device *dev)
{
	struct emac_v3s_data *data = DEV_DATA(dev);

	for (int i = 0; i < TX_DESC_COUNT; i++) {
		data->tx_descs[i].status = 0;
		/* TDES1: chain mode bit must be set for sun8i (forced chain mode) */
		data->tx_descs[i].ctl = TX_SECOND_ADDR_CHAINED;
		data->tx_descs[i].buf_addr =
			(uint32_t)(uintptr_t)data->tx_bufs[i];
		data->tx_descs[i].next =
			(uint32_t)(uintptr_t)&data->tx_descs[(i + 1) % TX_DESC_COUNT];
	}

	data->tx_desc_idx = 0;
	data->tx_desc_submitted = 0;
}

static void emac_setup_rx_descs(const struct device *dev)
{
	struct emac_v3s_data *data = DEV_DATA(dev);

	for (int i = 0; i < RX_DESC_COUNT; i++) {
		data->rx_descs[i].status = RX_DESC_CTL;
		/* RDES1: buffer size in bits[10:0] + chain mode bit (BIT(24)).
		 * Sun8i requires chain mode (Linux forces chain_mode=1 for sun8i).
		 * Without this bit the DMA engine ignores the next descriptor
		 * pointer and the RX ring stalls after the first descriptor.
		 */
		data->rx_descs[i].ctl = EMAC_MAX_FRAME_SIZE | RX_SECOND_ADDR_CHAINED;
		data->rx_descs[i].buf_addr =
			(uint32_t)(uintptr_t)data->rx_bufs[i];
		data->rx_descs[i].next =
			(uint32_t)(uintptr_t)&data->rx_descs[(i + 1) % RX_DESC_COUNT];
	}

	data->rx_desc_idx = 0;
}

static int emac_soft_reset(const struct device *dev)
{
	uint32_t val;
	int retries = EMAC_RESET_TIMEOUT_US / 10;

	val = emac_read(dev, EMAC_BASIC_CTL1);
	emac_write(dev, EMAC_BASIC_CTL1, val | EMAC_SOFT_RST);

	while (emac_read(dev, EMAC_BASIC_CTL1) & EMAC_SOFT_RST) {
		if (retries-- <= 0) {
			LOG_ERR("EMAC soft reset timeout");
			return -ETIMEDOUT;
		}
		k_usleep(10);
	}

	return 0;
}

static int emac_configure_syscon(const struct device *dev)
{
	const struct emac_v3s_config *cfg = DEV_CFG(dev);
	uint32_t reg;
	int ret;

	reg = V3S_DEFAULT_SYSCON_VALUE;

	/* Select internal PHY */
	reg |= H3_EPHY_SELECT;
	/* Power up EPHY (clear shutdown bit) */
	reg &= ~H3_EPHY_SHUTDOWN;
	/* Force 24MHz crystal */
	reg |= H3_EPHY_CLK_SEL;

	/* LED polarity */
	if (cfg->leds_active_low) {
		reg |= H3_EPHY_LED_POL;
	} else {
		reg &= ~H3_EPHY_LED_POL;
	}

	/* PHY address */
	reg &= ~(0x1F << H3_EPHY_ADDR_SHIFT);
	reg |= (cfg->phy_addr << H3_EPHY_ADDR_SHIFT);

	ret = syscon_write_reg(cfg->syscon_dev, SYSCON_EPHY_REG, reg);
	if (ret < 0) {
		LOG_ERR("Failed to write syscon: %d", ret);
		return ret;
	}

	LOG_INF("EPHY syscon configured: 0x%08x", reg);

	/* Read back to verify */
	uint32_t readback;

	ret = syscon_read_reg(cfg->syscon_dev, SYSCON_EPHY_REG, &readback);
	if (ret == 0) {
		LOG_INF("EPHY syscon readback: 0x%08x", readback);
		if (readback != reg) {
			LOG_WRN("syscon readback mismatch! wrote=0x%08x got=0x%08x",
				reg, readback);
		}
	}

	return 0;
}

static void emac_set_mac_addr(const struct device *dev, const uint8_t *mac)
{
	uint32_t hi, lo;

	/* Synopsys DW MAC register layout:
	 *   MACADDR_HI[15:0] = MAC address upper 16 bits (mac[4]:mac[5])
	 *   MACADDR_LO[31:0] = MAC address lower 32 bits (mac[0]:mac[1]:mac[2]:mac[3])
	 * BIT(31) in MACADDR_HI enables the address filter entry.
	 * See Linux stmmac_set_mac_addr() in dwmac_lib.c.
	 */
	lo = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
	hi = mac[4] | (mac[5] << 8);

	emac_write(dev, EMAC_MACADDR_HI(0), hi | MAC_ADDR_TYPE_DST);
	emac_write(dev, EMAC_MACADDR_LO(0), lo);
}

static int emac_enable_clocks_and_reset(const struct device *dev)
{
	const struct emac_v3s_config *cfg = DEV_CFG(dev);
	int ret;

	/* Enable EMAC bus clock */
	ret = clock_control_on(cfg->clock_dev,
			       (clock_control_subsys_t)(uintptr_t)CLK_BUS_EMAC);
	if (ret < 0) {
		LOG_ERR("Failed to enable CLK_BUS_EMAC: %d", ret);
		return ret;
	}

	/* Deassert EMAC reset */
	ret = reset_line_deassert(cfg->reset_dev, RST_BUS_EMAC);
	if (ret < 0) {
		LOG_ERR("Failed to deassert RST_BUS_EMAC: %d", ret);
		return ret;
	}

	/* Enable EPHY bus clock */
	ret = clock_control_on(cfg->clock_dev,
			       (clock_control_subsys_t)(uintptr_t)CLK_BUS_EPHY);
	if (ret < 0) {
		LOG_ERR("Failed to enable CLK_BUS_EPHY: %d", ret);
		return ret;
	}

	/* Reset EPHY (assert then deassert) */
	ret = reset_line_assert(cfg->reset_dev, RST_BUS_EPHY);
	if (ret < 0) {
		LOG_ERR("Failed to assert RST_BUS_EPHY: %d", ret);
		return ret;
	}

	k_usleep(100);

	ret = reset_line_deassert(cfg->reset_dev, RST_BUS_EPHY);
	if (ret < 0) {
		LOG_ERR("Failed to deassert RST_BUS_EPHY: %d", ret);
		return ret;
	}

	/* Give EPHY time to initialize */
	k_msleep(10);

	return 0;
}

static void emac_rx_refill(const struct device *dev, uint32_t desc_idx)
{
	struct emac_v3s_data *data = DEV_DATA(dev);

	data->rx_descs[desc_idx].status = RX_DESC_CTL;
	data->rx_descs[desc_idx].ctl = EMAC_MAX_FRAME_SIZE | RX_SECOND_ADDR_CHAINED;

	/* Flush descriptor from CPU cache to RAM */
	sys_cache_data_flush_range(&data->rx_descs[desc_idx],
				   sizeof(struct emac_dma_desc));
}

static void emac_process_rx(const struct device *dev)
{
	struct emac_v3s_data *data = DEV_DATA(dev);
	struct net_pkt *pkt;
	uint32_t rx_len;
	int res;

	/* Invalidate the current RX descriptor from cache */
	sys_cache_data_invd_range(&data->rx_descs[data->rx_desc_idx],
				  sizeof(struct emac_dma_desc));
	/*
	 * DSB: ensure cache invalidation completes before the CPU
	 * reads the descriptor fields.  Without this the CPU may
	 * speculatively fetch stale cache lines.
	 */
	barrier_dsync_fence_full();

	/* Debug: log descriptor state and DMA status before checking */
	LOG_DBG("RX poll desc[%u]: status=0x%08x ctl=0x%08x OWN=%d",
		data->rx_desc_idx,
		data->rx_descs[data->rx_desc_idx].status,
		data->rx_descs[data->rx_desc_idx].ctl,
		!!(data->rx_descs[data->rx_desc_idx].status & RX_DESC_CTL));
	LOG_DBG("  RX_DMA_STA=0x%x RX_CUR_DESC=0x%x RX_CUR_BUF=0x%x",
		emac_read(dev, EMAC_RX_DMA_STA),
		emac_read(dev, EMAC_RX_CUR_DESC),
		emac_read(dev, EMAC_RX_CUR_BUF));

	while (!(data->rx_descs[data->rx_desc_idx].status & RX_DESC_CTL)) {
		/* Frame received */
		uint32_t status = data->rx_descs[data->rx_desc_idx].status;

		/* Frame length from DMA includes 4-byte CRC.
		 * Subtract it so the network stack gets the actual
		 * frame data without the trailing FCS.
		 */
		rx_len = ((status & RX_FRM_LEN_MASK) >> RX_FRM_LEN_SHIFT) - 4;
		if (rx_len == 0) {
			LOG_WRN("RX frame length is 0, status=0x%08x", status);
			goto next_rx;
		}

		LOG_DBG("RX desc[%u] status=0x%08x len=%u",
			data->rx_desc_idx, status, rx_len);

		/* Check for errors (following Synopsys normal descriptor format):
		 * Only drop frame if ERROR_SUMMARY is set, or if individual
		 * fatal errors (LENGTH_ERR, MII_ERR) are present.
		 * Checksum-related bits (BIT(0), BIT(7)) are informational
		 * and should not cause frame drops.
		 */
		if (status & RX_ERR_SUMMARY) {
			LOG_WRN("RX error summary, status=0x%08x", status);
			if (status & RX_DESC_ERR) {
				LOG_WRN("  descriptor error");
			}
			if (status & RX_OVERFLOW_ERR) {
				LOG_WRN("  overflow error");
			}
			if (status & RX_CRC_ERR) {
				LOG_WRN("  CRC error");
			}
			goto next_rx;
		}
		if (status & (RX_LENGTH_ERR | RX_MII_ERR)) {
			LOG_WRN("RX length/MII error, status=0x%08x", status);
			goto next_rx;
		}

		/* Invalidate the RX buffer from cache */
		sys_cache_data_invd_range(
			data->rx_bufs[data->rx_desc_idx],
			rx_len);

		/* DIAG: dump Ethernet header (first 14 bytes) */
		{
			const uint8_t *eh = data->rx_bufs[data->rx_desc_idx];

			LOG_DBG("  ETH dst=%02x:%02x:%02x:%02x:%02x:%02x "
				"src=%02x:%02x:%02x:%02x:%02x:%02x "
				"type=0x%02x%02x",
				eh[0], eh[1], eh[2], eh[3], eh[4], eh[5],
				eh[6], eh[7], eh[8], eh[9], eh[10], eh[11],
				eh[12], eh[13]);
		}

		pkt = net_pkt_rx_alloc_with_buffer(
			data->iface, rx_len, NET_AF_UNSPEC, 0, K_NO_WAIT);
		if (pkt == NULL) {
			LOG_WRN("Failed to allocate RX net_pkt");
			goto next_rx;
		}

		res = net_pkt_write(pkt, data->rx_bufs[data->rx_desc_idx],
				    rx_len);
		if (res < 0) {
			LOG_WRN("Failed to write to net_pkt: %d", res);
			net_pkt_unref(pkt);
			goto next_rx;
		}

		res = net_recv_data(data->iface, pkt);
		if (res < 0) {
			LOG_WRN("net_recv_data failed: %d", res);
			net_pkt_unref(pkt);
		} else {
			LOG_DBG("RX frame delivered to stack: %u bytes", rx_len);
		}

next_rx:
		emac_rx_refill(dev, data->rx_desc_idx);
		data->rx_desc_idx = (data->rx_desc_idx + 1) % RX_DESC_COUNT;

		/* Invalidate next descriptor */
		sys_cache_data_invd_range(
			&data->rx_descs[data->rx_desc_idx],
			sizeof(struct emac_dma_desc));
		barrier_dsync_fence_full();
	}
}

static void emac_process_tx(const struct device *dev)
{
	struct emac_v3s_data *data = DEV_DATA(dev);

	if (!data->tx_busy) {
		return;
	}

	/* Invalidate the submitted TX descriptor from cache */
	sys_cache_data_invd_range(&data->tx_descs[data->tx_desc_submitted],
				  sizeof(struct emac_dma_desc));
	barrier_dsync_fence_full();

	if (!(data->tx_descs[data->tx_desc_submitted].status & TX_DESC_CTL)) {
		/* TX completed: hardware cleared the OWN bit */
		data->tx_busy = false;
	}
}

static void emac_v3s_isr(const struct device *dev)
{
	struct emac_v3s_data *data = DEV_DATA(dev);
	uint32_t int_sta;

	int_sta = emac_read(dev, EMAC_INT_STA);

	LOG_DBG("ISR: int_sta=0x%08x", int_sta);

	/* Clear all pending interrupts */
	emac_write(dev, EMAC_INT_STA, int_sta);

	if (int_sta & (EMAC_TX_INT | EMAC_TX_BUF_UA_INT |
		       EMAC_TX_DMA_STOP_INT)) {
		emac_process_tx(dev);
	}

	if (int_sta & (EMAC_RX_INT | EMAC_RX_BUF_UA_INT |
		       EMAC_RX_DMA_STOP_INT)) {
		emac_process_rx(dev);
	}

	if (int_sta & (EMAC_TX_TIMEOUT_INT | EMAC_TX_UNDERFLOW_INT)) {
		LOG_WRN("TX error interrupt: 0x%08x", int_sta);
		/* Fatal TX errors: force clear tx_busy to allow recovery */
		data->tx_busy = false;
	}

	if (int_sta & (EMAC_RX_TIMEOUT_INT | EMAC_RX_OVERFLOW_INT)) {
		LOG_WRN("RX error interrupt: 0x%08x", int_sta);
	}
}

static int emac_v3s_send(const struct device *dev, struct net_pkt *pkt)
{
	struct emac_v3s_data *data = DEV_DATA(dev);
	uint32_t pkt_len = net_pkt_get_len(pkt);
	uint32_t desc_idx;
	int ret;

	if (data->tx_busy) {
		LOG_WRN("TX busy, dropping packet");
		return -EBUSY;
	}

	if (pkt_len > EMAC_MAX_FRAME_SIZE) {
		LOG_WRN("Packet too large: %u > %u", pkt_len,
			EMAC_MAX_FRAME_SIZE);
		return -ENOMEM;
	}

	desc_idx = data->tx_desc_idx;

	/* Copy packet data to TX buffer */
	ret = net_pkt_read(pkt, data->tx_bufs[desc_idx], pkt_len);
	if (ret < 0) {
		LOG_ERR("Failed to read net_pkt: %d", ret);
		return ret;
	}

	/* Flush TX buffer from CPU cache to RAM */
	sys_cache_data_flush_range(data->tx_bufs[desc_idx], pkt_len);

	/* Setup TX descriptor (chain mode bit required for sun8i) */
	data->tx_descs[desc_idx].status = TX_DESC_CTL;
	data->tx_descs[desc_idx].ctl = TX_FIR_DESC | TX_LAST_DESC |
					TX_INT_CTL | TX_SECOND_ADDR_CHAINED |
					pkt_len;
	data->tx_descs[desc_idx].buf_addr =
		(uint32_t)(uintptr_t)data->tx_bufs[desc_idx];

	/* Flush TX descriptor from CPU cache to RAM */
	sys_cache_data_flush_range(&data->tx_descs[desc_idx],
				   sizeof(struct emac_dma_desc));

	data->tx_busy = true;
	data->tx_desc_submitted = desc_idx;

	/* Start TX DMA */
	emac_write(dev, EMAC_TX_CTL1,
		   emac_read(dev, EMAC_TX_CTL1) | EMAC_TX_DMA_START);

	data->tx_desc_idx = (desc_idx + 1) % TX_DESC_COUNT;

	return 0;
}

static enum ethernet_hw_caps emac_v3s_get_capabilities(const struct device *dev,
						       struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
}

static int emac_v3s_set_config(const struct device *dev,
			       struct net_if *iface,
			       enum ethernet_config_type type,
			       const struct ethernet_config *config)
{
	ARG_UNUSED(iface);
	struct emac_v3s_data *data = DEV_DATA(dev);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr,
		       sizeof(data->mac_addr));
		emac_set_mac_addr(dev, data->mac_addr);
		net_if_set_link_addr(data->iface, data->mac_addr,
				     sizeof(data->mac_addr),
				     NET_LINK_ETHERNET);
		return 0;

	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		if (config->promisc_mode) {
			emac_write(dev, EMAC_RX_FRM_FLT, EMAC_FRM_FLT_RXALL);
		} else {
			emac_write(dev, EMAC_RX_FRM_FLT, EMAC_FRM_FLT_CTL);
		}
		return 0;

	default:
		break;
	}

	return -ENOTSUP;
}

static int emac_v3s_start(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(iface);
	uint32_t val;

	/* Enable transmitter */
	val = emac_read(dev, EMAC_TX_CTL0);
	val |= EMAC_TX_TRANSMITTER_EN;
	emac_write(dev, EMAC_TX_CTL0, val);

	/* Enable receiver with CRC checking */
	val = emac_read(dev, EMAC_RX_CTL0);
	val |= EMAC_RX_RECEIVER_EN | EMAC_RX_DO_CRC;
	emac_write(dev, EMAC_RX_CTL0, val);

	/* Set store-and-forward mode for TX */
	val = emac_read(dev, EMAC_TX_CTL1);
	val |= EMAC_TX_MD | EMAC_TX_NEXT_FRM;
	emac_write(dev, EMAC_TX_CTL1, val);

	/* Set store-and-forward mode for RX */
	val = emac_read(dev, EMAC_RX_CTL1);
	val |= EMAC_RX_MD;
	emac_write(dev, EMAC_RX_CTL1, val);

	/* Enable TX DMA */
	emac_write(dev, EMAC_TX_CTL1,
		   emac_read(dev, EMAC_TX_CTL1) | EMAC_TX_DMA_EN);

	/* Enable RX DMA */
	emac_write(dev, EMAC_RX_CTL1,
		   emac_read(dev, EMAC_RX_CTL1) | EMAC_RX_DMA_EN);

	/* Enable interrupts */
	emac_write(dev, EMAC_INT_EN,
		   EMAC_TX_INT | EMAC_TX_DMA_STOP_INT | EMAC_TX_BUF_UA_INT |
		   EMAC_TX_TIMEOUT_INT | EMAC_TX_UNDERFLOW_INT |
		   EMAC_RX_INT | EMAC_RX_DMA_STOP_INT | EMAC_RX_BUF_UA_INT |
		   EMAC_RX_TIMEOUT_INT | EMAC_RX_OVERFLOW_INT);

	/* Start RX DMA */
	emac_write(dev, EMAC_RX_CTL1,
		   emac_read(dev, EMAC_RX_CTL1) | EMAC_RX_DMA_START);

	LOG_DBG("EMAC started: TX_CTL0=0x%x TX_CTL1=0x%x RX_CTL0=0x%x RX_CTL1=0x%x",
		emac_read(dev, EMAC_TX_CTL0), emac_read(dev, EMAC_TX_CTL1),
		emac_read(dev, EMAC_RX_CTL0), emac_read(dev, EMAC_RX_CTL1));
	LOG_DBG("  TX_DESC_LIST=0x%x RX_DESC_LIST=0x%x INT_EN=0x%x",
		emac_read(dev, EMAC_TX_DESC_LIST), emac_read(dev, EMAC_RX_DESC_LIST),
		emac_read(dev, EMAC_INT_EN));

	return 0;
}

static int emac_v3s_stop(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(iface);
	uint32_t val;

	/* Stop TX DMA */
	val = emac_read(dev, EMAC_TX_CTL1);
	val &= ~EMAC_TX_DMA_EN;
	emac_write(dev, EMAC_TX_CTL1, val);

	/* Stop RX DMA */
	val = emac_read(dev, EMAC_RX_CTL1);
	val &= ~EMAC_RX_DMA_EN;
	emac_write(dev, EMAC_RX_CTL1, val);

	/* Disable transmitter */
	val = emac_read(dev, EMAC_TX_CTL0);
	val &= ~EMAC_TX_TRANSMITTER_EN;
	emac_write(dev, EMAC_TX_CTL0, val);

	/* Disable receiver */
	val = emac_read(dev, EMAC_RX_CTL0);
	val &= ~EMAC_RX_RECEIVER_EN;
	emac_write(dev, EMAC_RX_CTL0, val);

	/* Disable interrupts */
	emac_write(dev, EMAC_INT_EN, 0);

	LOG_DBG("EMAC stopped");

	return 0;
}

static void phy_link_state_changed(const struct device *phy_dev,
				   struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct emac_v3s_data *data = DEV_DATA(dev);
	uint32_t val;

	ARG_UNUSED(phy_dev);

	if (state->is_up) {
		val = emac_read(dev, EMAC_BASIC_CTL0);
		val &= ~(EMAC_DUPLEX_FULL | EMAC_SPEED_100 | EMAC_SPEED_10);

		switch (state->speed) {
		case LINK_HALF_10BASE:
			val |= EMAC_SPEED_10;
			break;
		case LINK_FULL_10BASE:
			val |= EMAC_SPEED_10 | EMAC_DUPLEX_FULL;
			break;
		case LINK_HALF_100BASE:
			val |= EMAC_SPEED_100;
			break;
		case LINK_FULL_100BASE:
			val |= EMAC_SPEED_100 | EMAC_DUPLEX_FULL;
			break;
		default:
			LOG_WRN("Unsupported link speed: %d", state->speed);
			return;
		}

		emac_write(dev, EMAC_BASIC_CTL0, val);
		net_eth_carrier_on(data->iface);
		LOG_DBG("PHY link up: speed=0x%x", state->speed);
	} else {
		net_eth_carrier_off(data->iface);
		LOG_DBG("PHY link down");
	}
}

static const struct device *emac_v3s_get_phy(const struct device *dev,
					     struct net_if *iface)
{
	ARG_UNUSED(iface);
	const struct emac_v3s_config *cfg = DEV_CFG(dev);

	return cfg->phy_dev;
}

static void emac_v3s_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct emac_v3s_data *data = DEV_DATA(dev);
	const struct emac_v3s_config *cfg = DEV_CFG(dev);

	data->iface = iface;

	ethernet_init(iface);

	/* Set MAC address */
	if (data->mac_addr[0] == 0 &&
	    data->mac_addr[1] == 0 &&
	    data->mac_addr[2] == 0 &&
	    data->mac_addr[3] == 0 &&
	    data->mac_addr[4] == 0 &&
	    data->mac_addr[5] == 0) {
		/* Generate random MAC address if not set */
		gen_random_mac(data->mac_addr, 0x02, 0x00, 0x00);
	}

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	/* Configure interrupts */
	cfg->config_func();

	/* Setup PHY link monitoring */
	if (cfg->phy_dev != NULL) {
		/* Do not start the interface until PHY link is up */
		net_if_carrier_off(iface);

		if (device_is_ready(cfg->phy_dev)) {
			phy_link_callback_set(cfg->phy_dev,
					      phy_link_state_changed,
					      (void *)dev);
		} else {
			LOG_ERR("PHY device not ready");
		}
	}
}

static int emac_v3s_dev_init(const struct device *dev)
{
	struct emac_v3s_data *data = DEV_DATA(dev);
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	/* Configure syscon BEFORE powering up EPHY.
	 * The EPHY latches its configuration (LED polarity, PHY address,
	 * clock select) from the syscon register during power-up/reset.
	 * This must be done before enabling the EPHY clock and deasserting
	 * its reset, following the same sequence as Linux's
	 * sun8i_dwmac_set_syscon() → sun8i_dwmac_power_internal_phy().
	 */
	ret = emac_configure_syscon(dev);
	if (ret < 0) {
		return ret;
	}

	ret = emac_enable_clocks_and_reset(dev);
	if (ret < 0) {
		return ret;
	}

	/* Soft reset EMAC */
	ret = emac_soft_reset(dev);
	if (ret < 0) {
		return ret;
	}

	/* DMA reset: clear all registers */
	emac_write(dev, EMAC_RX_CTL1, 0);
	emac_write(dev, EMAC_TX_CTL1, 0);
	emac_write(dev, EMAC_RX_FRM_FLT, 0);
	emac_write(dev, EMAC_RX_DESC_LIST, 0);
	emac_write(dev, EMAC_TX_DESC_LIST, 0);
	emac_write(dev, EMAC_INT_EN, 0);
	emac_write(dev, EMAC_INT_STA, EMAC_INT_STA_CLEAR);

	/* Configure burst length in BASIC_CTL1 */
	emac_write(dev, EMAC_BASIC_CTL1,
		   EMAC_BURST_LEN << EMAC_BURSTLEN_SHIFT);

	/* Setup DMA descriptors */
	emac_setup_tx_descs(dev);
	emac_setup_rx_descs(dev);

	/* Flush all descriptors to RAM */
	sys_cache_data_flush_range(data->tx_descs,
				   sizeof(data->tx_descs));
	sys_cache_data_flush_range(data->rx_descs,
				   sizeof(data->rx_descs));

	/* Set descriptor list base addresses */
	emac_write(dev, EMAC_TX_DESC_LIST,
		   (uint32_t)(uintptr_t)data->tx_descs);
	emac_write(dev, EMAC_RX_DESC_LIST,
		   (uint32_t)(uintptr_t)data->rx_descs);

	/* Set MAC address */
	emac_set_mac_addr(dev, data->mac_addr);

	/* Frame filter: EMAC_FRM_FLT_CTL (BIT(13)) enables the control frame
	 * filter.  Combined with the MAC address programmed in slot 0, the
	 * hardware accepts unicast frames addressed to our MAC and broadcast
	 * frames.  Matches Linux sun8i_dwmac_set_filter() normal mode.
	 */
	emac_write(dev, EMAC_RX_FRM_FLT, EMAC_FRM_FLT_CTL);

	LOG_INF("V3s EMAC initialized at base 0x%lx",
		(unsigned long)DEVICE_MMIO_GET(dev));
	LOG_INF("  TX desc ring: %u descs at %p, first desc[0].ctl=0x%x",
		TX_DESC_COUNT, (void *)data->tx_descs, data->tx_descs[0].ctl);
	LOG_INF("  RX desc ring: %u descs at %p, first desc[0].ctl=0x%x",
		RX_DESC_COUNT, (void *)data->rx_descs, data->rx_descs[0].ctl);

	return 0;
}

#define EMAC_V3S_IRQ_CONFIG(n)							\
	static void emac_v3s_irq_config_##n(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    emac_v3s_isr, DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQN(n));					\
	}

#define EMAC_V3S_INIT(n)								\
	EMAC_V3S_IRQ_CONFIG(n);							\
											\
	static const struct emac_v3s_config emac_v3s_config_##n = {			\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),					\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),			\
		.reset_dev = DEVICE_DT_GET(DT_INST_RESET_CTLR(n)),			\
		.syscon_dev = DEVICE_DT_GET(DT_NODELABEL(syscon)),			\
		.phy_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(n, phy_handle)),	\
		.phy_addr = DT_INST_PROP_OR(n, allwinner_phy_addr, 1),			\
		.leds_active_low = DT_INST_PROP_OR(n, allwinner_leds_active_low, 0),	\
		.config_func = emac_v3s_irq_config_##n,				\
	};										\
											\
	static struct emac_v3s_data emac_v3s_data_##n = {				\
		.tx_busy = false,							\
		.rx_pkt = NULL,								\
		.mac_addr = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00},			\
	};										\
											\
	static const struct ethernet_api emac_v3s_api_##n = {				\
		.iface_api.init = emac_v3s_iface_init,					\
		.start = emac_v3s_start,						\
		.stop = emac_v3s_stop,							\
		.send = emac_v3s_send,							\
		.get_capabilities = emac_v3s_get_capabilities,				\
		.set_config = emac_v3s_set_config,					\
		.get_phy = emac_v3s_get_phy,						\
	};										\
											\
	ETH_NET_DEVICE_DT_INST_DEFINE(n, emac_v3s_dev_init, NULL,			\
				      &emac_v3s_data_##n, &emac_v3s_config_##n,	\
				      CONFIG_ETH_INIT_PRIORITY,				\
				      &emac_v3s_api_##n, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(EMAC_V3S_INIT)
