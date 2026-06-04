/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_SUN8I_V3S_EMAC_PRIV_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_SUN8I_V3S_EMAC_PRIV_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/syscon.h>

/* EMAC register offsets (from dwmac-sun8i.c) */
#define EMAC_BASIC_CTL0		0x00
#define EMAC_BASIC_CTL1		0x04
#define EMAC_INT_STA		0x08
#define EMAC_INT_EN		0x0C
#define EMAC_TX_CTL0		0x10
#define EMAC_TX_CTL1		0x14
#define EMAC_TX_FLOW_CTL	0x1C
#define EMAC_TX_DESC_LIST	0x20
#define EMAC_RX_CTL0		0x24
#define EMAC_RX_CTL1		0x28
#define EMAC_RX_DESC_LIST	0x34
#define EMAC_RX_FRM_FLT	0x38
#define EMAC_MDIO_CMD		0x48
#define EMAC_MDIO_DATA		0x4C
#define EMAC_MACADDR_HI(n)	(0x50 + (n) * 8)
#define EMAC_MACADDR_LO(n)	(0x54 + (n) * 8)
#define EMAC_TX_DMA_STA		0xB0
#define EMAC_TX_CUR_DESC	0xB4
#define EMAC_TX_CUR_BUF		0xB8
#define EMAC_RX_DMA_STA		0xC0
#define EMAC_RX_CUR_DESC	0xC4
#define EMAC_RX_CUR_BUF		0xC8

/* BASIC_CTL0 bits */
#define EMAC_DUPLEX_FULL		BIT(0)
#define EMAC_LOOPBACK			BIT(1)
#define EMAC_SPEED_1000			0
#define EMAC_SPEED_100			(0x03 << 2)
#define EMAC_SPEED_10			(0x02 << 2)

/* BASIC_CTL1 bits */
#define EMAC_BURSTLEN_SHIFT		24
#define EMAC_SOFT_RST			BIT(0)

/* INT_STA / INT_EN bits */
#define EMAC_TX_INT			BIT(0)
#define EMAC_TX_DMA_STOP_INT		BIT(1)
#define EMAC_TX_BUF_UA_INT		BIT(2)
#define EMAC_TX_TIMEOUT_INT		BIT(3)
#define EMAC_TX_UNDERFLOW_INT		BIT(4)
#define EMAC_TX_EARLY_INT		BIT(5)
#define EMAC_RX_INT			BIT(8)
#define EMAC_RX_BUF_UA_INT		BIT(9)
#define EMAC_RX_DMA_STOP_INT		BIT(10)
#define EMAC_RX_TIMEOUT_INT		BIT(11)
#define EMAC_RX_OVERFLOW_INT		BIT(12)
#define EMAC_RX_EARLY_INT		BIT(13)
#define EMAC_RGMII_STA_INT		BIT(16)

#define EMAC_INT_STA_CLEAR		0x1FFFFFF

/* TX_CTL0 bits */
#define EMAC_TX_TRANSMITTER_EN		BIT(31)

/* TX_CTL1 bits */
#define EMAC_TX_MD			BIT(1)
#define EMAC_TX_NEXT_FRM		BIT(2)
#define EMAC_TX_TH_MASK			GENMASK(10, 8)
#define EMAC_TX_DMA_EN			BIT(30)
#define EMAC_TX_DMA_START		BIT(31)

/* RX_CTL0 bits */
#define EMAC_RX_RECEIVER_EN		BIT(31)
#define EMAC_RX_DO_CRC			BIT(27)
#define EMAC_RX_FLOW_CTL_EN		BIT(16)

/* RX_CTL1 bits */
#define EMAC_RX_MD			BIT(1)
#define EMAC_RX_TH_MASK			GENMASK(5, 4)
#define EMAC_RX_DMA_EN			BIT(30)
#define EMAC_RX_DMA_START		BIT(31)

/* RX_FRM_FLT bits */
#define EMAC_FRM_FLT_RXALL		BIT(0)
#define EMAC_FRM_FLT_CTL		BIT(13)
#define EMAC_FRM_FLT_MULTICAST		BIT(16)

/* TX_FLOW_CTL bits */
#define EMAC_TX_FLOW_CTL_EN		BIT(0)

/* MAC address bits */
#define MAC_ADDR_TYPE_DST		BIT(31)

/* MDIO_CMD bits */
#define EMAC_MII_BUSY			BIT(0)
#define EMAC_MII_WRITE			BIT(1)
#define EMAC_MII_CLK_SEL_SHIFT		20
#define EMAC_MII_CLK_SEL_MASK		GENMASK(22, 20)
#define EMAC_MII_PHY_ADDR_SHIFT		12
#define EMAC_MII_PHY_ADDR_MASK		GENMASK(16, 12)
#define EMAC_MII_REG_ADDR_SHIFT		4
#define EMAC_MII_REG_ADDR_MASK		GENMASK(8, 4)

/* syscon register at offset 0x30 - EPHY control */
#define SYSCON_EPHY_REG			0x30
#define H3_EPHY_ADDR_SHIFT		20
#define H3_EPHY_CLK_SEL			BIT(18)
#define H3_EPHY_LED_POL			BIT(17)
#define H3_EPHY_SHUTDOWN		BIT(16)
#define H3_EPHY_SELECT			BIT(15)
#define H3_EPHY_MUX_MASK		(H3_EPHY_SHUTDOWN | H3_EPHY_SELECT)

#define V3S_DEFAULT_SYSCON_VALUE	0x38000

/* TX descriptor bits (TDES1, Synopsys normal descriptor) */
#define TX_DESC_CTL			BIT(31)  /* TDES0_OWN: DMA owns descriptor */
#define TX_INT_CTL			BIT(31)  /* TDES1_INTERRUPT */
#define TX_LAST_DESC			BIT(30)  /* TDES1_LAST_SEGMENT */
#define TX_FIR_DESC			BIT(29)  /* TDES1_FIRST_SEGMENT */
#define TX_SECOND_ADDR_CHAINED		BIT(24)  /* TDES1_SECOND_ADDRESS_CHAINED */
#define TX_CHECKSUM_CTL_SHIFT		27
#define TX_CRC_CTL			BIT(26)  /* TDES1_CRC_DISABLE */

/* RX descriptor status bits (RDES0 write-back, Synopsys normal descriptor) */
#define RX_DESC_CTL			BIT(31)  /* OWN: 1=DMA owns, 0=host owns */
#define RX_DAF_FAIL			BIT(30)  /* Destination Address Filter Fail */
#define RX_FRM_LEN_SHIFT		16
#define RX_FRM_LEN_MASK		GENMASK(29, 16)
#define RX_ERR_SUMMARY			BIT(15)  /* Error Summary */
#define RX_DESC_ERR			BIT(14)  /* Descriptor Error */
#define RX_SA_FILTER_FAIL		BIT(13)  /* Source Address Filter Fail */
#define RX_LENGTH_ERR			BIT(12)  /* Length Error */
#define RX_OVERFLOW_ERR		BIT(11)  /* Overflow Error (RX FIFO) */
#define RX_VLAN_TAG			BIT(10)  /* VLAN Tag detected */
#define RX_FIRST_DESC			BIT(9)   /* First Descriptor */
#define RX_LAST_DESC			BIT(8)   /* Last Descriptor */
#define RX_IPC_CSUM_ERR		BIT(7)   /* IPC Header Checksum Error */
#define RX_COLLISION			BIT(6)   /* Late Collision */
#define RX_FRAME_TYPE			BIT(5)   /* Frame Type */
#define RX_WATCHDOG_ERR		BIT(4)   /* Receive Watchdog Timeout */
#define RX_MII_ERR			BIT(3)   /* MII Error */
#define RX_DRIBBLING			BIT(2)   /* Dribbling (extra nibble) */
#define RX_CRC_ERR			BIT(1)   /* CRC Error */
#define RX_PAYLOAD_CSUM_ERR		BIT(0)   /* Payload Checksum Error/Giant Frame */

/* RX descriptor ctl bits (RDES1) */
#define RX_SECOND_ADDR_CHAINED		BIT(24)  /* RDES1_SECOND_ADDRESS_CHAINED */
#define RX_INT_DISABLE			BIT(31)  /* Disable interrupt on completion */

/*
 * DMA descriptor: 4 words for DMA hardware + padding.
 *
 * Padded to DCACHE_LINE_SIZE (32 bytes on Cortex-A7) so that each
 * descriptor occupies exactly one cache line.  This is critical for
 * cache coherency: when arch_dcache_invd_range() receives a size
 * equal to the cache line size on a cache-line-aligned address it
 * takes the "fast path" that uses pure L1C_InvalidateDCacheMVA()
 * (invalidate without clean).  Without the padding the descriptor is
 * only 16 bytes, which the invalidation code treats as a partial
 * cache line and falls back to L1C_CleanInvalidateDCacheMVA().
 * The "clean" half of that operation writes the CPU's stale cached
 * copy back to RAM, overwriting the DMA engine's updated descriptor
 * and causing the driver to never see OWN=0.
 *
 * The DMA hardware reads only words 0-3; the padding is ignored.
 */
struct emac_dma_desc {
	uint32_t status;	/* Word 0 */
	uint32_t ctl;		/* Word 1 */
	uint32_t buf_addr;	/* Word 2 */
	uint32_t next;		/* Word 3 */
	uint32_t _pad[4];	/* Pad to 32 bytes (one cache line) */
} __aligned(32);

#define TX_DESC_COUNT	CONFIG_ETH_SUN8I_V3S_EMAC_TX_DESC_COUNT
#define RX_DESC_COUNT	CONFIG_ETH_SUN8I_V3S_EMAC_RX_DESC_COUNT

/* Maximum frame size: 1536 (standard Ethernet + some margin) */
#define EMAC_MAX_FRAME_SIZE		1536

/* MDIO timeout (microseconds) */
#define EMAC_MDIO_TIMEOUT_US		10000

/* Soft reset timeout (microseconds) */
#define EMAC_RESET_TIMEOUT_US		100000

/* Default burst length */
#define EMAC_BURST_LEN			8

/*
 * Device configuration (const, from DT)
 */
struct emac_v3s_config {
	DEVICE_MMIO_ROM;
	const struct device *clock_dev;
	const struct device *reset_dev;
	const struct device *syscon_dev;
	const struct device *phy_dev;
	uint8_t phy_addr;
	bool leds_active_low;
	void (*config_func)(void);
};

/*
 * Device runtime data
 */
struct emac_v3s_data {
	DEVICE_MMIO_RAM;
	struct net_if *iface;
	uint8_t mac_addr[6];
	bool tx_busy;

	/* TX DMA descriptor ring */
	struct emac_dma_desc tx_descs[TX_DESC_COUNT];
	uint8_t tx_bufs[TX_DESC_COUNT][EMAC_MAX_FRAME_SIZE]
		__aligned(32);
	uint32_t tx_desc_idx;       /* Next free descriptor index */
	uint32_t tx_desc_submitted; /* Index of last submitted descriptor */

	/* RX DMA descriptor ring */
	struct emac_dma_desc rx_descs[RX_DESC_COUNT];
	uint8_t rx_bufs[RX_DESC_COUNT][EMAC_MAX_FRAME_SIZE]
		__aligned(32);
	uint32_t rx_desc_idx;

	/* RX packet assembly */
	struct net_pkt *rx_pkt;
};

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_SUN8I_V3S_EMAC_PRIV_H_ */
