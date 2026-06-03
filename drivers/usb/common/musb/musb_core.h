/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MUSB HDRC controller core — shared between UDC and UHC.
 *
 * Three-layer architecture:
 *   Core  (this file) - hardware abstraction, ISR dispatch, FIFO management
 *   Glue  (glue dir) - platform-specific register mapping, PHY control
 *   Adapter (udc dir) - Zephyr UDC API implementation
 *
 * Glue registration: single weak function pattern.
 * Each glue layer provides a static struct musb_glue_ops + struct musb_glue_config
 * and registers via MUSB_REGISTER_GLUE().
 */

#ifndef ZEPHYR_DRIVERS_USB_COMMON_MUSB_MUSB_CORE_H_
#define ZEPHYR_DRIVERS_USB_COMMON_MUSB_MUSB_CORE_H_

#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include "musb_regs.h"

/* ========== Quirks ========== */

#define MUSB_INDEXED_EP     BIT(0)   /* Indexed EP mode (Sunxi), not Flat */
#define MUSB_NO_CONFIGDATA  BIT(1)   /* No CONFIGDATA register, use hardcoded value */
#define MUSB_DISABLE_HS     BIT(2)   /* FS-only PHY (V3s/A33); skip HSEN chirp */

/* ========== FIFO Configuration ========== */

enum musb_fifo_style {
	FIFO_RXTX,
	FIFO_TX,
	FIFO_RX,
};

struct musb_fifo_cfg {
	uint8_t  hw_ep_num;
	enum musb_fifo_style style;
	uint16_t maxpacket;
};

/* ========== EP0 State Machine ========== */

enum musb_ep0_stage {
	MUSB_EP0_STAGE_IDLE,
	MUSB_EP0_STAGE_SETUP,
	MUSB_EP0_STAGE_TX,
	MUSB_EP0_STAGE_RX,
	MUSB_EP0_STAGE_STATUSIN,
	MUSB_EP0_STAGE_STATUSOUT,
	MUSB_EP0_STAGE_ACKWAIT,     /* zero-data SETUP waiting for status phase */
};

/* ========== Hardware Endpoint State ========== */

struct musb_hw_ep {
	uint16_t max_packet_sz_tx;
	uint16_t max_packet_sz_rx;
	uint16_t fifo_addr;       /* FIFO start address in 8-byte units */

	/* Multi-packet TX state (valid while TX transfer is active) */
	const uint8_t *tx_buf;    /* Current TX data pointer (from net_buf) */
	uint16_t tx_len;          /* Total bytes to send */
	uint16_t tx_sent;         /* Bytes sent so far */
};

/* ========== Forward Declaration ========== */

struct musb_dev;

/* ========== Bus Events (Glue → Core → Adapter) ========== */

enum musb_bus_event {
	MUSB_BUS_RESET,
	MUSB_BUS_SUSPEND,
	MUSB_BUS_RESUME,
	MUSB_BUS_DISCONNECT,
	MUSB_BUS_VBUS_ERROR,
	MUSB_BUS_SOF,
};

/* ========== Callbacks (Core → Adapter) ========== */

struct musb_callbacks {
	/**
	 * @brief EP0 SETUP packet received.
	 * The adapter layer should call udc_submit_event() or handle the
	 * setup packet via udc_setup_received().
	 */
	void (*ep0_setup)(struct musb_dev *musb,
			  const void *setup_data);

	/** TX transfer completed on given endpoint */
	void (*ep_tx_done)(struct musb_dev *musb, uint8_t ep);

	/** RX data ready on given endpoint, len bytes available in FIFO */
	void (*ep_rx_ready)(struct musb_dev *musb, uint8_t ep,
			    uint16_t len);

	/** Bus event (reset/suspend/resume/disconnect) */
	void (*bus_event)(struct musb_dev *musb,
			  enum musb_bus_event event);
};

/* ========== Glue Ops Table ========== */

struct musb_glue_ops {
	/* --- Register I/O --- */
	uint8_t  (*readb)(mm_reg_t base, uint32_t offset);
	void     (*writeb)(mm_reg_t base, uint32_t offset, uint8_t data);
	uint16_t (*readw)(mm_reg_t base, uint32_t offset);
	void     (*writew)(mm_reg_t base, uint32_t offset, uint16_t data);

	/* FIFO access (byte-at-a-time for PIO mode) */
	void (*read_fifo)(mm_reg_t fifo, uint16_t len, uint8_t *buf);
	void (*write_fifo)(mm_reg_t fifo, uint16_t len, const uint8_t *buf);

	/* --- Address Offset Calculation --- */
	uint32_t (*ep_offset)(uint8_t epnum, uint16_t offset);
	uint32_t (*fifo_offset)(uint8_t epnum);
	uint32_t (*busctl_offset)(uint8_t epnum, uint16_t offset);

	/* --- Platform Lifecycle --- */
	int  (*init)(const struct device *dev);
	void (*enable)(const struct device *dev);
	void (*disable)(const struct device *dev);

	/* --- DMA (optional, NULL = PIO-only) --- */
	/**
	 * @brief Prepare DMA channel for an endpoint.
	 * Called during ep_enable.  Must set TXCSR_DMAMODE / RXCSR_DMAENAB
	 * in the hardware CSR if successful.
	 * @returns 0 on success, negative on failure (falls back to PIO)
	 */
	int  (*dma_ep_prepare)(const struct device *dev, uint8_t ep,
			       bool is_tx, uint8_t drq, uint16_t maxpacket);
	/** Release DMA channel for an endpoint */
	void (*dma_ep_unprepare)(const struct device *dev, uint8_t ep,
				 bool is_tx);
	/**
	 * @brief Start a DMA transfer.
	 * @param cb  called from DMA ISR when transfer completes
	 * @param user_data  passed to cb
	 */
	int  (*dma_ep_transfer)(const struct device *dev, uint8_t ep,
				bool is_tx, uint8_t *buf, uint32_t len,
				void (*cb)(void *user_data, int result),
				void *user_data);
	/** Abort an in-flight DMA transfer */
	int  (*dma_ep_abort)(const struct device *dev, uint8_t ep,
			     bool is_tx);
};

/* ========== Glue Config (Compile-time platform constants) ========== */

struct musb_glue_config {
	const struct musb_fifo_cfg *fifo_cfg;
	size_t fifo_cfg_size;
	uint8_t num_eps;           /* Total endpoints including EP0 */
	uint8_t ram_bits;          /* FIFO RAM size in 2^n bytes (11 = 2KB) */
	uint32_t quirks;           /* MUSB_INDEXED_EP | MUSB_NO_CONFIGDATA ... */
	uint8_t configdata_val;    /* Hardcoded CONFIGDATA value */
	uint16_t epmask;            /* Bitmask of available endpoints (for INTRTXE/INTRRXE) */
	clock_control_subsys_t clk_subsys;  /* Bus clock subsys from DT */
	uint32_t rst_id;                    /* Bus reset ID from DT */
	/*
	 * DMA channel map: dma_tx_drq[ep] and dma_rx_drq[ep] give the
	 * system DMA DRQ / dma_slot numbers for each endpoint.
	 * 0 means no DMA for that EP/direction.
	 * EP0 is always PIO (index 0 is unused).
	 */
	uint8_t dma_tx_drq[16];
	uint8_t dma_rx_drq[16];
};

/* ========== MUSB Device Instance ========== */

struct musb_dev {
	const struct musb_glue_ops *ops;
	const struct musb_glue_config *config;
	const struct musb_callbacks *callbacks;
	mm_reg_t base;
	uint8_t ep0_stage;
	uint8_t int_usb;
	uint16_t int_tx;
	uint16_t int_rx;
	uint16_t ackpend;             /* Deferred CSR0 write bits (Linux musb->ackpend) */
	bool     set_address;         /* SET_ADDRESS pending until status phase */
	uint8_t  address;             /* New device address to apply */
	struct musb_hw_ep *hw_eps;
	uint16_t dma_tx_mask;      /* Bitmask: EPs with DMA-TX active */
	uint16_t dma_rx_mask;      /* Bitmask: EPs with DMA-RX active */
	const struct device *glue_dev; /* Glue layer device for DMA ops */
	const struct device *clk_dev;
	const struct device *rst_dev;
	const struct sunxi_usb_phy *phy_dev;
	void *user_data;            /* Adapter layer private data */

	/*
	 * EP0 multi-packet TX state — ISR-driven chunking.
	 *
	 * When the USB stack provides more data than EP0 max packet (64 bytes),
	 * musb_ep_tx() caches the full request here and sends the first chunk.
	 * Subsequent chunks are sent by ep0_txstate() from the ISR, matching
	 * Linux's ep0_txstate() / musb_gadget_ep0.c behavior.
	 */
	const uint8_t *ep0_tx_buf;    /* Full TX buffer (valid across packets) */
	uint16_t ep0_tx_len;          /* Total bytes to send */
	uint16_t ep0_tx_sent;         /* Bytes sent so far */
};

/* EP0 max packet size — USB 2.0 fixed at 64 bytes */
#define MUSB_EP0_MAX_PACKET  64

/* ========== Glue Registration ========== */

struct musb_glue_info {
	const struct musb_glue_ops *ops;
	const struct musb_glue_config *config;
};

/**
 * @brief Weak function: core provides a default that returns -ENODEV.
 * Glue layers override with MUSB_REGISTER_GLUE().
 */
__weak int musb_platform_info_get(const struct musb_glue_info **info);

/**
 * @brief Register a platform glue ops + config.
 *
 * Usage in glue .c file:
 *   static const struct musb_glue_ops my_ops = { ... };
 *   static const struct musb_glue_config my_config = { ... };
 *   MUSB_REGISTER_GLUE(&my_ops, &my_config);
 */
#define MUSB_REGISTER_GLUE(_ops, _config)                              \
	int musb_platform_info_get(const struct musb_glue_info **info) \
	{                                                              \
		static const struct musb_glue_info _glue_info = {      \
			.ops = (_ops),                                 \
			.config = (_config),                           \
		};                                                     \
		*info = &_glue_info;                                   \
		return 0;                                              \
	}

/* ========== Exported I/O Helpers ========== */

/*
 * Control register access: addr = musb->base.
 * The glue layer's readb/writeb uses the addr parameter to distinguish
 * control registers from EP CSR registers (Linux architecture).
 */
uint8_t  musb_readb(const struct musb_dev *musb, uint32_t offset);
void     musb_writeb(const struct musb_dev *musb, uint32_t offset, uint8_t val);
uint16_t musb_readw(const struct musb_dev *musb, uint32_t offset);
void     musb_writew(const struct musb_dev *musb, uint32_t offset, uint16_t val);

/*
 * EP CSR register access: addr = musb->base + ep_offset(ep, 0).
 * Matches Linux's pattern where hw_ep->regs = mregs + ep_offset(i, 0)
 * and the per-register offset is added at the access site.
 *
 * These helpers also call musb_ep_select() for indexed EP mode.
 */
static inline uint16_t musb_ep_readw(const struct musb_dev *musb,
				      uint8_t ep, uint16_t offset)
{
	mm_reg_t ep_base = musb->base + musb->ops->ep_offset(ep, 0);
	return musb->ops->readw(ep_base, offset);
}

static inline void musb_ep_writew(const struct musb_dev *musb,
				   uint8_t ep, uint16_t offset, uint16_t val)
{
	mm_reg_t ep_base = musb->base + musb->ops->ep_offset(ep, 0);
	musb->ops->writew(ep_base, offset, val);
}

static inline uint8_t musb_ep_readb(const struct musb_dev *musb,
				     uint8_t ep, uint16_t offset)
{
	mm_reg_t ep_base = musb->base + musb->ops->ep_offset(ep, 0);
	return musb->ops->readb(ep_base, offset);
}

static inline void musb_ep_writeb(const struct musb_dev *musb,
				   uint8_t ep, uint16_t offset, uint8_t val)
{
	mm_reg_t ep_base = musb->base + musb->ops->ep_offset(ep, 0);
	musb->ops->writeb(ep_base, offset, val);
}

/* ========== Core Function Declarations ========== */

int  musb_core_init(struct musb_dev *musb);
void musb_irq_handler(struct musb_dev *musb);

int  musb_ep_enable(struct musb_dev *musb, uint8_t ep,
		    uint8_t ep_type, uint16_t mps);
int  musb_ep_disable(struct musb_dev *musb, uint8_t ep);

int  musb_ep_tx(struct musb_dev *musb, uint8_t ep,
		const uint8_t *data, uint16_t len);
int  musb_ep_rx(struct musb_dev *musb, uint8_t ep);

int  musb_ep_set_halt(struct musb_dev *musb, uint8_t ep, bool stall);

void musb_set_address(struct musb_dev *musb, uint8_t addr);
void musb_connect(struct musb_dev *musb);
void musb_disconnect(struct musb_dev *musb);
struct musb_dev *to_musb(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_USB_COMMON_MUSB_MUSB_CORE_H_ */
