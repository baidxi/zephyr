/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MUSB HDRC controller core implementation.
 * Ported from Linux drivers/usb/musb/musb_core.c + musb_gadget.c + musb_gadget_ep0.c
 *
 * EP0 state machine and ISR dispatch follow Linux musb_g_ep0_irq() exactly.
 */

#include "musb_core.h"

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(musb_core, CONFIG_MUSB_LOG_LEVEL);

__weak int musb_platform_info_get(const struct musb_glue_info **info)
{
	*info = NULL;
	return -ENODEV;
}

uint8_t musb_readb(const struct musb_dev *musb, uint32_t offset)
{
	return musb->ops->readb(musb->base, offset);
}

void musb_writeb(const struct musb_dev *musb, uint32_t offset, uint8_t val)
{
	musb->ops->writeb(musb->base, offset, val);
}

uint16_t musb_readw(const struct musb_dev *musb, uint32_t offset)
{
	return musb->ops->readw(musb->base, offset);
}

void musb_writew(const struct musb_dev *musb, uint32_t offset, uint16_t val)
{
	musb->ops->writew(musb->base, offset, val);
}

static inline void musb_ep_select(const struct musb_dev *musb, uint8_t ep)
{
	if (musb->config->quirks & MUSB_INDEXED_EP) {
		musb_writeb(musb, MUSB_INDEX, ep);
	}
}
static inline uint32_t musb_fifo_offset(const struct musb_dev *musb, uint8_t ep)
{
	return musb->ops->fifo_offset(ep);
}

/* Convert a max-packet size to hardware FIFOSZ encoding (log2(size / 8)) */
static uint8_t musb_fifosize_encode(uint16_t maxpacket)
{
	uint32_t size = maxpacket / 8;
	uint8_t bits = 0;

	while (size > 1) {
		size >>= 1;
		bits++;
	}
	return bits;
}

static int musb_fifo_setup(struct musb_dev *musb, const struct musb_fifo_cfg *cfg,
			   uint16_t *fifo_addr)
{
	uint8_t ep = cfg->hw_ep_num;
	uint16_t maxpacket = cfg->maxpacket;
	uint16_t addr = *fifo_addr;
	uint8_t fifosize = musb_fifosize_encode(maxpacket);

	musb_ep_select(musb, ep);

	switch (cfg->style) {
	case FIFO_TX:
		musb_writeb(musb, MUSB_TXFIFOSZ, fifosize);
		musb_writew(musb, MUSB_TXFIFOADD, addr);
		musb->hw_eps[ep].max_packet_sz_tx = maxpacket;
		break;
	case FIFO_RX:
		musb_writeb(musb, MUSB_RXFIFOSZ, fifosize);
		musb_writew(musb, MUSB_RXFIFOADD, addr);
		musb->hw_eps[ep].max_packet_sz_rx = maxpacket;
		break;
	default:
		return -EINVAL;
	}

	musb->hw_eps[ep].fifo_addr = addr;
	*fifo_addr += maxpacket / 8;

	return 0;
}

int musb_core_init(struct musb_dev *musb)
{
	const struct musb_fifo_cfg *fifo_cfg = musb->config->fifo_cfg;
	size_t fifo_cfg_size = musb->config->fifo_cfg_size;
	uint16_t fifo_addr = 0; /* Start of shared FIFO RAM */
	int ret;

	/* 1. Disable all interrupts */
	musb_writeb(musb, MUSB_INTRUSBE, 0);
	musb_writew(musb, MUSB_INTRTXE, 0);
	musb_writew(musb, MUSB_INTRRXE, 0);

	/*
	 * 2. Init controller mode.  Set HSEN only if the PHY supports HS;
	 * on FS-only PHYs (sunxi V3s/A33) HS chirp negotiation corrupts
	 * the data rate and causes host-side protocol errors (-71).
	 */
	if (!(musb->config->quirks & MUSB_DISABLE_HS)) {
		musb_writeb(musb, MUSB_POWER, MUSB_POWER_HSENAB);
	} else {
		/* FS-only: explicitly clear all POWER bits */
		musb_writeb(musb, MUSB_POWER, 0x00);
	}
	k_busy_wait(100);

	musb->ep0_stage = MUSB_EP0_STAGE_SETUP;  /* Linux: musb_g_reset() */
	musb->ackpend = 0;
	musb->set_address = false;
	musb->address = 0;

	LOG_DBG("core_init: base=0x%lx POWER=0x%02x DEVCTL=0x%02x", (unsigned long)musb->base,
		musb_readb(musb, MUSB_POWER), musb_readb(musb, MUSB_DEVCTL));

	/* 3. Select EP0; no explicit FIFO config needed (fixed 64-byte bidir) */
	musb_ep_select(musb, 0);

	/* 4. Configure FIFO for each endpoint (EP1+) */
	for (size_t i = 0; i < fifo_cfg_size; i++) {
		ret = musb_fifo_setup(musb, &fifo_cfg[i], &fifo_addr);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static void musb_handle_stage0(struct musb_dev *musb)
{
	uint8_t int_usb = musb->int_usb;

	if (int_usb & MUSB_INTR_RESET) {
		/*
		 * USB bus reset: match Linux musb_g_reset() and
		 * sunxi_musb_interrupt() exactly.
		 *
		 * The bus reset automatically clears the FIFO and
		 * resets the data toggle.  Do NOT call FlushFIFO —
		 * per R818 manual, FlushFIFO should only be used when
		 * RxPktRdy or TxPktRdy is set.
		 */

		/* Linux sunxi.c: explicitly write FADDR=0 on RESET */
		musb_set_address(musb, 0);

		musb->set_address = false;
		musb->address = 0;
		musb->ackpend = 0;
		musb->ep0_stage = MUSB_EP0_STAGE_SETUP;

		/* Re-enable EP0 interrupts (cleared by bus reset) */
		musb_writew(musb, MUSB_INTRTXE,
			    musb_readw(musb, MUSB_INTRTXE) | BIT(0));

		LOG_DBG("RESET: POWER=0x%02x DEVCTL=0x%02x",
			musb_readb(musb, MUSB_POWER),
			musb_readb(musb, MUSB_DEVCTL));

		if (musb->callbacks->bus_event) {
			musb->callbacks->bus_event(musb, MUSB_BUS_RESET);
		}
	}

	if (int_usb & MUSB_INTR_SUSPEND) {
		if (musb->callbacks->bus_event) {
			musb->callbacks->bus_event(musb, MUSB_BUS_SUSPEND);
		}
	}

	if (int_usb & MUSB_INTR_DISCONNECT) {
		musb->ep0_stage = MUSB_EP0_STAGE_SETUP;
		musb->ackpend = 0;
		if (musb->callbacks->bus_event) {
			musb->callbacks->bus_event(musb, MUSB_BUS_DISCONNECT);
		}
	}

	if (int_usb & MUSB_INTR_SOF) {
		if (musb->callbacks->bus_event) {
			musb->callbacks->bus_event(musb, MUSB_BUS_SOF);
		}
	}

	if (int_usb & MUSB_INTR_VBUSERROR) {
		if (musb->callbacks->bus_event) {
			musb->callbacks->bus_event(musb, MUSB_BUS_VBUS_ERROR);
		}
	}
}

/*
 * Send the next chunk of EP0 TX data from ISR context.
 *
 * Reads progress from musb->ep0_tx_* fields (cached by musb_ep_tx()),
 * writes the next min(remaining, maxpacket) bytes to FIFO, and sets
 * TXPKTRDY.  DATAEND is set only on the last packet.
 *
 * Mirrors Linux ep0_txstate() — musb_gadget_ep0.c:517-565.
 */
static void ep0_txstate(struct musb_dev *musb)
{
	uint16_t remaining = musb->ep0_tx_len - musb->ep0_tx_sent;
	uint16_t chunk = remaining > MUSB_EP0_MAX_PACKET
			 ? MUSB_EP0_MAX_PACKET : remaining;
	mm_reg_t fifo = musb->base + musb_fifo_offset(musb, 0);

	/* Write the next chunk to FIFO */
	musb->ops->write_fifo(fifo, chunk,
			      musb->ep0_tx_buf + musb->ep0_tx_sent);
	musb->ep0_tx_sent += chunk;

	/*
	 * Set DATAEND when this is the last packet.
	 * Linux condition: fifo_count < MUSB_MAX_END0_PACKET
	 *   || (request->actual == request->length && !request->zero)
	 * We don't support ZLP for EP0 yet.
	 */
	uint16_t csr = MUSB_CSR0_TXPKTRDY;

	if (musb->ep0_tx_sent == musb->ep0_tx_len) {
		csr |= MUSB_CSR0_P_DATAEND;
		musb->ep0_stage = MUSB_EP0_STAGE_STATUSOUT;
	}
	/* else: stay in EP0_STAGE_TX — next TX interrupt calls us again */

	musb_ep_select(musb, 0);
	musb_ep_writew(musb, 0, MUSB_CSR0, csr);
	/* Read-back to ensure the write has taken effect */
	(void)musb_ep_readw(musb, 0, MUSB_CSR0);

	LOG_DBG("ep0_txstate: chunk=%u sent=%u/%u csr=0x%04x stage=%d",
		chunk, musb->ep0_tx_sent, musb->ep0_tx_len,
		csr, musb->ep0_stage);
}

/*
 * Read a chunk of data from EP0 FIFO.
 * Called when RXPKTRDY is set and we're in RX stage.
 *
 * Mirrors Linux ep0_rxstate() — musb_gadget_ep0.c:460-509.
 */
static void ep0_rxstate(struct musb_dev *musb)
{
	uint16_t count;

	musb_ep_select(musb, 0);
	count = musb_ep_readw(musb, 0, MUSB_COUNT0);

	/*
	 * For RX data on EP0, we notify the adapter layer which reads
	 * the FIFO.  The adapter will then call musb_ep_rx() to re-arm.
	 *
	 * We set ackpend = SVDRXPKTRDY (and possibly DATAEND).
	 * The adapter callback (ep_rx_ready) can choose to stall
	 * by modifying ackpend.
	 */
	musb->ackpend = MUSB_CSR0_P_SVDRXPKTRDY;

	/*
	 * Short packet (< max_packet_sz_rx) or all data received: also set
	 * DATAEND and transition to STATUSIN.
	 */
	if (count < musb->hw_eps[0].max_packet_sz_rx) {
		musb->ackpend |= MUSB_CSR0_P_DATAEND;
		musb->ep0_stage = MUSB_EP0_STAGE_STATUSIN;
	}

	if (musb->callbacks->ep_rx_ready) {
		musb->callbacks->ep_rx_ready(musb, 0, count);
	}

	/* Write ackpend now (adapter may have modified it via stall) */
	if (musb->ackpend) {
		musb_ep_select(musb, 0);
		musb_ep_writew(musb, 0, MUSB_CSR0, musb->ackpend);
		musb->ackpend = 0;
	}
}

/*
 * Read SETUP packet from FIFO and determine the next EP0 state.
 *
 * Mirrors Linux musb_read_setup() + ep0_setup() —
 *   musb_gadget_ep0.c:574-620.
 */
static void ep0_read_setup(struct musb_dev *musb)
{
	mm_reg_t fifo = musb->base + musb_fifo_offset(musb, 0);
	uint8_t setup_data[8];
	uint8_t bmRequestType;
	uint16_t wLength;
	uint16_t count;
	int retry;

	/*
	 * On Allwinner MUSB (R818/V3s), COUNT0 may read back as an
	 * intermediate value (0-7) immediately after RXPKTRDY is set.
	 * Retry up to 16 times waiting for it to reach >=8.
	 * A genuine STATUS stage (0 bytes) will stay at 0.
	 * Matches Linux sunxi_udc_read_fifo_crq() and original Zephyr code.
	 */
	count = musb_ep_readw(musb, 0, MUSB_COUNT0);
	for (retry = 0; retry < 16 && count < 8; retry++) {
		count = musb_ep_readw(musb, 0, MUSB_COUNT0);
	}

	if (count < 8) {
		/*
		 * This is a STATUS stage (zero-length) or residual data.
		 * Drain bytes left in the FIFO so they don't corrupt
		 * the next SETUP packet.
		 *
		 * CRITICAL: Clear RXPKTRDY after draining.  If RXPKTRDY
		 * remains set, the hardware cannot properly accept the next
		 * SETUP packet — the FIFO write pointer may not reset,
		 * leading to corrupted SETUP data on sunxi MUSB.
		 */
		LOG_DBG("ep0: STATUS stage, count=%u, draining", count);
		while (count >= 4) {
			sys_read32(fifo);
			count -= 4;
		}
		while (count > 0) {
			sys_read8(fifo);
			count--;
		}
		musb_ep_select(musb, 0);
		musb_ep_writew(musb, 0, MUSB_CSR0, MUSB_CSR0_P_SVDRXPKTRDY);
		return;  /* STATUS stage, no SETUP to process */
	}

	/* Drain stale prefix bytes if COUNT0 > 8 */
	if (count > 8) {
		uint16_t stale = count - 8;
		LOG_DBG("ep0: draining %u stale bytes before SETUP", stale);
		while (stale >= 4) {
			sys_read32(fifo);
			stale -= 4;
		}
		while (stale > 0) {
			sys_read8(fifo);
			stale--;
		}
	}

	/* Read 8-byte SETUP from FIFO */
	musb->ops->read_fifo(fifo, 8, setup_data);

	bmRequestType = setup_data[0];
	wLength = setup_data[6] | (setup_data[7] << 8);

	LOG_HEXDUMP_DBG(setup_data, 8, "ep0: SETUP");
	LOG_DBG("SETUP: %02x %02x %04x %04x len=%u dir=%s",
		bmRequestType, setup_data[1],
		setup_data[2] | (setup_data[3] << 8),
		setup_data[4] | (setup_data[5] << 8),
		wLength,
		(bmRequestType & 0x80) ? "IN" : "OUT");

	/* Clean up any leftover transfer from previous SETUP */
	musb->set_address = false;
	musb->ackpend = MUSB_CSR0_P_SVDRXPKTRDY;

	/*
	 * SET_ADDRESS: set the address flag so that the STATUSIN /
	 * SETUPEND handlers below can write FADDR immediately when the
	 * status phase completes — BEFORE the deferred work thread runs.
	 *
	 * Without this, the host sends the next SETUP to the new address
	 * before the work thread has updated FADDR, causing EPROTO (-71).
	 * The work thread's udc_musb_set_address() will also write FADDR
	 * (harmless redundant write) as a safety net.
	 */
	if (bmRequestType == 0x00 && setup_data[1] == 0x05) {
		musb->set_address = true;
		musb->address = setup_data[2];
	}

	if (wLength == 0) {
		/*
		 * Zero-data request (SET_ADDRESS, SET_CONFIGURATION, etc.)
		 * Linux: ACKWAIT state.  ackpend is written LATER in the
		 * SETUP case of musb_handle_ep0, after the adapter callback
		 * has had a chance to process the request and we add DATAEND.
		 */
		if (bmRequestType & 0x80) {
			/* IN direction (GET_STATUS etc.) */
			musb->ackpend |= MUSB_CSR0_TXPKTRDY;
		}
		musb->ep0_stage = MUSB_EP0_STAGE_ACKWAIT;
	} else if (bmRequestType & 0x80) {
		/*
		 * IN data transfer (GET_DESCRIPTOR etc.)
		 * Linux: go to TX state, immediately clear RXPKTRDY,
		 * then wait for RXPKTRDY to actually clear.
		 */
		musb->ep0_stage = MUSB_EP0_STAGE_TX;
		musb_ep_select(musb, 0);
		musb_ep_writew(musb, 0, MUSB_CSR0, MUSB_CSR0_P_SVDRXPKTRDY);
		/* Poll until RXPKTRDY is cleared */
		while (musb_ep_readw(musb, 0, MUSB_CSR0)
		       & MUSB_CSR0_RXPKTRDY) {
			k_busy_wait(1);
		}
		musb->ackpend = 0;
	} else {
		/*
		 * OUT data transfer
		 * Linux: go to RX state, wait for data
		 */
		musb->ep0_stage = MUSB_EP0_STAGE_RX;
	}

	/* Notify adapter layer */
	if (musb->callbacks->ep0_setup) {
		musb->callbacks->ep0_setup(musb, setup_data);
	}
}

/*
 * Handle EP0 interrupt.
 *
 *
 * Order:
 *   1. Read CSR0, COUNT0
 *   2. DATAEND set → return early
 *   3. SENTSTALL → clear, go IDLE
 *   4. SETUPEND → transition to early status
 *   5. switch(ep0_stage):
 *        TX:      !TXPKTRDY → ep0_txstate
 *        RX:      RXPKTRDY  → ep0_rxstate
 *        STATUSIN/STATUSOUT: complete → check coalesced SETUP
 *        IDLE:    → SETUP → fallthrough
 *        SETUP:   RXPKTRDY → read SETUP → dispatch (ep0_read_setup)
 *        ACKWAIT: nothing
 *   6. Write ackpend if pending
 */
static void musb_handle_ep0(struct musb_dev *musb)
{
	uint16_t csr;

	musb_ep_select(musb, 0);

	csr = musb_ep_readw(musb, 0, MUSB_CSR0);

	LOG_DBG("ep0: CSR0=0x%04x stage=%d", csr, musb->ep0_stage);

	/*
	 * Step 2: Handle DATAEND.
	 *
	 * On sunxi MUSB, DATAEND auto-clears with a lag — TXPKTRDY has
	 * already cleared (ZLP sent & ACK'd) but DATAEND still reads as
	 * set.  The Linux driver returns early and waits for a second
	 * interrupt where DATAEND has cleared, but on sunxi Zephyr that
	 * second interrupt often doesn't come until the next SETUP
	 * arrives — too late for SET_ADDRESS.
	 *
	 * Fix: poll CSR0 until DATAEND clears (up to 100 µs).  Once
	 * DATAEND clears, fall through to the state machine which will
	 * handle STATUSIN/STATUSOUT normally (write FADDR, call
	 * ep_tx_done, process coalesced SETUP).
	 *
	 * This replicates the timing that LOG_DBG calls naturally
	 * provide — without it, the ISR is too fast for the hardware.
	 */
	if (csr & MUSB_CSR0_P_DATAEND) {
		if (csr & MUSB_CSR0_P_SETUPEND) {
			/*
			 * DATAEND + SETUPEND: transfer aborted by new SETUP.
			 * Clear SETUPEND and transition state so the new
			 * SETUP can be processed by the state machine below.
			 */
			musb_ep_writew(musb, 0, MUSB_CSR0, MUSB_CSR0_P_SVDSETUPEND);
			musb->ackpend = 0;
			musb->ep0_stage = MUSB_EP0_STAGE_SETUP;
			csr = musb_ep_readw(musb, 0, MUSB_CSR0);
			LOG_DBG("ep0: DATAEND+SETUPEND, cleared SETUPEND, "
				"new CSR0=0x%04x", csr);
			/* Fall through to state machine to process SETUP */
		} else if (csr & MUSB_CSR0_RXPKTRDY) {
			/*
			 * DATAEND + RXPKTRDY, no SETUPEND:
			 * Previous transfer completed, new SETUP in FIFO.
			 * Fall through — state machine will process it.
			 */
		} else {
			/*
			 * DATAEND only (sunxi timing quirk).
			 * Poll until DATAEND auto-clears (up to 200 µs).
			 * TXPKTRDY has already cleared, meaning the ZLP
			 * was sent and ACK'd — the status phase IS done,
			 * DATAEND just hasn't reflected this yet.
			 */
			int timeout = 200;
			while ((csr & MUSB_CSR0_P_DATAEND) && timeout-- > 0) {
				k_busy_wait(1);
				csr = musb_ep_readw(musb, 0, MUSB_CSR0);
			}
			if (csr & MUSB_CSR0_P_DATAEND) {
				/* Timeout — give up, return early */
				return;
			}
			/* DATAEND cleared — fall through to state machine */
		}
	}

	/*
	 * Step 3: Handle sent stall.
	 * Linux musb_g_ep0_irq:665-671.
	 */
	if (csr & MUSB_CSR0_P_SENTSTALL) {
		musb_ep_writew(musb, 0, MUSB_CSR0, csr & ~MUSB_CSR0_P_SENTSTALL);
		musb->ep0_stage = MUSB_EP0_STAGE_IDLE;
		musb->ackpend = 0;
		csr = musb_ep_readw(musb, 0, MUSB_CSR0);
	}

	/*
	 * Step 4: Handle setup end (previous transfer aborted by new SETUP).
	 * Linux musb_g_ep0_irq:674-691.
	 *
	 * On sunxi MUSB, the status phase completion (DATAEND auto-clear)
	 * and the next SETUP arrival are often coalesced into a single
	 * interrupt with SETUPEND set but DATAEND already cleared.
	 * When stage is STATUSIN/STATUSOUT, the status phase actually
	 * completed — apply SET_ADDRESS and notify the adapter before
	 * processing the new SETUP.
	 */
	if (csr & MUSB_CSR0_P_SETUPEND) {
		musb_ep_writew(musb, 0, MUSB_CSR0, MUSB_CSR0_P_SVDSETUPEND);
		musb->ackpend = 0;

		/*
		 * If we were in STATUSIN/STATUSOUT, the status phase
		 * completed (DATAEND was auto-cleared by hardware before
		 * this SETUPEND).  Apply pending completion actions that
		 * the STATUSIN/STATUSOUT handler would have done.
		 */
		if (musb->ep0_stage == MUSB_EP0_STAGE_STATUSIN) {
			if (musb->set_address) {
				musb->set_address = false;
				musb_writeb(musb,
					    musb->ops->busctl_offset(0, MUSB_FADDR),
					    musb->address);
				LOG_DBG("SET_ADDRESS: addr=%u applied (SETUPEND)",
					musb->address);
			}
			/* fall through to ep_tx_done below */
		}
		if (musb->ep0_stage == MUSB_EP0_STAGE_STATUSIN ||
		    musb->ep0_stage == MUSB_EP0_STAGE_STATUSOUT) {
			if (musb->callbacks->ep_tx_done) {
				musb->callbacks->ep_tx_done(musb, 0);
			}
		}

		/* Transition to a state that will pick up the new SETUP. */
		switch (musb->ep0_stage) {
		case MUSB_EP0_STAGE_TX:
			musb->ep0_stage = MUSB_EP0_STAGE_STATUSOUT;
			break;
		case MUSB_EP0_STAGE_RX:
			musb->ep0_stage = MUSB_EP0_STAGE_STATUSIN;
			break;
		default:
			/*
			 * STATUSIN/STATUSOUT/IDLE/SETUP/ACKWAIT:
			 * go to IDLE, state machine below will
			 * pick up the new SETUP via RXPKTRDY.
			 */
			musb->ep0_stage = MUSB_EP0_STAGE_IDLE;
			break;
		}
		csr = musb_ep_readw(musb, 0, MUSB_CSR0);
	}

	/*
	 * Step 5: Main state machine switch.
	 * Linux musb_g_ep0_irq:697-883.
	 */
	switch (musb->ep0_stage) {

	case MUSB_EP0_STAGE_TX:
		/*
		 * TX complete interrupt (TXPKTRDY cleared by hardware).
		 *
		 * If musb_ep_tx() cached a multi-packet request and there
		 * is more data to send, ep0_txstate() writes the next chunk.
		 * If the first chunk hasn't been sent yet (musb_ep_tx hasn't
		 * been called from the thread), ep0_tx_sent == 0 and we
		 * skip — the hardware NAKs the host's IN tokens until
		 * TXPKTRDY is set by musb_ep_tx().
		 *
		 * Matches Linux musb_g_ep0_irq() TX handling.
		 */
		if (musb->ep0_tx_sent > 0 &&
		    musb->ep0_tx_sent < musb->ep0_tx_len) {
			ep0_txstate(musb);
		}
		break;

	case MUSB_EP0_STAGE_RX:
		/* RX: IRQ on setting RXPKTRDY */
		if (csr & MUSB_CSR0_RXPKTRDY) {
			LOG_DBG("ep0: RXPKTRDY set → rxstate");
			ep0_rxstate(musb);
		}
		break;

	case MUSB_EP0_STAGE_STATUSIN:
		/*
		 * End of sequence #2 (OUT/RX state) or #3 (no data).
		 * Apply SET_ADDRESS here (USB spec: after status phase).
		 */
		if (musb->set_address) {
			musb->set_address = false;
			musb_writeb(musb, musb->ops->busctl_offset(0, MUSB_FADDR),
				    musb->address);
			LOG_DBG("SET_ADDRESS: addr=%u applied", musb->address);
		}
		/* fallthrough */
	case MUSB_EP0_STAGE_STATUSOUT:
		/* End of sequence #1: write to host (TX state). */
		if (musb->callbacks->ep_tx_done) {
			musb->callbacks->ep_tx_done(musb, 0);
		}
		/*
		 * Check for coalesced SETUP packet.
		 * Linux: if (csr & MUSB_CSR0_RXPKTRDY) goto setup;
		 */
		if (csr & MUSB_CSR0_RXPKTRDY) {
			LOG_DBG("ep0: coalesced SETUP in status stage");
			musb->ep0_stage = MUSB_EP0_STAGE_SETUP;
			/* fall through to SETUP handling */
			goto setup;
		}
		musb->ep0_stage = MUSB_EP0_STAGE_IDLE;
		break;

	case MUSB_EP0_STAGE_IDLE:
		/*
		 * Linux: in the IDLE state, transition to SETUP and
		 * fallthrough to handle RXPKTRDY.
		 */
		musb->ep0_stage = MUSB_EP0_STAGE_SETUP;
		/* fallthrough */

	case MUSB_EP0_STAGE_SETUP:
	setup:
		if (csr & MUSB_CSR0_RXPKTRDY) {
			ep0_read_setup(musb);

			/*
			 * After the adapter callback, handle the
			  * zero-data ACKWAIT case inline (matching
			  * Linux's service_zero_data_request).
			  *
			  * For zero-data IN (GET_STATUS etc.):
			  *   ackpend already has TXPKTRDY
			  *   ackpend |= DATAEND, stage = STATUSOUT
			  *
			  * For zero-data OUT (SET_ADDRESS etc.):
			  *   ackpend |= TXPKTRDY | DATAEND, stage = STATUSIN
			  *   TXPKTRDY is needed to trigger the status-phase
			  *   ZLP on sunxi MUSB (writing DATAEND without
			  *   TXPKTRDY does NOT auto-send the ZLP).
			  */
			 if (musb->ep0_stage == MUSB_EP0_STAGE_ACKWAIT) {
			  musb->ackpend |= MUSB_CSR0_P_DATAEND;
			  if (musb->ackpend & MUSB_CSR0_TXPKTRDY) {
			 	 musb->ep0_stage = MUSB_EP0_STAGE_STATUSOUT;
			  } else {
			 	 musb->ackpend |= MUSB_CSR0_TXPKTRDY;
			 	 musb->ep0_stage = MUSB_EP0_STAGE_STATUSIN;
			  }
			 }

			/* Write accumulated ackpend */
			if (musb->ackpend) {
				musb_ep_select(musb, 0);
				musb_ep_writew(musb, 0, MUSB_CSR0, musb->ackpend);
				LOG_DBG("ACKWAIT: CSR0=0x%04x stage=%d set_addr=%d",
					musb->ackpend, musb->ep0_stage,
					musb->set_address);
				musb->ackpend = 0;
			}

		}
		break;

	case MUSB_EP0_STAGE_ACKWAIT:
		/*
		 * ACKWAIT should have been resolved in the SETUP case above.
		 * If we get here, it means the ISR fired for a different
		 * reason (e.g. SETUPEND which was handled above). No-op.
		 */
		break;

	default:
		LOG_ERR("Unknown ep0_stage %d, stalling", musb->ep0_stage);
		musb_ep_select(musb, 0);
		musb_ep_writew(musb, 0, MUSB_CSR0, MUSB_CSR0_P_SENDSTALL);
		musb->ep0_stage = MUSB_EP0_STAGE_IDLE;
		break;
	}
}

/*
 * Handle TX interrupt for non-EP0 endpoints.
 * Mirrors Linux musb_g_tx() — musb_gadget.c:407-511.
 *
 * For multi-packet transfers (tx_len > max_packet_sz_tx), this function
 * sends the next chunk from the cached request state and returns without
 * calling ep_tx_done.  When the final packet has been sent, ep_tx_done
 * is invoked so the adapter layer can dequeue the net_buf and start the
 * next queued transfer.
 */
static void musb_handle_tx(struct musb_dev *musb, uint8_t ep)
{
	uint16_t csr;
	struct musb_hw_ep *hw_ep = &musb->hw_eps[ep];

	musb_ep_select(musb, ep);
	csr = musb_ep_readw(musb, ep, MUSB_TXCSR);

	/* Handle sent stall */
	if (csr & MUSB_TXCSR_P_SENTSTALL) {
		csr &= ~MUSB_TXCSR_P_SENTSTALL;
		musb_ep_writew(musb, ep, MUSB_TXCSR, csr);
		LOG_DBG("tx EP%d: SENTSTALL cleared", ep);
		return;
	}

	/* Handle underrun */
	if (csr & MUSB_TXCSR_P_UNDERRUN) {
		csr &= ~(MUSB_TXCSR_P_UNDERRUN | MUSB_TXCSR_TXPKTRDY);
		musb_ep_writew(musb, ep, MUSB_TXCSR, csr);
		LOG_DBG("tx EP%d: UNDERRUN cleared", ep);
	}

	/* Multi-packet continuation: more data pending for current request? */
	if (hw_ep->tx_len > 0 && hw_ep->tx_sent < hw_ep->tx_len) {
		uint16_t remaining = hw_ep->tx_len - hw_ep->tx_sent;
		uint16_t chunk = remaining > hw_ep->max_packet_sz_tx
				 ? hw_ep->max_packet_sz_tx : remaining;
		mm_reg_t fifo = musb->base + musb_fifo_offset(musb, ep);

		musb->ops->write_fifo(fifo, chunk,
				      hw_ep->tx_buf + hw_ep->tx_sent);
		hw_ep->tx_sent += chunk;

		csr = musb_ep_readw(musb, ep, MUSB_TXCSR);
		csr |= MUSB_TXCSR_TXPKTRDY;
		csr &= ~MUSB_TXCSR_P_UNDERRUN;
		musb_ep_writew(musb, ep, MUSB_TXCSR, csr);

		LOG_DBG("tx EP%d: next chunk=%u sent=%u/%u",
			ep, chunk, hw_ep->tx_sent, hw_ep->tx_len);
		return;
	}

	/* All data sent — clear state and notify adapter */
	hw_ep->tx_buf = NULL;
	hw_ep->tx_len = 0;
	hw_ep->tx_sent = 0;

	if (musb->callbacks->ep_tx_done) {
		musb->callbacks->ep_tx_done(musb, ep);
	}
}

/*
 * Handle RX interrupt for non-EP0 endpoints.
 * Mirrors Linux musb_g_rx() — musb_gadget.c:791-900.
 */
static void musb_handle_rx(struct musb_dev *musb, uint8_t ep)
{
	uint16_t csr, count;

	musb_ep_select(musb, ep);
	csr = musb_ep_readw(musb, ep, MUSB_RXCSR);
	count = musb_ep_readw(musb, ep, MUSB_RXCOUNT);

	LOG_DBG("rx EP%d: RXCSR=0x%04x count=%u", ep, csr, count);

	/* Handle sent stall */
	if (csr & MUSB_RXCSR_P_SENTSTALL) {
		csr &= ~MUSB_RXCSR_P_SENTSTALL;
		musb_ep_writew(musb, ep, MUSB_RXCSR, csr);
		LOG_DBG("rx EP%d: SENTSTALL cleared", ep);
		return;
	}

	/* Handle overrun */
	if (csr & MUSB_RXCSR_OVERRUN) {
		csr &= ~MUSB_RXCSR_OVERRUN;
		musb_ep_writew(musb, ep, MUSB_RXCSR, csr);
		LOG_DBG("rx EP%d: OVERRUN cleared", ep);
	}

	/* Notify adapter: data is ready in FIFO */
	if (musb->callbacks->ep_rx_ready) {
		musb->callbacks->ep_rx_ready(musb, ep, count);
	}
}

/* ========== Main ISR ========== */

void musb_irq_handler(struct musb_dev *musb)
{
	int i;

	/* Read and clear interrupt status */
	musb->int_usb = musb_readb(musb, MUSB_INTRUSB);
	if (musb->int_usb) {
		musb_writeb(musb, MUSB_INTRUSB, musb->int_usb);
		LOG_DBG("ISR: int_usb=0x%02x POWER=0x%02x DEVCTL=0x%02x",
			musb->int_usb,
			musb_readb(musb, MUSB_POWER),
			musb_readb(musb, MUSB_DEVCTL));
	}

	musb->int_tx = musb_readw(musb, MUSB_INTRTX);
	if (musb->int_tx) {
		musb_writew(musb, MUSB_INTRTX, musb->int_tx);
		LOG_DBG("ISR: int_tx=0x%04x", musb->int_tx);
	}

	musb->int_rx = musb_readw(musb, MUSB_INTRRX);
	if (musb->int_rx) {
		musb_writew(musb, MUSB_INTRRX, musb->int_rx);
		LOG_DBG("ISR: int_rx=0x%04x", musb->int_rx);
	}

	if (!musb->int_usb && !musb->int_tx && !musb->int_rx) {
		/* Spurious IRQ — dump state for diagnosis */
		LOG_WRN("ISR: spurious IRQ, POWER=0x%02x DEVCTL=0x%02x "
			"INTRTXE=0x%04x INTRUSBE=0x%02x",
			musb_readb(musb, MUSB_POWER),
			musb_readb(musb, MUSB_DEVCTL),
			musb_readw(musb, MUSB_INTRTXE),
			musb_readb(musb, MUSB_INTRUSBE));
	}

	/* Dispatch: stage0 first, then EP0, then EP1+ */
	if (musb->int_usb) {
		musb_handle_stage0(musb);
	}

	if (musb->int_tx & BIT(0)) {
		musb_handle_ep0(musb);
	}

	for (i = 1; i < musb->config->num_eps; i++) {
		if (musb->int_tx & BIT(i)) {
			musb_handle_tx(musb, i);
		}
		if (musb->int_rx & BIT(i)) {
			musb_handle_rx(musb, i);
		}
	}
}

/* ========== Endpoint Operations ========== */

int musb_ep_enable(struct musb_dev *musb, uint8_t ep, uint8_t ep_type, uint16_t mps)
{
	const struct musb_glue_ops *ops = musb->ops;
	const struct musb_glue_config *cfg = musb->config;
	uint16_t int_txe, int_rxe, txcsr, rxcsr;
	int ret;

	musb_ep_select(musb, ep);

	/*
	 * EP0 has a fixed 64-byte FIFO and no TXMAXP/RXMAXP registers
	 * (verified on Allwinner sunxi R818).  Skip max-packet writes for
	 * EP0 to avoid corrupting CSR0.
	 * EP0 also has no TXTYPE/RXTYPE registers.
	 */
	if (ep > 0) {
		musb_ep_writew(musb, ep, MUSB_TXMAXP, mps);
		musb_ep_writew(musb, ep, MUSB_RXMAXP, mps);
		musb_ep_writeb(musb, ep, MUSB_TXTYPE, ep_type);
		musb_ep_writeb(musb, ep, MUSB_RXTYPE, ep_type);

		/*
		 * Flush FIFO and clear data toggle.
		 * Matches Linux musb_ep_enable() — flush pending data
		 * and reset the data toggle sequence for a clean start.
		 */
		txcsr = musb_ep_readw(musb, ep, MUSB_TXCSR);
		if (txcsr & MUSB_TXCSR_TXPKTRDY) {
			musb_ep_writew(musb, ep, MUSB_TXCSR,
				       MUSB_TXCSR_FLUSHFIFO);
		}
		txcsr = musb_ep_readw(musb, ep, MUSB_TXCSR);
		txcsr |= MUSB_TXCSR_CLRDATATOG;
		musb_ep_writew(musb, ep, MUSB_TXCSR, txcsr);

		rxcsr = musb_ep_readw(musb, ep, MUSB_RXCSR);
		if (rxcsr & MUSB_RXCSR_RXPKTRDY) {
			musb_ep_writew(musb, ep, MUSB_RXCSR,
				       MUSB_RXCSR_FLUSHFIFO);
		}
		rxcsr = musb_ep_readw(musb, ep, MUSB_RXCSR);
		rxcsr |= MUSB_RXCSR_CLRDATATOG;
		musb_ep_writew(musb, ep, MUSB_RXCSR, rxcsr);
	}

	/* Enable interrupts for this endpoint */
	int_txe = musb_readw(musb, MUSB_INTRTXE);
	int_rxe = musb_readw(musb, MUSB_INTRRXE);
	musb_writew(musb, MUSB_INTRTXE, int_txe | BIT(ep));
	if (ep > 0) {
		musb_writew(musb, MUSB_INTRRXE, int_rxe | BIT(ep));
	}

	/* --- DMA setup (optional, falls back to PIO if unavailable) --- */
	if (ops->dma_ep_prepare && musb->glue_dev && ep > 0) {
		/* TX DMA */
		if (cfg->dma_tx_drq[ep]) {
			txcsr = musb_ep_readw(musb, ep, MUSB_TXCSR);
			ret = ops->dma_ep_prepare(musb->glue_dev, ep, true, cfg->dma_tx_drq[ep],
						  mps);
			if (ret == 0) {
				txcsr |= MUSB_TXCSR_DMAMODE | MUSB_TXCSR_AUTOSET;
				musb->dma_tx_mask |= BIT(ep);
			}
			musb_ep_writew(musb, ep, MUSB_TXCSR, txcsr);
		}
		/* RX DMA */
		if (cfg->dma_rx_drq[ep]) {
			rxcsr = musb_ep_readw(musb, ep, MUSB_RXCSR);
			ret = ops->dma_ep_prepare(musb->glue_dev, ep, false, cfg->dma_rx_drq[ep],
						  mps);
			if (ret == 0) {
				rxcsr |= MUSB_RXCSR_DMAENAB | MUSB_RXCSR_AUTOCLEAR;
				musb->dma_rx_mask |= BIT(ep);
			}
			musb_ep_writew(musb, ep, MUSB_RXCSR, rxcsr);
		}
	}

	return 0;
}

int musb_ep_disable(struct musb_dev *musb, uint8_t ep)
{
	const struct musb_glue_ops *ops = musb->ops;
	uint16_t int_txe, int_rxe, csr;

	musb_ep_select(musb, ep);

	/* Release DMA channels if active */
	if (ops->dma_ep_unprepare && musb->glue_dev) {
		if (musb->dma_tx_mask & BIT(ep)) {
			csr = musb_ep_readw(musb, ep, MUSB_TXCSR);
			csr &= ~(MUSB_TXCSR_DMAMODE | MUSB_TXCSR_AUTOSET);
			musb_ep_writew(musb, ep, MUSB_TXCSR, csr);
			ops->dma_ep_unprepare(musb->glue_dev, ep, true);
			musb->dma_tx_mask &= ~BIT(ep);
		}
		if (musb->dma_rx_mask & BIT(ep)) {
			csr = musb_ep_readw(musb, ep, MUSB_RXCSR);
			csr &= ~(MUSB_RXCSR_DMAENAB | MUSB_RXCSR_AUTOCLEAR);
			musb_ep_writew(musb, ep, MUSB_RXCSR, csr);
			ops->dma_ep_unprepare(musb->glue_dev, ep, false);
			musb->dma_rx_mask &= ~BIT(ep);
		}
	}

	/* Disable interrupts for this endpoint */
	int_txe = musb_readw(musb, MUSB_INTRTXE);
	int_rxe = musb_readw(musb, MUSB_INTRRXE);
	musb_writew(musb, MUSB_INTRTXE, int_txe & ~BIT(ep));
	musb_writew(musb, MUSB_INTRRXE, int_rxe & ~BIT(ep));

	/* Flush FIFO (TXCSR/RXCSR are 16-bit registers — must use writew) */
	musb_ep_writew(musb, ep, MUSB_TXCSR, MUSB_TXCSR_FLUSHFIFO);
	musb_ep_writew(musb, ep, MUSB_RXCSR, MUSB_RXCSR_FLUSHFIFO);

	return 0;
}

int musb_ep_tx(struct musb_dev *musb, uint8_t ep, const uint8_t *data, uint16_t len)
{
	musb_ep_select(musb, ep);

	/* DMA path: program transfer, hardware handles TXPKTRDY via AUTOSET */
	if ((musb->dma_tx_mask & BIT(ep)) && musb->ops->dma_ep_transfer) {
		return musb->ops->dma_ep_transfer(musb->glue_dev, ep, true, (uint8_t *)data, len,
						  NULL, NULL);
	}

	if (ep == 0) {
		if (len == 0) {
			/*
			 * Zero-length EP0 transfer: the ACKWAIT handler in
			 * musb_handle_ep0() already wrote SVDRXPKTRDY|DATAEND
			 * (or SVDRXPKTRDY|TXPKTRDY|DATAEND) to CSR0 for this
			 * request.  Do NOT write CSR0 again — a second
			 * TXPKTRDY|DATAEND would corrupt the status phase
			 * handshake.
			 *
			 * In Linux this doesn't arise because
			 * service_zero_data_request() handles everything
			 * synchronously in the ISR, but Zephyr's async USB
			 * stack always calls ep_enqueue for EP0 even for
			 * zero-data requests.
			 */
			LOG_DBG("ep_tx EP0: zero-len, ACKWAIT handled "
				"(stage=%d)", musb->ep0_stage);
			return 0;
		}

		/*
		 * EP0 multi-packet TX — cache the full request and send
		 * only the first chunk.  Subsequent chunks are sent by
		 * ep0_txstate() from the ISR on TX complete interrupts.
		 * Matches Linux ep0_txstate() / musb_gadget_ep0.c.
		 *
		 * IMPORTANT: update stage BEFORE writing CSR0.
		 * Writing TXPKTRDY can trigger an immediate hardware
		 * interrupt.  If the ISR preempts us before the stage
		 * update, it sees the wrong stage and may skip
		 * completion actions.
		 */
		musb->ep0_tx_buf = data;
		musb->ep0_tx_len = len;
		musb->ep0_tx_sent = 0;

		uint16_t chunk = len > MUSB_EP0_MAX_PACKET
				 ? MUSB_EP0_MAX_PACKET : len;
		mm_reg_t fifo = musb->base + musb_fifo_offset(musb, 0);

		musb->ops->write_fifo(fifo, chunk, data);
		musb->ep0_tx_sent = chunk;

		uint16_t csr = MUSB_CSR0_TXPKTRDY;

		/*
		 * DATAEND is set only when this is the last (or only)
		 * packet.  Linux: fifo_count < MUSB_MAX_END0_PACKET
		 * || (request->actual == request->length && !zero).
		 */
		if (musb->ep0_tx_sent == musb->ep0_tx_len) {
			csr |= MUSB_CSR0_P_DATAEND;
			musb->ep0_stage = MUSB_EP0_STAGE_STATUSOUT;
		}
		/* else: stay in EP0_STAGE_TX for ISR-driven continuation */

		musb_ep_select(musb, 0);
		musb_ep_writew(musb, 0, MUSB_CSR0, csr);
		/* Read-back to ensure the write has taken effect */
		(void)musb_ep_readw(musb, 0, MUSB_CSR0);
		LOG_DBG("ep_tx EP0: len=%u chunk=%u stage=%d csr=0x%04x",
			len, chunk, musb->ep0_stage, csr);
	} else {
		/*
		 * EP1+: write data to FIFO and set TXPKTRDY.
		 *
		 * Mirrors Linux txstate() — musb_gadget.c:378-391:
		 *   musb_write_fifo(hw_ep, fifo_count, buf + actual);
		 *   csr |= TXPKTRDY;
		 *
		 * The FIFO write is mandatory — the previous code only set
		 * TXPKTRDY without ever writing data, so the host always
		 * received an empty/zero-length packet.
		 *
		 * For multi-packet transfers (len > max_packet_sz_tx), we
		 * cache the request state and send the first chunk here.
		 * The ISR (musb_handle_tx) sends subsequent chunks.
		 *
		 * CRITICAL: musb_ep_select() through TXCSR TXPKTRDY write
		 * must be atomic w.r.t. the ISR.  On indexed-EP hardware
		 * (MUSB_INDEXED_EP quirk), all EP CSR registers share the
		 * same address and the INDEX register selects the target EP.
		 * If the ISR preempts us between musb_ep_select() and the
		 * TXCSR write, it changes INDEX to service a different EP,
		 * causing our TXPKTRDY write to go to the wrong endpoint.
		 */
		struct musb_hw_ep *hw_ep = &musb->hw_eps[ep];
		int key = irq_lock();

		musb_ep_select(musb, ep);

		uint16_t csr = musb_ep_readw(musb, ep, MUSB_TXCSR);

		if (csr & MUSB_TXCSR_TXPKTRDY) {
			/*
			 * Previous transfer still in flight.
			 * The net_buf is already queued by udc_buf_put().
			 * musb_handle_tx() → ep_tx_done() will dequeue it
			 * and start the next transfer.
			 */
			irq_unlock(key);
			LOG_INF("ep_tx EP%d: TXPKTRDY set, queuing", ep);
			return 0;
		}

		uint16_t chunk = len > hw_ep->max_packet_sz_tx
				 ? hw_ep->max_packet_sz_tx : len;
		mm_reg_t fifo = musb->base + musb_fifo_offset(musb, ep);

		/* Cache multi-packet TX state (ISR reads progress from here) */
		hw_ep->tx_buf = data;
		hw_ep->tx_len = len;
		hw_ep->tx_sent = 0;

		musb->ops->write_fifo(fifo, chunk, data);
		hw_ep->tx_sent = chunk;

		csr = musb_ep_readw(musb, ep, MUSB_TXCSR);
		csr |= MUSB_TXCSR_TXPKTRDY;
		csr &= ~MUSB_TXCSR_P_UNDERRUN;
		musb_ep_writew(musb, ep, MUSB_TXCSR, csr);

		irq_unlock(key);

		LOG_DBG("ep_tx EP%d: len=%u chunk=%u TXCSR=0x%04x",
			ep, len, chunk, csr);
	}

	return 0;
}

int musb_ep_rx(struct musb_dev *musb, uint8_t ep)
{
	/*
	 * EP0 has no RXCSR and SETUP reception is always active on the
	 * control endpoint.  The UDC layer calls ep_enqueue for EP0 OUT
	 * to provide a buffer for the next SETUP packet — this is an
	 * arming gesture on the software side only.  The actual SETUP
	 * reception is handled entirely by the ISR via ep0_read_setup().
	 * Return success so the UDC stack can proceed with initialization.
	 */
	if (ep == 0) {
		return 0;
	}

	/*
	 * CRITICAL: protect INDEX-dependent register access from ISR
	 * preemption.  Same rationale as musb_ep_tx() — on indexed-EP
	 * hardware the ISR can clobber INDEX between our musb_ep_select()
	 * and the RXCSR write.
	 */
	int key = irq_lock();

	musb_ep_select(musb, ep);

	/* DMA path: AUTOCLEAR handles RXPKTRDY clearing after DMA done */
	if ((musb->dma_rx_mask & BIT(ep)) && musb->ops->dma_ep_transfer) {
		irq_unlock(key);
		return 0; /* buffer programmed by caller via dma_ep_transfer */
	}

	/* PIO path: clear RXPKTRDY to re-arm the endpoint */
	uint16_t csr = musb_ep_readw(musb, ep, MUSB_RXCSR);

	csr &= ~MUSB_RXCSR_RXPKTRDY;
	musb_ep_writew(musb, ep, MUSB_RXCSR, csr);

	irq_unlock(key);

	LOG_DBG("ep_rx EP%d: RXCSR=0x%04x (re-armed)", ep, csr);
	return 0;
}

int musb_ep_set_halt(struct musb_dev *musb, uint8_t ep, bool stall)
{
	uint16_t csr;

	musb_ep_select(musb, ep);

	if (ep == 0) {
		/*
		 * EP0 uses CSR0, not TXCSR.  The bit layout differs:
		 *   CSR0 SENDSTALL = BIT(5)  (MUSB_CSR0_P_SENDSTALL)
		 *   TXCSR SENDSTALL = BIT(4) (MUSB_TXCSR_P_SENDSTALL)
		 * The previous code used MUSB_TXCSR_P_SENDSTALL (BIT(4))
		 * which writes to CSR0 SETUPEND (read-only status bit),
		 * so the STALL was never actually sent to the host.
		 */
		csr = musb_ep_readw(musb, 0, MUSB_CSR0);

		if (stall) {
			csr |= MUSB_CSR0_P_SENDSTALL;
		} else {
			csr &= ~MUSB_CSR0_P_SENDSTALL;
		}

		musb_ep_writew(musb, 0, MUSB_CSR0, csr);
		LOG_DBG("EP0 STALL: csr0=0x%04x stall=%d", csr, stall);
		return 0;
	}

	/* TX direction (EP1+) */
	csr = musb_ep_readw(musb, ep, MUSB_TXCSR);

	if (stall) {
		csr |= MUSB_TXCSR_P_SENDSTALL;
	} else {
		csr &= ~MUSB_TXCSR_P_SENDSTALL;
		csr |= MUSB_TXCSR_CLRDATATOG;
	}

	musb_ep_writew(musb, ep, MUSB_TXCSR, csr);

	/* RX direction */
	csr = musb_ep_readw(musb, ep, MUSB_RXCSR);

	if (stall) {
		csr |= MUSB_RXCSR_P_SENDSTALL;
	} else {
		csr &= ~MUSB_RXCSR_P_SENDSTALL;
		csr |= MUSB_RXCSR_CLRDATATOG;
	}

	musb_ep_writew(musb, ep, MUSB_RXCSR, csr);

	return 0;
}

void musb_set_address(struct musb_dev *musb, uint8_t addr)
{
	uint32_t faddr_off = musb->ops->busctl_offset(0, MUSB_FADDR);

	musb_writeb(musb, faddr_off, addr);
}

void musb_connect(struct musb_dev *musb)
{
	uint8_t power, devctl;

	power = musb_readb(musb, MUSB_POWER);
	/*
	 * MUSB_POWER_SOFTCONN pulls D+ up (connect to host).
	 * HSEN is only set when PHY supports HS; FS-only PHYs (sunxi
	 * V3s) must skip HS chirp negotiation to avoid protocol errors.
	 */
	power |= MUSB_POWER_SOFTCONN;
	if (!(musb->config->quirks & MUSB_DISABLE_HS)) {
		power |= MUSB_POWER_HSENAB;
	}
	musb_writeb(musb, MUSB_POWER, power);

	/*
	 * Ensure a session is active for peripheral mode.
	 * The hardware should auto-set Session when VBUS is detected, but
	 * on some PHYs (e.g. Allwinner with forced VBUS) this may not
	 * propagate.  Explicitly set Session if we are a B-device.
	 */
	devctl = musb_readb(musb, MUSB_DEVCTL);
	if ((devctl & MUSB_DEVCTL_BDEVICE) && !(devctl & MUSB_DEVCTL_SESSION)) {
		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb, MUSB_DEVCTL, devctl);
	}

	LOG_DBG("connect: POWER=0x%02x DEVCTL=0x%02x", musb_readb(musb, MUSB_POWER),
		musb_readb(musb, MUSB_DEVCTL));
}

void musb_disconnect(struct musb_dev *musb)
{
	uint8_t power;

	power = musb_readb(musb, MUSB_POWER);
	power &= ~MUSB_POWER_SOFTCONN;
	musb_writeb(musb, MUSB_POWER, power);
}
