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
		uint8_t power;
		uint8_t faddr_back;

		musb_ep_select(musb, 0);

		/* Linux sunxi.c:188 — explicitly write FADDR=0 on RESET */
		musb_set_address(musb, 0);

		musb->set_address = false;
		musb->address = 0;
		musb->ackpend = 0;
		musb->ep0_stage = MUSB_EP0_STAGE_SETUP;

		/* Re-enable EP0 interrupts (cleared by bus reset) */
		musb_writew(musb, MUSB_INTRTXE,
			    musb_readw(musb, MUSB_INTRTXE) | BIT(0));

		faddr_back = musb_readb(musb,
					musb->ops->busctl_offset(0, MUSB_FADDR));
		power = musb_readb(musb, MUSB_POWER);
		LOG_INF("RESET: POWER=0x%02x (%s%s) DEVCTL=0x%02x FADDR=0x%02x",
			power,
			(power & MUSB_POWER_HSMODE) ? "HS" : "FS",
			(power & MUSB_POWER_RESET) ? " RESET" : "",
			musb_readb(musb, MUSB_DEVCTL),
			faddr_back);

		if (faddr_back != 0) {
			LOG_WRN("RESET: FADDR read-back=0x%02x (expected 0x00!)",
				faddr_back);
		}

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

static void ep0_rxstate(struct musb_dev *musb)
{
	uint16_t count;

	musb_ep_select(musb, 0);
	count = musb_ep_readw(musb, 0, MUSB_COUNT0);
	musb->ackpend = MUSB_CSR0_P_SVDRXPKTRDY;

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

static void ep0_read_setup(struct musb_dev *musb)
{
	mm_reg_t fifo = musb->base + musb_fifo_offset(musb, 0);
	uint8_t setup_data[8];
	uint8_t bmRequestType;
	uint16_t wLength;
	uint16_t count;
	int retry;

	count = musb_ep_readw(musb, 0, MUSB_COUNT0);
	for (retry = 0; retry < 16 && count < 8; retry++) {
		count = musb_ep_readw(musb, 0, MUSB_COUNT0);
	}

	if (count < 8) {
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

	if (bmRequestType == 0x00 && setup_data[1] == 0x05) {
		musb->set_address = true;
		musb->address = setup_data[2];
	}

	if (wLength == 0) {
		if (bmRequestType & 0x80) {
			/* IN direction (GET_STATUS etc.) */
			musb->ackpend |= MUSB_CSR0_TXPKTRDY;
		}
		musb->ep0_stage = MUSB_EP0_STAGE_ACKWAIT;
	} else if (bmRequestType & 0x80) {
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

static void musb_handle_ep0(struct musb_dev *musb)
{
	uint16_t csr;

	musb_ep_select(musb, 0);

	csr = musb_ep_readw(musb, 0, MUSB_CSR0);

	LOG_DBG("ep0: CSR0=0x%04x stage=%d", csr, musb->ep0_stage);

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
			 * DATAEND only (sunxi timing quirk):
			 * TXPKTRDY has already cleared, meaning the data
			 * (or status ZLP) was sent and ACK'd by the host.
			 * The transfer is functionally complete; the DATAEND
			 * bit just lingers as a stale status on sunxi MUSB.
			 *
			 * Do NOT poll for DATAEND to clear.  In full-speed
			 * mode the host's status-phase OUT can arrive up to
			 * one 1 ms frame later, far exceeding any reasonable
			 * poll budget.  Polling with a short timeout (the
			 * previous 200 µs) caused the ISR to return early
			 * WITHOUT completing the transfer — ep_tx_done was
			 * never called and SET_ADDRESS was never applied —
			 * leaving the UDC stack believing the transfer was
			 * still in flight, surfacing as -EPIPE (-32) STALL.
			 *
			 * Instead, fall straight through to the state machine,
			 * which processes the completion (STATUSIN → apply
			 * FADDR, STATUSOUT → ep_tx_done) based on the fact
			 * that TXPKTRDY has cleared.  DATAEND will be cleared
			 * by hardware when the status-phase OUT is received;
			 * we simply don't wait for it.
			 */
		}
	}

	if (csr & MUSB_CSR0_P_SENTSTALL) {
		LOG_WRN("ep0: SENTSTALL! CSR0=0x%04x stage=%d — host got STALL",
			csr, musb->ep0_stage);
		musb_ep_writew(musb, 0, MUSB_CSR0, csr & ~MUSB_CSR0_P_SENTSTALL);
		musb->ep0_stage = MUSB_EP0_STAGE_IDLE;
		musb->ackpend = 0;
		csr = musb_ep_readw(musb, 0, MUSB_CSR0);
	}


	if (csr & MUSB_CSR0_P_SETUPEND) {
		musb_ep_writew(musb, 0, MUSB_CSR0, MUSB_CSR0_P_SVDSETUPEND);
		musb->ackpend = 0;

		if (musb->ep0_stage == MUSB_EP0_STAGE_STATUSIN) {
			if (musb->set_address) {
				uint8_t faddr_rb;
				musb->set_address = false;
				musb_ep_select(musb, 0);
				musb_writeb(musb,
					    musb->ops->busctl_offset(0, MUSB_FADDR),
					    musb->address);
				faddr_rb = musb_readb(musb,
					musb->ops->busctl_offset(0, MUSB_FADDR));
				LOG_INF("SET_ADDRESS: addr=%u applied (SETUPEND) readback=0x%02x",
					musb->address, faddr_rb);
				if (faddr_rb != musb->address) {
					LOG_WRN("FADDR mismatch: wrote=0x%02x read=0x%02x",
						musb->address, faddr_rb);
				}
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

	switch (musb->ep0_stage) {

	case MUSB_EP0_STAGE_TX:
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
			uint8_t faddr_rb;
			musb->set_address = false;
			musb_ep_select(musb, 0);
			musb_writeb(musb, musb->ops->busctl_offset(0, MUSB_FADDR),
				    musb->address);
			faddr_rb = musb_readb(musb,
				musb->ops->busctl_offset(0, MUSB_FADDR));
			LOG_INF("SET_ADDRESS: addr=%u applied readback=0x%02x",
				musb->address, faddr_rb);
			if (faddr_rb != musb->address) {
				LOG_WRN("FADDR mismatch: wrote=0x%02x read=0x%02x",
					musb->address, faddr_rb);
			}
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
		LOG_DBG("EP0 irq: FADDR=0x%02x POWER=0x%02x INDEX=%u stage=%d",
			musb_readb(musb, musb->ops->busctl_offset(0, MUSB_FADDR)),
			musb_readb(musb, MUSB_POWER),
			musb_readb(musb, MUSB_INDEX),
			musb->ep0_stage);
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

int musb_ep_enable(struct musb_dev *musb, uint8_t ep, uint8_t ep_type, uint16_t mps)
{
	const struct musb_glue_ops *ops = musb->ops;
	const struct musb_glue_config *cfg = musb->config;
	uint16_t int_txe, int_rxe, txcsr, rxcsr;
	int ret;

	musb_ep_select(musb, ep);

	if (ep > 0) {
		musb_ep_writew(musb, ep, MUSB_TXMAXP, mps);
		musb_ep_writew(musb, ep, MUSB_RXMAXP, mps);
		musb_ep_writeb(musb, ep, MUSB_TXTYPE, ep_type);
		musb_ep_writeb(musb, ep, MUSB_RXTYPE, ep_type);

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
			LOG_DBG("ep_tx EP0: zero-len, ACKWAIT handled "
				"(stage=%d)", musb->ep0_stage);
			return 0;
		}

		musb->ep0_tx_buf = data;
		musb->ep0_tx_len = len;
		musb->ep0_tx_sent = 0;

		uint16_t chunk = len > MUSB_EP0_MAX_PACKET
				 ? MUSB_EP0_MAX_PACKET : len;
		mm_reg_t fifo = musb->base + musb_fifo_offset(musb, 0);

		musb->ops->write_fifo(fifo, chunk, data);
		musb->ep0_tx_sent = chunk;

		uint16_t csr = MUSB_CSR0_TXPKTRDY;

		if (musb->ep0_tx_sent == musb->ep0_tx_len) {
			csr |= MUSB_CSR0_P_DATAEND;
			musb->ep0_stage = MUSB_EP0_STAGE_STATUSOUT;
		}

		musb_ep_select(musb, 0);
		musb_ep_writew(musb, 0, MUSB_CSR0, csr);
		/* Read-back to ensure the write has taken effect */
		(void)musb_ep_readw(musb, 0, MUSB_CSR0);
		LOG_DBG("ep_tx EP0: len=%u chunk=%u stage=%d csr=0x%04x",
			len, chunk, musb->ep0_stage, csr);
	} else {
		struct musb_hw_ep *hw_ep = &musb->hw_eps[ep];
		int key = irq_lock();

		musb_ep_select(musb, ep);

		uint16_t csr = musb_ep_readw(musb, ep, MUSB_TXCSR);

		if (csr & MUSB_TXCSR_TXPKTRDY) {
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
	if (ep == 0) {
		return 0;
	}

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
	uint8_t faddr_rb;

	musb_writeb(musb, faddr_off, addr);

	/* Diagnostic: verify the write actually took effect */
	faddr_rb = musb_readb(musb, faddr_off);
	LOG_DBG("set_address: wrote=0x%02x readback=0x%02x", addr, faddr_rb);
	if (faddr_rb != addr) {
		LOG_WRN("FADDR write FAILED: wrote=0x%02x read=0x%02x",
			addr, faddr_rb);
	}
}

void musb_connect(struct musb_dev *musb)
{
	uint8_t power, devctl;

	power = musb_readb(musb, MUSB_POWER);

	power |= MUSB_POWER_SOFTCONN;
	if (!(musb->config->quirks & MUSB_DISABLE_HS)) {
		power |= MUSB_POWER_HSENAB;
	}
	musb_writeb(musb, MUSB_POWER, power);

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
