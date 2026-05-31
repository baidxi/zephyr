/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mentor Graphics MUSB HDRC register definitions.
 * Common across all MUSB-compatible controllers.
 *
 * Based on Linux drivers/usb/musb/musb_regs.h
 */

#ifndef ZEPHYR_DRIVERS_USB_COMMON_MUSB_MUSB_REGS_H_
#define ZEPHYR_DRIVERS_USB_COMMON_MUSB_MUSB_REGS_H_

#include <zephyr/sys/util.h>

/* ========== Standard MUSB Register Offsets ========== */

#define MUSB_POWER              0x00
#define MUSB_INTRTX             0x02
#define MUSB_INTRRX             0x04
#define MUSB_INTRTXE            0x06
#define MUSB_INTRRXE            0x08
#define MUSB_INTRUSB            0x0a
#define MUSB_INTRUSBE           0x0b
#define MUSB_FRAME              0x0c
#define MUSB_INDEX              0x0e
#define MUSB_TESTMODE           0x0f
#define MUSB_CONFIGDATA         0x1f

/* ========== EP CSR Offsets (relative to ep_offset() base) ========== */

#define MUSB_TXMAXP             0x00
#define MUSB_TXCSR              0x02
#define MUSB_RXMAXP             0x04
#define MUSB_RXCSR              0x06
#define MUSB_RXCOUNT            0x08
#define MUSB_TXTYPE             0x0a
#define MUSB_TXINTERVAL         0x0b
#define MUSB_RXTYPE             0x0c
#define MUSB_RXINTERVAL         0x0d
#define MUSB_TXFIFOSZ           0x62
#define MUSB_TXFIFOADD          0x64
#define MUSB_RXFIFOSZ           0x63
#define MUSB_RXFIFOADD          0x66

/* EP0 CSR offset */
#define MUSB_CSR0               0x02
#define MUSB_COUNT0             0x08

/* Shared offsets */
#define MUSB_DEVCTL             0x60
#define MUSB_FADDR              0x00   /* Relative to 0x78 offset base */

/* ========== POWER Register Bits ========== */

#define MUSB_POWER_ISOUPDATE    BIT(7)
#define MUSB_POWER_SOFTCONN     BIT(6)
#define MUSB_POWER_HSENAB       BIT(5)
#define MUSB_POWER_HSMODE       BIT(4)
#define MUSB_POWER_RESET        BIT(3)
#define MUSB_POWER_RESUME       BIT(2)
#define MUSB_POWER_SUSPENDM     BIT(1)
#define MUSB_POWER_ENABLESUSP   BIT(0)

/* ========== INTRUSB Register Bits ========== */

#define MUSB_INTR_VBUSERROR     BIT(7)
#define MUSB_INTR_SESSREQ       BIT(6)
#define MUSB_INTR_DISCONNECT    BIT(5)
#define MUSB_INTR_CONNECT       BIT(4)
#define MUSB_INTR_SOF           BIT(3)
#define MUSB_INTR_RESET         BIT(2)
#define MUSB_INTR_BABBLE        BIT(1)
#define MUSB_INTR_SUSPEND       BIT(0)

/* ========== CSR0 Register Bits ==========
 *
 * Sunxi CSR0 is a 32-bit register at offset 0x80.  Peripheral-mode
 * control bits sit at hardware bits 16-24.  A 16-bit read from offset
 * 0x82 returns bits 31:16, so the standard MUSB CSR0 bit layout
 * (matching Linux musb_regs.h) is directly applicable:
 *
 *   HW bit 16 -> 16-bit BIT(0) -> RxPktRdy
 *   HW bit 17 -> 16-bit BIT(1) -> TxPktRdy
 *   HW bit 18 -> 16-bit BIT(2) -> SentStall
 *   HW bit 19 -> 16-bit BIT(3) -> DataEnd
 *   HW bit 20 -> 16-bit BIT(4) -> SetupEnd
 *   HW bit 21 -> 16-bit BIT(5) -> SendStall
 *   HW bit 22 -> 16-bit BIT(6) -> ServicedRxPktRdy
 *   HW bit 23 -> 16-bit BIT(7) -> ServicedSetupEnd
 *   HW bit 24 -> 16-bit BIT(8) -> FlushFIFO
 *
 * Verified against R818 USB OTG register manual (USB_CSR0_P).
 * Also matches Linux drivers/usb/musb/musb_regs.h.
 */

#define MUSB_CSR0_FLUSHFIFO      BIT(8)
#define MUSB_CSR0_TXPKTRDY       BIT(1)
#define MUSB_CSR0_RXPKTRDY       BIT(0)
#define MUSB_CSR0_P_SENTSTALL    BIT(2)
#define MUSB_CSR0_P_DATAEND      BIT(3)
#define MUSB_CSR0_P_SETUPEND     BIT(4)
#define MUSB_CSR0_P_SENDSTALL    BIT(5)
#define MUSB_CSR0_P_SVDRXPKTRDY  BIT(6)
#define MUSB_CSR0_P_SVDSETUPEND  BIT(7)

#define MUSB_CSR0_H_SETUPPEND    BIT(8)
#define MUSB_CSR0_H_RXPKTRDY     BIT(0)
#define MUSB_CSR0_H_TXPKTRDY     BIT(1)
#define MUSB_CSR0_H_ERROR        BIT(2)
#define MUSB_CSR0_H_NAKTIMEOUT   BIT(3)
#define MUSB_CSR0_H_STATUSPEND   BIT(4)
#define MUSB_CSR0_H_REQPKT       BIT(5)
#define MUSB_CSR0_H_DIS_PING     BIT(7)

/* ========== TXCSR Register Bits ========== */

#define MUSB_TXCSR_TXPKTRDY     BIT(0)
#define MUSB_TXCSR_FIFONOTEMPTY BIT(1)
#define MUSB_TXCSR_FLUSHFIFO    BIT(3)
#define MUSB_TXCSR_CLRDATATOG   BIT(6)
#define MUSB_TXCSR_P_UNDERRUN   BIT(2)
#define MUSB_TXCSR_P_SENDSTALL  BIT(4)
#define MUSB_TXCSR_P_SENTSTALL  BIT(5)
#define MUSB_TXCSR_DMAMODE      BIT(11)
#define MUSB_TXCSR_MODE         BIT(13)
#define MUSB_TXCSR_AUTOSET      BIT(15)

/* ========== RXCSR Register Bits ========== */

#define MUSB_RXCSR_RXPKTRDY     BIT(0)
#define MUSB_RXCSR_FIFOFULL     BIT(1)
#define MUSB_RXCSR_OVERRUN      BIT(2)
#define MUSB_RXCSR_DATAERROR    BIT(3)
#define MUSB_RXCSR_FLUSHFIFO    BIT(4)
#define MUSB_RXCSR_P_SENDSTALL  BIT(5)
#define MUSB_RXCSR_P_SENTSTALL  BIT(6)
#define MUSB_RXCSR_CLRDATATOG   BIT(7)
#define MUSB_RXCSR_DMAENAB      BIT(13)
#define MUSB_RXCSR_AUTOCLEAR    BIT(15)

/* ========== DEVCTL Register Bits ========== */

#define MUSB_DEVCTL_BDEVICE     BIT(7)
#define MUSB_DEVCTL_FSDEV       BIT(6)
#define MUSB_DEVCTL_HSDEV       BIT(5)
#define MUSB_DEVCTL_VBUS        GENMASK(4, 3)
#define MUSB_DEVCTL_VBUS_NONE   (0 << 3)
#define MUSB_DEVCTL_VBUS_BELOW  (1 << 3)
#define MUSB_DEVCTL_VBUS_ABOVE  (2 << 3)
#define MUSB_DEVCTL_SESSION     BIT(0)

/* ========== TXTYPE / RXTYPE Bits ========== */

#define MUSB_TYPE_PROTO         GENMASK(5, 4)
#define MUSB_TYPE_PROTO_CTRL    (0 << 4)
#define MUSB_TYPE_PROTO_BULK    (2 << 4)
#define MUSB_TYPE_PROTO_INTR    (3 << 4)
#define MUSB_TYPE_SPEED         GENMASK(7, 6)
#define MUSB_TYPE_SPEED_FS      (1 << 6)
#define MUSB_TYPE_SPEED_HS      (2 << 6)

#endif /* ZEPHYR_DRIVERS_USB_COMMON_MUSB_MUSB_REGS_H_ */
