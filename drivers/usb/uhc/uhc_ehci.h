/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * EHCI (Enhanced Host Controller Interface) register definitions
 * and data structures.
 *
 * References:
 *   - EHCI Specification Rev 1.0
 *   - T113-S3 USB HCD manual (section 9.6)
 *   - Linux drivers/usb/host/ehci.h
 */

#ifndef ZEPHYR_DRIVERS_USB_UHC_UHC_EHCI_H_
#define ZEPHYR_DRIVERS_USB_UHC_UHC_EHCI_H_

#include <zephyr/types.h>
#include <zephyr/sys/util.h>

/* CAPLENGTH (0x00) - Capability Register Length */
#define EHCI_CAPLENGTH		0x00

/* HCIVERSION (0x02) - Host Interface Version Number (BCD) */
#define EHCI_HCIVERSION		0x02

/* HCSPARAMS (0x04) - Host Control Structural Parameters */
#define EHCI_HCSPARAMS		0x04
#define HCSPARAMS_N_PORTS_MASK		GENMASK(3, 0)
#define HCSPARAMS_N_PORTS(x)		((x) & HCSPARAMS_N_PORTS_MASK)
#define HCSPARAMS_PORT_ROUTE_RULES	BIT(7)
#define HCSPARAMS_N_PCC_MASK		GENMASK(11, 8)
#define HCSPARAMS_N_CC_MASK		GENMASK(15, 12)

/* HCCPARAMS (0x08) - Host Control Capability Parameters */
#define EHCI_HCCPARAMS		0x08
#define HCCPARAMS_64BIT_ADDR		BIT(0)
#define HCCPARAMS_PROG_FRAMELIST	BIT(1)
#define HCCPARAMS_ASYC_PARK		BIT(2)
#define HCCPARAMS_ISO_THRES_MASK	GENMASK(7, 4)
#define HCCPARAMS_EECP_MASK		GENMASK(15, 8)

/* HCSPPORTROUTE (0x0C) - Companion Port Route Description */
#define EHCI_HCSPPORTROUTE	0x0C

/*
 * EHCI operational register offsets are relative to the
 * operational base (op_base = reg_base + CAPLENGTH).
 * e.g. USBCMD is at reg_base+0x10 = op_base+0x00.
 */

/* USBCMD (op_base+0x00, absolute 0x10) - USB Command Register */
#define EHCI_USBCMD		0x00
#define USBCMD_RS			BIT(0)	/* Run/Stop */
#define USBCMD_HCRESET			BIT(1)	/* Host Controller Reset */
#define USBCMD_FLS_MASK			GENMASK(3, 2)	/* Frame List Size */
#define USBCMD_FLS_1024			(0 << 2)
#define USBCMD_FLS_512			(1 << 2)
#define USBCMD_FLS_256			(2 << 2)
#define USBCMD_PSE			BIT(4)	/* Periodic Schedule Enable */
#define USBCMD_ASE			BIT(5)	/* Asynchronous Schedule Enable */
#define USBCMD_IAAD			BIT(6)	/* Interrupt on Async Advance Doorbell */
#define USBCMD_LHCR			BIT(7)	/* Light Host Controller Reset */
#define USBCMD_ASPC_MASK		GENMASK(9, 8)	/* Async Schedule Park Count */
#define USBCMD_ASPE			BIT(11)	/* Async Schedule Park Mode Enable */
#define USBCMD_ITC_MASK			GENMASK(23, 16)	/* Interrupt Threshold Ctrl */
#define USBCMD_ITC_1			(0x01 << 16)
#define USBCMD_ITC_8			(0x08 << 16)	/* default, ~1 ms */

/* USBSTS (op_base+0x04) - USB Status Register */
#define EHCI_USBSTS		0x04
#define USBSTS_USBINT			BIT(0)	/* USB Interrupt */
#define USBSTS_USBERRINT		BIT(1)	/* USB Error Interrupt */
#define USBSTS_PCD			BIT(2)	/* Port Change Detect */
#define USBSTS_FLR			BIT(3)	/* Frame List Rollover */
#define USBSTS_HSE			BIT(4)	/* Host System Error */
#define USBSTS_IAA			BIT(5)	/* Interrupt on Async Advance */
#define USBSTS_HCH			BIT(12)	/* HC Halted */
#define USBSTS_RECLAMATION		BIT(13)	/* Reclamation */
#define USBSTS_PSS			BIT(14)	/* Periodic Schedule Status */
#define USBSTS_ASS			BIT(15)	/* Asynchronous Schedule Status */

/* USBINTR (op_base+0x08) - USB Interrupt Enable Register */
#define EHCI_USBINTR		0x08
#define USBINTR_USBINTE			BIT(0)
#define USBINTR_USBERRE			BIT(1)
#define USBINTR_PCDE			BIT(2)
#define USBINTR_FLRE			BIT(3)
#define USBINTR_HSEE			BIT(4)
#define USBINTR_IAAE			BIT(5)

/* FRINDEX (op_base+0x0C) - Frame Index Register */
#define EHCI_FRINDEX		0x0C
#define FRINDEX_MASK			GENMASK(13, 0)

/* CTRLDSSEGMENT (op_base+0x10) - 4G Segment Selector (only if 64-bit) */
#define EHCI_CTRLDSSEGMENT	0x10

/* PERIODICLISTBASE (op_base+0x14) - Periodic Frame List Base Address */
#define EHCI_PERIODICLISTBASE	0x14

/* ASYNCLISTADDR (op_base+0x18) - Current Asynchronous List Address */
#define EHCI_ASYNCLISTADDR	0x18

/* CONFIGFLAG (op_base+0x40) - Configured Flag Register */
#define EHCI_CONFIGFLAG		0x40
#define CONFIGFLAG_CF			BIT(0)

/* PORTSC (op_base+0x44) - Port Status and Control Register */
#define EHCI_PORTSC_BASE	0x44
#define PORTSC_CCS			BIT(0)	/* Current Connect Status */
#define PORTSC_CSC			BIT(1)	/* Connect Status Change */
#define PORTSC_PE			BIT(2)	/* Port Enabled/Disabled */
#define PORTSC_PEC			BIT(3)	/* Port Enable/Disable Change */
#define PORTSC_OCA			BIT(4)	/* Over-current Active */
#define PORTSC_OCC			BIT(5)	/* Over-current Change */
#define PORTSC_FPR			BIT(6)	/* Force Port Resume */
#define PORTSC_SUSPEND			BIT(7)	/* Suspend */
#define PORTSC_PR			BIT(8)	/* Port Reset */
#define PORTSC_LS_MASK			GENMASK(11, 10)	/* Line Status */
#define PORTSC_LS_SE0			(0 << 10)
#define PORTSC_LS_KSTATE		(1 << 10)	/* Low-speed device */
#define PORTSC_LS_JSTATE			(2 << 10)
#define PORTSC_PP			BIT(12)	/* Port Power */
#define PORTSC_PO			BIT(13)	/* Port Owner */
#define PORTSC_PIC_MASK			GENMASK(15, 14)
#define PORTSC_PTC_MASK			GENMASK(19, 16)	/* Port Test Control */
#define PORTSC_WKCNNT_E			BIT(20)	/* Wake on Connect Enable */
#define PORTSC_WKDSCNNT_E		BIT(21)	/* Wake on Disconnect Enable */
#define PORTSC_WKOC_E			BIT(22)	/* Wake on Over-current Enable */

/* Port indicator LED control (optional, EECP-based) */
#define PORTSC_PIC_OFF			(0 << 14)
#define PORTSC_PIC_AMBER		(1 << 14)
#define PORTSC_PIC_GREEN		(2 << 14)

/*
 * Queue Element Transfer Descriptor (qTD)
 *
 * Size: 32 bytes (5 DWords + 5 upper DWords for 64-bit)
 * Alignment: 32 bytes
 *
 * EHCI Spec section 3.5
 */
struct ehci_qtd {
	volatile uint32_t next;			/* Next qTD pointer (bits 4:0 = 0) */
	volatile uint32_t alt_next;		/* Alternate next qTD pointer */
	volatile uint32_t token;		/* Transfer status + length */
#define QTD_TOGGLE			BIT(31)
#define QTD_LENGTH_SHIFT		16
#define QTD_LENGTH_MASK			GENMASK(30, 16)
#define QTD_IOC				BIT(15)	/* Interrupt On Complete */
#define QTD_CERR_SHIFT			10
#define QTD_CERR_MASK			GENMASK(11, 10)
#define QTD_PID_OUT			0x00000000
#define QTD_PID_IN			0x00000100
#define QTD_PID_SETUP			0x00000200
#define QTD_STS_ACTIVE			BIT(7)
#define QTD_STS_HALT			BIT(6)
#define QTD_STS_DBE			BIT(5)	/* Data Buffer Error */
#define QTD_STS_BABBLE			BIT(4)
#define QTD_STS_XACT			BIT(3)	/* Transaction Error */
#define QTD_STS_MMF			BIT(2)	/* Missed Micro-Frame */
#define QTD_STS_STS			BIT(1)	/* Split Transaction State */
#define QTD_STS_PING			BIT(0)	/* Ping State */

	volatile uint32_t buf[5];		/* Buffer page pointers (5 pages max) */
	/*
	 * The following 5 DWords are for 64-bit addressing.
	 * For 32-bit systems, these are not used by hardware.
	 */
	volatile uint32_t buf_hi[5];		/* Buffer page pointers (high 32 bits) */
} __aligned(32);

/* QTD next pointer: Terminate bit (bit 0) */
#define QTD_NEXT_TERMINATE		BIT(0)
/* QTD alternate next pointer: NakCount reload field */
#define QTD_ALT_NEXT_NAKCNT_MASK	GENMASK(4, 1)

/*
 * Queue Head (qH)
 *
 * Size: 48 bytes hardware + software fields
 * Alignment: 32 bytes (hardware part is 48 bytes at 32-byte boundary)
 *
 * EHCI Spec section 3.6
 */
struct ehci_qh {
	/* --- Hardware part (48 bytes) --- */
	volatile uint32_t horiz_link;		/* Horizontal link pointer */
	/* Endpoint characteristics */
	volatile uint32_t ep_char;
/* EHCI Spec §3.6 Queue Head DWord 1 — Endpoint/Device Characteristics.
	* All fields match the EHCI specification bit positions exactly. */
#define QH_EP_CHAR_DEVADDR_SHIFT	0
#define QH_EP_CHAR_DEVADDR_MASK		GENMASK(6, 0)
#define QH_EP_CHAR_I			BIT(7)	/* Inactivate on next transaction */
#define QH_EP_CHAR_ENDPT_SHIFT		8
#define QH_EP_CHAR_ENDPT_MASK		GENMASK(11, 8)
#define QH_EP_CHAR_EPS_SHIFT		12
#define QH_EP_CHAR_EPS_FULL		(0 << 12)	/* Full-speed (12 Mbps) */
#define QH_EP_CHAR_EPS_LOW		(1 << 12)	/* Low-speed (1.5 Mbps) */
#define QH_EP_CHAR_EPS_HIGH		(2 << 12)	/* High-speed (480 Mbps) */
#define QH_EP_CHAR_DTC			BIT(14)	/* Data Toggle Control */
#define QH_EP_CHAR_H			BIT(15)	/* Head of Reclamation List */
#define QH_EP_CHAR_MAX_PKT_LEN_SHIFT	16
#define QH_EP_CHAR_MAX_PKT_LEN_MASK	GENMASK(26, 16)
#define QH_EP_CHAR_C			BIT(27)	/* Control endpoint flag */
#define QH_EP_CHAR_RL_SHIFT		28
#define QH_EP_CHAR_RL_MASK		GENMASK(31, 28)

	/* Endpoint capabilities */
	volatile uint32_t ep_caps;
#define QH_EP_CAPS_SSMASK_SHIFT		0
#define QH_EP_CAPS_SSMASK_MASK		GENMASK(7, 0)
#define QH_EP_CAPS_SCMASK_SHIFT		8
#define QH_EP_CAPS_SCMASK_MASK		GENMASK(15, 8)
#define QH_EP_CAPS_HUB_ADDR_SHIFT	16
#define QH_EP_CAPS_HUB_ADDR_MASK	GENMASK(22, 16)
#define QH_EP_CAPS_PORT_SHIFT		23
#define QH_EP_CAPS_PORT_MASK		GENMASK(29, 23)
#define QH_EP_CAPS_MULT_SHIFT		30
/* bits 30-31: High-bandwidth pipe multiplier */

	/* Current qTD pointer */
	volatile uint32_t current_qtd;

	/* Overlay qTD area (next, alt_next, token, buf[5], buf_hi[5]) */
	volatile uint32_t overlay_next;
	volatile uint32_t overlay_alt_next;
	volatile uint32_t overlay_token;
	volatile uint32_t overlay_buf[5];
	volatile uint32_t overlay_buf_hi[5];	/* 64-bit only */

	/* --- Software part --- */
	uint32_t qh_dma;		/* DMA address of this qH */
	struct ehci_qh *next;		/* Software link */
	void *xfer;			/* Back-pointer to uhc_transfer */
	struct ehci_qtd *qtd_head;	/* First qTD in chain (for cleanup) */
	struct ehci_qtd *data_qtd_sw;	/* Data-phase qTD (NULL if none) */
	uint16_t data_initial_len;	/* Initial NBYTES of data qTD */
	uint8_t xacterrs;		/* Consecutive XACT-error software retries */
} __aligned(32);

/* Maximum software retries for XACT errors before declaring failure.
 * Each retry lets the hardware attempt 3 transactions (CERR=3), so the
 * total attempt count is EHCI_XACTERR_MAX * 3. Matches Linux's limit. */
#define EHCI_XACTERR_MAX		32

/*
 * Periodic Frame Span Traversal Node (FSTN) - not used in Phase 1
 */
struct ehci_fstn {
	volatile uint32_t horiz_link;
	volatile uint32_t back_link;
} __aligned(32);

/* EHCI List Terminator */
#define EHCI_PTR_TERMINATE		BIT(0)

/* Type tags in periodic schedule entries (bits 1:0 of pointer) */
#define EHCI_PTR_TYPE_ITD		0x00
#define EHCI_PTR_TYPE_QH		0x02
#define EHCI_PTR_TYPE_SITD		0x04
#define EHCI_PTR_TYPE_FSTN		0x06

/* Maximum frame list size */
#define EHCI_FRAME_LIST_SIZE		1024
#define EHCI_FRAME_LIST_SIZE_512	512
#define EHCI_FRAME_LIST_SIZE_256	256

/* Default number of root hub ports (overridden by HCSPARAMS) */
#define EHCI_MAX_ROOT_PORTS		15

/* Memory pool configuration */
#define EHCI_MAX_QH			8
#define EHCI_MAX_QTD			16

/* Timeouts (microseconds) */
#define EHCI_RESET_TIMEOUT_US		100000	/* 100ms for HCRESET */
#define EHCI_RUN_STOP_TIMEOUT_US	16000	/* 16 micro-frames */
#define EHCI_PORT_RESET_MS		50	/* 50ms SE0 */

#endif /* ZEPHYR_DRIVERS_USB_UHC_UHC_EHCI_H_ */
