/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * OHCI (Open Host Controller Interface) register definitions
 * and data structures.
 *
 * References:
 *   - OHCI Specification Rev 1.0a
 *   - T113-S3 USB HCD manual (section 9.6.6)
 *   - Linux drivers/usb/host/ohci.h
 */

#ifndef ZEPHYR_DRIVERS_USB_UHC_UHC_OHCI_H_
#define ZEPHYR_DRIVERS_USB_UHC_UHC_OHCI_H_

#include <zephyr/types.h>
#include <zephyr/sys/util.h>

/* ================================================================
 * OHCI Register Offsets (relative to OHCI base = EHCI base + 0x0400)
 *
 * On T113-S3, the OHCI registers start at EHCI base + 0x0400.
 * The OHCI base address is 0x04200400 (from DT).
 * ================================================================ */

#define OHCI_HC_REVISION	0x00	/* HcRevision */
#define OHCI_HC_CONTROL		0x04	/* HcControl */
#define OHCI_HC_CMDSTATUS	0x08	/* HcCommandStatus */
#define OHCI_HC_INTSTATUS	0x0C	/* HcInterruptStatus */
#define OHCI_HC_INTENABLE	0x10	/* HcInterruptEnable */
#define OHCI_HC_INTDISABLE	0x14	/* HcInterruptDisable */
#define OHCI_HC_HCCA		0x18	/* HcHCCA */
#define OHCI_HC_PERIOD_CURED	0x1C	/* HcPeriodCurrentED */
#define OHCI_HC_CTRL_HEADED	0x20	/* HcControlHeadED */
#define OHCI_HC_CTRL_CURED	0x24	/* HcControlCurrentED */
#define OHCI_HC_BULK_HEADED	0x28	/* HcBulkHeadED */
#define OHCI_HC_BULK_CURED	0x2C	/* HcBulkCurrentED */
#define OHCI_HC_DONEHEAD	0x30	/* HcDoneHead */
#define OHCI_HC_FMINTERVAL	0x34	/* HcFmInterval */
#define OHCI_HC_FMREMAINING	0x38	/* HcFmRemaining */
#define OHCI_HC_FMNUMBER	0x3C	/* HcFmNumber */
#define OHCI_HC_PERIODICSTART	0x40	/* HcPeriodicStart */
#define OHCI_HC_LSTHRESHOLD	0x44	/* HcLSThreshold */
#define OHCI_HC_RHDESCA		0x48	/* HcRhDescriptorA */
#define OHCI_HC_RHDESCB		0x4C	/* HcRhDescriptorB */
#define OHCI_HC_RHSTATUS	0x50	/* HcRhStatus */
#define OHCI_HC_RHPORTSTATUS	0x54	/* HcRhPortStatus[n] */

/* --- HcControl bits --- */
#define OHCI_CTRL_CBSR_MASK	GENMASK(1, 0)	/* Control/Bulk service ratio */
#define OHCI_CTRL_PLE		BIT(2)	/* Periodic list enable */
#define OHCI_CTRL_IE		BIT(3)	/* Isochronous enable */
#define OHCI_CTRL_CLE		BIT(4)	/* Control list enable */
#define OHCI_CTRL_BLE		BIT(5)	/* Bulk list enable */
#define OHCI_CTRL_HCFS_MASK	GENMASK(7, 6)	/* Host controller functional state */
#define OHCI_CTRL_HCFS_RESET	(0 << 6)
#define OHCI_CTRL_HCFS_RESUME	(1 << 6)
#define OHCI_CTRL_HCFS_OPER	(2 << 6)
#define OHCI_CTRL_HCFS_SUSPEND	(3 << 6)
#define OHCI_CTRL_IR		BIT(8)	/* Interrupt routing */
#define OHCI_CTRL_RWC		BIT(9)	/* Remote wakeup connected */
#define OHCI_CTRL_RWE		BIT(10)	/* Remote wakeup enable */

/* --- HcCommandStatus bits --- */
#define OHCI_CMDS_HCR		BIT(0)	/* Host controller reset */
#define OHCI_CMDS_CLF		BIT(1)	/* Control list filled */
#define OHCI_CMDS_BLF		BIT(2)	/* Bulk list filled */
#define OHCI_CMDS_OCR		BIT(3)	/* Ownership change request */

/* --- Interrupt bits --- */
#define OHCI_INTR_SO		BIT(0)	/* Scheduling overrun */
#define OHCI_INTR_WDH		BIT(1)	/* Writeback Done Head */
#define OHCI_INTR_SF		BIT(2)	/* Start of frame */
#define OHCI_INTR_RD		BIT(3)	/* Resume detected */
#define OHCI_INTR_UE		BIT(4)	/* Unrecoverable error */
#define OHCI_INTR_FNO		BIT(5)	/* Frame number overflow */
#define OHCI_INTR_RHSC		BIT(6)	/* Root hub status change */
#define OHCI_INTR_OC		BIT(30)	/* Ownership change */
#define OHCI_INTR_MIE		BIT(31)	/* Master interrupt enable */

/* --- HcRhPortStatus bits --- */
#define OHCI_RHPS_CCS		BIT(0)	/* Current connect status */
#define OHCI_RHPS_PES		BIT(1)	/* Port enable status */
#define OHCI_RHPS_PSS		BIT(2)	/* Port suspend status */
#define OHCI_RHPS_POCI		BIT(3)	/* Port over-current indicator */
#define OHCI_RHPS_PRS		BIT(4)	/* Port reset status */
#define OHCI_RHPS_PPS		BIT(8)	/* Port power status */
#define OHCI_RHPS_LSDA		BIT(9)	/* Low speed device attached */
#define OHCI_RHPS_CSC		BIT(16)	/* Connect status change */
#define OHCI_RHPS_PESC		BIT(17)	/* Port enable status change */
#define OHCI_RHPS_PSSC		BIT(18)	/* Port suspend status change */
#define OHCI_RHPS_OCIC		BIT(19)	/* Over-current indicator change */
#define OHCI_RHPS_PRSC		BIT(20)	/* Port reset status change */

/* ================================================================
 * OHCI Data Structures
 * ================================================================ */

/*
 * Host Controller Communications Area (HCCA)
 * Must be 256-byte aligned.
 */
struct ohci_hcca {
	volatile uint32_t int_table[32];	/* Interrupt ED table */
	volatile uint16_t frame_no;		/* Current frame number */
	volatile uint16_t pad1;
	volatile uint32_t done_head;		/* Done queue head */
	uint8_t reserved[116];
	uint8_t what[4];
} __aligned(256);

/*
 * Endpoint Descriptor (ED)
 * Must be 16-byte aligned.
 */
struct ohci_ed {
	volatile uint32_t hw_info;
#define ED_SKIP		BIT(14)
#define ED_LOWSPEED	BIT(13)
#define ED_IN		(2 << 11)
#define ED_OUT		(1 << 11)
#define ED_ISO		BIT(15)

	volatile uint32_t hw_tailp;	/* Tail TD pointer */
	volatile uint32_t hw_headp;	/* Head TD pointer */
#define ED_H		BIT(0)	/* Halted */
#define ED_C		BIT(1)	/* Toggle carry */

	volatile uint32_t hw_nexted;	/* Next ED */
} __aligned(16);

/*
 * Transfer Descriptor (TD)
 * Must be 32-byte aligned (general TD is 16 bytes but pad for ISO).
 */
struct ohci_td {
	volatile uint32_t hw_info;
#define TD_CC_SHIFT		28
#define TD_CC_MASK		GENMASK(31, 28)
#define TD_CC_NOERROR		0
#define TD_CC_CRC		1
#define TD_CC_BITSTUFFING	2
#define TD_CC_DATATOGGLE	3
#define TD_CC_STALL		4
#define TD_DI_SHIFT		21
#define TD_DI_MASK		GENMASK(23, 21)
#define TD_DONE			BIT(17)
#define TD_EC_SHIFT		26
#define TD_T_SHIFT		24
#define TD_T_DATA0		(2 << 24)
#define TD_T_DATA1		(3 << 24)
#define TD_DP_SHIFT		19
#define TD_DP_SETUP		(0 << 19)
#define TD_DP_IN		(1 << 19)
#define TD_DP_OUT		(2 << 19)
#define TD_R			BIT(18)	/* Round: short packets OK */

	volatile uint32_t hw_cbp;	/* Current Buffer Pointer */
	volatile uint32_t hw_nexttd;	/* Next TD */
	volatile uint32_t hw_be;	/* Buffer End */
} __aligned(32);

/* ================================================================
 * Memory pool configuration
 * ================================================================ */

#define OHCI_MAX_ED		16
#define OHCI_MAX_TD		16

/* Frame interval nominal value (11,999 bit times) */
#define OHCI_FM_INTERVAL	0x2EDF

/* Periodic start: ~90% of frame interval */
#define OHCI_PERIODIC_START	0x2A3F

/* LS threshold default */
#define OHCI_LS_THRESHOLD	0x0628

#endif /* ZEPHYR_DRIVERS_USB_UHC_UHC_OHCI_H_ */
