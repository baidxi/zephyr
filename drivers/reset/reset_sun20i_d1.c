/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner D1 / T113-S3 CCU reset controller driver.
 *
 * The CCU register bank contains the per-peripheral "Bus Gating and
 * Reset" (BGR) registers.  In every BGR register the reset bits occupy
 * the upper half (bit 16 and above) and are active-low: writing 1 keeps
 * the peripheral out of reset, writing 0 holds it in reset.
 *
 * This driver mirrors the existing sun8i V3s reset driver and the Linux
 * reset-simple / sunxi reset controller behaviour (active-low).  It is
 * modelled as a child node of the CCU clock controller so that it can
 * share the CCU register bank via DT_INST_PARENT().
 *
 * line_toggle() performs a clock-domain-synchronised reset release: it
 * enables the bus-clock gate, waits for the clock to stabilise, then
 * deasserts the reset line.  This matches the behaviour of Linux's
 * DesignWare 8250 driver (8250_dw.c), which only calls
 * reset_control_deassert() at probe time — never a full assert/deassert
 * cycle.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/reset/sun20i-d1-ccu.h>

#define DT_DRV_COMPAT allwinner_sun20i_d1_ccu_reset

/*
 * Reset timing constant (microseconds).
 *
 * D1_RESET_CLK_SYNC_US — guard time for bus-clock domain synchronisation,
 *                        applied after the gate is enabled and again after
 *                        the reset line is released.  At the 24 MHz APB
 *                        bus clock two microseconds is ~48 cycles, far
 *                        more than the handful the hardware actually
 *                        needs, yet negligible even during PRE_KERNEL_1.
 */
#define D1_RESET_CLK_SYNC_US	2

struct d1_reset_info {
	uint8_t id;
	uint16_t offset;
	uint32_t bit;
};

struct d1_reset_config {
	uintptr_t base;
};

struct d1_reset_data {
	struct k_spinlock lock;
	const struct d1_reset_info *rst_info;
	uint32_t nb_rst;
};

#define RST_INFO(_id, _offset, _bit)	\
{					\
	.id = _id,			\
	.offset = _offset,		\
	.bit = _bit,			\
}

/*
 * Mapping from the logical reset IDs (sun20i-d1-ccu.h) to the physical
 * BGR register offset and bit.  Only the peripherals that are wired up
 * in the devicetree (and a few common ones) are listed; unknown IDs are
 * rejected with -EINVAL.
 */
static const struct d1_reset_info d1_rst_infos[] = {
	/* UART_BGR_REG (0x090c): gate bits 0-5, reset bits 16-21 */
	RST_INFO(D1_RST_BUS_UART0, 0x90c, 16),
	RST_INFO(D1_RST_BUS_UART1, 0x90c, 17),
	RST_INFO(D1_RST_BUS_UART2, 0x90c, 18),
	RST_INFO(D1_RST_BUS_UART3, 0x90c, 19),
	RST_INFO(D1_RST_BUS_UART4, 0x90c, 20),
	RST_INFO(D1_RST_BUS_UART5, 0x90c, 21),
	/* TWI_BGR_REG (0x091c): gate bits 0-3, reset bits 16-19 */
	RST_INFO(D1_RST_BUS_I2C0, 0x91c, 16),
	RST_INFO(D1_RST_BUS_I2C1, 0x91c, 17),
	RST_INFO(D1_RST_BUS_I2C2, 0x91c, 18),
	RST_INFO(D1_RST_BUS_I2C3, 0x91c, 19),
	/* SPI_BGR_REG (0x096c): gate bits 0-1, reset bits 16-17 */
	RST_INFO(D1_RST_BUS_SPI0, 0x96c, 16),
	RST_INFO(D1_RST_BUS_SPI1, 0x96c, 17),
	/* SMHC_BGR_REG (0x084c): gate bits 0-2, reset bits 16-18 */
	RST_INFO(D1_RST_BUS_MMC0, 0x84c, 16),
	RST_INFO(D1_RST_BUS_MMC1, 0x84c, 17),
	RST_INFO(D1_RST_BUS_MMC2, 0x84c, 18),
	/* DMA_BGR_REG (0x070c): gate bit 0, reset bit 16 */
	RST_INFO(D1_RST_BUS_DMA, 0x70c, 16),
	/* EMAC_BGR_REG (0x097c): gate bit 0, reset bit 16 */
	RST_INFO(D1_RST_BUS_EMAC, 0x97c, 16),
	/* CAN_BGR_REG (0x092c): gate bits 0-1, reset bits 16-17 */
	RST_INFO(D1_RST_BUS_CAN0, 0x92c, 16),
	RST_INFO(D1_RST_BUS_CAN1, 0x92c, 17),
	/* USB PHY resets — dedicated PHY reset regs (active-low), bit 30 */
	RST_INFO(D1_RST_USB_PHY0, 0xa70, 30),
	RST_INFO(D1_RST_USB_PHY1, 0xa74, 30),
	/* USB_BGR_REG (0x0a8c): bus reset bits (active-low, set=deassert)
	 * OHCI0=16, OHCI1=17, EHCI0=20, EHCI1=21, OTG=24
	 */
	RST_INFO(D1_RST_BUS_OHCI0, 0xa8c, 16),
	RST_INFO(D1_RST_BUS_OHCI1, 0xa8c, 17),
	RST_INFO(D1_RST_BUS_EHCI0, 0xa8c, 20),
	RST_INFO(D1_RST_BUS_EHCI1, 0xa8c, 21),
	RST_INFO(D1_RST_BUS_OTG,  0xa8c, 24),
};

static const struct d1_reset_info *d1_reset_lookup(uint32_t id, uint32_t nb,
						   const struct d1_reset_info *table)
{
	for (uint32_t i = 0; i < nb; i++) {
		if (table[i].id == id) {
			return &table[i];
		}
	}

	return NULL;
}

static int d1_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct d1_reset_config *config = dev->config;
	struct d1_reset_data *data = dev->data;
	const struct d1_reset_info *info;
	uint32_t regval;
	k_spinlock_key_t key;

	info = d1_reset_lookup(id, data->nb_rst, data->rst_info);
	if (info == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + info->offset);
	/* Reset bits are active-low: clear the bit to assert reset. */
	regval &= ~BIT(info->bit);
	sys_write32(regval, config->base + info->offset);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int d1_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct d1_reset_config *config = dev->config;
	struct d1_reset_data *data = dev->data;
	const struct d1_reset_info *info;
	uint32_t regval;
	k_spinlock_key_t key;

	info = d1_reset_lookup(id, data->nb_rst, data->rst_info);
	if (info == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + info->offset);
	/* Reset bits are active-low: set the bit to deassert reset. */
	regval |= BIT(info->bit);
	sys_write32(regval, config->base + info->offset);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int d1_reset_line_toggle(const struct device *dev, uint32_t id)
{
	const struct d1_reset_config *config = dev->config;
	struct d1_reset_data *data = dev->data;
	const struct d1_reset_info *info;
	k_spinlock_key_t key;
	uint32_t gate_bit;

	info = d1_reset_lookup(id, data->nb_rst, data->rst_info);
	if (info == NULL) {
		return -EINVAL;
	}

	/*
	 * Clock-domain-synchronised reset release.
	 *
	 * Every BGR register pairs a bus-gate bit (lower half, bit N)
	 * with its reset bit (upper half, bit N + 16).  Before releasing
	 * the reset the bus clock is enabled and allowed to stabilise so
	 * the peripheral powers up against a clean, stable clock edge:
	 *
	 *   1. Enable the bus-clock gate.
	 *   2. Wait for the clock to propagate and stabilise.
	 *   3. Deassert the reset line (active-low: set the bit).
	 *   4. Wait for the clock domain to synchronise.
	 *
	 * The T113-S3 UART does not recover from a full assert/deassert
	 * cycle during PRE_KERNEL_1 (it returns all-zero reads, LSR == 0,
	 * hanging the console).  Linux's DesignWare 8250 driver
	 * (8250_dw.c) has the same constraint: at probe time it calls
	 * only reset_control_deassert(), never reset_control_reset().
	 * This implementation therefore synchronises the clock domain
	 * around the deassert — the correct upstream behaviour — rather
	 * than performing a destructive assert.
	 */
	gate_bit = info->bit - 16;

	key = k_spin_lock(&data->lock);

	/* 1. Enable the bus clock. */
	sys_set_bit(config->base + info->offset, gate_bit);

	/* 2. Let the clock stabilise. */
	k_busy_wait(D1_RESET_CLK_SYNC_US);

	/* 3. Deassert reset (active-low: set the bit). */
	sys_set_bit(config->base + info->offset, info->bit);

	/* 4. Wait for the clock domain to synchronise. */
	k_busy_wait(D1_RESET_CLK_SYNC_US);

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int d1_reset_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct d1_reset_config *config = dev->config;
	struct d1_reset_data *data = dev->data;
	const struct d1_reset_info *info;
	k_spinlock_key_t key;

	info = d1_reset_lookup(id, data->nb_rst, data->rst_info);
	if (info == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	/* Active-low: bit set => not in reset (0), bit clear => in reset (1). */
	*status = (sys_read32(config->base + info->offset) & BIT(info->bit)) ? 0U : 1U;
	k_spin_unlock(&data->lock, key);

	return 0;
}

DEVICE_API(reset, d1_reset_driver_api) = {
	.status = d1_reset_status,
	.line_assert = d1_reset_line_assert,
	.line_deassert = d1_reset_line_deassert,
	.line_toggle = d1_reset_line_toggle,
};

static int d1_reset_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define D1_RESET_INIT(n)							    \
	static const struct d1_reset_config d1_reset_config_##n = {		    \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)),				    \
	};									    \
	static struct d1_reset_data d1_reset_data_##n = {			    \
		.rst_info = d1_rst_infos,					    \
		.nb_rst = ARRAY_SIZE(d1_rst_infos),				    \
	};									    \
	DEVICE_DT_INST_DEFINE(n, d1_reset_init, NULL,				    \
			      &d1_reset_data_##n, &d1_reset_config_##n,		    \
			      PRE_KERNEL_1, CONFIG_RESET_INIT_PRIORITY,		    \
			      &d1_reset_driver_api);

DT_INST_FOREACH_STATUS_OKAY(D1_RESET_INIT)
