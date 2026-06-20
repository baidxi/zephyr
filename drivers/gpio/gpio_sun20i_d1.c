/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner D1 / T113-S3 GPIO driver.
 * 6 ports (B-G), 6 IRQ banks.
 * Register layout: D1 NEW_REG_LAYOUT (PDF §9.7.4, BANK_MEM_SIZE=0x30).
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT allwinner_sun20i_d1_gpio

LOG_MODULE_REGISTER(gpio_d1, LOG_LEVEL_WRN);

/* D1 / T113-S3 new register layout: BANK_MEM_SIZE=0x30 */
#define PORT_STRIDE 0x30
/* PIO base → each port CFG starts at port_idx * 0x30 */
#define PORTS       6
#define PINS_PER    32

#define REG_CFG(p)  (((p) / 8) * 4)
#define REG_DRV(p)  (((p) >= 16) ? 0x18 : 0x14)
/* PULL at +0x24 (pins 16-31), +0x28 (pins 0-15) on D1 */
#define REG_PULL(p) (((p) >= 16) ? 0x28 : 0x24)
#define REG_DAT     0x10

/* IRQ registers (per port, base offset 0x200, stride 0x20) */
#define IRQ_BASE      0x200
#define IRQ_STRIDE    0x20
#define IRQ_CFG0_OFF  0x00
#define IRQ_CTL_OFF   0x10
#define IRQ_STATUS_OFF 0x14
#define IRQ_DEB_OFF   0x18

/* IRQ bank mapping: PB→1, PC→2, PD→3, PE→4, PF→5, PG→6 */
#define IRQ_BANK(port) ((port) + 1)  /* port 0=PB→bank1, port 5=PG→bank6 */

struct gpio_d1_cfg {
	uintptr_t base;      /* absolute base address for this port */
	uint8_t   port;      /* port index (0=PB, 1=PC, ... 5=PG) */
};

struct gpio_d1_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	struct k_spinlock lock;
};

static int gpio_d1_config(const struct device *dev, gpio_pin_t pin0, gpio_flags_t flags)
{
	uint32_t port = pin0 / PINS_PER;
	uint32_t pin  = pin0 % PINS_PER;
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t base = cfg->base;
	uint32_t val;

	if (port >= PORTS || pin >= PINS_PER)
		return -EINVAL;

	val = sys_read32(base + REG_CFG(pin));
	val &= ~(0xf << ((pin % 8) * 4));
	if (flags & GPIO_OUTPUT)
		val |= 1 << ((pin % 8) * 4);
	sys_write32(val, base + REG_CFG(pin));

	if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
		uint32_t pull = (flags & GPIO_PULL_UP) ? 1 : 2;
		uint32_t off = ((pin >= 16) ? (pin - 16) : pin) * 2;

		val = sys_read32(base + REG_PULL(pin));
		val &= ~(0x3 << off);
		val |= pull << off;
		sys_write32(val, base + REG_PULL(pin));
	}

	if (flags & GPIO_OUTPUT_INIT_HIGH)
		sys_set_bit(base + REG_DAT, pin);
	else if (flags & GPIO_OUTPUT_INIT_LOW)
		sys_clear_bit(base + REG_DAT, pin);

	return 0;
}

static int gpio_d1_port_get_raw(const struct device *dev, uint32_t *val)
{
	const struct gpio_d1_cfg *cfg = dev->config;

	*val = sys_read32(cfg->base + REG_DAT);
	return 0;
}

static int gpio_d1_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t val)
{
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t addr = cfg->base + REG_DAT;

	sys_write32((sys_read32(addr) & ~mask) | (val & mask), addr);
	return 0;
}

static int gpio_d1_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_d1_cfg *cfg = dev->config;
	uint32_t bit = __builtin_ctz(mask);

	sys_set_bit(cfg->base + REG_DAT, bit);
	return 0;
}

static int gpio_d1_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_d1_cfg *cfg = dev->config;
	uint32_t bit = __builtin_ctz(mask);

	sys_clear_bit(cfg->base + REG_DAT, bit);
	return 0;
}

static int gpio_d1_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t addr = cfg->base + REG_DAT;

	sys_write32(sys_read32(addr) ^ mask, addr);
	return 0;
}

static int gpio_d1_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					   enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t irq_base = cfg->base + IRQ_BASE + IRQ_BANK(cfg->port) * IRQ_STRIDE;
	uint32_t cfg_reg = irq_base + IRQ_CFG0_OFF + ((pin / 8) * 4);
	uint32_t val, cfg_val = 0;

	/* Map Zephyr mode/trig to D1 EINT config (PDF §9.7.5.36) */
	if (mode == GPIO_INT_MODE_LEVEL) {
		cfg_val = (trig == GPIO_INT_TRIG_HIGH) ? 0x2 : 0x3;
	} else if (mode == GPIO_INT_MODE_EDGE) {
		switch ((int)trig) {
		case GPIO_INT_TRIG_HIGH:
			cfg_val = 0x0; /* Positive Edge */
			break;
		case GPIO_INT_TRIG_LOW:
			cfg_val = 0x1; /* Negative Edge */
			break;
		case GPIO_INT_TRIG_BOTH:
			cfg_val = 0x4; /* Double Edge */
			break;
		default:
			return -ENOTSUP;
		}
	} else {
		/* Disable interrupt */
		cfg_val = 0xf; /* IO Disable (reserved value) */
	}

	val = sys_read32(cfg_reg);
	val &= ~(0xf << ((pin % 8) * 4));
	val |= cfg_val << ((pin % 8) * 4);
	sys_write32(val, cfg_reg);

	return 0;
}

static int gpio_d1_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set)
{
	struct gpio_d1_data *data = dev->data;
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t irq_base = cfg->base + IRQ_BASE + IRQ_BANK(cfg->port) * IRQ_STRIDE;
	uintptr_t ctl_reg = irq_base + IRQ_CTL_OFF;

	if (set) {
		/* Enable EINT for this pin */
		uint32_t mask = sys_read32(ctl_reg);

		mask |= cb->pin_mask;
		sys_write32(mask, ctl_reg);
	} else {
		/* Disable EINT for this pin */
		uint32_t mask = sys_read32(ctl_reg);

		mask &= ~cb->pin_mask;
		sys_write32(mask, ctl_reg);
	}

	return gpio_manage_callback(&data->callbacks, cb, set);
}

static uint32_t gpio_d1_get_pending_int(const struct device *dev)
{
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t irq_base = cfg->base + IRQ_BASE + IRQ_BANK(cfg->port) * IRQ_STRIDE;

	return sys_read32(irq_base + IRQ_STATUS_OFF);
}

static void gpio_d1_isr(const struct device *dev)
{
	struct gpio_d1_data *data = dev->data;
	const struct gpio_d1_cfg *cfg = dev->config;
	uintptr_t irq_base = cfg->base + IRQ_BASE + IRQ_BANK(cfg->port) * IRQ_STRIDE;
	uint32_t status = sys_read32(irq_base + IRQ_STATUS_OFF);

	/* W1C: write 1 to clear pending bits */
	sys_write32(status, irq_base + IRQ_STATUS_OFF);

	/* Fire callbacks for matching pins */
	gpio_fire_callbacks(&data->callbacks, dev, status);
}

static const struct gpio_driver_api gpio_d1_api = {
	.pin_configure           = gpio_d1_config,
	.port_get_raw            = gpio_d1_port_get_raw,
	.port_set_masked_raw     = gpio_d1_port_set_masked_raw,
	.port_set_bits_raw       = gpio_d1_port_set_bits_raw,
	.port_clear_bits_raw     = gpio_d1_port_clear_bits_raw,
	.port_toggle_bits        = gpio_d1_port_toggle_bits,
	.pin_interrupt_configure = gpio_d1_pin_interrupt_configure,
	.manage_callback         = gpio_d1_manage_callback,
	.get_pending_int         = gpio_d1_get_pending_int,
};

static int gpio_d1_init(const struct device *dev)
{
	const struct gpio_d1_cfg *cfg = dev->config;

	LOG_DBG("port %u ready @ 0x%lx", cfg->port, (unsigned long)cfg->base);
	return 0;
}

/*
 * V3S-style init macro: conditionally connect IRQ if the GPIO node
 * has an 'interrupts' property, then enable it at the GIC level.
 */
#define GPIO_D1_INIT(n)							\
	static struct gpio_d1_data gpio_d1_data_##n;			\
	static const struct gpio_d1_cfg gpio_d1_cfg_##n = {		\
		.port = DT_REG_ADDR(DT_DRV_INST(n)),			\
		.base = DT_REG_ADDR(DT_INST_PARENT(n)) +		\
			PORT_STRIDE * DT_REG_ADDR(DT_DRV_INST(n)),	\
	};								\
	COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, 0),				\
		(static void gpio_d1_irq_init_##n(void)			\
		{							\
			IRQ_CONNECT(DT_INST_IRQN(n),			\
				    DT_INST_IRQ(n, priority),		\
				    gpio_d1_isr,			\
				    DEVICE_DT_GET(DT_DRV_INST(n)),	\
				    0);					\
			irq_enable(DT_INST_IRQN(n));			\
		}),							\
		(static void gpio_d1_irq_init_##n(void) { ; })		\
	)								\
	static int gpio_d1_init_##n(const struct device *dev)		\
	{								\
		gpio_d1_irq_init_##n();					\
		return gpio_d1_init(dev);				\
	}								\
	DEVICE_DT_INST_DEFINE(n, gpio_d1_init_##n, NULL,		\
			      &gpio_d1_data_##n, &gpio_d1_cfg_##n,	\
			      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,	\
			      &gpio_d1_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_D1_INIT)
