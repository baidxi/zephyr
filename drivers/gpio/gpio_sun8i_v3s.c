/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/sys_io.h>
#include <soc.h>
#include <sys/errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio/gpio_utils.h>


#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT allwinner_sun8i_v3s_gpio

LOG_MODULE_REGISTER(sun8i, CONFIG_GPIO_LOG_LEVEL);

#define SUN8I_GPIO_INT_DOUBLE_EDGE  4
#define SUN8I_GPIO_INT_LEVEL_LO  3
#define SUN8I_GPIO_INT_LEVEL_HI  2
#define SUN8I_GPIO_INT_FAILLING 1
#define SUN8I_GPIO_INT_RISING  0

struct sun8i_gpio_data {
  struct gpio_driver_data common;
  sys_slist_t callbacks;
  uint8_t irq_count;
};

struct sun8i_gpio_config {
  struct gpio_driver_config common;
  uint8_t port_id;
  void (*irq_config)(const struct device *dev);
  uintptr_t reg;
  bool is_intc;
};

static int sun8i_gpio_pin_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
  const struct sun8i_gpio_config *config = dev->config;
  uint8_t cfg_offset = pin / 8;
  uint8_t pin_offset = pin > 8 ? pin % 8 : pin;
  uint32_t val;

  val = sys_read32(config->reg + 0x24 * config->port_id + cfg_offset * 0x4);
  val &= ~(0x7 << (pin_offset * 4));
  if (flags & GPIO_OUTPUT)
    val |= 1 << (pin_offset * 4);

  sys_write32(val, config->reg + 0x24 * config->port_id + cfg_offset * 0x4);

  return 0;
}

static int sun8i_gpio_port_get_raw(const struct device *dev, uint32_t *value)
{
  const struct sun8i_gpio_config *config = dev->config;

  *value = sys_read32(config->reg + config->port_id * 0x24 + 0x10);

  return 0;
}

static int sun8i_gpio_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value)
{
  const struct sun8i_gpio_config *config = dev->config;
  uint32_t v = sys_read32(config->reg + config->port_id * 0x24 + 0x10);
  int i;


  for (i = 0; i < 14; i++)
  {
    if ((1 << i) & mask)
    {
      if (value & (1 << i))
        v |= 1 << i;
      else
        v &= ~(1 << i);
    }
  }

  sys_write32(v, config->reg + config->port_id * 0x24 + 0x10);

  return 0;
}

static int sun8i_gpio_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
  const struct sun8i_gpio_config *config = port->config;
  uint32_t value = sys_read32(config->reg + config->port_id * 0x24 + 0x10);
  int i;

  for (i = 0; i < 14; i++)
  {
    if ((1 << i) & pins)
    {
      value |= 1 << i;
    }
  }

  sys_write32(value, config->reg + config->port_id * 0x24 + 0x10);

  return 0;
}

static int sun8i_gpio_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
  const struct sun8i_gpio_config *config = port->config;
  uint32_t value = sys_read32(config->reg + config->port_id * 0x24 + 0x10);
  int i;

  for (i = 0; i < 14; i++)
  {
    if ((1 << i) & pins)
      value &= ~(1 << i);
  }

  sys_write32(value, config->reg + config->port_id * 0x24 + 0x10);

  return 0;
}

static int sun8i_gpio_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
  const struct sun8i_gpio_config *config = port->config;
  uint32_t value = sys_read32(config->reg + config->port_id * 0x24 + 0x10);
  int i;

  for (i = 0; i < 14; i++)
  {
    if ((1 << i) & pins)
    {
      if (value & (1 << i))
        value &= ~(1 << i);
      else
        value |= 1 << i;
    }
  }

  sys_write32(value, config->reg + config->port_id * 0x24 + 0x10);

  return 0;
}

static int sun8i_gpio_pin_interrupt_configure(const struct device *port, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
  const struct sun8i_gpio_config *config = port->config;
  uint8_t cfg_offset = pin / 8;
  uint8_t pin_offset = pin > 8 ? pin % 8 : pin;
  uint32_t value;
  uint8_t cfg = 0;

  if (!config->is_intc)
    return -EOPNOTSUPP;

  switch((int)mode)
  {
    case GPIO_INT_MODE_LEVEL:
      cfg = trig == GPIO_INT_TRIG_LOW ? SUN8I_GPIO_INT_LEVEL_LO : SUN8I_GPIO_INT_LEVEL_HI;
      break;
    case GPIO_INT_MODE_EDGE:
      switch((int)trig)
      {
        case GPIO_INT_TRIG_BOTH:
          cfg = SUN8I_GPIO_INT_DOUBLE_EDGE;
          break;
        case GPIO_INT_TRIG_HIGH:
          cfg = SUN8I_GPIO_INT_RISING;
          break;
        case GPIO_INT_TRIG_LOW:
          cfg = SUN8I_GPIO_INT_FAILLING;
          break;
      }
      break;
    default:
      cfg = 0;
      break;
  }

  value = sys_read32(config->reg + 0x200 + 0x20 * config->port_id + cfg_offset * 4);
  value &= ~(0xf << pin_offset);
  value |= cfg << pin_offset;

  sys_write32(value, config->reg + 0x200 + 0x20 * config->port_id + cfg_offset * 4);

  return 0;
}

static int sun8i_gpio_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
  struct sun8i_gpio_data *data = port->data;
  const struct sun8i_gpio_config *config = port->config;
  struct gpio_callback *callback;
  sys_snode_t *node;
  uint32_t value;

  if (!config->is_intc)
    return -EOPNOTSUPP;

  if (set)
  {
    SYS_SLIST_FOR_EACH_NODE(&data->callbacks, node)
    {
      callback = CONTAINER_OF(node, struct gpio_callback, node);
      if (cb->handler == callback->handler)
      {
        callback->pin_mask |= cb->pin_mask;
      }
    }

    if (data->irq_count == 0)
    {
      sys_slist_append(&data->callbacks, &cb->node);
      value = sys_read32(config->reg + 0x200 + 0x20 * config->port_id + 0x10);
      value |= cb->pin_mask;
      sys_write32(value, config->reg + 0x200 + 0x20 * config->port_id + 0x10);
      data->irq_count++;
    }

  } else {
    if (data->irq_count <= 0)
      return 0;

    data->irq_count--;

    value = sys_read32(config->reg + 0x200 + 0x20 * config->port_id + 0x10);
    value &= ~cb->pin_mask;
    sys_write32(value, config->reg + 0x200 + 0x20 * config->port_id + 0x10);
    sys_slist_find_and_remove(&data->callbacks, &cb->node);
  }

  return 0;
}

static void sun8i_gpio_isr(const struct device *port)
{
  const struct sun8i_gpio_config *config = port->config;
  struct sun8i_gpio_data *data = port->data;
  uint32_t value = sys_read32(config->reg + 0x200 + 0x20 * config->port_id + 0x14);
  struct gpio_callback *cb;
  sys_snode_t *node;

  sys_write32(value, config->reg + 0x200 + 0x20 * config->port_id + 0x14);

  SYS_SLIST_FOR_EACH_NODE(&data->callbacks, node)
  {
    cb = CONTAINER_OF(node, struct gpio_callback, node);
    if (cb->pin_mask & value)
    {
      cb->handler(port, cb, value);
    }
  }
}

static uint32_t sun8i_gpio_get_pending_int(const struct device *port)
{
  return 0;
}

#ifdef CONFIG_GPIO_GET_DIRECTION
static int sun8i_gpio_get_direction(const struct device *port, gpio_port_pins_t map, 
            gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
  return 0;
}
#endif

#ifdef CONFIG_GPIO_GET_CONFIG

static int gpio_sun8i_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *flags)
{
  return 0;
}

#endif

static int sun8i_gpio_init(const struct device *port)
{
  struct sun8i_gpio_data *data = port->data;
  const struct sun8i_gpio_config *config = port->config;
  sys_slist_init(&data->callbacks);

  if (config->is_intc)
    config->irq_config(port);

  return 0;
}

#ifdef CONFIG_PM_DEVICE

static int sun8i_gpio_pm_action(const struct device *port, enum pm_device_action action)
{
  return 0;
}

#endif

DEVICE_API(gpio, sun8i_v3s_gpio_driver) = {
  .pin_configure = sun8i_gpio_pin_config,
#ifdef CONFIG_GPIO_GET_CONFIG
  .pin_get_config = gpio_sun8i_get_config,
#endif
  .port_get_raw = sun8i_gpio_port_get_raw,
  .port_set_masked_raw = sun8i_gpio_port_set_masked_raw,
  .port_clear_bits_raw = sun8i_gpio_port_clear_bits_raw,
  .port_toggle_bits = sun8i_gpio_port_toggle_bits,
  .port_set_bits_raw = sun8i_gpio_port_set_bits_raw,
  .pin_interrupt_configure = sun8i_gpio_pin_interrupt_configure,
  .manage_callback = sun8i_gpio_manage_callback,
  .get_pending_int = sun8i_gpio_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
  .port_get_direction = sun8i_gpio_get_direction,
#endif
};

#define SUN8I_GPIO_IRQ_FLAGS(n) \
  _CONCAT(SUN8I_GPIO_IRQ_FLAGS_SENSE, DT_INST_IRQ_HAS_CELL(n, sense))(n)

#define SUN8I_GPIO_DEVICE_INIT(n) \
  static struct sun8i_gpio_data gpio##n##_data; \
  COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, 0), \
    (static void sun8i_gpio_irq_config_##n(const struct device *dev) \
    { \
      IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), \
      sun8i_gpio_isr, \
      DEVICE_DT_GET(DT_DRV_INST(n)), 0); \
      irq_enable(DT_INST_IRQN(n)); \
    }), \
    (static void sun8i_gpio_irq_config_##n(const struct device *dev) { ARG_UNUSED(dev); }) \
  ) \
  static const struct sun8i_gpio_config gpio##n##_config = {  \
    .common = { 0xffffffff }, \
    .port_id = DT_REG_ADDR(DT_DRV_INST(n)), \
    .reg = DT_REG_ADDR(DT_INST_PARENT(n)),  \
    .irq_config = sun8i_gpio_irq_config_##n,  \
    .is_intc = DT_NODE_HAS_PROP(DT_DRV_INST(n), interrupt_controller),  \
  };  \
  PM_DEVICE_DEFINE(gpio_##n, sun8i_gpio_pm_action); \
  DEVICE_DT_INST_DEFINE(n,  \
    sun8i_gpio_init,  \
    PM_DEVICE_GET(gpio_##n),  \
    &gpio##n##_data,  \
    &gpio##n##_config,  \
    POST_KERNEL,    \
    CONFIG_GPIO_INIT_PRIORITY,  \
    &sun8i_v3s_gpio_driver); 

DT_INST_FOREACH_STATUS_OKAY(SUN8I_GPIO_DEVICE_INIT)