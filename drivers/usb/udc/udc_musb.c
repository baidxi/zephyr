/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MUSB UDC adapter — implements Zephyr UDC API on top of the MUSB core.
 *
 * Three-layer architecture:
 *   udc_musb.c (this file)  -> struct udc_api, callbacks, DEVICE_DEFINE
 *   musb_core.c             -> ISR dispatch, FIFO, EP management
 *   glue/sunxi.c            -> platform register mapping, PHY control
 *
 * PHY init happens in driver_preinit (POST_KERNEL, before main()),
 * so clocks/resets/calibration are ready before any USB stack call.
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_musb

#include "udc_common.h"

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/util.h>

#include "../common/musb/musb_core.h"
#include "../common/sunxi/usb_common.h"

LOG_MODULE_REGISTER(udc_musb, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define MUSB_MAX_EPS 16

struct udc_musb_config {
	mm_reg_t base;
	void (*irq_config)(const struct device *dev);
};

struct udc_musb_data {
	struct udc_data udc;
	struct musb_dev musb;
	struct musb_hw_ep hw_eps[MUSB_MAX_EPS];

	/* Deferred EP0 SETUP processing.
	 * udc_setup_received() takes a mutex internally, which is illegal
	 * from ISR context.  Cache the 8-byte SETUP payload and submit
	 * a work item to the UDC work queue so it runs in thread context.
	 */
	struct k_work ep0_setup_work;
	uint8_t ep0_setup_buf[8];
};

struct musb_dev *to_musb(const struct device *dev)
{
	struct udc_musb_data *priv = udc_get_private(dev);
	return &priv->musb;
}

/* =========================================================================
 * MUSB Callbacks
 * ========================================================================= */

/*
 * Work handler for deferred EP0 SETUP processing.
 *
 * udc_setup_received() takes a mutex (k_mutex_lock) internally,
 * which is illegal from ISR context and causes intermittent
 * deadlocks / ETIMEDOUT when the mutex is contested by the USBD
 * work thread (e.g. during Shell TX/RX activity).
 *
 * We cache the 8-byte SETUP payload in ep0_setup_buf and submit
 * this work item to the UDC work queue so the actual call runs
 * in a normal thread context where mutex operations are legal.
 */
static void udc_musb_ep0_setup_work(struct k_work *item)
{
	struct udc_musb_data *priv =
		CONTAINER_OF(item, struct udc_musb_data, ep0_setup_work);
	const struct device *dev = priv->musb.user_data;

	udc_setup_received(dev, priv->ep0_setup_buf);
}

static void udc_musb_ep0_setup_cb(const struct musb_dev *musb,
				  const void *setup_data)
{
	const struct device *dev = musb->user_data;
	struct udc_musb_data *priv = udc_get_private(dev);

	/* Cache SETUP data (8 bytes) — the FIFO buffer in musb_core.c
	 * is a local variable that goes out of scope when the ISR returns.
	 */
	memcpy(priv->ep0_setup_buf, setup_data, 8);

	LOG_DBG("SETUP: %02x %02x %04x %04x dir=%s (deferred)",
		priv->ep0_setup_buf[0], priv->ep0_setup_buf[1],
		(uint16_t)(priv->ep0_setup_buf[2] | (priv->ep0_setup_buf[3] << 8)),
		(uint16_t)(priv->ep0_setup_buf[6] | (priv->ep0_setup_buf[7] << 8)),
		(priv->ep0_setup_buf[0] & 0x80) ? "IN" : "OUT");

	/* Submit to UDC work queue — runs in thread context */
	k_work_submit_to_queue(udc_get_work_q(), &priv->ep0_setup_work);
}

static void udc_musb_tx_done_cb(const struct musb_dev *musb, uint8_t ep)
{
	const struct device *dev = musb->user_data;
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	if (ep == 0) {
		LOG_DBG("tx_done: EP0");
	} else {
		LOG_DBG("tx_done: EP%u", ep);
	}
	ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | ep);
	if (!ep_cfg)
		return;

	buf = udc_buf_peek(ep_cfg);
	if (buf) {
		LOG_DBG("tx_done EP%u: dequeue buf=%p len=%u", ep, buf, buf->len);
		udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, buf, 0);
	} else {
		LOG_WRN("tx_done EP%u: no buf in queue!", ep);
	}

	/*
	 * Start the next queued transfer (if any) for non-EP0 endpoints.
	 * In the Zephyr UDC model, the upper layer can queue multiple bufs
	 * before the current transfer completes.  After dequeueing the
	 * finished buf above, peek at the queue head and kick off the next
	 * transfer by calling musb_ep_tx() from ISR context.
	 */
	if (ep > 0) {
		buf = udc_buf_peek(ep_cfg);
		if (buf && !ep_cfg->stat.halted) {
			LOG_DBG("tx_done EP%u: start next buf=%p len=%u",
				ep, buf, buf->len);
			musb_ep_tx(musb, ep, buf->data, buf->len);
		}
	}
}

static void udc_musb_rx_ready_cb(const struct musb_dev *musb, uint8_t ep,
				 uint16_t len)
{
	const struct device *dev = musb->user_data;
	struct udc_ep_config *ep_cfg;
	struct net_buf *buf;

	ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_OUT | ep);
	if (!ep_cfg)
		return;

	buf = udc_buf_peek(ep_cfg);
	if (buf) {
		musb->ops->read_fifo(
			musb->base + musb->ops->fifo_offset(ep),
			len, buf->data);
		net_buf_add(buf, len);
		udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, buf, 0);
	}
}

static void udc_musb_bus_event_cb(const struct musb_dev *musb,
				  enum musb_bus_event event)
{
	static const char * const names[] = {
		[MUSB_BUS_RESET]      = "RESET",
		[MUSB_BUS_SUSPEND]    = "SUSPEND",
		[MUSB_BUS_RESUME]     = "RESUME",
		[MUSB_BUS_DISCONNECT] = "DISCONNECT",
		[MUSB_BUS_VBUS_ERROR] = "VBUS_ERR",
		[MUSB_BUS_SOF]        = "SOF",
	};
	static const enum udc_event_type map[] = {
		[MUSB_BUS_RESET]      = UDC_EVT_RESET,
		[MUSB_BUS_SUSPEND]    = UDC_EVT_SUSPEND,
		[MUSB_BUS_RESUME]     = UDC_EVT_RESUME,
		[MUSB_BUS_DISCONNECT] = UDC_EVT_VBUS_REMOVED,
	};

	if (event == MUSB_BUS_RESET || event == MUSB_BUS_DISCONNECT) {
		LOG_DBG("bus_event: %s", names[event]);
	}
	if (event < ARRAY_SIZE(map) && map[event])
		udc_submit_event(musb->user_data, map[event], 0);
}

static const struct musb_callbacks udc_musb_callbacks = {
	.ep0_setup   = udc_musb_ep0_setup_cb,
	.ep_tx_done  = udc_musb_tx_done_cb,
	.ep_rx_ready = udc_musb_rx_ready_cb,
	.bus_event   = udc_musb_bus_event_cb,
};

/* =========================================================================
 * ISR
 * ========================================================================= */

static void udc_musb_isr(const void *arg)
{
	LOG_DBG("ISR triggered");
	musb_irq_handler(&((struct udc_musb_data *)
			   udc_get_private(arg))->musb);
}

/* =========================================================================
 * UDC API
 * ========================================================================= */

static int udc_musb_init(const struct device *dev)
{
	struct udc_musb_data *priv = udc_get_private(dev);
	struct musb_dev *musb = &priv->musb;
	int ret;

	if (musb->ops && musb->ops->init)
	{
		ret = musb->ops->init(dev);
		if (ret) {
			LOG_ERR("Failed to init");
			return ret;
		}
	}
	/*
	 * musb_dev and PHY were already set up in driver_preinit.
	 * Only runtime init here: global reset, FIFO, EP, ISR.
	 */
	ret = musb_core_init(musb);
	if (ret) {
		LOG_ERR("core_init failed: %d", ret);
		return ret;
	}
	LOG_DBG("core reset done");

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				     USB_EP_TYPE_CONTROL, 64, 0);
	if (ret) {
		LOG_ERR("EP0 OUT enable failed: %d", ret);
		return ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				     USB_EP_TYPE_CONTROL, 64, 0);
	if (ret) {
		LOG_ERR("EP0 IN enable failed: %d", ret);
		return ret;
	}

	((const struct udc_musb_config *)dev->config)->irq_config(dev);
	LOG_DBG("init done, IRQ connected");
	return 0;
}

static int udc_musb_enable(const struct device *dev)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;
	uint16_t epmask = musb->config->epmask;
	uint8_t power;

	/*
	 * Align with Linux musb_init_controller + musb_start():
	 *   0. Reset controller state (DEVCTL=0, flush pending IRQs)
	 *   1. Enable all interrupts (INTRUSBE=0xf7, INTRTXE=epmask,
	 *      INTRRXE=epmask & ~BIT(0))
	 *   2. Write TESTMODE=0
	 *   3. Write POWER = ISOUPDATE | (HSENAB if !DISABLE_HS)
	 *   4. Platform enable
	 *   5. musb_pullup (SOFTCONN) + DEVCTL SESSION
	 */

	/* Step 0: Reset controller state to clear any stale conditions */
	musb_writeb(musb, MUSB_DEVCTL, 0);
	musb_writeb(musb, MUSB_POWER, 0);
	/* Flush any pending interrupts */
	musb_writeb(musb, MUSB_INTRUSB, 0xff);
	musb_writew(musb, MUSB_INTRTX, 0xffff);
	musb_writew(musb, MUSB_INTRRX, 0xffff);

	/* Step 1: Enable interrupts */
	musb_writeb(musb, MUSB_INTRUSBE, 0xf7);  /* All USB events */
	musb_writew(musb, MUSB_INTRTXE, epmask);  /* All available endpoints */
	musb_writew(musb, MUSB_INTRRXE, epmask & ~BIT(0)); /* All except EP0 (bidir shared) */

	/* Step 2: Clear TESTMODE */
	musb_writeb(musb, MUSB_TESTMODE, 0);

	/* Step 3: Set POWER */
	power = MUSB_POWER_ISOUPDATE;
	if (!(musb->config->quirks & MUSB_DISABLE_HS)) {
		power |= MUSB_POWER_HSENAB;
	}
	musb_writeb(musb, MUSB_POWER, power);

	/* Step 4: Platform enable */
	if (musb->ops->enable)
		musb->ops->enable(dev);

	/* Step 5: Connect (SOFTCONN + Session) */
	musb_connect(musb);

	LOG_INF("enable: POWER=0x%02x DEVCTL=0x%02x INTRUSBE=0x%02x INTRTXE=0x%04x",
		musb_readb(musb, MUSB_POWER),
		musb_readb(musb, MUSB_DEVCTL),
		musb_readb(musb, MUSB_INTRUSBE),
		musb_readw(musb, MUSB_INTRTXE));
	return 0;
}

static int udc_musb_disable(const struct device *dev)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;

	musb_disconnect(musb);
	musb_writeb(musb, MUSB_INTRUSBE, 0);
	musb_writew(musb, MUSB_INTRTXE, 0);
	musb_writew(musb, MUSB_INTRRXE, 0);

	if (musb->ops->disable)
		musb->ops->disable(dev);
	return 0;
}

static int udc_musb_shutdown(const struct device *dev)
{
	udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
	udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
	return 0;
}

static int udc_musb_ep_enqueue(const struct device *dev,
			       struct udc_ep_config *const cfg,
			       struct net_buf *const buf)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;
	uint8_t ep = USB_EP_GET_IDX(cfg->addr);

	udc_buf_put(cfg, buf);
	if (cfg->stat.halted)
		return 0;

	if (USB_EP_DIR_IS_IN(cfg->addr)) {
		if (ep == 0) {
			LOG_DBG("enqueue EP0 IN len=%u", buf->len);
		}
		return musb_ep_tx(musb, ep, buf->data, buf->len);
	} else {
		return musb_ep_rx(musb, ep);
	}
}

static int udc_musb_ep_dequeue(const struct device *dev,
			       struct udc_ep_config *const cfg)
{
	unsigned int key = irq_lock();

	udc_ep_cancel_queued(dev, cfg);
	irq_unlock(key);
	return 0;
}

static int udc_musb_ep_enable(const struct device *dev,
			      struct udc_ep_config *const cfg)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;

	return musb_ep_enable(musb, USB_EP_GET_IDX(cfg->addr),
			      cfg->attributes, cfg->mps);
}

static int udc_musb_ep_disable(const struct device *dev,
			       struct udc_ep_config *const cfg)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;

	return musb_ep_disable(musb, USB_EP_GET_IDX(cfg->addr));
}

static int udc_musb_ep_set_halt(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;
	uint8_t ep = USB_EP_GET_IDX(cfg->addr);

	if (ep != 0)
		cfg->stat.halted = true;
	return musb_ep_set_halt(musb, ep, true);
}

static int udc_musb_ep_clear_halt(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;

	cfg->stat.halted = false;
	return musb_ep_set_halt(musb, USB_EP_GET_IDX(cfg->addr), false);
}

static int udc_musb_set_address(const struct device *dev, uint8_t addr)
{
	struct musb_dev *musb = &((struct udc_musb_data *)
				  udc_get_private(dev))->musb;

	/*
	 * Defer FADDR write to the STATUSIN phase, matching Linux.
	 * musb->set_address flag is checked in musb_handle_ep0()
	 * at STATUSIN and musb_writeb(FADDR) is done there.
	 *
	 * If the UDC stack asserts addr_before_status capability,
	 * we can write FADDR immediately.  But for correctness and
	 * alignment with Linux, we use the deferred approach.
	 */
	musb->set_address = true;
	musb->address = addr;

	LOG_DBG("SET_ADDRESS: addr=%u deferred to status phase", addr);
	return 0;
}

static int udc_musb_host_wakeup(const struct device *dev)
{
	return 0;
}

static enum udc_bus_speed udc_musb_device_speed(const struct device *dev)
{
	uint8_t power = musb_readb(&((struct udc_musb_data *)
				     udc_get_private(dev))->musb, MUSB_POWER);

	return (power & MUSB_POWER_HSMODE) ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static void udc_musb_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_musb_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api udc_musb_api = {
	.lock           = udc_musb_lock,
	.unlock         = udc_musb_unlock,
	.init           = udc_musb_init,
	.enable         = udc_musb_enable,
	.disable        = udc_musb_disable,
	.shutdown       = udc_musb_shutdown,
	.set_address    = udc_musb_set_address,
	.host_wakeup    = udc_musb_host_wakeup,
	.ep_try_config  = NULL,
	.ep_enable      = udc_musb_ep_enable,
	.ep_disable     = udc_musb_ep_disable,
	.ep_set_halt    = udc_musb_ep_set_halt,
	.ep_clear_halt  = udc_musb_ep_clear_halt,
	.ep_enqueue     = udc_musb_ep_enqueue,
	.ep_dequeue     = udc_musb_ep_dequeue,
	.device_speed   = udc_musb_device_speed,
};

#define UDC_MUSB_DEVICE_DEFINE(n)                                            \
	static struct udc_ep_config ep_cfg_out_##n[MUSB_MAX_EPS];            \
	static struct udc_ep_config ep_cfg_in_##n[MUSB_MAX_EPS];             \
									       \
	static void musb_irq_config_##n(const struct device *dev)            \
	{                                                                    \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),      \
			    udc_musb_isr, DEVICE_DT_INST_GET(n), 0);        \
		irq_enable(DT_INST_IRQN(n));                                 \
	}                                                                    \
									       \
	static const struct udc_musb_config musb_config_##n = {              \
		.base = DT_INST_REG_ADDR(n),                                 \
		.irq_config = musb_irq_config_##n,                           \
	};                                                                   \
									       \
	static struct udc_musb_data musb_priv_##n = { \
		.musb = { \
			.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
			.rst_dev = DEVICE_DT_GET(DT_INST_RESET_CTLR(n)), \
		}, \
	}; \
									       \
	static int udc_musb_driver_preinit_##n(const struct device *dev)     \
	{                                                                    \
		struct udc_data *data = dev->data;                           \
		uint16_t mps = 512;                                          \
		int err;                                                     \
		struct musb_dev *m; \
									       							\
		const struct musb_glue_info *glue;                           \
		err = musb_platform_info_get(&glue);                         \
		if (err || !glue || !glue->ops) {                            \
			LOG_ERR("No MUSB glue registered");                  \
			return -ENODEV;                                      \
		}                                                            \
		uint8_t num_eps = glue->config                               \
			  ? glue->config->num_eps : 5;              		\
		/* Bind musb_dev and init PHY at POST_KERNEL */              \
		m = &musb_priv_##n.musb;            \
		m->ops       = glue->ops;                            \
		m->config    = glue->config;                         \
		m->base      = musb_config_##n.base;                 \
		m->hw_eps    = musb_priv_##n.hw_eps;                 \
		m->callbacks = &udc_musb_callbacks;                  \
		m->glue_dev  = (void *)dev;                          \
		m->phy_dev   = USB_SUNXI_PHY_PSEUDODEV_GET_OR_NULL( \
				       DT_NODELABEL(usbotg));          \
		m->user_data = (void *)dev;                          \
									       \
		k_mutex_init(&data->mutex);                                  \
		k_work_init(&musb_priv_##n.ep0_setup_work,                   \
							udc_musb_ep0_setup_work);                         \
		data->priv = &musb_priv_##n;                                 \
		data->caps.rwup = true;                                      \
		data->caps.mps0 = UDC_MPS0_64;                               \
			data->caps.addr_before_status = true;                                      \
		data->caps.hs = true;                                        \
									       \
		for (uint8_t i = 0; i < num_eps; i++) {                      \
			ep_cfg_out_##n[i].caps.out = 1;                      \
			if (i == 0) {                                        \
				ep_cfg_out_##n[i].caps.control = 1;          \
				ep_cfg_out_##n[i].caps.mps = 64;             \
			} else {                                             \
				ep_cfg_out_##n[i].caps.bulk = 1;             \
				ep_cfg_out_##n[i].caps.interrupt = 1;        \
				ep_cfg_out_##n[i].caps.mps = mps;            \
			}                                                    \
			ep_cfg_out_##n[i].addr =                             \
				USB_EP_DIR_OUT | i;                          \
			err = udc_register_ep(dev, &ep_cfg_out_##n[i]);      \
			if (err) {                                           \
				LOG_ERR("reg ep 0x%02x fail: %d",            \
					ep_cfg_out_##n[i].addr, err);        \
				return err;                                  \
			}                                                    \
		}                                                            \
									       \
		for (uint8_t i = 0; i < num_eps; i++) {                      \
			ep_cfg_in_##n[i].caps.in = 1;                        \
			if (i == 0) {                                        \
				ep_cfg_in_##n[i].caps.control = 1;           \
				ep_cfg_in_##n[i].caps.mps = 64;              \
			} else {                                             \
				ep_cfg_in_##n[i].caps.bulk = 1;              \
				ep_cfg_in_##n[i].caps.interrupt = 1;         \
				ep_cfg_in_##n[i].caps.mps = mps;             \
			}                                                    \
			ep_cfg_in_##n[i].addr =                              \
				USB_EP_DIR_IN | i;                           \
			err = udc_register_ep(dev, &ep_cfg_in_##n[i]);       \
			if (err) {                                           \
				LOG_ERR("reg ep 0x%02x fail: %d",            \
					ep_cfg_in_##n[i].addr, err);         \
				return err;                                  \
			}                                                    \
		}                                                            \
									       \
		LOG_INF("MUSB UDC preinit done, %u eps", num_eps);           \
		return 0;                                                    \
	}                                                                    \
									       \
	DEVICE_DT_INST_DEFINE(n, udc_musb_driver_preinit_##n, NULL,          \
			      &musb_priv_##n, &musb_config_##n,              \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			      &udc_musb_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_MUSB_DEVICE_DEFINE)
