/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner sun8i V3s TWI (I2C) driver
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_i2c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_sun8i_v3s);

#include "i2c-priv.h"

/* Register offsets (ref: v3s_twi.pdf section 8.1.4) */
#define TWI_ADDR	0x00	/* Slave address */
#define TWI_XADDR	0x04	/* Extended slave address */
#define TWI_DATA	0x08	/* Data */
#define TWI_CNTR	0x0C	/* Control */
#define TWI_STAT	0x10	/* Status */
#define TWI_CCR		0x14	/* Clock control */
#define TWI_SRST	0x18	/* Software reset */
#define TWI_EFR		0x1C	/* Enhance feature */
#define TWI_LCR		0x20	/* Line control */

/* TWI_CNTR bits (section 8.1.5.4) */
#define TWI_CNTR_INT_EN		BIT(7)
#define TWI_CNTR_BUS_EN		BIT(6)
#define TWI_CNTR_M_STA		BIT(5)
#define TWI_CNTR_M_STP		BIT(4)
#define TWI_CNTR_INT_FLAG	BIT(3)
#define TWI_CNTR_A_ACK		BIT(2)

/* TWI_STAT codes (section 8.1.5.5) */
#define TWI_STAT_START		0x08
#define TWI_STAT_RESTART	0x10
#define TWI_STAT_ADDR_W_ACK	0x18
#define TWI_STAT_ADDR_W_NACK	0x20
#define TWI_STAT_DATA_W_ACK	0x28
#define TWI_STAT_DATA_W_NACK	0x30
#define TWI_STAT_ARB_LOST	0x38
#define TWI_STAT_ADDR_R_ACK	0x40
#define TWI_STAT_ADDR_R_NACK	0x48
#define TWI_STAT_DATA_R_ACK	0x50
#define TWI_STAT_DATA_R_NACK	0x58
#define TWI_STAT_BUS_ERROR	0x00
#define TWI_STAT_IDLE		0xF8

/* TWI_CCR bit fields (section 8.1.5.6) */
#define TWI_CCR_CLK_M_SHIFT	3
#define TWI_CCR_CLK_M_MASK	GENMASK(6, 3)
#define TWI_CCR_CLK_N_SHIFT	0
#define TWI_CCR_CLK_N_MASK	GENMASK(2, 0)

/* TWI_SRST (section 8.1.5.7) */
#define TWI_SRST_SOFT_RST	BIT(0)

/* TWI_EFR (section 8.1.5.8) */
#define TWI_EFR_DBN_MASK	GENMASK(1, 0)

/* APB bus clock input = 24 MHz */
#define TWI_APB_FREQ		24000000U

/*
 * The base CNTR value for master operation: bus enabled, interrupt enabled.
 * M_STA, M_STP, INT_FLAG, A_ACK are set per-operation.
 */
#define TWI_CNTR_BASE		(TWI_CNTR_BUS_EN | TWI_CNTR_INT_EN)

struct i2c_sun8i_v3s_config {
	uintptr_t phys_base;
	size_t reg_size;
	const struct device *clock_dev;
	clock_control_subsys_t clk_bus;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pincfg;
	uint32_t bus_freq;
};

struct i2c_sun8i_v3s_data {
	mm_reg_t base;
	struct k_sem sync_sem;
	uint32_t dev_config;
};

static inline uint32_t twi_read32(const struct i2c_sun8i_v3s_data *data,
				  uint32_t offset)
{
	return sys_read32(data->base + offset);
}

static inline void twi_write32(const struct i2c_sun8i_v3s_data *data,
			       uint32_t offset, uint32_t val)
{
	sys_write32(val, data->base + offset);
}

/*
 * Poll for INT_FLAG to be set. Returns 0 on success, -ETIMEDOUT on timeout.
 */
static int twi_wait_int_flag(const struct i2c_sun8i_v3s_data *data)
{
	uint32_t timeout = 1000000;

	while (timeout--) {
		if (twi_read32(data, TWI_CNTR) & TWI_CNTR_INT_FLAG) {
			return 0;
		}
	}
	LOG_ERR("Timeout waiting for INT_FLAG");
	return -ETIMEDOUT;
}

/*
 * Calculate CLK_M and CLK_N from APB frequency and target I2C bus speed.
 *
 * Formula: Foscl = Fin / (2^CLK_N * (CLK_M + 1) * 10)
 *
 * Searches all valid N (0..7) and M (0..15) for the closest match.
 */
static int twi_calc_ccr(uint32_t fin, uint32_t target, uint32_t *ccr)
{
	uint32_t best_diff = UINT32_MAX;
	uint32_t best_m = 0, best_n = 0;
	uint32_t best_actual = 0;

	if (target == 0) {
		return -EINVAL;
	}

	for (int n = 0; n <= 7; n++) {
		uint32_t div_n = 1U << n;

		for (int m = 0; m <= 15; m++) {
			uint32_t div = div_n * (m + 1) * 10;

			if (div == 0) {
				continue;
			}

			uint32_t actual = fin / div;

			if (actual == 0) {
				continue;
			}

			uint32_t diff = (actual > target) ?
				(actual - target) : (target - actual);

			if (diff < best_diff) {
				best_diff = diff;
				best_actual = actual;
				best_m = m;
				best_n = n;
				if (diff == 0) {
					goto done;
				}
			}
		}
	}

done:
	if (best_actual == 0) {
		return -EINVAL;
	}

	*ccr = (best_m << TWI_CCR_CLK_M_SHIFT) |
	       (best_n << TWI_CCR_CLK_N_SHIFT);
	LOG_DBG("CCR: target=%u actual=%u M=%u N=%u", target, best_actual,
		best_m, best_n);
	return 0;
}

static int twi_soft_reset(const struct i2c_sun8i_v3s_data *data)
{
	uint32_t timeout = 1000;

	twi_write32(data, TWI_SRST, TWI_SRST_SOFT_RST);
	while (timeout--) {
		if (!(twi_read32(data, TWI_SRST) & TWI_SRST_SOFT_RST)) {
			return 0;
		}
	}
	LOG_ERR("Soft reset timeout");
	return -ETIMEDOUT;
}

static int i2c_sun8i_v3s_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_sun8i_v3s_data *data = dev->data;
	uint32_t i2c_speed;
	uint32_t ccr;
	int ret;

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		i2c_speed = 100000U;
		break;
	case I2C_SPEED_FAST:
		i2c_speed = 400000U;
		break;
	default:
		LOG_ERR("Unsupported I2C speed");
		return -ENOTSUP;
	}

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		LOG_ERR("Only master mode supported");
		return -ENOTSUP;
	}

	if (dev_config & I2C_ADDR_10_BITS) {
		LOG_ERR("10-bit addressing not supported");
		return -ENOTSUP;
	}

	ret = twi_calc_ccr(TWI_APB_FREQ, i2c_speed, &ccr);
	if (ret) {
		return ret;
	}

	twi_write32(data, TWI_CCR, ccr);
	data->dev_config = dev_config;

	return 0;
}

/*
 * Send START condition, wait for status. Returns 0 or -errno.
 */
static int twi_send_start(struct i2c_sun8i_v3s_data *data)
{
	twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_M_STA);

	int ret = twi_wait_int_flag(data);
	if (ret) {
		return ret;
	}

	uint32_t stat = twi_read32(data, TWI_STAT) & 0xFF;

	if (stat != TWI_STAT_START) {
		LOG_ERR("START failed, stat=0x%02x", stat);
		twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_M_STP |
			    TWI_CNTR_INT_FLAG);
		return -EIO;
	}

	return 0;
}

/*
 * Send repeated START condition.
 */
static int twi_send_restart(struct i2c_sun8i_v3s_data *data)
{
	twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_M_STA |
		    TWI_CNTR_INT_FLAG);

	int ret = twi_wait_int_flag(data);
	if (ret) {
		return ret;
	}

	uint32_t stat = twi_read32(data, TWI_STAT) & 0xFF;

	if (stat != TWI_STAT_RESTART) {
		LOG_ERR("RESTART failed, stat=0x%02x", stat);
		twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_M_STP |
			    TWI_CNTR_INT_FLAG);
		return -EIO;
	}

	return 0;
}

/*
 * Send STOP condition. Does not wait for INT_FLAG because IDLE state (0xF8)
 * does not set INT_FLAG.
 */
static void twi_send_stop(struct i2c_sun8i_v3s_data *data)
{
	twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_M_STP |
		    TWI_CNTR_INT_FLAG);
}

/*
 * Send the 7-bit slave address + R/W bit, wait for ACK.
 * Returns 0 on ACK, -EIO on NACK.
 */
static int twi_send_addr(struct i2c_sun8i_v3s_data *data, uint16_t addr,
			 bool is_read)
{
	uint8_t addr_byte = (uint8_t)(addr << 1) | (is_read ? 1 : 0);
	uint8_t expected = is_read ? TWI_STAT_ADDR_R_ACK : TWI_STAT_ADDR_W_ACK;

	twi_write32(data, TWI_DATA, addr_byte);
	twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_INT_FLAG);

	int ret = twi_wait_int_flag(data);
	if (ret) {
		return ret;
	}

	uint32_t stat = twi_read32(data, TWI_STAT) & 0xFF;

	if (stat == TWI_STAT_ADDR_W_NACK || stat == TWI_STAT_ADDR_R_NACK) {
		LOG_DBG("Address NACK: addr=0x%02x", addr);
		twi_send_stop(data);
		return -EIO;
	}

	if (stat != expected) {
		LOG_ERR("Addr unexpected stat=0x%02x expected=0x%02x",
			stat, expected);
		twi_send_stop(data);
		return -EIO;
	}

	return 0;
}

/*
 * Send a single data byte. For the last byte, the caller must send STOP
 * separately after this returns successfully.
 */
static int twi_send_byte(struct i2c_sun8i_v3s_data *data, uint8_t byte)
{
	twi_write32(data, TWI_DATA, byte);
	twi_write32(data, TWI_CNTR, TWI_CNTR_BASE | TWI_CNTR_INT_FLAG);

	int ret = twi_wait_int_flag(data);

	if (ret) {
		return ret;
	}

	uint32_t stat = twi_read32(data, TWI_STAT) & 0xFF;

	if (stat == TWI_STAT_DATA_W_NACK) {
		LOG_DBG("Data NACK");
		twi_send_stop(data);
		return -EIO;
	}

	if (stat != TWI_STAT_DATA_W_ACK) {
		LOG_ERR("Data write unexpected stat=0x%02x", stat);
		twi_send_stop(data);
		return -EIO;
	}

	return 0;
}

/*
 * Receive a single data byte into *byte.
 * On the last byte, send NACK (A_ACK=0). The caller must send STOP separately.
 */
static int twi_recv_byte(struct i2c_sun8i_v3s_data *data, uint8_t *byte,
			 bool last)
{
	uint32_t cntr = TWI_CNTR_BASE | TWI_CNTR_INT_FLAG;

	if (!last) {
		cntr |= TWI_CNTR_A_ACK;
	}
	/* On last byte: A_ACK=0 means NACK */

	twi_write32(data, TWI_CNTR, cntr);

	int ret = twi_wait_int_flag(data);

	if (ret) {
		return ret;
	}

	uint32_t stat = twi_read32(data, TWI_STAT) & 0xFF;

	if (last) {
		if (stat == TWI_STAT_DATA_R_NACK) {
			/* Expected: NACK sent after receiving last byte */
			*byte = twi_read32(data, TWI_DATA) & 0xFF;
			return 0;
		}
		LOG_ERR("Last read unexpected stat=0x%02x", stat);
		*byte = twi_read32(data, TWI_DATA) & 0xFF;
		twi_send_stop(data);
		return -EIO;
	}

	if (stat != TWI_STAT_DATA_R_ACK) {
		LOG_ERR("Data read unexpected stat=0x%02x", stat);
		twi_send_stop(data);
		return -EIO;
	}

	*byte = twi_read32(data, TWI_DATA) & 0xFF;
	return 0;
}

static int i2c_sun8i_v3s_transfer(const struct device *dev,
				  struct i2c_msg *msgs,
				  uint8_t num_msgs,
				  uint16_t addr)
{
	struct i2c_sun8i_v3s_data *data = dev->data;
	int ret;

	if (!num_msgs || !msgs) {
		return -EINVAL;
	}

	k_sem_take(&data->sync_sem, K_FOREVER);

	/* Send START */
	ret = twi_send_start(data);
	if (ret) {
		goto out;
	}

	bool started = true;

	for (int i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool is_read = (msg->flags & I2C_MSG_READ) != 0;

		if (msg->flags & I2C_MSG_ADDR_10_BITS) {
			ret = -ENOTSUP;
			goto stop;
		}

		if (!started) {
			/* Repeated START between messages */
			ret = twi_send_restart(data);
			if (ret) {
				goto out;
			}
		}
		started = false;

		/* Send address + R/W */
		ret = twi_send_addr(data, addr, is_read);
		if (ret) {
			goto out;
		}

		/* Transfer data bytes */
		for (uint32_t j = 0; j < msg->len; j++) {
			bool last = (j == msg->len - 1);

			if (is_read) {
				ret = twi_recv_byte(data, &msg->buf[j], last);
			} else {
				ret = twi_send_byte(data, msg->buf[j]);
			}

			if (ret) {
				goto out;
			}
		}

		/*
		 * Send STOP if this message has STOP flag set
		 * (which the framework guarantees for the last message).
		 */
		if (msg->flags & I2C_MSG_STOP) {
			twi_send_stop(data);
		}
	}

out:
	/*
	 * On error, the bus may be in an unknown state. Send STOP to release it.
	 * This is safe even if STOP was already sent (it's a no-op or auto-clears).
	 */
	if (ret) {
		twi_send_stop(data);
		/* Drain any remaining INT_FLAG from the STOP state */
		twi_read32(data, TWI_STAT);
	}

	k_sem_give(&data->sync_sem);
	return ret;

stop:
	twi_send_stop(data);
	k_sem_give(&data->sync_sem);
	return ret;
}

static int i2c_sun8i_v3s_init(const struct device *dev)
{
	const struct i2c_sun8i_v3s_config *cfg = dev->config;
	struct i2c_sun8i_v3s_data *data = dev->data;
	uint32_t dev_config;
	int ret;

	if (cfg->pincfg) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Failed to apply pinctrl (err %d)", ret);
			return ret;
		}
	}

	device_map(&data->base, cfg->phys_base, cfg->reg_size,
		   K_MEM_CACHE_NONE);

	ret = clock_control_on(cfg->clock_dev, (void *)cfg->clk_bus);
	if (ret) {
		LOG_ERR("Failed to enable bus clock");
		return ret;
	}

	if (!device_is_ready(cfg->reset.dev)) {
		clock_control_off(cfg->clock_dev, (void *)cfg->clk_bus);
		LOG_ERR("Reset controller not ready");
		return -ENODEV;
	}

	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret) {
		LOG_ERR("Failed to deassert reset");
		clock_control_off(cfg->clock_dev, (void *)cfg->clk_bus);
		return ret;
	}

	/* Soft reset the TWI controller */
	ret = twi_soft_reset(data);
	if (ret) {
		clock_control_off(cfg->clock_dev, (void *)cfg->clk_bus);
		return ret;
	}

	/* Disable enhanced features (DBN = 0: no write-after-read bytes) */
	twi_write32(data, TWI_EFR, 0);

	k_sem_init(&data->sync_sem, 1, 1);

	/* Initial configuration */
	dev_config = I2C_MODE_CONTROLLER |
		     i2c_map_dt_bitrate(cfg->bus_freq);
	ret = i2c_sun8i_v3s_configure(dev, dev_config);
	if (ret) {
		LOG_ERR("Failed to configure I2C");
		clock_control_off(cfg->clock_dev, (void *)cfg->clk_bus);
		return ret;
	}

	LOG_DBG("I2C%u ready", 0);
	return 0;
}

static DEVICE_API(i2c, i2c_sun8i_v3s_api) = {
	.configure = i2c_sun8i_v3s_configure,
	.transfer = i2c_sun8i_v3s_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

#define I2C_SUN8I_V3S_INIT(inst)						\
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst)));		\
	static struct i2c_sun8i_v3s_data i2c_sun8i_v3s_data_##inst;		\
	static const struct i2c_sun8i_v3s_config				\
			i2c_sun8i_v3s_config_##inst = {			\
		.phys_base = DT_INST_REG_ADDR(inst),				\
		.reg_size = DT_INST_REG_SIZE(inst),				\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),		\
		.clk_bus = (clock_control_subsys_t)				\
			DT_INST_CLOCKS_CELL_BY_NAME(inst, bus, clk_id),		\
		.reset = RESET_DT_SPEC_INST_GET(inst),				\
		IF_ENABLED(CONFIG_PINCTRL,					\
			(.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),))	\
		.bus_freq = DT_INST_PROP(inst, clock_frequency),		\
	};									\
	I2C_DEVICE_DT_INST_DEFINE(inst,						\
		i2c_sun8i_v3s_init,						\
		NULL,								\
		&i2c_sun8i_v3s_data_##inst,					\
		&i2c_sun8i_v3s_config_##inst,					\
		POST_KERNEL,							\
		CONFIG_I2C_INIT_PRIORITY,					\
		&i2c_sun8i_v3s_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_SUN8I_V3S_INIT)
