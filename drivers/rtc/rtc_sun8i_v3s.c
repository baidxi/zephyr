/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTC driver for Allwinner SUN8I V3s SoC (sun6i-RTC compatible).
 * Reference: Allwinner V3s User Manual §4.12, Linux rtc-sun6i.c
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rtc_sun8i_v3s, CONFIG_RTC_LOG_LEVEL);

/* ========== Register offsets ========== */

/* Low-power oscillator control */
#define SUN8I_LOSC_CTRL             0x00
#define SUN8I_LOSC_CTRL_KEY         (0x16AA << 16)
#define SUN8I_LOSC_CTRL_AUTO_SWT    BIT(14)
#define SUN8I_LOSC_CTRL_ALM_ACC     BIT(9)
#define SUN8I_LOSC_CTRL_HMS_ACC     BIT(8)
#define SUN8I_LOSC_CTRL_YMD_ACC     BIT(7)
#define SUN8I_LOSC_CTRL_ACC_MASK    GENMASK(9, 7)
#define SUN8I_LOSC_CTRL_SRC_SEL     BIT(0)

/* RTC date and time */
#define SUN8I_RTC_YMD               0x10
#define SUN8I_RTC_HMS               0x14

/* Alarm 0 (countdown) */
#define SUN8I_ALRM_COUNTER          0x20
#define SUN8I_ALRM_CUR_VAL          0x24
#define SUN8I_ALRM_EN               0x28
#define SUN8I_ALRM_EN_CNT_EN        BIT(0)
#define SUN8I_ALRM_IRQ_EN           0x2C
#define SUN8I_ALRM_IRQ_EN_CNT_IRQ   BIT(0)
#define SUN8I_ALRM_IRQ_STA          0x30
#define SUN8I_ALRM_IRQ_STA_PEND     BIT(0)

/* Alarm config */
#define SUN8I_ALARM_CONFIG          0x50
#define SUN8I_ALARM_CONFIG_WAKEUP   BIT(0)

/* ========== YMD bit fields ========== */
#define SUN8I_YMD_LEAP              BIT(22)
#define SUN8I_YMD_DAY_MASK          0x1F
#define SUN8I_YMD_MON_SHIFT         8
#define SUN8I_YMD_MON_MASK          0x0F
#define SUN8I_YMD_YEAR_SHIFT        16
#define SUN8I_YMD_YEAR_MASK         0x3F

/* ========== HMS bit fields ========== */
#define SUN8I_HMS_SEC_MASK          0x3F
#define SUN8I_HMS_MIN_SHIFT         8
#define SUN8I_HMS_MIN_MASK          0x3F
#define SUN8I_HMS_HOUR_SHIFT        16
#define SUN8I_HMS_HOUR_MASK         0x1F
#define SUN8I_HMS_WEEK_SHIFT        29
#define SUN8I_HMS_WEEK_MASK         0x07

/* ========== Year range ========== */
#define SUN8I_YEAR_MIN              1970
#define SUN8I_YEAR_MAX              2033
#define SUN8I_YEAR_OFF              (SUN8I_YEAR_MIN - 1900)

/* ========== Timeout for ACCE bits ========== */
#define SUN8I_ACCE_TIMEOUT_MS       50

/* ========== Helper macros ========== */
#define SUN8I_DATE_GET_DAY(x)       ((x) & SUN8I_YMD_DAY_MASK)
#define SUN8I_DATE_GET_MON(x)       (((x) >> SUN8I_YMD_MON_SHIFT) & SUN8I_YMD_MON_MASK)
#define SUN8I_DATE_GET_YEAR(x)      (((x) >> SUN8I_YMD_YEAR_SHIFT) & SUN8I_YMD_YEAR_MASK)

#define SUN8I_TIME_GET_SEC(x)       ((x) & SUN8I_HMS_SEC_MASK)
#define SUN8I_TIME_GET_MIN(x)       (((x) >> SUN8I_HMS_MIN_SHIFT) & SUN8I_HMS_MIN_MASK)
#define SUN8I_TIME_GET_HOUR(x)      (((x) >> SUN8I_HMS_HOUR_SHIFT) & SUN8I_HMS_HOUR_MASK)

#define SUN8I_DATE_SET_DAY(x)       ((x) & SUN8I_YMD_DAY_MASK)
#define SUN8I_DATE_SET_MON(x)       (((x) & SUN8I_YMD_MON_MASK) << SUN8I_YMD_MON_SHIFT)
#define SUN8I_DATE_SET_YEAR(x)      (((x) & SUN8I_YMD_YEAR_MASK) << SUN8I_YMD_YEAR_SHIFT)

#define SUN8I_TIME_SET_SEC(x)       ((x) & SUN8I_HMS_SEC_MASK)
#define SUN8I_TIME_SET_MIN(x)       (((x) & SUN8I_HMS_MIN_MASK) << SUN8I_HMS_MIN_SHIFT)
#define SUN8I_TIME_SET_HOUR(x)      (((x) & SUN8I_HMS_HOUR_MASK) << SUN8I_HMS_HOUR_SHIFT)

/* ========== Seconds per day ========== */
#define SECS_PER_DAY                (24ULL * 3600ULL)
#define SECS_PER_HOUR               3600ULL
#define SECS_PER_MIN                60ULL

/* ========== Device data ========== */
struct rtc_sun8i_v3s_data {
	struct k_spinlock lock;
#ifdef CONFIG_RTC_ALARM
	bool alarm_pending;
	rtc_alarm_callback alarm_cb;
	void *alarm_cb_data;
#endif
};

/* ========== Device config ========== */
struct rtc_sun8i_v3s_config {
	mm_reg_t base;
};

/* ========== MMIO helpers ========== */

static inline uint32_t sun8i_rtc_read(const struct device *dev, uint32_t offset)
{
	const struct rtc_sun8i_v3s_config *config = dev->config;

	return sys_read32(config->base + offset);
}

static inline void sun8i_rtc_write(const struct device *dev, uint32_t offset, uint32_t value)
{
	const struct rtc_sun8i_v3s_config *config = dev->config;

	sys_write32(value, config->base + offset);
}

/* ========== Internal helpers ========== */

/**
 * Wait for specific ACCE bits in LOSC_CTRL to clear.
 * Returns 0 on success, -ETIMEDOUT on timeout.
 */
static int sun8i_rtc_wait_acce(const struct device *dev, uint32_t mask, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	uint32_t val;

	do {
		val = sun8i_rtc_read(dev, SUN8I_LOSC_CTRL) & mask;
		if (val == 0) {
			return 0;
		}
	} while (k_uptime_get() < deadline);

	return -ETIMEDOUT;
}

/**
 * Check if a year is a leap year.
 */
static bool sun8i_is_leap_year(int year)
{
	return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

/**
 * Validate rtc_time fields.
 */
static bool sun8i_rtc_validate_time(const struct rtc_time *timeptr)
{
	int year;

	if (timeptr->tm_sec < 0 || timeptr->tm_sec > 59) {
		return false;
	}
	if (timeptr->tm_min < 0 || timeptr->tm_min > 59) {
		return false;
	}
	if (timeptr->tm_hour < 0 || timeptr->tm_hour > 23) {
		return false;
	}
	if (timeptr->tm_mday < 1 || timeptr->tm_mday > 31) {
		return false;
	}
	if (timeptr->tm_mon < 0 || timeptr->tm_mon > 11) {
		return false;
	}

	year = timeptr->tm_year + 1900;
	if (year < SUN8I_YEAR_MIN || year > SUN8I_YEAR_MAX) {
		return false;
	}

	return true;
}

/* ========== RTC API implementation ========== */

static int rtc_sun8i_v3s_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	struct rtc_sun8i_v3s_data *data = dev->data;
	int ret;
	uint32_t date, time_val;
	int year;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (!sun8i_rtc_validate_time(timeptr)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for all ACCE bits to clear */
	ret = sun8i_rtc_wait_acce(dev, SUN8I_LOSC_CTRL_ACC_MASK, SUN8I_ACCE_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("RTC busy, ACCE bits not cleared");
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	/* Encode HMS */
	time_val = SUN8I_TIME_SET_SEC(timeptr->tm_sec) |
		   SUN8I_TIME_SET_MIN(timeptr->tm_min) |
		   SUN8I_TIME_SET_HOUR(timeptr->tm_hour);

	/* Write HMS */
	sun8i_rtc_write(dev, SUN8I_RTC_HMS, time_val);

	/* Wait for HMS ACC to clear */
	ret = sun8i_rtc_wait_acce(dev, SUN8I_LOSC_CTRL_HMS_ACC, SUN8I_ACCE_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for HMS ACC to clear");
		k_spin_unlock(&data->lock, key);
		return -ETIMEDOUT;
	}

	/* Encode YMD */
	year = timeptr->tm_year + 1900 - SUN8I_YEAR_MIN;

	date = SUN8I_DATE_SET_DAY(timeptr->tm_mday) |
	       SUN8I_DATE_SET_MON(timeptr->tm_mon + 1) |
	       SUN8I_DATE_SET_YEAR(year);

	/* Set leap year flag (software-managed) */
	if (sun8i_is_leap_year(timeptr->tm_year + 1900)) {
		date |= SUN8I_YMD_LEAP;
	}

	/* Write YMD */
	sun8i_rtc_write(dev, SUN8I_RTC_YMD, date);

	/* Wait for YMD ACC to clear */
	ret = sun8i_rtc_wait_acce(dev, SUN8I_LOSC_CTRL_YMD_ACC, SUN8I_ACCE_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for YMD ACC to clear");
		k_spin_unlock(&data->lock, key);
		return -ETIMEDOUT;
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int rtc_sun8i_v3s_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct rtc_sun8i_v3s_data *data = dev->data;
	uint32_t date, time_val, date2, time2;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/*
	 * Read YMD and HMS repeatedly until two consecutive reads
	 * return the same values (prevent cross-second reads).
	 */
	do {
		date = sun8i_rtc_read(dev, SUN8I_RTC_YMD);
		time_val = sun8i_rtc_read(dev, SUN8I_RTC_HMS);
		date2 = sun8i_rtc_read(dev, SUN8I_RTC_YMD);
		time2 = sun8i_rtc_read(dev, SUN8I_RTC_HMS);
	} while (date != date2 || time_val != time2);

	k_spin_unlock(&data->lock, key);

	/* Decode YMD */
	timeptr->tm_mday = SUN8I_DATE_GET_DAY(date);
	timeptr->tm_mon = SUN8I_DATE_GET_MON(date) - 1;
	timeptr->tm_year = SUN8I_DATE_GET_YEAR(date) + SUN8I_YEAR_OFF;

	/* Decode HMS */
	timeptr->tm_sec = SUN8I_TIME_GET_SEC(time_val);
	timeptr->tm_min = SUN8I_TIME_GET_MIN(time_val);
	timeptr->tm_hour = SUN8I_TIME_GET_HOUR(time_val);

	/* Fields not provided by hardware */
	timeptr->tm_nsec = 0;
	timeptr->tm_wday = -1;
	timeptr->tm_yday = -1;

	/* Validate result */
	if (!sun8i_rtc_validate_time(timeptr)) {
		return -ENODATA;
	}

	return 0;
}

#ifdef CONFIG_RTC_ALARM

static int rtc_sun8i_v3s_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						     uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != 0) {
		return -EINVAL;
	}

	/*
	 * Alarm 0 is a countdown alarm: it counts down in seconds from
	 * the value written to ALARM0_COUNTER. We support setting alarm
	 * by HOUR, MINUTE, SECOND which get converted to total seconds.
	 */
	*mask = RTC_ALARM_TIME_MASK_SECOND |
		RTC_ALARM_TIME_MASK_MINUTE |
		RTC_ALARM_TIME_MASK_HOUR;

	return 0;
}

static int rtc_sun8i_v3s_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
					 const struct rtc_time *timeptr)
{
	struct rtc_sun8i_v3s_data *data = dev->data;
	int ret;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (id != 0) {
		ret = -EINVAL;
		goto out;
	}

	if ((mask > 0) && (timeptr == NULL)) {
		ret = -EINVAL;
		goto out;
	}

	/* Validate alarm fields */
	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		if (timeptr->tm_sec < 0 || timeptr->tm_sec > 59) {
			ret = -EINVAL;
			goto out;
		}
	}
	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		if (timeptr->tm_min < 0 || timeptr->tm_min > 59) {
			ret = -EINVAL;
			goto out;
		}
	}
	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		if (timeptr->tm_hour < 0 || timeptr->tm_hour > 23) {
			ret = -EINVAL;
			goto out;
		}
	}

	/* Disable alarm first */
	sun8i_rtc_write(dev, SUN8I_ALRM_EN, 0);
	sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_EN, 0);
	sun8i_rtc_write(dev, SUN8I_ALARM_CONFIG, 0);

	/* Clear any pending interrupt */
	sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_STA, SUN8I_ALRM_IRQ_STA_PEND);

	/* Write 0 to counter and wait */
	sun8i_rtc_write(dev, SUN8I_ALRM_COUNTER, 0);
	k_busy_wait(200);

	if (mask == 0) {
		/* No fields to set - alarm disabled */
		data->alarm_pending = false;
		ret = 0;
		goto out;
	}

	/*
	 * Alarm 0 is a countdown alarm: calculate total seconds from
	 * the masked fields and write to ALARM0_COUNTER.
	 */
	uint32_t counter_val = 0;

	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		counter_val += timeptr->tm_hour * 3600;
	}
	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		counter_val += timeptr->tm_min * 60;
	}
	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		counter_val += timeptr->tm_sec;
	}

	if (counter_val == 0) {
		/* 0 means alarm fires in 1 second (hardware counts 0 as 1) */
		counter_val = 1;
	}

	sun8i_rtc_write(dev, SUN8I_ALRM_COUNTER, counter_val);

	/* Enable alarm counter and interrupt */
	sun8i_rtc_write(dev, SUN8I_ALRM_EN, SUN8I_ALRM_EN_CNT_EN);
	sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_EN, SUN8I_ALRM_IRQ_EN_CNT_IRQ);
	sun8i_rtc_write(dev, SUN8I_ALARM_CONFIG, SUN8I_ALARM_CONFIG_WAKEUP);

	data->alarm_pending = false;
	ret = 0;

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int rtc_sun8i_v3s_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
					 struct rtc_time *timeptr)
{
	struct rtc_sun8i_v3s_data *data = dev->data;
	uint32_t counter;

	if (id != 0) {
		return -EINVAL;
	}

	if (timeptr == NULL || mask == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	counter = sun8i_rtc_read(dev, SUN8I_ALRM_COUNTER);

	k_spin_unlock(&data->lock, key);

	/* Decode counter back to HMS */
	memset(timeptr, 0, sizeof(*timeptr));
	*mask = 0;

	if (counter > 0) {
		timeptr->tm_hour = counter / 3600;
		counter %= 3600;
		timeptr->tm_min = counter / 60;
		timeptr->tm_sec = counter % 60;

		*mask = RTC_ALARM_TIME_MASK_SECOND |
			RTC_ALARM_TIME_MASK_MINUTE |
			RTC_ALARM_TIME_MASK_HOUR;
	}

	return 0;
}

static int rtc_sun8i_v3s_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_sun8i_v3s_data *data = dev->data;
	int ret;

	if (id != 0) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	ret = data->alarm_pending ? 1 : 0;
	data->alarm_pending = false;

	k_spin_unlock(&data->lock, key);
	return ret;
}

static int rtc_sun8i_v3s_alarm_set_callback(const struct device *dev, uint16_t id,
					     rtc_alarm_callback callback, void *user_data)
{
	struct rtc_sun8i_v3s_data *data = dev->data;

	if (id != 0) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->alarm_cb = callback;
	data->alarm_cb_data = user_data;

	if (callback == NULL) {
		/* Disable alarm interrupt */
		sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_EN, 0);
	} else {
		/* Enable alarm interrupt if counter is set */
		uint32_t counter = sun8i_rtc_read(dev, SUN8I_ALRM_COUNTER);

		if (counter > 0) {
			sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_EN, SUN8I_ALRM_IRQ_EN_CNT_IRQ);
		}
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

#endif /* CONFIG_RTC_ALARM */

/* ========== ISR ========== */

static void rtc_sun8i_v3s_isr(const struct device *dev)
{
	struct rtc_sun8i_v3s_data *data = dev->data;
	uint32_t status;

	ARG_UNUSED(data);

	status = sun8i_rtc_read(dev, SUN8I_ALRM_IRQ_STA);

	if (status & SUN8I_ALRM_IRQ_STA_PEND) {
		/* Clear pending interrupt (write 1 to clear) */
		sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_STA, SUN8I_ALRM_IRQ_STA_PEND);

#ifdef CONFIG_RTC_ALARM
		if (data->alarm_cb != NULL) {
			data->alarm_cb(dev, 0, data->alarm_cb_data);
		} else {
			data->alarm_pending = true;
		}
#endif
	}
}

/* ========== Driver API struct ========== */

static DEVICE_API(rtc, rtc_sun8i_v3s_driver_api) = {
	.set_time = rtc_sun8i_v3s_set_time,
	.get_time = rtc_sun8i_v3s_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_sun8i_v3s_alarm_get_supported_fields,
	.alarm_set_time = rtc_sun8i_v3s_alarm_set_time,
	.alarm_get_time = rtc_sun8i_v3s_alarm_get_time,
	.alarm_is_pending = rtc_sun8i_v3s_alarm_is_pending,
	.alarm_set_callback = rtc_sun8i_v3s_alarm_set_callback,
#endif
};

/* ========== Init and device instantiation ========== */

static int rtc_sun8i_v3s_init(const struct device *dev)
{
	uint32_t val;

	/*
	 * Configure LOSC_CTRL:
	 * - Select external 32.768KHz crystal (bit 0 = 1)
	 * - Enable auto-switch (bit 14 = 1)
	 * - KEY = 0x16AA to allow writing
	 */
	val = SUN8I_LOSC_CTRL_KEY |
	      SUN8I_LOSC_CTRL_AUTO_SWT |
	      SUN8I_LOSC_CTRL_SRC_SEL;
	sun8i_rtc_write(dev, SUN8I_LOSC_CTRL, val);

	/* Disable alarm and interrupt initially */
	sun8i_rtc_write(dev, SUN8I_ALRM_EN, 0);
	sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_EN, 0);

	/* Clear any pending alarm interrupt */
	sun8i_rtc_write(dev, SUN8I_ALRM_IRQ_STA, SUN8I_ALRM_IRQ_STA_PEND);

	/* Configure IRQ */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    rtc_sun8i_v3s_isr,
		    DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

#define RTC_SUN8I_V3S_INIT(inst)							\
	static struct rtc_sun8i_v3s_data rtc_sun8i_v3s_data_##inst;			\
	static const struct rtc_sun8i_v3s_config rtc_sun8i_v3s_config_##inst = {	\
		.base = DT_INST_REG_ADDR(inst),					\
	};										\
	DEVICE_DT_INST_DEFINE(inst,							\
			      &rtc_sun8i_v3s_init,					\
			      NULL,							\
			      &rtc_sun8i_v3s_data_##inst,				\
			      &rtc_sun8i_v3s_config_##inst,				\
			      POST_KERNEL,						\
			      CONFIG_RTC_INIT_PRIORITY,				\
			      &rtc_sun8i_v3s_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_SUN8I_V3S_INIT)
