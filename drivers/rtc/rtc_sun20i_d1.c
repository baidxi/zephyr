/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTC driver for Allwinner SUN20I D1 / SUN8I T113-S3 SoC.
 *
 * Uses the linear-day counter mode (RTC_LINEAR_DAY in Linux terms):
 *   - RTC_DAY_REG holds a 16-bit day number (UNIX-epoch based)
 *   - RTC_HH_MM_SS_REG holds HOUR/MINUTE/SECOND
 *   - Alarm is a hardware comparator (matches DAY + HMS against current time)
 *
 * Reference:
 *   - T113-S3 User Manual §3.14 (t113-rtc.pdf)
 *   - Linux rtc-sun6i.c (RTC_LINEAR_DAY branch)
 */

#define DT_DRV_COMPAT allwinner_sun20i_d1_rtc

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

LOG_MODULE_REGISTER(rtc_sun20i_d1, CONFIG_RTC_LOG_LEVEL);

/* ========== Register offsets (T113 §3.14.6) ========== */

#define SUN20I_RTC_LOSC_CTRL            0x0000
#define SUN20I_RTC_AUTO_SWT_STA         0x0004
#define SUN20I_RTC_CLK_PRESCAL          0x0008
#define SUN20I_RTC_DAY_REG              0x0010
#define SUN20I_RTC_HH_MM_SS_REG         0x0014
#define SUN20I_RTC_ALRM_DAY_SET         0x0020
#define SUN20I_RTC_ALRM_CUR_VAL         0x0024
#define SUN20I_RTC_ALRM_EN              0x0028
#define SUN20I_RTC_ALRM_IRQ_EN          0x002C
#define SUN20I_RTC_ALRM_IRQ_STA         0x0030
#define SUN20I_RTC_ALARM_CONFIG         0x0050
#define SUN20I_RTC_SPI_CLK_CTRL         0x0310

/* ========== LOSC_CTRL bit definitions (T113 §3.14.6.1) ========== */

#define SUN20I_RTC_LOSC_CTRL_KEY        (0x16AA << 16)
#define SUN20I_RTC_LOSC_CTRL_KEY_MASK   GENMASK(31, 16)

/* bit15: LOSC_AUTO_SWT_FUNCTION (0=Enable, 1=Disable auto-switch) */
#define SUN20I_RTC_LOSC_CTRL_AUTO_SWT_FUNC   BIT(15)

/* bit14: LOSC_AUTO_SWT_32K_SEL_EN */
#define SUN20I_RTC_LOSC_CTRL_AUTO_SWT_EN     BIT(14)

/* bit8: RTC_HHMMSS_ACCE (write indicator for HMS register) */
#define SUN20I_RTC_LOSC_CTRL_HMS_ACC         BIT(8)

/* bit7: RTC_DAY_ACCE (write indicator for DAY register) */
#define SUN20I_RTC_LOSC_CTRL_DAY_ACC         BIT(7)

/* bit[9:7]: all ACCE bits mask */
#define SUN20I_RTC_LOSC_CTRL_ACC_MASK        GENMASK(9, 7)

/* bit4: EXT_LOSC_EN */
#define SUN20I_RTC_LOSC_CTRL_EXT_LOSC_EN     BIT(4)

/* bit1: RTC_SRC_SEL (0=LOSC, 1=24MDIV32K) */
#define SUN20I_RTC_LOSC_CTRL_RTC_SRC_SEL     BIT(1)

/* bit0: LOSC_SRC_SEL (0=RC16M, 1=EXT32K) */
#define SUN20I_RTC_LOSC_CTRL_SRC_SEL         BIT(0)

/* ========== Alarm register bit definitions ========== */

#define SUN20I_RTC_ALRM_EN_CNT_EN            BIT(0)
#define SUN20I_RTC_ALRM_IRQ_EN_CNT_IRQ       BIT(0)
#define SUN20I_RTC_ALRM_IRQ_STA_PEND         BIT(0)
#define SUN20I_RTC_ALARM_CONFIG_WAKEUP       BIT(0)

/* ========== HMS bit fields (T113 §3.14.6.5) ========== */

#define SUN20I_RTC_HMS_SEC_MASK              0x3F
#define SUN20I_RTC_HMS_MIN_SHIFT             8
#define SUN20I_RTC_HMS_MIN_MASK              0x3F
#define SUN20I_RTC_HMS_HOUR_SHIFT            16
#define SUN20I_RTC_HMS_HOUR_MASK             0x1F

/* ========== DAY register bit fields ========== */

#define SUN20I_RTC_DAY_MASK                  0xFFFF

/* ========== SPI clock control (T113 §3.14.6.21) ========== */

#define SUN20I_RTC_SPI_CLK_GATING            BIT(31)
#define SUN20I_RTC_SPI_CLK_DIV_MASK          GENMASK(4, 0)
/* Default divider M=9 → SPI_CLK = AHBS1/(9+1) = 200MHz/10 = 20MHz */
#define SUN20I_RTC_SPI_CLK_DIV_DEFAULT       9

/* ========== Constants ========== */

#define SUN20I_SECS_PER_DAY                  (24ULL * 3600ULL)
#define SUN20I_SECS_PER_HOUR                 3600ULL
#define SUN20I_SECS_PER_MIN                  60ULL

/* ACCE timeout in milliseconds */
#define SUN20I_ACCE_TIMEOUT_MS               50

/* Maximum day counter value (16-bit) */
#define SUN20I_DAY_MAX                       65535U

/* ========== Helper macros for HMS encoding/decoding ========== */

#define SUN20I_HMS_GET_SEC(x)                ((x) & SUN20I_RTC_HMS_SEC_MASK)
#define SUN20I_HMS_GET_MIN(x)                (((x) >> SUN20I_RTC_HMS_MIN_SHIFT) & \
					      SUN20I_RTC_HMS_MIN_MASK)
#define SUN20I_HMS_GET_HOUR(x)               (((x) >> SUN20I_RTC_HMS_HOUR_SHIFT) & \
					      SUN20I_RTC_HMS_HOUR_MASK)

#define SUN20I_HMS_SET_SEC(x)                ((x) & SUN20I_RTC_HMS_SEC_MASK)
#define SUN20I_HMS_SET_MIN(x)                (((x) & SUN20I_RTC_HMS_MIN_MASK) << \
					      SUN20I_RTC_HMS_MIN_SHIFT)
#define SUN20I_HMS_SET_HOUR(x)               (((x) & SUN20I_RTC_HMS_HOUR_MASK) << \
					      SUN20I_RTC_HMS_HOUR_SHIFT)

/* ========== Device data ========== */

struct rtc_sun20i_d1_data {
	struct k_spinlock lock;
#ifdef CONFIG_RTC_ALARM
	bool alarm_pending;
	rtc_alarm_callback alarm_cb;
	void *alarm_cb_data;
#endif
};

/* ========== Device config ========== */

struct rtc_sun20i_d1_config {
	mm_reg_t base;
};

/* ========== MMIO helpers ========== */

static inline uint32_t sun20i_rtc_read(const struct device *dev, uint32_t offset)
{
	const struct rtc_sun20i_d1_config *config = dev->config;

	return sys_read32(config->base + offset);
}

static inline void sun20i_rtc_write(const struct device *dev, uint32_t offset,
				    uint32_t value)
{
	const struct rtc_sun20i_d1_config *config = dev->config;

	sys_write32(value, config->base + offset);
}

/* ========== Internal helpers ========== */

/**
 * Wait for specific ACCE bits in LOSC_CTRL to clear.
 * Returns 0 on success, -ETIMEDOUT on timeout.
 */
static int sun20i_rtc_wait_acce(const struct device *dev, uint32_t mask,
				uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;
	uint32_t val;

	do {
		val = sun20i_rtc_read(dev, SUN20I_RTC_LOSC_CTRL) & mask;
		if (val == 0) {
			return 0;
		}
	} while (k_uptime_get() < deadline);

	return -ETIMEDOUT;
}

/**
 * Validate rtc_time fields.
 */
static bool sun20i_rtc_validate_time(const struct rtc_time *timeptr)
{
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
	/* tm_year is years since 1900; no range limit for linear day */
	if (timeptr->tm_year < 70) {
		/* UNIX epoch starts at 1970 */
		return false;
	}

	return true;
}

/*
 * Convert a Zephyr rtc_time (which is struct tm compatible) to
 * UNIX epoch seconds.
 *
 * This is a simplified version — Zephyr doesn't provide mktime in
 * minimal libc, so we implement a basic day-to-seconds conversion.
 */

/* Days per month (non-leap year) */
static const uint16_t days_in_month[12] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static bool sun20i_is_leap_year(int year)
{
	return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

/**
 * Compute days from 1970-01-01 to the given date.
 */
static int64_t sun20i_date_to_days(int year, int mon, int mday)
{
	int64_t days = 0;
	int y;

	/* Add days for each full year since 1970 */
	for (y = 1970; y < year; y++) {
		days += sun20i_is_leap_year(y) ? 366 : 365;
	}

	/* Add days for months in current year */
	for (int m = 0; m < mon; m++) {
		days += days_in_month[m];
		if (m == 1 && sun20i_is_leap_year(year)) {
			days += 1; /* February leap day */
		}
	}

	/* Add days in current month (mday is 1-based) */
	days += mday - 1;

	return days;
}

/**
 * Convert days since 1970-01-01 to year/month/day.
 */
static void sun20i_days_to_date(int64_t days, int *year, int *mon, int *mday)
{
	int y = 1970;

	while (days >= 366) {
		int days_in_year = sun20i_is_leap_year(y) ? 366 : 365;

		if (days < days_in_year) {
			break;
		}
		days -= days_in_year;
		y++;
	}

	*year = y;

	int m = 0;

	for (m = 0; m < 12; m++) {
		int dim = days_in_month[m];

		if (m == 1 && sun20i_is_leap_year(y)) {
			dim = 29;
		}
		if (days < dim) {
			break;
		}
		days -= dim;
	}

	*mon = m;
	*mday = (int)days + 1;
}

/* ========== RTC API implementation ========== */

/**
 * Set RTC time using linear-day mode.
 *
 * The DAY register holds a 16-bit day number (UNIX epoch based).
 * The HH_MM_SS register holds hour/minute/second.
 */
static int sun20i_rtc_set_time(const struct device *dev,
			       const struct rtc_time *timeptr)
{
	struct rtc_sun20i_d1_data *data = dev->data;
	int ret;
	uint32_t hms_val;
	int64_t days;
	int year;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (!sun20i_rtc_validate_time(timeptr)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for all ACCE bits to clear */
	ret = sun20i_rtc_wait_acce(dev, SUN20I_RTC_LOSC_CTRL_ACC_MASK,
				    SUN20I_ACCE_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("RTC busy, ACCE bits not cleared");
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	/* Encode HMS */
	hms_val = SUN20I_HMS_SET_SEC(timeptr->tm_sec) |
		  SUN20I_HMS_SET_MIN(timeptr->tm_min) |
		  SUN20I_HMS_SET_HOUR(timeptr->tm_hour);

	/* Write HMS */
	sun20i_rtc_write(dev, SUN20I_RTC_HH_MM_SS_REG, hms_val);

	/* Wait for HMS ACC to clear */
	ret = sun20i_rtc_wait_acce(dev, SUN20I_RTC_LOSC_CTRL_HMS_ACC,
				    SUN20I_ACCE_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for HMS ACC to clear");
		k_spin_unlock(&data->lock, key);
		return -ETIMEDOUT;
	}

	/* Convert calendar date to days since 1970-01-01 */
	year = timeptr->tm_year + 1900;
	days = sun20i_date_to_days(year, timeptr->tm_mon, timeptr->tm_mday);

	if (days < 0 || days > SUN20I_DAY_MAX) {
		LOG_ERR("Date out of range: days=%lld", days);
		k_spin_unlock(&data->lock, key);
		return -EINVAL;
	}

	/* Write DAY */
	sun20i_rtc_write(dev, SUN20I_RTC_DAY_REG, (uint32_t)days);

	/* Wait for DAY ACC to clear */
	ret = sun20i_rtc_wait_acce(dev, SUN20I_RTC_LOSC_CTRL_DAY_ACC,
				    SUN20I_ACCE_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for DAY ACC to clear");
		k_spin_unlock(&data->lock, key);
		return -ETIMEDOUT;
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

/**
 * Get RTC time using linear-day mode.
 */
static int sun20i_rtc_get_time(const struct device *dev,
			       struct rtc_time *timeptr)
{
	struct rtc_sun20i_d1_data *data = dev->data;
	uint32_t day_val, day_val2;
	uint32_t hms_val, hms_val2;
	int64_t days;
	int year, mon, mday;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/*
	 * Read DAY and HMS repeatedly until two consecutive reads
	 * return the same values (prevent cross-second reads).
	 */
	do {
		day_val = sun20i_rtc_read(dev, SUN20I_RTC_DAY_REG);
		hms_val = sun20i_rtc_read(dev, SUN20I_RTC_HH_MM_SS_REG);
		day_val2 = sun20i_rtc_read(dev, SUN20I_RTC_DAY_REG);
		hms_val2 = sun20i_rtc_read(dev, SUN20I_RTC_HH_MM_SS_REG);
	} while (day_val != day_val2 || hms_val != hms_val2);

	k_spin_unlock(&data->lock, key);

	/* Decode HMS */
	timeptr->tm_sec  = SUN20I_HMS_GET_SEC(hms_val);
	timeptr->tm_min  = SUN20I_HMS_GET_MIN(hms_val);
	timeptr->tm_hour = SUN20I_HMS_GET_HOUR(hms_val);

	/* Decode linear day → calendar date */
	days = day_val & SUN20I_RTC_DAY_MASK;
	sun20i_days_to_date(days, &year, &mon, &mday);

	timeptr->tm_mday = mday;
	timeptr->tm_mon  = mon;         /* 0-based month */
	timeptr->tm_year = year - 1900; /* years since 1900 */

	/* Fields not provided by hardware */
	timeptr->tm_nsec = 0;
	timeptr->tm_wday = -1;
	timeptr->tm_yday = -1;

	/* Validate result */
	if (!sun20i_rtc_validate_time(timeptr)) {
		return -ENODATA;
	}

	return 0;
}

/* ========== Alarm API (CONFIG_RTC_ALARM) ========== */

#ifdef CONFIG_RTC_ALARM

/**
 * T113 alarm is a hardware comparator:
 * When RTC DAY + HMS equals ALARM0_DAY_SET + ALARM0_CUR_VAL,
 * the interrupt fires.
 */
static int sun20i_rtc_alarm_get_supported_fields(const struct device *dev,
						  uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != 0) {
		return -EINVAL;
	}

	/*
	 * The comparator supports matching on DAY, HOUR, MINUTE, SECOND.
	 * All fields must be set for correct comparator behavior.
	 */
	*mask = RTC_ALARM_TIME_MASK_SECOND |
		RTC_ALARM_TIME_MASK_MINUTE |
		RTC_ALARM_TIME_MASK_HOUR |
		RTC_ALARM_TIME_MASK_MONTHDAY;

	return 0;
}

static int sun20i_rtc_alarm_set_time(const struct device *dev, uint16_t id,
				      uint16_t mask,
				      const struct rtc_time *timeptr)
{
	struct rtc_sun20i_d1_data *data = dev->data;
	struct rtc_time now;
	int ret;
	uint32_t alrm_hms, alrm_day;
	int year;

	if (id != 0) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Disable alarm first */
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_EN, 0);
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_EN, 0);
	sun20i_rtc_write(dev, SUN20I_RTC_ALARM_CONFIG, 0);

	/* Clear any pending interrupt */
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_STA,
			 SUN20I_RTC_ALRM_IRQ_STA_PEND);

	if (mask == 0) {
		/* No fields to set — alarm disabled */
		data->alarm_pending = false;
		ret = 0;
		goto out;
	}

	if (timeptr == NULL) {
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
	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		if (timeptr->tm_mday < 1 || timeptr->tm_mday > 31) {
			ret = -EINVAL;
			goto out;
		}
	}

	/*
	 * Read current time to fill in fields not covered by mask.
	 * The comparator needs all fields to match.
	 */
	sun20i_rtc_get_time(dev, &now);

	/* Build alarm HMS from mask + current time fallback */
	alrm_hms = 0;
	if (mask & RTC_ALARM_TIME_MASK_SECOND) {
		alrm_hms |= SUN20I_HMS_SET_SEC(timeptr->tm_sec);
	} else {
		alrm_hms |= SUN20I_HMS_SET_SEC(now.tm_sec);
	}
	if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
		alrm_hms |= SUN20I_HMS_SET_MIN(timeptr->tm_min);
	} else {
		alrm_hms |= SUN20I_HMS_SET_MIN(now.tm_min);
	}
	if (mask & RTC_ALARM_TIME_MASK_HOUR) {
		alrm_hms |= SUN20I_HMS_SET_HOUR(timeptr->tm_hour);
	} else {
		alrm_hms |= SUN20I_HMS_SET_HOUR(now.tm_hour);
	}

	/* Build alarm day */
	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		year = timeptr->tm_year + 1900;
		alrm_day = (uint32_t)sun20i_date_to_days(year, timeptr->tm_mon,
							  timeptr->tm_mday);
	} else {
		alrm_day = (uint32_t)sun20i_date_to_days(now.tm_year + 1900,
							  now.tm_mon, now.tm_mday);
	}

	/* Write alarm target values */
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_CUR_VAL, alrm_hms);
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_DAY_SET, alrm_day);

	/* Enable alarm counter, interrupt, and wakeup */
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_EN,
			 SUN20I_RTC_ALRM_EN_CNT_EN);
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_EN,
			 SUN20I_RTC_ALRM_IRQ_EN_CNT_IRQ);
	sun20i_rtc_write(dev, SUN20I_RTC_ALARM_CONFIG,
			 SUN20I_RTC_ALARM_CONFIG_WAKEUP);

	data->alarm_pending = false;
	ret = 0;

out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int sun20i_rtc_alarm_get_time(const struct device *dev, uint16_t id,
				      uint16_t *mask, struct rtc_time *timeptr)
{
	struct rtc_sun20i_d1_data *data = dev->data;
	uint32_t alrm_hms, alrm_day;
	int year, mon, mday;

	if (id != 0) {
		return -EINVAL;
	}

	if (timeptr == NULL || mask == NULL) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	alrm_day = sun20i_rtc_read(dev, SUN20I_RTC_ALRM_DAY_SET);
	alrm_hms = sun20i_rtc_read(dev, SUN20I_RTC_ALRM_CUR_VAL);

	/* Check if alarm is enabled */
	uint32_t alrm_en = sun20i_rtc_read(dev, SUN20I_RTC_ALRM_EN);

	k_spin_unlock(&data->lock, key);

	if (!(alrm_en & SUN20I_RTC_ALRM_EN_CNT_EN)) {
		/* Alarm not enabled */
		memset(timeptr, 0, sizeof(*timeptr));
		*mask = 0;
		return 0;
	}

	/* Decode HMS */
	timeptr->tm_sec  = SUN20I_HMS_GET_SEC(alrm_hms);
	timeptr->tm_min  = SUN20I_HMS_GET_MIN(alrm_hms);
	timeptr->tm_hour = SUN20I_HMS_GET_HOUR(alrm_hms);

	/* Decode day → calendar */
	sun20i_days_to_date(alrm_day, &year, &mon, &mday);
	timeptr->tm_mday = mday;
	timeptr->tm_mon  = mon;
	timeptr->tm_year = year - 1900;

	*mask = RTC_ALARM_TIME_MASK_SECOND |
		RTC_ALARM_TIME_MASK_MINUTE |
		RTC_ALARM_TIME_MASK_HOUR |
		RTC_ALARM_TIME_MASK_MONTHDAY;

	return 0;
}

static int sun20i_rtc_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_sun20i_d1_data *data = dev->data;
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

static int sun20i_rtc_alarm_set_callback(const struct device *dev, uint16_t id,
					  rtc_alarm_callback callback,
					  void *user_data)
{
	struct rtc_sun20i_d1_data *data = dev->data;

	if (id != 0) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->alarm_cb = callback;
	data->alarm_cb_data = user_data;

	if (callback == NULL) {
		/* Disable alarm interrupt */
		sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_EN, 0);
	} else {
		/* Enable alarm interrupt if alarm is active */
		uint32_t alrm_en = sun20i_rtc_read(dev, SUN20I_RTC_ALRM_EN);

		if (alrm_en & SUN20I_RTC_ALRM_EN_CNT_EN) {
			sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_EN,
					 SUN20I_RTC_ALRM_IRQ_EN_CNT_IRQ);
		}
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

#endif /* CONFIG_RTC_ALARM */

/* ========== ISR ========== */

static void sun20i_rtc_isr(const struct device *dev)
{
#ifdef CONFIG_RTC_ALARM
	struct rtc_sun20i_d1_data *data = dev->data;
#endif
	uint32_t status;

	status = sun20i_rtc_read(dev, SUN20I_RTC_ALRM_IRQ_STA);

	if (status & SUN20I_RTC_ALRM_IRQ_STA_PEND) {
		/* Clear pending interrupt (write 1 to clear) */
		sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_STA,
				 SUN20I_RTC_ALRM_IRQ_STA_PEND);

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

static DEVICE_API(rtc, sun20i_rtc_driver_api) = {
	.set_time = sun20i_rtc_set_time,
	.get_time = sun20i_rtc_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = sun20i_rtc_alarm_get_supported_fields,
	.alarm_set_time = sun20i_rtc_alarm_set_time,
	.alarm_get_time = sun20i_rtc_alarm_get_time,
	.alarm_is_pending = sun20i_rtc_alarm_is_pending,
	.alarm_set_callback = sun20i_rtc_alarm_set_callback,
#endif
};

/* ========== Init ========== */

static int sun20i_rtc_init(const struct device *dev)
{
	uint32_t val;

	/*
	 * Step 1: Configure SPI clock for RTC register access.
	 *
	 * The RTC core registers (offset < 0x300) are in VDD_RTC domain
	 * and accessed via an internal SPI bus from VDD_SYS domain.
	 * We must configure the SPI clock divider and enable gating
	 * before any RTC register access.
	 *
	 * Default SPI_CLK = AHBS1 / (M+1) = 200MHz / 10 = 20MHz
	 */
	val = sun20i_rtc_read(dev, SUN20I_RTC_SPI_CLK_CTRL);

	/* Set divider to default if not configured (reset value is 0x9) */
	val &= ~SUN20I_RTC_SPI_CLK_DIV_MASK;
	val |= SUN20I_RTC_SPI_CLK_DIV_DEFAULT;

	/* Enable SPI clock (cancel gating) */
	val |= SUN20I_RTC_SPI_CLK_GATING;

	sun20i_rtc_write(dev, SUN20I_RTC_SPI_CLK_CTRL, val);

	/*
	 * Step 2: Configure LOSC_CTRL.
	 *
	 * - Write KEY field to allow register modification
	 * - Select external 32.768 kHz crystal (bit0 = 1)
	 * - Enable external crystal (bit4 = 1)
	 * - Enable auto-switch to RC if external clock fails (bit14 = 1)
	 * - Leave auto-switch function enabled (bit15 = 0)
	 */
	val = SUN20I_RTC_LOSC_CTRL_KEY |
	      SUN20I_RTC_LOSC_CTRL_SRC_SEL |
	      SUN20I_RTC_LOSC_CTRL_EXT_LOSC_EN |
	      SUN20I_RTC_LOSC_CTRL_AUTO_SWT_EN;

	sun20i_rtc_write(dev, SUN20I_RTC_LOSC_CTRL, val);

	/*
	 * Step 3: Disable alarms initially.
	 */
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_EN, 0);
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_EN, 0);

	/* Clear any pending alarm interrupt */
	sun20i_rtc_write(dev, SUN20I_RTC_ALRM_IRQ_STA,
			 SUN20I_RTC_ALRM_IRQ_STA_PEND);

	/*
	 * Step 4: Configure interrupt.
	 */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    sun20i_rtc_isr,
		    DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

/* ========== Device instantiation ========== */

#define RTC_SUN20I_D1_INIT(inst)						\
	static struct rtc_sun20i_d1_data rtc_sun20i_d1_data_##inst;		\
	static const struct rtc_sun20i_d1_config rtc_sun20i_d1_config_##inst = { \
		.base = DT_INST_REG_ADDR(inst),					\
	};										\
	DEVICE_DT_INST_DEFINE(inst,							\
			      &sun20i_rtc_init,						\
			      NULL,							\
			      &rtc_sun20i_d1_data_##inst,				\
			      &rtc_sun20i_d1_config_##inst,				\
			      POST_KERNEL,						\
			      CONFIG_RTC_INIT_PRIORITY,				\
			      &sun20i_rtc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_SUN20I_D1_INIT)
