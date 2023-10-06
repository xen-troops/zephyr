/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT renesas_rza2m_rtc

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rtc_rza2m, CONFIG_RTC_LOG_LEVEL);

#define MIN_EXTAL_CLK_RATE 10000000

#define MIN_YEAR 2000
#define MAX_YEAR 2099

#define R64CNT_OFFSET       0x0 /* 64-Hz Counter */
#define R64CNT_SUB_SEC_BITS 6
#define R64CNT_SUB_SEC_MASK BIT_MASK(R64CNT_SUB_SEC_BITS)

#define RSECCNT_OFFSET 0x02 /* Second Counter/Binary Counter 0 */
#define RMINCNT_OFFSET 0x04 /* Minute Counter/Binary Counter 1 */
#define RHRCNT_OFFSET  0x06 /* Hour Counter/Binary Counter 2 */
#define RWKCNT_OFFSET  0x08 /* Day-of-Week Counter/Binary Counter 3 */
#define RDAYCNT_OFFSET 0x0A /* Day Counter */
#define RMONCNT_OFFSET 0x0C /* Month Counter */
#define RYRCNT_OFFSET  0x0E /* Year Counter */

#define RSECCNT_BCD_MASK  0x7f
#define RMINCNT_BCD_MASK  0x7f
#define RHRCNT_BCD_MASK   0x3f
#define RHRCNT_BCD_PM_BIT BIT(6)
#define RWKCNT_BCD_MASK   0x07
#define RDAYCNT_BCD_MASK  0x3f
#define RMONCNT_BCD_MASK  0x1f

#define RSECAR_OFFSET 0x10 /* Second Alarm Register/Binary Alarm Register 0 */
#define RMINAR_OFFSET 0x12 /* Minute Alarm Register/Binary Alarm Register 1 */
#define RHRAR_OFFSET  0x14 /* Hour Alarm Register/Binary Alarm Register 2 */
#define RWKAR_OFFSET  0x16 /* Day-of-Week Alarm Register/Binary Alarm Register 3 */
#define RDAYAR_OFFSET 0x18 /* Day Alarm Register/Binary Counter 0 Alarm Enable Register */
#define RMONAR_OFFSET 0x1A /* Month Alarm Register/Binary Counter 1 Alarm Enable Register */
#define RYRAR_OFFSET  0x1C /* Year Alarm Register/Binary Counter 2 Alarm Enable Register */
#define RYRAREN       0x1E /* Year Alarm Enable Register/Binary Counter 3 Alarm Enable Register */

/* it is used for all alarm regs, except year */
#define ALARM_EN_BIT BIT(7)

#define RSR    0x20   /* RTC Status Register */
#define RSR_AF BIT(0) /* Alarm Interrupt Flag */
#define RSR_CF BIT(1) /* Carry Interrupt Flag */
#define RSR_PF BIT(2) /* Periodic Interrupt Flag */

#define RCR1          0x22   /* RTC Control Register 1 */
#define RCR1_AIE      BIT(0) /* Alarm Interrupt Enable */
#define RCR1_CIE      BIT(1) /* Carry Interrupt Enable */
#define RCR1_PIE      BIT(2) /* Periodic Interrupt Enable */
#define RCR1_PES_MASK 0xf0   /* Periodic Interrupt Select Mask */

#define RCR2       0x24   /* RTC Control Register 2 */
#define RCR2_START BIT(0) /* Prescaler and time counter operate normally, otherwise stopped */
#define RCR2_RESET BIT(1) /* RTC Software Reset */
#define RCR2_ADJ30 BIT(2) /* 30-Second Adjustment */
#define RCR2_AADJE BIT(4) /* Automatic Adjustment Enable */
#define RCR2_AADJP BIT(5) /* Automatic Adjustment Period Select 10 sec*/
#define RCR2_HR24  BIT(6) /* the RTC operates in 24-hour mode */
#define RCR2_CNTMD BIT(7) /* binary count mode */

#define RCR3       0x26   /* RTC Control Register 3 */
#define RCR3_RTCEN BIT(0) /* The 32-kHz clock oscillator is operated */

#define RCR4        0x28   /* RTC Control Register 4 */
#define RCR4_RCKSEL BIT(0) /* EXTAL clock is selected */

#define RFRH 0x2A /* Frequency Register H */
#define RFRL 0x2C /* Frequency Register L */

#define RADJ         0x2E /* Time Error Adjustment Register */
#define RADJ_OP_ADD  0x40
#define RADJ_OP_SUB  0x80

enum period_isr_interval {
	period_part_of_sec_256 = 0x60, /* 1/256 sec, 1/128 in case when we use EXTAL */
	period_part_of_sec_128 = 0x70, /* 1/128 sec */
	period_part_of_sec_64 = 0x80,  /* 1/64  sec */
	period_part_of_sec_32 = 0x90,  /* 1/32  sec */
	period_part_of_sec_16 = 0xA0,  /* 1/16  sec */
	period_part_of_sec_8 = 0xB0,   /* 1/8   sec */
	period_part_of_sec_4 = 0xC0,   /* 1/4   sec */
	period_part_of_sec_2 = 0xD0,   /* 1/2   sec */
	period_part_of_sec_1 = 0xE0,   /* 1     sec */
	period_two_sec = 0xF0,         /* 2     sec */
};

struct rtc_rza2m_config {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *cpg;
	struct renesas_cpg_clk rtc_clk;
	uint32_t clk_freq;
	bool is_rtc_x1_src;

	unsigned int alarm_irq_line;
	unsigned int period_irq_line;
	unsigned int carry_irq_line;

	void (*cfg_irqs)(const struct device *dev);
};

struct rtc_rza2m_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct k_spinlock lock;
#ifdef CONFIG_RTC_ALARM
	struct rtc_time set_alarm_time;
	uint16_t set_alram_mask;
	bool alarm_is_pending;
	rtc_alarm_callback alarm_callback;
	void *alarm_user_data;
#endif
#ifdef CONFIG_RTC_UPDATE
	rtc_update_callback update_callback;
	void *update_user_data;
#endif
#ifdef CONFIG_RTC_CALIBRATION
	int32_t calibration;
	uint8_t radj;
	bool manual_cal;
#endif
};

/* prescaler and time counter start/stop */
static inline void rtc_rza2m_start_stop_timer(const struct device *dev, bool start)
{
	uint8_t reg = sys_read8(DEVICE_MMIO_GET(dev) + RCR2);
	const uint8_t wait_val = start ? RCR2_START : 0;

	if (start) {
		reg |= RCR2_START;
	} else {
		reg &= ~RCR2_START;
	}
	sys_write8(reg, DEVICE_MMIO_GET(dev) + RCR2);

	while ((sys_read8(DEVICE_MMIO_GET(dev) + RCR2) & RCR2_START) != wait_val)
		;
}

/* software reset of RTC */
static inline void rtc_rza2m_soft_reset(const struct device *dev)
{
	uint8_t reg = sys_read8(DEVICE_MMIO_GET(dev) + RCR2);

	sys_write8(reg | RCR2_RESET, DEVICE_MMIO_GET(dev) + RCR2);

	while ((sys_read8(DEVICE_MMIO_GET(dev) + RCR2) & RCR2_RESET) == RCR2_RESET)
		;
}

#ifdef CONFIG_RTC_ALARM
static int rtc_rza2m_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				    const struct rtc_time *timeptr);
#endif

#ifdef CONFIG_RTC_CALIBRATION
static int rtc_rza2m_set_calibration(const struct device *dev, int32_t calibration);
#endif

static int rtc_rza2m_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	struct rtc_rza2m_data *data = dev->data;
	k_spinlock_key_t key;
	mem_addr_t base;
	uint32_t year;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	year = timeptr->tm_year + 1900;
	if (year < MIN_YEAR || year > MAX_YEAR) {
		LOG_ERR("%s:%s: year can be set only in range [%d; %d]", __func__, dev->name,
			MIN_YEAR, MAX_YEAR);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);
	rtc_rza2m_start_stop_timer(dev, false);
	rtc_rza2m_soft_reset(dev);

	sys_write8(bin2bcd(timeptr->tm_sec), base + RSECCNT_OFFSET);
	sys_write8(bin2bcd(timeptr->tm_min), base + RMINCNT_OFFSET);
	sys_write8(bin2bcd(timeptr->tm_hour), base + RHRCNT_OFFSET);
	sys_write8(bin2bcd(timeptr->tm_mday), base + RDAYCNT_OFFSET);
	sys_write8(bin2bcd(timeptr->tm_mon + 1), base + RMONCNT_OFFSET);
	sys_write16(bin2bcd(year - MIN_YEAR), base + RYRCNT_OFFSET);
	sys_write8(bin2bcd(timeptr->tm_wday), base + RWKCNT_OFFSET);

	rtc_rza2m_start_stop_timer(dev, true);
	k_spin_unlock(&data->lock, key);

#ifdef CONFIG_RTC_ALARM
	/* alarm registers reseted by RTC soft reset, so try to restore them here */
	rtc_rza2m_alarm_set_time(dev, 0, data->set_alram_mask, &data->set_alarm_time);
#endif
#ifdef CONFIG_RTC_CALIBRATION
	/* restore calibration */
	rtc_rza2m_set_calibration(dev, data->calibration);
#endif
	return 0;
}

static int rtc_rza2m_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct rtc_rza2m_data *data = dev->data;
	k_spinlock_key_t key;
	mem_addr_t base;
	uint8_t reg;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);
	/* disable the carry interrupt request */
	reg = sys_read8(base + RCR1);
	reg &= ~RCR1_CIE;
	sys_write8(reg, base + RCR1);

	do {
		/* clear the carry flag */
		reg = sys_read8(base + RSR);
		reg &= ~RSR_CF;
		sys_write8(reg, base + RSR);

		timeptr->tm_sec = bcd2bin(sys_read8(base + RSECCNT_OFFSET) & RSECCNT_BCD_MASK);
		timeptr->tm_min = bcd2bin(sys_read8(base + RMINCNT_OFFSET) & RMINCNT_BCD_MASK);

		reg = sys_read8(base + RHRCNT_OFFSET);
		timeptr->tm_hour = bcd2bin(reg & RHRCNT_BCD_MASK);

		timeptr->tm_mday = bcd2bin(sys_read8(base + RDAYCNT_OFFSET) & RDAYCNT_BCD_MASK);
		timeptr->tm_mon = bcd2bin(sys_read8(base + RMONCNT_OFFSET) & RMONCNT_BCD_MASK) - 1;

		/* works in range 2000-2099 */
		timeptr->tm_year = bcd2bin(sys_read16(base + RYRCNT_OFFSET)) + 100;

		timeptr->tm_wday = bcd2bin(sys_read8(base + RWKCNT_OFFSET) & RWKCNT_BCD_MASK);
		timeptr->tm_yday = -1;
		timeptr->tm_isdst = -1;

		timeptr->tm_nsec = sys_read8(base + R64CNT_OFFSET) & R64CNT_SUB_SEC_MASK;
		timeptr->tm_nsec = ((timeptr->tm_nsec * 1000000U) >> R64CNT_SUB_SEC_BITS) * 1000;
	} while ((sys_read8(base + RSR) & RSR_CF) == RSR_CF);
	k_spin_unlock(&data->lock, key);

	return 0;
}

#ifdef CONFIG_RTC_ALARM

static int rtc_rza2m_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						uint16_t *mask)
{
	if (id != 0) {
		LOG_ERR("%s:%s: only one alarm is supported by this timer", __func__, dev->name);
		return -EINVAL;
	}

	*mask = RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |
		RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_MONTHDAY |
		RTC_ALARM_TIME_MASK_YEAR | RTC_ALARM_TIME_MASK_WEEKDAY;
	return 0;
}

static int rtc_rza2m_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				    const struct rtc_time *timeptr)
{
	uint8_t reg;
	int ret = 0;
	mem_addr_t base;
	k_spinlock_key_t key;
	uint16_t rtc_alarm_mask;
	uint32_t year;
	const struct rtc_rza2m_config *config = dev->config;
	struct rtc_rza2m_data *data = dev->data;

	if (id != 0) {
		LOG_ERR("%s:%s: only one alarm is supported by this timer", __func__, dev->name);
		return -EINVAL;
	}

	rtc_rza2m_alarm_get_supported_fields(dev, 0, &rtc_alarm_mask);
	if (~rtc_alarm_mask & mask) {
		LOG_ERR("%s: requested alarm mask (%hx) isn't supported ", dev->name, mask);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);
	/* disable alarm here */
	if (mask == 0) {
		reg = sys_read8(base + RCR1);
		irq_disable(config->alarm_irq_line);
		sys_write8(reg & ~RCR1_AIE, base + RCR1);
		while ((sys_read8(base + RCR1) & RCR1_AIE) == RCR1_AIE)
			;
		data->alarm_is_pending = false;
		data->set_alram_mask = 0;
		goto out;
	}

	if (timeptr == NULL) {
		LOG_ERR("%s:%s: timeptr is NULL", __func__, dev->name);
		ret = -EINVAL;
		goto out;
	}

	year = timeptr->tm_year + 1900;
	if ((mask & RTC_ALARM_TIME_MASK_YEAR) && (year < MIN_YEAR || year > MAX_YEAR)) {
		LOG_ERR("%s:%s: year can be set only in range [%d; %d]", __func__, dev->name,
			MIN_YEAR, MAX_YEAR);
		ret = -EINVAL;
		goto out;
	}

	reg = sys_read8(base + RCR2);
	if ((reg & RCR2_START) != RCR2_START) {
		LOG_ERR("%s: timer isn't started, so it isn't possible to set alarm", dev->name);
		ret = -EINVAL;
		goto out;
	}

	reg = sys_read8(base + RCR1);
	sys_write8(reg & ~RCR1_AIE, base + RCR1);
	while ((sys_read8(base + RCR1) & RCR1_AIE) == RCR1_AIE)
		;

	rtc_alarm_mask = 1;
	while (rtc_alarm_mask) {
		switch (mask & rtc_alarm_mask) {
		case RTC_ALARM_TIME_MASK_SECOND:
			sys_write8(bin2bcd(timeptr->tm_sec) | ALARM_EN_BIT, base + RSECAR_OFFSET);
			break;
		case RTC_ALARM_TIME_MASK_MINUTE:
			sys_write8(bin2bcd(timeptr->tm_min) | ALARM_EN_BIT, base + RMINAR_OFFSET);
			break;
		case RTC_ALARM_TIME_MASK_HOUR:
			sys_write8(bin2bcd(timeptr->tm_hour) | ALARM_EN_BIT, base + RHRAR_OFFSET);
			break;
		case RTC_ALARM_TIME_MASK_MONTH:
			sys_write8(bin2bcd(timeptr->tm_mon + 1) | ALARM_EN_BIT,
				   base + RMONAR_OFFSET);
			break;
		case RTC_ALARM_TIME_MASK_MONTHDAY:
			sys_write8(bin2bcd(timeptr->tm_mday) | ALARM_EN_BIT, base + RDAYAR_OFFSET);
			break;
		case RTC_ALARM_TIME_MASK_YEAR:
			sys_write16(bin2bcd(year - MIN_YEAR), base + RYRAR_OFFSET);
			sys_write8(ALARM_EN_BIT, base + RYRAREN);
			break;
		case RTC_ALARM_TIME_MASK_WEEKDAY:
			sys_write8(bin2bcd(timeptr->tm_wday) | ALARM_EN_BIT, base + RWKAR_OFFSET);
			break;
		default:
			break;
		}
		rtc_alarm_mask <<= 1;
	}

	/*
	 * we should here to set periodic interrupt to 1/64 sec,
	 * but this irq is used for update_set_callback, so just wait 2 * 1/64 secs here
	 */
	k_busy_wait(31250);

	reg = sys_read8(base + RSR);
	sys_write8(reg & ~RSR_AF, base + RSR);

	irq_enable(config->alarm_irq_line);

	reg = sys_read8(base + RCR1);
	sys_write8(reg | RCR1_AIE, base + RCR1);
	while ((sys_read8(base + RCR1) & RCR1_AIE) != RCR1_AIE)
		;

	memcpy(&data->set_alarm_time, timeptr, sizeof(data->set_alarm_time));
	data->set_alram_mask = mask;
out:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int rtc_rza2m_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				    struct rtc_time *timeptr)
{
	uint8_t reg;
	int ret = 0;
	k_spinlock_key_t key;
	mem_addr_t base;
	struct rtc_rza2m_data *data = dev->data;
	const struct rtc_rza2m_config *config = dev->config;

	if (id != 0) {
		LOG_ERR("%s:%s: only one alarm is supported by this timer", __func__, dev->name);
		return -EINVAL;
	}

	if (timeptr == NULL) {
		LOG_ERR("%s:%s: timeptr is NULL", __func__, dev->name);
		return -EINVAL;
	}

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);

	reg = sys_read8(base + RCR2);
	if ((reg & RCR2_START) != RCR2_START) {
		LOG_ERR("%s:%s: timer isn't started", __func__, dev->name);
		ret = -EINVAL;
		*mask = 0;
		goto out;
	}

	reg = sys_read8(base + RCR1);
	if ((reg & RCR1_AIE) != RCR1_AIE || !irq_is_enabled(config->alarm_irq_line)) {
		LOG_ERR("%s:%s: alarm interrupt is disabled (%hhx)", __func__, dev->name, reg);
		ret = -EINVAL;
		*mask = 0;
		goto out;
	}

	memcpy(timeptr, &data->set_alarm_time, sizeof(data->set_alarm_time));
	*mask = data->set_alram_mask;
out:
	k_spin_unlock(&data->lock, key);

	return ret;
}

/*
 * unfortunately, this api doesn't represent the real alarm status it is only for testing purpose
 * see, description of `rtc_alarm_is_pending` and how it is used in tests
 */
static int rtc_rza2m_alarm_is_pending(const struct device *dev, uint16_t id)
{
	int ret;
	k_spinlock_key_t key;
	struct rtc_rza2m_data *data = dev->data;

	if (id != 0) {
		LOG_ERR("%s:%s: only one alarm is supported by this timer", __func__, dev->name);
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	ret = data->alarm_is_pending ? 1 : 0;
	data->alarm_is_pending = false;
	k_spin_unlock(&data->lock, key);

	return ret;
}

static int rtc_rza2m_alarm_set_callback(const struct device *dev, uint16_t id,
					rtc_alarm_callback callback, void *user_data)
{
	k_spinlock_key_t key;
	struct rtc_rza2m_data *data = dev->data;

	if (id != 0) {
		LOG_ERR("%s:%s: only one alarm is supported by this timer", __func__, dev->name);
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	data->alarm_callback = callback;
	data->alarm_user_data = user_data;
	k_spin_unlock(&data->lock, key);

	return 0;
}

static void rtc_rza2m_alarm_isr(const struct device *dev)
{
	struct rtc_rza2m_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t reg;
	rtc_alarm_callback callback;
	void *user_data;

	key = k_spin_lock(&data->lock);
	callback = data->alarm_callback;
	user_data = data->alarm_user_data;

	reg = sys_read8(DEVICE_MMIO_GET(dev) + RSR);
	sys_write8(reg & ~RSR_AF, DEVICE_MMIO_GET(dev) + RSR);

	if (callback) {
		/* alarm can be set for some interval, but it is how zephyr works with rtc */
		data->alarm_is_pending = false;
	} else {
		data->alarm_is_pending = true;
	}
	k_spin_unlock(&data->lock, key);

	if (callback) {
		callback(dev, 0, user_data);
	}
}

#else  /* CONFIG_RTC_ALARM */
static void rtc_rza2m_alarm_isr(const struct device *dev)
{
}
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE

static void rtc_rza2m_period_isr(const struct device *dev)
{
	struct rtc_rza2m_data *data = dev->data;
	k_spinlock_key_t key;
	uint8_t reg;
	rtc_update_callback callback;
	void *user_data;

	key = k_spin_lock(&data->lock);

	callback = data->update_callback;
	user_data = data->update_user_data;

	reg = sys_read8(DEVICE_MMIO_GET(dev) + RSR);
	sys_write8(reg & ~RSR_PF, DEVICE_MMIO_GET(dev) + RSR);

#ifdef CONFIG_RTC_CALIBRATION
	if (data->manual_cal) {
		sys_write8(data->radj, DEVICE_MMIO_GET(dev) + RADJ);
	}
#endif

	k_spin_unlock(&data->lock, key);

	if (callback) {
		callback(dev, user_data);
	}
}

static int rtc_rza2m_update_set_callback(const struct device *dev, rtc_update_callback callback,
					 void *user_data)
{
	struct rtc_rza2m_data *data = dev->data;
	const struct rtc_rza2m_config *config = dev->config;
	k_spinlock_key_t key;
	uint8_t reg;
	mem_addr_t base;

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);

	data->update_callback = callback;
	data->update_user_data = user_data;

	reg = sys_read8(base + RCR1);

	if (callback == NULL) {
		irq_disable(config->period_irq_line);
		sys_write8(reg & ~RCR1_PIE, base + RCR1);
		while ((sys_read8(base + RCR1) & RCR1_PIE) == RCR1_PIE)
			;

		reg = sys_read8(DEVICE_MMIO_GET(dev) + RSR);
		sys_write8(reg & ~RSR_PF, DEVICE_MMIO_GET(dev) + RSR);
	} else {
		irq_enable(config->period_irq_line);
		/* periodic interrupt is generated every second */
		reg &= ~RCR1_PES_MASK;
		sys_write8(reg | period_part_of_sec_1 | RCR1_PIE, base + RCR1);
		while ((sys_read8(base + RCR1) & RCR1_PIE) != RCR1_PIE)
			;
	}
	k_spin_unlock(&data->lock, key);

	return 0;
}

#else  /* CONFIG_RTC_UPDATE */
static void rtc_rza2m_period_isr(const struct device *dev)
{
}
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION

static int rtc_rza2m_set_calibration(const struct device *dev, int32_t calibration)
{
	mem_addr_t base;
	uint8_t reg;
	const struct rtc_rza2m_config *config = dev->config;
	struct rtc_rza2m_data *data = dev->data;
	k_spinlock_key_t key;
	int64_t ticks = (int64_t)config->clk_freq * calibration;
	uint8_t period = 1;

	if (!config->is_rtc_x1_src) {
		LOG_ERR("%s:%s: calibration isn't supported for EXTAL clock source", __func__,
			dev->name);
		return -ENOTSUP;
	}

	/* calibration in parts per billion */
	if (IN_RANGE((ticks * 60), -63000000000LL, 63000000000LL)) {
		ticks *= 60;
		period = 60;
	} else if (IN_RANGE((ticks * 10), -63000000000LL, 63000000000LL)) {
		ticks *= 10;
		period = 10;
#ifdef CONFIG_RTC_UPDATE
	} else if (IN_RANGE(ticks, -63000000000LL, 63000000000LL)) {
		ticks *= 1;
#endif
	} else {
		LOG_ERR("%s: calibration value (%d) isn't supported by driver", dev->name,
			calibration);
		return -ENOTSUP;
	}

	ticks /= 1000000000;

	base = DEVICE_MMIO_GET(dev);

	key = k_spin_lock(&data->lock);
	/* disable adjustment */
	sys_write8(0, base + RADJ);

	reg = sys_read8(base + RCR2);
	switch (period) {
	case 10:
		sys_write8(reg | RCR2_AADJE | RCR2_AADJP, base + RCR2);
		data->manual_cal = false;
		break;
	case 60:
		reg &= ~RCR2_AADJP;
		sys_write8(reg | RCR2_AADJE, base + RCR2);
		data->manual_cal = false;
		break;
	default:
		reg &= ~RCR2_AADJE; /* manual mode */
		sys_write8(reg, base + RCR2);
		data->manual_cal = true;
		break;
	}

	if (ticks < 0) {
		ticks = -ticks;
		data->radj = ticks | RADJ_OP_SUB;
	} else {
		data->radj = ticks | RADJ_OP_ADD;
	}

	if (data->manual_cal == false) {
		sys_write8(data->radj, base + RADJ);
	}

	k_spin_unlock(&data->lock, key);

	/* store information for restoring on reset and for get calibration callback */
	data->calibration = calibration;
	return 0;
}

static int rtc_rza2m_get_calibration(const struct device *dev, int32_t *calibration)
{
	const struct rtc_rza2m_config *config = dev->config;
	struct rtc_rza2m_data *data = dev->data;

	if (!config->is_rtc_x1_src) {
		LOG_ERR("%s:%s: calibration isn't supported for EXTAL clock source", __func__,
			dev->name);
		return -ENOTSUP;
	}

	*calibration = data->calibration;
	return 0;
}

#endif /* CONFIG_RTC_CALIBRATION */

struct rtc_driver_api rtc_rza2m_driver_api = {
	.set_time = rtc_rza2m_set_time,
	.get_time = rtc_rza2m_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_rza2m_alarm_get_supported_fields,
	.alarm_set_time = rtc_rza2m_alarm_set_time,
	.alarm_get_time = rtc_rza2m_alarm_get_time,
	.alarm_is_pending = rtc_rza2m_alarm_is_pending,
	.alarm_set_callback = rtc_rza2m_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_UPDATE
	.update_set_callback = rtc_rza2m_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */
#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_rza2m_set_calibration,
	.get_calibration = rtc_rza2m_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

static int rtc_rza2m_init(const struct device *dev)
{
	int ret;
	const struct rtc_rza2m_config *config = dev->config;
	const struct device *cpg = config->cpg;
	uint8_t reg;
	mem_addr_t base;

	if (!device_is_ready(cpg)) {
		LOG_ERR("%s: error cpg device isn't ready", dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(cpg, (clock_control_subsys_t)&config->rtc_clk);
	if (ret < 0) {
		LOG_ERR("%s: can't enable clock %s", dev->name, cpg->name);
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	base = DEVICE_MMIO_GET(dev);

	/* disable all IRQs */
	sys_write8(0, base + RCR1);

	/* clock select */
	if (config->is_rtc_x1_src) {
		sys_write8(0, base + RCR4);
		sys_write8(RCR3_RTCEN, base + RCR3);
	} else {
		sys_write8(RCR4_RCKSEL, base + RCR4);
		sys_write8(0, base + RCR3);
	}

	rtc_rza2m_start_stop_timer(dev, false);

	/* set 24-hour format and calendar mode */
	reg = sys_read8(base + RCR2);
	reg &= ~RCR2_CNTMD;
	reg |= RCR2_HR24;
	sys_write8(reg | RCR2_HR24, base + RCR2);

	if (!config->is_rtc_x1_src) {
		uint32_t extal_freq_comp = config->clk_freq;

		if (extal_freq_comp > 12000000) {
			extal_freq_comp /= 256;
		} else {
			extal_freq_comp /= 128;
		}
		extal_freq_comp -= 1;

		/* Set the frequency comparison value to generate 128-Hz clock from the EXTAL */
		sys_write16((extal_freq_comp >> 16) & 0x1, base + RFRH);
		sys_write16(extal_freq_comp & 0xffff, base + RFRL);
	}

	rtc_rza2m_soft_reset(dev);
	rtc_rza2m_start_stop_timer(dev, true);

	config->cfg_irqs(dev);
	return 0;
}

#define RTC_RZ_GET_RATE(inst) DT_INST_PROP_BY_PHANDLE_IDX(inst, clocks, 1, clock_frequency)

#define IRQ_CONNECT_BY_NAME(inst, name, isr)                                                       \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, name, irq),                                          \
		    DT_INST_IRQ_BY_NAME(inst, name, priority), isr, DEVICE_DT_INST_GET(inst),      \
		    DT_INST_IRQ_BY_NAME(inst, name, flags))

#define RTC_RZ_A2M_DEV_CFG(n)                                                                      \
	static struct rtc_rza2m_data rtc_data_##n;                                                 \
                                                                                                   \
	static void irq_config_func_##n(const struct device *dev)                                  \
	{                                                                                          \
		IRQ_CONNECT_BY_NAME(n, alarm, rtc_rza2m_alarm_isr);                                \
		IRQ_CONNECT_BY_NAME(n, period, rtc_rza2m_period_isr);                              \
	}                                                                                          \
	static const struct rtc_rza2m_config rtc_cfg_##n = {                                       \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.cpg = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                      \
		.rtc_clk.module = DT_INST_CLOCKS_CELL(n, module),                                  \
		.rtc_clk.domain = DT_INST_CLOCKS_CELL(n, domain),                                  \
		.clk_freq = RTC_RZ_GET_RATE(n),                                                    \
		.is_rtc_x1_src = ((RTC_RZ_GET_RATE(n) < MIN_EXTAL_CLK_RATE) ? true : false),       \
		.alarm_irq_line = DT_INST_IRQ_BY_NAME(n, alarm, irq),                              \
		.period_irq_line = DT_INST_IRQ_BY_NAME(n, period, irq),                            \
		.carry_irq_line = DT_INST_IRQ_BY_NAME(n, carry, irq),                              \
		.cfg_irqs = irq_config_func_##n,                                                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &rtc_rza2m_init, NULL, &rtc_data_##n, &rtc_cfg_##n, POST_KERNEL,  \
			      CONFIG_RTC_INIT_PRIORITY, &rtc_rza2m_driver_api);
DT_INST_FOREACH_STATUS_OKAY(RTC_RZ_A2M_DEV_CFG);
