/*
 * Copyright (c) 2026 jeck chen <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PWM driver for Allwinner V3s SoC.
 *
 * The V3s PWM controller has 2 channels (CH0, CH1), each with a 16-bit
 * up-counter clocked from OSC24M (24 MHz). A programmable prescaler
 * divides the source clock before it reaches the counter.
 *
 * Limitations:
 *   - Only cycle mode (continuous square wave) is supported.
 *   - No input capture capability.
 *   - Bypass mode (direct 24 MHz output) is not exposed.
 *   - Changing the period while the channel is running may glitch.
 */

#define DT_DRV_COMPAT allwinner_sun8i_v3s_pwm

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(pwm_v3s, CONFIG_PWM_LOG_LEVEL);

#define PWM_CTRL_REG       0x00
#define PWM_CH_PRD_BASE    0x04
#define PWM_CH_PRD_OFFSET  0x04
#define PWM_CH_PRD(ch)     (PWM_CH_PRD_BASE + PWM_CH_PRD_OFFSET * (ch))
#define PWMCH_OFFSET       15
#define PWM_EN             BIT(4)
#define PWM_ACT_STATE      BIT(5)
#define PWM_CLK_GATING     BIT(6)
#define PWM_MODE           BIT(7)
#define PWM_PULSE          BIT(8)
#define PWM_BYPASS         BIT(9)
#define PWM_PRESCAL_MASK   GENMASK(3, 0)
#define PWM_RDY(ch)        BIT(28 + (ch))
#define PWM_ENTIRE_CYC(prd)  (((prd) - 1) & 0xFFFF)
#define PWM_ACT_CYC(dty)     ((dty) & 0xFFFF)
#define PWM_PERIOD_REG(prd, dty) \
	((PWM_ENTIRE_CYC(prd) << 16) | PWM_ACT_CYC(dty))
#define BIT_CH(bit, ch)   ((bit) << ((ch) * PWMCH_OFFSET))
#define PWM_NUM_CHANNELS   2
#define PWM_SRC_CLK_FREQ   24000000U
#define PWM_PRD_MAX        65536U

/*
 * Prescaler table – index directly corresponds to the 4-bit register value.
 * Reserved entries are 0 and must be skipped during search.
 *
 * Per V3s datasheet (PWM_CH0_PRESCAL / PWM_CH1_PRESCAL):
 *   0x0=/120  0x1=/180  0x2=/240  0x3=/360  0x4=/480
 *   0x5–0x7 = reserved (blank in datasheet)
 *   0x8=/12k  0x9=/24k  0xA=/36k  0xB=/48k  0xC=/72k
 *   0xD–0xE = reserved (blank in datasheet)
 *   0xF = /1  (prescaler bypass, valid on V3s)
 *
 */
static const uint32_t prescaler_table[16] = {
 120,    /* 0x0 : /120  → 200 kHz  */
 180,    /* 0x1 : /180  → 133.33kHz */
 240,    /* 0x2 : /240  → 100 kHz  */
 360,    /* 0x3 : /360  → 66.67 kHz */
 480,    /* 0x4 : /480  → 50 kHz   */
 0,      /* 0x5 : reserved          */
 0,      /* 0x6 : reserved          */
 0,      /* 0x7 : reserved          */
 12000,  /* 0x8 : /12k  → 2 kHz    */
 24000,  /* 0x9 : /24k  → 1 kHz    */
 36000,  /* 0xA : /36k  → 666.67Hz */
 48000,  /* 0xB : /48k  → 500 Hz   */
 72000,  /* 0xC : /72k  → 333.33Hz */
 0,      /* 0xD : reserved          */
 0,      /* 0xE : reserved          */
 1,      /* 0xF : /1    → 24 MHz (bypass, valid on V3s) */
};

struct pwm_v3s_cfg {
	uint32_t base;
	const struct pinctrl_dev_config *pcfg;
};

struct pwm_v3s_data {};

static inline uint32_t pwm_v3s_read32(const struct pwm_v3s_cfg *cfg,
				      uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void pwm_v3s_write32(const struct pwm_v3s_cfg *cfg,
				   uint32_t offset, uint32_t value)
{
	sys_write32(value, cfg->base + offset);
}

static int pwm_v3s_calculate(uint32_t period_cycles, uint32_t pulse_cycles,
 		     uint8_t *prescaler_reg,
 		     uint32_t *entire_cyc, uint32_t *act_cyc)
{
 if (prescaler_table[0xF] != 0) {
 	uint32_t div_period = period_cycles / prescaler_table[0xF];

 	if (div_period >= 1 && div_period <= PWM_PRD_MAX) {
 		*prescaler_reg = 0xF;
 		*entire_cyc = div_period;
 		*act_cyc = pulse_cycles / prescaler_table[0xF];
 		return 0;
 	}
 }

 unsigned int psc;

 for (psc = 0; psc < 16; psc++) {
		uint32_t pval = prescaler_table[psc];

		if (pval == 0) {
			continue;  /* skip reserved entries (0x5–0x7, 0xD–0xE) */
		}

		/*
		 * period_cycles is expressed in 24 MHz ticks.
		 * After dividing by pval the result must fit in 16 bits
		 * and be at least 1 (otherwise the frequency is too high
		 * for this prescaler).
		 */
		uint32_t div_period = period_cycles / pval;

		if (div_period >= 1 && div_period <= PWM_PRD_MAX) {
			*prescaler_reg = (uint8_t)psc;
			*entire_cyc = div_period;
			*act_cyc = pulse_cycles / pval;
			return 0;
		}
	}

	return -EINVAL;
}

static int pwm_v3s_set_cycles(const struct device *dev, uint32_t channel,
			      uint32_t period_cycles, uint32_t pulse_cycles,
			      pwm_flags_t flags)
{
	const struct pwm_v3s_cfg *cfg = dev->config;
	uint32_t ctrl, prd_val;
	uint8_t presc_reg;
	uint32_t entire_cyc, act_cyc;
	int ret;

	if (channel >= PWM_NUM_CHANNELS) {
		LOG_ERR("invalid channel %u (max %u)", channel,
			PWM_NUM_CHANNELS - 1);
		return -EINVAL;
	}

	if (flags & ~(PWM_POLARITY_INVERTED)) {
		return -ENOTSUP;
	}

	ret = pwm_v3s_calculate(period_cycles, pulse_cycles,
				&presc_reg, &entire_cyc, &act_cyc);
	if (ret < 0) {
		LOG_ERR("cannot represent period=%u pulse=%u",
			period_cycles, pulse_cycles);
		return ret;
	}

	/* Ensure pulse <= period after prescaling */
	if (act_cyc > entire_cyc) {
		act_cyc = entire_cyc;
	}

	ctrl = pwm_v3s_read32(cfg, PWM_CTRL_REG);

	ctrl &= ~BIT_CH(PWM_BYPASS, channel);

	/* If prescaler is changing, gate the clock first */
	if ((ctrl & BIT_CH(PWM_PRESCAL_MASK, channel)) !=
	    (BIT_CH(presc_reg, channel) & BIT_CH(PWM_PRESCAL_MASK, channel))) {
		/* Disable clock gating before changing prescaler */
		ctrl &= ~BIT_CH(PWM_CLK_GATING, channel);
		pwm_v3s_write32(cfg, PWM_CTRL_REG, ctrl);

		/* Update prescaler value */
		ctrl &= ~BIT_CH(PWM_PRESCAL_MASK, channel);
		ctrl |= BIT_CH(presc_reg, channel);
	}

	prd_val = ((entire_cyc - 1) << 16) | (act_cyc & 0xFFFF);
	pwm_v3s_write32(cfg, PWM_CH_PRD(channel), prd_val);

	/* Set polarity */
	if ((flags & PWM_POLARITY_INVERTED) != 0) {
		ctrl &= ~BIT_CH(PWM_ACT_STATE, channel);
	} else {
		ctrl |= BIT_CH(PWM_ACT_STATE, channel);
	}

	ctrl &= ~BIT_CH(PWM_MODE, channel);

	ctrl |= BIT_CH(PWM_CLK_GATING, channel);
	ctrl |= BIT_CH(PWM_EN, channel);

	pwm_v3s_write32(cfg, PWM_CTRL_REG, ctrl);

	uint32_t ctrl_rb = pwm_v3s_read32(cfg, PWM_CTRL_REG);
	uint32_t prd_rb = pwm_v3s_read32(cfg, PWM_CH_PRD(channel));
	uint32_t ctrl_mask = ~(BIT(28) | BIT(29));

	LOG_DBG("ch=%u presc=0x%x entire=%u act=%u period=%u pulse=%u",
		channel, presc_reg, entire_cyc, act_cyc, period_cycles, pulse_cycles);
	LOG_DBG("  CTRL wr=0x%08x rb=0x%08x  PRD wr=0x%08x rb=0x%08x",
		ctrl, ctrl_rb, prd_val, prd_rb);

	if ((ctrl_rb & ctrl_mask) != (ctrl & ctrl_mask)) {
		LOG_ERR("CTRL mismatch! wrote=0x%08x read=0x%08x", ctrl, ctrl_rb);
	}
	if (prd_rb != prd_val) {
		LOG_ERR("PRD mismatch! wrote=0x%08x read=0x%08x", prd_val, prd_rb);
	}

	return 0;
}

static int pwm_v3s_get_cycles_per_sec(const struct device *dev,
				      uint32_t channel, uint64_t *cycles)
{
	if (channel >= PWM_NUM_CHANNELS) {
		return -EINVAL;
	}

	*cycles = PWM_SRC_CLK_FREQ;
	return 0;
}

static int pwm_v3s_init(const struct device *dev)
{
	const struct pwm_v3s_cfg *cfg = dev->config;
	int ret;

#ifdef CONFIG_PINCTRL
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl setup failed (%d)", ret);
		return ret;
	}
#endif

	pwm_v3s_write32(cfg, PWM_CTRL_REG, 0);
	for (uint32_t ch = 0; ch < PWM_NUM_CHANNELS; ch++) {
		pwm_v3s_write32(cfg, PWM_CH_PRD(ch), 0);
	}

	LOG_DBG("V3s PWM initialised at 0x%08x", cfg->base);
	return 0;
}

static DEVICE_API(pwm, pwm_v3s_api) = {
	.set_cycles = pwm_v3s_set_cycles,
	.get_cycles_per_sec = pwm_v3s_get_cycles_per_sec,
};

#define PWM_V3S_INIT(n)						       \
	PINCTRL_DT_INST_DEFINE(n);					       \
	static struct pwm_v3s_data pwm_v3s_data_##n;			       \
	static const struct pwm_v3s_cfg pwm_v3s_cfg_##n = {		       \
		.base = DT_INST_REG_ADDR(n),				       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		       \
	};								       \
	DEVICE_DT_INST_DEFINE(n,					       \
			      pwm_v3s_init,				       \
			      NULL,					       \
			      &pwm_v3s_data_##n,			       \
			      &pwm_v3s_cfg_##n,			       \
			      POST_KERNEL,				       \
			      CONFIG_PWM_INIT_PRIORITY,			       \
			      &pwm_v3s_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_V3S_INIT)
