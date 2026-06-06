/*
 * Copyright (c) 2026 jeck chen  <baidxi404629@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control_sun8i_v3s.h>
#include <zephyr/dt-bindings/clock/allwinner,sun8i-v3s-ccu.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT allwinner_sun8i_v3s_ccu

LOG_MODULE_REGISTER(ccu, CONFIG_LOG_DEFAULT_LEVEL);

#define FREQ_MIN_KHZ       200000
#define FREQ_MAX_KHZ       2600000
#define P_LOW_THRESH_KHZ   288000

#define CCU_PLL_CPU_CTRL          (0x000)
#define CCU_CPU_AXI_CFG           (0x050)
#define CCU_BUS_CLK_GATING_REG0   (0x060)
#define CCU_BUS_CLK_GATING_REG1   (0x064)
#define CCU_BUS_CLK_GATING_REG2   (0x068)

#define PLL_ENABLE               BIT(31)
#define PLL_LOCK                 BIT(28)
#define PLL_OUT_EN               BIT(24)
#define PLL_P_SHIFT							 (16)
#define PLL_P_MASK							 (0x3 << PLL_P_SHIFT)
#define PLL_N_SHIFT              (8)
#define PLL_N_MASK               (0x1F << PLL_N_SHIFT)
#define PLL_K_SHIFT              (4)
#define PLL_K_MASK               (0x3 << PLL_K_SHIFT)
#define PLL_M_SHIFT              (0)
#define PLL_M_MASK               (0x3 << PLL_M_SHIFT)

#define CPU_CLK_SRC_SEL_SHIFT    (16)
#define CPU_CLK_SRC_SEL_MASK     (0x3 << CPU_CLK_SRC_SEL_SHIFT)

#define CLK_GATE_BIT(clk_id)      ((clk_id) & 0x1F)
#define CLK_GATE_REG(clk_id)      (((clk_id) >> 5) & 0x3)


#define MMC_CLK_GATE_BIT		BIT(31)
#define MMC_CLK_SRC_SHIFT		24
#define MMC_CLK_SRC_HOSC		0	/* OSC24M  24 MHz */
#define MMC_CLK_SRC_PLL_PERIPH0	1	/* PLL_PERIPH0 600 MHz */
#define MMC_CLK_SRC_PLL_PERIPH1	2	/* PLL_PERIPH1 600 MHz */
#define MMC_CLK_N_SHIFT		16
#define MMC_CLK_M_SHIFT		0
#define MMC_CLK_MAX_OUTPUT		200000000U

#define CE_CLK_SRC_SHIFT	24
#define CE_CLK_SRC_MASK		GENMASK(26, 24)
#define CE_CLK_SRC_HOSC		0
#define CE_CLK_SRC_PLL_PERIPH0	1
#define CE_CLK_SRC_PLL_PERIPH1	2
#define CE_CLK_M_SHIFT		16
#define CE_CLK_M_MASK		GENMASK(17, 16)
#define CE_CLK_N_SHIFT		0
#define CE_CLK_N_MASK		GENMASK(1, 0)
#define CE_CLK_GATE		BIT(31)
#define CE_CLK_MAX_OUTPUT	150000000U

#define SPI0_CLK_SRC_SHIFT	24
#define SPI0_CLK_SRC_MASK	GENMASK(26, 24)
#define SPI0_CLK_SRC_HOSC	0
#define SPI0_CLK_M_SHIFT	16
#define SPI0_CLK_M_MASK		GENMASK(17, 16)
#define SPI0_CLK_N_SHIFT	0
#define SPI0_CLK_N_MASK		GENMASK(1, 0)
#define SPI0_CLK_GATE		BIT(31)
#define SPI0_CLK_PLL_BYPASS	BIT(30)  /* Bypass PLL lock when using OSC24M */

/* PLL_VIDEO (0x010): NM PLL for video/display clocks
 * Integer mode: PLL = 24MHz * N / M
 * Fractional mode (bit24=0): output = 297MHz (bit25=1) or 270MHz (bit25=0)
 * N: bits[14:8] (7-bit), M: bits[3:0] (4-bit)
 */
#define PLL_VIDEO_CTRL          0x010
#define PLL_VIDEO_EN            BIT(31)
#define PLL_VIDEO_LOCK          BIT(28)
#define PLL_VIDEO_MODE_SEL      BIT(24)
#define PLL_VIDEO_FRAC_SEL      BIT(25)
#define PLL_VIDEO_N_SHIFT       8
#define PLL_VIDEO_N_MASK        GENMASK(14, 8)
#define PLL_VIDEO_M_SHIFT       0
#define PLL_VIDEO_M_MASK        GENMASK(3, 0)

/* DE_CLK (0x104): divider + mux + gate
 * Source mux: 000=PLL_VIDEO, 010=PLL_PERIPH0
 */
#define DE_CLK_REG              0x104
#define DE_CLK_GATE             BIT(31)
#define DE_CLK_SRC_MASK         GENMASK(26, 24)
#define DE_CLK_SRC_PLL_VIDEO    0
#define DE_CLK_SRC_PLL_PERIPH0  2
#define DE_CLK_DIV_M_SHIFT      0
#define DE_CLK_DIV_M_MASK       GENMASK(3, 0)

/* TCON_CLK (0x118): divider + mux + gate
 * Source mux: 000=PLL_VIDEO, 001=PLL_PERIPH0  (DIFFERENT from DE!)
 */
#define TCON_CLK_REG            0x118
#define TCON_CLK_GATE           BIT(31)
#define TCON_CLK_SRC_MASK       GENMASK(26, 24)
#define TCON_CLK_SRC_PLL_VIDEO  0
#define TCON_CLK_SRC_PLL_PERIPH0 1
#define TCON_CLK_DIV_M_SHIFT    0
#define TCON_CLK_DIV_M_MASK     GENMASK(3, 0)


struct mmc_clk_parent {
	uint32_t src;	/* CLK_SRC_SEL value */
	uint32_t freq;	/* Parent frequency in Hz */
};

struct sun8i_osc_freq_config {
	uint32_t freq;
	uint8_t id;
	const char *name;
};

struct sun8i_ccu_config {
	uintptr_t base;
	const struct sun8i_osc_freq_config *osc;
	uint8_t num_clocks;
};

struct sun8i_clock_gate_info {
	uint16_t offset;
	uint32_t bit;
	uint32_t id;
};

struct sun8i_clock_info {
	uint32_t id;
	uint16_t offset;
	int (*set)(const struct device *dev, const struct sun8i_clock_info *info, void *data);
};

struct sun8i_ccu_data {
	struct k_spinlock lock;
	const struct sun8i_clock_gate_info *clock_gate_info;
	const struct sun8i_clock_info *clock_info;
	uint32_t nb_clock_info;
	uint32_t nb_clock_gate_info;
};

#define CLK_GATE(_id, _offset, _bit)	\
{ \
	.id = _id,	\
	.offset = _offset,	\
	.bit = _bit	\
}

#define CLK_PROP(_id, _offset, _set) \
{	\
	.id = _id, \
	.offset = _offset, \
	.set = _set,	\
}

static int _sun8i_set_clk_cpu(const struct device *dev, const struct sun8i_clock_info *info, void *data);
static int _sun8i_set_clk_spi0(const struct device *dev, const struct sun8i_clock_info *info, void *data);
static int _sun8i_set_clk_mmc(const struct device *dev, const struct sun8i_clock_info *info, void *data);
static int _sun8i_set_clk_ce(const struct device *dev, const struct sun8i_clock_info *info, void *data);
static int _sun8i_set_clk_de_tcon(const struct device *dev, const struct sun8i_clock_info *info, void *data);

static const struct sun8i_clock_info sun8i_v3s_clock_info[] = {
	CLK_PROP(CLK_CPU, 0, _sun8i_set_clk_cpu),
	CLK_PROP(CLK_MMC0, 0x88, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC0_SAMPLE, 0x88, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC0_OUTPUT, 0x88, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC1, 0x8c, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC1_SAMPLE, 0x8c, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC1_OUTPUT, 0x8c, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC2, 0x90, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC2_SAMPLE, 0x90, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_MMC2_OUTPUT, 0x90, _sun8i_set_clk_mmc),
	CLK_PROP(CLK_CE, 0x9c, _sun8i_set_clk_ce),
	CLK_PROP(CLK_SPI0, 0xa0, _sun8i_set_clk_spi0),
	CLK_PROP(CLK_DE, DE_CLK_REG, _sun8i_set_clk_de_tcon),
	CLK_PROP(CLK_TCON0, TCON_CLK_REG, _sun8i_set_clk_de_tcon),
};

static const struct sun8i_clock_gate_info sun8i_clock_gate_info[] = {
	CLK_GATE(CLK_BUS_CE, 0x60, 5),
	CLK_GATE(CLK_BUS_DMA, 0x60, 6),
	CLK_GATE(CLK_BUS_MMC0, 0x60, 8),
	CLK_GATE(CLK_BUS_MMC1, 0x60, 9),
	CLK_GATE(CLK_BUS_MMC2, 0x60, 10),
	CLK_GATE(CLK_BUS_DRAM, 0x60, 14),
	CLK_GATE(CLK_BUS_EMAC, 0x60, 17),
	CLK_GATE(CLK_BUS_HSTIMER, 0x60, 19),
	CLK_GATE(CLK_BUS_SPI0, 0x60, 20),
	CLK_GATE(CLK_BUS_OTG, 0x60, 24),
	CLK_GATE(CLK_BUS_EHCI0, 0x60, 26),
	CLK_GATE(CLK_BUS_OHCI0, 0x60, 29),
	CLK_GATE(CLK_BUS_VE, 0x64, 0),
	CLK_GATE(CLK_BUS_TCON0, 0x64, 4),
	CLK_GATE(CLK_BUS_CSI, 0x64, 8),
	CLK_GATE(CLK_BUS_DE, 0x64, 12),
	CLK_GATE(CLK_BUS_CODEC, 0x68, 0),
	CLK_GATE(CLK_BUS_PIO, 0x68, 5),
	CLK_GATE(CLK_BUS_I2C0, 0x6c, 0),
	CLK_GATE(CLK_BUS_I2C1, 0x6c, 1),
	CLK_GATE(CLK_BUS_UART0, 0x6c, 16),
	CLK_GATE(CLK_BUS_UART1, 0x6c, 17),
	CLK_GATE(CLK_BUS_UART2, 0x6c, 18),
	CLK_GATE(CLK_BUS_EPHY, 0x70, 0),
	CLK_GATE(CLK_BUS_DBG, 0x70, 7),
	CLK_GATE(CLK_USB_PHY0, 0xcc, 8),
	CLK_GATE(CLK_USB_OHCI0, 0xcc, 16),
};

/*
 * Module clock enable/disable: use bit 31 (CLK_GATE) at the clock register.
 * Gate clocks use a per-bit offset in gating registers.
 */
#define MOD_CLK_GATE	BIT(31)

static int sun8i_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct sun8i_ccu_config *config = dev->config;
	struct sun8i_ccu_data *data = dev->data;
	uint32_t clk_id = (uint32_t)sys;
	uint32_t offset = 0;
	uint32_t bit = 0;
	bool found = false;
	int i;
	uint32_t regval;
	k_spinlock_key_t key;

	/* Search gate clocks (CLK_GATE) first */
	const struct sun8i_clock_gate_info *gate = data->clock_gate_info;

	for (i = 0; i < data->nb_clock_gate_info; i++, gate++) {
		if (gate->id == clk_id) {
			offset = gate->offset;
			bit = gate->bit;
			found = true;
			break;
		}
	}

	/* If not a gate clock, search module clocks (CLK_PROP) */
	if (!found) {
		const struct sun8i_clock_info *mod = data->clock_info;

		for (i = 0; i < data->nb_clock_info; i++, mod++) {
			if (mod->id == clk_id) {
				offset = mod->offset;
				bit = 31; /* MOD_CLK_GATE */
				found = true;
				break;
			}
		}
	}

	if (!found) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + offset);
	regval |= BIT(bit);
	sys_write32(regval, config->base + offset);
	LOG_DBG("clk_on: off=0x%x bit=%u val=0x%08x",
		offset, bit, sys_read32(config->base + offset));
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int sun8i_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct sun8i_ccu_config *config = dev->config;
	struct sun8i_ccu_data *data = dev->data;
	uint32_t clk_id = (uint32_t)sys;
	uint32_t offset = 0;
	uint32_t bit = 0;
	bool found = false;
	int i;
	uint32_t regval;
	k_spinlock_key_t key;

	/* Search gate clocks first */
	const struct sun8i_clock_gate_info *gate = data->clock_gate_info;

	for (i = 0; i < data->nb_clock_gate_info; i++, gate++) {
		if (gate->id == clk_id) {
			offset = gate->offset;
			bit = gate->bit;
			found = true;
			break;
		}
	}

	/* If not a gate clock, search module clocks */
	if (!found) {
		const struct sun8i_clock_info *mod = data->clock_info;

		for (i = 0; i < data->nb_clock_info; i++, mod++) {
			if (mod->id == clk_id) {
				offset = mod->offset;
				bit = 31;
				found = true;
				break;
			}
		}
	}

	if (!found) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + offset);
	regval &= ~BIT(bit);
	sys_write32(regval, config->base + offset);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static inline uint32_t pll_freq_khz(uint32_t ref_freq, uint32_t n_reg, uint32_t k_reg,
                                    uint32_t m_reg, uint32_t p_reg)
{
    uint32_t N = n_reg + 1;
    uint32_t K = k_reg + 1;
    uint32_t M = m_reg + 1;
    uint32_t P = (p_reg == 0) ? 1 : (p_reg == 1 ? 2 : 4);
    uint64_t freq = (uint64_t)ref_freq * N * K / (M * P);
    return (uint32_t)freq;
}

uint32_t find_best_cpu_pll(uint32_t ref_freq, uint32_t target_freq_khz,
                           uint32_t *best_n, uint32_t *best_k,
                           uint32_t *best_m, uint32_t *best_p)
{
    uint32_t min_error = UINT32_MAX;
    uint32_t fitted_freq = 0;
    *best_n = *best_k = *best_m = *best_p = 0;
    for (uint32_t n_reg = 0; n_reg <= 31; n_reg++) {
        for (uint32_t k_reg = 0; k_reg <= 3; k_reg++) {
            for (uint32_t m_reg = 0; m_reg <= 3; m_reg++) {
                for (uint32_t p_reg = 0; p_reg <= 2; p_reg++) {
                    uint32_t freq = pll_freq_khz(ref_freq, n_reg, k_reg, m_reg, p_reg);
                    if (freq < FREQ_MIN_KHZ || freq > FREQ_MAX_KHZ)
                        continue;
                    if (p_reg != 0 && freq >= P_LOW_THRESH_KHZ)
                        continue;
                    uint32_t error = (freq > target_freq_khz) ?
                                     (freq - target_freq_khz) :
                                     (target_freq_khz - freq);
                    if (error < min_error) {
                        min_error = error;
                        fitted_freq = freq;
                        *best_n = n_reg;
                        *best_k = k_reg;
                        *best_m = m_reg;
                        *best_p = p_reg;
                        if (error == 0) break;
                    }
                }
            }
        }
    }
    return (min_error == UINT32_MAX) ? 0 : fitted_freq;
}

static int sun8i_pll_cpu_set_rate(uint32_t base, uint32_t input_freq, uint32_t target_freq)
{
	uint32_t n, k, m, p;
	uint32_t reg_value;
	uint32_t timeout = 1000;
	int ret;

	ret = find_best_cpu_pll(input_freq, target_freq, &n, &k, &m, &p);

	reg_value = sys_read32(base + CCU_PLL_CPU_CTRL);
	reg_value &= ~PLL_ENABLE;
	sys_write32(reg_value, base + CCU_PLL_CPU_CTRL);

	reg_value = (p << PLL_P_SHIFT) | (n << PLL_N_SHIFT) | (k << PLL_K_SHIFT) | m;
	sys_write32(reg_value, base + CCU_PLL_CPU_CTRL);

	reg_value |= PLL_ENABLE | PLL_OUT_EN;
	sys_write32(reg_value, base + CCU_PLL_CPU_CTRL);

	while (!(sys_read32(base + CCU_PLL_CPU_CTRL) & PLL_LOCK)) {
		if (--timeout == 0) {
			LOG_ERR("PLL CPU lock timeout");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int sun8i_cpu_clk_set_rate(uint32_t base, uint32_t input_freq, uint32_t target_freq)
{
	uint32_t reg_value;

	int ret = sun8i_pll_cpu_set_rate(base, input_freq, target_freq);
	if (ret < 0) {
		return ret;
	}

	reg_value = sys_read32(base + CCU_CPU_AXI_CFG);
	reg_value &= ~(CPU_CLK_SRC_SEL_MASK);
	reg_value |= 2 << CPU_CLK_SRC_SEL_SHIFT;

	sys_write32(reg_value, base + CCU_CPU_AXI_CFG);

	return 0;
}

static int sun8i_ccu_set_cpu_clock(const struct device *dev, uint32_t target_freq)
{
	const struct sun8i_ccu_config *config;
	const struct sun8i_osc_freq_config *osc;
	int i;
	uint32_t reg_val;

	LOG_INF("%s", __func__);

	config = dev->config;
	for (i = 0; i < config->num_clocks; i++)
	{
		osc = &config->osc[i];
		if (strcmp("hosc", osc->name) == 0)
		{
			reg_val = sys_read32(config->base + CCU_CPU_AXI_CFG);
			reg_val &= ~(CPU_CLK_SRC_SEL_MASK);
			reg_val |= 1 << CPU_CLK_SRC_SEL_SHIFT;
			sys_write32(reg_val, config->base + CCU_CPU_AXI_CFG);
			/* Convert Hz to KHz for internal functions */
			return sun8i_cpu_clk_set_rate(config->base,
				osc->freq / 1000, target_freq / 1000);
		}
	}

	return 0;
}

static int _sun8i_set_clk_cpu(const struct device *dev, const struct sun8i_clock_info *info, void *data)
{
	uint32_t freq = *(uint32_t *)data;

	if (freq)
		return sun8i_ccu_set_cpu_clock(dev, freq);
	return -EINVAL;
}

static int _sun8i_set_clk_spi0(const struct device *dev,
				const struct sun8i_clock_info *info, void *data)
{
	const struct sun8i_ccu_config *ccu_cfg = dev->config;
	const struct sun8i_spi_clk_config *clk_cfg = data;
	uint32_t parent_freq;
	uint32_t target_freq = clk_cfg->output_freq;

	switch (clk_cfg->src) {
	case SUN8I_SPI_CLK_SRC_HOSC:
		parent_freq = 24000000;
		break;
	case SUN8I_SPI_CLK_SRC_PLL_PERIPH0:
	case SUN8I_SPI_CLK_SRC_PLL_PERIPH1:
		parent_freq = 600000000;
		break;
	default:
		return -EINVAL;
	}
	uint32_t best_m = 0, best_n = 0;
	uint32_t best_diff = UINT32_MAX;
	uint32_t best_actual = 0;
	uint32_t reg;

	if (target_freq == 0 || parent_freq == 0) {
		return -EINVAL;
	}

	/* Try all M (0..3) and N (0..3) divider combinations */
	for (int m = 0; m <= 3; m++) {
		for (int n = 0; n <= 3; n++) {
			uint32_t div = (1U << m) * (1U << n);
			uint32_t actual = parent_freq / div;

			if (actual == 0) {
				continue;
			}

			uint32_t diff = (actual > target_freq) ?
				(actual - target_freq) :
				(target_freq - actual);

			if (diff < best_diff) {
				best_diff = diff;
				best_actual = actual;
				best_m = m;
				best_n = n;
			}
		}
	}

	if (best_actual == 0) {
		return -EINVAL;
	}

	/* Build register: source=HOSC, dividers, enable gate */
	reg = (SPI0_CLK_SRC_HOSC << SPI0_CLK_SRC_SHIFT) |
	      (best_m << SPI0_CLK_M_SHIFT) |
	      (best_n << SPI0_CLK_N_SHIFT) |
	      SPI0_CLK_GATE;

	LOG_DBG("SPI0 clk: base=0x%lx offset=0x%lx addr=0x%lx val=0x%08x",
		(unsigned long)ccu_cfg->base, (unsigned long)info->offset,
		(unsigned long)(ccu_cfg->base + info->offset), reg);
	sys_write32(reg, ccu_cfg->base + info->offset);

	return 0;
}

static int _sun8i_set_clk_ce(const struct device *dev,
			     const struct sun8i_clock_info *info, void *data)
{
	const struct sun8i_ccu_config *ccu_cfg = dev->config;
	uint32_t target_freq = (uint32_t)(uintptr_t)data;
	uint32_t reg, best_src, best_m, best_n, best_actual, best_diff;
	uint32_t actual, diff, after_m;
	int s, m, n;

	struct {
		uint32_t src;
		uint32_t freq;
	} static const ce_parents[] = {
		{ CE_CLK_SRC_HOSC,        24000000 },
		{ CE_CLK_SRC_PLL_PERIPH0, 600000000 },
		{ CE_CLK_SRC_PLL_PERIPH1, 600000000 },
	};

	if (target_freq == 0) {
		/* No frequency specified: enable with HOSC 24 MHz, divider /1 */
		reg = (CE_CLK_SRC_HOSC << CE_CLK_SRC_SHIFT) | CE_CLK_GATE;
		sys_write32(reg, ccu_cfg->base + info->offset);
		return 0;
	}

	/* Clamp to maximum output */
	if (target_freq > CE_CLK_MAX_OUTPUT) {
		target_freq = CE_CLK_MAX_OUTPUT;
	}

	best_src = CE_CLK_SRC_HOSC;
	best_m = 0;
	best_n = 0;
	best_actual = 0;
	best_diff = UINT32_MAX;

	for (s = 0; s < (int)ARRAY_SIZE(ce_parents); s++) {
		uint32_t parent = ce_parents[s].freq;

		for (m = 0; m <= 3; m++) {
			after_m = parent >> m;
			if (after_m == 0) {
				continue;
			}
			for (n = 0; n <= 3; n++) {
				actual = after_m >> n;
				if (actual == 0 || actual > CE_CLK_MAX_OUTPUT) {
					continue;
				}
				diff = (actual > target_freq) ?
					(actual - target_freq) :
					(target_freq - actual);
				if (diff < best_diff) {
					best_diff = diff;
					best_actual = actual;
					best_src = ce_parents[s].src;
					best_m = m;
					best_n = n;
				}
			}
		}
	}

	if (best_actual == 0) {
		reg = (CE_CLK_SRC_HOSC << CE_CLK_SRC_SHIFT) | CE_CLK_GATE;
		sys_write32(reg, ccu_cfg->base + info->offset);
		LOG_WRN("CE clk: no valid divider for %u Hz, fallback to HOSC",
			target_freq);
		return -EINVAL;
	}

	reg = (best_src << CE_CLK_SRC_SHIFT) |
	      (best_m << CE_CLK_M_SHIFT) |
	      (best_n << CE_CLK_N_SHIFT) |
	      CE_CLK_GATE;

	LOG_DBG("CE clk: target=%u src=%u m=%u n=%u actual=%u reg=0x%08x",
		target_freq, best_src, best_m, best_n, best_actual, reg);

	sys_write32(reg, ccu_cfg->base + info->offset);
	return 0;
}

/*
 * Configure PLL_VIDEO in integer mode for a target pixel clock.
 *
 * Searches for PLL_VIDEO N, M_pll values such that:
 *   F_pll = 24 MHz × (N+1) / (M_pll+1)
 *   F_tcon = F_pll / (M_tcon+1) ≈ target_freq
 *
 * Prefers M_pll=0 (divide by 1) for the cleanest clock output.
 * Enforces a minimum PLL VCO frequency of 200 MHz.
 *
 * Returns the actual PLL_VIDEO frequency, or 0 on failure.
 */
#define PLL_VIDEO_MIN_FREQ	200000000U

static uint32_t configure_pll_video_for_target(uintptr_t ccu_base,
					       uint32_t target_freq)
{
	uint32_t best_n = 0, best_m_pll = 0;
	uint32_t best_pll_freq = 0;
	bool found_exact = false;

	/* Pass 1: exact match with M_pll = 0 (divide by 1, cleanest).
	 * Accept the first valid match (lowest tcon_m → lowest PLL freq
	 * above minimum).  Lower PLL frequencies are more stable.
	 */
	for (uint32_t tcon_m = 0; tcon_m <= 15 && !found_exact; tcon_m++) {
		uint64_t pll64 = (uint64_t)target_freq * (tcon_m + 1);

		if (pll64 % 24000000U == 0) {
			uint32_t n_val = (uint32_t)(pll64 / 24000000U);

			if (n_val >= 1 && n_val <= 128
			    && (uint32_t)pll64 >= PLL_VIDEO_MIN_FREQ) {
				best_n = n_val - 1;
				best_m_pll = 0;
				best_pll_freq = (uint32_t)pll64;
				found_exact = true;
			}
		}
	}

	/* Pass 2: exact match with M_pll = 1..3 */
	if (!found_exact) {
		for (uint32_t tcon_m = 0; tcon_m <= 15 && !found_exact;
		     tcon_m++) {
			for (uint32_t m_pll = 1; m_pll <= 3 && !found_exact;
			     m_pll++) {
				uint64_t num = (uint64_t)target_freq
					     * (tcon_m + 1) * (m_pll + 1);

				if (num % 24000000U == 0) {
					uint32_t n_val = (uint32_t)(num / 24000000U);
					uint32_t pll_freq = 24000000U * n_val
							  / (m_pll + 1);

					if (n_val >= 1 && n_val <= 128
					    && pll_freq >= PLL_VIDEO_MIN_FREQ) {
						best_n = n_val - 1;
						best_m_pll = m_pll;
						best_pll_freq = pll_freq;
						found_exact = true;
					}
				}
			}
		}
	}

	/* Pass 3: best approximate match with M_pll = 0 */
	if (!found_exact) {
		uint32_t best_err = UINT32_MAX;

		for (uint32_t tcon_m = 0; tcon_m <= 15; tcon_m++) {
			for (uint32_t n_val = 1; n_val <= 128; n_val++) {
				uint32_t pll_freq = 24000000U * n_val;
				uint32_t actual = pll_freq / (tcon_m + 1);
				uint32_t err = (actual > target_freq)
					     ? (actual - target_freq)
					     : (target_freq - actual);

				if (err < best_err) {
					best_err = err;
					best_n = n_val - 1;
					best_m_pll = 0;
					best_pll_freq = pll_freq;
				}
			}
		}
	}

	if (best_pll_freq == 0) {
		return 0;
	}

	/* Configure PLL_VIDEO */
	uint32_t reg;

	/* Disable PLL (required for reconfiguration) */
	reg = sys_read32(ccu_base + PLL_VIDEO_CTRL);
	reg &= ~PLL_VIDEO_EN;
	sys_write32(reg, ccu_base + PLL_VIDEO_CTRL);

	/* Set integer mode with N, M values */
	reg = PLL_VIDEO_MODE_SEL
	    | (best_n << PLL_VIDEO_N_SHIFT)
	    | (best_m_pll << PLL_VIDEO_M_SHIFT);
	sys_write32(reg, ccu_base + PLL_VIDEO_CTRL);

	/* Enable PLL */
	reg |= PLL_VIDEO_EN;
	sys_write32(reg, ccu_base + PLL_VIDEO_CTRL);

	/* Wait for PLL lock */
	int timeout = 10000;

	while (!(sys_read32(ccu_base + PLL_VIDEO_CTRL) & PLL_VIDEO_LOCK)) {
		if (--timeout == 0) {
			break;
		}
	}

	LOG_INF("PLL_VIDEO: integer mode N=%u M=%u → %u Hz",
		best_n + 1, best_m_pll + 1, best_pll_freq);

	return best_pll_freq;
}

/*
 * DE and TCON clocks: similar register layout but DIFFERENT source mux mapping!
 *   DE_CLK  (0x104): bits[26:24] = 000:PLL_VIDEO, 010:PLL_PERIPH0
 *   TCON_CLK(0x118): bits[26:24] = 000:PLL_VIDEO, 001:PLL_PERIPH0
 *   bits[31]: gate, bits[3:0]: M divider (0→/1 ... 15→/16)
 */
static int _sun8i_set_clk_de_tcon(const struct device *dev,
				  const struct sun8i_clock_info *info, void *data)
{
	const struct sun8i_ccu_config *ccu_cfg = dev->config;
	uint32_t target_freq = (uint32_t)(uintptr_t)data;
	uint32_t reg, best_m, best_actual, best_diff;
	uint32_t parent_freqs[] = { 0, 0 };
	int best_src_type = 0;  /* 0=pll-video, 1=pll-periph0 */
	bool is_tcon = (info->offset == TCON_CLK_REG);
	int i;

	/*
	 * PLL_VIDEO was already configured during CCU init (PRE_KERNEL_1)
	 * based on the panel's pixel clock from the device tree.
	 * Just read the current PLL register to determine the frequency.
	 */
	{
		reg = sys_read32(ccu_cfg->base + PLL_VIDEO_CTRL);
		if (reg & PLL_VIDEO_EN) {
			if (reg & PLL_VIDEO_MODE_SEL) {
				uint32_t n = ((reg >> PLL_VIDEO_N_SHIFT) & 0x7f) + 1;
				uint32_t m = ((reg >> PLL_VIDEO_M_SHIFT) & 0xf) + 1;

				parent_freqs[0] = 24000000U * n / m;
				LOG_INF("PLL_VIDEO integer mode: n=%u m=%u freq=%u",
					n, m, parent_freqs[0]);
			} else {
				parent_freqs[0] = (reg & PLL_VIDEO_FRAC_SEL)
						  ? 297000000U : 270000000U;
				LOG_INF("PLL_VIDEO fractional: freq=%u",
					parent_freqs[0]);
			}
		} else {
			parent_freqs[0] = 297000000U;
			LOG_WRN("PLL_VIDEO NOT enabled, assuming %u Hz",
				parent_freqs[0]);
		}
	}
	parent_freqs[1] = 600000000U;  /* pll-periph0 */

	if (target_freq == 0) {
		/* No target: just enable gate with current settings */
		reg = sys_read32(ccu_cfg->base + info->offset);
		reg |= BIT(31);
		sys_write32(reg, ccu_cfg->base + info->offset);
		return 0;
	}

	best_actual = 0;
	best_m = 0;
	best_diff = UINT32_MAX;

	/* Try both parent sources */
	for (i = 0; i < 2; i++) {
		if (i == 0 && parent_freqs[0] == 0) {
			continue;  /* skip pll-video if not available */
		}

		/* M divider range: 0-15 (divide by m+1) */
		for (uint32_t m = 0; m <= 15; m++) {
			uint32_t actual = parent_freqs[i] / (m + 1);

			if (actual == 0) {
				continue;
			}

			uint32_t diff = (actual > target_freq) ?
				(actual - target_freq) :
				(target_freq - actual);

			if (diff < best_diff || (diff == best_diff && actual > best_actual)) {
				best_diff = diff;
				best_actual = actual;
				best_m = m;
				best_src_type = i;  /* 0=pll-video, 1=pll-periph0 */
			}
		}
	}

	if (best_actual == 0) {
		LOG_ERR("DE/TCON clk: no valid divider for %u Hz", target_freq);
		return -EINVAL;
	}

	/* Map abstract source type to register-specific source select value */
	uint32_t src_reg_val;
	if (is_tcon) {
		src_reg_val = (best_src_type == 0)
			? TCON_CLK_SRC_PLL_VIDEO
			: TCON_CLK_SRC_PLL_PERIPH0;  /* TCON: 001 */
	} else {
		src_reg_val = (best_src_type == 0)
			? DE_CLK_SRC_PLL_VIDEO
			: DE_CLK_SRC_PLL_PERIPH0;    /* DE:   010 */
	}

	reg = (src_reg_val << 24) | (best_m << 0) | BIT(31);

	LOG_INF("%s clk offset=0x%lx: target=%u src_type=%d src_reg=%u m=%u actual=%u reg=0x%08x",
		is_tcon ? "TCON" : "DE",
		(unsigned long)info->offset, target_freq, best_src_type,
		src_reg_val, best_m, best_actual, reg);

	sys_write32(reg, ccu_cfg->base + info->offset);

	/* Read back and verify */
	reg = sys_read32(ccu_cfg->base + info->offset);
	LOG_INF("  readback: 0x%08x", reg);

	return 0;
}

static const struct mmc_clk_parent mmc_parents[] = {
	{ MMC_CLK_SRC_HOSC,		24000000 },
	{ MMC_CLK_SRC_PLL_PERIPH0,	600000000 },
	{ MMC_CLK_SRC_PLL_PERIPH1,	600000000 },
};

static int _sun8i_set_clk_mmc(const struct device *dev,
			      const struct sun8i_clock_info *info, void *data)
{
	const struct sun8i_ccu_config *ccu_cfg = dev->config;
	uint32_t target_freq = (uint32_t)(uintptr_t)data;
	uint32_t reg, best_src, best_n, best_m, best_actual, best_diff;
	uint32_t actual, diff, after_n;
	int s, n, m;

	if (target_freq == 0) {
		/* No frequency specified: enable with HOSC 24 MHz, divider /1 */
		reg = MMC_CLK_GATE_BIT;
		sys_write32(reg, ccu_cfg->base + info->offset);
		return 0;
	}

	/* Clamp to maximum output */
	if (target_freq > MMC_CLK_MAX_OUTPUT) {
		target_freq = MMC_CLK_MAX_OUTPUT;
	}

	/* Search all source/N/M combinations for closest match */
	best_src = MMC_CLK_SRC_HOSC;
	best_n = 0;
	best_m = 0;
	best_actual = 0;
	best_diff = UINT32_MAX;

	for (s = 0; s < (int)ARRAY_SIZE(mmc_parents); s++) {
		uint32_t parent = mmc_parents[s].freq;

		for (n = 0; n <= 3; n++) {
			after_n = parent >> n;
			if (after_n == 0) {
				continue;
			}
			for (m = 0; m <= 15; m++) {
				actual = after_n / (m + 1);
				if (actual == 0 || actual > MMC_CLK_MAX_OUTPUT) {
					continue;
				}
				diff = (actual > target_freq) ?
					(actual - target_freq) :
					(target_freq - actual);
				if (diff < best_diff) {
					best_diff = diff;
					best_actual = actual;
					best_src = mmc_parents[s].src;
					best_n = n;
					best_m = m;
				}
			}
		}
	}

	if (best_actual == 0) {
		/* Fall back to HOSC /1 */
		reg = MMC_CLK_GATE_BIT;
		sys_write32(reg, ccu_cfg->base + info->offset);
		LOG_WRN("MMC clk: no valid divider for %u Hz, fallback to HOSC",
			target_freq);
		return -EINVAL;
	}

	reg = (best_src << MMC_CLK_SRC_SHIFT) |
	      (best_n << MMC_CLK_N_SHIFT) |
	      (best_m << MMC_CLK_M_SHIFT) |
	      MMC_CLK_GATE_BIT;

	LOG_DBG("MMC clk: target=%u src=%u n=%u m=%u actual=%u reg=0x%08x",
		target_freq, best_src, best_n, best_m, best_actual, reg);

	sys_write32(reg, ccu_cfg->base + info->offset);
	return 0;
}

static int sun8i_set_clk_cpu(const struct device *dev,
				 clock_control_subsys_t sys,
				 clock_control_subsys_rate_t rate)
{
	struct sun8i_ccu_data *data = dev->data;
	const struct sun8i_clock_info *info = data->clock_info;
	uint32_t clk_id = (uint32_t)sys;
	int i;
	bool found = false;

	for (i = 0; i < data->nb_clock_info; i++, info++)
	{
		if (info->id == clk_id)
		{
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	if (info->set)
		return info->set(dev, info, rate);

	return -EOPNOTSUPP;
}

static int sun8i_get_rate(const struct device *dev,
			  clock_control_subsys_t sys,
			  uint32_t *rate)
{
	const struct sun8i_ccu_config *ccu_cfg = dev->config;
	struct sun8i_ccu_data *ccu_data = dev->data;
	const struct sun8i_clock_info *info = ccu_data->clock_info;
	uint32_t clk_id = (uint32_t)sys;
	int i;
	bool found = false;

	for (i = 0; i < (int)ccu_data->nb_clock_info; i++, info++) {
		if (info->id == clk_id) {
			found = true;
			break;
		}
	}

	if (!found) {
		return -EINVAL;
	}

	/* MMC clocks: read register and compute output rate */
	if (info->set == _sun8i_set_clk_mmc) {
		uint32_t reg = sys_read32(ccu_cfg->base + info->offset);
		uint32_t src_sel = (reg >> MMC_CLK_SRC_SHIFT) & 0x3;
		uint32_t n = (reg >> MMC_CLK_N_SHIFT) & 0x3;
		uint32_t m = (reg >> MMC_CLK_M_SHIFT) & 0xf;
		uint32_t parent_freq;

		switch (src_sel) {
		case MMC_CLK_SRC_HOSC:
			parent_freq = 24000000;
			break;
		case MMC_CLK_SRC_PLL_PERIPH0:
			parent_freq = 600000000;
			break;
		case MMC_CLK_SRC_PLL_PERIPH1:
			parent_freq = 600000000;
			break;
		default:
			return -EIO;
		}

		*rate = (parent_freq >> n) / (m + 1);
		return 0;
	}

	/* DE and TCON clocks: read register and compute output rate */
	if (info->set == _sun8i_set_clk_de_tcon) {
		uint32_t reg = sys_read32(ccu_cfg->base + info->offset);
		uint32_t src = (reg >> 24) & 0x7;
		uint32_t m = reg & 0xf;
		uint32_t parent_freq;
		bool is_tcon = (info->offset == TCON_CLK_REG);

		/* Check source based on register-specific mapping:
		 * DE:   0=PLL_VIDEO, 2=PLL_PERIPH0
		 * TCON: 0=PLL_VIDEO, 1=PLL_PERIPH0
		 */
		bool is_pll_video = (src == 0);
		bool is_pll_periph0 = is_tcon
			? (src == TCON_CLK_SRC_PLL_PERIPH0)
			: (src == DE_CLK_SRC_PLL_PERIPH0);

		if (is_pll_video) {
			uint32_t pll_reg = sys_read32(ccu_cfg->base + PLL_VIDEO_CTRL);
			if (pll_reg & PLL_VIDEO_EN) {
				if (pll_reg & PLL_VIDEO_MODE_SEL) {
					/* Integer mode */
					uint32_t n = ((pll_reg >> PLL_VIDEO_N_SHIFT) & 0x7f) + 1;
					uint32_t div_m = ((pll_reg >> PLL_VIDEO_M_SHIFT) & 0xf) + 1;
					parent_freq = 24000000U * n / div_m;
				} else {
					/* Fractional mode */
					parent_freq = (pll_reg & PLL_VIDEO_FRAC_SEL)
						      ? 297000000U : 270000000U;
				}
			} else {
				parent_freq = 297000000U;
			}
		} else if (is_pll_periph0) {
			parent_freq = 600000000U;
		} else {
			LOG_WRN("Unknown %s clock source %u at offset 0x%x",
				is_tcon ? "TCON" : "DE", src, info->offset);
			return -EIO;
		}

		*rate = parent_freq / (m + 1);
		return 0;
	}

	return -ENOSYS;
}

DEVICE_API(clock_control, sun8i_clock_control_api) = {
	.on = sun8i_clock_on,
	.off = sun8i_clock_off,
	.get_rate = sun8i_get_rate,
	.set_rate = sun8i_set_clk_cpu,
};

static int sun8i_ccu_init(const struct device *dev)
{
	const struct sun8i_ccu_config *ccu_cfg = dev->config;

	/*
	 * Configure PLL_VIDEO in integer mode based on the panel's
	 * pixel clock from the device tree.  This must be done early
	 * (during CCU init, PRE_KERNEL_1) so that PLL_VIDEO is stable
	 * before any display driver (DE2, TCON) starts using it.
	 *
	 * The algorithm finds the optimal integer-mode PLL frequency
	 * and TCON M-divider to produce an exact pixel clock match.
	 */
#if DT_NODE_EXISTS(DT_CHILD(DT_NODELABEL(tcon0), display_timings))
	uint32_t pixel_clock = DT_PROP(
		DT_CHILD(DT_NODELABEL(tcon0), display_timings),
		clock_frequency);

	if (pixel_clock > 0) {
		uint32_t pll_freq = configure_pll_video_for_target(
			ccu_cfg->base, pixel_clock);

		if (pll_freq > 0) {
			LOG_INF("PLL_VIDEO configured for panel: %u Hz pixel clock → %u Hz PLL",
				pixel_clock, pll_freq);
		} else {
			LOG_WRN("PLL_VIDEO auto-config failed for %u Hz pixel clock",
				pixel_clock);
		}
	}
#else
	LOG_INF("No panel timings in DT, PLL_VIDEO not auto-configured");
#endif

	return 0;
}

#define CCU_NODE DT_NODELABEL(ccu)
#define CLOCKS_COUNT	DT_NUM_CLOCKS(CCU_NODE)
#define CLOCK_ENTRIES               \
    X(0, DT_NODELABEL(osc24m))     \
    X(1, DT_NODELABEL(osc32k)) 

#define X(idx, clk_node)                                    \
    {                                                       \
        .freq = DT_PROP(clk_node, clock_frequency),         \
        .id   = idx,                                        \
        .name = DT_PROP_BY_IDX(CCU_NODE, clock_names, idx), \
    },
static const struct sun8i_osc_freq_config sun8i_osc_config[] = {
    CLOCK_ENTRIES
};
#undef X

#define SUN8I_CCU_INIT(n) \
	static const struct sun8i_ccu_config sun8i_ccu_config_##n = { \
		.base = DT_INST_REG_ADDR(n), \
		.osc = sun8i_osc_config, \
		.num_clocks = CLOCKS_COUNT,	\
	}; \
	\
	static struct sun8i_ccu_data sun8i_ccu_data_##n = {	\
		.clock_gate_info = sun8i_clock_gate_info,	\
		.nb_clock_gate_info = ARRAY_SIZE(sun8i_clock_gate_info),	\
		.clock_info = sun8i_v3s_clock_info,	\
		.nb_clock_info = ARRAY_SIZE(sun8i_v3s_clock_info),	\
	}; \
	\
	DEVICE_DT_INST_DEFINE(n, sun8i_ccu_init, NULL, \
				&sun8i_ccu_data_##n, &sun8i_ccu_config_##n, \
				PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
				&sun8i_clock_control_api);

DT_INST_FOREACH_STATUS_OKAY(SUN8I_CCU_INIT)
