/*
 * Copyright (c) 2026 juno <baidxi404629@gmail.com>
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

static const struct sun8i_clock_info sun8i_v3s_clock_info[] = {
	CLK_PROP(CLK_CPU, 0, _sun8i_set_clk_cpu),
	CLK_PROP(CLK_MMC0, 0x88, 0),
	CLK_PROP(CLK_MMC1, 0x8c, 0),
	CLK_PROP(CLK_MMC2, 0x90, 0),
	CLK_PROP(CLK_CE, 0x9c, 0),
	CLK_PROP(CLK_SPI0, 0xa0, _sun8i_set_clk_spi0),
	CLK_PROP(CLK_USB_PHY0, 0xcc, 0),
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
};

static int sun8i_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct sun8i_ccu_config *config = dev->config;
	struct sun8i_ccu_data *data = dev->data;
	const struct sun8i_clock_gate_info *info = data->clock_gate_info;
	uint32_t clk_id = (uint32_t)sys;
	bool found = false;
	int i;
	uint32_t regval;
	k_spinlock_key_t key;


	for (i = 0; i < data->nb_clock_gate_info; i++, info++)
	{
		if (info->id == clk_id)
		{
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + info->offset);
	regval |= BIT(info->bit);
	sys_write32(regval, config->base + info->offset);
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int sun8i_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct sun8i_ccu_config *config = dev->config;
	struct sun8i_ccu_data *data = dev->data;
	const struct sun8i_clock_gate_info *info = data->clock_gate_info;
	uint32_t clk_id = (uint32_t)sys;
	bool found = false;
	k_spinlock_key_t key;
	uint32_t regval;
	int i;

	for (i = 0; i < data->nb_clock_gate_info; i++, info++)
	{
		if (info->id == clk_id)
		{
			found = true;
			break;
		}
	}

	if (!found)
		return -EINVAL;

	key = k_spin_lock(&data->lock);
	regval = sys_read32(config->base + info->offset);
	regval &= ~BIT(info->bit);
	sys_write32(regval, config->base + info->offset);
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

/*
 * SPI0_CLK register (CCU offset 0xA0):
 *   bit 31:      Gating (1 = clock enabled)
 *   bits 26:24:  Clock source (000=HOSC, 001=PLL_PERIPH0, 010=PLL_PERIPH1)
 *   bits 17:16:  Pre-divider M (00=/1, 01=/2, 10=/4, 11=/8)
 *   bits 1:0:    Divider N (00=/1, 01=/2, 10=/4, 11=/8)
 *   f_out = f_parent / (2^M * 2^N)
 *
 * The SPI driver passes a struct sun8i_spi_clk_config with the parent
 * clock frequency and the desired output frequency.
 */

#define SPI0_CLK_SRC_SHIFT	24
#define SPI0_CLK_SRC_MASK	GENMASK(26, 24)
#define SPI0_CLK_SRC_HOSC	0
#define SPI0_CLK_M_SHIFT	16
#define SPI0_CLK_M_MASK		GENMASK(17, 16)
#define SPI0_CLK_N_SHIFT	0
#define SPI0_CLK_N_MASK		GENMASK(1, 0)
#define SPI0_CLK_GATE		BIT(31)
#define SPI0_CLK_PLL_BYPASS	BIT(30)  /* Bypass PLL lock when using OSC24M */

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

DEVICE_API(clock_control, sun8i_clock_control_api) = {
	.on = sun8i_clock_on,
	.off = sun8i_clock_off,
	.set_rate = sun8i_set_clk_cpu,
};

static int sun8i_ccu_init(const struct device *dev)
{
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
