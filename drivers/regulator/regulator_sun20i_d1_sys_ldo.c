/*
 * Copyright (c) 2026 Juno (@bingpi).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT allwinner_sun20i_d1_system_ldos

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/sys/linear_range.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(regulator_sun20i_d1_sys_ldo, CONFIG_REGULATOR_LOG_LEVEL);

/*
 * SYS_LDO_CTRL register offset within the SYS_CFG module (base 0x03000000).
 *
 *   bits [4:0]   LDOA output voltage select (0–31)
 *   bits [13:8]  LDOB output voltage select (0–63)
 *
 * The voltage step is 40000/3 µV ≈ 13333.33 µV (non-integer); it is
 * approximated as 13333 µV for the linear-range lookup tables.  The
 * accumulated error over the full range is < 11 µV.
 */
#define SUN20I_SYS_LDO_CTRL_REG	0x150U

enum sun20i_sys_ldo_id {
	SUN20I_LDOA = 0,
	SUN20I_LDOB = 1,
};

struct sun20i_sys_ldo_desc {
	enum sun20i_sys_ldo_id id;
	/** Mask of the voltage-select field within SYS_LDO_CTRL. */
	uint32_t vsel_mask;
	/** Bit shift of the voltage-select field. */
	uint8_t vsel_shift;
	/** Linear range(s) describing selector → voltage mapping. */
	const struct linear_range *ranges;
	/** Number of entries in @p ranges. */
	uint8_t num_ranges;
};

struct sun20i_sys_ldo_config {
	struct regulator_common_config common;
	/** Syscon device for the parent system-control node. */
	const struct device *syscon;
	/** Static per-LDO descriptor. */
	const struct sun20i_sys_ldo_desc *desc;
};

struct sun20i_sys_ldo_data {
	struct regulator_common_data common;
};

/* LDOA: 1.593V → 2.007V, 32 steps. */
static const struct linear_range ldoa_ranges[] = {
	LINEAR_RANGE_INIT(1593333U, 13333U, 0U, 31U),
};

/* LDOB: 1.167V → 2.007V, 64 steps. */
static const struct linear_range ldob_ranges[] = {
	LINEAR_RANGE_INIT(1166666U, 13333U, 0U, 63U),
};

static const struct sun20i_sys_ldo_desc ldoa_desc = {
	.id = SUN20I_LDOA,
	.vsel_mask = 0x1FU,
	.vsel_shift = 0U,
	.ranges = ldoa_ranges,
	.num_ranges = ARRAY_SIZE(ldoa_ranges),
};

static const struct sun20i_sys_ldo_desc ldob_desc = {
	.id = SUN20I_LDOB,
	.vsel_mask = 0x3FU,
	.vsel_shift = 8U,
	.ranges = ldob_ranges,
	.num_ranges = ARRAY_SIZE(ldob_ranges),
};

/*
 * The internal system LDOs are always powered in hardware; there is no
 * enable/disable control bit.  Both callbacks are no-ops.
 */
static int sun20i_sys_ldo_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int sun20i_sys_ldo_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int sun20i_sys_ldo_set_voltage(const struct device *dev, int32_t min_uv,
				      int32_t max_uv)
{
	const struct sun20i_sys_ldo_config *config = dev->config;
	const struct sun20i_sys_ldo_desc *desc = config->desc;
	uint32_t mask;
	uint16_t idx;
	int ret;

	ret = linear_range_group_get_win_index(desc->ranges, desc->num_ranges,
					       min_uv, max_uv, &idx);
	if (ret != 0) {
		return ret;
	}

	mask = desc->vsel_mask << desc->vsel_shift;

	return syscon_update_bits(config->syscon, SUN20I_SYS_LDO_CTRL_REG, mask,
				  (uint32_t)idx << desc->vsel_shift);
}

static int sun20i_sys_ldo_get_voltage(const struct device *dev, int32_t *volt_uv)
{
	const struct sun20i_sys_ldo_config *config = dev->config;
	const struct sun20i_sys_ldo_desc *desc = config->desc;
	uint32_t val;
	uint16_t idx;
	int ret;

	ret = syscon_read_reg(config->syscon, SUN20I_SYS_LDO_CTRL_REG, &val);
	if (ret < 0) {
		return ret;
	}

	idx = (uint16_t)((val >> desc->vsel_shift) & desc->vsel_mask);

	return linear_range_group_get_value(desc->ranges, desc->num_ranges, idx,
					    volt_uv);
}

static int sun20i_sys_ldo_list_voltage(const struct device *dev, unsigned int idx,
				       int32_t *volt_uv)
{
	const struct sun20i_sys_ldo_config *config = dev->config;
	const struct sun20i_sys_ldo_desc *desc = config->desc;

	return linear_range_group_get_value(desc->ranges, desc->num_ranges,
					    (uint16_t)idx, volt_uv);
}

static unsigned int sun20i_sys_ldo_count_voltages(const struct device *dev)
{
	const struct sun20i_sys_ldo_config *config = dev->config;
	const struct sun20i_sys_ldo_desc *desc = config->desc;

	return linear_range_group_values_count(desc->ranges, desc->num_ranges);
}

static DEVICE_API(regulator, sun20i_sys_ldo_api) = {
	.enable = sun20i_sys_ldo_enable,
	.disable = sun20i_sys_ldo_disable,
	.set_voltage = sun20i_sys_ldo_set_voltage,
	.get_voltage = sun20i_sys_ldo_get_voltage,
	.list_voltage = sun20i_sys_ldo_list_voltage,
	.count_voltages = sun20i_sys_ldo_count_voltages,
};

static int sun20i_sys_ldo_init(const struct device *dev)
{
	regulator_common_data_init(dev);

	/* System LDOs are always enabled in hardware. */
	return regulator_common_init(dev, true);
}

#define SUN20I_LDO_DATA_NAME(node_id) _CONCAT(data_, DT_DEP_ORD(node_id))
#define SUN20I_LDO_CONF_NAME(node_id) _CONCAT(config_, DT_DEP_ORD(node_id))

/*
 * Define one regulator device for child node @p node_id.
 *
 * The syscon device is the grand-parent of the child node:
 *   syscon → system-ldos (DT_DRV_COMPAT) → ldoa/ldob (child-binding)
 */
#define SUN20I_LDO_DEFINE(node_id, desc_ptr)                                   \
	static struct sun20i_sys_ldo_data SUN20I_LDO_DATA_NAME(node_id);       \
                                                                               \
	static const struct sun20i_sys_ldo_config SUN20I_LDO_CONF_NAME(node_id) = {\
		.common = REGULATOR_DT_COMMON_CONFIG_INIT(node_id),            \
		.syscon = DEVICE_DT_GET(DT_PARENT(DT_PARENT(node_id))),        \
		.desc = (desc_ptr),                                            \
	};                                                                     \
                                                                               \
	DEVICE_DT_DEFINE(node_id, sun20i_sys_ldo_init, NULL,                   \
			 &SUN20I_LDO_DATA_NAME(node_id),                       \
			 &SUN20I_LDO_CONF_NAME(node_id), POST_KERNEL,          \
			 CONFIG_REGULATOR_SUN20I_D1_SYS_LDO_INIT_PRIORITY,     \
			 &sun20i_sys_ldo_api);

#define SUN20I_LDO_DEFINE_COND(inst, child_name, desc_ptr)                     \
	COND_CODE_1(DT_NODE_HAS_STATUS(DT_INST_CHILD(inst, child_name), okay),  \
		    (SUN20I_LDO_DEFINE(DT_INST_CHILD(inst, child_name), desc_ptr)),\
		    ())

#define SUN20I_LDO_DEFINE_ALL(inst)                                            \
	SUN20I_LDO_DEFINE_COND(inst, ldoa, &ldoa_desc)                         \
	SUN20I_LDO_DEFINE_COND(inst, ldob, &ldob_desc)

DT_INST_FOREACH_STATUS_OKAY(SUN20I_LDO_DEFINE_ALL)
