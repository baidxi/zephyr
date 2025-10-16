#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(KTH7823, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT conntek_kth7823

/* KTH7823 registers */
#define KTH7823_REG_Z_LSB 0x00
#define KTH7823_REG_Z_MSB 0x01
#define KTH7823_REG_GAIN_TRIM 0x02
#define KTH7823_REG_TRIM 0x03
#define KTH7823_REG_PPT_Z 0x04
#define KTH7823_REG_PPT_MSB 0x05
#define KTH7823_REG_MGH_MGL 0x06
#define KTH7823_REG_NPP 0x07
#define KTH7823_REG_ABZ_LIMIT 0x08
#define KTH7823_REG_RD 0x09

/* KTH7823 commands */
#define KTH7823_CMD_READ_ANGLE 0x0000
#define KTH7823_CMD_READ_REG_PREFIX 0x4000
#define KTH7823_CMD_WRITE_REG_PREFIX 0x8000

struct kth7823_data {
	uint16_t angle;
};

struct kth7823_config {
#ifdef CONFIG_KTH7823_INTERFACE_SPI
	struct spi_dt_spec spi;
#endif
#ifdef CONFIG_KTH7823_INTERFACE_SSI
	struct gpio_dt_spec ssck;
	struct gpio_dt_spec ssd;
#endif
#ifdef CONFIG_KTH7823_INTERFACE_PWM
	struct pwm_dt_spec pwm;
#endif
#ifdef CONFIG_KTH7823_ABZ
	struct gpio_dt_spec a;
	struct gpio_dt_spec b;
	struct gpio_dt_spec z;
	uint16_t abz_resolution;
#endif
	bool rotation_inverse;
	uint16_t zero_offset;
	uint8_t mag_high_threshold;
	uint8_t mag_low_threshold;
};

static int kth7823_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct kth7823_config *config = dev->config;
	int ret;

	uint16_t cmd = sys_cpu_to_be16(KTH7823_CMD_READ_REG_PREFIX | (reg << 8));
	uint16_t data;

	const struct spi_buf tx_buf = {
		.buf = &cmd,
		.len = sizeof(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf = {
		.buf = &data,
		.len = sizeof(data),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
		return ret;
	}

	*val = (uint8_t)sys_be16_to_cpu(data);

	return 0;
}

static int kth7823_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct kth7823_config *config = dev->config;
	int ret;

	uint16_t cmd = sys_cpu_to_be16(KTH7823_CMD_WRITE_REG_PREFIX | (reg << 8) | val);

	const struct spi_buf tx_buf = {
		.buf = &cmd,
		.len = sizeof(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	ret = spi_write_dt(&config->spi, &tx);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
		return ret;
	}

	/* Per datasheet, wait 20ms for write to complete */
	k_sleep(K_MSEC(20));

	return 0;
}

#ifdef CONFIG_KTH7823_INTERFACE_SSI
static int kth7823_ssi_read_angle(const struct device *dev, uint16_t *angle)
{
	const struct kth7823_config *config = dev->config;
	uint16_t val = 0;

	for (int i = 0; i < 16; i++) {
		gpio_pin_set_dt(&config->ssck, 1);
		k_busy_wait(1);
		val |= (gpio_pin_get_dt(&config->ssd) << (15 - i));
		gpio_pin_set_dt(&config->ssck, 0);
		k_busy_wait(1);
	}

	*angle = val;

	return 0;
}
#endif

#ifdef CONFIG_KTH7823_INTERFACE_PWM
static int kth7823_pwm_read_angle(const struct device *dev, uint16_t *angle)
{
	const struct kth7823_config *config = dev->config;
	uint32_t period, pulse;
	int ret;

	ret = pwm_get_cycles(config->pwm.dev, config->pwm.channel, &period, &pulse);
	if (ret < 0) {
		LOG_ERR("Failed to get PWM cycles: %d", ret);
		return ret;
	}

	/*
	 * Convert PWM duty cycle to 16-bit angle value.
	 * See datasheet for formula.
	 */
	if (period == 0) {
		return -EINVAL;
	}

	uint64_t raw = ((uint64_t)pulse * (16384 + 64) / period) - 32;
	*angle = (uint16_t)((raw * 65535) / 16384);

	return 0;
}
#endif

static int kth7823_spi_read_angle(const struct device *dev, uint16_t *angle)
{
	const struct kth7823_config *config = dev->config;
	int ret;

	uint16_t cmd = sys_cpu_to_be16(KTH7823_CMD_READ_ANGLE);
	uint16_t data;

	const struct spi_buf tx_buf = {
		.buf = &cmd,
		.len = sizeof(cmd),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf = {
		.buf = &data,
		.len = sizeof(data),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %d", ret);
		return ret;
	}

	*angle = sys_be16_to_cpu(data);

	return 0;
}

static int kth7823_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct kth7823_data *data = dev->data;
	int ret = -ENOTSUP;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

#if defined(CONFIG_KTH7823_INTERFACE_SSI)
	ret = kth7823_ssi_read_angle(dev, &data->angle);
#elif defined(CONFIG_KTH7823_INTERFACE_PWM)
	ret = kth7823_pwm_read_angle(dev, &data->angle);
#elif defined(CONFIG_KTH7823_INTERFACE_ABZ)
	LOG_ERR("sample_fetch is not supported for ABZ interface. "
		"Use a QDEC driver instead.");
	ret = -ENOTSUP;
#else
	ret = kth7823_spi_read_angle(dev, &data->angle);
#endif

	return ret;
}

static int kth7823_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct kth7823_data *data = dev->data;

	if (chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}

	/*
	 * Convert 16-bit raw value (0-65535) to degrees (0-360).
	 * val = (raw / 65535.0) * 360.0
	 */
	int64_t micro_degrees = (int64_t)data->angle * 360LL * 1000000LL / 65535LL;

	val->val1 = micro_degrees / 1000000LL;
	val->val2 = micro_degrees % 1000000LL;

	return 0;
}

static const struct sensor_driver_api kth7823_driver_api = {
	.sample_fetch = kth7823_sample_fetch,
	.channel_get = kth7823_channel_get,
};

static int kth7823_init(const struct device *dev)
{
	const struct kth7823_config *config = dev->config;
	int ret;

#ifdef CONFIG_KTH7823_INTERFACE_SPI
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}
#endif

#ifdef CONFIG_KTH7823_INTERFACE_SSI
	if (!gpio_is_ready_dt(&config->ssck) || !gpio_is_ready_dt(&config->ssd)) {
		LOG_ERR("SSI GPIOs not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&config->ssck, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&config->ssd, GPIO_INPUT);
#endif

#ifdef CONFIG_KTH7823_INTERFACE_PWM
	if (!pwm_is_ready_dt(&config->pwm)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}
#endif

#ifdef CONFIG_KTH7823_ABZ
	if (!gpio_is_ready_dt(&config->a) || !gpio_is_ready_dt(&config->b) ||
	    !gpio_is_ready_dt(&config->z)) {
		LOG_ERR("ABZ GPIOs not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&config->a, GPIO_INPUT);
	gpio_pin_configure_dt(&config->b, GPIO_INPUT);
	gpio_pin_configure_dt(&config->z, GPIO_INPUT);

	if (config->abz_resolution > 0) {
		if (config->abz_resolution < 1 || config->abz_resolution > 1024) {
			LOG_ERR("Invalid ABZ resolution: %d", config->abz_resolution);
			return -EINVAL;
		}
		uint16_t ppt = config->abz_resolution - 1;
		uint8_t ppt_lsb = ppt & 0x03;
		uint8_t ppt_msb = (ppt >> 2) & 0xFF;

		ret = kth7823_reg_write(dev, KTH7823_REG_PPT_MSB, ppt_msb);
		if (ret < 0) {
			return ret;
		}

		uint8_t ppt_z_val;

		ret = kth7823_reg_read(dev, KTH7823_REG_PPT_Z, &ppt_z_val);
		if (ret < 0) {
			return ret;
		}

		ppt_z_val &= ~0xC0; /* Clear PPT bits */
		ppt_z_val |= (ppt_lsb << 6);

		ret = kth7823_reg_write(dev, KTH7823_REG_PPT_Z, ppt_z_val);
		if (ret < 0) {
			return ret;
		}
	}
#endif

	if (config->rotation_inverse) {
		uint8_t rd_val;

		ret = kth7823_reg_read(dev, KTH7823_REG_RD, &rd_val);
		if (ret < 0) {
			return ret;
		}
		rd_val |= 0x01; /* Set RD bit */
		ret = kth7823_reg_write(dev, KTH7823_REG_RD, rd_val);
		if (ret < 0) {
			return ret;
		}
	}

	if (config->zero_offset > 0) {
		uint8_t z_lsb = config->zero_offset & 0xFF;
		uint8_t z_msb = (config->zero_offset >> 8) & 0xFF;

		ret = kth7823_reg_write(dev, KTH7823_REG_Z_LSB, z_lsb);
		if (ret < 0) {
			return ret;
		}
		ret = kth7823_reg_write(dev, KTH7823_REG_Z_MSB, z_msb);
		if (ret < 0) {
			return ret;
		}
	}

	if (config->mag_high_threshold > 0 || config->mag_low_threshold > 0) {
		if (config->mag_high_threshold > 7 || config->mag_low_threshold > 7) {
			LOG_ERR("Invalid mag threshold");
			return -EINVAL;
		}
		uint8_t mgh_mgl_val =
			(config->mag_high_threshold << 4) | config->mag_low_threshold;
		ret = kth7823_reg_write(dev, KTH7823_REG_MGH_MGL, mgh_mgl_val);
		if (ret < 0) {
			return ret;
		}
	}

	LOG_DBG("KTH7823 initialized");
	return 0;
}

#define KTH7823_INIT(inst)                                                                         \
	static struct kth7823_data kth7823_data_##inst;                                             \
	                                                                                                   \
	static const struct kth7823_config kth7823_config_##inst = {                                  \
		IF_ENABLED(CONFIG_KTH7823_INTERFACE_SPI,(										\
			.spi = SPI_DT_SPEC_INST_GET(inst,                                                    \
					SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |     \
						SPI_WORD_SET(16) | SPI_TRANSFER_MSB,             \
					0),))																\
		IF_ENABLED(CONFIG_KTH7823_INTERFACE_SSI,                                              \
			   (.ssck = GPIO_DT_SPEC_INST_GET(inst, ssck_gpios),                          \
			    .ssd = GPIO_DT_SPEC_INST_GET(inst, ssd_gpios),))                          \
		IF_ENABLED(CONFIG_KTH7823_INTERFACE_PWM,                                              \
			   (.pwm = PWM_DT_SPEC_INST_GET(inst),))                                      \
		IF_ENABLED(CONFIG_KTH7823_ABZ,                                                        \
			   (.a = GPIO_DT_SPEC_INST_GET(inst, a_gpios),                                \
			    .b = GPIO_DT_SPEC_INST_GET(inst, b_gpios),                                \
			    .z = GPIO_DT_SPEC_INST_GET(inst, z_gpios),                                \
			    .abz_resolution = DT_INST_PROP_OR(inst, abz_resolution, 0),))             \
		.rotation_inverse = DT_INST_PROP(inst, rotation_inverse),                             \
		.zero_offset = DT_INST_PROP_OR(inst, zero_offset, 0),                                 \
		.mag_high_threshold = DT_INST_PROP_OR(inst, mag_high_threshold, 0),                   \
		.mag_low_threshold = DT_INST_PROP_OR(inst, mag_low_threshold, 0),                     \
	};                                                                                         \
	                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &kth7823_init, NULL, &kth7823_data_##inst,                      \
			      &kth7823_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
			      &kth7823_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KTH7823_INIT)
