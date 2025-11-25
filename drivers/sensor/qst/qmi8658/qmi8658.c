#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT qst_qmi8658

LOG_MODULE_REGISTER(QMI8658, CONFIG_SENSOR_LOG_LEVEL);

#define QMI8658_REG_WHO_AM_I		0x00
#define QMI8658_REG_REVISION_ID		0x01
#define QMI8658_REG_CTRL1		0x02
#define QMI8658_REG_CTRL2		0x03
#define QMI8658_REG_CTRL3		0x04
#define QMI8658_REG_CTRL4		0x05
#define QMI8658_REG_CTRL5		0x06
#define QMI8658_REG_CTRL6		0x07
#define QMI8658_REG_CTRL7		0x08
#define QMI8658_REG_CTRL8		0x09
#define QMI8658_REG_CTRL9		0x0A

#define QMI8658_REG_CAL1_L		0x0B
#define QMI8658_REG_CAL1_H		0x0C

#define QMI8658_REG_FIFO_WTM_TH		0x13
#define QMI8658_REG_FIFO_CTRL		0x14
#define QMI8658_REG_FIFO_SMPL_CNT	0x15
#define QMI8658_REG_FIFO_STATUS		0x16
#define QMI8658_REG_FIFO_DATA		0x17

#define QMI8658_REG_STATUSINT		0x2D
#define QMI8658_REG_STATUS0		0x2E
#define QMI8658_REG_STATUS1		0x2F

#define QMI8658_REG_TIMESTAMP_L		0x30
#define QMI8658_REG_TIMESTAMP_M		0x31
#define QMI8658_REG_TIMESTAMP_H		0x32

#define QMI8658_REG_TEMP_L		0x33
#define QMI8658_REG_TEMP_H		0x34
#define QMI8658_REG_AX_L		0x35
#define QMI8658_REG_AX_H		0x36
#define QMI8658_REG_AY_L		0x37
#define QMI8658_REG_AY_H		0x38
#define QMI8658_REG_AZ_L		0x39
#define QMI8658_REG_AZ_H		0x3A
#define QMI8658_REG_GX_L		0x3B
#define QMI8658_REG_GX_H		0x3C
#define QMI8658_REG_GY_L		0x3D
#define QMI8658_REG_GY_H		0x3E
#define QMI8658_REG_GZ_L		0x3F
#define QMI8658_REG_GZ_H		0x40

#define QMI8658_REG_RESET		0x60

#define QMI8658_WHO_AM_I_VALUE		0x05
#define QMI8658_RESET_VALUE		0xB0

#define QMI8658_CTRL1_ADDR_AI		BIT(6)
#define QMI8658_CTRL1_INT1_EN		BIT(3)
#define QMI8658_CTRL1_INT1_LVL		BIT(4)

#define QMI8658_CTRL7_DISABLE_ALL	0x00
#define QMI8658_CTRL7_ACC_EN		BIT(0)
#define QMI8658_CTRL7_GYR_EN		BIT(1)

#define QMI8658_STATUS1_ANY_MOTION	BIT(0)
#define QMI8658_CTRL4_WOM_AXES_EN	0x07
#define QMI8658_CTRL5_INT1_WOM		BIT(0)

struct qmi8658_data {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t temp;
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t handler;
	const struct sensor_trigger *trigger;
};

struct qmi8658_config {
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpio;
};

static int qmi8658_i2c_read(const struct device *dev, uint8_t reg, uint8_t *data, uint32_t len)
{
	const struct qmi8658_config *config = dev->config;

	return i2c_write_read_dt(&config->i2c, &reg, 1, data, len);
}

static int qmi8658_i2c_write(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct qmi8658_config *config = dev->config;
	uint8_t buf[2] = {reg, data};

	return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

int qmi8658_reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	return qmi8658_i2c_read(dev, reg, val, 1);
}

static int qmi8658_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qmi8658_data *data = dev->data;
	uint8_t buf[14];

	if (qmi8658_i2c_read(dev, QMI8658_REG_TEMP_L, buf, sizeof(buf)) < 0) {
		LOG_ERR("Failed to read sample data");
		return -EIO;
	}

	data->temp = sys_get_le16(&buf[0]);
	data->ax = sys_get_le16(&buf[2]);
	data->ay = sys_get_le16(&buf[4]);
	data->az = sys_get_le16(&buf[6]);
	data->gx = sys_get_le16(&buf[8]);
	data->gy = sys_get_le16(&buf[10]);
	data->gz = sys_get_le16(&buf[12]);

	return 0;
}

static void qmi8658_convert(struct sensor_value *val, int16_t raw_val, float scale)
{
	double d_val = (double)raw_val * scale;

	val->val1 = (int32_t)d_val;
	val->val2 = (int32_t)((d_val - val->val1) * 1000000);
}

static int qmi8658_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct qmi8658_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		qmi8658_convert(val, data->ax, 1.0f / 16384.0f * SENSOR_G);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		qmi8658_convert(val, data->ay, 1.0f / 16384.0f * SENSOR_G);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		qmi8658_convert(val, data->az, 1.0f / 16384.0f * SENSOR_G);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		qmi8658_convert(&val[0], data->ax, 1.0f / 16384.0f * SENSOR_G);
		qmi8658_convert(&val[1], data->ay, 1.0f / 16384.0f * SENSOR_G);
		qmi8658_convert(&val[2], data->az, 1.0f / 16384.0f * SENSOR_G);
		break;
	case SENSOR_CHAN_GYRO_X:
		qmi8658_convert(val, data->gx, 1.0f / 16.0f * SENSOR_PI / 180.0f);
		break;
	case SENSOR_CHAN_GYRO_Y:
		qmi8658_convert(val, data->gy, 1.0f / 16.0f * SENSOR_PI / 180.0f);
		break;
	case SENSOR_CHAN_GYRO_Z:
		qmi8658_convert(val, data->gz, 1.0f / 16.0f * SENSOR_PI / 180.0f);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		qmi8658_convert(&val[0], data->gx, 1.0f / 16.0f * SENSOR_PI / 180.0f);
		qmi8658_convert(&val[1], data->gy, 1.0f / 16.0f * SENSOR_PI / 180.0f);
		qmi8658_convert(&val[2], data->gz, 1.0f / 16.0f * SENSOR_PI / 180.0f);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		qmi8658_convert(val, data->temp, 1.0f / 256.0f);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int qmi8658_trigger_set(const struct device *dev,
			      const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler);

static void qmi8658_handle_interrupt(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct qmi8658_data *data = CONTAINER_OF(cb, struct qmi8658_data, gpio_cb);
	const struct device *dev = data->dev;
	const struct qmi8658_config *config = dev->config;
	uint8_t status1, statusint;

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);

	/* Read STATUS1 to know the interrupt source, then read STATUSINT to clear it */
	if (qmi8658_i2c_read(dev, QMI8658_REG_STATUS1, &status1, 1) < 0) {
		LOG_ERR("Failed to read status1 register");
	} else {
		if ((status1 & QMI8658_STATUS1_ANY_MOTION) && data->handler) {
			data->handler(dev, data->trigger);
		}
	}

	/* Reading STATUSINT clears the interrupt line */
	if (qmi8658_i2c_read(dev, QMI8658_REG_STATUSINT, &statusint, 1) < 0) {
		LOG_ERR("Failed to read statusint to clear interrupt");
	}

	gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_LEVEL_ACTIVE);
}

static int qmi8658_trigger_set(const struct device *dev,
			      const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	struct qmi8658_data *data = dev->data;
	const struct qmi8658_config *config = dev->config;
	uint8_t status_int_val;
	int ret;

	if (!config->int_gpio.port) {
		LOG_ERR("int-gpios not specified in DTS");
		return -ENOTSUP;
	}

	/* First, disable the GPIO interrupt */
	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("Failed to disable GPIO interrupt: %d", ret);
		return ret;
	}

	/* Disable all sensor interrupts on the chip to ensure a clean state */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL1, 0x00) < 0) {
		LOG_ERR("Failed to disable INT1");
		return -EIO;
	}
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL4, 0x00) < 0) {
		LOG_ERR("Failed to disable WoM axes");
		return -EIO;
	}
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL5, 0x00) < 0) {
		LOG_ERR("Failed to disable INT1 routing");
		return -EIO;
	}

	data->handler = handler;
	data->trigger = trig;

	if (handler == NULL) {
		LOG_DBG("Trigger handler is NULL, disabling trigger");
		return 0;
	}

	if (trig->type != SENSOR_TRIG_MOTION) {
		LOG_ERR("Unsupported trigger type");
		return -ENOTSUP;
	}

	LOG_DBG("Configuring Wake on Motion trigger");

	/* 1. Disable sensors */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE_ALL) < 0) {
		LOG_ERR("Failed to disable sensors");
		return -EIO;
	}

	/* 2. Configure Accelerometer: ODR=62.5Hz (low power), Range=8g */
	/* 2. Configure Accelerometer: ODR=21Hz (low power), Range=8g */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL2, 0x0D | (0x02 << 4)) < 0) {
		LOG_ERR("Failed to configure accel for WOM");
		return -EIO;
	}

	/* 3. Configure WoM */
	/* Enable WoM for X, Y, Z axes */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL4, QMI8658_CTRL4_WOM_AXES_EN) < 0) {
		LOG_ERR("Failed to enable WOM axes");
		return -EIO;
	}
	/* Enable the "Any Motion" engine */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL8, 0x02) < 0) {
		LOG_ERR("Failed to enable Any Motion engine");
		return -EIO;
	}

	/* 4. Configure Interrupt Pin INT1 */
	/* Route WOM interrupt to INT1 */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL5, QMI8658_CTRL5_INT1_WOM) < 0) {
		LOG_ERR("Failed to route WOM interrupt to INT1");
		return -EIO;
	}
	/* Enable INT1 pin, non-latched, level-triggered */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL1, QMI8658_CTRL1_INT1_EN | QMI8658_CTRL1_INT1_LVL) < 0) {
		LOG_ERR("Failed to enable INT1 pin");
		return -EIO;
	}

	/* 5. Clear stale interrupts by reading status register */
	if (qmi8658_i2c_read(dev, QMI8658_REG_STATUSINT, &status_int_val, 1) < 0) {
		LOG_ERR("Failed to read STATUSINT to clear stale interrupts");
		return -EIO;
	}

	/* 6. Re-enable sensors to the state they were in before (from init) */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_EN | QMI8658_CTRL7_GYR_EN) < 0) {
		LOG_ERR("Failed to re-enable sensors");
		return -EIO;
	}

	/* 7. Enable GPIO interrupt on MCU */
	ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_LEVEL_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
		return ret;
	}

	LOG_DBG("WoM trigger set successfully");
	return 0;
}

static const struct sensor_driver_api qmi8658_driver_api = {
	.sample_fetch = qmi8658_sample_fetch,
	.channel_get = qmi8658_channel_get,
	.trigger_set = qmi8658_trigger_set,
};

static int qmi8658_init(const struct device *dev)
{
	struct qmi8658_data *data = dev->data;
	const struct qmi8658_config *config = dev->config;
	uint8_t who_am_i;

	data->dev = dev;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	if (qmi8658_i2c_read(dev, QMI8658_REG_WHO_AM_I, &who_am_i, 1) < 0) {
		LOG_ERR("Failed to read WHO_AM_I register");
		return -EIO;
	}

	if (who_am_i != QMI8658_WHO_AM_I_VALUE) {
		LOG_ERR("Invalid WHO_AM_I value");
		return -EINVAL;
	}

	if (qmi8658_i2c_write(dev, QMI8658_REG_RESET, QMI8658_RESET_VALUE) < 0) {
		LOG_ERR("Failed to reset device");
		return -EIO;
	}

	k_sleep(K_MSEC(5));

	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL1, QMI8658_CTRL1_ADDR_AI) < 0) {
		LOG_ERR("Failed to enable address auto increment");
		return -EIO;
	}

	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL7, QMI8658_CTRL7_ACC_EN | QMI8658_CTRL7_GYR_EN) < 0) {
		LOG_ERR("Failed to enable sensors");
		return -EIO;
	}

	LOG_DBG("Setting default sensor configuration");
	/* Accel ODR=500Hz, Range=8g */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL2, 0x06 | (0x02 << 4)) < 0) {
		LOG_ERR("Failed to configure accel");
		return -EIO;
	}
	/* Gyro ODR=500Hz, Range=2048dps */
	if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL3, 0x06 | (0x03 << 4)) < 0) {
		LOG_ERR("Failed to configure gyro");
		return -EIO;
	}

	if (config->int_gpio.port) {
		uint8_t ctrl6 = 0;

		if (config->int_gpio.dt_flags & GPIO_ACTIVE_HIGH) {
			/* INT1 active high */
			ctrl6 |= BIT(0);
		}
		/* INT2 is not used, default to active low */

		if (qmi8658_i2c_write(dev, QMI8658_REG_CTRL6, ctrl6) < 0) {
			LOG_ERR("Failed to configure interrupt polarity");
			return -EIO;
		}

		if (!gpio_is_ready_dt(&config->int_gpio)) {
			LOG_ERR("GPIO port %s not ready", config->int_gpio.port->name);
			return -ENODEV;
		}
		gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
		gpio_init_callback(&data->gpio_cb, qmi8658_handle_interrupt,
				   BIT(config->int_gpio.pin));
		if (gpio_add_callback(config->int_gpio.port, &data->gpio_cb) < 0) {
			LOG_ERR("Could not set gpio callback");
			return -EIO;
		}
	}

	LOG_DBG("Init complete");

	return 0;
}

#define QMI8658_DEFINE(inst)									\
	static struct qmi8658_data qmi8658_data_##inst;						\
	static const struct qmi8658_config qmi8658_config_##inst = {			\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {}),			\
	};											\
	DEVICE_DT_INST_DEFINE(inst, qmi8658_init, NULL, &qmi8658_data_##inst,		\
			      &qmi8658_config_##inst, POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY, &qmi8658_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QMI8658_DEFINE)
