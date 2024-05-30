/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90395

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MLX90395, CONFIG_SENSOR_LOG_LEVEL);

#include "mlx90395.h"

static int mlx90395_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct mlx90395_config *config = dev->config;
	struct mlx90395_data *data = dev->data;

	// uint8_t buffer[7] = {0};

	// k_sem_take(&data->sem, K_FOREVER);

	// if (i2c_burst_read_dt(&config->i2c, mlx90395_XOUT_0, buffer, sizeof(buffer)) < 0) {
	// 	LOG_ERR("Failed to read data sample");
	// 	return -EIO;
	// }

	// k_sem_give(&data->sem);

	// data->magn_x = (uint32_t)(buffer[0] << 10 | buffer[1] << 2 | (buffer[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value
	// data->magn_y = (uint32_t)(buffer[2] << 10 | buffer[3] << 2 | (buffer[6] & 0x30) >> 4); // Turn the 18 bits into a unsigned 32-bit value
	// data->magn_z = (uint32_t)(buffer[4] << 10 | buffer[5] << 2 | (buffer[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value

	return 0;
}

static int mlx90395_convert(struct sensor_value *val, uint32_t raw_val)
{
	double value = sys_le16_to_cpu(raw_val);
	value = (value - PARAM_NULLFIELD_18BIT) / PARAM_MAGN_LSB_GAUSS;
	
	return sensor_value_from_double(val, value);
}

static int mlx90395_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	int ret = 0;
	struct mlx90395_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);

	// switch (chan) {
	// 	case SENSOR_CHAN_MAGN_X:
	// 		mlx90395_convert(val, data->magn_x);
	// 		break;
	// 	case SENSOR_CHAN_MAGN_Y:
	// 		mlx90395_convert(val, data->magn_y);
	// 		break;
	// 	case SENSOR_CHAN_MAGN_Z:
	// 		mlx90395_convert(val, data->magn_z);
	// 		break;
	// 	case SENSOR_CHAN_MAGN_XYZ:
	// 		mlx90395_convert(&val[0], data->magn_x);
	// 		mlx90395_convert(&val[1], data->magn_y);
	// 		mlx90395_convert(&val[2], data->magn_z);
	// 		break;
	// 	default:
	// 		LOG_ERR("Unsupported channel");
	// 		ret = -ENOTSUP;	
	// }

	k_sem_give(&data->sem);

	return ret;
}

static const struct sensor_driver_api mlx90395_driver_api = {
	.sample_fetch = mlx90395_sample_fetch,
	.channel_get = mlx90395_channel_get,
};

// static int mlx90395_set_internal_control_0(const struct device *dev, bool auto_sr_en, bool drdy_interrupt)
// {
// 	const struct mlx90395_config *config = dev->config;
// 	uint8_t reg = 0;

// 	reg |= (auto_sr_en << BIT_AUTO_SR_EN) | (drdy_interrupt << BIT_DRDY_INTERRUPT);

// 	return i2c_reg_write_byte_dt(&config->i2c, mlx90395_CONTROL_0, reg);
// }

static int mlx90395_init(const struct device *dev)
{
	const struct mlx90395_config *config = dev->config;
	struct mlx90395_data *data = dev->data;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	// /* Set internal control 0 */
	// if (mlx90395_set_internal_control_0(dev, true, false) < 0) {
	// 	LOG_ERR("Failed to set internal control 0");
	// 	return -EIO;
	// }

	/* Initialize semaphore */
	k_sem_init(&data->sem, 1, 1);

	LOG_INF("MLX90395 Initialized");

	return 0;
}

#define MLX90395_DEFINE(inst)								\
	static struct mlx90395_data mlx90395_data_##inst;				\
												\
	static const struct mlx90395_config mlx90395_config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mlx90395_init, NULL,				\
			      &mlx90395_data_##inst, &mlx90395_config_##inst,	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,				\
			      &mlx90395_driver_api);					\

DT_INST_FOREACH_STATUS_OKAY(MLX90395_DEFINE)
