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
	uint8_t cmd[7] = { 0 };
	int ret = 0;

	cmd[0] = MLX90395_CMD_READ_MEASUREMENT | X_FLAG | Y_FLAG | Z_FLAG; // TODO: Individual channels

	k_sem_take(&data->sem, K_FOREVER);

	if (i2c_write_read_dt(&config->i2c, &cmd[0], 1, &cmd[0], 7) < 0) {
		LOG_ERR("Failed to read data sample");
		ret = -EIO;
	}

	k_sem_give(&data->sem);

	data->magn_x = (uint16_t)(cmd[1] << 8 | cmd[2]);
	data->magn_y = (uint16_t)(cmd[3] << 8 | cmd[4]);
	data->magn_z = (uint16_t)(cmd[5] << 8 | cmd[6]);

	return ret;
}

// Convert functions
// XY
static int mlx90395_convert_xy(struct sensor_value *val, uint32_t raw_val)
{
	int32_t value = raw_val / 140 * 1000;
	val->val1 = value / 1000;
	val->val2 = value % 1000;

	return 0;
}
//Z
static int mlx90395_convert_z(struct sensor_value *val, uint32_t raw_val)
{
	int32_t value = raw_val / 140 * 1000;
	val->val1 = value / 1000;
	val->val2 = value % 1000;

	return 0;
}

static int mlx90395_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct mlx90395_data *data = dev->data;
	int ret = 0;

	k_sem_take(&data->sem, K_FOREVER);

	switch (chan) {
		case SENSOR_CHAN_MAGN_X:
			break;
		case SENSOR_CHAN_MAGN_Y:
			break;
		case SENSOR_CHAN_MAGN_Z:
			break;
		case SENSOR_CHAN_MAGN_XYZ:
			mlx90395_convert_xy(&val[0], data->magn_x);
			mlx90395_convert_xy(&val[1], data->magn_y);
			mlx90395_convert_z(&val[2], data->magn_z);
			break;
		default:
			LOG_ERR("Unsupported channel");
			ret = -ENOTSUP;	
	}

	k_sem_give(&data->sem);

	return ret;
}

static const struct sensor_driver_api mlx90395_driver_api = {
	.sample_fetch = mlx90395_sample_fetch,
	.channel_get = mlx90395_channel_get,
};

static int mlx90395_exit(const struct device *dev)
{
	const struct mlx90395_config *config = dev->config;
	uint8_t cmd = MLX90395_CMD_EXIT;

	return i2c_write_read_dt(&config->i2c, &cmd, 1, &cmd, 1);
}

static int mlx90395_reset(const struct device *dev)
{
	const struct mlx90395_config *config = dev->config;
	uint8_t cmd = MLX90395_CMD_RESET;

	return i2c_write_read_dt(&config->i2c, &cmd, 1, &cmd, 1);
}

static int mlx90395_write_register(const struct device *dev, uint8_t reg, uint16_t data)
{
	const struct mlx90395_config *config = dev->config;
	uint8_t cmd[3] = { 0 };

	cmd[0] = (reg & 0x7f) << 1;
	cmd[1] = (data & 0xff00) >> 8;
	cmd[2] = 	data & 0x00ff;

	return i2c_write_read_dt(&config->i2c, cmd, 3, cmd, 1);
}

static int mlx90395_start_burst(const struct device *dev, uint8_t zyxt_flags)
{
	const struct mlx90395_config *config = dev->config;
	uint8_t cmd = MLX90395_CMD_START_BURST | zyxt_flags;

	return i2c_write_read_dt(&config->i2c, &cmd, 1, &cmd, 1);
}

static int mlx90395_set_mode(const struct device *dev, uint8_t zyxt_flags, enum mlx90395_mode mode, uint8_t bdr)
{
	switch (mode) {
		case MLX90395_MODE_IDLE:
			mlx90395_exit(dev);
			break;
		case MLX90395_MODE_BURST:
			mlx90395_write_register(dev, BURST_SEL_REG, bdr);
			mlx90395_start_burst(dev, zyxt_flags);
			break;
		case MLX90395_MODE_SINGLE_MEASUREMENT:
			// TODO: Implement
			break;
		case MLX90395_MODE_WAKE_ON_CHANGE:
			// TODO: Implement
			break;
		default:
			LOG_ERR("Invalid mode");
			return -EINVAL;
	}
}

static int mlx90395_init(const struct device *dev)
{
	const struct mlx90395_config *config = dev->config;
	struct mlx90395_data *data = dev->data;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* Exit the sensor */
	if (mlx90395_exit(dev) < 0) {
		LOG_ERR("Failed to exit sensor");
		return -EIO;
	}

	k_msleep(50);

	/* Reset the sensor */
	if (mlx90395_reset(dev) < 0) {
		LOG_ERR("Failed to reset sensor");
		return -EIO;
	}

	k_msleep(50);

	/* Set mode */
	uint8_t zyxt_flags_tmp = X_FLAG | Y_FLAG | Z_FLAG;
	uint8_t bdr_tmp = 1;

	if (mlx90395_set_mode(dev, zyxt_flags_tmp, MLX90395_MODE_BURST, bdr_tmp) < 0) {
		LOG_ERR("Failed to set mode");
		return -EIO;
	}

	k_msleep(20);

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
