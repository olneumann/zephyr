/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_mmc5983ma

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mmc5983ma, CONFIG_SENSOR_LOG_LEVEL);

#include "mmc5983ma.h"

static int mmc5983ma_sample_fetch(const struct device *dev,
				   enum sensor_channel chan)
{
	const struct mmc5983ma_config *config = dev->config;
	struct mmc5983ma_data *drv_data = dev->data;
	uint8_t magn_buf[6];
	uint8_t status;

	/* Check data ready flag */
	if (i2c_reg_read_byte_dt(&config->i2c, mmc5983ma_SR_REG_M,
				 &status) < 0) {
		LOG_ERR("Failed to read status register.");
		return -EIO;
	}

	if (!(status & mmc5983ma_DRDY)) {
		LOG_ERR("Sensor data not available.");
		return -EIO;
	}

	if (i2c_burst_read_dt(&config->i2c, mmc5983ma_REG_MAGN_X_LSB,
			      magn_buf, 6) < 0) {
		LOG_ERR("Could not read magn axis data.");
		return -EIO;
	}

	drv_data->magn_x = (magn_buf[0] << 8) | magn_buf[1];
	drv_data->magn_y = (magn_buf[4] << 8) | magn_buf[5];
	drv_data->magn_z = (magn_buf[2] << 8) | magn_buf[3];

	return 0;
}

static void mmc5983ma_convert_xy(struct sensor_value *val,
			       int64_t raw_val)
{
	val->val1 = raw_val / mmc5983ma_LSB_GAUSS_XY;
	val->val2 = (1000000 * raw_val / mmc5983ma_LSB_GAUSS_XY) % 1000000;
}

static void mmc5983ma_convert_z(struct sensor_value *val,
			       int64_t raw_val)
{
	val->val1 = raw_val / mmc5983ma_LSB_GAUSS_Z;
	val->val2 = (1000000 * raw_val / mmc5983ma_LSB_GAUSS_Z) % 1000000;
}

static int mmc5983ma_channel_get(const struct device *dev,
				  enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct mmc5983ma_data *drv_data = dev->data;

	switch (chan) {
	case  SENSOR_CHAN_MAGN_X:
		mmc5983ma_convert_xy(val, drv_data->magn_x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		mmc5983ma_convert_xy(val, drv_data->magn_y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		mmc5983ma_convert_z(val, drv_data->magn_z);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		mmc5983ma_convert_xy(val, drv_data->magn_x);
		mmc5983ma_convert_xy(val + 1, drv_data->magn_y);
		mmc5983ma_convert_z(val + 2, drv_data->magn_z);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static const struct sensor_driver_api mmc5983ma_driver_api = {
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
};

static int mmc5983ma_init(const struct device *dev)
{
	const struct mmc5983ma_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* Set magnetometer output data rate */
	if (i2c_reg_write_byte_dt(&config->i2c, mmc5983ma_CRA_REG_M,
				  mmc5983ma_ODR_BITS) < 0) {
		LOG_ERR("Failed to configure chip.");
		return -EIO;
	}

	/* Set magnetometer full scale range */
	if (i2c_reg_write_byte_dt(&config->i2c, mmc5983ma_CRB_REG_M,
				  mmc5983ma_FS_BITS) < 0) {
		LOG_ERR("Failed to set magnetometer full scale range.");
		return -EIO;
	}

	/* Continuous update */
	if (i2c_reg_write_byte_dt(&config->i2c, mmc5983ma_MR_REG_M,
				  mmc5983ma_CONT_UPDATE) < 0) {
		LOG_ERR("Failed to enable continuous data update.");
		return -EIO;
	}
	return 0;
}

#define mmc5983ma_DEFINE(inst)								\
	static struct mmc5983ma_data mmc5983ma_data_##inst;				\
												\
	static const struct mmc5983ma_config mmc5983ma_config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mmc5983ma_init, NULL,				\
			      &mmc5983ma_data_##inst, &mmc5983ma_config_##inst,	\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,				\
			      &mmc5983ma_driver_api);					\

DT_INST_FOREACH_STATUS_OKAY(mmc5983ma_DEFINE)
