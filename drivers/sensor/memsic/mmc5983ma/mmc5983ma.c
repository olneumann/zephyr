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

LOG_MODULE_REGISTER(mmc5983ma, config_SENSOR_LOG_LEVEL);

#include "mmc5983ma.h"

static int mmc5983ma_sample_fetch(const struct device *dev,
				   enum sensor_channel chan)
{
	const struct mmc5983ma_config *config = dev->config;
	struct mmc5983ma_data *drv_data = dev->data;



	return 0;
}

static int mmc5983ma_channel_get(const struct device *dev,
				  enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct mmc5983ma_data *drv_data = dev->data;



	return 0;
}

static const struct sensor_driver_api mmc5983ma_driver_api = {
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
};






static int mmc5983ma_set_bandwidth(const struct device *dev, uint8_t bw)
{
	const struct mmc5983ma_config *config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, MMC5983MA_CONTROL_1, MASK_BANDWIDTH, bw);
}

static int mmc5983ma_set_frequency(const struct device *dev, uint8_t rate)
{
	const struct mmc5983ma_config *config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, MMC5983MA_CONTROL_2, MASK_FREQUENCY, rate);
}

static int mmc5983ma_set_prd_set(const struct device *dev, uint8_t prd)
{
	const struct mmc5983ma_config *config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, MMC5983MA_CONTROL_2, MASK_PRD_SET, prd);
}


static int mmc5983ma_init(const struct device *dev)
{
	const struct mmc5983ma_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}


	return 0;
}

#define mmc5983ma_DEFINE(inst)								\
	static struct mmc5983ma_data mmc5983ma_data_##inst;				\
												\
	static const struct mmc5983ma_config mmc5983ma_config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		.frequency = DT_INST_PROP(inst, frequency),				\
		.bandwidth = DT_INST_PROP(inst, bandwidth),				\
		.prd_set = DT_INST_PROP(inst, prd_set),					\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mmc5983ma_init, NULL,				\
			      &mmc5983ma_data_##inst, &mmc5983ma_config_##inst,	\
			      POST_KERNEL, config_SENSOR_INIT_PRIORITY,				\
			      &mmc5983ma_driver_api);					\

DT_INST_FOREACH_STATUS_OKAY(mmc5983ma_DEFINE)
