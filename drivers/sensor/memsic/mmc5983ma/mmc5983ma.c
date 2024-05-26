/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT memsic_mmc5983ma

#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MMC5983MA, CONFIG_SENSOR_LOG_LEVEL);

#include "mmc5983ma.h"

static int mmc5983ma_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct mmc5983ma_config *config = dev->config;
	struct mmc5983ma_data *drv_data = dev->data;

	uint8_t buffer[7] = {0};

	if (i2c_burst_read_dt(&config->i2c, MMC5983MA_XOUT_0, buffer, sizeof(buffer)) < 0) {
		LOG_ERR("Failed to read data sample");
		return -EIO;
	}

	drv_data->magn_x = buffer[0];
	drv_data->magn_x = (drv_data->magn_x << 8) | buffer[1];
	drv_data->magn_x = (drv_data->magn_x << 2) | (buffer[6] >> 6);

	drv_data->magn_y = buffer[2];
	drv_data->magn_y = (drv_data->magn_y << 8) | buffer[3];
	drv_data->magn_y = (drv_data->magn_y << 2) | ((buffer[6] >> 4) & 0x03);

	drv_data->magn_z = buffer[4];
	drv_data->magn_z = (drv_data->magn_z << 8) | buffer[5];
	drv_data->magn_z = (drv_data->magn_z << 2) | ((buffer[6] >> 2) & 0x03);

	return 0;
}

static void mmc5983ma_convert(struct sensor_value *val, uint32_t raw_val)
{
	val->val1 = ((int32_t)raw_val-PARAM_NULLFIELD_18BIT) / PARAM_MAGN_LSB_GAUSS;
	val->val2 = (1000000 * ((int32_t)raw_val-PARAM_NULLFIELD_18BIT) / PARAM_MAGN_LSB_GAUSS) % 1000000;
}

static int mmc5983ma_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	int ret = 0;
	struct mmc5983ma_data *drv_data = dev->data;

	//k_sem_take(&drv_data->sem, K_FOREVER);
	switch (chan) {
		case SENSOR_CHAN_MAGN_X:
			mmc5983ma_convert(val, drv_data->magn_x);
			break;
		case SENSOR_CHAN_MAGN_Y:
			mmc5983ma_convert(val, drv_data->magn_y);
			break;
		case SENSOR_CHAN_MAGN_Z:
			mmc5983ma_convert(val, drv_data->magn_z);
			break;
		case SENSOR_CHAN_MAGN_XYZ:
			mmc5983ma_convert(&val[0], drv_data->magn_x);
			mmc5983ma_convert(&val[1], drv_data->magn_y);
			mmc5983ma_convert(&val[2], drv_data->magn_z);
			break;
		default:
			LOG_ERR("Unsupported channel");
			ret = -ENOTSUP;	
	}

	//k_sem_give(&drv_data->sem);
	return ret;
}

static const struct sensor_driver_api mmc5983ma_driver_api = {
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
};

static int mmc5983ma_set_internal_control_0(const struct device *dev, bool auto_sr_en, bool drdy_interrupt)
{
	const struct mmc5983ma_config *config = dev->config;
	uint8_t reg = 0;

	reg |= (auto_sr_en << BIT_AUTO_SR_EN) | (drdy_interrupt << BIT_DRDY_INTERRUPT);

	return i2c_reg_write_byte_dt(&config->i2c, MMC5983MA_CONTROL_0, reg);
}

static int mmc5983ma_set_internal_control_1(const struct device *dev, uint16_t bandwidth, bool sw_rst)
{
	const struct mmc5983ma_config *config = dev->config;
	uint8_t bw_bits;
	uint8_t reg = 0;

	switch (bandwidth) {
		case 100:
			bw_bits = MMC5983MA_MBW_100Hz;
			break;
		case 200:
			bw_bits = MMC5983MA_MBW_200Hz;
			break;
		case 400:
			bw_bits = MMC5983MA_MBW_400Hz;
			break;
		case 800:
			bw_bits = MMC5983MA_MBW_800Hz;
			break;
		default:
			LOG_ERR("Invalid bandwidth");
			return -EINVAL;
	}

	reg |= (sw_rst << BIT_SW_RST) | bw_bits;

	return i2c_reg_write_byte_dt(&config->i2c, MMC5983MA_CONTROL_1, reg);
}

static int mmc5983ma_set_internal_control_2(const struct device *dev, uint16_t frequency, uint16_t prd, bool cmm_en, bool en_prd_set)
{
	const struct mmc5983ma_config *config = dev->config;
	uint8_t freq_bits, prd_bits;
	uint8_t reg = 0;

	switch (frequency) {
		case 1:
			freq_bits = MMC5983MA_FREQ_1Hz;
			break;
		case 10:
			freq_bits = MMC5983MA_FREQ_10Hz;
			break;
		case 20:
			freq_bits = MMC5983MA_FREQ_20Hz;
			break;
		case 50:
			freq_bits = MMC5983MA_FREQ_50Hz;
			break;
		case 100:
			freq_bits = MMC5983MA_FREQ_100Hz;
			break;
		case 200:
			freq_bits = MMC5983MA_FREQ_200Hz;
			break;
		case 1000:
			freq_bits = MMC5983MA_FREQ_1000Hz;
			break;
		default:
			LOG_ERR("Invalid frequency");
			return -EINVAL;
	}
	
	switch (prd) {
		case 1:
			prd_bits = MMC5983MA_DT_PRD_SET_1ms;
			break;
		case 25:
			prd_bits = MMC5983MA_DT_PRD_SET_25ms;
			break;
		case 75:
			prd_bits = MMC5983MA_DT_PRD_SET_75ms;
			break;
		case 100:
			prd_bits = MMC5983MA_DT_PRD_SET_100ms;
			break;
		case 250:
			prd_bits = MMC5983MA_DT_PRD_SET_250ms;
			break;
		case 500:
			prd_bits = MMC5983MA_DT_PRD_SET_500ms;
			break;
		case 1000:
			prd_bits = MMC5983MA_DT_PRD_SET_1000ms;
			break;
		case 2000:
			prd_bits = MMC5983MA_DT_PRD_SET_2000ms;
			break;
		default:
			LOG_ERR("Invalid prd set");
			return -EINVAL;
	}

	reg |= (en_prd_set << BIT_EN_PRD_SET) | (prd_bits << 4) | (cmm_en << BIT_CMM_EN) | freq_bits;

	return i2c_reg_write_byte_dt(&config->i2c, MMC5983MA_CONTROL_2, reg);
}

static int mmc5983ma_init(const struct device *dev)
{
	const struct mmc5983ma_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* Set internal control 0 */
	if (mmc5983ma_set_internal_control_0(dev, true, false) < 0) {
		LOG_ERR("Failed to set internal control 0");
		return -EIO;
	}

	/* Set internal control 1 */
	if (mmc5983ma_set_internal_control_1(dev, config->bandwidth, false) < 0) {
		LOG_ERR("Failed to set internal control 1");
		return -EIO;
	}

	/* Set internal control 2 */
	if (mmc5983ma_set_internal_control_2(dev, config->frequency, config->prd_set, true, true) < 0) {
		LOG_ERR("Failed to set internal control 2");
		return -EIO;
	}

	return 0;
}

#define MMC5983MA_DEFINE(inst)								\
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
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,				\
			      &mmc5983ma_driver_api);					\

DT_INST_FOREACH_STATUS_OKAY(MMC5983MA_DEFINE)
