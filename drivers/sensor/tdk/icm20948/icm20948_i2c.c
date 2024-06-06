/* ST Microelectronics ISM330DHCX 6-axis IMU sensor driver
 *
 * Copyright (c) 2020 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ism330dhcx.pdf
 */

#define DT_DRV_COMPAT st_ism330dhcx

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "icm20948.h"

// #if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_i2c_read(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			       uint8_t len)
{
	const struct icm20948_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, reg_addr, value, len);
}

static int icm20948_i2c_write(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				uint8_t len)
{
	const struct icm20948_config *cfg = dev->config;

	return i2c_burst_write_dt(&cfg->i2c, reg_addr, value, len);
}

int icm20948_i2c_init(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus device is not ready");
		return -ENODEV;
	};

	data->ctx_i2c.read_reg = (stmdev_read_ptr) icm20948_i2c_read;
	data->ctx_i2c.write_reg = (stmdev_write_ptr) icm20948_i2c_write;
	data->ctx_i2c.mdelay = (stmdev_mdelay_ptr) stmemsc_mdelay;

	data->ctx = &data->ctx_i2c;
	data->ctx->handle = (void *)dev;

	return 0;
}
// #endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
