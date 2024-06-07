#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "icm20948.h"
#include "icm20948_reg.h"

LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

#define ICM20948_TEMP_RANGE 334
#define AK09916_MAGNETOMETER_RANGE 7

static const uint16_t icm20948_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164 // LSB/(dps/10)
};

static const uint16_t icm20948_accel_sensitivity_shift[]= { // binary shift
	14, 13, 12, 11 // LSB/g
};

static const double icm20948_magn_sensitivity = 6666; // LSB/mG

void icm20948_set_correct_bank(const struct device *dev, uint8_t bank)
{
	const struct icm20948_config *config = dev->config;
	static uint8_t current_bank = 255; // Save the previously set user bank

	if (bank == current_bank) {
		return; // Return if wanted user bank is the same as previously set
	}

	i2c_reg_write_byte_dt(&config->i2c, ICM20948_REG_BANK_SEL, bank << 4);

	current_bank = bank;
}

static int icm20948_write_register(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t data)
{
	const struct icm20948_config *config = dev->config;

	icm20948_set_correct_bank(dev, bank);

	return i2c_reg_write_byte_dt(&config->i2c, reg, data);
}

static int icm20948_read_register(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t *data)
{
	const struct icm20948_config *config = dev->config;

	icm20948_set_correct_bank(dev, bank);

	return i2c_reg_read_byte_dt(&config->i2c, reg, data);
	;
}

static int ak09916_write_register(const struct device *dev, uint8_t reg, uint8_t data)
{
	int err = 0;

	err |= icm20948_write_register(
		dev, 3, ICM20948_REG_I2C_SLV0_ADDR,
		AK09916_DEFAULT_ADDRESS); // set I2C slave 0 to write to AK09916 address
	err |= icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_REG,
				       reg); // start slave 0 write from register
	err |= icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_DO,
				       data); // set data to be written

	k_msleep(100);

	return err == 0 ? 0 : -1;
}

static int ak09916_read_register(const struct device *dev, uint8_t reg, uint8_t *data)
{
	int err = 0;

	err |= icm20948_write_register(
		dev, 3, ICM20948_REG_I2C_SLV0_ADDR,
		0x80 | AK09916_DEFAULT_ADDRESS); // set I2C slave 0 to read from AK09916 address
	err |= icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_REG,
				       reg); // set register to start reading from
	err |= icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_CTRL,
				       0x81); // enable read data from slave 0 and read 1 byte

	k_msleep(100);

	err |= icm20948_read_register(dev, 0, ICM20948_REG_EXT_SLV_SENS_DATA_00,
				      data); // read data from I2C

	return err == 0 ? 0 : -1;
}

static void icm20948_convert_accel(struct sensor_value *val, int16_t raw_val, uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static void icm20948_convert_gyro(struct sensor_value *val, int16_t raw_val, uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) / (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

static void icm20948_convert_magn(struct sensor_value *val, int16_t raw_val)
{
	double value = (double)raw_val / icm20948_magn_sensitivity;
	sensor_value_from_double(val, value);
}

static void icm20948_convert_temp(struct sensor_value *val, int16_t raw_val)
{
	val->val1 = ((raw_val - 21) / ICM20948_TEMP_RANGE) + 21;
	val->val2 = (raw_val - 21) % ICM20948_TEMP_RANGE;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20948_convert_accel(&val[0], data->accel_x, icm20948_accel_sensitivity_shift[3]);
		icm20948_convert_accel(&val[1], data->accel_y, icm20948_accel_sensitivity_shift[3]);
		icm20948_convert_accel(&val[2], data->accel_z, icm20948_accel_sensitivity_shift[3]);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm20948_convert_accel(val, data->accel_x, icm20948_accel_sensitivity_shift[3]);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20948_convert_accel(val, data->accel_y, icm20948_accel_sensitivity_shift[3]);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_convert_accel(val, data->accel_z, icm20948_accel_sensitivity_shift[3]);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20948_convert_gyro(&val[0], data->gyro_x, icm20948_gyro_sensitivity_x10[3]);
		icm20948_convert_gyro(&val[1], data->gyro_y, icm20948_gyro_sensitivity_x10[3]);
		icm20948_convert_gyro(&val[2], data->gyro_z, icm20948_gyro_sensitivity_x10[3]);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20948_convert_gyro(val, data->gyro_x, icm20948_gyro_sensitivity_x10[3]);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20948_convert_gyro(val, data->gyro_y, icm20948_gyro_sensitivity_x10[3]);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20948_convert_gyro(val, data->gyro_z, icm20948_gyro_sensitivity_x10[3]);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		icm20948_convert_magn(&val[0], data->magn_x);
		icm20948_convert_magn(&val[1], data->magn_y);
		icm20948_convert_magn(&val[2], data->magn_z);
		break;
	case SENSOR_CHAN_MAGN_X:
		icm20948_convert_magn(val, data->magn_x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		icm20948_convert_magn(val, data->magn_y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		icm20948_convert_magn(val, data->magn_z);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20948_convert_temp(val, data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct icm20948_config *config = dev->config;
	struct icm20948_data *data = dev->data;
	int err;

	uint8_t rx_buffer[20];
	icm20948_set_correct_bank(dev, 0); // set bank to 0

	err = i2c_burst_read_dt(&config->i2c, ICM20948_REG_ACCEL_XOUT_H, rx_buffer,
				sizeof(rx_buffer));
	if (err) {
		LOG_ERR("ERROR: Failed to read data from sensor (%d)", err);
		return err;
	}

	int16_t *p = (int16_t *)&rx_buffer[0];

	data->accel_x = sys_be16_to_cpu(p[0]);
	data->accel_y = sys_be16_to_cpu(p[1]);
	data->accel_z = sys_be16_to_cpu(p[2]);
	data->gyro_x = sys_be16_to_cpu(p[3]);
	data->gyro_y = sys_be16_to_cpu(p[4]);
	data->gyro_z = sys_be16_to_cpu(p[5]);
	data->temp = sys_be16_to_cpu(p[6]);
	data->magn_x = sys_le16_to_cpu(p[7]);
	data->magn_y = sys_le16_to_cpu(p[8]);
	data->magn_z = sys_le16_to_cpu(p[9]);

	return 0;
}

static int icm20948_init(const struct device *dev)
{
	const struct icm20948_config *config = dev->config;
	uint8_t value;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	icm20948_write_register(dev, 0, ICM20948_REG_PWR_MGMT_1, 0x81); // reset unit
	k_sleep(K_MSEC(100)); // wait for reset to complete
	icm20948_write_register(dev, 0, ICM20948_REG_PWR_MGMT_1, 0x01); // exit sleep mode

	icm20948_read_register(dev, 0, ICM20948_REG_WHO_AM_I, &value); // whoami
	if (value != ICM20948_DEFAULT_ADDRESS) {
		LOG_ERR("Error: Unexpected address at sensor. Expected 0x%02X, "
			"received 0x%02X",
			ICM20948_DEFAULT_ADDRESS, value);
		return -EIO;
	}

	icm20948_write_register(dev, 2, ICM20948_REG_ODR_ALIGN_EN, 0x01); // enable ODR start-time alignment

	/* Gyro Settings:
	
	- Gyro sample rate divider. Divides the internal sample rate to generate the sample
	  rate that controls sensor data output rate, FIFO sample rate, and DMP sequence rate. 
	  
	  		ODR: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
	
	- Gyro config register. Set gyro full scale to 2000dps and disable DLPF
	
	*/

	icm20948_write_register(dev, 2, ICM20948_REG_GYRO_SMPLRT_DIV, 0x00); 
	icm20948_write_register(dev, 2, ICM20948_REG_GYRO_CONFIG_1,	0x06); 

	/* Accel Settings:

	- Accel sample rate divider. Divides the internal sample rate to generate the sample
	  rate that controls sensor data output rate, FIFO sample rate, and DMP sequence rate. 
	  
	  		ODR: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]

	- Accel config register. Set accel full scale to 16g and disable DLPF
	
	*/

	icm20948_write_register(dev, 2, ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00); 
	icm20948_write_register(dev, 2, ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x00);
	icm20948_write_register(dev, 2, ICM20948_REG_ACCEL_CONFIG, 0x06); 

	/* User Bank 0 Settings:

	- Enable the I2C Master I/F module and disable FIFO and DMP

	*/

	icm20948_write_register(dev, 0, ICM20948_REG_USER_CTRL,	0x20); // enable I2C master, disable FIFO and DMP

	/* User Bank 3 Settings: 
	
	- Set I2C master clock to 345.6 kHz with 46.67% duty cycle
	- Enable delay control on slave 0
	- Set slave address to AK09916
	- Set slave register to start reading from
	- Set slave control to read 1 byte
	
	*/

	icm20948_write_register(dev, 3, ICM20948_REG_I2C_MST_CTRL, 0x07); 
	icm20948_write_register(dev, 3, ICM20948_REG_I2C_MST_DELAY_CTRL, 0x01); 


	ak09916_write_register(dev, AK09916_REG_CNTL3, 0x01); // reset AK09916
	k_msleep(100);

	icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_ADDR, 0x80 | AK09916_DEFAULT_ADDRESS); // set address and register to start reading from
	icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_REG, 0x01);
	icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_CTRL,	0x81); // enable read data from slave 0 and read 1 byte

	ak09916_read_register(dev, AK09916_REG_WIA2, &value);
	if (value != AK09916_DEFAULT_ID) {
		LOG_WRN("Error: Unexpected ID at magnetometer. Expected 0x%02X, "
			"received 0x%02X",
			AK09916_DEFAULT_ID, value);
	}

	ak09916_write_register(dev, AK09916_REG_CNTL2, 0x08); // set AK09916 to continous mode 100Hz
	icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_ADDR,	0x80 | AK09916_DEFAULT_ADDRESS); // set I2C slave 0 to write to AK09916 address
	icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_HXL); // start slave 0 read from data register
	icm20948_write_register(dev, 3, ICM20948_REG_I2C_SLV0_CTRL,	0x88); // enable read data from slave 0 and read 8 bytes every time

	return 0;
}

static const struct sensor_driver_api icm20948_driver_api = {
	.sample_fetch = &icm20948_sample_fetch,
	.channel_get = &icm20948_channel_get,
};

#define ICM20948_DEFINE(inst)                                                                      \
	static struct icm20948_data icm20948_data_##inst;                                          \
                                                                                                   \
	static const struct icm20948_config icm20948_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, icm20948_init, NULL, &icm20948_data_##inst,             \
				     &icm20948_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &icm20948_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_DEFINE)
