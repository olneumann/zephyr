#ifndef DRIVERS_SENSOR_ICM20948_REG_H_
#define DRIVERS_SENSOR_ICM20948_REG_H_

#define ICM20948_DEFAULT_ADDRESS                0xEA
#define AK09916_DEFAULT_ADDRESS                 0x0C
#define AK09916_DEFAULT_ID                      0x09
#define ICM20948_REG_BANK_SEL                   0x7F

/* BANK 0 */
#define ICM20948_REG_WHO_AM_I                   0x00
#define ICM20948_REG_USER_CTRL                  0x03
#define ICM20948_REG_PWR_MGMT_1                 0x06
#define ICM20948_REG_ACCEL_XOUT_H               0x2D
#define ICM20948_REG_ACCEL_XOUT_L               0x2E
#define ICM20948_REG_ACCEL_YOUT_H               0x2F
#define ICM20948_REG_ACCEL_YOUT_L               0x30
#define ICM20948_REG_ACCEL_ZOUT_H               0x31
#define ICM20948_REG_ACCEL_ZOUT_L               0x32
#define ICM20948_REG_GYRO_XOUT_H                0x33
#define ICM20948_REG_GYRO_XOUT_L                0x34
#define ICM20948_REG_GYRO_YOUT_H                0x35
#define ICM20948_REG_GYRO_YOUT_L                0x36
#define ICM20948_REG_GYRO_ZOUT_H                0x37
#define ICM20948_REG_GYRO_ZOUT_L                0x38
#define ICM20948_REG_TEMP_OUT_H                 0x39
#define ICM20948_REG_TEMP_OUT_L                 0x3A
#define ICM20948_REG_EXT_SLV_SENS_DATA_00       0x3B

/* BANK 2 */
#define ICM20948_REG_GYRO_SMPLRT_DIV            0x00
#define ICM20948_REG_GYRO_CONFIG_1              0x01
#define ICM20948_REG_GYRO_CONFIG_2              0x02
#define ICM20948_REG_ODR_ALIGN_EN               0x09
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1         0x10
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2         0x11
#define ICM20948_REG_ACCEL_CONFIG               0x14
#define ICM20948_REG_ACCEL_CONFIG_2             0x15

/* BANK 3 */
#define ICM20948_REG_I2C_MST_CTRL               0x01
#define ICM20948_REG_I2C_MST_DELAY_CTRL         0x02
#define ICM20948_REG_I2C_SLV0_ADDR              0x03
#define ICM20948_REG_I2C_SLV0_REG               0x04
#define ICM20948_REG_I2C_SLV0_CTRL              0x05
#define ICM20948_REG_I2C_SLV0_DO                0x06

/* AK09916 registers */
#define AK09916_REG_WIA1                        0x00
#define AK09916_REG_WIA2                        0x01
#define AK09916_REG_HXL                         0x11
#define AK09916_REG_HXH                         0x12
#define AK09916_REG_HYL                         0x13
#define AK09916_REG_HYH                         0x14
#define AK09916_REG_HZL                         0x15
#define AK09916_REG_HZH                         0x16
#define AK09916_REG_CNTL2                       0x31
#define AK09916_REG_CNTL3                       0x32

#endif