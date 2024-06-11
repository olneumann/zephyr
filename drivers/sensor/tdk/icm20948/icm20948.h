/#ifndef DRIVERS_SENSOR_ICM20948_H_
#define DRIVERS_SENSOR_ICM20948_H_

#include <zephyr/drivers/i2c.h>

struct icm20948_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t magn_x;
    int16_t magn_y;
    int16_t magn_z;

    int16_t temp;
};

struct icm20948_config {
	struct i2c_dt_spec i2c;
};

#endif