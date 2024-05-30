/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_MLX90395_H_
#define __SENSOR_MLX90395_H_

#include <zephyr/drivers/i2c.h>

#define MLX90395_I2C_ADDR      0x30

#define MLX90395_XOUT_0        0x00
#define MLX90395_XOUT_1        0x01
#define MLX90395_YOUT_0        0x02
#define MLX90395_YOUT_1        0x03
#define MLX90395_ZOUT_0        0x04
#define MLX90395_ZOUT_1        0x05
#define MLX90395_XYZOUT_2      0x06
#define MLX90395_TOUT          0x07
#define MLX90395_STATUS        0x08
#define MLX90395_CONTROL_0     0x09
#define MLX90395_CONTROL_1     0x0A
#define MLX90395_CONTROL_2     0x0B
#define MLX90395_CONTROL_3     0x0C
#define MLX90395_PRODUCT_ID    0x2F // Should be 0x30

// internal control 0 register
#define BIT_TM_M			    0
#define BIT_TM_T 				1
#define BIT_DRDY_INTERRUPT	    2
#define BIT_SET_MODE        	3
#define BIT_RESET_MODE      	4
#define BIT_AUTO_SR_EN			5

// internal control 1 register
#define MASK_BANDWIDTH          GENMASK(1,0)
#define BIT_SW_RST		        7

// internal control 2 register
#define MASK_FREQUENCY	        GENMASK(2,0) 
#define BIT_CMM_EN			    3	
#define MASK_PRD_SET            GENMASK(6,4)
#define BIT_EN_PRD_SET			7

#define PARAM_NULLFIELD_18BIT 	131072
#define PARAM_NULLFIELD_16BIT 	32768
#define PARAM_MAGN_LSB_GAUSS    16000 // 0.25mG/LSB or 0.0625mG/LSB for 16 or 18 bit

// Frequencies
enum mlx90395_frequency {
	MLX90395_FREQ_ONESHOT,
	MLX90395_FREQ_1Hz,
	MLX90395_FREQ_10Hz,
	MLX90395_FREQ_20Hz,
	MLX90395_FREQ_50Hz,
	MLX90395_FREQ_100Hz,
	MLX90395_FREQ_200Hz, // BW = 0x01 only
	MLX90395_FREQ_1000Hz, // BW = 0x11 only
};

//Bandwidths
enum mlx90395_bandwidth {
	MLX90395_MBW_100Hz,
	MLX90395_MBW_200Hz,
	MLX90395_MBW_400Hz,
	MLX90395_MBW_800Hz,
};

// Set/Reset as a function of measurements
enum mlx90395_prd_set {
	MLX90395_DT_PRD_SET_1ms,
	MLX90395_DT_PRD_SET_25ms,
	MLX90395_DT_PRD_SET_75ms,
	MLX90395_DT_PRD_SET_100ms,
	MLX90395_DT_PRD_SET_250ms,
	MLX90395_DT_PRD_SET_500ms,
	MLX90395_DT_PRD_SET_1000ms,
	MLX90395_DT_PRD_SET_2000ms,
};

struct mlx90395_config {
	struct i2c_dt_spec i2c;
	uint16_t bandwidth;
	uint16_t frequency;
	uint16_t prd_set;
};

struct mlx90395_data {
	uint32_t magn_x;
	uint32_t magn_y;
	uint32_t magn_z;
	struct k_sem sem;
};

#endif /* _SENSOR_MLX90395_H_ */
