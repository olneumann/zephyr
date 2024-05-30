/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_MLX90393_H_
#define __SENSOR_MLX90393_H_

#include <zephyr/drivers/i2c.h>

enum { STATUS_OK = 0, STATUS_ERROR = 0xff } return_status_t;
enum { Z_FLAG = 0x8, Y_FLAG = 0x4, X_FLAG = 0x2, T_FLAG = 0x1 } axis_flag_t;
enum { I2C_BASE_ADDR = 0x0c };
enum { GAIN_SEL_REG = 0x0, GAIN_SEL_MASK = 0x0070, GAIN_SEL_SHIFT = 4 };
enum { HALLCONF_REG = 0x0, HALLCONF_MASK = 0x000f, HALLCONF_SHIFT = 0 };
enum { BURST_SEL_REG = 0x1, BURST_SEL_MASK = 0x03c0, BURST_SEL_SHIFT = 6};
enum { TRIG_INT_SEL_REG = 0x1, TRIG_INT_SEL_MASK = 0x8000, TRIG_INT_SEL_SHIFT = 15 };
enum { EXT_TRIG_REG = 0x1, EXT_TRIG_MASK = 0x0800, EXT_TRIG_SHIFT = 11 };
enum { OSR_REG = 0x2, OSR_MASK = 0x0003, OSR_SHIFT = 0 };
enum { OSR2_REG = 0x2, OSR2_MASK = 0x1800, OSR2_SHIFT = 11 };
enum { DIG_FLT_REG = 0x2, DIG_FLT_MASK = 0x001c, DIG_FLT_SHIFT = 2 };
enum { RES_XYZ_REG = 0x2, RES_XYZ_MASK = 0x07e0, RES_XYZ_SHIFT = 5 };
enum { TCMP_EN_REG = 0x1, TCMP_EN_MASK = 0x0400, TCMP_EN_SHIFT = 10 };
enum { X_OFFSET_REG = 4, Y_OFFSET_REG = 5, Z_OFFSET_REG = 6 };
enum { WOXY_THRESHOLD_REG = 7, WOZ_THRESHOLD_REG = 8, WOT_THRESHOLD_REG = 9 };
enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,
		POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,
		RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };

enum {
	MLX90393_CMD_NOP 				= 0x00, /* No operation */
	MLX90393_CMD_START_BURST		= 0x10, /* Start burst mode */
	MLX90393_CMD_WAKE_ON_CHANGE		= 0x20, /* Start wake-up on change mode */
	MLX90393_CMD_START_MEASUREMENT	= 0x30, /* Start single measurment (polling mode) */
	MLX90393_CMD_READ_MEASUREMENT	= 0x40, /* Read measurement */
	MLX90393_CMD_READ_REGISTER		= 0x50, /* Read form a register */
	MLX90393_CMD_WRITE_REGISTER		= 0x60, /* Write to a register */
	MLX90393_CMD_EXIT				= 0x80, /* Exit */
	MLX90393_CMD_MEMORY_RECALL		= 0xD0, /* Memory recall */
	MLX90393_CMD_MEMORY_STORE		= 0xE0, /* Memory store */
	MLX90393_CMD_RESET				= 0xF0, /* Reset */	
};

/* 
Example conversion:
- HALLCONF 	= 0xC
- GAIN_SEL 	= 0x7 (default)
- RES 		= 0 (default)

-> Table 17, T = 25°C:

 	SENS_XY  = 0.150 [LSB/µT]
 	SENS_Z   = 0.242 [LSB/µT]
*/

#define PARAM_MAGN_LSB_GAUSS_XY 666666 // [T/LSB]
#define PARAM_MAGN_LSB_GAUSS_Z 413223 // [T/LSB] 

enum mlx90393_mode {
	MLX90393_MODE_IDLE,
	MLX90393_MODE_BURST,
	MLX90393_MODE_SINGLE_MEASUREMENT,
	MLX90393_MODE_WAKE_ON_CHANGE,
};

struct mlx90393_config {
	struct i2c_dt_spec i2c;
	uint8_t zyxt;
	uint8_t bdr;
	enum mlx90393_mode mode;
};

struct mlx90393_data {
 	uint16_t temp;
 	uint16_t magn_x;
 	uint16_t magn_y;
 	uint16_t magn_z;
	struct k_sem sem;
};

#endif /* _SENSOR_MLX90393_H_ */
