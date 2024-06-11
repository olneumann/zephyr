/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_MLX90395_H_
#define __SENSOR_MLX90395_H_

#include <zephyr/drivers/i2c.h>

enum { STATUS_OK = 0, STATUS_ERROR = 0xff } return_status_t;
enum { Z_FLAG = 0x8, Y_FLAG = 0x4, X_FLAG = 0x2, T_FLAG = 0x1 } axis_flag_t;
enum { I2C_BASE_ADDR = 0x0c };
enum { GAIN_SEL_REG = 0x0, GAIN_SEL_MASK = 0x00f0, GAIN_SEL_SHIFT = 4 };
enum { HALLCONF_REG = 0x0, HALLCONF_MASK = 0x000f, HALLCONF_SHIFT = 0 };
enum { BURST_SEL_REG = 0x1, BURST_SEL_MASK = 0x03c0, BURST_SEL_SHIFT = 6};
enum { TRIG_INT_SEL_REG = 0x1, TRIG_INT_SEL_MASK = 0x8000, TRIG_INT_SEL_SHIFT = 15 };
enum { EXT_TRIG_REG = 0x1, EXT_TRIG_MASK = 0x0800, EXT_TRIG_SHIFT = 11 };
enum { OSR_REG = 0x2, OSR_MASK = 0x0003, OSR_SHIFT = 0 };
enum { OSR2_REG = 0x2, OSR2_MASK = 0x1800, OSR2_SHIFT = 11 };
enum { DIG_FLT_REG = 0x2, DIG_FLT_MASK = 0x001c, DIG_FLT_SHIFT = 2 };
enum { RES_XYZ_REG = 0x2, RES_XYZ_MASK = 0x07e0, RES_XYZ_SHIFT = 5 };
enum { TCMP_EN_REG = 0x1, TCMP_EN_MASK = 0x0400, TCMP_EN_SHIFT = 10 };
enum { X_OFFSET_REG = 0x06, Y_OFFSET_REG = 0x07, Z_OFFSET_REG = 0x08 };
enum { WOXY_THRESHOLD_REG = 0x0E, WOZ_THRESHOLD_REG = 0x0F, WOTT_THRESHOLD_REG = 0x10, WOVT_THRESHOLD_REG = 0x11 };
enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,
		POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,
		RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };

enum {
	MLX90395_CMD_NOP 				= 0x00, /* No operation */
	MLX90395_CMD_START_BURST		= 0x10, /* Start burst mode */
	MLX90395_CMD_WAKE_ON_CHANGE		= 0x20, /* Start wake-up on change mode */
	MLX90395_CMD_START_MEASUREMENT	= 0x30, /* Start single measurement (polling mode) */
	MLX90395_CMD_READ_MEASUREMENT	= 0x80, /* Read measurement, 0x40 for MLX90393 */
	MLX90395_CMD_EXIT				= 0x80, /* Exit */
	MLX90395_CMD_MEMORY_RECALL		= 0xD0, /* Memory recall */
	MLX90395_CMD_MEMORY_STORE		= 0xE0, /* Memory store */
	MLX90395_CMD_RESET				= 0xF0, /* Reset */	
};

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

enum mlx90393_res {
  MLX90395_RES_16,
  MLX90395_RES_17,
  MLX90395_RES_18,
  MLX90395_RES_19,
} mlx90393_res_t;

static const float gainMultipliers[16] = {
    0.2, 0.25,  0.3333, 0.4, 0.5,  0.6, 0.75,  1,
    0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.375, 0.5};

enum mlx90395_mode {
	MLX90395_MODE_IDLE,
	MLX90395_MODE_BURST,
	MLX90395_MODE_SINGLE_MEASUREMENT,
	MLX90395_MODE_WAKE_ON_CHANGE,
};

struct mlx90395_config {
	struct i2c_dt_spec i2c;
	uint16_t bandwidth;
	uint16_t frequency;
	uint16_t prd_set;
	uint8_t zyxt;
	uint8_t bdr;
	enum mlx90395_mode mode;
};

struct mlx90395_data {
	uint16_t magn_x;
	uint16_t magn_y;
	uint16_t magn_z;
	struct k_sem sem;
};

#endif /* _SENSOR_MLX90395_H_ */
