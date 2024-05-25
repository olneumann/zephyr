/*
 * Copyright (c) 2024 Oliver Neumann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_MMC5983MA_
#define __SENSOR_MMC5983MA_

#include <zephyr/drivers/i2c.h>

#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_XOUT_1        0x01
#define MMC5983MA_YOUT_0        0x02
#define MMC5983MA_YOUT_1        0x03
#define MMC5983MA_ZOUT_0        0x04
#define MMC5983MA_ZOUT_1        0x05
#define MMC5983MA_XYZOUT_2      0x06
#define MMC5983MA_TOUT          0x07
#define MMC5983MA_STATUS        0x08
#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B
#define MMC5983MA_CONTROL_3     0x0C
#define MMC5983MA_PRODUCT_ID    0x2F // Should be 0x30

#define MASK_BANDWIDTH          GENMASK(1,0)
#define MASK_FREQUENCY	        GENMASK(2,0) 
#define MASK_PRD_SET            GENMASK(6,4)

#define MMC5983MA_ADDRESS       0x30

// Frequencies
enum mmc5983ma_frequency {
	MMC5983MA_FREQ_ONESHOT,
	MMC5983MA_FREQ_1Hz,
	MMC5983MA_FREQ_10Hz,
	MMC5983MA_FREQ_20Hz,
	MMC5983MA_FREQ_50Hz,
	MMC5983MA_FREQ_100Hz,
	MMC5983MA_FREQ_200Hz, // BW = 0x01 only
	MMC5983MA_FREQ_1000Hz, // BW = 0x11 only
};

//Bandwidths
enum mmc5983ma_bandwidth {
	MMC5983MA_MBW_100Hz,
	MMC5983MA_MBW_200Hz,
	MMC5983MA_MBW_400Hz,
	MMC5983MA_MBW_800Hz,
};

// Set/Reset as a function of measurements
enum mmc5983ma_prd_set {
	MMC5983MA_DT_PRD_SET_1ms,
	MMC5983MA_DT_PRD_SET_25ms,
	MMC5983MA_DT_PRD_SET_75ms,
	MMC5983MA_DT_PRD_SET_100ms,
	MMC5983MA_DT_PRD_SET_250ms,
	MMC5983MA_DT_PRD_SET_500ms,
	MMC5983MA_DT_PRD_SET_1000ms,
	MMC5983MA_DT_PRD_SET_2000ms,
};

struct mmc5983ma_config {
	struct i2c_dt_spec i2c;
	uint8_t bandwidth;
	uint8_t frequency;
	uint8_t prd_set;
};

struct mmc5983ma_data {
	int16_t magn_x;
	int16_t magn_y;
	int16_t magn_z;
};

#endif /* _SENSOR_MMC5983MA_ */
