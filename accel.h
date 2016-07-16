/*
 * accel.h
 *
 *  Created on: 18/06/2016
 *      Author: l.espinal
 */

#ifndef SOURCE_ACCEL_H_
#define SOURCE_ACCEL_H_

#include <stdio.h>
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

#include "fsl_flexio_i2c_master.h"

#define BOARD_FLEXIO_BASE FLEXIO0
#define FLEXIO_CLOCK_FREQUENCY 12000000U

#define FLEXIO_I2C_SDA_PIN 4U
#define FLEXIO_I2C_SCL_PIN 5U
#define I2C_BAUDRATE (100000) /* 100K */

#define FOXS8700_WHOAMI (0xC7U)
#define MMA8451_WHOAMI (0x1AU)
#define ACCEL_STATUS (0x00U)
#define ACCEL_XYZ_DATA_CFG (0x0EU)
#define ACCEL_CTRL_REG1 (0x2AU)
#define ACCEL_SYSMOD			0x0B
#define ACCEL_X_OFFSET_REG		0x2F
#define ACCEL_Y_OFFSET_REG		0x30
#define ACCEL_Z_OFFSET_REG		0x31
/* FOXS8700 and MMA8451 have the same who_am_i register address. */
#define ACCEL_WHOAMI_REG (0x0DU)
#define ACCEL_READ_TIMES (10U)

struct accelDat_t{
	int16_t xDat;
	int16_t yDat;
	int16_t zDat;
};

/*  FOXS8700 and MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};
flexio_i2c_master_handle_t g_m_handle;
FLEXIO_I2C_Type i2cDev;
uint8_t g_accel_addr_found = 0x00;
volatile bool completionFlag = false;
volatile bool nakFlag = false;

// accel calibration values, offset relative to actual mount
#define xCal	0xED					// -19
#define yCal	0x12					// 18
#define zCal	0xF1					// -15
const uint8_t offsetVal[]={xCal, yCal, zCal};

static bool I2C_example_readAccelWhoAmI(void);
static bool I2C_write_accel_reg(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_read_accel_regs(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);





#endif /* SOURCE_ACCEL_H_ */
