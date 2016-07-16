/*
 * accel.c
 *
 *  Created on: 17/06/2016
 *      Author: l.espinal
 */

#include "accel.h"

static void flexio_i2c_master_callback(FLEXIO_I2C_Type *base,
                                       flexio_i2c_master_handle_t *handle,
                                       status_t status,
                                       void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_FLEXIO_I2C_Nak)
    {
        nakFlag = true;
    }
}

static bool I2C_example_readAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device = false;
    bool result = false;
    uint8_t i = 0;
    uint32_t j = 0;

    flexio_i2c_master_config_t masterConfig;

    /*do hardware configuration*/
    i2cDev.flexioBase = BOARD_FLEXIO_BASE;
    i2cDev.SDAPinIndex = FLEXIO_I2C_SDA_PIN;
    i2cDev.SCLPinIndex = FLEXIO_I2C_SCL_PIN;
    i2cDev.shifterIndex[0] = 0U;
    i2cDev.shifterIndex[1] = 1U;
    i2cDev.timerIndex[0] = 0U;
    i2cDev.timerIndex[1] = 1U;

    /*
     * masterConfig.enableMaster = true;
     * masterConfig.enableInDoze = false;
     * masterConfig.enableInDebug = true;
     * masterConfig.enableFastAccess = false;
     * masterConfig.baudRate_Bps = 100000U;
     */
    FLEXIO_I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    FLEXIO_I2C_MasterInit(&i2cDev, &masterConfig, FLEXIO_CLOCK_FREQUENCY);

    flexio_i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = g_accel_address[0];
    masterXfer.direction = kFLEXIO_I2C_Read;
    masterXfer.subaddress = who_am_i_reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &who_am_i_value;
    masterXfer.dataSize = 1;

    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_accel_address[i];

        FLEXIO_I2C_MasterTransferNonBlocking(&i2cDev, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((nakFlag == false) && (completionFlag == false))
        {
        }
        if (nakFlag == true)
        {
            nakFlag = false;
        }
        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
        for (j = 0; j < 0xFFF; j++)
        {
            __NOP();
        }
    }

    if (find_device == true)
    {
        switch (who_am_i_value)
        {
            case FOXS8700_WHOAMI:
                PRINTF("Found a FOXS8700 on board, the device address is 0x%02X. \r\n", masterXfer.slaveAddress);
                result = true;
                break;
            case MMA8451_WHOAMI:
                PRINTF("Found a MMA8451 on board, the device address is 0x%02X. \r\n", masterXfer.slaveAddress);
                result = true;
                break;
            default:

                PRINTF("Found a device, the WhoAmI value is 0x%02X\r\n", who_am_i_value);
                PRINTF("It's not MMA8451 or FXOS8700. \r\n");
                PRINTF("The device address is 0x%02X. \r\n", masterXfer.slaveAddress);
                result = false;
                break;
        }

        return result;
    }
    else
    {
        PRINTF("Not a successful i2c communication\r\n");
        return false;
    }
}

static bool I2C_write_accel_reg(FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    flexio_i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kFLEXIO_I2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    FLEXIO_I2C_MasterTransferNonBlocking(&i2cDev, &g_m_handle, &masterXfer);

    /*  Wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_read_accel_regs(
    FLEXIO_I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    flexio_i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kFLEXIO_I2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    FLEXIO_I2C_MasterTransferNonBlocking(&i2cDev, &g_m_handle, &masterXfer);

    /*  Wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

void initAccel(void)
{
	bool isThereAccel = false;

    FLEXIO_I2C_MasterTransferCreateHandle(&i2cDev, &g_m_handle, flexio_i2c_master_callback, NULL);
    isThereAccel = I2C_example_readAccelWhoAmI();

    /*  read the accel xyz value if there is accel device on board */
    if (true == isThereAccel)
    {
        uint8_t databyte = 0;
        uint8_t accel_reg = 0;
        uint8_t buff[8];
        /*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
        /*  write 0000 0000 = 0x00 to accelerometer control register 1 */
        /*  standby */
        /*  [7-1] = 0000 000 */
        /*  [0]: active=0 */
        accel_reg = ACCEL_CTRL_REG1;
        databyte = 0;
        I2C_write_accel_reg(&i2cDev, g_accel_addr_found, accel_reg, databyte);


        databyte = 0xFF;
        while((databyte&0x03)!=0)
        {
        	I2C_read_accel_regs(&i2cDev, g_accel_addr_found, ACCEL_SYSMOD, &databyte, 1);
        }
        // calibration
 /**/
        accel_reg= ACCEL_X_OFFSET_REG;
        I2C_write_accel_reg(&i2cDev, g_accel_addr_found, accel_reg, xCal);
        accel_reg= ACCEL_Y_OFFSET_REG;
        I2C_write_accel_reg(&i2cDev, g_accel_addr_found, accel_reg, yCal);
        accel_reg= ACCEL_Z_OFFSET_REG;
        I2C_write_accel_reg(&i2cDev, g_accel_addr_found, accel_reg, zCal);
       	I2C_read_accel_regs(&i2cDev, g_accel_addr_found, ACCEL_X_OFFSET_REG, buff, 3);
        /*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
        /*  [7]: reserved */
        /*  [6]: reserved */
        /*  [5]: reserved */
        /*  [4]: hpf_out=0 */
        /*  [3]: reserved */
        /*  [2]: reserved */
        /*  [1-0]: fs=00 for accelerometer range of +/-2g range with 0.488mg/LSB */
        /*  databyte = 0x00; */
        accel_reg = ACCEL_XYZ_DATA_CFG;
        databyte = 0x00;
        I2C_write_accel_reg(&i2cDev, g_accel_addr_found, accel_reg, databyte);

        /*  write 0000 1101 = 0x0D to accelerometer control register 1 */
        /*  [7-6]: aslp_rate=00 */
        /*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
        /*  [2]: lnoise=1 for low noise mode */
        /*  [1]: f_read=0 for normal 16 bit reads */
        /*  [0]: active=1 to take the part out of standby and enable sampling */
        /*   databyte = 0x0D; */
        accel_reg = ACCEL_CTRL_REG1;
        databyte = 0x0d;
        I2C_write_accel_reg(&i2cDev, g_accel_addr_found, accel_reg, databyte);
    }
}
/*
void readAccelData(void)
{
	uint32_t i = 0;
	uint8_t readBuff[7];
	int16_t x, y, z;
	uint8_t status0_value = 0;

	PRINTF("The accel values:\r\n");
    for (i = 0; i < ACCEL_READ_TIMES; i++)
    {
        status0_value = 0;
        //  wait for new data are ready.
        while (status0_value != 0xff)
        {
            I2C_read_accel_regs(&i2cDev, g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
        }

        //  Multiple-byte Read from STATUS (0x00) register
        I2C_read_accel_regs(&i2cDev, g_accel_addr_found, ACCEL_STATUS, readBuff, 7);

        status0_value = readBuff[0];
        x = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) / 4U;
        y = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) / 4U;
        z = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) / 4U;

        PRINTF("status_reg = 0x%x , x = %5d , y = %5d , z = %5d \r\n", status0_value, x, y, z);
    }
}
*/
struct accelDat_t readAccelData(void){

	uint8_t status0_value = 0;
    uint8_t readBuff[7];
    struct accelDat_t temp;

    status0_value=0;
    //  wait for new data are ready.
    while (status0_value != 0xff)
     {
         I2C_read_accel_regs(&i2cDev, g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
     }

     //  Multiple-byte Read from STATUS (0x00) register
     I2C_read_accel_regs(&i2cDev, g_accel_addr_found, ACCEL_STATUS, readBuff, 7);

     status0_value = readBuff[0];
     temp.xDat = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) / 4U;
     temp.yDat = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) / 4U;
     temp.zDat = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) / 4U;
     return temp;
}


