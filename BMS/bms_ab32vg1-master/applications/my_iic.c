/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-29     yjm       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <string.h>
#include <stdlib.h>
#include <rthw.h>

#include "my_iic.h"
#define SLAVE_ADDRESS1    (0x80)

/*
 *  Include MCU Specific Header Files Here
 */

/********* MCU SPECIFIC I2C CODE STARTS HERE**********/
struct rt_i2c_bus_device *inc237;
#define INC237_ADDR 0x80
void mcu_i2cInit(uint8_t busId)
{
    /* Add MCU specific init necessary for I2C to be used */
    //I2C_Init(1);
    inc237 = rt_i2c_bus_device_find("i2c1");
}

int8_t mcu_i2cTransfer( uint8_t busId, uint8_t i2cAddr,
                        uint8_t *dataToWrite, uint8_t writeLength,
                        uint8_t *dataToRead,  uint8_t readLength)
{
    struct rt_i2c_msg msgs[1];

    /*
     *  Add MCU specific I2C read/write code here.
     */
    if(writeLength)
    {
        rt_i2c_master_send(inc237, INC237_ADDR, 0, dataToWrite, writeLength);
        //HAL_I2C_Master_Transmit(&I2C_Handle1, SLAVE_ADDRESS1,dataToWrite,writeLength,1000);
    }
    if(writeLength)
    {
        msgs[0].addr = INC237_ADDR | 1;
        msgs[0].flags = RT_I2C_RD;
        msgs[0].buf = dataToRead;
        msgs[0].len = readLength;

        rt_i2c_transfer(inc237,msgs,1);
         //HAL_I2C_Master_Receive(&I2C_Handle1, SLAVE_ADDRESS1, dataToRead, readLength,1000);
    }


    /*
     *  Add MCU specific return code for error handling
     */

    return (0);
}
