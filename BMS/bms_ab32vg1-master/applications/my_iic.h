/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-29     yjm       the first version
 */
#ifndef APPLICATIONS_MY_IIC_H_
#define APPLICATIONS_MY_IIC_H_

#include "stdint.h"
/*
 *  ======== mcu_i2cInit ========
 *  Initialize the specified I2C bus for first use
 */
extern void mcu_i2cInit(uint8_t busId);

/*
 *  ======== mcu_i2cTransfer ========
 *  Transfer data to and from an I2C slave
 *
 *  If writeLength is non-zero, mcu_i2cTransfer always performs the write
 *  transfer first.
 *
 *  @param busId         id of an I2C bus to access for the transfer
 *  @param sensorAddress I2C address of peripheral to access
 *  @param dataToWrite   non-NULL pointer to a buffer of at least writeLength
 *                       bytes; may be NULL if writeLength = 0.
 *  @param writeLength   number of bytes to write from the dataToWrite array
 *  @param dataToRead    non-NULL pointer to a buffer of at least readLength
 *                       bytes; may be NULL if readLength = 0.
 *  @param readLength    number of bytes to read into dataToRead array
 *
 *  @return              0 if successful, otherwise non-zero
 */
extern int8_t mcu_i2cTransfer(uint8_t busId, uint8_t sensorAddress,
                              uint8_t *dataToWrite, uint8_t writeLength,
                              uint8_t *dataToRead,  uint8_t readLength);

#endif /* APPLICATIONS_MY_IIC_H_ */
