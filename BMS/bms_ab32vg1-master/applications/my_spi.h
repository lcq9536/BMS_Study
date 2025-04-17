/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-29     yjm       the first version
 */
#ifndef APPLICATIONS_MY_SPI_H_
#define APPLICATIONS_MY_SPI_H_
#include "stdint.h"

int InitLTC68xx(void);
void spi_init(void);
void cs_high(int n);
void cs_low(int n);
void delay_u(uint16_t micro);
void delay_m(uint16_t milli);
uint8_t spi_read_byte(uint8_t tx_dat);
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    );
void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   );

#endif /* APPLICATIONS_MY_SPI_H_ */
