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
#include "my_spi.h"


#define SPI_DEVICE_NAME     "qspi10"
//uint8_t pin_cs = rt_pin_get("PE.6");
struct rt_spi_device *spi_dev_6811;
void spi_init(void)
{
    spi_dev_6811= (struct rt_spi_device *)rt_device_find(SPI_DEVICE_NAME);
}
void cs_low(int pin)
{
    //rt_pin_write(pin_cs, PIN_LOW);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_CLEAR);
}

void cs_high(int pin)
{
    //rt_pin_write(pin_cs, PIN_HIGH);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void delay_u(uint16_t micro)
{
  //delayMicroseconds(micro);
    while (micro >0)
        micro--;
}

void delay_m(uint16_t milli)
{
  //delay(milli);
    int loop=milli*2;
    while (loop >0)
        loop--;
}

/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
    uint8_t i;
    uint8_t recvBuff[200];

    for ( i = 0; i < len; i++)
    {
    //HAL_SPI_Transmit(&SPI_Handle2, data+i, 1, 0);
        rt_spi_send(spi_dev_6811,data+i,len);
    }
}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
    uint8_t i;
    uint8_t recvBuff[200];
    //HAL_SPI_TransmitReceive(&SPI_Handle2,tx_Data,recvBuff,tx_len+rx_len,0);
    rt_spi_send_then_recv(spi_dev_6811,tx_Data,tx_len,rx_data,rx_len);
    //for(i=0;i<rx_len;i++)
    {
        //rx_data[i]=recvBuff[i+tx_len];
    }
}


uint8_t spi_read_byte(uint8_t tx_dat)
{
    uint8_t data=0;
    //data = (uint8_t)SPI.transfer(0xFF);
    //LPSPI_DRV_MasterTransfer(LTC6811SPI,(int8_t)0x00,&data,1);  //�����ܵ���Ӳ���쳣������ȥ���ˡ�
    return(data);
}
