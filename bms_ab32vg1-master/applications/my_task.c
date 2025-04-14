/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-15     yjm       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "my_timer.h"

#include "ina237.h"
#include "temperature.h"
#include "cellvt.h"
#include "packetvc.h"
#include "bms.h"

struct rt_semaphore sem_timer;

ALIGN(RT_ALIGN_SIZE)

static struct rt_semaphore rx_sem;
struct rt_semaphore sem_sox;

/* VVVVVVVVVVVVVVVVVVVVVVVVVV INA237  VVVVVVVVVVVVVVVVVVVVVVVVVV */
static rt_uint8_t ina237_stack[ 512 ];
static struct rt_thread ina237_thread;
static void ina237_thread_entry(void* parameter)
{
    Pack_Init(0);
    while (1)
    {
        g_fPackVoltV = GetPacketVoltage();
        rt_thread_mdelay(10);
        g_fPackCurrA = GetPacketCurrent();
        rt_thread_mdelay(10);
    }
}

/* VVVVVVVVVVVVVVVVVVVVVVVVVV LTC6811  VVVVVVVVVVVVVVVVVVVVVVVVVV */
static rt_uint8_t ltc6811_stack[ 1024 ];
static struct rt_thread ltc6811_thread;
static void ltc6811_thread_entry(void* parameter)
{
    LTC6811_init();
    while (1)
    {
        GetCellV(&MyPacket);
        rt_thread_mdelay(10);
        GetCellT(&MyPacket);
        ProcessData(&MyPacket,&stAlarm,&uDataOverFlag);
        //SyncData(&MyPacket);
        rt_thread_mdelay(10);
    }
}

/* VVVVVVVVVVVVVVVVVVVVVVVVVV SOX Thread  VVVVVVVVVVVVVVVVVVVVVVVVVV */
/* about sync thread */
static rt_mutex_t uart_mutex = RT_NULL;
static rt_uint8_t soc_stack[ 512 ];
static struct rt_thread soc_thread;
static void get_soc(void* parameter)
{
    g_Soc = GetSocFromCellV((float )MyPacket.usEvgCellV/1000.0f);
    while(1)
    {
        rt_sem_take(&sem_sox, RT_WAITING_FOREVER);
        g_Soc = g_Soc + (float )TIMCYCLE*g_fPackCurrA/fSOC_Q;
    }
}


/* VVVVVVVVVVVVVVVVVVVVVVVVVV loop thread  VVVVVVVVVVVVVVVVVVVVVVVVVV */
#define MAXTEMPNUMBER 3
int g_iTemperature[MAXTEMPNUMBER];
uint8_t gu8_TxBuffer_Fram[128];
uint8_t gu8_RxBuffer_Fram[128];
static rt_mutex_t temp_mutex = RT_NULL;
static rt_uint8_t loop_stack[ 1024 ];
static struct rt_thread loop_st;
static void loop_thread(void* parameter)
{
    int iCnt=0;
    int i,m,j;
    temp_mutex = rt_mutex_create("dmutex", RT_IPC_FLAG_PRIO);

    while(1)
    {
        if(iCnt%10==0)
        {
            //
            PrintCellMsg();

            //
            rt_kprintf("Pack current[%.4f]|Pack voltage[%.4f]",g_fPackCurrA,g_fPackVoltV);//g_fPackVoltV);

            //
            CheckWarning();
            rt_kprintf("\r\n");
        }//
        iCnt++;
        rt_thread_mdelay(100);
    }
}

int my_application_init(void)
{
    rt_thread_t init_thread;
    rt_err_t result;
    hwtimer_sample();       /* initial hardware timer */
    rt_sem_init(&sem_timer, "sem_timer", 0, RT_IPC_FLAG_FIFO);
    /* vvvvvvvvvvvvvvvv INA237(I2C) vvvvvvvvvvvvvvvv */
    /* INA237 thread 总电压、总电流采集 */
    result = rt_thread_init(&ina237_thread,
                            "ina237",
                            ina237_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&ina237_stack[0],
                            sizeof(ina237_stack),
                            2,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&ina237_thread);
    }

    /* vvvvvvvvvvvvvvvv ltc6811(SPI) vvvvvvvvvvvvvvvv */
    /* ltc6811 thread 单体电压、温度采集 */
    result = rt_thread_init(&ltc6811_thread,
                            "ltc6811",
                            ltc6811_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&ltc6811_stack[0],
                            sizeof(ltc6811_stack),
                            3,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&ltc6811_thread);
    }

    /* vvvvvvvvvvvvvvvv soc vvvvvvvvvvvvvvvv */
    /* 计算SOC值  */
    result = rt_thread_init(&soc_thread,
                            "soc",
                            get_soc,
                            RT_NULL,
                            (rt_uint8_t*)&soc_stack[0],
                            sizeof(soc_stack),
                            6,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&soc_thread);
    }

    /* vvvvvvvvvvvvvvvv loop thread vvvvvvvvvvvvvvvv */
    /* uart recv thread 串口上报单体电压、单体温度、总电压、总电流、SOC值 */
    /* 如果有欠压、欠温、过压、过温、过流、压差过大，串口上报，LED报警 */
    result = rt_thread_init(&loop_st,
                            "loop",
                            loop_thread,
                            RT_NULL,
                            (rt_uint8_t*)&loop_stack[0],
                            sizeof(loop_stack),
                            25,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&loop_st);
    }

}
