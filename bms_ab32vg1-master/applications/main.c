/*
 * Copyright (c) 2020-2021, Bluetrum Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/12/10     greedyhao    The first version
 */

/**
 * Notice!
 * All functions or data that are called during an interrupt need to be in RAM.
 * You can do it the way exception_isr() does.
 */

#include <rtthread.h>
#include "board.h"
#include "my_task.h"

int main(void)
{
    int iCnt=0,n;
    uint8_t pin_r = rt_pin_get("PE.1");
    uint8_t pin_g = rt_pin_get("PE.4");
    uint8_t pin_b = rt_pin_get("PA.1");

    rt_pin_mode(pin_r, PIN_MODE_OUTPUT);
    rt_pin_mode(pin_g, PIN_MODE_OUTPUT);
    rt_pin_mode(pin_b, PIN_MODE_OUTPUT);

    //rt_kprintf("Hello, world\n");

    my_application_init();  /* go to my application */

    while (1)
    {
        n=iCnt%3;
        if(n==0)
        {
            rt_pin_write(pin_r, PIN_LOW);
            rt_pin_write(pin_g, PIN_HIGH);
            rt_pin_write(pin_b, PIN_HIGH);
        }else if(n==1)
        {
            rt_pin_write(pin_r, PIN_HIGH);
            rt_pin_write(pin_g, PIN_LOW);
            rt_pin_write(pin_b, PIN_HIGH);
        }else if(n==2)
        {
            rt_pin_write(pin_r, PIN_HIGH);
            rt_pin_write(pin_g, PIN_HIGH);
            rt_pin_write(pin_b, PIN_LOW);
        }
        rt_thread_mdelay(500);
        iCnt++;
    }
}
