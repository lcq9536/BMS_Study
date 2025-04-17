/*=======================================================================
 *Subsystem:   ���
 *File:        Task_SysTimeGet.c
 *Author:      Wenming
 *Description: 
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "includes.h"

SysTime_T g_SysTime;
/*=======================================================================
 *������:      Time_Init(void)
 *����:        ʱ�ӳ�ʼ��
 *����:        ��       
 *���أ�       ��
 
 *˵����       
========================================================================*/
void Time_Init(void)
{
  uint32 timestart;
  DS3231_Read_Time();
  timestart = ((uint32)Read_IIC_Time.IIC_Read_Year<<24) + ((uint32)Read_IIC_Time.IIC_Read_Month<<16) + ((uint32)Read_IIC_Time.IIC_Read_Day<<8) + ((uint32)(Read_IIC_Time.IIC_Read_Hour));
  g_SysTime.SOC_Static_Time = (timestart - g_SysTime.BMS_PowerOff_Time);//SOC�����õľ���ʱ��(�ܶϵ�ʱ��)
  
  g_SysTime.BMS_StartRun_Time = Read_IIC_Time.IIC_Read_Minute;
  
  g_SysTime.BMS_SleepStatic_Time = ((uint16)Read_IIC_Time.IIC_Read_Hour<<8) | Read_IIC_Time.IIC_Read_Minute ;//�����ϵ�����Ϊ0ʱ,�˳�ʼֵΪ0,��̬�ĵ����ܿ����㾲̬ʱ��
}
/*=======================================================================
 *������:      Sleep_StaticTime
 *����:        ���㵱ϵͳΪ����ʱ��ϵͳSOC����ʱ��
 *����:        readhour��readminite:ʵʱ��ȡ��ʱ��(ʱ/��)
               current��ʵʱ���������Ĵ�С 
               currentset����̬������С����
               starttime����������ʱ�洢�ĳ�ʼʱ��
               hourset:�û��趨�Ķ೤ʱ����������
               
 *���أ�       uint8�������Ƿ���Ҫ���,1:���в��;0:����� 
 *˵����       ��ϵͳΪ����ʱ���жϵ���Ϊ0��SOC��[0~20]��[90~100]��Χ��ʱ�䳬��
               3Сʱʱ���в���ʼ��ʼ��
========================================================================*/
uint8 Sleep_StaticTime(uint8 readhour, uint8 readminite, float current, float currentset, uint16 hourset)
{
    static uint8 Time_firstflag;
    static uint16 cnt[2];
    if(abs(current) < currentset)
    { 
      
      if((++cnt[0])*PEWERONOFF_PERIO/1000 >= 10) //��10S�ڶ��ǵ���С��currentsetA��ʼ��ʱ
      {  
        cnt[1] = 0;  
        cnt[0] = (uint8)(10000/PEWERONOFF_PERIO); 
        if(Time_firstflag == 0)
        { 
           Time_firstflag = 1;    
           g_SysTime.BMS_SleepStatic_Time = ((uint16)readhour<<8) | readminite ;           
        }
        else
        {   
          //�˴�����ʼֵΪ0,��ô�ڵ���С��5A������¿��ܻ�ֱ�ӵ��´˴��ж�������ͨ��
          if((((uint16)readhour<<8) | readminite)-g_SysTime.BMS_SleepStatic_Time >= 0)
          {
            if((((uint16)readhour<<8) | readminite)-g_SysTime.BMS_SleepStatic_Time >= (hourset<<8))//����12Сʱ���
            {
               return TRUE;
            }
          }
          else
          {
            if(((uint16)24<<8)-g_SysTime.BMS_SleepStatic_Time+(((uint16)readhour<<8) | readminite) >= (hourset<<8))
            {   
               return TRUE;
            }
          }
        }
      }       
    }
    else if((++cnt[1])*PEWERONOFF_PERIO/1000 >= 2)//���������������2s����Ϊ���Ǵ��ڷǹ���״̬
    {
       cnt[0] = 0; 
       cnt[1] = (uint8)(2000/PEWERONOFF_PERIO);
       Time_firstflag = 0;
    }
    return FALSE;
}


//BMS�ϵ��ʼʱ��
/*=======================================================================
 *������:      Task_SysTimeGet(void)
 *����:        BMS���ϵ�ʱ��Ļ�ȡ
 *����:        starttime:��ȡ��ʼʱ��ı���       
 *���أ�       ��
 *˵����       
========================================================================*/
void Task_SysTimeGet(void)
{
   uint8 timestart;//����һ�ε�ʱ���¼
   uint8 timediff; //BMS���е�ʱ������
   
   DS3231_Read_Time();
   
   timestart = Read_IIC_Time.IIC_Read_Minute;
   
   if(timestart - g_SysTime.BMS_StartRun_Time >=0) 
   {
      timediff = timestart - g_SysTime.BMS_StartRun_Time;//���εļ�¼ʱ��-�ϴεļ�¼ʱ��
   }
   else
   {
      timediff = 60 - g_SysTime.BMS_StartRun_Time + timestart;
   }
   g_SysTime.BMS_StartRun_Time = Read_IIC_Time.IIC_Read_Minute;
   g_SysTime.BMS_TotalRun_MiniteTime = g_SysTime.BMS_TotalRun_MiniteTime + timediff; //ϵͳ��������ʱ��
   
   g_SysTime.BMS_PowerOff_Time = ((uint32)Read_IIC_Time.IIC_Read_Year<<24) + ((uint32)Read_IIC_Time.IIC_Read_Month<<16)\
                                  + ((uint32)Read_IIC_Time.IIC_Read_Day<<8) + ((uint32)(Read_IIC_Time.IIC_Read_Hour));
   
   g_Roll_Tick.Roll_SysTime++;

}