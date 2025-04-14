/*=======================================================================
 *Subsystem:   ���
 *File:        Task_SysTimeGet.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _STATICYIMEGET_H_
#define _STATICYIMEGET_H_  
  
  #include  "TypeDefinition.h"

  typedef struct
  {
    uint32  SOC_Static_Time;           //SOC����ʱ��,���ο���ʱ��-�ϴιر�ʱ��  
    uint32  BMS_PowerOff_Time;         //�ϵ�ǰ�Ĵ洢ʱ��
    uint16  BMS_SleepStatic_Time;      //ͳ��BMS��������״̬ǰ����ʱ��
    uint32  BMS_TotalRun_MiniteTime;   //BMS������ʱ��,��λ:����
    
    uint8  BMS_StartRun_Time;         //�ϵ�ʱ�ķ���ʱ��
    
  }SysTime_T;
  extern SysTime_T g_SysTime;

  void  Time_Init(void);
  uint8 Sleep_StaticTime(uint8, uint8, float, float, uint16);
  void Task_SysTimeGet(void);

#endif