 /*=======================================================================
 *Subsystem:   ���
 *File:        Task_SOCSOH.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef  _TASK_SOCSOH_H_
#define  _TASK_SOCSOH_H_
  
  #include  "TypeDefinition.h"
 

  #define SOC_PERIOD      100          /* SOC�ļ���������100ms */
 
  #define HALLCHANNEL     7           //�궨�����������ͨ��
  #define HALL_RANGE      750         //����������������
  
  //SOC��Ϣ�ṹ��
  typedef  struct
  {
    uint16  SOC_Init;                   //SOC��ʼ��
    float   SOC_LowestVoltGet;          //��͵�ѹ��Ӧ��SOCֵ��0-1��
    float   SOC_HighestVoltGet;         //��ߵ�ѹ��Ӧ��SOCֵ��0-1��  
    uint16  SOC_CalTime;                //SOC��ʱ 1s����һ��
    float   SOC_ValueRead;              //SOC��ȡֵ����λΪ1��0-1,��ȡ��ֵ
    float   SOC_ValueVoltGet;           //SOC����ֵ����λΪ1��0-1
    float   SOC_ValueInitDiff;         //SOC��ֵ����ȡֵ-����ֵ
    float   SOC_ValueRealtimeDiff;     //SOCʵʱ���𽥵ݼ�
    uint8   SOC_CheckTable_Flag;        //����״̬�£����Ƿ���в����б��
    
  }SOCInfo_T;
  extern    SOCInfo_T    g_SOCInfo;
  
  
  typedef  struct
  {
    float   Energy_Once_DisCharge;           //���ηŵ���
    float   Energy_Total_Charge;             //�ܳ���� 
    float   Energy_Total_DisCharge;          //�ܷŵ���
     
  }EnergyInfo_T;
  extern    EnergyInfo_T    g_EnergyInfo;

  void Task_SOCSOH(void);
  
#endif