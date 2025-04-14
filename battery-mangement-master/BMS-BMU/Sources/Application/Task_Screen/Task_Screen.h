/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Screen.h
 *Author:      WenMing
 *Description: ͨ�ţ�SCI2.
               �ӿڣ�PJ0��RXD����PJ1��TXD��
               �����ʣ�
 ========================================================================
 * History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ���
 * 1. Date:
      Author:
      Modification:
========================================================================*/


#ifndef _TASK_SCREEN_H_
#define _TASK_SCREEN_H_

#include  "TypeDefinition.h"
#include  "MC9S12XEP100.h"


/*����ͷ�ļ����궨��*/
  
  #define Array_couple              20  //�궨�巢��˫�ֽ��������Ϊ20
  #define Array_single              17  //�궨�巢�͵��ֽ��������Ϊ17

  #define RS485_Enable       PORTA_PA6
  #define RS485_EnableDir    DDRA_DDRA6


  /*======��ʾ����ʾ����˫�ֽ�======*/
  typedef union
  { 
    uint8 SCI_Content2[Array_couple];
    struct
    {       
        uint16 RunningTime;           //����ʱ��
        uint16 BMS_Current;           //��������
      	uint16 BMS_SOC;               //SOC
      	uint16 BMS_SOH;               //SOH
      	uint16 Pack_Hightemp;         //���������¶�
      	uint16 Pack_Lowtemp;          //���������¶�
      	uint16 Pack_Volt;             //�������ѹ
      	uint16 Single_Maxvolt;        //������ߵ�ѹ
      	uint16 Single_Lowvolt;        //������͵�ѹ
      	uint16 iso_resistance;        //��Ե������ֵ
    }TX2;
  }RS485_couple;                  //˫�ֽ�SCI����   
  
  
  /*======��ʾ����ʾ���ݵ��ֽ�======*/
  typedef union
  {
    uint8 SCI_Content1[Array_single];
    struct
    {
      uint8	Alam_SOC;               //SOC�澯
      uint8 Alam_VoltSL;            //�ŵ���ѹ��
      uint8 Alam_VoltCL;            //�ŵ絥���
      uint8 Alam_TempH_DisChg;      //�ŵ����
      uint8 Alam_TempL_DisChg;      //�ŵ����
      uint8 Alam_CurrH_DisChg;      //�ŵ����
      uint8 Alam_VoltCD_DisChg;     //�ŵ絥��ѹ��
      uint8 Alam_TempD_DisChg;      //�ŵ��²�
        
      uint8 Alam_VoltSH;            //�����ѹ��
      uint8 Alam_VoltCH;            //��絥���
      uint8 Alam_TempH_Charge;      //������
      uint8 Alam_TempL_Charge;      //������
      uint8 Alam_CurrH_Charge;      //������
      uint8 Alam_VoltCD_Charge;     //��絥��ѹ��
      uint8 Alam_TempD_Charge;      //����²�
      
      uint8 Alam_Insul;             //��Ե����
      uint8 Alam_Checkself;         //�Լ�״̬  
      
    }TX1;     
  }RS485_single;                   //���ֽ�SCI����
  
  typedef struct
  {
    RS485_couple TxData_couple;
    RS485_single TxData_single;   
  }RS485;
      
  uint8 Init_Screen(void); 
  void Task_ScreenTransfer(void);
   
  
 #endif 

