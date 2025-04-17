/*=======================================================================
 *Subsystem:   ���
 *File:        Task_EEEReadWrite.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _TASK_EEE_RAEDWRITE_H_
#define _TASK_EEE_RAEDWRITE_H_

  #include  "TypeDefinition.h"
  

  #define EEprom_Ptr      0x0E00  //���ݵ�ַ���λ��ʼ��ַ
  #define EEprom_Baseadrr 0x0E10  //�����洢�Ļ���ַ
  #define EEprom_Length   16      //�洢��ѯ����
  #define Elem_Num 26             //����洢������ַ�ĳ���
  
  typedef struct 
  {
    uint8   pEErom_base;
    uint8   EE_Value;       //EEPROM������ȷ���ж�
    uint16  Charge_Times;    
    
  }EEprom_Data_T;
  extern EEprom_Data_T EEprom_Data; 

  void Get_EEprom_Value(void);
  void EEprom_DateMemset(void);
  void Task_EEpromWrite(void); 

#endif
