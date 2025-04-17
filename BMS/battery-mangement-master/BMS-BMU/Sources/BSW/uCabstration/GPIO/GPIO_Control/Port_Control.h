/*=======================================================================
 *Subsystem:   ���
 *File:        Port_Control.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _PORTCONTROL_H_
#define _PORTCONTROL_H_  

  #include  "TypeDefinition.h"
  #include  "MC9S12XEP100.h"
  #include  "derivative.h"
  #include  "GPIO.h"

    //1.��״̬�궨��(����ʵ�ʵ�·����) 
    #define Light_ON        0    //����(��Ӧ�͵�ƽ)
    #define Light_OFF       1    //�ư�(��Ӧ�ߵ�ƽ)
    
    //2.�ƿ������ź궨��
    #define LED1_PORT      PT    //���⾯ʾ��
    #define LED1_pin        0    //
    
    #define LED2_PORT      PT    //�Լ�ɹ���
    #define LED2_pin        1    // 
    
    #define LED3_PORT      PB    //����
    #define LED3_pin        4    //   
      
//��������������
uint8 Light_Init(uint16 port,uint8 name,uint8 state);   
void  Light_Control(uint16 port,uint8 name,uint8 state);
void  Light_Change(uint16 port,uint8 name);
uint8 Light_StateGet(uint16 port,uint8 pin);
//****************************�̵���***************************************//
uint8 Port_Init(uint16 port,uint8 name,uint8 state);
void  Port_Control(uint16 port,uint8 name,uint8 state);
uint8 Port_StateGet(uint16 port,uint8 pin);
   
//===========================================================================

#endif 