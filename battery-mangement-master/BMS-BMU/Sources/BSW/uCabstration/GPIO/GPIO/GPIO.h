/*=======================================================================
 *Subsystem:   ���
 *File:        GPIO.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _GPIO_H_
#define _GPIO_H_  

  #include  "TypeDefinition.h"
  #include  "MC9S12XEP100.h"
  //#include  "derivative.h" 

  //���üĴ�����ĳһλΪ1
  #define BSET(bit,Register)     ((Register)|= (1<<(bit)))
  //���üĴ�����ĳһλΪ0  
  #define BCLR(bit,Register)     ((Register) &= ~(1<<(bit))) 
  //�õ��Ĵ�����ĳһλ״̬
  #define BGET(bit,Register)     (((Register) >> (bit)) & 1) 

  //XEP100�˿���(��Ӧ�ö˿����ݼĴ���)���ַ�Ķ�Ӧ�궨��
  #define PA      0x00000000  
  #define PB      0x00000001
  #define PE      0x00000008
  #define PK      0x00000032
  #define PT      0x0240
  #define PS      0x0248     
  #define PM      0x0250
  #define PP      0x0258
  #define PH      0x0260
  #define PJ      0x0268
  #define PAD0    0x0270
  #define PAD1    0x0271

  //�˿ڵĸ����Ĵ�����ƫ�Ƶ�ַ�Ķ�Ӧ��ϵ  ??????????????????????????????
  #define PRT  0     //���ݼĴ���
  #define PTI  1     //����Ĵ���
  #define DDR  2     //����Ĵ���
  #define RDR  3     //�͹��������Ĵ���
  #define PER  4     //��������ʹ�ܼĴ���
  #define PPS  5     //������������ѡ��Ĵ���
  #define PIE  6     //�����ж�����Ĵ���
  #define PIF  7     //�����жϱ�־�Ĵ���
  #define WOM  6     //�����߻�Ĵ���
  #define PRR  7     //���Ź���ѡ��Ĵ���     

  extern uint8 QUICKCHARGE;

  typedef enum
  {
    Init_Normal_GPIO = 0
  };

  uint8 GPIO_Init(uint16 port,uint8 pin,uint8 direction,uint8 state);  
  uint8 GPIO_Get(uint16 port,uint8 reg,uint8 pin);                  
  void  GPIO_Set(uint16 port,uint8 reg,uint8 pin,uint8 state);  
  uint8 GPPort_Get(uint16 port,uint8 reg);
  void  GPPort_Set(uint16 port,uint8 reg,uint8 setFlag,uint8 bValue);

#endif 