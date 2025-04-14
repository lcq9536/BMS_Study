/*=======================================================================
 *Subsystem:   ���
 *File:        PIT.C
 *Author:      WENMING
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "PIT.h"
#include  "Task_Init.h"
               
 //��������ʵ��

 PIT_TimePeriod_T PIT_TimePeriod;  
/*=======================================================================
 *������:      PITInit(uint8 channel,uint8 MUXSEL,uint8 MTLD,uint16 LD)
 *����:        ��ʱ����ʼ������
 *����:        ��       
 *���أ�       ��
 
 *˵����       
========================================================================*/

uint8 PIT0_Init(void)
{
   PITMTLD0=249;       //Ϊ0ͨ��8λ��������ֵ
   PITLD0 = 1279;      //Ϊ0ͨ��16λ��������ֵ   //(249+1)*(1279+1)/32M=10ms
   PITMUX_PMUX0=0;     //��0ͨ��ʹ��΢������0
   PITCE_PCE0=1;       //��0ͨ������������ 
   PITCFLMT=0x80;      //ʹ��PITģ��
   PITINTE_PINTE0 = 1; //0ͨ����ʱ����ʱ�жϱ�ʹ��
   return 0;
}
/*   
void PIT1_Init(void)
{
 
     PITMTLD1=249;     //Ϊ1ͨ��8λ��������ֵ
     PITLD1=63999;     //Ϊ1ͨ��16λ��������ֵ   //(249+1)*(63999+1)=16000000����������=0.5��
     PITMUX_PMUX1=1;   //��1ͨ��ʹ��΢������1
     PITCE_PCE1=1;     //��1ͨ������������ 
     PITCFLMT=0X80;    //ʹ��PITģ��
     PITINTE_PINTE1=1; //0ͨ����ʱ����ʱ�жϱ�ʹ��
}    
*/ 
//PIT����ʹ��
void PITInit(uint8 channel,uint8 MUXSEL,uint8 MTLD,uint16 LD)
{     
  if (channel >= 7)
  {
      channel = 7;
  }
  //��ֹPITģ��
  PITCFLMT_PITE = 0;
  // ʹ��PITͨ��channel
  PITCE |= 1<<channel;
  //��channelͨ��ʹ��΢������MUXSEL,��Ҫ���ֵ�����Ӧ��΢��ʱ���ؼĴ�����
  if (MUXSEL == 0)//��0ͨ�� 
  {
    PITMUX &= ~(1<<channel);
    PITMTLD0 = MTLD;
  }
  else//��1ͨ�� 
  {
    PITMUX |= 1<<channel;
    PITMTLD1 = MTLD;
  }
  // ��ʱ��һ���ж�ʱ�� = (PITMTLD + 1) * (PITLD + 1) / fBUS
  //                    =(249+1)*(63999+1)/32MHz=0.5s
  switch (channel)  
  {
    case 0:                                               
      PITLD0=LD;
      break;
    case 1:
      PITLD1=LD;
      break;
    case 2:
      PITLD2=LD;
      break;
    case 3:
      PITLD3=LD;
      break;
    case 4:
      PITLD4=LD;
      break;
    case 5:
      PITLD5=LD;
      break;
    case 6:
      PITLD6=LD;
      break;
    case 7:
      PITLD7=LD;
      break;
    default:
      break;    
  }
  // ʹ��PITģ�� 
  PITCFLMT_PITE = 1;
  // ��ͨ��0�����־,�����µļ�ʱʱ��
  PITTF|=1<<channel;
  // PIT�ж�ʹ��
  PITINTE |= (1<<channel);       
}


//extern void Task_Roll();
#pragma CODE_SEG __NEAR_SEG NON_BANKED
//��ʱ�жϣ��ж�����Ϊ10ms
interrupt void Interrupt_PIT0()//PIT0��ʱ�жϺ���:10ms����
{  
   if (PITTF_PTF0 ==1) 
   {   
     PITTF_PTF0 = 1;
   }
   
   Task_Roll();      
   PIT_TimePeriod.T500ms++;               
   if(PIT_TimePeriod.T500ms > 50)
   {
      PIT_TimePeriod.T500ms = 0;
   } 
}
#pragma CODE_SEG DEFAULT 


