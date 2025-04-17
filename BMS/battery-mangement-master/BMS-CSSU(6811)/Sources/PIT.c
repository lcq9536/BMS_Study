//--------------------------------------------------------------------------*
// �ļ��� : PIT.c                                                           *
// ˵  �� : ��ͷ�ļ�ΪPIT(��ʱ��)ģ���ʼ�������ܺ���ʵ���ļ�               *
//          (1)PITInit:  ��ʱ����ʼ��                                       *
//          (2)SecAdd1:��ʱ������                                           *
//--------------------------------------------------------------------------*
//ͷ�ļ����������궨���� 
//ͷ�ļ�����
#include "Includes.h"                  
 //��������ʵ��

//PITInit:��ʱ����ʼ������----------------------------------------------*
//��  ��:��ʱ����ʼ�����ж�һ��ʱ��Ϊ1/38��                             *
//��  ��:��                                                             *
//��  ��:��                                                             *
//----------------------------------------------------------------------*
  
void init_PIT0(void)
{
 
     PITMTLD0=24;     //Ϊ0ͨ��8λ��������ֵ
     PITLD0=1279;      //Ϊ0ͨ��16λ��������ֵ   //(249+1)*(1279+1)/32M=10ms
     PITMUX_PMUX0=0;   //��0ͨ��ʹ��΢������0
     PITCE_PCE0=1;     //��0ͨ������������ 
     PITCFLMT=0X80;    //ʹ��PITģ��
     PITINTE_PINTE0=1; //0ͨ����ʱ����ʱ�жϱ�ʹ��
}
  
void init_PIT1(void)
{
 
     PITMTLD1=249;     //Ϊ1ͨ��8λ��������ֵ
     PITLD1=63999;     //Ϊ1ͨ��16λ��������ֵ   //(249+1)*(63999+1)=16000000����������=0.5��
     PITMUX_PMUX1=1;   //��1ͨ��ʹ��΢������1
     PITCE_PCE1=1;     //��1ͨ������������ 
     PITCFLMT=0X80;    //ʹ��PITģ��
     PITINTE_PINTE1=0; //0ͨ����ʱ����ʱ�жϱ�ʹ��
}    
  
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
      
      if (MUXSEL == 0) 
      {
        PITMUX &= ~(1<<channel);
        PITMTLD0 = MTLD;
      }
      else 
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
      // ��ֹPITͨ��0�ж� 
      PITINTE &= ~(1<<channel);
}