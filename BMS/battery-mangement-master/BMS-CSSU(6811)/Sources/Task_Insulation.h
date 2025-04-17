/*=======================================================================
 *Subsystem:   ���
 *File:        Insulation.h
 *Author:      WenMing
 *Description: �ӿ�
               ��ѹ��⿪�أ��P               PB0                 
               ��ԵV+���أ�                   PB1
               ��ԵV-���أ�                   PB2
               ��Ե��ѹ�����⿪������       PB3
               ��Ե��ѹ�����⿪������       PB4
               ��ѹ���                       PAD00
               V+��ѹ���:                    PAD01
               V-��ѹ���:                    PAD08
               ����Ƶ��:                      2MHz
 ========================================================================
 * History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ���
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _Insulation_H
#define _Insulation_H 

   //#define HighVoltSV       PORTB_PB0     //��ѹ��⿪��
   //#define HighVoltSV_Dir   DDRB_DDRB0
   
   #define HighVoltS1       PORTB_PB3     //V+����
   #define HighVoltS1_Dir   DDRB_DDRB3
   
   #define HighVoltS2       PORTB_PB2     //V-����
   #define HighVoltS2_Dir   DDRB_DDRB2
   
   //#define HighVoltLS1      PORTB_PB4     //��ѹ�����⿪����
   //#define HighVoltLS1_Dir  DDRB_DDRB4
   
   //#define HighVoltLS2      PORTB_PB1     //��ѹ�����⿪�ظ�
   //#define HighVoltLS2_Dir  DDRB_DDRB1
   
   #define INS_SwitchON         1
   #define INS_SwitchOFF        0             //��ԵMOS�ܿ���
   
   #define DetectCount      12            //������ѹ12��
   
   #define LowVoltageDec    5            //��ѹʱ�ľ�Ե���ֵ��δ����
   #define Bias_Resitance   100            //ƫ�õ�����ֵ
   
   #define Resistance_Alarm1  100         //��Ե������ϵȼ���ֵ��δ����
   #define Resistance_Alarm2  50          //��Ե������ϵȼ���ֵ ��δ���� 
     
   #define HVPositiveChannel  6          //��Ե�������AD��
   #define HVNegtiveChannel    13          //��Ե��⸺��AD��
   #define Stand_Volt          4.5               //�궨���׼��ѹ
   typedef struct
   {
     uint8  insulation_grade;    //��Ե���ϵȼ�
     uint8  insulation_curr;     //�Ƿ�©��0��1
     uint16 insulation_resist;   //��Ե��ֵ
     float insulation_Vposit;    //������ѹ
     float insulation_Vnegt;     //�Ը���ѹ
     uint32 insulation_TotalVolt; //��Ե�����ѹ
     uint8 insulation_Disable;   //ʹ�ܱ��(���ǰ�ڲ���ʹ��)
     
     uint16 insulation_resist_P;  //��������
     uint16 insulation_resist_N;  //��������
   }IsoResist;
   extern IsoResist IsoDetect;

   void VoltageDetectInit(void);
   void InsulationDetect(void);


#endif