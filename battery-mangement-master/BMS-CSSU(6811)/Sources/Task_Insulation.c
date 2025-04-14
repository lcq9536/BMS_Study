 /*=======================================================================
 *Subsystem:   ���
 *File:        Insulation.c
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
 =============================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    
      Author:  
      Modification:
===============================================================================*/
 
 
#include"Includes.h"


  IsoResist IsoDetect;        //�����Ե���ṹ��
  
/*=======================================================================
 *������:      VoltageDetect
 *����:        ��ذ���ѹ����ʼ������ѹ��⣬��Ե��⣩
 *����:        ��         
                      
 *���أ�       ��
 *˵����       ADC ʱ��Ƶ�ʣ�2MHz
========================================================================*/
  void VoltageDetectInit(void) 
  {
     HighVoltS1_Dir = 1;
     HighVoltS1 = INS_SwitchOFF;      //��Ե���V+���عر�
     
     HighVoltS2_Dir = 1; 
     HighVoltS2 = INS_SwitchOFF;      //��Ե���V-���عر�
     
     //memset(&IsoDetect,0,sizeof(IsoResist));     
  }

/*=======================================================================
 *������:      InsulationDetect
 *����:        ��ذ���Ե���
 *����:        ��         
                      
 *���أ�       ��
 *˵����       ADC ʱ��Ƶ�ʣ�2MHz
========================================================================*/

      
  void InsulationDetect(void) 
  {
      uint8 count,i,j;
      static uint8 Time_Flag,Time_Cnt;
      uint32 SumVpositive,SumVnegtive,total_VOL,total_VOL1;
      static uint16 VoltFlag1,VoltFlag2;
      float Vpositive_1,Vnegtive_1;
      uint16 VposBuff[12],VnegBuff[12],VposBuff1[12],VnegBuff1[12],VposBuff2[12],VnegBuff2[12];
      uint16 Max_Volt,Max_Volt1,Min_Volt,Min_Volt1;
      
      VoltageDetectInit();
      
      if(Time_Flag == 0) 
      { 
        Time_Cnt = 0;           
        for(count = 0; count < 12; count++)//����12����ȥ�����ֵ�����ֵ 
        {
          VposBuff[count] = ADCValue(HVPositiveChannel);   //���Ե��̣�PAD1
          
          VnegBuff[count] = ADCValue(HVNegtiveChannel);    //���Ե��̣�PAD8
  
        }    
        
        Max_Volt = 0;                 
        Max_Volt1 = 0; 
        Min_Volt = 0xFFFF;
        Min_Volt1 = 0xFFFF;
        SumVpositive =0;
        SumVnegtive =0;
        
        for(i =0; i<12; i++)            // ����ÿһ�������е������Сֵ(12��)
        {
           if(VposBuff[i] >=Max_Volt)
           {
              Max_Volt = VposBuff[i];
           } 
           if(VposBuff[i] <= Min_Volt)
           {
              Min_Volt = VposBuff[i];
           }
           
           if(VnegBuff[i] >Max_Volt1)
           {
              Max_Volt1 = VnegBuff[i];
           } 
           if(VnegBuff[i] < Min_Volt1)
           {
              Min_Volt1 = VnegBuff[i];
           }
        }
        
        for(count = 0; count < 12; count++)      //������ֵȫ�������� //
        {
           SumVpositive += VposBuff[count];
           SumVnegtive += VnegBuff[count];
        }
                
        SumVpositive = SumVpositive - Max_Volt - Min_Volt;
        SumVnegtive = SumVnegtive - Max_Volt1 - Min_Volt1; 
                
        IsoDetect.insulation_Vposit = SumVpositive/10.0;
        IsoDetect.insulation_Vnegt = SumVnegtive/10.0;               // ȥ�������Сֵ֮����ƽ��ֵ
                
        SumVpositive =0;
        SumVnegtive =0;
      
        IsoDetect.insulation_TotalVolt = ((IsoDetect.insulation_Vposit + IsoDetect.insulation_Vnegt) * Stand_Volt)/4096.0/100.0*5100.0*10000.0;
      }
       
      
    
    Time_Cnt++;      
    //Task_Flag_Counter.Counter_Insulation++; */
  
  }