/*=======================================================================
 *Subsystem:   ���
 *File:        LTC6811_OpWire.C
 *Author:      WenM
 *Description: ����6811����ⲿ�ĵ��߿�·���
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "includes.h"

LTC6811_OpwireInfo_T g_LTC6811_OpwireInfo;
/*=======================================================================
 *������:      LTC6811_OpenwireDetect(void)
 *����:        ���߿�·�ļ��
 *����:        ��       
 *���أ�       ��
 *˵����       ͨ�����������裬ѹ�����4000���߿�·
========================================================================*/
void LTC6811_OpenwireDetect()
{   
  uint16 Cell_vol_down[NUM_IC][12];
  uint16 Cell_vol_up[NUM_IC][12];
  int16  Cell_vol_dif[NUM_IC][12];
  uint16 Open_wire[NUM_IC][13];             // ���߿�·״̬(13������)
  
  uint8  *PEC_error_a = ((void *) 0);
  uint8  i,j;
  uint8  temp=0;

  set_adc(MD_NORMAL,1,CELL_CH_ALL,CELL_CHST_ALL,pup_up, chg);     // ת��PUP=1ģʽ����

  for(j = 0; j <250;j++)
  {                                                                         
      LTC6804_adow();                                                        // ����ADOW����
  }
  LTC6811_Wakeup();
  
  LTC6804_rdcv(0, NUM_IC, Cell_vol_up, PEC_error_a);                          // ��ȡAD��ѹֵӦ���õ�ѹ�ɼ�����
                                                              
  set_adc(MD_NORMAL,1,CELL_CH_ALL,CELL_CHST_ALL,pup_down, chg);   // ת��PUP=0ģʽ����

  for(j = 0;j <250;j++)
  {
      LTC6804_adow();                                                        // ����ADOW����
  }
  
  LTC6804_rdcv(0,NUM_IC, Cell_vol_down, PEC_error_a);                         // ��ȡAD��ѹֵ
  
  //��1�����߿�·���ж�
  for(j = 0; j < NUM_IC; j++)
  {
      //Openwire_flag[j]=0;
      g_LTC6811_OpwireInfo.OpenwireLocation[j] = 0;
      for(i = 0; i < 13; i++) 
      {  
        Open_wire[j][i] = 0;
      }
  }
   
  //��2~11�����߿�·���ж�
  for(j = 0;j <NUM_IC;j++)                
  {
    switch(j)
    {
      case 0:
        for(i = 1; i < 12; i++)
        { 
          Cell_vol_dif[j][i] = Cell_vol_down[j][i] - Cell_vol_up[j][i];
          if(Cell_vol_dif[j][i] > 4000)
          Open_wire[j][i] = 1;                          // C1-11��·
        }
       if(Cell_vol_up[j][0] == 0)    
         Open_wire[j][0] = 1;                                   // C0��·
       if(Cell_vol_down[j][NUM1_Batper -1] == 0)                 
         Open_wire[j][NUM1_Batper ] = 1;                           // C12��·
       break;
     
      case 1:
        for(i = 1;i < 12 ;i++)
        { 
          Cell_vol_dif[j][i] = Cell_vol_down[j][i] - Cell_vol_up[j][i];
          if(Cell_vol_dif[j][i] > 4000)
          Open_wire[j][i] = 1;                          // C1-11��·
        }
       if(Cell_vol_up[j][0] == 0)    
         Open_wire[j][0] = 1;                                   // C0��·
       if(Cell_vol_down[j][NUM2_Batper -1] == 0)                 
         Open_wire[j][NUM2_Batper ] = 1;                           // C12��·
       break;
       
      case 2:
        for(i = 1;i < 12 ;i++)
        { 
          Cell_vol_dif[j][i] = Cell_vol_down[j][i] - Cell_vol_up[j][i];
          if(Cell_vol_dif[j][i] > 4000)
          Open_wire[j][i] = 1;                          // C1-11��·
        }
       if(Cell_vol_up[j][0] == 0)    
         Open_wire[j][0] = 1;                                   // C0��·
       if(Cell_vol_down[j][NUM3_Batper -1] == 0)                 
         Open_wire[j][NUM3_Batper ] = 1;                           // C12��·
       break;
       
      case 3:
        for(i = 1;i < 12 ;i++)
        { 
          Cell_vol_dif[j][i] = Cell_vol_down[j][i] - Cell_vol_up[j][i];
          if(Cell_vol_dif[j][i] > 4000)
          Open_wire[j][i] = 1;                          // C1-11��·
        }
       if(Cell_vol_up[j][0] == 0)    
         Open_wire[j][0] = 1;                                   // C0��·
       if(Cell_vol_down[j][NUM4_Batper -1] == 0)                 
         Open_wire[j][NUM4_Batper ] = 1;                           // C12��·
       break;
        
     case 4:
        for(i = 1;i < 12 ;i++)
        { 
          Cell_vol_dif[j][i] = Cell_vol_down[j][i] - Cell_vol_up[j][i];
          if(Cell_vol_dif[j][i] > 4000)
          Open_wire[j][i] = 1;                          // C1-11��·
        }
       if(Cell_vol_up[j][0] == 0)    
         Open_wire[j][0] = 1;                                   // C0��·
       if(Cell_vol_down[j][NUM5_Batper -1] == 0)                 
         Open_wire[j][NUM5_Batper ] = 1;                           // C12��·
       break;
        
       default:
           break;
     }
  }
  //��13�����߿�·���ж�
  for(j = 0;j < NUM_IC;j++)
  {
    for(i = 0;i <13;i++) 
    {
      //Openwire_flag[j]=Openwire_flag[j]+(Open_wire[j][i]<<i);
      g_LTC6811_OpwireInfo.OpenwireLocation[j] = g_LTC6811_OpwireInfo.OpenwireLocation[j]+(Open_wire[j][i]<<i);
    }
  } 
  for(j = 0;j < NUM_IC;j++)
  {
   if(g_LTC6811_OpwireInfo.OpenwireLocation[j] != 0)
    {
      //g_TempInfo.Openwire_error=1;
      g_LTC6811_OpwireInfo.OpenwireErr = 1;
    }
  }
}