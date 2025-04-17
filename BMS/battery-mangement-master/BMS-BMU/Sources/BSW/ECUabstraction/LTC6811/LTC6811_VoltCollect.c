/*=======================================================================
 *Subsystem:   ���
 *File:        LTC6811_VoltCollect.C
 *Author:      WenM
 *Description: ��Դ�ļ���Ҫ���е�ص�ѹ�ɼ��Լ����ɼ������ݽ��д���
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    2017 - 11 -10            
      Author:  ZWB
      Modification:��Task_Volt_Process������������������SS�ĸ�ֵ��Ϊ�˻���SPI��
                   ����ᵼ�¶����ĵ�һ���Ĵ����ĵ�ѹֵ��65535�� 
 
========================================================================*/
#include  "includes.h"

LTC6811_VoltInfo_T g_LTC6811_VoltInfo;
/*=======================================================================
 *������:      Task_Volt_Collect(void)
 *����:        ��ѹ�ɼ�����                 
 *����:        ��       
 *���أ�       ��
 *˵����       ��LTC6804���ĸ���ѹ�Ĵ����顣
========================================================================*/
void LTC6811_VoltCMDSend(void)
{                      
  Ltc6804_Clrcell();                           /* 6804��ѹ�Ĵ�����0 */
                                                                   
  LTC6804_adcv();                              /* ����ADת�����ȴ�2.4ms�ռ���ѹֵ */
}
                                  
/*=======================================================================
 *������:      Task_Volt_Collect(void)
 *����:        ��ѹ������
 *����:        ��       
 *���أ�       ��
 *˵����       ��LTC6804���ĸ���ѹ�Ĵ����顣
========================================================================*/
void LTC6811_VoltCollect(void) 
{
  uint8  i,j;   
  uint16 cell_vol[NUM_IC][12]; 
  uint16 cell_vvol[NUM_IC][12];
  uint16 Cell_vol[NUM_IC][12];
  uint16 maxvol=0x0000,minvol=0xffff;
  uint16 pecv_flag=0;
  uint16 PEC_error_v[NUM_IC*4];
  uint8  pecv_error_ce=0;          

  LTC6811_Wakeup();//����

  for(i = 0;i < NUM_IC*4; i++)
  {
     PEC_error_v[i]=0;                              //��PEC����������»���
  }                      
                        
  LTC6804_rdcv(0, NUM_IC, cell_vol, PEC_error_v);   //��ȡAD��ѹֵ   cell_vol����ֱ����Ϊ���յĵ�ѹ
  
  for(i = 0; i < NUM_IC*4; i++)
  {
     pecv_flag= pecv_flag+(PEC_error_v[i]<<i);
  } 
  if(!pecv_flag) 
  {
    pecv_error_ce = 0x00;
    for(i=0; i<NUM_IC; i++) 
    {
      for(j=0; j<12; j++)
      {  
        if(cell_vol[i][j]!=65535)
        {
           Cell_vol[i][j]=cell_vol[i][j]; 
        }
      }
    }
  }
  
  //�Ӱ����ݴ���(���Խ����ͬ�Ľ��߷�ʽ)
  for(j=0; j<NUM_IC; j++)
  {
    switch(j)
    {
      case 0:
        for(i = 0; i < NUM1_Batper_front; i++)
        {
           cell_vvol[0][i] = Cell_vol[0][i];            
        }
        for(i = 0; i < NUM1_Batper_rear; i++)
        {
           cell_vvol[0][i+NUM1_Batper_front] = Cell_vol[0][i+NUM1_Batper_front+NUM1_Batper_empty];            
        }
        break;
        
      case 1:
        for(i = 0; i < NUM2_Batper_front; i++)
        {
           // if(Cell_vol[1][i] > 20000 && Cell_vol[1][i] < 38000)
           cell_vvol[1][i] = Cell_vol[1][i];            
        }
        for(i = 0; i < NUM2_Batper_rear; i++)
        {
           // if(Cell_vol[1][i+NUM2_Batper_front+NUM2_Batper_empty] > 20000 && Cell_vol[1][i+NUM2_Batper_front+NUM2_Batper_empty] < 38000)
           cell_vvol[1][i+NUM2_Batper_front] = Cell_vol[1][i+NUM2_Batper_front+NUM2_Batper_empty];            
        }
        break;
        
       case 2:
         for(i = 0; i < NUM3_Batper_front; i++)
         {
           // if(Cell_vol[2][i] > 20000 && Cell_vol[2][i] < 38000)
            cell_vvol[2][i] = Cell_vol[2][i];            
         }
         for(i = 0; i < NUM3_Batper_rear; i++)
         {
           // if(Cell_vol[2][i+NUM3_Batper_front+NUM3_Batper_empty] > 20000 && Cell_vol[2][i+NUM3_Batper_front+NUM3_Batper_empty] < 38000)
            cell_vvol[2][i+NUM3_Batper_front] = Cell_vol[2][i+NUM3_Batper_front+NUM3_Batper_empty];            
         }
         break;
        
       case 3:
         for(i = 0; i < NUM4_Batper_front; i++)
         {
           // if(Cell_vol[3][i] > 20000 && Cell_vol[3][i] < 38000)
            cell_vvol[3][i] = Cell_vol[3][i];            
         }
         for(i = 0; i < NUM4_Batper_rear; i++)
         {
           // if(Cell_vol[3][i+NUM4_Batper_front+NUM4_Batper_empty] > 20000 && Cell_vol[3][i+NUM4_Batper_front+NUM4_Batper_empty] < 38000)
            cell_vvol[3][i+NUM4_Batper_front] = Cell_vol[3][i+NUM4_Batper_front+NUM4_Batper_empty];            
         }
         break;
        
       case 4:
         for(i = 0; i < NUM5_Batper_front; i++)
         {
           // if(Cell_vol[4][i] > 20000 && Cell_vol[4][i] < 38000)
            cell_vvol[4][i] = Cell_vol[4][i];            
         }
         for(i = 0; i < NUM5_Batper_rear; i++)
         {
            //if(Cell_vol[4][i+NUM5_Batper_front+NUM5_Batper_empty] > 20000 && Cell_vol[4][i+NUM5_Batper_front+NUM5_Batper_empty] < 38000)
            cell_vvol[4][i+NUM5_Batper_front] = Cell_vol[4][i+NUM5_Batper_front+NUM5_Batper_empty];            
         }
         break;
        
       default:
         break;  
    }  
  }
     
  // ÿ���Ӱ��ѹȫ�ֱ����ĸ�ֵ 
  g_LTC6811_VoltInfo.CellVolt_Total = 0; 
  for(j=0; j<NUM_IC; j++)
  {
     switch(j)
     {
        case 0:
          for(i=0; i<NUM1_Batper_true; i++)
          {
            g_LTC6811_VoltInfo.CellVolt[i] = cell_vvol[0][i];
            g_LTC6811_VoltInfo.CellVolt_Total = g_LTC6811_VoltInfo.CellVolt_Total + cell_vvol[0][i];                      // �ܵ�ѹ
          }
          break;
          
        case 1:
          for(i=0; i<NUM2_Batper_true; i++)
          {
            g_LTC6811_VoltInfo.CellVolt[i+NUM1_Batper_true] = cell_vvol[1][i];
            g_LTC6811_VoltInfo.CellVolt_Total = g_LTC6811_VoltInfo.CellVolt_Total + cell_vvol[1][i];                      // �ܵ�ѹ
          }
          break;
          
        case 2:
          for(i=0; i<NUM3_Batper_true; i++)
          {
            g_LTC6811_VoltInfo.CellVolt[i+NUM1_Batper_true+NUM2_Batper_true] = cell_vvol[2][i];
            g_LTC6811_VoltInfo.CellVolt_Total = g_LTC6811_VoltInfo.CellVolt_Total + cell_vvol[2][i];                      // �ܵ�ѹ
          }
          break;
          
        case 3:
          for(i=0; i<NUM4_Batper_true; i++)
          {
            g_LTC6811_VoltInfo.CellVolt[i+NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true] = cell_vvol[3][i];
            g_LTC6811_VoltInfo.CellVolt_Total = g_LTC6811_VoltInfo.CellVolt_Total + cell_vvol[3][i];                      // �ܵ�ѹ
          }
          break;
          
        case 4:
          for(i=0; i<NUM5_Batper_true; i++)
          {
            g_LTC6811_VoltInfo.CellVolt[i+NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true+NUM4_Batper_true] = cell_vvol[4][i];
            g_LTC6811_VoltInfo.CellVolt_Total = g_LTC6811_VoltInfo.CellVolt_Total + cell_vvol[4][i];                      // �ܵ�ѹ
          }
          break;
          
        default:
          break;  
     }  
  }
          
  //�����С��ѹ����� 
  maxvol = g_LTC6811_VoltInfo.CellVolt[0];  
  minvol = g_LTC6811_VoltInfo.CellVolt[0];
  
  for(i=0; i<NUM_Battery; i++)    
  {
    if(maxvol<g_LTC6811_VoltInfo.CellVolt[i]) 
    {
      maxvol = g_LTC6811_VoltInfo.CellVolt[i];
      g_LTC6811_VoltInfo.CellVolt_MaxNode = i;
    }
          
    if (g_LTC6811_VoltInfo.CellVolt[i]<minvol)
    {
      minvol = g_LTC6811_VoltInfo.CellVolt[i];
      g_LTC6811_VoltInfo.CellVolt_MinNode = i;
    }
  }  
   
  g_LTC6811_VoltInfo.CellVolt_Max = maxvol;
  g_LTC6811_VoltInfo.CellVolt_Min = minvol;
}