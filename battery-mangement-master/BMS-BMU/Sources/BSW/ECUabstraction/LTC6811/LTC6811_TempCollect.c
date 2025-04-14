/*=======================================================================
 *Subsystem:   ���
 *File:        LTC6811_TempCollect.C
 *Author:      WenM
 *Description: ��Դ�ļ���Ҫ�ɼ�����¶�(����GPIO��)�Ͳɼ�6811��оƬ�¶�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    2017 - 11 - 10
      Author:  ZWB
      Modification:������SS�ĸ�ֵ��Ϊ���Ȼ���LTC6804��SPI��
========================================================================*/
#include  "includes.h"
LTC6811_TempInfo_T g_LTC6811_TempInfo; //LTC6811�¶Ȳɼ�ȫ�ֱ���(GPIO�¶ȡ�оƬ�¶�)
/*=======================================================================
 *������:      Task_Pack_Temp_Collect(void)
 *����:        ��ȡ�����Ĵ���GPIO
 *����:        ��       
 *���أ�       ��
 *˵����       �ռ�Pack�¶�
========================================================================*/
void LTC6811_TempCMDSend(void)
{
   Ltc6804_clraux();                             /* ��������Ĵ��� */ 
                                    
   LTC6804_adax();                               /* ����GPIO ADCת�� */
   
   LTC6804_wrcfg(NUM_IC, CFG1);
}

/*=======================================================================
 *������:      Task_Pack_Temp_Process(void)
 *����:        Pack�¶ȴ�����
 *����:        ��       
 *���أ�       ��
 *˵����       ��Pack�¶Ƚ��д���
========================================================================*/
void LTC6811_TempCollect(void)        
{
   uint8  j,k;
   uint16 cell_temp[NUM_IC][6];    // ��ȡ�����Ĵ����е�Pack�¶�ֵ 
   uint16 Cell_temp[NUM_IC][6];      // �¶ȼĴ���
   uint8  PEC_error_t[NUM_IC];
   uint8  maxtemp,mintemp;           // maxtemp=0x8000,mintemp=0X7FFF 32767 = (15��1);
   int16  Temperature[NUM_IC][5];    // PACK�¶� 
   
   for(k = 0; k < NUM_IC; k++)
   {
      PEC_error_t[k] = 0;            // PECλ����
   }                      
   for(k = 0;k < NUM_IC;k++)
   {
      for(j = 0;j < 6;j++)
      {
         Cell_temp[k][j] = 0;
      }
   }

   LTC6811_Wakeup();//����
       
   LTC6804_rdaux(0, NUM_IC, cell_temp, PEC_error_t);
          
   for(k = 0; k < NUM_IC; k++)
   {
       if(!PEC_error_t[k])
       {
           for(j = 0; j < 6; j++)
           {
               if(cell_temp[k][j]<29152 && cell_temp[k][j]>1098) 
               Cell_temp[k][j] = cell_temp[k][j];
           }
       } 
   } 
  //��1��6811�ɼ����¶�
  Temperature[0][0] = HXYA_Gpio_Search(Cell_temp[0][1]);
  Temperature[0][1] = HXYA_Gpio_Search(Cell_temp[0][2]);
  //��2��6811�ɼ����¶�
  Temperature[1][0] = HXYA_Gpio_Search(Cell_temp[1][1]);
  Temperature[1][1] = HXYA_Gpio_Search(Cell_temp[1][2]);
  Temperature[1][2] = HXYA_Gpio_Search(Cell_temp[1][3]);
  //ȫ�ֱ�����ֵ
  g_LTC6811_TempInfo.CellTemp[0] = Temperature[0][0]+40; //ƫ����Ϊ40
  g_LTC6811_TempInfo.CellTemp[1] = Temperature[0][1]+40; //ƫ����Ϊ40
  g_LTC6811_TempInfo.CellTemp[2] = Temperature[1][0]+40; //ƫ����Ϊ40
  g_LTC6811_TempInfo.CellTemp[3] = Temperature[1][1]+40; //ƫ����Ϊ40
  g_LTC6811_TempInfo.CellTemp[4] = Temperature[1][2]+40; //ƫ����Ϊ40
  
  g_LTC6811_TempInfo.CellTemp_Tatoltemp = 0;
  
  for(k = 0; k < NUM_Tem ; k++)
  {  
    g_LTC6811_TempInfo.CellTemp_Tatoltemp = g_LTC6811_TempInfo.CellTemp_Tatoltemp + g_LTC6811_TempInfo.CellTemp[k];
  }
  g_LTC6811_TempInfo.CellTemp_Ave = g_LTC6811_TempInfo.CellTemp_Tatoltemp/NUM_Tem;
  
  maxtemp=mintemp=g_LTC6811_TempInfo.CellTemp[0];
  g_LTC6811_TempInfo.CellTemp_MaxNode = g_LTC6811_TempInfo.CellTemp_MinNode=0;
  
  for(k = 0; k <NUM_Tem; k++) 
  {
    if(maxtemp < g_LTC6811_TempInfo.CellTemp[k]) 
    {
      maxtemp=g_LTC6811_TempInfo.CellTemp[k];
      g_LTC6811_TempInfo.CellTemp_MaxNode=k;
    }
    if(mintemp > g_LTC6811_TempInfo.CellTemp[k]) 
    {
      mintemp=g_LTC6811_TempInfo.CellTemp[k];
      g_LTC6811_TempInfo.CellTemp_MinNode=k; 
    }
  }  
  g_LTC6811_TempInfo.CellTemp_Max=maxtemp;
  g_LTC6811_TempInfo.CellTemp_Min=mintemp;
}
/*=======================================================================
 *������:      Task_Chip_Temp_Collect(void)
 *����:        ��оƬ�¶Ȳɼ�
 *����:        ��       
 *���أ�       ��
 *˵����       ��״̬�Ĵ����ɼ�
========================================================================*/
void LTC6811_ChipTempCMDSend(void)
{                                                             
    Ltc6804_Clrstat();                                               /* �Ը����Ĵ�������; */
    
    LTC6804_adstat();                                                /* ������״̬�Ĵ����������ռ�;*/ 
}    

/*=======================================================================
 *������:      Task_Chip_Temp_Process(void)
 *����:        ��оƬ�¶ȴ���
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
void LTC6811_ChipTempCollect(void)                         
{ 
   uint8  i=0;
   uint16 cell_state[NUM_IC][6];
   static uint16 voltage_temp[NUM_IC];
   float  Temp_IC[NUM_IC];                                       
   uint8  PEC_error_s[NUM_IC];
   uint8  maxtemp, mintemp;
    
    for(i = 0;i < NUM_IC;i++)
    {                                                                                                                                                                                                                                                                                          
        PEC_error_s[i]=0;                
    } 
    
    //SS2 = 0;
    //SS2 = 1;
    LTC6811_Wakeup();//����
        
    LTC6804_rdstat(0, NUM_IC, cell_state, PEC_error_s); // ֻ��Ҫ״̬�Ĵ�����A
    
    for(i = 0; i < NUM_IC; i++)
    {
        if( PEC_error_s[i] ==0 ) 
        {
            //if( cell_state[i][1] < 31000 ) 
            {  
                voltage_temp[i] = cell_state[i][1]; 
            }
        }
    }
    
    for(i = 0; i < NUM_IC; i++)
    {
        Temp_IC[i] = (voltage_temp[i] + 37) / 75.0 - 273.0;                   
    }  
    
    for(i = 0; i < NUM_IC; i++)
    {
       if ( Temp_IC[i] >= -40 && Temp_IC[i] <=120 ) 
       {
          g_LTC6811_TempInfo.ICTemp[i] = (uint8)(Temp_IC[i]+40);  
       }
    }  
    
    maxtemp = mintemp = g_LTC6811_TempInfo.ICTemp[0];
    for(i = 0; i< NUM_IC; i++)
    {
        if(maxtemp < g_LTC6811_TempInfo.ICTemp[i])
        {
           maxtemp = g_LTC6811_TempInfo.ICTemp[i]; 
        }
    }
    
    if( maxtemp > Temp_IC_over+40)
    {
        g_LTC6811_TempInfo.ICTemp_OverState = 0x01;
    }
    else
    {
        g_LTC6811_TempInfo.ICTemp_OverState = 0x00;
    }
}
