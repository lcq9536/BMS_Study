/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Temp_CalProces.C
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
               ���ԣ��۲�pack�¶ȣ���Send_Data_VCU�����۲������С��ѹ��ֵ��
                     ��GPS���͵ı���ֵsingle_temperature�۲������¶�ֵ;оƬ
                     �¶��Ȳ��ùܣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    2017 - 11 - 10
      Author:  ZWB
      Modification:������SS�ĸ�ֵ��Ϊ���Ȼ���LTC6804��SPI��
========================================================================*/
#include  "Includes.h"                          

TempInfo_T TempInfo;


/*=======================================================================
 *������:      Task_Pack_Temp_Collect(void)
 *����:        ��ȡ�����Ĵ���GPIO
 *����:        ��       
 *���أ�       ��
 *˵����       �ռ�Pack�¶�
========================================================================*/
void Task_Pack_Temp_Collect(void)
{
                
   Ltc6804_clraux();                             /* ��������Ĵ��� */ 
                                    
   LTC6804_adax();                               /* ����GPIO ADCת�� */
   
   LTC6804_wrcfg(NUM_IC,CFG1);
   
   Task_Flag_Counter.Counter_Pack_Collect++;    
}

/*=======================================================================
 *������:      Task_Pack_Temp_Process(void)
 *����:        Pack�¶ȴ�����
 *����:        ��       
 *���أ�       ��
 *˵����       ��Pack�¶Ƚ��д���
========================================================================*/
uint16 cell_temp[NUM_IC][6];
void Task_Pack_Temp_Process(void)        
{
   uint8 j,r,i,k,valu8,m;
   //uint16 cell_temp[NUM_IC][6];                 /* ��ȡ�����Ĵ����е�Pack�¶�ֵ */
   uint16 Cell_temp[NUM_IC][6];              // �¶ȼĴ���
   uint8  PEC_error_t[NUM_IC];
   uint8  maxtemp,mintemp;           //maxtemp=0x8000,mintemp=0X7FFF 32767 = (15��1);
   int16  Single_Temperature[2*NUM_IC]; 
   uint8  TEr;
   uint8  TEr1;
   uint8  flag_ft=0;
   uint8  flag_ft1=0;
   uint16 totaltemp;
   int16  Temperature[NUM_IC][5];            // PACK�¶� 
   
   
 
   
   
   for(i = 0;i < NUM_IC;i++)
   {
      PEC_error_t[i]=0;                                             // ��PEC����������»���
   }                      
      

   for(k = 0;k < NUM_IC;k++)
   {
      for(j = 0;j < 6;j++)
      {
          Cell_temp[k][j] = 0;
      }
   
   }
   
   SS2=0;
   
   SS2=1;
  
       
   LTC6804_rdaux(0, NUM_IC, cell_temp,PEC_error_t);
   
          
   for(k = 0;k < NUM_IC;k++)
   {
       if(!PEC_error_t[k])
       {
           for(j = 0;j < 6;j++)
           {
               if(cell_temp[k][j]<29152 && cell_temp[k][j]>1098) 
               Cell_temp[k][j] = cell_temp[k][j];
           }
       } 
   } 
   
   /***************************convert_temp_pack******************************/
 /* for(i = 0;i < NUM_IC;i++)
  { 
    for(m = 0; m < Cell_GPIO; m++) 
    {
      
        Temperature[i][m] = HXYA_Gpio_Search(Cell_temp[i][m+1]);         //ȷ���ļ���GPIO��
        
    }
  }
  */
  Temperature[0][0] = HXYA_Gpio_Search(Cell_temp[0][1]);
  Temperature[0][1] = HXYA_Gpio_Search(Cell_temp[0][2]);
  
  Temperature[1][0] = HXYA_Gpio_Search(Cell_temp[1][1]);
  Temperature[1][1] = HXYA_Gpio_Search(Cell_temp[1][2]);
  Temperature[1][2] = HXYA_Gpio_Search(Cell_temp[1][3]);
  
          
       
  TempInfo.CellTemp[0] =   Temperature[0][0] + 40;
  TempInfo.CellTemp[1] =   Temperature[0][1] + 40;
  TempInfo.CellTemp[2] =   Temperature[1][0] + 40;
  TempInfo.CellTemp[3] =   Temperature[1][1] + 40;
  TempInfo.CellTemp[4] =   Temperature[1][2] + 40;
  
  TempInfo.CellTemp_tatoltemp = 0;
  
  for(i = 0; i< 5 ; i++)
  {  
      TempInfo.CellTemp_tatoltemp = TempInfo.CellTemp_tatoltemp + TempInfo.CellTemp[i];
  }
  /*totaltemp=0;
    if(flag_ft==0)
   { 
     flag_ft1++;
     if(flag_ft1>100) 
     {
      flag_ft=1;
     } 
     for(i = 0; i < (NUM_IC-1); i++)
    {
        for(j = 0; j < Cell_GPIO; j++)
        {
           TempInfo.CellTemp[i*2 + j] = Temperature[i][j];
           //Single_Temperature[i*2 + j]=Temperature[i][j];
           //TempInfo.CellTemp[i*2 + j]=( uint8)((Single_Temperature[i*2+j]+50)/100 +40);   // ��Ϊ�˴���50�Ľ�һλ,��ʵ40��0��
           TempInfo.CellTemp_tatoltemp=TempInfo.CellTemp_tatoltemp+ TempInfo.CellTemp[i*2 + j];
           TempInfo.CellTemp_Ave = ( uint8)(TempInfo.CellTemp_tatoltemp/NUM_Tem);
        } 
    } 
    
   } 
   else
   {
    
    for(i = 0; i < (NUM_IC-1); i++)
    {
        for(j = 0; j < Cell_GPIO; j++)
        {
           TempInfo.CellTemp[i*2 + j] = Temperature[i][j];
           //Single_Temperature[i*2 + j]=Temperature[i][j];
           TEr=( uint8)((TempInfo.CellTemp[i*2+j]+50)/100 +40);   // ��Ϊ�˴���50�Ľ�һλ,��ʵ40��0��                 
           if(TEr>TempInfo.CellTemp[i*2 + j]) 
           {
            TEr1=TEr-TempInfo.CellTemp[i*2 + j];
           } 
           else
           {
            TEr1=TempInfo.CellTemp[i*2 + j]-TEr;
           }
           if(TEr1<=3) 
           {
           TempInfo.CellTemp[i*2 + j]=TEr;
           }
           
           TempInfo.CellTemp_tatoltemp=TempInfo.CellTemp_tatoltemp+ TempInfo.CellTemp[i*2 + j];
           TempInfo.CellTemp_Ave = ( uint8)(TempInfo.CellTemp_tatoltemp/NUM_Tem);
        } 
    } 
   } 
   */      
          maxtemp=mintemp=TempInfo.CellTemp[0];
          TempInfo.CellTemp_MaxNode=TempInfo.CellTemp_MinNode=0;
         for(k = 0; k <NUM_Tem; k++) 
         {
          
           if(maxtemp<TempInfo.CellTemp[k]) 
           {
             maxtemp=TempInfo.CellTemp[k];
             TempInfo.CellTemp_MaxNode=k;
           }
           if(mintemp>TempInfo.CellTemp[k]) 
           {
             mintemp=TempInfo.CellTemp[k];
             TempInfo.CellTemp_MinNode=k; 
           }
        } 
       
       TempInfo.CellTemp_Max=maxtemp;
       TempInfo.CellTemp_Min=mintemp;
         
  
     
    
   Task_Flag_Counter.Counter_Pack_Process++;  
      
}
/*=======================================================================
 *������:      Task_Chip_Temp_Collect(void)
 *����:        ��оƬ�¶Ȳɼ�
 *����:        ��       
 *���أ�       ��
 *˵����       ��״̬�Ĵ����ɼ�
========================================================================*/
void Task_Chip_Temp_Collect(void)
{                                                             
    Ltc6804_Clrstat();                                               /* �Ը����Ĵ�������; */
    
    LTC6804_adstat();                                                /* ������״̬�Ĵ����������ռ�;*/ 
    
    Task_Flag_Counter.Counter_Chip_Collect++;                      
}    

/*=======================================================================
 *������:      Task_Chip_Temp_Process(void)
 *����:        ��оƬ�¶ȴ���
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/

void Task_Chip_Temp_Process(void)                         
{ 
   uint8 i,j,k=0;
   uint16 cell_state[NUM_IC][6];
   static int16 voltage_temp[NUM_IC];
   int16  Temp_IC[NUM_IC];                                       
   uint8  PEC_error_s[NUM_IC];
   uint8  maxtemp, mintemp;
    
    for(i = 0;i < NUM_IC;i++)
    {                                                                                                                                                                                                                                                                                          
        PEC_error_s[i]=0;                
    } 
    
    SS2 = 0;
    SS2 = 1;
        
    LTC6804_rdstat(0,NUM_IC,cell_state,PEC_error_s);     /* ֻ��Ҫ״̬�Ĵ�����A��*/
      
    
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
        Temp_IC[i] = ( voltage_temp[i] + 37 ) / 75 - 273;                   
    }  
    
    for(i = 0; i < NUM_IC; i++)
    {
       
       if ( Temp_IC[i] >= -40 && Temp_IC[i] <=120 ) 
       {
          TempInfo.CellTemp_ic[i] = Temp_IC[i];  
       }
    }  
    
    maxtemp = mintemp = TempInfo.CellTemp_ic[0];
    for(k = 0; k< NUM_IC; k++)
    {
        if(maxtemp < TempInfo.CellTemp_ic[k])
        {
           maxtemp = TempInfo.CellTemp_ic[k]; 
        }
    }
    
    if( maxtemp > Temp_IC_over)
    {
        TempInfo.CellTemp_over_ic = 0x01;
    }
    
    Task_Flag_Counter.Counter_Chip_Process++; 
    
}
