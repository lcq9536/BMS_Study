/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Cssu_Data_Collect.C
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    2017 - 11 -10
      Author:  ZWB
      Modification:�޸���LTC6804_Init���������е�DDCCֵ��ֱ��ȥ�����ֵ��
                   ԭ�����ݴ����д��󣬴��˾���ģ�顣
                        
 * 2. Date:    2017 - 11 -11              
      Author:  ZWB
      Modification:Volt_Data_Process()���Ӱ��ѹ��ֵ���⣬Total_VoltֵҪ�ȼ��������
                   ��ǿ��ת��Ϊ16λ�������������ݶ�ʧ��           
========================================================================*/

#include  "Includes.h"                           /* ��������h�ļ������Ե������к�������� */ 
                                       
uint8  CFG1[NUM_IC][6];                          // ���üĴ������� ,��ȫ�ֽϺ�
uint16 Openwire_flag[NUM_IC];                   // ���߿�·PEC���� 
/*=======================================================================
 *������:      Config_Fun();
 *����:        ��LTC6804������
 *����:        ��       
 *���أ�       ��
 *˵����       IC=3;GPIO=0XFF; REFON=1; SWTEN=0;  ADCOPT=0;  VUV=3.2; VOV=4.2; DCTO=0;
========================================================================*/
void Config_Fun( uint8 total_ic,uint8 gpio, uint8 refon,uint8 swtrd, uint8 adcopt,
                 float  vuv,float   vov,uint16 ddc,uint8 dcto) 
{           
   uint8 char1,char2,char3;
   uint8 current_ic;                             /* ��LTC6804_1�ĳ�ʼ���������ò��� */
   
   for( current_ic = 0; current_ic<total_ic; current_ic++)
   {
      
      CFG1[current_ic][0]=(gpio<<3)+(refon<<2)+(swtrd<<1)+adcopt;
      
      CFG1[current_ic][1]=(uint8)(((uint16)vuv*10000/16-1)&0x00ff);
      
      char1=(uint8)((((uint16)vov*10000/16)&0x000f)<<4);
   
      char2=(uint8)((((uint16)vuv*10000/16-1)&0x0f00)>>8);
       
      CFG1[current_ic][2]=char1+char2;
      
      CFG1[current_ic][3]= (uint8)(((uint16)(vov*10000/16)&0x0ff0)>>4);
      
      CFG1[current_ic][4]= ddc;
      
      char3 = (uint8)(ddc>>8);
      
      char3 = char3 &0x0f; 
     
      CFG1[current_ic][5]=(dcto<<4)+ char3;    
   }
}

/*=======================================================================
 *������:      LTC6804_Init(void)
 *����:        ��LTC6804�ĳ�ʼ��
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
void LTC6804_Init(void)
{     
     
    Config_Fun(NUM_IC,DGPIO ,DREFON,DSWTRD,DADCOPT,UNDER_V,OVER_V,DDCC,DDCTW) ;     /* ����ֵ�����ṹ�� */ 
                                               
    LTC6804_wrcfg(NUM_IC,CFG1);                                                     /* д����оƬ���üĴ��� */
    
    set_adc(MD_NORMAL,DCP_ENABLED,CELL_CH_ALL,CELL_CHST_ALL, pup_up, chg);          /* ת��ģʽ���� */
}  

/*=======================================================================
 *������:      circuit_detection(void)
 *����:        ���߿�·�ļ��
 *����:        ��       
 *���أ�       ��
 *˵����       ͨ�����������裬ѹ�����4000���߿�·
========================================================================*/
void circuit_detection()
{   

    uint16 Cell_vol_down[NUM_IC][12];
    uint16 Cell_vol_up[NUM_IC][12];
    int16  Cell_vol_dif[NUM_IC][12];
    //uint16 Openwire_flag[NUM_IC];                   // ���߿�·PEC����
    uint16 Open_wire[NUM_IC][13];             // ���߿�·״̬
    
    uint8 *PEC_error_a;
    uint8  i,j;
    uint8  temp=0;
 
    set_adc(MD_NORMAL,1,CELL_CH_ALL,CELL_CHST_ALL,pup_up, chg);     // ת��PUP=1ģʽ����

    for(j = 0; j <250;j++)
    {                                                                         
        LTC6804_adow();                                                        // ����ADOW����
    }
    wakeup_idle();
    
    LTC6804_rdcv(0, NUM_IC, Cell_vol_up, PEC_error_a);                          // ��ȡAD��ѹֵӦ���õ�ѹ�ɼ�����
                                                                
    set_adc(MD_NORMAL,1,CELL_CH_ALL,CELL_CHST_ALL,pup_down, chg);   // ת��PUP=0ģʽ����

    for(j = 0;j <250;j++)
    {
        LTC6804_adow();                                                        // ����ADOW����
    }
    
    LTC6804_rdcv(0,NUM_IC, Cell_vol_down, PEC_error_a);                         // ��ȡAD��ѹֵ
    
    //Openwire_error=0x01;
    for(j = 0;j < NUM_IC;j++)
    {
        Openwire_flag[j]=0;
        for(i = 0;i < 13;i++) 
        {  
          Open_wire[j][i] = 0;
        }
    } 
    //13��������
    
    for(j = 0;j <NUM_IC;j++)                
    {
        switch(j)
         {
          case 0:
            for(i = 1;i < 12 ;i++)
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

    for(j = 0;j < NUM_IC;j++)
    {
      for(i = 0;i <13;i++) 
      {
        Openwire_flag[j]=Openwire_flag[j]+(Open_wire[j][i]<<i);
      }
    } 
    for(j = 0;j < NUM_IC;j++)
    {
     if(Openwire_flag[j]!=0)
      {
        VoltInfo.Openwire_error=1;
      }
    }
    Task_Flag_Counter.Counter_Wire_Open++;  
}
 

/*=======================================================================*/