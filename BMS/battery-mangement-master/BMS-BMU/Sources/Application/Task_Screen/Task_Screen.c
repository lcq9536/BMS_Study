/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Screen.C
 *Author:      WenMing
 *Description: ͨ�ţ�SCI2.
               �ӿڣ�PJ0��RXD����PJ1��TXD��
               �����ʣ�
               ���ԣ��۲�RS485_Receive��������ʾ�������Ƿ�һ��,�Լ�IICģ��ʱ�����ʾ��
 ========================================================================
 * History:           �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ���
 * 1. Date:           
      Author:         
      Modification:   
========================================================================*/
#include  "includes.h"
RS485  RS485_Receive; 

static uint8 RS485_Init(void);        
/*=======================================================================
 *������:      Init_Screen(void)
 *����:        ��ʼ����ʾ���ӿ�
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
uint8 Init_Screen(void)
{
   uint8 state;
   
   state = RS485_Init();
   state = state|SCI1_Init();
   return state;
}


/*=======================================================================
 *������:      Init_Screen(void)
 *����:        ��ʼ����ʾ���ӿ�
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
static
void SCI_ScreenTransfer(uint8 numbyte, uint8 *data)
{
   SCI1_Send_NByte(numbyte, data);
}
/*=======================================================================
 *������:      RS485_DataMemset(void)
 *����:        ��ʼ��485����
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
static
uint8 RS485_Init(void) 
{
  RS485_EnableDir = 1;
  RS485_Enable = 1; 
  
  memset(&RS485_Receive,0x00, sizeof(RS485));//���485������
  return 0;
}

/*=======================================================================
 *������:      RS485_DataReceive
 *����:        SCI���ݽ���
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
static
void RS485_DataReceice(void) 
{ 
 /*----------------------------------------˫�ֽڽ���-----------------------------------*/
  RS485_Receive.TxData_couple.TX2.BMS_SOH             = (uint16)((g_BMSMonitor_SOH.SOH+0.005)*100); 
  RS485_Receive.TxData_couple.TX2.RunningTime         = (uint16)(g_SysTime.BMS_TotalRun_MiniteTime/60);           
  RS485_Receive.TxData_couple.TX2.BMS_Current         = (uint16)((g_DataColletInfo.DataCollet_Current_Filter + 750)*10);                  
  RS485_Receive.TxData_couple.TX2.BMS_SOC             = (uint16)((g_SOCInfo.SOC_ValueRead+0.005)*100);                                                    
  RS485_Receive.TxData_couple.TX2.Pack_Hightemp       = (uint16)g_TempInfo.CellTemp_Max;               
  RS485_Receive.TxData_couple.TX2.Pack_Lowtemp        = (uint16)g_TempInfo.CellTemp_Min;                
  RS485_Receive.TxData_couple.TX2.Pack_Volt           = (uint16)(g_VoltInfo.SysVolt_Total/1000.0);                 
  RS485_Receive.TxData_couple.TX2.Single_Maxvolt      = (g_VoltInfo.CellVolt_Max +5)/10;      
  RS485_Receive.TxData_couple.TX2.Single_Lowvolt      = (g_VoltInfo.CellVolt_Min +5)/10;        
  RS485_Receive.TxData_couple.TX2.iso_resistance      = g_IsoDetect.insulation_resist;    //��Ե����
                       
 /*------------------------------------------���ֽڽ���---------------------------------*/       
  RS485_Receive.TxData_single.TX1.Alam_SOC            = 0x00;
  RS485_Receive.TxData_single.TX1.Alam_VoltSL         = (uint8)g_Flt_DisChg.Level_Volt_Sys_Low;
  RS485_Receive.TxData_single.TX1.Alam_VoltCL         = (uint8)g_Flt_DisChg.Level_Volt_Cell_Low;
  RS485_Receive.TxData_single.TX1.Alam_TempH_DisChg   = (uint8)g_Flt_DisChg.Level_Temp_High;
  RS485_Receive.TxData_single.TX1.Alam_TempL_DisChg   = (uint8)g_Flt_DisChg.Level_Temp_Low;
  RS485_Receive.TxData_single.TX1.Alam_CurrH_DisChg   = (uint8)g_Flt_DisChg.Level_Current_DisCharge_High;
  RS485_Receive.TxData_single.TX1.Alam_VoltCD_DisChg  = (uint8)g_Flt_DisChg.Level_Volt_Cell_Diff_High;
  RS485_Receive.TxData_single.TX1.Alam_TempD_DisChg   = (uint8)g_Flt_DisChg.Level_Temp_Diff_High;
  
  RS485_Receive.TxData_single.TX1.Alam_VoltSH         = (uint8)g_Flt_Charge.Level_Volt_Sys_High;
  RS485_Receive.TxData_single.TX1.Alam_VoltCH         = (uint8)g_Flt_Charge.Level_Volt_Cell_High;
  RS485_Receive.TxData_single.TX1.Alam_TempH_Charge   = (uint8)g_Flt_Charge.Level_Temp_High;
  RS485_Receive.TxData_single.TX1.Alam_TempL_Charge   = (uint8)g_Flt_Charge.Level_Temp_Low;
  RS485_Receive.TxData_single.TX1.Alam_CurrH_Charge   = (uint8)g_Flt_Charge.Level_Current_Charge_High;
  RS485_Receive.TxData_single.TX1.Alam_VoltCD_Charge  = (uint8)g_Flt_Charge.Level_Volt_Cell_Diff_High;
  RS485_Receive.TxData_single.TX1.Alam_TempD_Charge   = (uint8)g_Flt_Charge.Level_Temp_Diff_High;
  
  RS485_Receive.TxData_single.TX1.Alam_Insul          = (uint8)(g_Flt_DisChg.Level_Insul|g_Flt_Charge.Level_Insul);
  RS485_Receive.TxData_single.TX1.Alam_Checkself      = 0x01;
  
 }   
/*=======================================================================
 *������:      Screen_delay(uint8)
 *����:        ��ʾ������ʱ����
 *����:        rs:Ҫ���ͽṹ����ֽ�ָ��       
 *���أ�       ��
 *˵����       
========================================================================*/
static
void Screen_delay(uint16 ticks)
{
  uint16 i;
  for(i=0; i<ticks; i++);
}
/*=======================================================================
 *������:      Task_ScreenTransfer
 *����:        SACI���ʹ���
 *����:        rs:Ҫ���ͽṹ����ֽ�ָ��       
 *���أ�       ��
 *˵����       
========================================================================*/
void Task_ScreenTransfer(void) 
{          
   RS485_DataReceice();
   Screen_delay(10);
   SCI_ScreenTransfer(Array_couple, RS485_Receive.TxData_couple.SCI_Content2);  
   SCI_ScreenTransfer(Array_single, RS485_Receive.TxData_single.SCI_Content1); 
   g_Roll_Tick.Roll_Screen++;
}