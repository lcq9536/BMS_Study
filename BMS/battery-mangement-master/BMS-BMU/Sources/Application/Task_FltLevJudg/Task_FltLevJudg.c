/*=======================================================================
 *Subsystem:   ���
 *File:        Task_FltLevJudg.c
 *Author:      Wenming
 *Description: 
 ========================================================================
 * History:    �޸���ʷ��¼�б��޸ļ�¼�����޸����ڡ��޸��߼��޸�����
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "includes.h"

/*=======================================================================
                                  ����
========================================================================*/ 
Fault_DisCharge_T  g_Flt_DisChg; 
Fault_Charge_T     g_Flt_Charge;
State_Offline_T    State_Offline;
HeartBeat_T        HeartBeat;

static void FltLevJudg(uint8 workstate);
//�ŵ�����ж�
static uint8 Fault_DisChg_VoltSL(uint32 Volt,uint8 Temp); //�ŵ���ѹ��
static uint8 Fault_DisChg_VoltCL(uint16 Volt,uint8 Temp); //�ŵ絥���ѹ��
static uint8 Fault_DisChg_VoltCD(uint16 V_Diff);         //�ŵ�ѹ��
static uint8 Fault_DisChg_TempH(uint8 Temp);             //�ŵ����
static uint8 Fault_DisChg_TempL(uint8 Temp);             //�ŵ����
static uint8 Fault_DisChg_TempD(uint8 T_Diff);           //�ŵ��²�
static uint8 Fault_DisChg_CurrH(float Current);          //�ŵ����
static uint8 Fault_DisChg_Insul(uint16 Insul);           //��Ե����
//�������ж�
static uint8 Fault_Charge_VoltSH(uint32 Volt);            //�����ѹ��
static uint8 Fault_Charge_VoltCH(uint16 Volt);           //��絥���
static uint8 Fault_Charge_VoltCD(uint16 V_Diff);         //���ѹ��
static uint8 Fault_Charge_TempH(uint8 Temp);             //������
static uint8 Fault_Charge_TempL(uint8 Temp);             //������
static uint8 Fault_Charge_TempD(uint8 T_Diff);           //����²�
static uint8 Fault_Charge_CurrH(float Current);          //������
static uint8 Fault_Charge_Insul(uint16 Insul);           //��Ե����
static uint8 Fault_Charge_OffLine(void);
//ͨ�ŵ���
static uint8 Fault_CSSU_OffLine(void);                   //�Ӱ����
static uint8 Fault_Relay_BreakDown(void);                //�̵���ճ������
 /*=======================================================================
                              �����жϺ���0x00
 ========================================================================
 *������1:     Init_TaskFltLevJudg(void)
 *����:        
 *����:        ��    
 *���أ�       ��
 
 *˵����       
========================================================================*/ 
void Init_TaskFltLevJudg(void)
{
  //���ϵȼ��жϱ�������
  memset(&g_Flt_DisChg,           0, sizeof(Fault_DisCharge_T));  //�ŵ���ϵȼ�����
  memset(&g_Flt_Charge,           0, sizeof(Fault_Charge_T));     //�����ϵȼ�����
  memset(&State_Offline,          0, sizeof(State_Offline_T));    //���߹���״̬����
  memset(&HeartBeat,              0, sizeof(HeartBeat_T));        //�����ź�����
}

/*=======================================================================
                              �����жϺ���0x00
 ========================================================================
 *������1:     Task_FaultLevelJudge(uint8 workstate)
 *����:        ����ϵͳ��ѹ���¶ȡ������жϹ��ϵȼ�
 *����:        ��    
 *���أ�       ��
 
 *˵����       
========================================================================*/ 
void Task_FltLevJudg(void)
{
  FltLevJudg(g_WorkStateJudge.WorkState);  
}

/*=======================================================================
                              �����жϺ���0x00
 ========================================================================
 *������1:     FltLevJudg(uint8 workstate)
 *����:        ����ϵͳ��ѹ���¶ȡ������жϹ��ϵȼ�
 *����:        ��    
 *���أ�       ��
 
 *˵����       
========================================================================*/ 
static
void FltLevJudg(uint8 workstate)
{
   State_Offline.CSSU1 = Fault_CSSU_OffLine();//�Ӱ����
   State_Offline.RelayFlt_Positive = Fault_Relay_BreakDown();//�̵���ճ������
   switch(workstate)
   {
    case MODE_DISCHARGE: //�ŵ�״̬
      //������״̬����
      g_Flt_Charge.Level_Charge_SwitchOff_flag  = 0; 
      g_Flt_Charge.Level_Charge_BalanceON_Flag  = 0;
      g_Flt_Charge.Level_Volt_Sys_High          = 0;
      g_Flt_Charge.Level_Volt_Cell_High         = 0;
      g_Flt_Charge.Level_Insul                  = 0;
      State_Offline.Charge                      = 0;
      //�жϷŵ�״̬����
      g_Flt_DisChg.Level_Volt_Sys_Low           = Fault_DisChg_VoltSL(g_VoltInfo.SysVolt_Total, g_TempInfo.CellTemp_Ave);
      g_Flt_DisChg.Level_Volt_Cell_Low          = Fault_DisChg_VoltCL(g_VoltInfo.CellVolt_Min, g_TempInfo.CellTemp_Ave);
      g_Flt_DisChg.Level_Volt_Cell_Diff_High    = Fault_DisChg_VoltCD(g_VoltInfo.CellVolt_Diff);
      g_Flt_DisChg.Level_Temp_High              = Fault_DisChg_TempH(g_TempInfo.CellTemp_Max);
      g_Flt_DisChg.Level_Temp_Low               = Fault_DisChg_TempL(g_TempInfo.CellTemp_Min);
      g_Flt_DisChg.Level_Temp_Diff_High         = Fault_DisChg_TempD(g_TempInfo.CellTemp_Diff);
      g_Flt_DisChg.Level_Current_DisCharge_High = Fault_DisChg_CurrH(g_DataColletInfo.DataCollet_Current_Filter);
      g_Flt_DisChg.Level_Insul                  = Fault_DisChg_Insul(g_IsoDetect.insulation_resist);
      //�Ͽ��̵����Ķ������ϱ��
      if((g_Flt_DisChg.Level_Volt_Sys_Low==2) ||\
         (g_Flt_DisChg.Level_Volt_Cell_Low == 2)||\
         (g_Flt_DisChg.Level_Temp_High == 2)||\
         (g_Flt_DisChg.Level_Temp_Low == 2) ||\
         (g_Flt_DisChg.Level_Current_DisCharge_High == 2) ||\
         (g_Flt_DisChg.Level_Insul == 2)||(State_Offline.CSSU1 == 1))
      {
        g_Flt_DisChg.Level_DisCharge_SwitchOff_flag = 1;  
      }
      
    break;
    
    case MODE_CHARGE:   //���״̬
      //����ŵ�״̬����
      g_Flt_DisChg.Level_Volt_Sys_Low             = 0;
      g_Flt_DisChg.Level_Volt_Cell_Low            = 0;
      g_Flt_DisChg.Level_DisCharge_SwitchOff_flag = 0;
      g_Flt_DisChg.Level_Insul                    = 0;
      //�жϳ��״̬����
      g_Flt_Charge.Level_Volt_Sys_High        = Fault_Charge_VoltSH(g_VoltInfo.SysVolt_Total);
      g_Flt_Charge.Level_Volt_Cell_High       = Fault_Charge_VoltCH(g_VoltInfo.CellVolt_Max);
      g_Flt_Charge.Level_Volt_Cell_Diff_High  = Fault_Charge_VoltCD(g_VoltInfo.CellVolt_Diff);
      g_Flt_Charge.Level_Temp_High            = Fault_Charge_TempH(g_TempInfo.CellTemp_Max);
      g_Flt_Charge.Level_Temp_Low             = Fault_Charge_TempL(g_TempInfo.CellTemp_Min);
      g_Flt_Charge.Level_Temp_Diff_High       = Fault_Charge_TempD(g_TempInfo.CellTemp_Diff);
      g_Flt_Charge.Level_Current_Charge_High  = Fault_Charge_CurrH(g_DataColletInfo.DataCollet_Current_Filter);
      g_Flt_Charge.Level_Insul                = Fault_Charge_Insul(g_IsoDetect.insulation_resist);
      State_Offline.Charge                    = Fault_Charge_OffLine();
      //�Ͽ��̵����Ķ������ϱ��
      if((g_Flt_Charge.Level_Volt_Sys_High==2) ||\
         (g_Flt_Charge.Level_Volt_Cell_High==2) ||\
         (g_Flt_Charge.Level_Temp_High == 2)||\
         (g_Flt_Charge.Level_Temp_Low == 2)||\
         (g_Flt_Charge.Level_Insul == 2)||(State_Offline.CSSU1 == 1)||\
         (State_Offline.Charge == 1))
      {
        g_Flt_Charge.Level_Charge_SwitchOff_flag = 1;//2�����ϱպϼ̵���  
      }
      //���⿪�����
      if((g_Flt_Charge.Level_Volt_Sys_High!=0) ||\
         (g_Flt_Charge.Level_Volt_Cell_High != 0)|\
         (g_Flt_Charge.Level_Temp_High != 0)||\
         (g_Flt_Charge.Level_Temp_Low != 0) ||\
         (g_Flt_Charge.Level_Temp_Diff_High != 0) ||\
         (g_Flt_Charge.Level_Current_Charge_High != 0) ||\
         (g_Flt_Charge.Level_Insul != 0)||(State_Offline.CSSU1 != 0)||\
         (State_Offline.RelayFlt_Positive != 0)||(State_Offline.Charge != 0))
      {
        g_Flt_Charge.Level_Charge_BalanceON_Flag = 0;//ֻҪ���ֹ�������������(��ѹ�����)  
      }
      else
      {
        g_Flt_Charge.Level_Charge_BalanceON_Flag = 1;//��������
      }
            
    break;
   }   
   g_Roll_Tick.Roll_FltJudg++; 
}
/*============================�����жϺ���===============================*/

/*=======================================================================
                              �ŵ���ѹ��0x01
 ======================================================================*/
//1�����Ϻ���
static
uint32 Fault1_VoltSys_DisCharge(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Sys_Low1);//(28500)*(25)
  }
  else //����
  {
    return (F1_DISCHG_VOLTSL_LT*0.001);    //����1��27000.0*25
  }
}
//2�����Ϻ���
static
uint32 Fault2_VoltSys_DisCharge(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Sys_Low2);
  }
  else //����
  {
    return (F2_DISCHG_VOLTSL_LT*0.001);  //����2��25000.0*25
  }
}
//1�ָ���0
static
uint32 Recover1_VoltSys_DisCharge(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Sys_Low1 + 2*SYS_SERIES_YiDongLi);
  }
  else //����
  {
    return (F1_DISCHG_VOLTSL_LT*0.001 + SYS_SERIES_YiDongLi);//�ɻ��ɱ궨����(�ֱ��ʣ�0.001V/λ)
  }
}
//�����ж�
static uint8 Fault_DisChg_VoltSL(uint32 Volt,uint8 Temp)  //����ϵͳ��ѹ�ͻ����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Volt/1000.0<=Fault1_VoltSys_DisCharge(Temp))  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Volt/1000.0<=Fault2_VoltSys_DisCharge(Temp))  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Volt/1000.0 <= Fault2_VoltSys_DisCharge(Temp))      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Volt/1000.0 >= Recover1_VoltSys_DisCharge(Temp))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                              �ŵ絥���ѹ��0x02
 ======================================================================*/

//1�����Ϻ���
static
uint16 Fault1_VoltCell_DisCharge(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Cell_Low1);
  }
  else //����
  {
    return (F1_DISCHG_VOLTCL_LT);
  }
}
//2�����Ϻ���
static
uint16 Fault2_VoltCell_DisCharge(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Cell_Low2);
  }
  else //����
  {
    return (F2_DISCHG_VOLTCL_LT);
  }
}
//�ָ���0
static
uint16 Recover1_VoltCell_DisCharge(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Cell_Low1 + 2000);
  }
  else //����
  {
    return (F1_DISCHG_VOLTCL_LT + 1000);
  }
}

//�жϹ��ϵȼ�
static uint8 Fault_DisChg_VoltCL(uint16 Volt,uint8 Temp)  //���뵥���ѹ�ͻ����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Volt<=Fault1_VoltCell_DisCharge(Temp))  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Volt<=Fault2_VoltCell_DisCharge(Temp))  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Volt <= Fault2_VoltCell_DisCharge(Temp))      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Volt >= Recover1_VoltCell_DisCharge(Temp))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                              �ŵ絥��ѹ��0x03
 ======================================================================*/
static uint8 Fault_DisChg_VoltCD(uint16 V_Diff)  //���뵥�����/�͵�ѹ
{
  static uint8 cnt[2];      //ʱ�����
  static uint8 FltL;
  
  //�жϹ��ϵȼ�
  if(FltL==0)           //0������
  {
    if(V_Diff>=g_BMSMonitor_Volt.Volt_Cell_Diff1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0; 
    }
    cnt[1] = 0; 
  }
  else     //1������
  {
    if(V_Diff<=(g_BMSMonitor_Volt.Volt_Cell_Diff1 - 500))    //1��0
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[0] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                �ŵ����0x04
 ======================================================================*/
static uint8 Fault_DisChg_TempH(uint8 Temp)  //�����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Temp>=g_BMSMonitor_Temp.Temp_DisCharge_High1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Temp>=g_BMSMonitor_Temp.Temp_DisCharge_High2)  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Temp>=g_BMSMonitor_Temp.Temp_DisCharge_High2)      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Temp<=(g_BMSMonitor_Temp.Temp_DisCharge_High1 - 2))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                �ŵ����0x05
 ======================================================================*/
  
//�жϹ��ϵȼ�
static uint8 Fault_DisChg_TempL(uint8 Temp)  //�����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Temp<=g_BMSMonitor_Temp.Temp_DisCharge_Low1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Temp<=g_BMSMonitor_Temp.Temp_DisCharge_Low2)  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Temp<=g_BMSMonitor_Temp.Temp_DisCharge_Low2)      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Temp>=(g_BMSMonitor_Temp.Temp_DisCharge_Low1 + 3))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                              �ŵ��²�0x06
 ======================================================================*/
static uint8 Fault_DisChg_TempD(uint8 T_Diff)  //���뵥�����/�͵�ѹ
{
  static uint8 cnt[2];      //ʱ�����
  static uint8 FltL;
  
  //�жϹ��ϵȼ�
  if(FltL==0)           //0������
  {
    if(T_Diff>=(g_BMSMonitor_Temp.Temp_DisCharge_Diff1))  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0; 
    }
    cnt[1] = 0; 
  }
  else     //1������
  {
    if(T_Diff<=(g_BMSMonitor_Temp.Temp_DisCharge_Diff1-2))    //1��0
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[0] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                �ŵ����0x07
 ======================================================================*/
static uint8 Fault_DisChg_CurrH(float Current)//�������
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Current>=(g_BMSMonitor_Curr.Current_DisCharge_High1*0.1-750))  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Current>=(g_BMSMonitor_Curr.Current_DisCharge_High2*0.1-750))  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Current>=(g_BMSMonitor_Curr.Current_DisCharge_High2*0.1-750))      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Current<=(g_BMSMonitor_Curr.Current_DisCharge_High1*0.1-750 - 20))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                �ŵ��Ե����0x08
 ======================================================================*/
static uint8 Fault_DisChg_Insul(uint16 Insul)
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    
    if(Insul<=(g_BMSMonitor_Insul.Insulation_Resis2*0.1))  //0��2
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[0] = 0;
    }
  }
  return(FltL);
}


/*=======================================================================
                              �����ѹ��0x11
 ======================================================================*/
static uint8 Fault_Charge_VoltSH(uint32 Volt)  //����ϵͳ��ѹ
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Volt/1000.0>=g_BMSMonitor_Volt.Volt_Sys_High1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Volt/1000.0>=g_BMSMonitor_Volt.Volt_Sys_High2)  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Volt/1000.0>=g_BMSMonitor_Volt.Volt_Sys_High2)      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Volt/1000.0<=(g_BMSMonitor_Volt.Volt_Sys_High1 - 0.5*SYS_SERIES_YiDongLi))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                              ��絥���ѹ��0x12
 ======================================================================*/
static uint8 Fault_Charge_VoltCH(uint16 Volt)  //���뵥���ѹ�ͻ����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Volt>=g_BMSMonitor_Volt.Volt_Cell_High1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Volt>=g_BMSMonitor_Volt.Volt_Cell_High2)  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Volt>=g_BMSMonitor_Volt.Volt_Cell_High2)      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Volt<=(g_BMSMonitor_Volt.Volt_Cell_High1 - 500))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                              ��絥��ѹ��0x13
 ======================================================================*/
static uint8 Fault_Charge_VoltCD(uint16 V_Diff)  //���뵥�����/�͵�ѹ
{
  static uint8 cnt[2];      //ʱ�����
  static uint8 FltL;
  
  //�жϹ��ϵȼ�
  if(FltL==0)           //0������
  {
    if(V_Diff>=g_BMSMonitor_Volt.Volt_Cell_Diff1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0; 
    }
    cnt[1] = 0; 
  }
  else     //1������
  {
    if(V_Diff<=(g_BMSMonitor_Volt.Volt_Cell_Diff1 - 500))    //1��0
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[0] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                ������0x14
 ======================================================================*/

//�жϹ��ϵȼ�
static uint8 Fault_Charge_TempH(uint8 Temp)  //�����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Temp>=g_BMSMonitor_Temp.Temp_Charge_High1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Temp>=g_BMSMonitor_Temp.Temp_Charge_High2)  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Temp>=g_BMSMonitor_Temp.Temp_Charge_High2)      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Temp<=(g_BMSMonitor_Temp.Temp_Charge_High1 - 2))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                ������0x15
 ======================================================================*/
 
//�жϹ��ϵȼ�
static 
uint8 Fault_Charge_TempL(uint8 Temp)  //�����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(Temp<=g_BMSMonitor_Temp.Temp_Charge_Low1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(Temp<=g_BMSMonitor_Temp.Temp_Charge_Low2)  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(Temp<=g_BMSMonitor_Temp.Temp_Charge_Low2)      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(Temp>=(g_BMSMonitor_Temp.Temp_Charge_Low1 + 5))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                              ����²�0x16
 ======================================================================*/
static 
uint8 Fault_Charge_TempD(uint8 T_Diff)  //���뵥�����/�͵�ѹ
{
  static uint8 cnt[2];      //ʱ�����
  static uint8 FltL;
  
  //�жϹ��ϵȼ�
  if(FltL==0)           //0������
  {
    if(T_Diff>=g_BMSMonitor_Temp.Temp_Charge_Diff1)  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0; 
    }
    cnt[1] = 0; 
  }
  else     //1������
  {
    if(T_Diff<=(g_BMSMonitor_Temp.Temp_Charge_Diff1-2))    //1��0
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[0] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                                ������0x17
 ======================================================================*/

static 
uint8 Fault_Charge_CurrH(float Current)//�����¶�
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    if(abs(Current)>=abs(g_BMSMonitor_Curr.Current_Charge_High1*0.1-750))  //0��1
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 1;
      }
    }
    else
    {
      cnt[0] = 0;
    }
    
    if(abs(Current)>=abs(g_BMSMonitor_Curr.Current_Charge_High2*0.1-750))  //0��2
    {
      if(++cnt[1]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[1] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[1] = 0;
    }
    cnt[2] = 0; 
    cnt[3] = 0;
  }
  else if(FltL == 1) //1������
  {
    if(abs(Current)>=abs(g_BMSMonitor_Curr.Current_Charge_High2*0.1-750))      //1��2
    {
      if(++cnt[2]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
         cnt[2] = 0;
         FltL = 2;
      }
    }
    else
    {
      cnt[2] = 0;
    }
    
    if(abs(Current)<=abs(g_BMSMonitor_Curr.Current_Charge_High1*0.1-750 - 5))    //1��0
    {
      if(++cnt[3]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[3] = 0;
        FltL = 0;
      }
    }
    else
    {
      cnt[3] = 0;
    }
    cnt[0] = 0; 
    cnt[1] = 0;
  }
  else   //2������
  {
    cnt[0] = 0; 
    cnt[1] = 0;
    cnt[2] = 0;
    cnt[3] = 0; 
  }
  return(FltL);
}

/*=======================================================================
                           �Ӱ���߹��Ϲ���0x18
 ======================================================================*/
static 
uint8 Fault_CSSU_OffLine(void)
{
  static uint16 cnt;      
  static uint8 state=0;
  if(HeartBeat.HeartBeat_CSSU1 == 1 )
  { 
     HeartBeat.HeartBeat_CSSU1 = 0;
     state = 0;
     cnt = 0;      
  }
  else
  {
     if(++cnt*PERIOD_DISCHARGE/1000 >= 10)//10S
     {
       cnt = 0;
       state = 1; 
       //memset(&g_FromCSSU_Temp, 0x00, sizeof(FromCSSU_Temp_T));//�Ӱ���ߺ�����Ӱ���Ϣ
       //memset(&g_FromCSSU_Volt, 0x00, sizeof(FromCSSU_Volt_T)); 
       //memset(&g_FromCSSU_FltData, 0x00, sizeof(FromCSSU_FltData_T));
     }
  }
  return state;
}

/*=======================================================================
                              ����Ե����0x19
 ======================================================================*/
static 
uint8 Fault_Charge_Insul(uint16 Insul)
{
  static uint8 cnt[4];      //ʱ�����
  static uint8 FltL;
  
  if(FltL==0)           //0������
  {
    
    if(Insul <= g_BMSMonitor_Insul.Insulation_Resis2*0.1)  //0��2
    {
      if(++cnt[0]*PERIOD_DISCHARGE/1000>=DELAYTIME_DANGERLEVEL2)
      {
        cnt[0] = 0;
        FltL = 2;
      }
    }
    else
    {
      cnt[0] = 0;
      //FltL = 0;
    }
  }
  return(FltL);
} 


/*=======================================================================
                            �̵���ճ������0x19
 ======================================================================*/ 
static 
uint8 Fault_Relay_BreakDown(void)
{
   static uint8 cnt[2];
   static uint8 flt; 
   if(Port_StateGet(Relay_Positive_PORT, Relay_Positive_pin) == Relay_ON)//�̵���������״̬
   {
      if(abs(g_VoltInfo.SysVolt_Total/10000.0-g_FromCSSU_Volt.InsulVolt_Total/10000.0)>0.5*CELL_VOLT_NOMINAL*SYS_SERIES_YiDongLi)
      {
          if(++cnt[0]*PERIOD_DISCHARGE/1000>=2)
          {
             cnt[0] = 0;    
             flt = 1;
          }
      }
      else
      {
         flt = 0;
         cnt[0] = 0;   
      }
      cnt[1] = 0;
   }
   else//�̵����رյ�״̬
   {
      if(abs(g_VoltInfo.SysVolt_Total/10000.0-g_FromCSSU_Volt.InsulVolt_Total/10000.0)<0.5*CELL_VOLT_NOMINAL*SYS_SERIES_YiDongLi)
      {
          if(++cnt[1]*PERIOD_DISCHARGE/1000>=2)
          {
             cnt[1] = 0;   
             flt = 1;
          }
      }
      else
      {
         flt = 0;
         cnt[1] = 0;     
      }
      cnt[0] = 0;
   }
   return flt;
}

/*=======================================================================
                           ���׮���߹���0x20
 ======================================================================*/
static 
uint8 Fault_Charge_OffLine(void)
{
  static uint16 cnt;      
  static uint8 state=0;
  if(HeartBeat.HeartBeat_Charge == 1 )
  { 
     HeartBeat.HeartBeat_Charge = 0;
     state = 0;
     cnt = 0;      
  }
  else
  {
     if(++cnt*PERIOD_DISCHARGE/1000 >= 50)//50S
     {
       cnt = 0;
       state = 1; 
     }
  }
  return state;
}
