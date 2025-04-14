/*=======================================================================
 *Subsystem:   ���
 *File:        BMSCheckSelf.C
 *Author:      WenYuhao
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
               ֻ���Լ�ɹ�����ܱպ���Ӧ�Ŀ���
 * ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    
      Author:  
      Modification: 
========================================================================*/
#include  "includes.h"

Flt_BMSCheckSelf_T  g_Flt_BMSCheckSelf;
/*=======================================================================
 *������:      �ŵ��Լ��ж��� 
 *����:        ��ʼ��������Լ�,�Լ�ֻҪһ��ͨ����պϼ̵���
 *����:        ���ո�������       
 *���أ�       ���ո�������
 *˵����       
========================================================================*/
/************************�ŵ�ϵͳ��ѹ����********************************/
static
uint32 CheckSelf_SysVoltLow_DischagTemp(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Sys_Low2);
  }
  else //����
  {
    return (g_BMSMonitor_New_LT.Voll_Sys_Low2_LT);
  }
}
static
uint8  CheckSelf_SysVoltLow_DisCharge(uint32 totalvolt, uint8 temp)
{
   static uint8 cnt=0;
   if(totalvolt/1000.0 > CheckSelf_SysVoltLow_DischagTemp(temp))
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
      
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1; //�Լ����
      }
      
   }
   return 2;
}

/************************�ŵ絥���ѹ����********************************/
static
uint16 CheckSelf_CellVoltLow_Temp(uint8 Temp)
{
  if(Temp>=NTEMP_BOUNDARY)//����(>=0��)
  { 
    return (g_BMSMonitor_Volt.Volt_Cell_Low2);//�ɻ��ɱ궨����(�ֱ��ʣ�0.001V/λ)
  }
  else //����
  {
    return (g_BMSMonitor_New_LT.Volt_Cell_Low2_LT);//�ɻ��ɱ궨����(�ֱ��ʣ�0.001V/λ)
  }
}

static
uint8  CheckSelf_CellVoltLow_DisCharge(uint16 Voltmin, uint8 temp)
{
   static uint8 cnt=0;
   if(Voltmin > CheckSelf_CellVoltLow_Temp(temp))
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1; //�Լ����
      }
      
   }
   return 2;
}

/************************�ŵ絥���¶ȹ���********************************/
static
uint8  CheckSelf_CellTempHigh_DisCharge(uint8 Temp)
{
   static uint8 cnt=0;
   if(Temp < g_BMSMonitor_Temp.Temp_DisCharge_High2)
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1; //�Լ����
      }
      
   }
   return 2;
}

/************************�ŵ絥���¶ȹ���********************************/
static
uint8  CheckSelf_CellTempLow_DisCharge(uint8 Temp)
{
   static uint8 cnt=0;
   if(Temp > g_BMSMonitor_Temp.Temp_DisCharge_Low2)
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1; //�Լ����
      }
      
   }
   return 2;
}

/************************�ŵ����������********************************/
static
uint8  CheckSelf_CurrentOver_DisCharge(float curr)
{
   static uint8 cnt=0;
   if(curr < (g_BMSMonitor_Curr.Current_DisCharge_High2*0.1-750))
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1;//�Լ����
      }
      
   }
   return 2;
}

/************************��Ե����********************************/
static
uint8  CheckSelf_InsulDetect(uint8 insul)
{
   static uint8 cnt=0;
   if(insul > (g_BMSMonitor_Insul.Insulation_Resis2*0.1)) //0.1k��/V
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1;//�Լ����
      }
      
   }
   return 2;
}

/*=======================================================================
 *������:      ����Լ��ж��� 
 *����:        ��ʼ��������Լ�,�Լ�ֻҪһ��ͨ����պϼ̵���
 *����:        ���ո�������       
 *���أ�       ���ո�������
 *˵����       
========================================================================*/
/************************���ϵͳ��ѹ����********************************/
static
uint8  CheckSelf_SysVoltHigh_Charge(uint32 totalvolt)
{
   static uint8 cnt=0;

   if(totalvolt/1000.0 < g_BMSMonitor_Volt.Volt_Sys_High2)//��ֲʱע��ֱ��ʺ�uint16�Ƿ����
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
      
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1;//�Լ����
      }
      
   }
   return 2;
}
/************************��絥���ѹ����********************************/
static
uint8  CheckSelf_CellVoltHigh_Charge(uint16 Voltmax)
{
   static uint8 cnt=0;
   if(Voltmax< g_BMSMonitor_Volt.Volt_Cell_High2)     //����ͬ��
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1;//�Լ����
      }
      
   }
   return 2;
}
/********************************************************************/
static
uint8  CheckSelf_CellTempHigh_Charge(uint8 Temp)
{
   static uint8 cnt=0;
   if(Temp < g_BMSMonitor_Temp.Temp_Charge_High2)  //ע��ƫ����
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1;//�Լ����
      }
   }
   return 2;
}
/********************************************************************/
static
uint8  CheckSelf_CellTempLow_Charge(uint8 Temp)
{
   static uint8 cnt=0;
   if(Temp > g_BMSMonitor_Temp.Temp_Charge_Low2) //ע��ƫ����
   {
      cnt = 0;
      return 0;  //�Լ�ɹ�
   }
   else
   {
      if(cnt >= 2)
      {
        cnt = 2;
      }
      if(++cnt>=2)
      {
        return 1;
        //�Լ����
      }
      
   }
   return 2;
} 

/*=======================================================================
 *������:      CheckSelf_OpenWireDetect 
 *����:        ��Ե����״̬�ж�
 *����:        state:��Ե�������״̬       
 *���أ�       uint8:�Ƿ���ھ�Ե����,0:����;1:����
 *˵����       ��ʼ����Ϊ�Լ�ʹ��
========================================================================*/
static
uint8  CheckSelf_OpenWireDetect(uint8 state)
{
  if(state == 0)
  {
     return 0;  //�Լ�ɹ�
  }
  return 1;
}

/*=======================================================================
 *������:      CheckSelf_Discharge() 
 *����:        �ŵ�ʱ�Լ��״̬�ж�
 *����:        ��      
 *���أ�       uint8:�Ƿ�����Լ����,0:����;1:����
 *˵����       ��ʼ����Ϊ�Լ�ʹ��
========================================================================*/
static
uint8 CheckSelf_Discharge(Flt_BMSCheckSelf_T*ptr)
{
   uint8 state=0;
   memset(ptr, 0x00, sizeof(Flt_BMSCheckSelf_T));
   
   ptr->SysVolt_Low    = CheckSelf_SysVoltLow_DisCharge(g_VoltInfo.SysVolt_Total, g_TempInfo.CellTemp_Ave);
   state = state|ptr->SysVolt_Low;
   
   ptr->CellVolt_Low   = CheckSelf_CellVoltLow_DisCharge(g_VoltInfo.CellVolt_Min, g_TempInfo.CellTemp_Ave);
   state = state|ptr->CellVolt_Low;
   
   ptr->CellTemp_Over  = CheckSelf_CellTempHigh_DisCharge(g_TempInfo.CellTemp_Max);
   state = state|ptr->CellTemp_Over;
   
   ptr->CellTemp_Low   = CheckSelf_CellTempLow_DisCharge(g_TempInfo.CellTemp_Min);
   state = state|ptr->CellTemp_Low;
   
   ptr->SysCurr_Over   = CheckSelf_CurrentOver_DisCharge(g_DataColletInfo.DataCollet_Current_Filter);
   state = state|ptr->SysCurr_Over;
   
   ptr->SysInsul_Flt   = CheckSelf_InsulDetect(g_IsoDetect.insulation_resist);
   state = state|ptr->SysInsul_Flt;
   
   ptr->OpenWire_Flt  = CheckSelf_OpenWireDetect(g_OpenWireInfo.OpenWire_Status);
   state = state|ptr->OpenWire_Flt;
   return state;
}

/*=======================================================================
 *������:      CheckSelf_Charge() 
 *����:        ���ʱ�Լ��״̬�ж�
 *����:        ��      
 *���أ�       uint8:�Ƿ�����Լ����,0:����;1:����
 *˵����       ��ʼ����Ϊ�Լ�ʹ��
========================================================================*/
static
uint8 CheckSelf_Charge(Flt_BMSCheckSelf_T*ptr)
{
   uint8 state=0;
   memset(ptr, 0x00, sizeof(Flt_BMSCheckSelf_T));
   
   ptr->SysVolt_Over   = CheckSelf_SysVoltHigh_Charge(g_VoltInfo.SysVolt_Total);
   state = state|ptr->SysVolt_Over;
   
   ptr->CellVolt_Over  = CheckSelf_CellVoltHigh_Charge(g_VoltInfo.CellVolt_Max);
   state = state|ptr->CellVolt_Over;
   
   ptr->CellTemp_Over  = CheckSelf_CellTempHigh_Charge(g_TempInfo.CellTemp_Max);
   state = state|ptr->CellTemp_Over;
   
   ptr->CellTemp_Low   = CheckSelf_CellTempLow_Charge(g_TempInfo.CellTemp_Min);
   state = state|ptr->CellTemp_Low;
   
   ptr->SysInsul_Flt   = CheckSelf_InsulDetect(g_IsoDetect.insulation_resist);
   state = state|ptr->SysInsul_Flt;
   
   ptr->OpenWire_Flt  = CheckSelf_OpenWireDetect(g_OpenWireInfo.OpenWire_Status);
   state = state|ptr->OpenWire_Flt;
   return state;
}

/*=======================================================================
 *������:      CheckSelf_DelayTime() 
 *����:        �Լ캯��
 *����:        time:����ʱ��
               mode:����ģʽ
 *���أ�       ��
 *˵����       BMS�Լ�����У�ֻҪBMS����״̬�ı���ô��֤�Լ�ֻ����1��
========================================================================*/
 static
 void CheckSelf_DelayTime(uint16 ts)
 {
    uint16 i,j;
    for(i=0; i<100; i++)
    {
      for(j=0; j<ts; j++);
    }
 }
 /*=======================================================================
 *������:      Checkself_BattState() 
 *����:        BMS�Լ�ǰ�����ݲɼ�������
 *����:        ��ͬ������Ƶ����Ҫ���ò�ͬ����ʱ      
 *���أ�       ��
 *˵����       ����ʱ�䵽��־λ��1��10msִ��һ��
========================================================================*/

static
void Checkself_BattState()
{ 
  Task_OpenWireDetect();          //���߿�·
  CheckSelf_DelayTime(5000);

  Task_VoltCMDSend();             //��ѹ�Ĵ�������
  CheckSelf_DelayTime(5000);

  Task_VoltCollect();             //��ѹ��ȡ����
  CheckSelf_DelayTime(5000);

  Task_TempCMDSend();             //�¶����㺯��
  CheckSelf_DelayTime(5000);

  Task_TempCollect();             //�¶Ȳɼ�����
  CheckSelf_DelayTime(5000);
  
  Task_InsulationDetect();        //������
  CheckSelf_DelayTime(1000);

  Task_DataProcess();             //���ݴ���
}
/*=======================================================================
 *������:      BMS_CheckSelf() 
 *����:        �Լ��ܺ���
 *����:        time:����ʱ��
               workmode������״̬
               sysinitstate:������ʼ��״̬      
 *���أ�       uint8:�Ƿ�����Լ����,0:����;1:����
 *˵����       ��ʼ����,���Լ�ɹ���ִ������,����һֱ�Լ�
========================================================================*/
static
uint8 CheckSelf_Process(uint8 workmode, uint8 sysinitstate)
{
   Checkself_BattState();//�Լ�������ݲɼ�    
   switch(workmode)
   {
     case MODE_DISCHARGE :
       if((CheckSelf_Discharge(&g_Flt_BMSCheckSelf) == 0) && (sysinitstate == 0)) //�����ж����ʼ��״̬0:����
       {
          return 0;
       }
     break; 
     
     case MODE_CHARGE:
       if((CheckSelf_Charge(&g_Flt_BMSCheckSelf) == 0) && (sysinitstate == 0)) //�����ж����ʼ��״̬0:����
       {
          return 0;
       }
     break; 
     
     default://���ܰ��������Լ�
     break;                                                                
   }
   return 1;
}

/*=======================================================================
 *������:      Task_BMSWorkModeCheckself() 
 *����:        �Լ캯��
 *����:        time:����ʱ��
               mode:����ģʽ
 *���أ�       ��
 *˵����       BMS�Լ�����У�ֻҪBMS����״̬�ı���ô��֤�Լ�ֻ����1��
========================================================================*/
void BMS_WorkModeCheckself(void)
{
   static uint8 workmode = 1;
   
   if(workmode != g_WorkStateJudge.WorkState)//�˴�ȷ���Լ�ֻ��״̬ת�������
   {
      workmode = g_WorkStateJudge.WorkState; 
      memset(&g_Roll_Tick, 0x00, sizeof(Roll_Tick_T)); 
      Light_Control(LED2_PORT, LED2_pin, Light_OFF);//�ر��Լ�ɹ���ʾ��
      DisablePIT(0);//��PIT0ʱ���ж�(ֻ�����Լ�)
      
      while((CheckSelf_Process(workmode, g_SysInitState.AllInit_State)!=0)\
             &&(Boot_Data.OnlineUpdateCheck == 0))//�Լ���ѯֱ���ɹ�����While,Ҳ����ֱ�ӽ�������
      {  
         BMSCheckself_UpMonitor(&g_SysInitState, &g_Flt_BMSCheckSelf);
         CheckSelf_DelayTime(100);
         if(workmode != WokeModeJudgment())        //״̬ת���������Լ�
         {
            workmode = !workmode;
            continue;
         }     
      }
      //while֮������Լ�ɹ�,�����¼�ʱ,�������Լ����
      Light_Control(LED2_PORT, LED2_pin, Light_ON);
      memset(&PIT_TimePeriod, 0x00, sizeof(PIT_TimePeriod_T));
      memset(&g_Flt_BMSCheckSelf, 0x00, sizeof(Flt_BMSCheckSelf_T));
      EnablePIT(0);//�Լ�ɹ�֮����ж�PIT0
   }
   g_WorkStateJudge.WorkState = WokeModeJudgment();
}

