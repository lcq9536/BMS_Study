/*=======================================================================
 *Subsystem:   ���
 *File:        Task_PowerOnOff.c
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:  
========================================================================*/
#include  "includes.h"
/*=======================================================================
 *������:      PositiveRelay_Init
               CSSUPowerRelay_Init
               ScreenPowerRelay_Init
 *����:        �ֱ�������̵�����CSSU��Դ���ء���ʾ�����صĳ�ʼ��
 *����:        switchstate:����״̬(Relay_OFF:���ع�,Relay_ON:���ؿ�)
               
 *���أ�       ��
 *˵����       �ֱ�������̵�����CSSU��Դ���ء���ʾ�����صĳ�ʼ��
========================================================================*/
uint8 Init_Relay(void)//�����̵���
{
  uint8 state;
  
  //����״̬���λ����
  memset(&g_WorkStateJudge,     0, sizeof(WorkStateJudge_T));    //������״̬����
  
  //�Լ���������
  memset(&g_Flt_BMSCheckSelf,   0, sizeof(Flt_BMSCheckSelf_T));     
    
  state = Port_Init(Relay_Positive_PORT, Relay_Positive_pin, Relay_OFF);
  state = state | Port_Init(Relay_CSSUPower_PORT, Relay_CSSUPower_pin, Relay_ON);
  state = state | Port_Init(Relay_ScreenPower_PORT, Relay_ScreenPower_pin, Relay_ON);
  state = state | Light_Init(LED1_PORT, LED1_pin, Light_OFF);//�����⾯ʾ��
  state = state | Light_Init(LED2_PORT, LED2_pin, Light_OFF);//�Լ�ɹ���
  return state;
}
/*=======================================================================
 *������:      PositiveRelay_Control
               CSSUPowerRelay_Control
               ScreenPowerRelay_ON
 *����:        �ֱ�������̵�����CSSU��Դ���ء���ʾ�����صĿ���
 *����:        switchstate:����״̬(Relay_OFF:���ع�,Relay_ON:���ؿ�)
               
 *���أ�       ��
 *˵����       �ֱ�������̵�����CSSU��Դ���ء���ʾ�����صĿ���
========================================================================*/
void PositiveRelay_Control(uint8 switchstate)//�����̵���
{
  switch(switchstate)
  {
    case Relay_ON:
      if(Port_StateGet(Relay_Positive_PORT, Relay_Positive_pin) == Relay_OFF)
      {
         Port_Control(Relay_Positive_PORT, Relay_Positive_pin, Relay_ON);
      }
      break;
    
    case Relay_OFF:
      if(Port_StateGet(Relay_Positive_PORT, Relay_Positive_pin) == Relay_ON)
      {
         Port_Control(Relay_Positive_PORT, Relay_Positive_pin, Relay_OFF);
      }
      break;
    
    default:
      break; 
  }

}
//CSSU��Դ���� 
static
void CSSUPowerRelay_Control(uint8 switchstate)
{
  switch(switchstate)
  {
    case Relay_ON:
      if(Port_StateGet(Relay_CSSUPower_PORT, Relay_CSSUPower_pin) == Relay_OFF)
      {
        Port_Control(Relay_CSSUPower_PORT, Relay_CSSUPower_pin, Relay_ON); 
      }
      break;   
    
    case Relay_OFF:
      if(Port_StateGet(Relay_CSSUPower_PORT, Relay_CSSUPower_pin) == Relay_ON)
      {
        Port_Control(Relay_CSSUPower_PORT, Relay_CSSUPower_pin, Relay_OFF); 
      }
      break;
      
    default:
      break; 
  }
}
//��ʾ����Դ���� 
static
void ScreenPowerRelay_Control(uint8 switchstate)
{
  switch(switchstate)
  {
    case Relay_ON:
      if(Port_StateGet(Relay_ScreenPower_PORT, Relay_ScreenPower_pin) == Relay_OFF)
      {
        Port_Control(Relay_ScreenPower_PORT, Relay_ScreenPower_pin, Relay_ON); 
      }
      break;   
    
    case Relay_OFF:
      if(Port_StateGet(Relay_ScreenPower_PORT, Relay_ScreenPower_pin) == Relay_ON)
      {
        Port_Control(Relay_ScreenPower_PORT, Relay_ScreenPower_pin, Relay_OFF); 
      }
      break;
      
    default:
      break; 
  }
}

/*=======================================================================
 *������:      PowerOnOff_Control
 *����:        ���µ�Ŀ���
 *����:        FltLevel:���ϵȼ�Ϊ2ʱ�����ж�
               FltSeconds:����2��������ʱ��ʱ��,��λ:s
               StaticCurrSet:�����������ߵľ�̬������С����
               StaticTimeSet:�����������ߵľ�̬ʱ���С����
 *���أ�       ��
 *˵����       ������������������CSSU����ʾ�����ص�״̬
========================================================================*/
static
void PowerOnOff_Control(uint8 FltLevel, uint8 FltSeconds, uint8 StaticCurrSet, uint8 StaticTimeSet)
{
   static uint16 cnt;
   static uint8  relaystate;
   BMS_WorkModeCheckself();            //�Լ�Ϊ��ѭ��,�Լ�ɹ�֮����ܽ������µ�
   
   if(FltLevel == 1)//�����������
   {  
      if((++cnt)*PEWERONOFF_PERIO/1000.0>FltSeconds) //��ʱTimes��֮��Ͽ��̵���
      {
        cnt = 0;
        PositiveRelay_Control(Relay_OFF); 
      }
   }
   else//��2������,ֻҪ�������С��2A�������伴����������״̬
   {  
      cnt = 0;
      //��������̬(����С��2Aʱ)����̵����رյ�״̬
      if(Sleep_StaticTime(Read_IIC_Time.IIC_Read_Hour, Read_IIC_Time.IIC_Read_Minute,\
         g_DataColletInfo.DataCollet_Current_Filter, StaticCurrSet, StaticTimeSet))
      {
         PositiveRelay_Control(Relay_OFF);
         CSSUPowerRelay_Control(Relay_OFF);
         ScreenPowerRelay_Control(Relay_OFF);
      }
      else
      {
         if(relaystate == 0)
         {
            relaystate = 1;
            PositiveRelay_Control(Relay_ON); //�ϵ�̵���ֻ����1��
         }
      }
   }    
}
/*=======================================================================
 *������:      Task_PowerOnOff
 *����:        ���µ�Ŀ���
 *����:        ��
 *���أ�       ��
 *˵����       �趨����2�����Ϻ�Ͽ��̵�������ʱʱ��Ϊ:30s
               �趨BMS�������������ľ�̬����Ϊ:2A
               �趨BMS�������ߵľ�̬ʱ��Ϊ:12h                
========================================================================*/
void Task_PowerOnOff(void)
{
   PowerOnOff_Control(g_Flt_DisChg.Level_DisCharge_SwitchOff_flag|g_Flt_Charge.Level_Charge_SwitchOff_flag,\
                      30, 5, 1);

   g_Roll_Tick.Roll_Power++;
}


