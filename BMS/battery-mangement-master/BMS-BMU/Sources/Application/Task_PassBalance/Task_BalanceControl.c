/*=======================================================================
 *Subsystem:   ���
 *File:        Task_BalanceControl.C
 *Author:      WenM
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:   
      Author:  
      Modification:  
      
========================================================================*/
#include  "includes.h"

PassiveBalance_T g_PassiveBalance;
/*=======================================================================
 *������:      BalanceControl_Strategy(flaot, uint8, uint16, uint32, uint8, float)
 *����:        �Ե������б�������
               ������Ʋ���:����������ƽ����ѹbalancevoltʱ���о���
 *����:        curr���жϵ����Ĵ�С,ֻ������С��3A(��̬)���о���
               faultflg:  �ڲ����ֹ��ϵ�����²Ž��о���
               voltmax��  ����ѹ
               totalvolt��25����ص���ѹ,������ƽ����ѹ
               balacenod: ����Ľڵ�                 
 
 *���أ�       uint8:     0��ʾ��������
                          1��ʾ������ִ��������ֹͣ����
                          2��ʾδ�ﵽ�������������о���
 *˵����       �������⺯��
========================================================================*/
static
uint8 BalanceControl_Strategy(float curr, uint8 BalanceOn, uint16 voltmax, uint32 totalvolt, uint8 balacenod, uint16 balancevolt)
{
  uint8 tskstate=2;                   //����2��ʾδ���о���
  static uint16 cnt;
  uint8 balanceNum;
  
  if(abs(curr)>=10 && (BalanceOn == 1) && (g_WorkStateJudge.WorkState == MODE_CHARGE)) //ֻ���ڳ��Ĺ����е�������5A�ſ���
  {  
    if((voltmax - (totalvolt/25.0)) > balancevolt)
    {  
       if(++cnt*BALANCEPERIO/1000>=3)//��״̬����2S
       {
         cnt=3000/BALANCEPERIO;  
         if(balacenod <= NUM1_Batper_true)//8
         {   
            tskstate   = LTC6811_BalanceControl(balacenod, 0x00, 0x00, 1); 
         }
         else if(balacenod <= (NUM1_Batper_true+NUM2_Batper_true)) //8+8
         {   
            balanceNum = balacenod-NUM1_Batper_true;
            tskstate   = LTC6811_BalanceControl(0x00, balanceNum, 0x00, 1);
         }
         else if(balacenod <= (NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true))//8+8+9
         {  
            balanceNum = balacenod-NUM1_Batper_true-NUM2_Batper_true;
            tskstate   = LTC6811_BalanceControl(0x00, 0x00, balanceNum, 1);
         }
         else
         {  
            return 1; //����ڵ����
         }
       }
    }
    else
    {
      cnt = 0;
    }
  }
  else
  {  
     tskstate = LTC6811_BalanceControl(0x00, 0x00, 0x00, 0); //δ�������������о��� 
     return 2;
  }
  return tskstate;
}

/*=======================================================================
 *������:      Task_BalanceControl_ON(void)
 *����:        ���⿪����������
 *����:        ��       
 *���أ�       ��
 *˵����       ���ݾ�����Ʋ��Կ����������,ֻ�ڵ�ѹ�ɼ����֮��������
========================================================================*/
void Task_BalanceControl_ON(void)
{
   uint8 balancestate;
   
   balancestate = BalanceControl_Strategy(g_DataColletInfo.DataCollet_Current_Filter, g_Flt_Charge.Level_Charge_BalanceON_Flag,\
                                          g_LTC6811_VoltInfo.CellVolt_Max, g_LTC6811_VoltInfo.CellVolt_Total, (g_LTC6811_VoltInfo.CellVolt_MaxNode+1), 500);
   
   if(balancestate == 0)//����������
   {  
      Light_Control(LED1_PORT, LED1_pin, Light_ON);
      g_PassiveBalance.BalanceNode = g_LTC6811_VoltInfo.CellVolt_MaxNode+1;
      g_PassiveBalance.BalanceOn   = 1;
   }
   else//������
   { 
      g_PassiveBalance.BalanceNode = 0;
      g_PassiveBalance.BalanceOn   = 0;
   }
   g_Roll_Tick.Roll_BalanOn++;
}
/*=======================================================================
 *������:      Task_BalanceControl_OFF(void)
 *����:        ����رտ�������
 *����:        ��       
 *���أ�       ��
 *˵����       �ڵ�ѹ�ɼ�֮ǰ�رվ��⿪��
========================================================================*/
void Task_BalanceControl_OFF(void)
{
   LTC6811_BalanceControl(0, 0x00, 0x00, 0);//�رվ��⹦��
   Light_Control(LED1_PORT, LED1_pin, Light_OFF);
   g_Roll_Tick.Roll_BalanOff++;
}