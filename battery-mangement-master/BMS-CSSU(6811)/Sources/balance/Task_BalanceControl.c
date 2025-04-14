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

#include  "Includes.h"

BalanceInfo_T balance_receive;
ToBMU_BalanceState_T ToBMU_BalanceState;
BMU_OffLineState_T BMU_OffLineState;

/*=======================================================================
 *������:      BMU_OffLineState(void)  
 *����:        ����ʱ�ж��Ƿ����
 *����:        ��       
 *���أ�       ��
 *˵����       �ھ������ǯ��Ҫ�ж������Ƿ����
========================================================================*/
static
uint8  BMU_OffLineStateJudge(void)
{
  static uint16 cnt;
  static uint8 state=0;
  if(BMU_OffLineState.BMU_Life == 1)
  {
    BMU_OffLineState.BMU_Life = 0;
    state = 0;
    cnt = 0;
  }
  else
  {
     if(++cnt*500/1000 >= 10)//10S
     {
       cnt = 0;
       state = 1; 
     }
  }
  return state;
}

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
uint8 BalanceControl_Strategy(uint8 balanceOn, uint16 voltmax, uint32 totalvolt, uint8 balacenod, uint16 balancevolt)
{
  uint8 tskstate=2;                   //����2��ʾδ���о���
  static uint16 cnt;
  uint8 balanceNum;
  
  if(1 == balanceOn)//��������Ž��г��
  {
    if((voltmax - (totalvolt/25.0)) > balancevolt)
    { 
       if(++cnt*BALANCEPERIO/1000>=3)//����2s,�����������Ƿ�����?
       {
         cnt = 3000/BALANCEPERIO;
         if(balacenod <= NUM1_Batper_true)
         {
            tskstate = LTC6811_BalanceControl(balacenod, 0x00, 0x00, 1); 
         }
         else if(balacenod <= (NUM1_Batper_true+NUM2_Batper_true))
         {
            balanceNum = balacenod-NUM1_Batper_true;
            tskstate =  LTC6811_BalanceControl(0x00, balanceNum, 0x00, 1);
         }
         else if(balacenod <= (NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true))
         {
            balanceNum = balacenod-NUM1_Batper_true-NUM2_Batper_true;
            tskstate = LTC6811_BalanceControl(0x00, 0x00, balanceNum, 1);
         }
         else
         {
            return 1;
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
     LTC6811_BalanceControl(0x00, 0x00, 0x00, 0); //δ�������������о��� 
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
   uint8 BalanceContol;
   
   BMU_OffLineState.BMU_OffLine = BMU_OffLineStateJudge();
   BalanceContol = (!BMU_OffLineState.BMU_OffLine)&balance_receive.BalanceOn;//����δ���߲����������о���
   balancestate = BalanceControl_Strategy(BalanceContol, VoltInfo.CellVolt_Max,balance_receive.total_volt,\
                                          (VoltInfo.CellVolt_MaxNode+1), 500); 
   
   if(balancestate == 0)
   {
      PTT_PTT0 = 0;//������ʾ�ƿ���
      ToBMU_BalanceState.CSSUBalanceOn   = 1;
      ToBMU_BalanceState.CSSUBalanceNode = VoltInfo.CellVolt_MaxNode+1;
   }
   else
   {
      ToBMU_BalanceState.CSSUBalanceOn   = 0;
      ToBMU_BalanceState.CSSUBalanceNode = 0;
   }
   
   Task_Flag_Counter.Counter_Balance_open++;                                       
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
   LTC6811_BalanceControl(0, 0, 0, 0);//�رվ��⹦��
   PTT_PTT0 = 1; //�رվ����
   Task_Flag_Counter.Counter_Balance_close++;
}

