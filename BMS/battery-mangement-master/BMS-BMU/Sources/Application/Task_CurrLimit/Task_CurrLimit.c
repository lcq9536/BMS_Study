/*=======================================================================
 *Subsystem:   ���
 *File:        Task_CurrLimit.C
 *Author:      Wenming
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
 CurrLimit_T CurrLimit;

 static float ChagCurrLimit_Cons(uint8 Temp);
 
 /*=======================================================================
 *������:      Init_TaskCurrLimit
 *����:       
 *����:        ��                                     
 *���أ�       ��
 *˵����       
========================================================================*/ 
void Init_TaskCurrLimit(void) 
{
  //�������Ʊ�������
  memset(&CurrLimit,     0, sizeof(CurrLimit_T));//����������������

}
 
/*=======================================================================
 *������:      Task_CurrLimit
 *����:        ��ŵ�����ж�
 *����:        ��                                     
 *���أ�       ��
 *˵����       BMS�����¶������ж���������������Ĵ�С
========================================================================*/ 
void Task_CurrLimit(void)
{
  //����������
  if(g_WorkStateJudge.WorkState == MODE_CHARGE)
  {
    CurrLimit.Curr_Charge_Cons = ChagCurrLimit_Cons(g_TempInfo.CellTemp_Ave); 
  }
  else
  {
    CurrLimit.Curr_Charge_Cons = 0;
  }
    
  g_Roll_Tick.Roll_Currlimit++;

} 

/*=======================================================================
 *������:      ChagCurrLimit_Cons(uint8 Temp)
 *����:        ��������������
 *����:        Temp:�¶�                                     
 *���أ�       Current:���Ƶ���
 *˵����       BMS�����¶������ж���������������Ĵ�С
========================================================================*/ 
static
float ChagCurrLimit_Cons(uint8 Temp)
{
  static float Current;
  
  if(Temp>=(0 +40) && Temp<=(55 +40))
  {
    if(Temp>(50 +40))
    {
      Current = 100;
    }
    else if(Temp>(10 +40))
    {
      Current = 150;
    }
    else if(Temp>(5 +40))
    {
      Current = 100;
    }
    else 
    {
      Current = 50;
    }
  }
  else
  {
    Current = 0;
  }
  return Current;
}