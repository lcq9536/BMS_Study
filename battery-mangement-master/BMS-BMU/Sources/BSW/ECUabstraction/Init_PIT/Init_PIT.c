/*=======================================================================
 *Subsystem:   ���
 *File:        Init_PIT.C
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
/*=======================================================================
 *������:      Init_PIT(uint8 channel, uint8 perio)
 *����:        ��ʱ����ʼ������
 *����:        channel:ͨ��
               perio:����(ֻ��ѡ��1,5,10)      
 *���أ�       ��
 
 *˵����       ʱ
========================================================================*/
uint8 Init_PIT(void)
{
   PIT0_Init();
}




/*uint8 Init_PIT(uint8 channel, uint8 perio)      
{
  if(channel>7)
  {
    return (Init_Fault_PIT_Channel);
  }
  if((perio==1)||(perio==5)||(perio==10))
  {
    switch (perio)
    {
      case 1:
        PITInit(channel, 0, 249, 129);
      break;
      
      case 5:
        PITInit(channel, 0, 249, 639);
      break;
      
      case 10:
        PITInit(channel, 0, 249, 1279);
      break;
    }
  }
  else
  {
     return Init_Fault_PIT_Period;
  }
  return (Init_Normal_PIT);
}*/