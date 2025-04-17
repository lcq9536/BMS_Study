/*=======================================================================
 *Subsystem:   ���
 *File:        Task_BootLoader.C
 *Author:      Wenming
 *Description: ͨ�ţ�CAN2
               �ӿڣ�
               ������:500kbps
               ���ԣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "includes.h"
Boot_Data_T Boot_Data;

/*=======================================================================
 *������:      Boot_DelayTime(void)
 *����:        Bootloader�е���ʱ����
 *����:        us:��ʱʱ��,��λ:us       
 *���أ�       ��
 *˵����       Bootloader�е���ʱ����
========================================================================*/
void Boot_DelayTime(uint16 us) 
{
	  uint16 delayval;
	  delayval = us * 9;
	  while(delayval--);
}
/*=======================================================================
 *������:      Structure_Init(void)
 *����:        ��ʼ���ṹ��,�ṹ����������
 *����:        ��       
 *���أ�       ��
 
 *˵����       
========================================================================*/  
void Task_BootLoader() 
{       
   DisableInterrupts;
   if(Boot_Data.OnlineUpdateCheck == 1)
   {
     Boot_Data.OnlineUpdateCheck = 0;
     Boot_Data.Boot=(uint16 *)0x0C00;
     *(Boot_Data.Boot) = 0x66;
     
     Boot_DelayTime(100);
     
     COPCTL=0x01;                              // enable watchdog     
     ARMCOP=0x00; 
   }
   EnableInterrupts; 
   g_Roll_Tick.Roll_Boot++;     
}