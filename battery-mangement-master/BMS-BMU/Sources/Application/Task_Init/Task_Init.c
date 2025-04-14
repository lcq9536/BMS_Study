/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Init.C
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

TASK tasks[ARRAY_SIZE];            //�����ջ���  

Roll_Tick_T g_Roll_Tick;   
/*=======================================================================
 *������:      Task_Init(uint8 FLAGS, void (*HANDLE)())  
 *����:        ���񴴽�����
 *����:        ��       
 *���أ�       ��
 *˵����       ÿ�����񴴽���Ҫһ��ʱ���ʱ�ı�־λ�뺯��ָ�룻
========================================================================*/
static
TASK Task_Register(uint8 FLAGS, void (*HANDLE)(void)) // ����������ṹ�庯����
{
	TASK task;
	task.flags = FLAGS;
	task.handle = HANDLE;
	return (task);
} 

/*=======================================================================
 *������:      Task_Handle(void)  
 *����:        ������ѯ
 *����:        ��       
 *���أ�       ��
 *˵����       �����������麯����������ѯִ������
========================================================================*/
void Task_Handle(void)// ����ִ�к�����������Ĵ���˳��ִ������                     
{
  uint8 i;
  for (i = 0; i < ARRAY_SIZE; i++)
  {   
     if (tasks[i].flags==1)                    // ��ѯ����ʱ���Ƿ񵽣�
     {             
        (*tasks[i].handle)();                  // ִ������
        tasks[i].flags = 0;                    // ����������λ              
     }        
  }
}
   
/*=======================================================================
 *������:      Task_Create(void)  
 *����:        �����񴴽�����
 *����:        ��       
 *���أ�       ��
 *˵����       �����������麯����������ѯִ������
========================================================================*/  
void Task_Init()                                      
{                   
  //BMS���ݽ��ռ���������     
  tasks[0] = Task_Register(0, Task_BalanceControl_OFF);   //�رվ������       
  tasks[1] = Task_Register(0, Task_PowerOnOff);           //BMS�Լ켰���µ�Ŀ���    
  tasks[2] = Task_Register(0, Task_SOCSOH);               //SOCSOH���㺯��
  tasks[3] = Task_Register(0, Task_VoltCMDSend);          //�ɼ���ѹǰ���͵�����
  tasks[4] = Task_Register(0, Task_VoltCollect);          //��ѹ�ɼ�
  tasks[5] = Task_Register(0, Task_TempCMDSend);          //�ɼ��¶�ǰ���͵�����
  tasks[6] = Task_Register(0, Task_TempCollect);          //�¶Ȳɼ� 
  tasks[7] = Task_Register(0, Task_InsulationDetect);     //��Ե��� 
  tasks[8] = Task_Register(0, Task_DataProcess);          //���ݴ���

  tasks[9] = Task_Register(0, Task_BalanceControl_ON);    //�����������(�������Ʋ���)

  tasks[10] = Task_Register(0, Task_CurrLimit);           //�������ƺ���
  tasks[11] = Task_Register(0, Task_SysTimeGet);          //ϵͳʱ��Ļ�ȡ/ϵͳ����ʱ��ļ���

  tasks[12] = Task_Register(0, Task_FltLevJudg);          //���ϵȼ��ж� 

  tasks[13] = Task_Register(0, Task_Charge);              //�������
  tasks[14] = Task_Register(0, Task_FltCodeStore);        //���ϴ���洢 
  tasks[15] = Task_Register(0, Task_EEpromWrite);         //����ֵ�Ĵ洢
  tasks[16] = Task_Register(0, Task_FltCodeProcess);      //���ϴ����ϴ�

  tasks[17] = Task_Register(0, Task_BMUToUpMonitor);      //�ϴ�����ɼ���ѹ�¶�������Ϣ    
  tasks[18] = Task_Register(0, Task_BMSToUpMonitor);      //�ϴ�����Ĺ�����Ϣ���� 

  tasks[19] = Task_Register(0, Task_ScreenTransfer);      //��ʾ��������,3S����
  tasks[20] = Task_Register(0, Task_BootLoader);          //Bootloader����
}

/*=======================================================================
 *������:      Task_Create(void)  
 *����:        �����񴴽�����
 *����:        ��       
 *���أ�       ��
 *˵����       �����������麯����������ѯִ������
========================================================================*/
void Task_Roll(void)
{
  static uint8 cnt[5]={0,0,0,0,0};
  switch(PIT_TimePeriod.T500ms)//500ms������
  {
    case 1:
      tasks[0].flags = 1;//Task_BalanceControl_OFF
    break;
      
    case 3:   
      tasks[1].flags = 1;//Task_PowerOnOff
    break;
    
    case 5: //SOCSOH�ļ�������Ϊ100ms
    case 15:
    case 25:
    case 35:
    case 45:
      tasks[2].flags = 1;//Task_SOCSOH
    break;
    
    case 7:
      tasks[3].flags = 1;//Task_VoltCMDSend
    break;
    
    case 9:
      tasks[4].flags = 1;//Task_VoltCollect
    break;
    
    case 11:
      tasks[5].flags = 1;//Task_TempCMDSend
    break;
    
    case 13:
      tasks[6].flags = 1;//Task_TempCollect
    break;
      
    case 17:
      tasks[7].flags = 1;//Task_InsulationDetect
    break;
    
    case 21:
      
      tasks[8].flags = 1;//Task_DataProcess
    break;
    
    case 23:
      tasks[9].flags = 1;//Task_BalanceControl_ON
    break;
    
    case 27:
      tasks[10].flags = 1;//Task_CurrLimit
    break; 
    
    case 29:
      if(cnt[4]%10 == 0)  //10*0.5S���� 
      {
        cnt[4] = 0;
        tasks[11].flags = 1;//Task_SysTimeGet
      }
      cnt[4]++;
    break;
    
    case 31:
      tasks[12].flags = 1;//Task_FltLevJudg
    break;
    
    case 33:
      tasks[13].flags = 1;//Task_Charge
    break;
    
    case 37:
      tasks[14].flags = 1;//Task_FltCodeStore
    break;

    case 39:
      tasks[16].flags = 1;//Task_FltCodeProcess
    break;
      
    case 40:
      tasks[20].flags = 1;//Task_BootLoader
      
    break;

    case 41:
      if(cnt[0]%40 == 0)//40*0.5S���� 
      {
        cnt[0] = 0;
        tasks[15].flags = 1;//Task_EEpromWrite
      }
      cnt[0]++;
    break;
    
    case 43:
      if(cnt[1]%2 == 0)//2*0.5S���� 
      {
        cnt[1] = 0;
        tasks[17].flags = 1;//Task_BMUToUpMonitor
      }
      cnt[1]++;
    break;
      
    case 47:
      if(cnt[2]%2 == 0)//2*0.5S���� 
      {
        cnt[2] = 0;
        tasks[18].flags = 1;//Task_BMSToUpMonitor
      }
      cnt[2]++;
    break;
    
    case 49:
      if(cnt[3]%6 == 0)//6*0.5S���� 
      {
        cnt[3] = 0;
        tasks[19].flags = 1;//Task_ScreenTransfer
      }
      cnt[3]++;
    break; 
    
    default:
    break;
  }   
}
 
