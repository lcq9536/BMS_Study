/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Create.C
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#include  "Includes.h"                 /* ��������h�ļ������Ե������к�������� */ 

TASK tasks[ARRAY_SIZE];                /* �������� */

/*=======================================================================
 *������:      CreateTask(uint8 FLAGS, void (*HANDLE)())  
 *����:        ���񴴽�����
 *����:        ��       
 *���أ�       ��
 *˵����       ÿ�����񴴽���Ҫһ��ʱ���ʱ�ı�־λ�뺯��ָ�룻
========================================================================*/
TASK CreateTask(uint8 FLAGS, void (*HANDLE)())            
{
  	TASK task;
  	
  	task.flags = FLAGS;                  /* �����־λ */            
  	task.handle = HANDLE;                /* ����ָ����������� */
  	return (task);
}   
/*=======================================================================
 *������:      Create_task(void)  
 *����:        �����񴴽�����
 *����:        ��       
 *���أ�       ��
 *˵����       �����������麯����������ѯִ�������࣬A����600msִ��һ�Σ�
========================================================================*/ 
void Create_task(void)                                          
{  
   
   tasks[0] = CreateTask( 0 ,Task_Volt_Collect);       /* A01 ��ѹ�ռ� */
   
   tasks[1] = CreateTask( 0 ,Task_Volt_Process);       /* A02 ��ѹ���� */
     
   tasks[2] = CreateTask( 0 ,Task_Pack_Temp_Collect);  /* A03 Pack�¶��ռ� */
   
   tasks[3] = CreateTask( 0 ,Task_Pack_Temp_Process);  /* A04 Pack�¶ȴ��� */
   
   tasks[4] = CreateTask( 0 ,Task_Chip_Temp_Collect);  /* A05 оƬ�¶��ռ� */ 
        
   tasks[5] = CreateTask( 0 ,Task_Chip_Temp_Process);  /* A06 оƬ�¶ȴ��� */ 
   
   tasks[6] = CreateTask( 0 ,Task_BalanceControl_OFF);       /*  �رվ��� */
   
   tasks[7] = CreateTask( 0 ,Bms_to_Up_Monitor);       /* A07 ���߿�·��� */
   
   tasks[8] = CreateTask( 0 ,InsulationDetect);       /* A07 ���߿�·��� */
   
   tasks[9] = CreateTask( 0 ,Task_BalanceControl_ON);       /*  �򿪾��� */
   
   tasks[10] = CreateTask( 0 ,Task_BootLoader);       /*  �򿪾��� */
}

/*=======================================================================
 *������:      Task_Handle(void)
 *����:        ������ѯ����
 *����:        ��       
 *���أ�       ��
 *˵����       ���������в��ϵĲ�������ʱ���Ƿ��ˣ�������˾�ִ�С�
========================================================================*/
void Task_Handle(void)                                                 
{
    uint8 i;
    
    for (i = 0; i < ARRAY_SIZE; ++i)   // ��ѯ��������;
    {
       
       if (tasks[i].flags==1)          // ��ѯ�����־λ�Ƿ���1����1��ִ��;
       {
            (*tasks[i].handle)();         // ִ������; ʹ�ú���ָ��;
            tasks[i].flags = 0;        // �������־λ����;
       }   
    }
    Task_Flag_Counter.Time_Handle++;
}