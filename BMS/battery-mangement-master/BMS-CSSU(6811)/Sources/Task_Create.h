/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Create.H
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================*/
#ifndef TASK_CREATE_
#define  TASK_CREATE_

#define ARRAY_SIZE   11                // ���������� 

typedef struct Task_TypeDef            // ����ṹ�壻
{
    uint8 flags;                       // �����ʶ��  
    void (*handle)();                  // ���������ĺ���ָ�룻 
}TASK;


extern TASK tasks[ARRAY_SIZE];   

TASK CreateTask(uint8 FLAGS, void (*HANDLE)());
void Create_task(void);
void Task_Handle(void);

#endif