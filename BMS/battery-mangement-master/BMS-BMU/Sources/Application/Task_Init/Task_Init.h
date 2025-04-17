#ifndef  TASK_CREATE_
#define  TASK_CREATE_

  #include  "TypeDefinition.h"
  
  #define ARRAY_SIZE   21                // �����ջ��С�� 

  typedef struct                         // ����ṹ�壻
  {
    uint8 flags;                       // �����ʶ��  
    void (*handle)();                  // ���������ĺ���ָ�룻 
  }TASK;   

  typedef struct
  {
    uint8 Roll_Power;
    uint8 Roll_SOCSOH;
    uint8 Roll_BalanOff;
    uint8 Roll_VoltCMD;
    uint8 Roll_VoltCol;
    uint8 Roll_TempCMD;
    uint8 Roll_TempCol;
    uint8 Roll_Insul;
    uint8 Roll_DataPro;
    uint8 Roll_BalanOn;
    uint8 Roll_Currlimit;
    uint8 Roll_SysTime;
    uint8 Roll_FltJudg;
    uint8 Roll_Charge;
    uint8 Roll_FltCodeS;
    uint8 Roll_EEEWrite;
    uint8 Roll_FltCodeP;
    uint8 Roll_BMUUp;
    uint8 Roll_BMSUp;
    uint8 Roll_Screen;
    uint8 Roll_Boot;
    uint8 Roll_OpWire;
    
  }Roll_Tick_T;
  extern Roll_Tick_T g_Roll_Tick;


  void Task_Handle(void);                //������ѯ
  void Task_Init(void);                  //������Ĵ���

#endif