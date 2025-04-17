 /*=======================================================================
 *Subsystem:   ���
 *File:        WorkModeJudge.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#ifndef _WORK_MODE_JUDGE_H_
#define _WORK_MODE_JUDGE_H_ 
  
  #include  "TypeDefinition.h"
/*=============================================================
                            ����״̬����
==============================================================*/
  #define           MODE_DISCHARGE          0     //�ŵ�
  #define           MODE_CHARGE             1     //���
  #define           MODE_SLOW_CHARGE        2     //���� 
  
  #define           QCharge_CC2             9      
  
  typedef struct
  {
    uint8  WorkState;                       // ����״̬
    //uint8  WorkState_EnterTime_DisCharge;   // �ŵ�״̬�����ʱ   
    //uint8  WorkState_EnterTime_Charge;      // ���״̬�����ʱ     
    //uint8  WorkState_EnterFlag;             // ����ŵ�����м�¼
  }WorkStateJudge_T;
  extern WorkStateJudge_T  g_WorkStateJudge;
  
  uint8 WokeModeJudgment(void);
#endif