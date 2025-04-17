/*=======================================================================
 *Subsystem:   ���
 *File:        Task_FltLevJudg.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _TASK_FAULTLEVELJUDGE_H_
#define _TASK_FAULTLEVELJUDGE_H_

  #include  "TypeDefinition.h"
  

/*======================================================================
                             �����жϽṹ��
========================================================================*/
  //==========================�������ж�============================
  typedef struct
  {
    //-------------------���ϵȼ��������趨Ҫ��-----------------------
    //�����ѹ
    uint8 Level_Volt_Cell_High;               
    uint8 Level_Volt_Cell_Low; 
    uint8 Level_Volt_Cell_Diff_High;
    //ϵͳ��ѹ    
    uint8 Level_Volt_Sys_High; 
    
    //�¶�     
    uint8 Level_Temp_High;    
    uint8 Level_Temp_Low;     
    uint8 Level_Temp_Diff_High;
    
    //������   
    uint8 Level_Current_Charge_High;
    
    //��Ե���
    uint8 Level_Insul; 
 
    uint8 Level_Charge_SwitchOff_flag; //����2�����Ͽ��̵���
    
    //����
    uint8 Level_Charge_BalanceON_Flag;//���ڹ���,�����о���
           
  }Fault_Charge_T;
  extern Fault_Charge_T g_Flt_Charge;             //�������ж�
  
  //==========================�ŵ�����ж�===========================
  typedef struct
  {
    //-------------------���ϵȼ��������趨Ҫ��-----------------------
    //�����ѹ
    uint8 Level_Volt_Cell_Low; 
    uint8 Level_Volt_Cell_Diff_High;
    //ϵͳ��ѹ    
    uint8 Level_Volt_Sys_Low;
    
    //�¶�     
    uint8 Level_Temp_High;    
    uint8 Level_Temp_Low;     
    uint8 Level_Temp_Diff_High;
    
    //�ŵ����   
    uint8 Level_Current_DisCharge_High;
    
    //��Ե���
    uint8 Level_Insul; 
    
    uint8 Level_DisCharge_SwitchOff_flag; //����2�����Ͽ��̵���
    
  }Fault_DisCharge_T;
  extern Fault_DisCharge_T g_Flt_DisChg;       //�ŵ�����ж�
  
  //==============================�����ź�==============================
  typedef struct
  {
    //---------------------����״̬��0������1����----------------------
    uint8 VCU;     
    uint8 HVU;
    
    uint8 CSSU1;
    uint8 Charge;
    uint8 RelayFlt_Positive;
    /*#if(SYS_NUMBER_MODULE>=2)
      uint8 CSSU2;
    #endif
    #if(SYS_NUMBER_MODULE>=3)
      uint8 CSSU3;
    #endif
    #if(SYS_NUMBER_MODULE>=4)
      uint8 CSSU4;
    #endif
    #if(SYS_NUMBER_MODULE>=5)
      uint8 CSSU5;
    #endif
    #if(SYS_NUMBER_MODULE>=6)
      uint8 CSSU6;
    #endif */
          
  }State_Offline_T;
  extern State_Offline_T State_Offline;             //�������ж� 
  
  //==============================�����ź�==============================
  typedef struct
  {
    //CSSU
    uint8 HeartBeat_CSSU1;
    uint8 HeartBeat_Charge;
    /*#if(SYS_NUMBER_MODULE>=2)
      uint8 HeartBeat_CSSU2;
    #endif
    #if(SYS_NUMBER_MODULE>=3)
      uint8 HeartBeat_CSSU3;
    #endif
    #if(SYS_NUMBER_MODULE>=4)
      uint8 HeartBeat_CSSU4;
    #endif
    #if(SYS_NUMBER_MODULE>=5)
      uint8 HeartBeat_CSSU5;
    #endif
    #if(SYS_NUMBER_MODULE>=6)
      uint8 HeartBeat_CSSU6;
    #endif*/
    //VCU
    //uint8 HeartBeat_VCU;
    //HVU
    //uint8 HeartBeat_HVU;
  }HeartBeat_T;
  extern HeartBeat_T HeartBeat;              //�����ź�
  
/*-======================================================================
                              ��������
=========================================================================*/
 void Init_TaskFltLevJudg(void);
 void Task_FltLevJudg(void);

#endif