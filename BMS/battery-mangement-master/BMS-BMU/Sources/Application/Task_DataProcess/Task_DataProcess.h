 /*=======================================================================
 *Subsystem:   ���
 *File:        Task_DataProcess.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef  _TASK_DATA_PROCESS_H_
#define  _TASK_DATA_PROCESS_H_

  #include "TypeDefinition.h"  
  #include "BattInfoConfig.h"   

  #define NUM_OPENWIRE   6

  //��ѹ��Ϣ�ṹ��
  typedef  struct
  {
    uint16  CellVolt[SYS_SERIES];    //�����ѹ
    uint16  CellVolt_Max;           //��������ѹ
    uint16  CellVolt_Min;           //������С��ѹ
    uint8   CellVolt_MaxNode;       //��������ѹ�ڵ�
    uint8   CellVolt_MinNode;       //������С��ѹ�ڵ�
    uint16  CellVolt_Diff;          //����ѹ��
    uint16  CellVolt_Ave;           //ֻ�����ھ������
    uint32  SysVolt_Total;          //ϵͳ��ѹ
  }VoltInfo_T;
  extern VoltInfo_T g_VoltInfo;

  //�¶���Ϣ�ṹ��
  typedef  struct
  {
    uint8   CellTemp[SYS_NUMBER_TEMP];   //�����¶�      �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_Max;           //��������¶�       �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_MaxNode;       //��������¶Ƚڵ�
    uint8   CellTemp_Min;           //��������¶�       �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_MinNode;       //��������¶Ƚڵ�
    uint8   CellTemp_Ave;           //����ƽ���¶�       �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_Diff;
  }TempInfo_T;
  extern TempInfo_T g_TempInfo;


  //��Ե��Ϣ�ṹ��
  typedef  struct
  {
    uint16  Insul_Resis_Pos;          //��Ե���Եص���
    uint16  Insul_Resis_Neg;          //��Ե���Եص���
    uint16  Insul_Resis;              //��Ե����,ȡ��/���Եص����е���Сֵ
    uint8   Insul_FaultGrade;         //��Ե���ϵȼ�
    uint8   Insul_Life;               //��Ե�ź�
    uint16  Insul_Volt;               //��Ե��ѹ
  }InsulInfo_T;
  extern InsulInfo_T g_InsulInfo;


  //���߿�·
  typedef struct
  {
    uint8   OpenWire_Status;             //���߿�·״̬
    uint16  OpenWire_Node[NUM_OPENWIRE]; //���߿�·�ڵ�
  }OpenWireInfo_T;
  extern OpenWireInfo_T g_OpenWireInfo;

  //�ɼ�������
  typedef struct
  {
    //uint32  SysVolt_Total;             //ϵͳ��ѹ�ܺ�    �ֱ���:0.0001V
    //uint16  InsulVolt_Total;
    uint8   CSSU_Flt_TempH;
    float   DataCollet_Current_Filter;   //�˲������ĵ���
  }DataColletInfo_T;
  extern DataColletInfo_T g_DataColletInfo;
    
  void Init_TaskDataProcess(void);
  void Task_DataProcess(void);

#endif
