/*=======================================================================
 *Subsystem:   ���
 *File:        DataFromCSSU.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _DATA_FROM_CSSU_H_
#define _DATA_FROM_CSSU_H_

  #include "TypeDefinition.h"
  #include  "BattInfoConfig.h"
  #include "CAN.h"
  
    //��ѹ��Ϣ�ṹ��
  typedef  struct
  {
    uint16  CellVolt[SYS_SERIES_YiDongLi];//�����ѹ
    uint16  CellVolt_Max;           //��������ѹ
    uint16  CellVolt_Min;           //������С��ѹ
    uint8   CellVolt_MaxNode;       //��������ѹ�ڵ�
    uint8   CellVolt_MinNode;       //������С��ѹ�ڵ�
    //uint16  CellVolt_Diff;          //�Ӱ�ѹ��
    //uint16  CellVolt_Ave;           //ֻ�����ھ������
    uint32  CSSUVolt_Total;         //�Ӱ���ѹ
    uint32  InsulVolt_Total;        //��Ե��ѹ
  }FromCSSU_Volt_T;
  extern FromCSSU_Volt_T g_FromCSSU_Volt;
  
  //�¶���Ϣ�ṹ��
  typedef  struct
  {
    uint8   CellTemp[SYS_NUMBER_MODULE_TEMP];   //�����¶�      �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_Max;           //��������¶�       �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_MaxNode;       //��������¶Ƚڵ�
    uint8   CellTemp_Min;           //��������¶�       �ֱ��ʣ�1��  ƫ������-40
    uint8   CellTemp_MinNode;       //��������¶Ƚڵ�
    uint8   CellTemp_Ave;           //����ƽ���¶�       �ֱ��ʣ�1��  ƫ������-40
    //uint8   CellTemp_Diff;          //�Ӱ��²�
  }FromCSSU_Temp_T;
  extern FromCSSU_Temp_T g_FromCSSU_Temp;

  
  typedef struct
  {
    uint8   OpenWire_Status;         //���߿�·״̬
    uint8   CSSUFlt_ChipTemp;        //�Ӱ�оƬ�¶ȹ���
    uint8   CSSU_BalanceState;       //�Ӱ�ľ���״̬,00:δ����
  }FromCSSU_FltData_T;
  extern FromCSSU_FltData_T g_FromCSSU_FltData;

  
  void DataFromCSSU(pCANFRAME data);

#endif