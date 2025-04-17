/*=======================================================================
 *Subsystem:   ���
 *File:        LTC6811_ConnectType.h
 *Author:      WenM
 *Description: �������ļ���Ҫ������ⲿ��ؽ��ߵ�����
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#ifndef _LTC6811_CONNECTTYPE_H_
#define _LTC6811_CONNECTTYPE_H_  
  
  #define NUM_pack               0   //�Ӱ�ı��(�����Ӱ��ID��)
  #define NUM_IC                 3   //�Ӱ�6811�ĸ���
  #define NUM_Tem                5   //�Ӱ���Ե��¶ȸ���
  
  //����6804�ľ�������
  //���߿�·��������
  #define NUM1_Batper            8
  #define NUM2_Batper            8  
  #define NUM3_Batper            9
  #define NUM4_Batper            0
  #define NUM5_Batper            0
  //��1��6811�ĵ�ѹ�ɼ����ӷ�ʽ
  #define NUM1_Batper_true       8 //ÿ��6804������صĸ���
  #define NUM1_Batper_front      4 //6804�е�һ��ADC1�ӵĵ�ظ���
  #define NUM1_Batper_rear       4 //6804�еڶ���ADC2�ӵĵ�ظ���
  #define NUM1_Batper_empty      (6-NUM1_Batper_front) //��һ��ADC�ճ��ĵ�ؽ���(6-NUM1_Batper_front)
  //��2��6811�ĵ�ѹ�ɼ����ӷ�ʽ
  #define NUM2_Batper_true       8
  #define NUM2_Batper_front      4
  #define NUM2_Batper_rear       4
  #define NUM2_Batper_empty      (6-NUM2_Batper_front)
  //��3��6811�ĵ�ѹ�ɼ����ӷ�ʽ
  #define NUM3_Batper_true       9
  #define NUM3_Batper_front      5
  #define NUM3_Batper_rear       4
  #define NUM3_Batper_empty      (6-NUM3_Batper_front)
  //��4��6811�ĵ�ѹ�ɼ����ӷ�ʽ
  #define NUM4_Batper_true       0
  #define NUM4_Batper_front      0
  #define NUM4_Batper_rear       0
  #define NUM4_Batper_empty      0
  //��5��6811�ĵ�ѹ�ɼ����ӷ�ʽ
  #define NUM5_Batper_true       0
  #define NUM5_Batper_front      0
  #define NUM5_Batper_rear       0
  #define NUM5_Batper_empty      0
  //һ���Ӱ���Եĵ�ѹ�ܸ���
  #define NUM_Battery      (NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true+NUM4_Batper_true+NUM5_Batper_true)

#endif