/*=======================================================================
 *Subsystem:   ���
 *File:        ADC.h
 *Author:      WenMing
 *Description: �ӿ�
               ��ѹ���PPAD0
               ������������PAD03
               ��������PAD10
               ����Ƶ�ʣ�2MHz                
========================================================================
 * History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ���
 * 1. Date:
      Author:
      Modification:
========================================================================*/


#ifndef _ADC_H
#define _ADC_H

#include"Includes.h"
  
 
  void ADC_init(void);                    //AD��ʼ��
  uint16 ADCValue(uint8 channel);         //AD�ɼ�ͨ������
    



#endif