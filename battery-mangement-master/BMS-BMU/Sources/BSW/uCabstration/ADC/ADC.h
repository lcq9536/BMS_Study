/*=======================================================================
 *Subsystem:   ���
 *File:        ADC.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _ADC_H_
#define _ADC_H_

  #include"TypeDefinition.h"
  #include"MC9S12XEP100.h"
  #include"ADC_cfg.h"

  uint8 ADC_init(void);                    //AD��ʼ��
  uint16 ADC_Value(uint8 channel);
  //float ADC_Volt_Sys(void);               //��ѹ���
 
  

#endif