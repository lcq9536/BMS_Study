/*=======================================================================
 *Subsystem:   ���
 *File:        Filter_Function.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _FILTER_FUNCTION_H_
#define _FILTER_FUNCTION_H_

  float FilterFunction_Ave(float*input, uint8 arrary);    //�����˲�����,��20�����ڵĵ���ֵ��ƽ��ֵ
  float FilterFunction_Median(float(*adc)(void), float Median);                //�����˲������������˲�

#endif