#include  "TypeDefinition.h"
#include  "Filter_Function.h"
#include  "ADC_cfg.h"

/*=======================================================================
 *������:      FilterFunction_Ave(float*input, uint8 arrary)
 *����:        �����˲�
 *����:        float*input:ָ��ADC���������ĺ���ָ��
               arrary:��õĵ�������������10��       
 *���أ�       �˲�����
 
 *˵����       �˲�����,��10�����ڵĵ���ֵ��ƽ��ֵ
========================================================================*/
float FilterFunction_Ave(float*input, uint8 arrary)
{
  uint8 i,j,k;
  float current[10]= {0,0,0,0,0,0,0,0,0,0}; 
  float temp=0,data=0; 
  
  if((arrary>10) || (arrary<0))
  {
    return 0xFF;
  }
  
  for(i=0; i<arrary; i++) 
  {      
    current[i] = *input++;
  }
  for(k=0; k<arrary; k++) //��С��������
  {
     for(j=k+1; j<arrary; j++) 
     {
        if(current[k] > current[j])
        {
           temp = current[k];
           current[k] = current[j];
           current[j] = temp;
        }
     }
  }
  for(i=1; i<arrary-1; i++) 
  {
    data += current[i]; 
  }
  
  data = data/(arrary-2); //ȡ10�����ֵȥ�����˵�2��ֵ
  return data;
}

/*=======================================================================
 *������:      FilterFunction_Median(void) 
 *����:        
 *����:        ��       
 *���أ�       ��ֵ�˲�����
 
 *˵����       ��ֵ�˲�����
========================================================================*/
float FilterFunction_Median(float(*adc)(void), float Median) 
{
   float Cur_Filter=0,curr=0;
   
   Cur_Filter = adc();//���μ���ֵ
  
   if(abs(Cur_Filter - Median)>2)
   {
      curr = 0.9*Median + Cur_Filter*0.1;
   }
   else
   {
      curr = Cur_Filter;
   }
   return curr;     
}    