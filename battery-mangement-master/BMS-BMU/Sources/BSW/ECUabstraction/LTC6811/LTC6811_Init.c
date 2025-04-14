/*=======================================================================
 *Subsystem:   ���
 *File:        LTC6811_Init.C
 *Author:      ZWB
 *Description: ��6811���г�ʼ��
 ========================================================================
 * History:    
 * 1. Date:    
      Author:  
      Modification:                     
           
========================================================================*/

#include  "includes.h" 
                         
uint8  CFG1[NUM_IC][6];                   // ���üĴ������� 
/*=======================================================================
 *������:      Config_Fun();
 *����:        ��LTC6804������
 *����:        ��       
 *���أ�       ��
 *˵����       IC=3;GPIO=0XFF; REFON=1; SWTEN=0;  ADCOPT=0;  VUV=3.2; VOV=4.2; DCTO=0;
========================================================================*/
void Config_Fun( uint8 total_ic,uint8 gpio, uint8 refon,uint8 swtrd, uint8 adcopt,
                 float vuv, float vov, uint16 ddc, uint8 dcto) 
{           
   uint8 char1,char2,char3;
   uint8 current_ic;                             /* ��LTC6804_1�ĳ�ʼ���������ò��� */
   
   for( current_ic = 0; current_ic<total_ic; current_ic++)
   {
      
      CFG1[current_ic][0]=(gpio<<3)+(refon<<2)+(swtrd<<1)+adcopt;
      
      CFG1[current_ic][1]=(uint8)(((uint16)vuv*10000/16-1)&0x00ff);
      
      char1=(uint8)((((uint16)vov*10000/16)&0x000f)<<4);
   
      char2=(uint8)((((uint16)vuv*10000/16-1)&0x0f00)>>8);
       
      CFG1[current_ic][2]=char1+char2;
      
      CFG1[current_ic][3]= (uint8)(((uint16)(vov*10000/16)&0x0ff0)>>4);
      
      CFG1[current_ic][4]= ddc;
      
      char3 = (uint8)(ddc>>8);
      
      char3 = char3 &0x0f; 
     
      CFG1[current_ic][5]=(dcto<<4)+ char3;    
   }
} 
/*=======================================================================
 *������:      LTC6804_Init(void)
 *����:        ��LTC6804�ĳ�ʼ��
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
uint8 LTC6804_Init(void)
{     
  uint8 ltcstate;
  ltcstate = Spi_LTC6811Init();
  
  Config_Fun(NUM_IC,DGPIO ,DREFON,DSWTRD, DADCOPT,UNDER_V, OVER_V,DDCC, DDCTW);   /* ����ֵ�����ṹ�� */ 
                                             
  LTC6804_wrcfg(NUM_IC, CFG1);                                                    /* д����оƬ���üĴ��� */
  
  set_adc(MD_NORMAL, DCP_ENABLED,CELL_CH_ALL,CELL_CHST_ALL, pup_up, chg);         /* ת��ģʽ���� */
  
  return ltcstate;
}  
