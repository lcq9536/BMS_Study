 /*=======================================================================
 *Subsystem:   ���
 *File:        BMSCheckself_UpMonitor.C
 *Author:      Wen Yuhao
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#include  "includes.h"

/*=======================================================================
 *������:      PhysicInitState_FltCode(SysInitState_T*) 
 *����:        ������ʼ����״̬�ж�
 *����:        ptr:ָ��������ʼ��״̬����      
 *���أ�       uint16:����������ʼ��״̬,0:�����޹���;����:���ϴ���
 *˵����       ������ʼ����״̬�ж�
========================================================================*/
static
uint16 PhysicInitState_FltCode(SysInitState_T*ptr)
{
  uint16 fltcode=0;
  
  fltcode = (ptr->ADC)|(ptr->IIC<<1)|(ptr->PIT0<<2)|(ptr->PLL<<3)|(ptr->Relay_Positvie<<4)|\
            (ptr->EEPROM<<5)|(ptr->CAN1<<6)|(ptr->CAN2<<7)|(ptr->Screen<<8)|(ptr->SPI<<9)|\
            (ptr->Insul<<10);  
  return fltcode;
}
/*=======================================================================
 *������:      Level2_FltCode(Flt_BMSCheckSelf_T*) 
 *����:        ��ʼ����2���ϼ̵������ϵ�״̬�ж�
 *����:        ptr:ָ��2�����ϵ�״̬����      
 *���أ�       uint16:����2�����ϵ�״̬,0:�����޹���;����:���ϴ���
 *˵����       ��ʼ����2���ϼ̵������ϵ�״̬�ж�
========================================================================*/
static
uint16 Level2_FltCode(Flt_BMSCheckSelf_T*ptr)
{
   uint16 fltcode=0;
   
   fltcode = (ptr->SysVolt_Over)|(ptr->SysVolt_Low<<1)|(ptr->CellVolt_Low<<2)|(ptr->CellVolt_Over<<3)|\
             (ptr->CellTemp_Low<<4)|(ptr->CellTemp_Over<<5)|(ptr->SysCurr_Over<<6)|\
             (ptr->SysInsul_Flt<<7)|(ptr->OpenWire_Flt<<8);
   
   return fltcode;
}

/*=======================================================================
 *������:      BMSCheckself_UpMonitor(Flt_BMSCheckSelf_T*, SysInitState_T*) 
 *����:        ���Լ�һֱ��ͨ��ʱ�����͹��ϴ��뵽��λ����
 *����:        lev2:ָ��2�����ϵ�״̬���� 
               sysint:ָ��������ʼ��״̬����
 *���أ�       ��
 *˵����       ���Լ�һֱ��ͨ��ʱ�����͹��ϴ��뵽��λ����
========================================================================*/
void BMSCheckself_UpMonitor(SysInitState_T*sysint, Flt_BMSCheckSelf_T*lev2)
{
  CANFRAME BMS_to_Upmonitor;
  uint8 CANstate;
  uint16 sysfltcode=0,lev2fltcode=0;
  
  BMS_to_Upmonitor.m_IDE = 1;
	BMS_to_Upmonitor.m_RTR = 0;
	BMS_to_Upmonitor.m_dataLen = 6;
	BMS_to_Upmonitor.m_priority = 6;
	//0x1820C0F4 
  BMS_to_Upmonitor.m_ID = 0x1820C0F4;
	
	sysfltcode  = PhysicInitState_FltCode(sysint);
	lev2fltcode = Level2_FltCode(lev2);    
  
  BMS_to_Upmonitor.m_data[0] = 0x02;    //��� 0x02
  BMS_to_Upmonitor.m_data[1] = 0x00;    //����Լ�״̬  00 �Լ��У�01 �ɹ���02 ʧ��

  BMS_to_Upmonitor.m_data[2] = (uint8)(sysfltcode&0xFF);     
  BMS_to_Upmonitor.m_data[3] = (uint8)(sysfltcode>>8);     
  
  BMS_to_Upmonitor.m_data[4] = (uint8)(lev2fltcode&0xFF);     
  BMS_to_Upmonitor.m_data[5] = (uint8)(lev2fltcode>>8);
  //BMS_to_Upmonitor.m_data[6] = 0xFF;//Ԥ��
  //BMS_to_Upmonitor.m_data[7] = 0xFF;//Ԥ��
   
  CANstate = CAN_ToUpMonitor(&BMS_to_Upmonitor);
}


