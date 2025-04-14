                                              /*=======================================================================
 *Subsystem:   ���
 *File:        ChargeToBMS.C
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
                                              
ChargePileBMS_T  ChargePileBMS;
Charge_State_T   g_Charge_State;
/*=======================================================================
 *������:      ChargePile_to_Bms(pCANFRAME data)
 *����:        ����������BMS
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
void CAN_ChargetoBMS(pCANFRAME data)
{
  ChargePileBMS.Volt_ChargePileOut = (((uint16)(data -> m_data[0]))<<8) + (data -> m_data[1]);   
  ChargePileBMS.Curr_ChargePileOut = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[3]);  

  g_Charge_State.Hard             = (data -> m_data[4])&0x01;       //���׮Ӳ������
  g_Charge_State.TempH_ChargePile = ((data -> m_data[4])>>1)&0x01;  //��������
  g_Charge_State.VoltL_ChargePile = ((data -> m_data[4])>>2)&0x01;  //���������ѹ����
  g_Charge_State.On_Line          = ((data -> m_data[4])>>3)&0x01;  //��������״̬
  g_Charge_State.GetMsg           = ((data -> m_data[4])>>4)&0x01;  //���׮����BMS��Ϣ��ʱ

  if((data -> m_data[4])&0x1F)
  {
    g_Charge_State.FltState = 1;
  }
  else
  {
    g_Charge_State.FltState = 0;
  }
   
}         

