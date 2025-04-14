/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Charge.C
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

BMSCharge_T        g_BMSCharge;
BMSCharge_State_T  BMSCharge_State;

/*=======================================================================
 *������:      CAN_ToChargeInit(void) 
 *����:        ����׮ͨ�ŵ�CANͨ����ʼ��
 *����:        ��       
 *���أ�       ��
 
 *˵����       
========================================================================*/ 
uint8 CAN_ToChargeInit(void)
{
  uint8 state;
  
  memset(&g_BMSCharge,      0, sizeof(BMSCharge_T));
  memset(&BMSCharge_State,  0, sizeof(BMSCharge_State_T));
  memset(&ChargePileBMS,    0, sizeof(ChargePileBMS_T));
  memset(&g_Charge_State,   0, sizeof(Charge_State_T));
 
  state = CAN1_Init(125);
  return(state);
}

/*=======================================================================
 *������:      CAN_BMSToCharge(pCANFRAME canptr) 
 *����:        BMS information to UpMonitor
 *����:        ��       
 *���أ�       ��
 *˵����       BMS������Ϣ����λ����CANͨ��ѡ��
========================================================================*/ 
static
uint8 CAN_BMSToCharge(pCANFRAME canptr)
{
  if(!CAN1_SendMsg(canptr))
  {
     return 0;  //����
  }  
  return 1;   //����
}

/*=======================================================================
 *������:      Charge_Strategy(void) 
 *����:        �����Ʋ���
 *����:        ��       
 *���أ�       ��
 *˵����       ���Ƴ��ĵ�����С�����ռ����״̬�����Ƴ�翪��
========================================================================*/ 
static
void Charge_Strategy(void) 
{
  CANFRAME BMS_to_ChargePile;
  static uint8 time=0;
  Charge_VoltCurrRequest();
  
  BMS_to_ChargePile.m_IDE = 0;//��׼֡
	BMS_to_ChargePile.m_RTR = 0;
	BMS_to_ChargePile.m_dataLen = 8;
	BMS_to_ChargePile.m_priority = 6;  
	switch(time)
	{                                                                                   
	  case 0:
      //0x111
      BMS_to_ChargePile.m_ID = 0x111;
     
      BMS_to_ChargePile.m_data[0] = (g_BMSCharge.Volt_Max_ChargePile*10) >> 8;    //���������˵�ѹ���ֽ�   0.1V/bit
    	BMS_to_ChargePile.m_data[1] = (uint8)(g_BMSCharge.Volt_Max_ChargePile*10);                              
    	BMS_to_ChargePile.m_data[2] = (g_BMSCharge.Curr_Max_ChargePile*10)>> 8;     //����������������ֽ�   �Ѿ����ֱ���(0.1A/bit)����   
    	BMS_to_ChargePile.m_data[3] = (uint8)(g_BMSCharge.Curr_Max_ChargePile*10); 
    	BMS_to_ChargePile.m_data[4] = g_BMSCharge.Control_ChargePile;               //���Ƴ��׮    0:����    1:�ر�
    	BMS_to_ChargePile.m_data[5] = (((0x00)&0x01) + \
    	                               ((BMSCharge_State.TempH_Cell<<1)&0x02) + \
    	                               ((BMSCharge_State.TempL_Cell<<2)&0x04) + \
    	                               ((BMSCharge_State.CurrH_Cell<<3)&0x08) + \
    	                               ((BMSCharge_State.Insul<<4)&0x10) + \
    	                               ((BMSCharge_State.BMSGetMsg<<5)&0x20) + \
    	                               ((BMSCharge_State.FaultFlag<<6)&0x40) + \
    	                               ((0x00)&0x80));
    	BMS_to_ChargePile.m_data[6] = 0xFF;                 
    	BMS_to_ChargePile.m_data[7] = 0xFF; 
      while(CAN_BMSToCharge(&BMS_to_ChargePile));
      time = 1;
    break;
   
	  case 1:
      //0x115
      BMS_to_ChargePile.m_ID = 0x115;
     
      BMS_to_ChargePile.m_data[0] = ((uint16)(g_BMSCharge.VoltC_Max*0.1)) >> 8;      //������ߵ�ѹ���ֽ�   
    	BMS_to_ChargePile.m_data[1] = (uint8)(g_BMSCharge.VoltC_Max*0.1);                              
    	BMS_to_ChargePile.m_data[2] = ((uint16)(g_BMSCharge.VoltC_Min*0.1)) >> 8;       //������͵�ѹ���ֽ�   
    	BMS_to_ChargePile.m_data[3] = (uint8)(g_BMSCharge.VoltC_Min*0.1); 
    	BMS_to_ChargePile.m_data[4] = (uint8)((g_BMSCharge.SOC + 0.005)*250);                        //SOC   �Ѿ����ֱ���(0.4%/bit)����
    	BMS_to_ChargePile.m_data[5] = g_BMSCharge.Temp_Max;                   //����¶�     1��/bit  ƫ����:-40��
    	BMS_to_ChargePile.m_data[6] = ((uint16)(g_BMSCharge.VoltS* 0.001)) >>8;                 //������ѹ���ֽ�     �Ѿ����ֱ��ʴ���
    	BMS_to_ChargePile.m_data[7] = (uint8)(g_BMSCharge.VoltS* 0.001);       
      while(CAN_BMSToCharge(&BMS_to_ChargePile));
      time = 0;
    break;
	}
}
/*=======================================================================
 *������:      Task_Charge(void) 
 *����:        �������
 *����:        ��       
 *���أ�       ��
 *˵����       ���ڳ��ĩ���Ǳ����������,�����״̬ת������Ҫ�������������
========================================================================*/
void Task_Charge(void)
{
  static uint8 flg;
  switch(g_WorkStateJudge.WorkState)
  {
    case MODE_CHARGE:
      Charge_Strategy();
      flg = 0;
      g_Roll_Tick.Roll_Charge++;
    break;
    
    case MODE_DISCHARGE: //������ṹ�����
      if(flg == 0) //��ֻ֤���1��
      {
        flg = 1;
        memset(&g_BMSCharge, 0x00, sizeof(BMSCharge_T));
        memset(&BMSCharge_State, 0x00, sizeof(BMSCharge_State_T));
        memset(&ChargePileBMS, 0x00, sizeof(ChargePileBMS_T));
        memset(&g_Charge_State, 0x00, sizeof(Charge_State_T));
      }
    break;
    
    default:
    break;
  }  
}






     
    
