/*=======================================================================
 *Subsystem:   ���
 *File:        CAN2.C
 *Author:      WenYuhao
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
 *������:      CAN2_Init(void)
 *����:        ��ʼ��CAN2
 *����:        
               sysclk��ʱ������ʱ��:32MHz
               baud:    
 *���أ�       ��
 *˵����       
========================================================================*/
uint8 CAN2_Init(uint16 Baud_Rate) 
{
  uint16 CAN2cnt[3] = {0,0,0};
  
  if((Baud_Rate != 125)&&(Baud_Rate != 250)&&(Baud_Rate != 500))
  {
    return(Init_Fault_CAN_BaudRate);
  }
  
  if(CAN2CTL0_INITRQ==0)       // ��ѯ�Ƿ�����ʼ��״̬   
  {
    CAN2CTL0_INITRQ = 1;       // �����ʼ��״̬
  }
  
  do
  {
    if(++CAN2cnt[0]>3000)
    {
      return(Init_Fault_CAN_Unready1);
    }
  }
  while (CAN2CTL1_INITAK==0);  //�ȴ������ʼ��״̬
  CAN2cnt[0] = 0;
  
  CAN2BTR0_SJW = 0;            //����ͬ��
  
  switch(Baud_Rate)
  {
    case 500:
      CAN2BTR0_BRP = 3;        //����Ԥ��Ƶֵ 
      CAN2BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
    
    case 250:
      CAN2BTR0_BRP = 7;            //����Ԥ��Ƶֵ  
      CAN2BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
      
    case 125:
      CAN2BTR0_BRP = 15;            //����Ԥ��Ƶֵ  
      CAN2BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
  } 
 
//�ر��˲���                                  
  CAN2IDMR0 = 0xFF;
  CAN2IDMR1 = 0xFF;
  CAN2IDMR2 = 0xFF;
  CAN2IDMR3 = 0xFF;
  CAN2IDMR4 = 0xFF;
  CAN2IDMR5 = 0xFF;
  CAN2IDMR6 = 0xFF;
  CAN2IDMR7 = 0xFF; 

  CAN2CTL1 = 0xC0;             //ʹ��MSCANģ��,����Ϊһ������ģʽ��ʹ������ʱ��Դ 

  CAN2CTL0 = 0x00;             //����һ��ģʽ����
  
  do
  {
    if(++CAN2cnt[1]>3000)
    {
      return(Init_Fault_CAN_Unready2);
    }
  }
  while(CAN2CTL1_INITAK);      //�ȴ��ص�һ������ģʽ
  CAN2cnt[1] = 0;
  
  do
  {
    if(++CAN2cnt[2]>3000)
    {
      return(Init_Fault_CAN_Synchr);
    }
  }
  while(CAN2CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��
  CAN2cnt[2] = 0;
  
  CAN2RIER_RXFIE = 1;          //ʹ�ܽ����ж�
  
  return (Init_Normal_CAN);
}

/*=======================================================================
 *������:      CAN2_SendMsg
 *����:        CAN2��������
 *����:        ��չ֡
               sysclk��ʱ������ʱ��:32MHz
               baud:   
 *���أ�       ��
 *˵����       
========================================================================*/ 
uint8 CAN2_SendMsg(pCANFRAME sendFrame)
{
  uint8  send_buf,i;
  uint16  Cnt=0;
  
  // ������ݳ���
  if(sendFrame->m_dataLen > 8)
    return (SendMsg_Fault_Lenth);

  // �������ʱ��
  if(CAN2CTL0_SYNCH==0)
    return (SendMsg_Fault_Synch);

  send_buf = 0;
  do
  {
    // Ѱ�ҿ��еĻ�����
    Cnt++;
    CAN2TBSEL=CAN2TFLG;
    send_buf=CAN2TBSEL;
  } 
  while((!send_buf)&&(Cnt<3000)); 
  //д���ʶ��ID
  
  if (sendFrame->m_IDE == 0)  //����׼֡���ID
  {
    CAN2TXIDR0 = (uint8)(sendFrame->m_ID>>3);
    CAN2TXIDR1 = (uint8)(sendFrame->m_ID<<5);
  } 
  else  //����չ֡���ID
  {
    CAN2TXIDR0 = (uint8)(sendFrame->m_ID>>21);
    CAN2TXIDR1 = (((uint8)(sendFrame->m_ID>>13)) & 0xe0)|0x18|(((uint8)(sendFrame->m_ID>>15))&0x07);
    CAN2TXIDR2 = (uint8)(sendFrame->m_ID>>7);
    CAN2TXIDR3 = (uint8)(sendFrame->m_ID<<1);
  }
  
  if(sendFrame->m_RTR==1)
  {     
    CAN2TXIDR1 |= 0x10;
  }
      
  for (i=0; i<sendFrame->m_dataLen; i++)  
  {
    *((&CAN2TXDSR0) + i) = sendFrame->m_data[i];
  } 
      
  // д�����ݳ���
  CAN2TXDLR = sendFrame->m_dataLen;
  
  // д�����ȼ�
  CAN2TXTBPR = sendFrame->m_priority;
  
  // �� TXx ��־ (������׼������)
  CAN2TFLG = send_buf;
  
  return(SendMsg_Normal);
  
}

/*=======================================================================
 *������:      CAN2_GetMsg(pCANFRAME receiveFrame)
 *����:        CAN2��������
 *����:        ��չ֡
               sysclk��ʱ������ʱ��:32MHz
               baud:  
 *���أ�       ��
 *˵����       
========================================================================*/ 
uint8 CAN2_GetMsg(pCANFRAME receiveFrame)
{
  
  uint8 i;
	  
	if (!(CAN2RFLG_RXF))                         // �����ձ�־λ��
		return (GetMsg_Fault_RFLG_RXF);
	 
	if (CAN2RXIDR1_IDE == 0)                     // �յ���׼֡��
	{
		receiveFrame->m_ID = (uint32)(CAN2RXIDR0<<3) | (uint32)(CAN2RXIDR1>>5);
		receiveFrame->m_RTR = (CAN2RXIDR1>>4) & 0x01;
    receiveFrame->m_IDE = 0;
	} 
	else                                         // �յ���չ֡��
	{
	 	receiveFrame->m_ID = (((uint32)CAN2RXIDR0)<<21)|((uint32)(CAN2RXIDR1&0xe0)<<13)|((uint32)(CAN2RXIDR1&0x07)<<15)|(((uint32)CAN2RXIDR2)<<7);
    receiveFrame->m_ID = receiveFrame->m_ID | ((uint32)(CAN2RXIDR3&0xfe)>>1);
	}
	
	if(CAN3RXIDR3 & 0x01)
    receiveFrame->m_RTR = TRUE;
  else
    receiveFrame->m_RTR = FALSE;    
  
  
  receiveFrame->m_dataLen = CAN2RXDLR&0X0F;
  
  for (i=0;i<receiveFrame->m_dataLen;i++)       // ��ȡ����
  {
	   receiveFrame->m_data[i] = *(&(CAN2RXDSR0)+i);
  }   

  CAN2_GetMsg_Process(receiveFrame);

  CAN2RFLG_RXF = 1;

  return(GetMsg_Normal);
}

/*=======================================================================
 *������:      CAN2_GetMsg_Process(pCANFRAME receiveFrame)
 *����:        CAN2��������ʱ�Ĵ�����
 *����:        ��չ֡
 *���أ�       ��
 
 *˵����       
========================================================================*/
void CAN2_GetMsg_Process(pCANFRAME receiveFrame)
{
  switch(receiveFrame->m_ID) 
  {
    case Boot_ID://0xF300:
      HeartBeat.HeartBeat_CSSU1 = 1;//���Ӱ���г���������ʱ����Ҫ���ε��ߵĹ���
      if(receiveFrame->m_data[1] == 0xAA)
      {
        if(receiveFrame->m_data[0] == BMU_IDNUM)
        {
           Boot_Data.OnlineUpdateCheck = 1;
        }           
      }
    break; 
       
    case 0x002:
    case 0x003:
    case 0x004:
    case 0x005:
    case 0x006:
    case 0x007:
    case 0x008:
    case 0x009:
       HeartBeat.HeartBeat_CSSU1 = 1;//���Ӱ���г���������ʱ����Ҫ���ε��ߵĹ���
    break;
        
    default:             //������λ���������ı���
      if((receiveFrame->m_ID)>>24 == 0x18)//�������� 
      {
        HeartBeat.HeartBeat_CSSU1 = 1;
        DataFromCSSU(receiveFrame);
      } 
      else if((receiveFrame->m_ID)>>24 == 0x19)//�궨����
      {            
        if((receiveFrame->m_ID)>>8 == 0x19FF99)
        {
          DataFromCSSU(receiveFrame); 
          HeartBeat.HeartBeat_CSSU1 = 1;
        }
        else 
        {
          UpMonitor_to_Bms(receiveFrame);
        }
      } 
    break;
  }
}
/*=======================================================================
 *������:      Interrupt_CAN2(void)
 *����:        CAN2�����жϺ���
 *����:        ��չ֡
 *���أ�       ��
 *˵����       CAN2�Ľ����жϺ���
========================================================================*/
#pragma CODE_SEG __NEAR_SEG NON_BANKED
//CAN1������������ 
interrupt void Interrupt_CAN2()            
{     
    CANFRAME mgGet2;
    //Task_Flag_Time.CAN1++;    
    if (CAN2_GetMsg(&mgGet2));
} 
#pragma CODE_SEG DEFAULT

















