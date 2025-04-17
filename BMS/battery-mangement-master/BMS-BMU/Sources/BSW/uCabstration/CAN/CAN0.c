/*=======================================================================
 *Subsystem:   ���
 *File:        CAN0.C
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
 *������:      CAN0_Init(void)
 *����:        ��ʼ��CAN0
 *����:        
               sysclk��ʱ������ʱ��:32MHz
               baud:    
 *���أ�       ��
 *˵����       
========================================================================*/
uint8 CAN0_Init(uint16 Baud_Rate) 
{
  if((Baud_Rate != 125)||(Baud_Rate != 250)||(Baud_Rate != 500))
  {
    return(Init_Fault_CAN_BaudRate);
  }
  
  if(CAN0CTL0_INITRQ==0)      // ��ѯ�Ƿ�����ʼ��״̬   
  {
    CAN0CTL0_INITRQ =1;        // �����ʼ��״̬
  }   

  while (CAN0CTL1_INITAK==0);  //�ȴ������ʼ��״̬

  CAN0BTR0_SJW = 0;            //����ͬ��
  
  switch(Baud_Rate)
  {
    case 500:
      CAN0BTR0_BRP = 3;            //����Ԥ��Ƶֵ 
      CAN0BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
    
    case 250:
      CAN0BTR0_BRP = 7;            //����Ԥ��Ƶֵ  
      CAN0BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
      
    case 125:
      CAN0BTR0_BRP = 15;            //����Ԥ��Ƶֵ  
      CAN0BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
  }
 
//�ر��˲���                                  
  CAN0IDMR0 = 0xFF;
  CAN0IDMR1 = 0xFF;
  CAN0IDMR2 = 0xFF;
  CAN0IDMR3 = 0xFF;
  CAN0IDMR4 = 0xFF;
  CAN0IDMR5 = 0xFF;
  CAN0IDMR6 = 0xFF;
  CAN0IDMR7 = 0xFF; 

  CAN0CTL1 = 0xC0;             //ʹ��MSCANģ��,����Ϊһ������ģʽ��ʹ������ʱ��Դ 

  CAN0CTL0 = 0x00;             //����һ��ģʽ����

  while(CAN0CTL1_INITAK);      //�ȴ��ص�һ������ģʽ

  while(CAN0CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��
  
  CAN0RIER_RXFIE = 1;          //ʹ�ܽ����ж�
  
  return (Init_Normal_CAN);
}

/*=======================================================================
 *������:      CAN0_SendMsg(pCANFRAME sendFrame)
 *����:        CAN0��������
 *����:        ��չ֡
               sysclk��ʱ������ʱ��:32MHz
               baud:   
 *���أ�       ��
 *˵����       
========================================================================*/ 
uint8 CAN0_SendMsg(pCANFRAME sendFrame)
{
  uint8 send_buf,i;
  uint8 Cnt[1];
  //uint8 Can_state;
  
  //Can_state = SendMsg_Fault_Busy;
  
  // ������ݳ���
  if(sendFrame->m_dataLen > 8) 
  {
    return (SendMsg_Fault_Lenth);   
  }

  // �������ʱ��
  if(CAN0CTL0_SYNCH==0)  
  {
    return (SendMsg_Fault_Synch);
  }
  send_buf = 0;
  do
  {
    // Ѱ�ҿ��еĻ�����
    CAN0TBSEL=CAN0TFLG;
    send_buf=CAN0TBSEL;
    if(++Cnt[0]>=200)
    {
      Cnt[0] = 0;
      return(SendMsg_Fault_NoEmptyNode); //3
    }
  } 
  while(!send_buf); 
  Cnt[0] = 0; 
  
  //д���ʶ��ID
  
  if (sendFrame->m_IDE == 0)  //����׼֡���ID
  {
    CAN0TXIDR0 = (uint8)(sendFrame->m_ID>>3);
    CAN0TXIDR1 = (uint8)(sendFrame->m_ID<<5);
  } 
  else  //����չ֡���ID
  {
    CAN0TXIDR0 = (uint8)(sendFrame->m_ID>>21);
    CAN0TXIDR1 = (((uint8)(sendFrame->m_ID>>13)) & 0xe0)|0x18|(((uint8)(sendFrame->m_ID>>15)) &0x07);
    CAN0TXIDR2 = (uint8)(sendFrame->m_ID>>7);
    CAN0TXIDR3 = (uint8)(sendFrame->m_ID<<1);
  }
  
  if(sendFrame->m_RTR==1)
  {     
      CAN0TXIDR1 |= 0x10;
  }
      
  for (i=0;i<sendFrame->m_dataLen;++i)  
  {
    *((&CAN0TXDSR0) + i) = sendFrame->m_data[i];
  } 
      
  // д�����ݳ���
  CAN0TXDLR = sendFrame->m_dataLen;
  
  // д�����ȼ�
  CAN0TXTBPR = sendFrame->m_priority;
  
  // �� TXx ��־ (������׼������)
  CAN0TFLG = send_buf;
  
  //Can_state = SendMsg_Normal; 
  
  return(SendMsg_Normal);
  
}

/*=======================================================================
 *������:      CAN0_GetMsg(pCANFRAME receiveFrame)
 *����:        CAN0��������
 *����:        ��չ֡
               sysclk��ʱ������ʱ��:32MHz
               baud:  
 *���أ�       ��
 *˵����       
========================================================================*/ 
uint8 CAN0_GetMsg(pCANFRAME receiveFrame)
{
  uint8 i;
	  
	if (!(CAN0RFLG_RXF))                         // �����ձ�־λ��
		return (GetMsg_Fault_RFLG_RXF);
	 
	if (CAN0RXIDR1_IDE == 0)                     // �յ���׼֡��
	{
		receiveFrame->m_ID = (uint32)(CAN0RXIDR0<<3) | (uint32)(CAN0RXIDR1>>5);
		receiveFrame->m_RTR = (CAN0RXIDR1>>4) & 0x01;
    receiveFrame->m_IDE = 0;
	} 
	else                                         // �յ���չ֡��
	{
	 	receiveFrame->m_ID = (((uint32)CAN0RXIDR0)<<21)|((uint32)(CAN0RXIDR1&0xe0)<<13)|((uint32)(CAN0RXIDR1&0x07)<<15)|(((uint32)CAN0RXIDR2)<<7);
    receiveFrame->m_ID = receiveFrame->m_ID | ((uint32)(CAN0RXIDR3&0xfe)>>1);
	}
	 
  receiveFrame->m_dataLen = CAN0RXDLR&0X0F;
  
  for (i=0;i<receiveFrame->m_dataLen;i++)       // ��ȡ����
  {
	   receiveFrame->m_data[i] = *(&(CAN0RXDSR0)+i);
  }   

  CAN0RFLG_RXF = 1;

  return(GetMsg_Normal);
}




















