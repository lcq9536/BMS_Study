/*=======================================================================
 *Subsystem:   ���
 *File:        CAN3.C
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
  #include  "CAN.h"
  #include  "MC9S12XEP100.h"
/*=======================================================================
 *������:      CAN3_Init(void)
 *����:        ��ʼ��CAN3
 *����:        
               sysclk��ʱ������ʱ��:32MHz
               baud:    
 *���أ�       ��
 *˵����       
========================================================================*/
uint8 CAN3_Init(uint16 Baud_Rate) 
{
  if((Baud_Rate != 125)||(Baud_Rate != 250)||(Baud_Rate != 500))
  {
    return(Init_Fault_CAN_BaudRate);
  }
  
  if(CAN3CTL0_INITRQ==0)      // ��ѯ�Ƿ�����ʼ��״̬   
  {
    CAN3CTL0_INITRQ =1;        // �����ʼ��״̬
  }
 
  while (CAN3CTL1_INITAK==0);  //�ȴ������ʼ��״̬

  CAN3BTR0_SJW = 0;            //����ͬ��
  
  switch(Baud_Rate)
  {
    case 500:
      CAN3BTR0_BRP = 3;            //����Ԥ��Ƶֵ 
      CAN3BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
    
    case 250:
      CAN3BTR0_BRP = 7;            //����Ԥ��Ƶֵ  
      CAN3BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
      
    case 125:
      CAN3BTR0_BRP = 15;            //����Ԥ��Ƶֵ  
      CAN3BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� 
      break;
  }
 
//�ر��˲���                                  
  CAN3IDMR0 = 0xFF;
  CAN3IDMR1 = 0xFF;
  CAN3IDMR2 = 0xFF;
  CAN3IDMR3 = 0xFF;
  CAN3IDMR4 = 0xFF;
  CAN3IDMR5 = 0xFF;
  CAN3IDMR6 = 0xFF;
  CAN3IDMR7 = 0xFF; 

  CAN3CTL1 = 0xC0;             //ʹ��MSCANģ��,����Ϊһ������ģʽ��ʹ������ʱ��Դ 

  CAN3CTL0 = 0x00;             //����һ��ģʽ����

  while(CAN3CTL1_INITAK);      //�ȴ��ص�һ������ģʽ

  while(CAN3CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��
  
  CAN3RIER_RXFIE = 1;          //ʹ�ܽ����ж�
  
  return (Init_Normal_CAN);
}

/*=======================================================================
 *������:      CAN3_SendMsg
 *����:        CAN3��������
 *����:        ��չ֡
               sysclk��ʱ������ʱ��:32MHz
               baud:   
 *���أ�       ��
 *˵����       
========================================================================*/ 
uint8 CAN3_SendMsg(pCANFRAME sendFrame)
{
  uint8 send_buf,i;
  uint8 Cnt[1];
  
  // ������ݳ���
  if(sendFrame->m_dataLen > 8)
    return (SendMsg_Fault_Lenth);

  // �������ʱ��
  if(CAN3CTL0_SYNCH==0)
    return (SendMsg_Fault_Synch);

  send_buf = 0;
  do
  {
    // Ѱ�ҿ��еĻ�����
    CAN3TBSEL=CAN3TFLG;
    send_buf=CAN3TBSEL;
    if(++Cnt[0]>=200)
    {
      Cnt[0] = 0;
      return(SendMsg_Fault_NoEmptyNode);
    }
  } 
  while(!send_buf); 
  //д���ʶ��ID
  
  if (sendFrame->m_IDE == 0)  //����׼֡���ID
  {
    CAN3TXIDR0 = (uint8)(sendFrame->m_ID>>3);
    CAN3TXIDR1 = (uint8)(sendFrame->m_ID<<5);
  } 
  else  //����չ֡���ID
  {
    CAN3TXIDR0 = (uint8)(sendFrame->m_ID>>21);
    CAN3TXIDR1 = (((uint8)(sendFrame->m_ID>>13)) & 0xe0)|0x18|(((uint8)(sendFrame->m_ID>>15)) &0x07);
    CAN3TXIDR2 = (uint8)(sendFrame->m_ID>>7);
    CAN3TXIDR3 = (uint8)(sendFrame->m_ID<<1);
  }
  
  if(sendFrame->m_RTR==1)
  {     
      CAN3TXIDR1 |= 0x10;
  }
      
  for (i=0;i<sendFrame->m_dataLen;++i)  
  {
    *((&CAN3TXDSR0) + i) = sendFrame->m_data[i];
  } 
      
  // д�����ݳ���
  CAN3TXDLR = sendFrame->m_dataLen;
  
  // д�����ȼ�
  CAN3TXTBPR = sendFrame->m_priority;
  
  // �� TXx ��־ (������׼������)
  CAN3TFLG = send_buf;
  
  return(SendMsg_Normal);
  
}

/*=======================================================================
 *������:      CAN3_GetMsg(pCANFRAME receiveFrame)
 *����:        CAN3��������
 *����:        ��չ֡
               sysclk��ʱ������ʱ��:32MHz
               baud:  
 *���أ�       ��
 *˵����       
========================================================================*/ 
uint8 CAN3_GetMsg(pCANFRAME receiveFrame)
{
  
  uint8 i;
	  
	if (!(CAN3RFLG_RXF))                         // �����ձ�־λ��
		return (GetMsg_Fault_RFLG_RXF);
	 
	if (CAN3RXIDR1_IDE == 0)                     // �յ���׼֡��
	{
		receiveFrame->m_ID = (uint32)(CAN3RXIDR0<<3) | (uint32)(CAN3RXIDR1>>5);
		receiveFrame->m_RTR = (CAN3RXIDR1>>4) & 0x01;
    receiveFrame->m_IDE = 0;
	} 
	else                                         // �յ���չ֡��
	{
	 	receiveFrame->m_ID = (((uint32)CAN3RXIDR0)<<21)|((uint32)(CAN3RXIDR1&0xe0)<<13)|((uint32)(CAN3RXIDR1&0x07)<<15)|(((uint32)CAN3RXIDR2)<<7);
    receiveFrame->m_ID = receiveFrame->m_ID | ((uint32)(CAN3RXIDR3&0xfe)>>1);
	}
	 
  receiveFrame->m_dataLen = CAN3RXDLR&0X0F;
  
  for (i=0;i<receiveFrame->m_dataLen;i++)       // ��ȡ����
  {
	   receiveFrame->m_data[i] = *(&(CAN3RXDSR0)+i);
  }   

  CAN3RFLG_RXF = 1;

  return(GetMsg_Normal);
}




