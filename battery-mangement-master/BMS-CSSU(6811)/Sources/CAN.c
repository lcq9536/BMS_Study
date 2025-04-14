#include "Includes.h"



/*************************************************************/
/*                        ��ʼ��CAN2                        */
/*************************************************************/
void INIT_CAN2(void) 
{
  
   //Task_Die |=0x00000010;
  
  if(CAN2CTL0_INITRQ==0)      // ��ѯ�Ƿ�����ʼ��״̬   
    CAN2CTL0_INITRQ =1;        // �����ʼ��״̬

  while (CAN2CTL1_INITAK==0);  //�ȴ������ʼ��״̬

  CAN2BTR0_SJW = 0;            //����ͬ��
  CAN2BTR0_BRP = 3;            //���ò�����  

  
  CAN2BTR1 = 0x1c;     //����ʱ��1��ʱ��2��Tq���� ,����Ƶ��Ϊ250kb/s

// �ر��˲���                                  
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

  while(CAN2CTL1_INITAK);      //�ȴ��ص�һ������ģʽ

  while(CAN2CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��

  CAN2RIER_RXFIE = 1;          //ʹ�ܽ����ж�
  
  //Task_Die &=0xFFFFFFEF;
}


/*************************************************************/
/*                        CAN2����                           */
/*************************************************************/
Bool MSCAN2SendMsg(pCANFRAME sendFrame)
{
  uint8 send_buf,i;
  uint16 Counter = 0;
  
  // ������ݳ���
  if(sendFrame->m_dataLen > 8)
    return 1;

  // �������ʱ��
  if(CAN2CTL0_SYNCH==0)
    return 2;

  send_buf = 0;
  do
  {
    // Ѱ�ҿ��еĻ�����
    CAN2TBSEL=CAN2TFLG;
    send_buf=CAN2TBSEL;
    
    Counter++;
  }  
  while( (!send_buf)&&(Counter<2000)); 
  //д���ʶ��ID
  
  if (sendFrame->m_IDE == 0)  //����׼֡���ID
  {
    CAN2TXIDR0 = (uint8)(sendFrame->m_ID>>3);
    CAN2TXIDR1 = (uint8)(sendFrame->m_ID<<5);
  } 
  else  //����չ֡���ID
  {
    CAN2TXIDR0 = (uint8)(sendFrame->m_ID>>21);
    CAN2TXIDR1 = (((uint8)(sendFrame->m_ID>>13)) & 0xe0)|0x18 | (((uint8)(sendFrame->m_ID>>15)) &0x07);
    CAN2TXIDR2 = (uint8)(sendFrame->m_ID>>7);
    CAN2TXIDR3 = (uint8)(sendFrame->m_ID<<1);
  }
  
  
  if(sendFrame->m_RTR==1)
  { 
      // RTR = ����
      CAN2TXIDR1 |= 0x10;
  }

  if (sendFrame->m_RTR == 0)  //������֡���
  {
    for (i=0;i<sendFrame->m_dataLen;++i)  
    {
      *((&CAN2TXDSR0) + i) = sendFrame->m_data[i];
    }
  }
  else   //��Զ��֡���
  {
    CAN2TXDLR = 0;
  }
    
  // д�����ݳ���
  CAN2TXDLR = sendFrame->m_dataLen;
  
  // д�����ȼ�
  CAN2TXTBPR = sendFrame->m_priority;
  
  // �� TXx ��־ (������׼������)
  CAN2TFLG = send_buf;
  
  return 0;
  
}
/*************************************************************/
/*                        CAN2����                           */
/*************************************************************/
Bool MSCAN2GetMsg(pCANFRAME receiveFrame)
{
  uint8 i,j;
  uint8 retu_value;
	  
	if (!(CAN2RFLG_RXF))                         // �����ձ�־λ��
		return 1;
	  
	if (CAN2RXIDR1_IDE == 0)                     // �յ���׼֡��
	{
		receiveFrame->m_ID = (uint32)(CAN2RXIDR0<<3) | (uint32)(CAN2RXIDR1>>5);
		receiveFrame->m_RTR = (CAN2RXIDR1>>4) & 0x01;
    receiveFrame->m_IDE = 0;
	} 
	else                                         // �յ���չ֡��
	{
		receiveFrame->m_ID = (((uint32)CAN2RXIDR0)<<21)|((uint32)(CAN2RXIDR1&0xe0)<<13)| \
		((uint32)(CAN2RXIDR1&0x07)<<15)|(((uint32)CAN2RXIDR2)<<7)| \
		((uint32)(CAN2RXIDR3&0xfe)>>1);
	}
  receiveFrame->m_dataLen = CAN2RXDLR_DLC&0X0F;

  for (i=0;i<receiveFrame->m_dataLen;i++)       // ��ȡ����
  {
	  receiveFrame->m_data[i]= *(&(CAN2RXDSR0)+i);
	   
  }
      
      
  switch(receiveFrame->m_ID)
  {
    case Boot_ID://0xF300
      if(receiveFrame->m_data[1] == 0xAA)
      {
          if(receiveFrame->m_data[0] == (CSSU_IDNUM&0x01))
          {
             Boot_Data.OnlineUpdateCheck = 1;
          }           
      }
    break;

    case 0x18FF9700:
      balance_receive.BalanceOn = receiveFrame->m_data[0];
      BMU_OffLineState.BMU_Life = 1;  //life�ź�
    break;
    
    case 0x18FF9900:
      balance_receive.total_volt = (receiveFrame->m_data[5] + ((uint16)receiveFrame->m_data[6]<<8) + ((uint32)receiveFrame->m_data[7]<<16)); 
    break;
    
    default:
    break;     
  }
  

  CAN2RFLG_RXF = 1;

  return 0;
}
