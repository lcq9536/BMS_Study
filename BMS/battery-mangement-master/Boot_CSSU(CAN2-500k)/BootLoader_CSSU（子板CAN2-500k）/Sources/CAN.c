#include <string.h>
#include "Types.h"
#include "MC9S12XEP100.h"
#include "CAN.h"
#include "Srec.h"
/*******************************************************************************/

#define RxBufSize 80
UINT8 RxIn;               //next available location in the Rx queue
UINT8 RxOut;              //next character to be removed from the Rx queue

UINT8 RxBuff[RxBufSize];  //receive queue
volatile UINT8 RxBAvail;  //number of bytes left in the Rx queue


CAN_MSG can_msg;
CAN_MSG can_send;

/******************************************************************************/
   //�ӻ�������ȡ��
INT8 getchar(void)
{
 UINT8 c;                   //holds the character we'll return
 
 //if there's no characters in the Rx buffer wait here
 while (RxBAvail == RxBufSize);
 
 DisableInterrupts;
 
 //get a character from the Rx buffer & advance the Rx index
 c = RxBuff[RxOut++];
 if (RxOut == RxBufSize)              //index go past the physical end of the buffer?
    RxOut = 0;                //yes wrap around to the start
 RxBAvail++;                //1 more byte available in the receive buffer

 EnableInterrupts;
 return(c);     //return the character retrieved from receive buffer
}         

/******************************************************************************/
void INIT_CAN2(void) 
{
  RxIn = RxOut= 0;  //set the Rx & Tx queue indexes to zero
  RxBAvail = RxBufSize;     //the receive buffer is empty
  can_send.id = ID_Reply; 
  can_send.len = 1;
  can_send.RTR = FALSE;
  can_send.prty = 0;
   
  if(CAN2CTL0_INITRQ==0)      // ��ѯ�Ƿ�����ʼ��״̬   
    CAN2CTL0_INITRQ =1;        // �����ʼ��״̬

  while (CAN2CTL1_INITAK==0);  //�ȴ������ʼ��״̬

  CAN2BTR0_SJW = 0;            //����ͬ��
  CAN2BTR0_BRP = 3;            //���ò�����,�趨Ϊ:500kb

  
  CAN2BTR1 = 0x5c;     //����Ƶ��Ϊ250kb/s=400000/(8*(1+5+1+12+1))
  CAN2IDAC=0x00;    //2��32λ�����˲������˲���0��Ч
                        /*  0b00000000
                         *    ||||||||__ bit0��1��2:��ʶ��������Чָʾ��0��Ч   
                         *    |||||||___| 
                         *    ||||||____| 
                         *    |||||_____  
                         *    ||||______ bit4��5:IDAM ��ʶ������ģʽ��2��32λ����4��16λ 
                         *    |||_______| 
                         *    ||________ bit6: 
                         *    |_________ bit7: 
                         */
  CAN2IDAR0=0x00;    //ֻ����IDΪ0x002ͨ��
  CAN2IDAR1=0x40; 
  CAN2IDAR2=0x00; 
  CAN2IDAR3=0x00;
 
  CAN2IDAR4=0x00; 
  CAN2IDAR5=0x40; 
  CAN2IDAR6=0x00; 
  CAN2IDAR7=0x00;   
// ���˲���                                  
  CAN2IDMR0 = 0x00;
  CAN2IDMR1 = 0x00;
  CAN2IDMR2 = 0x00;
  CAN2IDMR3 = 0x00;
  CAN2IDMR4 = 0x00;
  CAN2IDMR5 = 0x00;
  CAN2IDMR6 = 0x00;
  CAN2IDMR7 = 0x00;  

  CAN2CTL1 = 0xC0;             //ʹ��MSCANģ��,����Ϊһ������ģʽ��ʹ������ʱ��Դ 

  CAN2CTL0 = 0x00;             //����һ��ģʽ����

  while(CAN2CTL1_INITAK);      //�ȴ��ص�һ������ģʽ

  while(CAN2CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��

  CAN2RIER_RXFIE = 1;          //ʹ�ܽ����ж�
}

/******************************************************************************/
Bool MSCAN2GetMsg(struct can_msg *msg)
{
  
  unsigned char sp2;

  // �����ձ�־
  if(!(CAN2RFLG_RXF))
    return(FALSE);
  
  // ��� CANЭ�鱨��ģʽ ��һ��/��չ�� ��ʶ��
  if(CAN2RXIDR1_IDE)
    // IDE = Recessive (Extended Mode)
    return(FALSE);

  // ����ʶ��
  msg->id = (unsigned int)(CAN2RXIDR0<<3) | 
            (unsigned char)(CAN2RXIDR1>>5);
  
  if(CAN2RXIDR1&0x10)
    msg->RTR = TRUE;
  else
    msg->RTR = FALSE;
  
  // ��ȡ���ݳ��� 
  msg->len = CAN2RXDLR;
  
  // ��ȡ����
  for(sp2 = 0; sp2 < msg->len; sp2++)
    msg->data[sp2] = *((&CAN2RXDSR0)+sp2);

  // �� RXF ��־λ (������׼������)
  CAN2RFLG = 0x01;

  return(TRUE);
}


/*************************************************************/
/*                       CAN1����                            */
/*************************************************************/
Bool MSCAN2SendMsg(struct can_send msg)
{
  unsigned char send_buf, sp;
  
  // ������ݳ���
  if(msg.len > 8)
    return(FALSE);

  // �������ʱ��
  if(CAN2CTL0_SYNCH==0)
    return(FALSE);

  send_buf = 0;
  do
  {
    // Ѱ�ҿ��еĻ�����
    CAN2TBSEL=CAN2TFLG;      
    send_buf=CAN2TBSEL;
  }
  while(!send_buf); 
  
  // д���ʶ��
  CAN2TXIDR0 = (unsigned char)(msg.id>>3);
  CAN2TXIDR1 = (unsigned char)(msg.id<<5);
  
  if(msg.RTR)
    // RTR = ���ԣ�����֡orԶ��֡��
    CAN2TXIDR1 |= 0x10;
    
  // д������
  for(sp = 0; sp < msg.len; sp++)
    *((&CAN2TXDSR0)+sp) = msg.data[sp];
    
  // д�����ݳ���
  CAN2TXDLR = msg.len; 
  
  // д�����ȼ�
  CAN2TXTBPR = msg.prty;
  
  // �� TXx ��־ (������׼������)
  CAN2TFLG = send_buf;
  
  return(TRUE);  //�������ݵ�״̬ 
  
}

/*********************************************************************************/
#pragma CODE_SEG RAM_CODE
interrupt void CAN2RxISR(void) 
{  
   char index;
   
   if(MSCAN2GetMsg(&can_msg)) //�������ݳɹ�
   {
     if(can_msg.id == ID01_6811) //����ID�Ž��н���CANͨ������
     {
       for(index = 0;index < can_msg.len;index++)                                                    
         {
            if (RxBAvail != 0)            //if there are bytes available in the Rx buffer
               {
                  RxBAvail--;
                  RxBuff[RxIn++] = can_msg.data[index];      //place the received byte in the buffer
                  can_msg.data[index]=0;
                  if (RxIn == RxBufSize)      //reached the physical end of the buffer?
                  { 
                    RxIn = 0;                  //yes. wrap around to the start
                  }
                         
               } 
         }                      
     } 
   }
}
#pragma CODE_SEG DEFAULT
/******************************************************************************/
