/*=======================================================================
 *Subsystem:   ���
 *File:        IIC.C
 *Author:      Wenming
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "IIC.h"


/*=======================================================================
 *������:      uint8 IIC_Init(void) 
 *����:        IIC��ʼ������
 *����:        ��       
 *���أ�       ��
 
 *˵����       
========================================================================*/
uint8 IIC_Init(void) 
{
  IIC0_IBFD = 0x94;   //����ʱ��32MHz,����SCL��ƵΪ100KHz
  IIC0_IBCR = 0x80;   //ʹ��IICģ��,��ֹ�ж�
  IIC0_IBSR_IBAL = 1; //���IBAL��־λ  
  
  return(Init_Normal_IIC);
}

/* IIC������д���� */
void IIC_write(uint8 addr,uint8 writeaddr,uint8 data) 
{   
  
  IIC0_IBCR_TXAK = 0;               // ���յ����ݺ���Ӧ��
  IIC0_IBCR_TX_RX = 1;              // ���õ�Ƭ��Ϊ����ģʽ
  IIC0_IBCR_MS_SL = 1;              // ���õ�Ƭ��Ϊ����ģʽ��������ʼ�ź�

  IIC0_IBDR = addr;
  while(IIC0_IBSR_IBIF == 0); 
  IIC0_IBSR_IBIF = 1;
  while(IIC0_IBSR_RXAK);
  
  IIC0_IBDR = writeaddr;
  while(IIC0_IBSR_IBIF == 0); 
  IIC0_IBSR_IBIF = 1;
  while(IIC0_IBSR_RXAK);
  
  IIC0_IBDR = data;
  while(IIC0_IBSR_IBIF == 0);
  IIC0_IBSR_IBIF = 1;
  while(IIC0_IBSR_RXAK);

  IIC0_IBCR_MS_SL = 0;
}

/*IIC�����Ķ�����*/
uint8 IIC_read(uint8 addr,uint8 readaddr) 
{
  uint8 data;
  IIC0_IBCR_TXAK = 0;               // ���յ����ݺ���Ӧ��
  IIC0_IBCR_TX_RX = 1;              // ���õ�Ƭ��Ϊ����ģʽ
  IIC0_IBCR_MS_SL = 1;              // ���õ�Ƭ��Ϊ����ģʽ��������ʼ�ź�

  IIC0_IBDR = addr;                 // ���ʹ������ĵ�ַģʽΪд��
  while(IIC0_IBSR_IBIF == 0); 
  IIC0_IBSR_IBIF = 1;
  while(IIC0_IBSR_RXAK);
  
  IIC0_IBDR = readaddr;             // ���ʹ洢����ַ
  while(IIC0_IBSR_IBIF == 0); 
  IIC0_IBSR_IBIF = 1;
  while(IIC0_IBSR_RXAK);
  
  IIC0_IBCR_RSTA=1;                  
  
  IIC0_IBCR_TXAK = 0;               // ���յ����ݺ���Ӧ��
  IIC0_IBCR_TX_RX = 1;              // ���õ�Ƭ��Ϊ����ģʽ
  IIC0_IBCR_MS_SL = 1;              // ���õ�Ƭ��Ϊ����ģʽ��������ʼ�ź�

  IIC0_IBDR = addr+1;               // ���ʹ������ĵ�ַģʽΪ�������Լ�1��
  while(IIC0_IBSR_IBIF == 0); 
  IIC0_IBSR_IBIF = 1;
  while(IIC0_IBSR_RXAK);

  IIC0_IBCR_TX_RX = 0;             // ���õ�Ƭ��Ϊ����ģʽ
  IIC0_IBCR_TXAK = 1;              // ���յ����ݺ���Ӧ��

  data = IIC0_IBDR;                // ���IICD�Ĵ�����׼�����գ�

  while(IIC0_IBSR_IBIF == 0); 
  IIC0_IBSR_IBIF = 1;
  IIC0_IBCR_MS_SL = 0;

  data = IIC0_IBDR;                // ��ȡ���յ�������
  return(data);
}

  