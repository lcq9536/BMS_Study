/*=======================================================================
 *Subsystem:   ���
 *File:        GPIO.C
 *Author:      WENMING
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/ 

//ͷ�ļ�����
#include  "GPIO.h"             //��ͨIO�ڹ���ͷ�ļ�
             
    //2���ڲ�����
static uint8 ABEK_Bit(uint16 port);
static uint8 ADEK_AddrChg(uint16 port,uint8 reg);
    
/*=======================================================================
 *������:      GPIO_Init(uint16 port,uint8 pin,uint8 direction,uint8 state)
 *����:        IO�ڳ�ʼ��
 *����:        port:port������
               pin:port���         
               direction:
               state:״̬
 *���أ�       Init_Normal_GPIO
 *˵����       
========================================================================*/
uint8 GPIO_Init(uint16 port,uint8 pin,uint8 direction,uint8 state)
{
  GPIO_Set(port,DDR,pin,direction);
  //1.���ŷ���Ϊ���ʱ,��������״̬�����ŷ���Ϊ���ʱ��Ҫд�����Ŷ�Ӧ���ݼĴ���λ�����ŷ���Ϊ����ʱ������ 
  if(direction==1) 
    GPIO_Set(port,PRT,pin,state);
  
  return (Init_Normal_GPIO);
}

/*=======================================================================
 *������:      GPIO_Get(uint16 port,uint8 reg,uint8 pin)
 *����:        ��ȡGPIO�ڵ�
 *����:        ��         
                      
 *���أ�       ��
 *˵����       ADC ʱ��Ƶ�ʣ�2MHz
========================================================================*/
uint8 GPIO_Get(uint16 port,uint8 reg,uint8 pin)
{
  uint8 reval=0,bit=0;
  //��ȡA��B��E��K����������Ĵ����͵͹�����������ֵ
  if((reg==PER)||(reg==RDR))
  {
    bit=ABEK_Bit(port);
    if((reg==PER)&&(bit!=8))
    {
      reval=BGET(bit,PUCR );
      return reval;
    }
    else if((reg==RDR)&&(bit!=8))
    {
      reval=BGET(bit,RDRIV);
      return reval;
    }
  }
  //��������Ĵ���ƫ����
  reg=ADEK_AddrChg(port,reg);
  //��ȡ�Ĵ���������ֵ
  reval = BGET(pin,(*(vuint8 *)(port+reg)));    //�õ����ŵ�״̬
  return reval;
}

/*=======================================================================
 *������:      GPIO_Set(uint16 port,uint8 reg,uint8 pin,uint8 state)
 *����:        ��������״̬
 *����:        port :�˿���                                                  *
               reg  :ָ���˿ڼĴ���                                          *
               pin  :ָ���˿�����
               state:0=�͵�ƽ,1=�ߵ�ƽ(ABEK������͹���,Ч��=0xFF��=0x00)     
 *���أ�       ��
 *˵����       
========================================================================*/
void GPIO_Set(uint16 port,uint8 reg,uint8 pin,uint8 state)
{
    uint8 bit=0;
    //A��B��E��K����������Ĵ����͵͹��������Ĵ�������
    if((reg==PER)||(reg==RDR))  
    {
      bit=ABEK_Bit(port);
      if((reg==PER)&&(bit!=8))
      {
        if(state == 1) 
          BSET(bit,PUCR);
        else  
          BCLR(bit,PUCR);
        return;
      }
      else if((reg==RDR)&&(bit!=8))
      {
        if(state == 1)
          BSET(bit,RDRIV);
        else 
          BCLR(bit,RDRIV);
        return;
      }
    }
    //��������Ĵ���ƫ����
    reg=ADEK_AddrChg(port,reg);
    //���üĴ�����ֵ
    if(state == 1)
      BSET(pin,(*(vuint8 *)(port+reg)));   //����ߵ�ƽ(1)
    else                                  
      BCLR(pin,(*(vuint8 *)(port+reg)));   //����͵�ƽ(0)
} 

/*=======================================================================
 *������:      GPPort_Get(uint16 port,uint8 reg)
 *����:        ��û�ȡ�˿ڸ�����״̬
 *����:        port :�˿���                                                  *
               reg  :ָ���˿ڼĴ���                                          *
 *���أ�       reval:�õ��˿�״̬(����ABEK������͹���,Ч��=0xFF��=0x00) 
 *˵����       
========================================================================*/
uint8 GPPort_Get(uint16 port,uint8 reg)
{
    uint8 reval=0,bit=0;
    //��ȡA��B��E��K����������Ĵ����͵͹�����������ֵ
    if((reg==PER)||(reg==RDR))
    {
      bit=ABEK_Bit(port);
      if((reg==PER)&&(bit!=8))
      {
        reval=BGET(bit,PUCR );
        if(reval==1) reval=0xFF;
        else reval=0x00;
        return reval;
      }
      else if((reg==RDR)&&(bit!=8))
      {
        reval=BGET(bit,RDRIV);
        if(reval==1) reval=0xFF;
        else reval=0x00;
        return reval;
      }
    }
    //��������Ĵ���ƫ����
    reg=ADEK_AddrChg(port,reg);
    //��ȡ�Ĵ���������ֵ
    reval = (*(vuint8 *)(port+reg));    //�õ����ŵ�״̬
    return reval;
}

/*=======================================================================
 *������:      GPPort_Set(uint16 port,uint8 reg,uint8 setFlag,uint8 bValue)
 *����:        ���ö˿ڸ�����״̬  
 *����:        port   :  �˿���                                              *
               reg    :  ָ���Ĵ���                                          *
               setFlag:  =0ʱ, bValue����=0��λ���Ĵ�����Ӧλ��0��           *
                         =1ʱ, bValue����=1��λ���Ĵ�����Ӧλ��1��           *
                         =����ֵ,�Ĵ���=bValue                               *
               bValue:ָ��ֵ(����ABEK������͹���,Ч��=0xFF��=0x00)                                            *
 *���أ�       ��
 *˵����       
========================================================================*/
void GPPort_Set(uint16 port,uint8 reg,uint8 setFlag,uint8 bValue)
{
    uint8 bit=0;
    //A��B��E��K����������Ĵ����͵͹��������Ĵ�������
   if((reg==PER)||(reg==RDR))
    {
      bit=ABEK_Bit(port);
      if((reg==PER)&&(bit!=8))
      {
        if(bValue == 0xFF) 
          BSET(bit,PUCR);
        else if(bValue == 0x00) 
          BCLR(bit,PUCR);
        return;
      }
      else if((reg==RDR)&&(bit!=8))
      {
        if(bValue == 0xFF) 
          BSET(bit,RDRIV);
        else if(bValue == 0x00)
          BCLR(bit,RDRIV);
        return;
      }
    }
    //��������Ĵ���ƫ����
    reg=ADEK_AddrChg(port,reg);        
    //���üĴ�����ֵ
    if(setFlag==1)      (*(vuint8 *)(port+reg)) |= bValue;          
    else if(setFlag==0) (*(vuint8 *)(port+reg)) &= bValue;          
    else                (*(vuint8 *)(port+reg))  = bValue;          
}         

/*=======================================================================
 *������:      ABEK_Bit(uint16 port)
 *����:        A��B��E��K����������Ĵ����͵͹��������Ĵ�����λȷ�� 
 *����:        port:�˿���                                        
 *���أ�       bit:�Ĵ�����Ӧ��λֵ
 *˵����       
========================================================================*/
static
uint8 ABEK_Bit(uint16 port)
{
    uint8 bit=0;
    switch(port)
    {
      case PA:bit=0;
      break;
      case PB:bit=1;
      break;
      case PE:bit=4;
      break;
      case PK:bit=7;
      break;
      default:bit=8;
    }
    return bit;
}

/*=======================================================================
 *������:      ADEK_AddrChg(uint16 port,uint8 reg)
 *����:        ��������Ĵ���ƫ���� 
 *����:        port:�˿���  
               reg :ָ���˿ڼĴ���                            
 *���أ�       reg :�Ĵ�����ַƫ��������ֵ  
 *˵����       
========================================================================*/
static
uint8 ADEK_AddrChg(uint16 port,uint8 reg)
{
    if((port==PE)||(port==PK))
    {
      if(reg==DDR) 
        reg=DDR-1;  //��ʱreg=PTI ����Ĵ���
    }
    else if((port==PAD0)||(port==PAD1))
    {
      if(reg==RDR) 
        reg=RDR+1;
      else if(reg==PER) 
        reg=PER+2; 
    }
    return reg;
}

   

