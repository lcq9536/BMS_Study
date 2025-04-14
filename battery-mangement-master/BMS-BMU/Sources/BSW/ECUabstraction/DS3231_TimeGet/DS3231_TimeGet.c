/*=======================================================================
 *Subsystem:   ���
 *File:        DS3231_TimeGet.c
 *Author:      Wenming
 *Description: ͨ�ţ�
               �ӿڣ�PJ1��PJ0��
               �����ʣ�100KHz
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "includes.h"
Read_IIC_Time_T Read_IIC_Time;  
/*=======================================================================
 *������:      BCD2HEX(uint8)
 *����:        ������תʮ��������
 *����:        ��       
 *���أ�       ��
 *˵����       ��BCD��ת����16������,BCD������4λ2��������ʾ1��10������,����λ��ʮ��������
               ����λ��ʮ�����������ӣ�0x00100100;����λ����2������λ��4������24��
========================================================================*/
static
uint8 BCD2HEX(uint8 val) 
{
  uint8 value;
  value=val&0x0f;
  val>>=4;
  val&=0x0f;
  val*=10;
  value+=val;
  return value;
}

/*=======================================================================
 *������:      HEX2BCD(uint8)
 *����:        ������תʮ��������
 *����:        ��       
 *���أ�       ��
 *˵����       ������Ļ����෴��
========================================================================*/
uint8 HEX2BCD(uint8 val) 
{
    uint8 i,j,k;
    i=val/10;
    j=val%10;
    k=j+(i<<4);
    return k;
} 

 /*=======================================================================
 *������:      TimeInit_DelayTime(void)
 *����:        Bootloader�е���ʱ����
 *����:        us:��ʱʱ��,��λ:us       
 *���أ�       ��
 *˵����       Bootloader�е���ʱ����
========================================================================*/
static
void DS3231_DelayTimeus(uint16 us) 
{
	  uint16 delayval;
	  delayval = us * 9;
	  while(delayval--);
}
 
 
/*=======================================================================
 *������:      DS3231SN_INIT()
 *����:        ��IICģ���������
 *����:        year:��,month:��,day:��       
 *���أ�       ��
 *˵����       �˴��õ�BCD��,��4λΪʮ���Ƶ�ʮλ,��4λΪʮ���Ƶĸ�λ
========================================================================*/
void DS3231SN_INIT(uint8 year, uint8 month, uint8 week, uint8 day, uint8 hour, uint8 min) // ���������һ������ʱ����ʹ��һ��
{
   IIC_write(0xd0,0x00,0x00);    /*��������ʼ��Ϊ0*/
   DS3231_DelayTimeus(10);
   IIC_write(0xd0,0x01,min);     /*�����ӳ�ʼ��Ϊ0*/
   DS3231_DelayTimeus(10);
   IIC_write(0xd0,0x02,hour);    /*��Сʱ����ʼ��Ϊ0*/
   DS3231_DelayTimeus(10);
   IIC_write(0xd0,0x03,week);     /*��������ʼ��Ϊ0*/
   DS3231_DelayTimeus(10);
   IIC_write(0xd0,0x04,day);    /*�����ʼ��Ϊ0*/
   DS3231_DelayTimeus(10);
   IIC_write(0xd0,0x05,month);   /*��������ʼ��Ϊ0*/
   DS3231_DelayTimeus(10);
   IIC_write(0xd0,0x06,year);    /*��������ʼ��Ϊ0*/
   DS3231_DelayTimeus(100);      //дʱ��Ͷ�ʱ��ʱ����Ҫ��ʱ����
}

/*=======================================================================
 *������:      DS3231_Read_Second()
 *����:        ������
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡ�����������59��
========================================================================*/
/*static
uint8 DS3231_Read_Second(void) 
{
  uint8 receivedata;
  receivedata=IIC_read(0xd0,0x00);
  receivedata=BCD2HEX(receivedata);
  return receivedata;
}*/
/*=======================================================================
 *������:      DS3231_Read_Minute()
 *����:        ������
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡ���������ֵ59��
========================================================================*/
/*static
uint8 DS3231_Read_Minute(void) 
{
  uint8 receivedata;
  receivedata=IIC_read(0xd0,0x01);
  receivedata=BCD2HEX(receivedata);
  return receivedata;
}*/

/*=======================================================================
 *������:      DS3231_Read_Hour()
 *����:        ��Сʱ��
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡСʱ�������ֵ��23Сʱ��
========================================================================*/
/*static
uint8 DS3231_Read_Hour(void) 
{
  uint8 receivedata;
  receivedata=IIC_read(0xd0,0x02);
  receivedata=BCD2HEX(receivedata);
  return receivedata;
}*/

/*=======================================================================
 *������:      DS3231_Read_Day()
 *����:        ������
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡ���������ֵ��31��
========================================================================*/
/*static
uint8 DS3231_Read_Day(void) 
{
  uint8 receivedata;
  receivedata=IIC_read(0xd0,0x04);
  receivedata=BCD2HEX(receivedata);
  return receivedata;
}*/

/*=======================================================================
 *������:      DS3231_Read_Month()
 *����:        ������
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡ�����������ֵ��12�£�
========================================================================*/
/*static
uint8 DS3231_Read_Month(void) 
{
  uint8 receivedata;
  receivedata=IIC_read(0xd0,0x05);
  receivedata=BCD2HEX(receivedata);
  return receivedata;
}*/

/*=======================================================================
 *������:      DS3231_Read_Year()
 *����:        ������
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡ���������ֵ��99�ꣻ
========================================================================*/
/*static
uint8 DS3231_Read_Year(void) 
{
  uint8 receivedata;
  receivedata = IIC_read(0xd0,0x06);
  receivedata = BCD2HEX(receivedata);
  return receivedata;
}*/

/*=======================================================================
 *������:      DS3231_Read_Year()
 *����:        ����ʱ��
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
 void DS3231_Read_Time(void) 
 {   
    uint8 receivedata;   
     
    receivedata=IIC_read(0xd0,0x01);
    receivedata=BCD2HEX(receivedata);
    Read_IIC_Time.IIC_Read_Minute = receivedata;
    DS3231_DelayTimeus(10);
    
    receivedata=IIC_read(0xd0,0x02);
    receivedata=BCD2HEX(receivedata);
    Read_IIC_Time.IIC_Read_Hour = receivedata;
    DS3231_DelayTimeus(10);
    
    receivedata=IIC_read(0xd0,0x04);
    receivedata=BCD2HEX(receivedata);
    Read_IIC_Time.IIC_Read_Day = receivedata;
    DS3231_DelayTimeus(10);
    
    receivedata=IIC_read(0xd0,0x05);
    receivedata=BCD2HEX(receivedata);
    Read_IIC_Time.IIC_Read_Month = receivedata;
    DS3231_DelayTimeus(10);
    
    receivedata=IIC_read(0xd0,0x06);
    receivedata=BCD2HEX(receivedata);
    Read_IIC_Time.IIC_Read_Year = receivedata;
 }

 
 
