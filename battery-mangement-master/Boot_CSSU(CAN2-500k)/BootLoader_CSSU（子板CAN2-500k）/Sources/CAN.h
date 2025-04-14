#ifndef _CAN_H
#define _CAN_H

/*=======================================================================
 *������:      Boot��ID˵��
 *����:        ˵��boot���������з��ͺͷ��ص�ID��,������
 *����:        ��
 *���أ�       �� 
 *˵����       1.������Ӱ��Ӧ�ó�����������ID��ͳһΪ:0xF300(��չ֡)
                 ����:Data[0]=0xF0,Data[1] =0xAA
                 �Ӱ�:Data[0]=0,1,2���������,Data[1] =0xAA
               2.������Boot���غ�ֱ��������IDΪ:0x001(��׼֡),boot���򷵻�����
                 ��ID��Ϊ:0x1FF,Data[0]=C3��ʾ����boot�п�������,֮�󷵻ص�����
                 Data[0]=0,��������һ��S19�ļ��ɹ�,������һ��S19�ļ�����ʧ�ܡ�
               3.�Ӱ���Boot���غ�ֱ��������IDΪ:0x002(��׼֡)�Դ�����,boot���򷵻�����
                 ��ID��Ϊ:0x1FF,Data[0]=C3��ʾ����boot�п�������,֮�󷵻ص�����
                 Data[0]=0,��������һ��S19�ļ��ɹ�,������һ��S19�ļ�����ʧ�ܡ� 
========================================================================*/
//#define ID_BMU 	    0x0001//����ID�̶����� 
#define ID_Reply    0x01FF//������Ӱ�һ��
#define ID01_6811   0x0002//�Ӱ��ID���Դ�����
//����ΪӦ�ó�����������ID
//#define  Boot_ID        0xF300   //��չ֡

 
typedef struct                    //���ͱ��ĵĽṹ��
{
    unsigned long id;             //ID��
    Bool RTR;                     //�Ƿ�ΪԶ����
    unsigned char data[8];        //�����������
    unsigned char len;            //can���͵����ݳ���Ϊ8  
    unsigned char prty;           //CANͨ�Ŵ������ȼ��趨
} CAN_MSG;

extern CAN_MSG can_send;

#pragma CODE_SEG RAM_CODE
extern  interrupt void CAN2RxISR(void); 
#pragma CODE_SEG DEFAULT

extern INT8 getchar(void);
extern void INIT_CAN2(void); 
extern Bool MSCAN2GetMsg(CAN_MSG *msg);
extern Bool MSCAN2SendMsg(CAN_MSG msg);
#endif