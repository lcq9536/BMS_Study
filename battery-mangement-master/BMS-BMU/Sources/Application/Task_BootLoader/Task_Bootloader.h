#ifndef  _TASK_BOOT_H_
#define  _TASK_BOOT_H_

  #include "TypeDefinition.h"
  
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
  #define  Boot_ID        0xF300   //��չ֡
  #define  BMU_IDNUM      0xF0     //���������ĵ�һ���ֽڱ��
  //����ΪBoot�����е�ID����
  //#define ID_BMU 	    0x0001//����ID�̶����� 
  //#define ID_Reply    0x01FF//������Ӱ�һ��
  //#define ID01_6811   0x0002//�Ӱ��ID���Դ�����
  //#define ID02_6811   0x0003//�Ӱ��ID���Դ�����

  typedef struct
  {
     uint16 *Boot;
     uint8  OnlineUpdateCheck;
  }Boot_Data_T;
  extern  Boot_Data_T Boot_Data;
  
  void Task_BootLoader();  
  
#endif