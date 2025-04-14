#ifndef _LTC6811_OPENWIREDETECT_
#define _LTC6811_OPENWIREDETECT_
 
  #include  "TypeDefinition.h"
  #include  "LTC6811_CMD.h"
  #include  "LTC6811_CMDConfig.h"
  #include  "LTC6811_ConnectType.h"                     //����Num_IC������
  //�����ȫ�ֱ���
  typedef struct
  {
    uint16 OpenwireLocation[NUM_IC];//���߿�·�ľ���λ��
    uint8  OpenwireErr;             //���߿�·����״̬
  }LTC6811_OpwireInfo_T;
  extern LTC6811_OpwireInfo_T g_LTC6811_OpwireInfo;

  void LTC6811_OpenwireDetect(void);
  
#endif