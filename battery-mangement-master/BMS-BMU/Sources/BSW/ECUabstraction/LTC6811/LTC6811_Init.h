#ifndef _LTC6811_INIT_H_
#define _LTC6811_INIT_H_

  #include  "TypeDefinition.h"
  #include  "LTC6811_CMDConfig.h"
  #include  "LTC6811_ConnectType.h"

  //�¶Ȳɼ�(���ڶ�·������)
  //void messagedata_temp();
  
  extern uint8 CFG1[NUM_IC][6];

  //LTC6811��ʼ������
  void  Config_Fun(uint8,uint8, uint8,uint8, uint8,\
                   float,float,uint16,uint8);
  uint8 LTC6804_Init(void);

#endif