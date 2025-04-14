#ifndef _LTC6811_TEMPCOLLECT_H_
#define _LTC6811_TEMPCOLLECT_H_

  #include  "TypeDefinition.h"
  #include  "LTC6811_CMDConfig.h"
  #include  "LTC6811_ConnectType.h"  

  typedef struct
  {
    uint8  CellTemp[NUM_Tem];
    uint8  CellTemp_Max;        //ƫ����Ϊ40
    uint8  CellTemp_Min;        //ƫ����Ϊ40
    uint8  CellTemp_MaxNode;    //����¶Ƚڵ�
    uint8  CellTemp_MinNode;    //����¶Ƚڵ�
    uint8  CellTemp_Ave;        //ƫ����Ϊ40
    uint8  ICTemp[NUM_IC];      //ƫ����Ϊ40
    uint8  ICTemp_OverState;    //оƬ�¶��Ƿ񳬱�
    uint32 CellTemp_Tatoltemp;  //ƫ����Ϊ40*NUM_Tem
  }LTC6811_TempInfo_T;
  extern LTC6811_TempInfo_T g_LTC6811_TempInfo;

  /* �¶ȴ������� */

  void LTC6811_TempCMDSend(void);

  void LTC6811_TempCollect(void);

  void LTC6811_ChipTempCMDSend(void);

  void LTC6811_ChipTempCollect(void);



#endif