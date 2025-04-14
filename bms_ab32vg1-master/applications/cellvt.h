/*
  ******************************************************************************
  * @file    cellvt.h
  * @author  Angel_YY
  * @version V1.0.0
  * @date    2021
  * @brief   get cell voltage and temperature.
  ******************************************************************************
*/

#ifndef __CELLVT_H__
#define __CELLVT_H__
#include "stdint.h"
#include "bms.h"
int LTC6811_init();
int GetCellV(Packet_t * pMyPacket);
int GetCellT(Packet_t * pMyPacket);
int GetCellTest(Packet_t * pMyPacket);
void ProcessData(Packet_t * pMyPacket,Packet_Alarm_Val_t * pstAlarm,uint32_t *puDataOverFlag);
int CheckWarning(void);
void PrintCellMsg(void);
#endif
