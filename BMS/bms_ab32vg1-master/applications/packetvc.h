/*
  ******************************************************************************
  * @file    packetvc.h
  * @author  Angel_YY
  * @version V1.0.0
  * @date    2021
  * @brief   get packet voltage and current.
  ******************************************************************************
*/

#ifndef __PACKETVC_H__
#define __PACKETVC_H__

#define SHUNTVALMV  10.0f  //定义分流电阻的阻值，单位mΩ
int Pack_Init(int n);
float GetPacketVoltage(void);
float GetPacketCurrent(void);
float GetPacketTemp(void);
unsigned short GetICMID(void);
#endif