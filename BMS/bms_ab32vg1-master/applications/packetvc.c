/*
  ******************************************************************************
  * @file    packetvc.c
  * @author  Angel_YY
  * @version V1.0.0
  * @date    2021
  * @brief   get packet voltage and current.
  ******************************************************************************
*/
#include "packetvc.h"
#include "ina237.h"
extern const INA237_Handle INA237_0;
int Pack_Init(int n)
{
    INA237_Init(n);
    return 0;
}
float GetPacketVoltage(void){

    float ret;
    ret = INA237_getVBUS_V(INA237_0);
    return ret;
}
float GetPacketCurrent(void){

    float ret;
    ret = INA237_getVSHUNT_mV(INA237_0)/SHUNTVALMV;  //I=U/R  
    return ret;
}
float GetPacketTemp(void){
    float ret;
    ret = INA237_getDIETEMP_C(INA237_0);
    return ret;
}
unsigned short GetICMID(void)
{
    return INA237_readReg(INA237_0,INA237_manufacturer_id_register);
}