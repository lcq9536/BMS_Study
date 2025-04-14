/*=======================================================================
 *Subsystem:   ���
 *File:        Includes.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/
       
#ifndef   _INCLUDES_H_
#define   _INCLUDES_H_
                   
  //XEP100�ڲ��ļ�����                         
  #include  "MC9S12XEP100.h"
  #include  "hidef.h"
  #include  "stddef.h"
  #include  "libdefs.h"
  #include  "derivative.h"
  //�ⲿ�ļ�����
  #include  "TypeDefinition.h"
  #include  "BattInfoConfig.h"
  //Ӧ�ò�ͷ�ļ�
  #include  "Init_Sys.h"
  #include  "Task_Init.h"
  #include  "Task_BootLoader.h"
  #include  "Task_PowerOnOff.h"
  #include  "WorkModeJudge.h"
  #include  "BMSCheckSelf.h"
  #include  "BMSCheckSelf_UpMonitor.h"
  #include  "Task_VoltTempCollect.h"
  #include  "DataFromCSSU.h"
  #include  "Task_DataProcess.h"
  #include  "Task_SOCSOH.h"
  #include  "Task_CurrLimit.h"
  #include  "Task_FltLevJudg.h"
  #include  "FltLevcfg.h"
  #include  "Task_InsulDetect.h"
  #include  "Task_UpMonitor.h"
  #include  "Task_Charge.h"
  #include  "Task_EEEReadWrite.h"
  #include  "Task_FltSave.h"
  #include  "Task_SysTimeGet.h"
  #include  "Task_Screen.h"
  #include  "Task_BalanceControl.h"
  
  //uC��ͷ�ļ�
  #include  "ADC.h"
  #include  "ADC_cfg.h"
  #include  "CAN.h"
  #include  "Flash.h"
  #include  "GPIO.h"
  #include  "Port_Control.h"
  #include  "IIC.h"
  #include  "PIT.h"
  #include  "PLL.h"
  #include  "SCI.h"
  #include  "SPI.h"
  //ECU�����
  #include  "Init_ADC.h"
  #include  "Init_PIT.h"
  #include  "Init_PLL.h"
  #include  "Init_Flash.h"
  #include  "Init_IIC.h"
  #include  "DS3231_TimeGet.h"
  #include  "LTC6811_ConnectType.h"
  #include  "LTC6811_CMDConfig.h"
  #include  "LTC6811_CMD.h"
  #include  "LTC6811_Init.h"
  #include  "LTC6811_OpWire.h"
  #include  "LTC6811_PassBalance.h"
  #include  "LTC6811_TempCollect.h"
  #include  "LTC6811_VoltCollect.h"
  //�⺯��ͷ�ļ�
  #include  "Filter_Function.h"
  
  
    
#endif