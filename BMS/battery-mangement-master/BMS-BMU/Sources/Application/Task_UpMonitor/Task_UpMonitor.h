/*=======================================================================
 *Subsystem:   ���
 *File:        Task_UpMonitor.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _TASK_UP_MONITOR_H_
#define _TASK_UP_MONITOR_H_  

  #include  "TypeDefinition.h"  
  #include  "CAN.h"
  /********BMS to Up Monitor information CSSU*******/

  #define    BMS_Send_Information1   (0x18FF9700+NUM_pack)
  #define    BMS_Send_Information2   (0x18FF9710+NUM_pack)
  #define    BMS_Send_Information3   (0x18FF9800+NUM_pack)
  #define    BMS_Send_Information4   (0x18FF9810+NUM_pack)
  #define    BMS_Send_Information5   (0x18FF9900+NUM_pack)
  #define    BMS_Send_Information6   (0x19FF9900+NUM_pack)
  #define    BMS_Send_Information7   (0x18FF9600+NUM_pack)

 //��λ�����ݱ궨
 //��ط�ֵ�궨
  //#define    Monitor_BMS_Volt   0x1810C0F4     //��ص�ѹ��ֵ��Ϣ        
  typedef struct 
  { //00
    uint16  Volt_Cell_High1;  //�����ѹ��ѹһ����ֵ�� ��λ 0.0001V/Bit
    uint16  Volt_Cell_High2;  //�����ѹ��ѹ������ֵ�� ��λ 0.0001V/Bit
    //01
    uint16  Volt_Cell_Low1; //�����ѹǷѹһ����ֵ�� ��λ 0.0001V/Bit
    uint16  Volt_Cell_Low2; //�����ѹǷѹ������ֵ�� ��λ 0.0001V/Bit
    //02
    uint16  Volt_Sys_High1;  //������ѹ��ֵ1��
    uint16  Volt_Sys_High2;  //������ѹ��ֵ2��
    //03
    uint16  Volt_Sys_Low1; //�����Ƿѹ��ֵ1��
    uint16  Volt_Sys_Low2; //�����Ƿѹ��ֵ2��
    //04
    uint16  Volt_Cell_Diff1;  //����ѹ��һ����ֵ�� ��λ 0.0001V/Bit
  
  }BMSMonitor_Volt_T;
  extern BMSMonitor_Volt_T g_BMSMonitor_Volt;
  
  //#define    Monitor_BMS_Temp   0x1811C0F4     //�¶���ֵ��Ϣ 
  typedef struct 
  { //00
    uint8  Temp_Charge_High1;   //�������¶ȹ���һ����ֵ�� ��λ 1��/Bit  ƫ������-40
    uint8  Temp_Charge_High2;   //�������¶ȹ��¶�����ֵ�� ��λ 1��/Bit  ƫ������-40
    
    uint8  Temp_Charge_Low1;  //�������¶ȵ���һ����ֵ�� ��λ 1��/Bit  ƫ������-40
    uint8  Temp_Charge_Low2;  //�������¶ȵ��¶�����ֵ�� ��λ 1��/Bit  ƫ������-40
    
    //01
    uint8  Temp_DisCharge_High1;    //����ŵ��¶ȹ���һ����ֵ�� ��λ 1��/Bit   ƫ������-40
    uint8  Temp_DisCharge_High2;    //����ŵ��¶ȹ��¶�����ֵ�� ��λ 1��/Bit   ƫ������-40
    
    uint8  Temp_DisCharge_Low1;   //����ŵ��¶ȵ���һ����ֵ�� ��λ 1��/Bit   ƫ������-40
    uint8  Temp_DisCharge_Low2;   //����ŵ��¶ȵ��¶�����ֵ�� ��λ 1��/Bit   ƫ������-40

    //02
    uint8  Temp_Charge_Diff1;     //�������²�һ����ֵ�� ��λ 1��/Bit    ƫ������-40
  
    uint8  Temp_DisCharge_Diff1;  //����ŵ��²�һ����ֵ�� ��λ 1��/Bit    ƫ������-40
 
  }BMSMonitor_Temp_T;
  extern BMSMonitor_Temp_T g_BMSMonitor_Temp;
 
  //#define    Monitor_BMS_Current   0x1812C0F4     //������ֵ��SOC��ֵ��Ϣ 
  typedef struct 
  { 
    //00
    uint16  Current_DisCharge_High1;  //�ŵ������ֵ1����0.1A/λ     ƫ����:-750
    uint16  Current_DisCharge_High2;  //�ŵ������ֵ2����0.1A/λ     ƫ����:-750

    //01
    uint16  Current_Charge_High1;     //��������ֵ1����0.1A/λ      ƫ����:-750
    uint16  Current_Charge_High2;     //��������ֵ2����0.1A/λ      ƫ����:-750
    
  }BMSMonitor_Curr_T;
  extern BMSMonitor_Curr_T g_BMSMonitor_Curr;
  
  //#define    Monitor_BMS_Insulation   0x1813C0F4     //��Ե������ֵ��Ϣ 
  typedef struct 
  { //00
    uint16 Insulation_Resis1;     // ��Ե����һ����ֵ�� ��λ 0.1K��/Bit;
    uint16 Insulation_Resis2;     // ��Ե���������ֵ�� ��λ 0.1K��/Bit;
    uint16 Insulation_Resis3;     // ��Ե����������ֵ�� ��λ 0.1K��/Bit;  
  
  }BMSMonitor_Insu_T;
  extern BMSMonitor_Insu_T g_BMSMonitor_Insul; 
  
  //��������λ����SOC��ʼ��/�����
  /*typedef struct 
  { 
    uint8   SOC_t; //���ڷ�������λ����ͷ�ļ���  
  
  }BMSMonitor_SOC_T;
  extern BMSMonitor_SOC_T g_BMSMonitor_SOC;*/ 
  
  //��������λ����SOHֵ
  typedef struct 
  { 
    float  SOH; //���ڷ�������λ����ͷ�ļ���  
  
  }BMSMonitor_SOH_T;
  extern BMSMonitor_SOH_T g_BMSMonitor_SOH; 
  
  //0x1814C0F4
  typedef struct
  {
     uint16 Voll_Sys_Low1_LT;
     uint16 Voll_Sys_Low2_LT;
     uint16 Volt_Cell_Low1_LT; //�����ѹǷѹһ����ֵ�� ��λ 0.0001V/Bit
     uint16 Volt_Cell_Low2_LT; //�����ѹǷѹ������ֵ�� ��λ 0.0001V/Bit
  
  }BMSMonitor_New_LT_T;
  extern BMSMonitor_New_LT_T g_BMSMonitor_New_LT;

//---------------------------------------------------------------------------------------------------- 
  
  //#define    Monitor_BMS_SOC   0x1915F4C0         //��������Ϣ�궨˵��
  typedef struct 
  { //00
    uint16  SOC_Init;             //SOC��ʼֵ�� ��λ 1%/Bit��
    union
    {
     int  Relay1_Station;       //�̵������ر���
      struct 
      {
        int Relay_Positive:2;    //�����̵���
        int Relay_Negtive:2;     //�����̵���
        int Relay_Precharge:2;   //Ԥ��̵���
        int Relay_Charge:2;      //���̵���
        
      }Monitor_BMS_DelayBit;
    }Monitor_BMS_Delay_T;
    
    uint16 Ref_Volt;               //�ο���ѹ�궨(0~65536)
    uint16 Ref_Current;            //������������б��
    //01
    uint16 Volt_Resis;             //��ѹ����з�ѹ����ı궨
    uint32 Pack_Total_Capacity;    //�궨SOH�е�������(2000*�����)
    //02
    uint16 BMS_Running_Time;       //BMS����ʱ��
      
  }MonitorBMS_SOC_T;
  extern MonitorBMS_SOC_T g_MonitorBMS_SOC;
   
  //#define    Monitor_to_BMSFF   0x19FFF4C0
  typedef struct 
  {
    uint8 Msg_StarUpload;              //BMS����λ��ͨ��ʼ���źţ�0xAA�� ͨ�ſ�ʼ�� ������ ������ͨ�ţ�
    uint8 Threshold_StarUpload;        //���ݸ��� 1~5�� ��1 ���� 500ms��  
  }MonitorBMS_Start_T;
  extern MonitorBMS_Start_T g_MonitorBMS_Start; 

  void Init_UpMonitor(void); 

  uint8 CAN_ToUpMonitor(pCANFRAME);
  uint8 CAN_UpMonitorInit(void);
  void Task_BMSToUpMonitor(void);
  void Task_BMUToUpMonitor(void);

  void UpMonitor_to_Bms(pCANFRAME);

#endif