/*=======================================================================
 *Subsystem:   ���
 *File:        Task_DataProcess.c
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:  
========================================================================*/
#include  "includes.h"
VoltInfo_T        g_VoltInfo;
TempInfo_T        g_TempInfo;
DataColletInfo_T  g_DataColletInfo; 
OpenWireInfo_T    g_OpenWireInfo;
InsulInfo_T       g_InsulInfo;

static void DataProcess_Volt(void);
static void DataProcess_Temp(void);
static void DataProcess_OpenWire(void);

/*=======================================================================
 *������:      Init_TaskDataProcess(void) 
 *����:        �ɼ���������ѹ���¶ȵ���Ϣ
 *����:        ��       
 *���أ�       ��
 *˵����       ����SOCʱ�����õĵ���ֵ,����Ҫ���ǻ����������Ƿ���һ����
========================================================================*/
void Init_TaskDataProcess(void)                       
{
  //�����Ϣ��������
  memset(&g_VoltInfo,       0, sizeof(VoltInfo_T));        //��ص�ѹ��������
  memset(&g_TempInfo,       0, sizeof(TempInfo_T));        //����¶���������
  memset(&g_InsulInfo,      0, sizeof(InsulInfo_T));       //��Ե��Ϣ����
  memset(&g_OpenWireInfo,   0, sizeof(OpenWireInfo_T));    //���߿�·��Ϣ����
  memset(&g_DataColletInfo, 0, sizeof(DataColletInfo_T));  //�ɼ���������
}


/*=======================================================================
 *������:      Task_DataProcess(void) 
 *����:        �ɼ���������ѹ���¶ȵ���Ϣ
 *����:        ��       
 *���أ�       ��
 *˵����       ����SOCʱ�����õĵ���ֵ,����Ҫ���ǻ����������Ƿ���һ����
========================================================================*/
void Task_DataProcess(void)                       
{
  DataProcess_Volt();
  DataProcess_Temp();
  DataProcess_OpenWire();
  
  g_Roll_Tick.Roll_DataPro++;
}

/*=======================================================================
 *������:      DataProcess_Volt()  
 *����:        �ɼ���ѹ��Ϣ
 *����:        ��       
 *���أ�       ��
 
 *˵����        
========================================================================*/
static
void DataProcess_Volt(void)
{
  uint8   i;
  
  //��ȡ0~49�ڵ�صĵ�ѹ
  for(i=0; i<NUM_Battery; i++) 
  {
    g_VoltInfo.CellVolt[i] = g_LTC6811_VoltInfo.CellVolt[i]; //����ĵ�ѹ��Ϣ��ǰ 
  }
  for(i=0; i<SYS_SERIES_YiDongLi; i++) 
  {
    g_VoltInfo.CellVolt[i+NUM_Battery] = g_FromCSSU_Volt.CellVolt[i];  
  }
  
  //��ȡ����ѹ�Լ�����λ��
  if (g_LTC6811_VoltInfo.CellVolt_Max >= g_FromCSSU_Volt.CellVolt_Max) 
  {
     g_VoltInfo.CellVolt_Max     = g_LTC6811_VoltInfo.CellVolt_Max;      
     g_VoltInfo.CellVolt_MaxNode = g_LTC6811_VoltInfo.CellVolt_MaxNode;
  }
  else
  {
     g_VoltInfo.CellVolt_Max     = g_FromCSSU_Volt.CellVolt_Max;
     g_VoltInfo.CellVolt_MaxNode = NUM_Battery + g_FromCSSU_Volt.CellVolt_MaxNode;
  }
  //��ȡ��С��ѹ�Լ�����λ��
  if (g_LTC6811_VoltInfo.CellVolt_Min <= g_FromCSSU_Volt.CellVolt_Min) 
  {
     g_VoltInfo.CellVolt_Min     = g_LTC6811_VoltInfo.CellVolt_Min;      
     g_VoltInfo.CellVolt_MinNode = g_LTC6811_VoltInfo.CellVolt_MinNode;
  }  
  else
  {
     g_VoltInfo.CellVolt_Min = g_FromCSSU_Volt.CellVolt_Min;
     g_VoltInfo.CellVolt_MinNode = NUM_Battery + g_FromCSSU_Volt.CellVolt_MinNode;
  }  
      
  g_VoltInfo.SysVolt_Total = (g_LTC6811_VoltInfo.CellVolt_Total + g_FromCSSU_Volt.CSSUVolt_Total)/2;                                 //ϵͳ��ѹ

  g_VoltInfo.CellVolt_Diff  = g_VoltInfo.CellVolt_Max - g_VoltInfo.CellVolt_Min;
  g_VoltInfo.CellVolt_Ave   = g_VoltInfo.SysVolt_Total/SYS_SERIES_YiDongLi;
} 

/*=======================================================================
 *������:      DataProcess_Temp(void)  
 *����:        �ɼ���ѹ��Ϣ
 *����:        ��       
 *���أ�       ��
 
 *˵����        
========================================================================*/
static
void DataProcess_Temp(void) 
{
  uint8   i;
  
  //��ȡ0~24�ڵ���¶���Ϣ
  for(i=0; i<SYS_NUMBER_MODULE_TEMP; i++)
  {
    g_TempInfo.CellTemp[i] = g_LTC6811_TempInfo.CellTemp[i];
  }
  for(i=0; i<SYS_NUMBER_MODULE_TEMP; i++)
  {
    g_TempInfo.CellTemp[i+SYS_NUMBER_MODULE_TEMP] = g_FromCSSU_Temp.CellTemp[i];
  }
  
  //��ȡ����¶ȼ�λ��
  if (g_LTC6811_TempInfo.CellTemp_Max >= g_FromCSSU_Temp.CellTemp_Max) 
  {
    g_TempInfo.CellTemp_Max      = g_LTC6811_TempInfo.CellTemp_Max;
    g_TempInfo.CellTemp_MaxNode  = g_LTC6811_TempInfo.CellTemp_MaxNode;
  } 
  else
  {
    g_TempInfo.CellTemp_Max      = g_FromCSSU_Temp.CellTemp_Max;
    g_TempInfo.CellTemp_MaxNode  = SYS_NUMBER_MODULE_TEMP+g_FromCSSU_Temp.CellTemp_MaxNode;
  }
  
  //��ȡ��С�¶ȼ�λ��
  if (g_LTC6811_TempInfo.CellTemp_Min <= g_FromCSSU_Temp.CellTemp_Min) 
  {
    g_TempInfo.CellTemp_Min      = g_LTC6811_TempInfo.CellTemp_Min;
    g_TempInfo.CellTemp_MinNode  = g_LTC6811_TempInfo.CellTemp_MinNode;
  } 
  else
  {
    g_TempInfo.CellTemp_Min      = g_FromCSSU_Temp.CellTemp_Min;
    g_TempInfo.CellTemp_MinNode  = SYS_NUMBER_MODULE_TEMP+g_FromCSSU_Temp.CellTemp_MinNode;
  }    
       
  g_TempInfo.CellTemp_Ave  = (g_LTC6811_TempInfo.CellTemp_Ave + g_FromCSSU_Temp.CellTemp_Ave)/2;                                 //ϵͳƽ���¶�
  g_TempInfo.CellTemp_Diff = g_TempInfo.CellTemp_Max - g_TempInfo.CellTemp_Min;
}

/*=======================================================================
 *������:      DataProcess_OpenWire(void)  
 *����:        �ɼ���ѹ��Ϣ
 *����:        ��       
 *���أ�       ��
 
 *˵����        
========================================================================*/
static
void DataProcess_OpenWire(void) 
{
   g_OpenWireInfo.OpenWire_Status = g_LTC6811_OpwireInfo.OpenwireErr|\
                                    g_FromCSSU_FltData.OpenWire_Status;
}





