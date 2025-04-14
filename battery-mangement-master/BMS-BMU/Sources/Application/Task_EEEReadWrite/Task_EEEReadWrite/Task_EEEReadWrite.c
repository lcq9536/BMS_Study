/*=======================================================================
 *Subsystem:   ���
 *File:        Task_EEE_ReadWrite.C
 *Author:      Wenming
 *Description: �洢����Ϊ�����ȶ�ȡEEP�е�EEprom_Data.pEErom_baseֵ��ȷ����
               �洢λ�ã������ô�ֵ���������洢�����ٶ�ȡ����ֵ
 *========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    2018 - 01 - 03
      Author:  Wenm
      Modification: 
========================================================================*/
#include  "includes.h"
EEprom_Data_T EEprom_Data; 

static void EEprom_read(uint16 addr, uint8 addrbase);
/*=======================================================================
 *������:      Get_EEprom_Value(void)
 *����:        ��ʼ����ȡEEprom��ֵ
 *����:        ��       
 *���أ�       ��
 *˵����       ��0x0D���16����ַ�д洢��ַ�������洢λ�ð���ֵ����λ��0x0D00~0x0D10��
               �����Ҫ���г�ʼ������ʱ����Get_EEprom_Value�е�Ԥ�����Ϊ1��
========================================================================*/ 
void Get_EEprom_Value() 
{
    uint8 j,addrError=0;
    uint8*ptr=(uint8*)EEprom_Ptr;
    
    for(j=0; j<EEprom_Length; j++)//�������λ�洢����ֵ
    {
      if(ptr[j] != 0xFF)        //�жϱ��λ�洢�����ֵ����Ч��
      {
         addrError++; 
         EEprom_Data.pEErom_base = ptr[j];  
      }            
    } 
    #if(RESET_EEPROM==1)   //�˴������Ƿ���Ҫ������ֵ��ʼ��
       addrError = 0;
    #endif         
            
    if(addrError == 1) //�ж�EEprom�洢����ȷ�ԣ�ֻ��һ�ζ�����0xffֵΪ��ȷ
    {
      EEprom_Data.EE_Value = 1;            
      EEprom_read(EEprom_Baseadrr, EEprom_Data.pEErom_base); //��ȡ���ε�ֵ
    } 
    else //����ȫ��0xff��ֵʱ����оƬ��һ��д�룩������ֹ��ϣ���Ҫ�������ݳ�ʼ��
    {
      EEprom_Data.EE_Value = 2;
      memset((uint8*)EEprom_Ptr, 0xFF, EEprom_Length*sizeof(uint8));
      memset((uint8*)EEprom_Baseadrr, 0xFF, EEprom_Length*Elem_Num*sizeof(uint8));
      EEprom_DateMemset();        //�������³�ʼ��
      EEprom_Data.pEErom_base = EEprom_Length-1;  //���洢���λ��λ��15����ַ�У��״α�����0x08OO��
    } 
} 
 
    
 /*=======================================================================
 *������:      EEprom_DateMemset
 *����:        EEprom�ڴ����ݳ�ʼ��
 *����:        ��                
 *���أ�       ��
 *˵����       ��һЩ�洢����ȫ�����г�ʼ���������ã�������ʱ�ӳ�ʼ��������Ҫ��
========================================================================*/ 

 void EEprom_DateMemset(void) 
 {
   //�����洢���ֽ�   
   EEprom_Data.Charge_Times = 0;
   g_SysTime.BMS_TotalRun_MiniteTime = 0; 
   g_SysTime.BMS_PowerOff_Time = 0;
   g_EnergyInfo.Energy_Total_Charge = 0;
   g_EnergyInfo.Energy_Total_DisCharge = 0;
   g_SOCInfo.SOC_LowestVoltGet = 0;
   g_SOCInfo.SOC_HighestVoltGet = 0;
   g_SOCInfo.SOC_ValueRead = 0;
   g_SOCInfo.SOC_ValueVoltGet = 0;
   
   //�Ǿ����洢���ֽ�
   //0x1810C0F4 ��ص�ѹ������ֵ��Ϣ
   //�����ѹ 00
   *(uint16*)0x0CA0 = F1_CHARGE_VOLTCH;   //�����ѹ��ѹһ����ֵ�� ��λ 0.0001V/Bit
   *(uint16*)0x0CA2 = F2_CHARGE_VOLTCH;   //�����ѹ��ѹ������ֵ�� ��λ 0.0001V/Bit
   
   //����Ƿѹ 01
   *(uint16*)0x0CA8 = F1_DISCHG_VOLTCL_NT;  //�����ѹǷѹһ����ֵ�� ��λ 0.0001V/Bit
   *(uint16*)0x0CAA = F2_DISCHG_VOLTCL_NT;  //�����ѹǷѹ������ֵ�� ��λ 0.0001V/Bit
   
   //ϵͳ��ѹ 02
   *(uint16*)0x0CB0 = F1_CHARGE_VOLTSH*0.001; //������ѹ��ֵ1��
   *(uint16*)0x0CB2 = F2_CHARGE_VOLTSH*0.001; //������ѹ��ֵ2��
   
   //ϵͳǷѹ 03
   *(uint16*)0x0CB8 = F1_DISCHG_VOLTSL_NT*0.001;  //�����Ƿѹ��ֵ1��
   *(uint16*)0x0CBA = F2_DISCHG_VOLTSL_NT*0.001;  //�����Ƿѹ��ֵ2��
   
   //����ѹ�� 04
   *(uint16*)0x0CC0 = F1_DISCHG_VOLTCD;    //����ѹ��һ����ֵ�� ��λ 0.0001V/Bit;
  
   //0x1811C0F4 ����¶ȱ�����ֵ
   //����磩��о�¶���ֵ 00
   //����
   *(uint8*)0x0CD0 = F1_CHARGE_TEMPH;    //�����¶ȹ���һ����ֵ�� ��λ 1��/Bit  ƫ������-40
   *(uint8*)0x0CD1 = F2_CHARGE_TEMPH;    //�����¶ȹ��¶�����ֵ�� ��λ 1��/Bit  ƫ������-40
   //����
   *(uint8*)0x0CD3 = F1_CHARGE_TEMPL;   //�����¶ȵ���һ����ֵ�� ��λ 1��/Bit  ƫ������-40
   *(uint8*)0x0CD4 = F2_CHARGE_TEMPL;   //�����¶ȵ��¶�����ֵ�� ��λ 1��/Bit  ƫ������-40
   
   //���ŵ磩��о�¶���ֵ 01
   //����
   *(uint8*)0x0CD8 = F1_DISCHG_TEMPH;    //�����¶ȹ���һ����ֵ�� ��λ 1��/Bit   ƫ������-40
   *(uint8*)0x0CD9 = F2_DISCHG_TEMPH;    //�����¶ȹ��¶�����ֵ�� ��λ 1��/Bit   ƫ������-40
   //����
   *(uint8*)0x0CDB = F1_DISCHG_TEMPL;   //�����¶ȵ���һ����ֵ�� ��λ 1��/Bit   ƫ������-40
   *(uint8*)0x0CDC = F2_DISCHG_TEMPL;   //�����¶ȵ��¶�����ֵ�� ��λ 1��/Bit   ƫ������-40
   
   //����²� 02
   //���
   *(uint8*)0x0CE0 = F1_CHARGE_TEMPD;   //�������²�һ����ֵ�� ��λ 1��/Bit    ƫ������-40
   //�ŵ�
   *(uint8*)0x0CE3 = F1_DISCHG_TEMPD;   //����ŵ��²�һ����ֵ�� ��λ 1��/Bit    ƫ������-40
   
   //0x1812C0F4 ��ص�����ֵ��SOC��ֵ��Ϣ
   //ϵͳ�������ŵ磩 00
   *(uint16*)0x0CF0 = (F1_DISCHG_CURRH + 750)*10;      //�ŵ������ֵ1����0.1A/λ     ƫ����:-750
   *(uint16*)0x0CF2 = (F2_DISCHG_CURRH + 750)*10;      //�ŵ������ֵ2����0.1A/λ     ƫ����:-750
   
   //ϵͳ��������磩 01
   *(uint16*)0x0CF8 = (F1_CHARGE_CURRH + 750)*10;      //��������ֵ1����0.1A/λ      ƫ����:-750
   *(uint16*)0x0CFA = (F2_CHARGE_CURRH + 750)*10;      //��������ֵ2����0.1A/λ      ƫ����:-750
   
   //0x1813C0F4 ��Ե������ֵ
   *(uint16*)0x0D0A = F2_DISCHG_INSUL*10;              // ��Ե���������ֵ�� ��λ 0.1K��/Bit;
   
   //0x1915F4C0 ��������Ϣ
   *(uint16*)0x0D10 = 0;                          //SOC_init       ��λ 1%/Bit
   *(uint16*)0x0D12 = STAND_VOLT*10000;           //Ref_Volt       �ο���ѹ�궨(0~65536)
   *(uint16*)0x0D14 = HALL_RANGE;                 //Ref_Current    ������������б��
   
   *(uint32*)0x0D1A = 2000*SYS_CAPACITY*10;        //Pack_Total_Capacity     �궨SOH�е�������(2000*�����)
   
   *(uint16*)0x0D20 = g_SysTime.BMS_TotalRun_MiniteTime;        //��λ���±굥λΪСʱ�洢Ϊ����
   /*
   //0x1814C0F4 �����������ڴ�ŵ�����ֵ
   //00
   *(uint16*)0x0D28 = F1_DISCHG_VOLTSL_LT*0.001; 
   //01
   *(uint16*)0x0D30 = F1_DISCHG_VOLTCL_LT;    */
   
   
   
   //0x1810C0F4 ��ص�ѹ������ֵ
   //�����ѹ 00
   g_BMSMonitor_Volt.Volt_Cell_High1 = (*(uint16*)(0x0CA0));
   g_BMSMonitor_Volt.Volt_Cell_High2 = (*(uint16*)(0x0CA2));
   
   //����Ƿѹ 01
   g_BMSMonitor_Volt.Volt_Cell_Low1 = (*(uint16*)(0x0CA8));
   g_BMSMonitor_Volt.Volt_Cell_Low2 = (*(uint16*)(0x0CAA));

   //ϵͳ��ѹ 02
   g_BMSMonitor_Volt.Volt_Sys_High1 = (*(uint16*)(0x0CB0));
   g_BMSMonitor_Volt.Volt_Sys_High2 = (*(uint16*)(0x0CB2));
   
   //ϵͳǷѹ 03 
   g_BMSMonitor_Volt.Volt_Sys_Low1 = (*(uint16*)(0x0CB8));
   g_BMSMonitor_Volt.Volt_Sys_Low2 = (*(uint16*)(0x0CBA));
   
   //����ѹ�� 04
   g_BMSMonitor_Volt.Volt_Cell_Diff1 = (*(uint16*)(0x0CC0));
  
   //0x1811C0F4 ����¶ȱ�����ֵ
   //����о�¶� 00
   //����
   g_BMSMonitor_Temp.Temp_Charge_High1 = (*(uint8*)(0x0CD0));
   g_BMSMonitor_Temp.Temp_Charge_High2 = (*(uint8*)(0x0CD1));

   //����
   g_BMSMonitor_Temp.Temp_Charge_Low1 = (*(uint8*)(0x0CD3));
   g_BMSMonitor_Temp.Temp_Charge_Low2 = (*(uint8*)(0x0CD4));
   
   //�ŵ��о�¶� 01
   //����
   g_BMSMonitor_Temp.Temp_DisCharge_High1 = (*(uint8*)(0x0CD8));
   g_BMSMonitor_Temp.Temp_DisCharge_High2 = (*(uint8*)(0x0CD9));
   //����
   g_BMSMonitor_Temp.Temp_DisCharge_Low1 = (*(uint8*)(0x0CDB));
   g_BMSMonitor_Temp.Temp_DisCharge_Low2 = (*(uint8*)(0x0CDC));
   
   //����²� 02
   //���
   g_BMSMonitor_Temp.Temp_Charge_Diff1 = (*(uint8*)(0x0CE0));
   //Monitor_BMS_Temp.Charge_Diff2_Temp = (*(uint8*)(0x0CE1));
   //Monitor_BMS_Temp.Charge_Diff3_Temp = (*(uint8*)(0x0CE2));
   //�ŵ�
   g_BMSMonitor_Temp.Temp_DisCharge_Diff1 = (*(uint8*)(0x0CE3));
 
   //0x1812C0F4 ��ص�����ֵ��Ϣ
   //ϵͳ�ŵ���� 00
   g_BMSMonitor_Curr.Current_DisCharge_High1 = (*(uint16*)(0x0CF0));
   g_BMSMonitor_Curr.Current_DisCharge_High2 = (*(uint16*)(0x0CF2));
   
   //ϵͳ������ 01
   g_BMSMonitor_Curr.Current_Charge_High1 = (*(uint16*)(0x0CF8));
   g_BMSMonitor_Curr.Current_Charge_High2 = (*(uint16*)(0x0CFA));
   
   //0x1813C0F4 ��Ե������ֵ��Ϣ
   g_BMSMonitor_Insul.Insulation_Resis2 = (*(uint16*)(0x0D0A));
   /*
   //1814C0F4 �����������ڴ�ŵ�����ֵ
   g_BMSMonitor_New_LT.Voll_Sys_Low1_LT = (*(uint16*)0x0D28);
   
   g_BMSMonitor_New_LT.Volt_Cell_Low1_LT = (*(uint16*)0x0D30); */
   
   
   //0x1915F4C0 ���������Ϣ�궨 
   //00
   g_MonitorBMS_SOC.SOC_Init = (*(uint16*)(0x0D10));
   g_SOCInfo.SOC_Init = g_MonitorBMS_SOC.SOC_Init;
   g_MonitorBMS_SOC.Ref_Volt = (*(uint16*)(0x0D12));
   g_MonitorBMS_SOC.Ref_Current = (*(uint16*)(0x0D14));
   
   //01
   g_MonitorBMS_SOC.Volt_Resis = (*(uint16*)(0x0D18));
   g_MonitorBMS_SOC.Pack_Total_Capacity = (*(uint32*)(0x0D1A)); 
   
   //02
   g_MonitorBMS_SOC.BMS_Running_Time = *(uint16*)(0x0D20);   //��λ���±굥λΪСʱ�洢Ϊ����
   
 }  
 
   
/*=======================================================================
 *������:      EEprom_read
 *����:        ��EEprom�ж�ȡֵ
 *����:        addr���洢����ʼλ��
               addrbase����ַ��ƫ����
 *���أ�       ��
 *˵����       ��ȡ����,��ַΪ��0x0D10~0x0E70
========================================================================*/ 
static
void EEprom_read(uint16 addr, uint8 addrbase) 
{     
   //�����洢���ֽ�
   EEprom_Data.Charge_Times =  *(uint16*)(addr + (addrbase%EEprom_Length)*Elem_Num); //������
   g_SysTime.BMS_TotalRun_MiniteTime =  *(uint32*)(addr + (addrbase%EEprom_Length)*Elem_Num +2);
   g_SysTime.BMS_PowerOff_Time = *(uint32*)(addr + (addrbase%EEprom_Length)*Elem_Num + 6); //SOC����ʱ����h��
   g_EnergyInfo.Energy_Total_Charge = (*(uint32*)(addr + (addrbase%EEprom_Length)*Elem_Num + 10))/100.0;
   g_EnergyInfo.Energy_Total_DisCharge =  (*(uint32*)(addr + (addrbase%EEprom_Length)*Elem_Num + 14))/100.0;
   g_SOCInfo.SOC_LowestVoltGet =  (*(uint16*)(addr + (addrbase%EEprom_Length)*Elem_Num + 18))/10000.0;
   g_SOCInfo.SOC_HighestVoltGet = (*(uint16*)(addr + (addrbase%EEprom_Length)*Elem_Num + 20))/10000.0;       
   g_SOCInfo.SOC_ValueRead =   (*(uint16*)(addr + (addrbase%EEprom_Length)*Elem_Num + 22))/10000.0;
   g_SOCInfo.SOC_ValueVoltGet =   (*(uint16*)(addr + (addrbase%EEprom_Length)*Elem_Num + 24))/10000.0;
   
   //�Ǿ����洢���ֽ�      
   //0x1810C0F4 ��ص�ѹ������ֵ
   //�����ѹ 00
   g_BMSMonitor_Volt.Volt_Cell_High1 = (*(uint16*)(0x0CA0));
   g_BMSMonitor_Volt.Volt_Cell_High2 = (*(uint16*)(0x0CA2));
   
   //����Ƿѹ 01
   g_BMSMonitor_Volt.Volt_Cell_Low1 = (*(uint16*)(0x0CA8));
   g_BMSMonitor_Volt.Volt_Cell_Low2 = (*(uint16*)(0x0CAA));

   //ϵͳ��ѹ 02
   g_BMSMonitor_Volt.Volt_Sys_High1 = (*(uint16*)(0x0CB0));
   g_BMSMonitor_Volt.Volt_Sys_High2 = (*(uint16*)(0x0CB2));
   
   //ϵͳǷѹ 03 
   g_BMSMonitor_Volt.Volt_Sys_Low1 = (*(uint16*)(0x0CB8));
   g_BMSMonitor_Volt.Volt_Sys_Low2 = (*(uint16*)(0x0CBA));
   
   //����ѹ�� 04
   g_BMSMonitor_Volt.Volt_Cell_Diff1 = (*(uint16*)(0x0CC0));
  
   //0x1811C0F4 ����¶ȱ�����ֵ
   //����о�¶� 00
   //����
   g_BMSMonitor_Temp.Temp_Charge_High1 = (*(uint8*)(0x0CD0));
   g_BMSMonitor_Temp.Temp_Charge_High2 = (*(uint8*)(0x0CD1));
   //����
   g_BMSMonitor_Temp.Temp_Charge_Low1 = (*(uint8*)(0x0CD3));
   g_BMSMonitor_Temp.Temp_Charge_Low2 = (*(uint8*)(0x0CD4));
   
   //�ŵ��о�¶� 01
   //����
   g_BMSMonitor_Temp.Temp_DisCharge_High1 = (*(uint8*)(0x0CD8));
   g_BMSMonitor_Temp.Temp_DisCharge_High2 = (*(uint8*)(0x0CD9));
   //����
   g_BMSMonitor_Temp.Temp_DisCharge_Low1 = (*(uint8*)(0x0CDB));
   g_BMSMonitor_Temp.Temp_DisCharge_Low2 = (*(uint8*)(0x0CDC));
   
   //����²� 02
   //���
   g_BMSMonitor_Temp.Temp_Charge_Diff1 = (*(uint8*)(0x0CE0));
   
   //�ŵ�
   g_BMSMonitor_Temp.Temp_DisCharge_Diff1 = (*(uint8*)(0x0CE3));
  
   //0x1812C0F4 ��ص�����ֵ��Ϣ
   //ϵͳ�ŵ���� 00
   g_BMSMonitor_Curr.Current_DisCharge_High1 = (*(uint16*)(0x0CF0));
   g_BMSMonitor_Curr.Current_DisCharge_High2 = (*(uint16*)(0x0CF2));
   
   //ϵͳ������ 01
   g_BMSMonitor_Curr.Current_Charge_High1 = (*(uint16*)(0x0CF8));
   g_BMSMonitor_Curr.Current_Charge_High2 = (*(uint16*)(0x0CFA));
   
  
   //0x1813C0F4 ��Ե������ֵ��Ϣ
   g_BMSMonitor_Insul.Insulation_Resis2 = (*(uint16*)(0x0D0A));
   /*
   //1814C0F4 �����������ڴ�ŵ�����ֵ
   g_BMSMonitor_New_LT.Voll_Sys_Low1_LT = (*(uint16*)0x0D28);
   g_BMSMonitor_New_LT.Voll_Sys_Low2_LT = (*(uint16*)0x0D2A);
   g_BMSMonitor_New_LT.Volt_Cell_Low1_LT = (*(uint16*)0x0D30);
   g_BMSMonitor_New_LT.Volt_Cell_Low2_LT = (*(uint16*)0x0D32); */
   
   //0x1915F4C0 ���������Ϣ�궨 
   //00
   g_MonitorBMS_SOC.SOC_Init = (*(uint16*)(0x0D10));
   g_SOCInfo.SOC_Init = g_MonitorBMS_SOC.SOC_Init;
   g_MonitorBMS_SOC.Ref_Volt = (*(uint16*)(0x0D12));
   g_MonitorBMS_SOC.Ref_Current = (*(uint16*)(0x0D14));
   
   //01
   g_MonitorBMS_SOC.Volt_Resis = (*(uint16*)(0x0D18));
   g_MonitorBMS_SOC.Pack_Total_Capacity = (*(uint32*)(0x0D1A)); 
   
   //02
   g_MonitorBMS_SOC.BMS_Running_Time = *(uint16*)(0x0D20);   //��λ���±굥λΪСʱ�洢Ϊ����
    
}
/*=======================================================================
 *������:      EEEWrite_Delayus(uint16 us)
 *����:        ��EEpromдֵ����ʱ
 *����:        us:��ʱʱ��.��λ:us
 *˵����      
========================================================================*/
static
void EEEWrite_Delayus(uint16 us) 
{
	  uint16 delayval;
	  delayval = us * 9;
	  while(delayval--);
}

/*=======================================================================
 *������:      EEprom_Write
 *����:        ��EEprom��дֵ
 *����:        addr���洢����ʼλ��
               addrbase����ַ��ƫ����
 *���أ�       ��
 *˵����       �������ݱ�ǵĻ���ַ�е�����ֵ��������ֵд������洢��
========================================================================*/ 
static
void EEprom_Write(uint16 addr)
{
   uint8*ptr = (uint8*)EEprom_Ptr; 
   EEprom_Data.pEErom_base++;
   if(EEprom_Data.pEErom_base >= EEprom_Length) 
   {
      EEprom_Data.pEErom_base = 0;//���»�λ
   }        
   *(ptr+EEprom_Data.pEErom_base) =  EEprom_Data.pEErom_base;         
          
   *(uint16*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num) = (uint16)EEprom_Data.Charge_Times;//������
   EEEWrite_Delayus(5);
   *(uint32*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 2) = (uint32)g_SysTime.BMS_TotalRun_MiniteTime;//BMS����ʱ��
   EEEWrite_Delayus(5);
   *(uint32*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 6) = (uint32)g_SysTime.BMS_PowerOff_Time; //SOC����ʱ��
   EEEWrite_Delayus(5);
   *(uint32*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 10) = (uint32)(g_EnergyInfo.Energy_Total_Charge*100);  //�ۼƳ����
   EEEWrite_Delayus(5);
   *(uint32*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 14) = (uint32)(g_EnergyInfo.Energy_Total_DisCharge*100);    //�ۼƷŵ���
   EEEWrite_Delayus(5);
   *(uint16*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 18) = (uint16)(g_SOCInfo.SOC_LowestVoltGet*10000);
   EEEWrite_Delayus(5);
   *(uint16*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 20) = (uint16)(g_SOCInfo.SOC_HighestVoltGet*10000);
   EEEWrite_Delayus(5);         
   *(uint16*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 22) = (uint16)(g_SOCInfo.SOC_ValueRead*10000);    //SOC��ʾֵ
   EEEWrite_Delayus(5);
   *(uint16*)(addr + (EEprom_Data.pEErom_base%EEprom_Length)*Elem_Num + 24) = (uint16)(g_SOCInfo.SOC_ValueVoltGet*10000);    //SOC���ֵ  
   EEEWrite_Delayus(5);
   ptr[(EEprom_Data.pEErom_base+EEprom_Length-1)%16] = 0xFF;//���ϴε�ַ����������Ϊ0xFF,���ܷ��ڴ洢֮ǰ���Է��洢�������жϡ�            

   g_Roll_Tick.Roll_EEEWrite++;
}


/*=======================================================================
 *������:      Task_EEpromWrite
 *����:        ��EEprom��дֵ
 *����:        addr���洢����ʼλ��
               addrbase����ַ��ƫ����
 *���أ�       ��
 *˵����       �������ݱ�ǵĻ���ַ�е�����ֵ��������ֵд������洢��
========================================================================*/   
void Task_EEpromWrite(void)
{
   EEprom_Write(EEprom_Baseadrr);
}
   