/*=======================================================================
 *Subsystem:   ���
 *File:        Task_UpMonitor.C
 *Author:      Wen Yuhao
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "includes.h"

BMSMonitor_Volt_T g_BMSMonitor_Volt;
BMSMonitor_Temp_T g_BMSMonitor_Temp;
BMSMonitor_Curr_T g_BMSMonitor_Curr;
BMSMonitor_Insu_T g_BMSMonitor_Insul; 
BMSMonitor_SOH_T  g_BMSMonitor_SOH;
BMSMonitor_New_LT_T g_BMSMonitor_New_LT;

/*=======================================================================
 *������:      CAN_UpMonitorInit(void) 
 *����:        ��λ��ʹ�õ�CANͨ����ʼ��
 *����:        ��       
 *���أ�       ��
 
 *˵����       
========================================================================*/ 
uint8 CAN_UpMonitorInit(void)
{
  uint8 state;
  state = CAN2_Init(500);
  return(state);
}

/*=======================================================================
 *������:      CAN_ToUpMonitor(pCANFRAME canptr) 
 *����:        BMS information to UpMonitor
 *����:        ��       
 *���أ�       ��
 *˵����       BMS������Ϣ����λ����CANͨ��ѡ��
========================================================================*/ 
uint8 CAN_ToUpMonitor(pCANFRAME canptr)
{
  if(!CAN2_SendMsg(canptr))
  {
     return 0;//���� 
  }  
  return 1;  //CAN���� */
}


/*=======================================================================
 *������:      Init_UpMonitor(void) 
 *����:        ��ʼ����λ��
 *����:        ��       
 *���أ�       ��
 *˵����       
 ========================================================================*/ 
void Init_UpMonitor(void)
{
  memset(&g_BMSMonitor_Volt, 0, sizeof(BMSMonitor_Volt_T));   //���µ�ѹ��ֵ
  memset(&g_BMSMonitor_Temp, 0, sizeof(BMSMonitor_Temp_T));   //�¶���ֵ
  memset(&g_BMSMonitor_Curr, 0, sizeof(BMSMonitor_Curr_T));   //������ֵ
  memset(&g_BMSMonitor_Insul, 0, sizeof(BMSMonitor_Insu_T)); //��Ե��ֵ
  memset(&g_BMSMonitor_New_LT, 0, sizeof(BMSMonitor_New_LT_T)); //���µ�ѹ��ֵ

  memset(&g_MonitorBMS_Start, 0, sizeof(MonitorBMS_Start_T)); //�±��������

}

/*=======================================================================
 *������:      UpMonitor_DelayTimeus(uint16 us)
 *����:        
 *����:              
 *���أ�       
 *˵����       
========================================================================*/
static
void UpMonitor_DelayTimeus(uint16 us) 
{
	  uint16 delayval;
	  delayval = us * 9;
	  while(delayval--); 
}


/*=======================================================================
 *������:      Task_BmsToUpMonitor(void) 
 *����:        BMS information to UpMonitor
 *����:        ��       
 *���أ�       ��
 *˵����       BMS������Ϣ����λ����
========================================================================*/  
static
void CAN_ToUpMonitorMsg(void)
{  
  uint8 i;
  
  uint8 Positive;
  CANFRAME BMS_to_Upmonitor;
  
  BMS_to_Upmonitor.m_IDE = 1;
	BMS_to_Upmonitor.m_RTR = 0;
	BMS_to_Upmonitor.m_dataLen = 8;
	BMS_to_Upmonitor.m_priority = 6; 
  
  if(g_MonitorBMS_Start.Threshold_StarUpload == 1)
  {
    //�������Ϣ
    //0x1800C0F4
    for(i=0;i<3;i++)
    { 
      BMS_to_Upmonitor.m_ID = 0x1800C0F4;     	
      switch(i)
      {
        case 0:
        	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
        	BMS_to_Upmonitor.m_data[1] = CELL_TYPE;                                //������� 
        	BMS_to_Upmonitor.m_data[2] = (uint8)(CELL_RESIS_INTERNAL*10);          //������裬��λ��0.1m��/Bit  
        	BMS_to_Upmonitor.m_data[3] = ((uint16)(CELL_RESIS_INTERNAL*10)) >> 8; 
        	BMS_to_Upmonitor.m_data[4] = (uint8)(SYS_CAPACITY*100);               //�����������λ��0.01AH/Bit
        	BMS_to_Upmonitor.m_data[5] = ((uint16)(SYS_CAPACITY*100)) >> 8;
        	BMS_to_Upmonitor.m_data[6] = (uint8)CELL_LIFE_CYCLE;                   //ѭ��������1��/Bit
        	BMS_to_Upmonitor.m_data[7] = ((uint16)(CELL_LIFE_CYCLE)) >> 8;       
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));  
          UpMonitor_DelayTimeus(20);
        break;
        
        case 1:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x01
          BMS_to_Upmonitor.m_data[1] = (uint8)(CELL_VOLT_NOMINAL*10000);     //�����Ƶ�ѹ����λ��0.0001V/Bit  
          BMS_to_Upmonitor.m_data[2] = ((uint16)(CELL_VOLT_NOMINAL*10000)) >> 8;          
          BMS_to_Upmonitor.m_data[3] = (uint8)(CELL_VOLT_MAX*10000);    //��ߵ����ѹ����λ��0.0001V/Bi
          BMS_to_Upmonitor.m_data[4] = ((uint16)(CELL_VOLT_MAX*10000)) >> 8;             
          BMS_to_Upmonitor.m_data[5] = (uint8)(CELL_VOLT_MIN*10000);        //��͵����ѹ����λ��0.0001V/Bit
          BMS_to_Upmonitor.m_data[6] = ((uint16)(CELL_VOLT_MIN*10000)) >> 8;                  
          BMS_to_Upmonitor.m_data[7] = CELL_TEMP_MAX_DISCHARGE;   //�����߱����¶�
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));  
          UpMonitor_DelayTimeus(20);
        break;
        
        case 2:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x02
          BMS_to_Upmonitor.m_data[1] = (uint8)(F1_DISCHG_VOLTCD);          //�����ѹ�� 
          BMS_to_Upmonitor.m_data[2] = (uint16)(F1_DISCHG_VOLTCD) >> 8;            
          BMS_to_Upmonitor.m_data[3] = (uint8)F2_DISCHG_TEMPL;             //������ʹ���¶�
          BMS_to_Upmonitor.m_data[4] = (uint8)SYS_SERIES_YiDongLi;         //���ϵͳ�ܴ���   
          BMS_to_Upmonitor.m_data[5] = (uint8)SYS_NUMBER_MODULE;           //���ϵͳģ����
          BMS_to_Upmonitor.m_data[6] = (uint8)SYS_NUMBER_BOX;              //���ϵͳ����
          BMS_to_Upmonitor.m_data[7] = (uint8)SYS_PARALLEL;                //���ϵͳ���ܲ���
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));  
          UpMonitor_DelayTimeus(20);
        break;
      }
    }

    //��ع��ϵȼ�
    //0x1810C0F4      ��ص�ѹ������ֵ��Ϣ
    for(i=0;i<6;i++)
    {
      BMS_to_Upmonitor.m_ID = 0x1810C0F4;
      switch(i)
      {
        //�����ѹ
        case 0 :
        	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Volt.Volt_Cell_High1);         //�����ѹ������ֵ1�� 
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Volt.Volt_Cell_High1)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Volt.Volt_Cell_High2);         //�����ѹ������ֵ2��
        	BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Volt.Volt_Cell_High2)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(g_BMSMonitor_Volt.Single_Over3_Volt);         //�����ѹ������ֵ3��
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(g_BMSMonitor_Volt.Single_Over3_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;       
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));  
          UpMonitor_DelayTimeus(20); 
        break;
        //����Ƿѹ   
        case 1:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x01
         	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Volt.Volt_Cell_Low1);         //�����ѹ������ֵ1�� 
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Volt.Volt_Cell_Low1)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Volt.Volt_Cell_Low2);         //�����ѹ������ֵ2��
        	BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Volt.Volt_Cell_Low2)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(g_BMSMonitor_Volt.Single_Under3_Volt);         //�����ѹ������ֵ3��
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(g_BMSMonitor_Volt.Single_Under3_Volt)) >> 8;                  
          BMS_to_Upmonitor.m_data[7] = 0xFF;                               
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));   
          UpMonitor_DelayTimeus(20);
        break;
        //ϵͳ��ѹ  
        case 2:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x02
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Volt.Volt_Sys_High1);         //ϵͳ��ѹ������ֵ1�� 
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Volt.Volt_Sys_High1)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Volt.Volt_Sys_High2);         //ϵͳ��ѹ������ֵ2��
        	BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Volt.Volt_Sys_High2)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Volt.Pack_Over3_Volt);         //ϵͳ��ѹ������ֵ3��
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Volt.Pack_Over3_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));  
          UpMonitor_DelayTimeus(20);
        break;
        //ϵͳǷѹ
        case 3:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x03
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Volt.Volt_Sys_Low1);         //ϵͳ��ѹ������ֵ1�� 
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Volt.Volt_Sys_Low1)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Volt.Volt_Sys_Low2);         //ϵͳ��ѹ������ֵ2��
        	BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Volt.Volt_Sys_Low2)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Volt.Pack_Under3_Volt);         //ϵͳ��ѹ������ֵ3��
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Volt.Pack_Under3_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor)); 
          UpMonitor_DelayTimeus(20);
        break;
        //����ѹ��
        case 4:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x04
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Volt.Volt_Cell_Diff1);         //�����ѹѹ����ֵ1�� 
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Volt.Volt_Cell_Diff1)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Volt.Single_Diff2_Volt);         //�����ѹѹ����ֵ2��
        	BMS_to_Upmonitor.m_data[4] = 0xFF;//((uint16)(Monitor_BMS_Volt.Single_Diff2_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Volt.Single_Diff3_Volt);         //�����ѹѹ����ֵ3��
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Volt.Single_Diff3_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));  
          UpMonitor_DelayTimeus(20);
        break; 
        //ϵͳѹ��
        case 5:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x05
        	BMS_to_Upmonitor.m_data[1] = 0xFF;//(uint8)(Monitor_BMS_Volt.System_Diff1_Volt);         //ϵͳ��ѹѹ����ֵ1�� 
        	BMS_to_Upmonitor.m_data[2] = 0xFF;//((uint16)(Monitor_BMS_Volt.System_Diff1_Volt)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Volt.System_Diff2_Volt);         //ϵͳ��ѹѹ����ֵ2��
        	BMS_to_Upmonitor.m_data[4] = 0xFF;//((uint16)(Monitor_BMS_Volt.System_Diff2_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Volt.System_Diff3_Volt);         //ϵͳ��ѹѹ����ֵ3��
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Volt.System_Diff3_Volt)) >> 8;       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));   
          UpMonitor_DelayTimeus(20);
        break; 
      }
    }

    //0x1811C0F4      ����¶ȱ�����ֵ��Ϣ
    for(i=0;i<4;i++)
    {
      BMS_to_Upmonitor.m_ID = 0x1811C0F4;
      switch(i)
      {
        //����о�¶� 
        case 0:
        	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Temp.Temp_Charge_High1);         //����о�¶ȹ��� 
        	BMS_to_Upmonitor.m_data[2] = (uint8)(g_BMSMonitor_Temp.Temp_Charge_High2);              
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Temp.Charge_Over3_Temp);         
        	BMS_to_Upmonitor.m_data[4] = (uint8)(g_BMSMonitor_Temp.Temp_Charge_Low1);        //����о�¶ȹ��� 
        	BMS_to_Upmonitor.m_data[5] = (uint8)(g_BMSMonitor_Temp.Temp_Charge_Low2);         
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//(uint8)(Monitor_BMS_Temp.Charge_Under3_Temp);        
        	BMS_to_Upmonitor.m_data[7] = 0xFF;       
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));   
          UpMonitor_DelayTimeus(20); 
        break;
        //�ŵ��о�¶�   
        case 1:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x01
         	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Temp.Temp_DisCharge_High1);         //�ŵ��о�¶ȹ��� 
        	BMS_to_Upmonitor.m_data[2] = (uint8)(g_BMSMonitor_Temp.Temp_DisCharge_High2);             
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Temp.DisCharge_Over3_Temp);         
        	BMS_to_Upmonitor.m_data[4] = (uint8)(g_BMSMonitor_Temp.Temp_DisCharge_Low1);        //�ŵ��о�¶ȹ��� 
        	BMS_to_Upmonitor.m_data[5] = (uint8)(g_BMSMonitor_Temp.Temp_DisCharge_Low2);         
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//(uint8)(Monitor_BMS_Temp.DisCharge_Under3_Temp);                  
          BMS_to_Upmonitor.m_data[7] = 0xFF;                               
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));    
          UpMonitor_DelayTimeus(20);
        break;
        //����²�   
        case 2:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x02
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Temp.Temp_Charge_Diff1);         //������²����
        	BMS_to_Upmonitor.m_data[2] = 0xFF;//(uint8)(Monitor_BMS_Temp.Charge_Diff2_Temp);             
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Temp.Charge_Diff3_Temp);         
        	BMS_to_Upmonitor.m_data[4] = (uint8)(g_BMSMonitor_Temp.Temp_DisCharge_Diff1);      //�ŵ����²���� 
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Temp.DisCharge_Diff2_Temp);       
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//(uint8)(Monitor_BMS_Temp.DisCharge_Diff3_Temp);       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));     
          UpMonitor_DelayTimeus(20);
        break;
        //���ǹ&BMS������ֵ
        case 3:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x03
        	BMS_to_Upmonitor.m_data[1] = 0xFF;//(uint8)(Monitor_BMS_Temp.ChargeGun_Over1_Temp);         //���ǹ����
        	BMS_to_Upmonitor.m_data[2] = 0xFF;//(uint8)(Monitor_BMS_Temp.ChargeGun_Over2_Temp);             
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Temp.ChargeGun_Over3_Temp);         
        	BMS_to_Upmonitor.m_data[4] = 0xFF;//(uint8)(Monitor_BMS_Temp.Chip_Over1_Temp);              //BMS����
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Temp.Chip_Over2_Temp);         
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//(uint8)(Monitor_BMS_Temp.Chip_Over3_Temp);       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));      
          UpMonitor_DelayTimeus(20);
        break;
      }
    }

    //0x1812C0F4      ��ص�����ֵ��Ϣ
    for(i=0;i<3;i++)
    {
      BMS_to_Upmonitor.m_ID = 0x1812C0F4;
      switch(i)
      {
        //ϵͳ�ŵ���� 
        case 0:
        	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
        	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Curr.Current_DisCharge_High1);          
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Curr.Current_DisCharge_High1)) >> 8;              
        	BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Curr.Current_DisCharge_High2);         
        	BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Curr.Current_DisCharge_High2)) >> 8;       
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Current.DisCharge_Over3_Current);         
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Current.DisCharge_Over3_Current)) >> 8;       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;       
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));     
          UpMonitor_DelayTimeus(20); 
        break;
        //ϵͳ������ 
        case 1:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x01
         	BMS_to_Upmonitor.m_data[1] = (uint8)(g_BMSMonitor_Curr.Current_Charge_High1);         
        	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_BMSMonitor_Curr.Current_Charge_High1)) >> 8;             
        	BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Curr.Current_Charge_High2);         
        	BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Curr.Current_Charge_High2)) >> 8;         
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Current.Charge_Over3_Current);         
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Current.Charge_Over3_Current)) >> 8;                  
          BMS_to_Upmonitor.m_data[7] = 0xFF;                               
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));      
          UpMonitor_DelayTimeus(20);
        break;
        //SOC��ֵ
        case 2:
          BMS_to_Upmonitor.m_data[0] = i;     //��� 0x02
        	BMS_to_Upmonitor.m_data[1] = 0xFF;//(uint8)(Monitor_BMS_Current.SOC_Low1);         //��SOC
        	BMS_to_Upmonitor.m_data[2] = 0xFF;//(uint8)(Monitor_BMS_Current.SOC_Low2);             
        	BMS_to_Upmonitor.m_data[3] = 0xFF;//(uint8)(Monitor_BMS_Current.SOC_Low3);         
        	BMS_to_Upmonitor.m_data[4] = 0xFF;//(uint8)(Monitor_BMS_Current.SOC_High1);        //��SOC
        	BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Current.SOC_High2);         
        	BMS_to_Upmonitor.m_data[6] = 0xFF;//(uint8)(Monitor_BMS_Current.SOC_High3);       
        	BMS_to_Upmonitor.m_data[7] = 0xFF;
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));        
          UpMonitor_DelayTimeus(20);
        break;
      }
    }

    //0x1813C0F4     ��Ե������ֵ��Ϣ
    BMS_to_Upmonitor.m_ID = 0x1813C0F4;     	
    BMS_to_Upmonitor.m_data[0] = 0x00;     //���  0x00
    BMS_to_Upmonitor.m_data[1] = 0xFF;//(uint8)(Monitor_BMS_Insulation.Insulation_Resis1);         
    BMS_to_Upmonitor.m_data[2] = 0xFF;//((uint16)(Monitor_BMS_Insulation.Insulation_Resis1)) >> 8;             
    BMS_to_Upmonitor.m_data[3] = (uint8)(g_BMSMonitor_Insul.Insulation_Resis2);         
    BMS_to_Upmonitor.m_data[4] = ((uint16)(g_BMSMonitor_Insul.Insulation_Resis2)) >> 8;         
    BMS_to_Upmonitor.m_data[5] = 0xFF;//(uint8)(Monitor_BMS_Insulation.Insulation_Resis3);         
    BMS_to_Upmonitor.m_data[6] = 0xFF;//((uint16)(Monitor_BMS_Insulation.Insulation_Resis3)) >> 8;                  
    BMS_to_Upmonitor.m_data[7] = 0xFF;                               
    while(CAN_ToUpMonitor(&BMS_to_Upmonitor));      
    UpMonitor_DelayTimeus(20);

    g_MonitorBMS_Start.Threshold_StarUpload = 0x00;
  }
  
  
  //ʵʱ����������Ϣ 
  //0x1820C0F4    ���ϵͳ�Լ켰ʱ����Ϣ
  for(i=0;i<3;i++)
  {
    BMS_to_Upmonitor.m_ID = 0x1820C0F4;
    switch(i)
    {
      case 0 :     //ϵͳʵʱʱ��
      	BMS_to_Upmonitor.m_data[0] = 0;     //��� 0x00
      	BMS_to_Upmonitor.m_data[1] = Read_IIC_Time.IIC_Read_Year;     //ϵͳʵʱʱ��:�� 
      	BMS_to_Upmonitor.m_data[2] = Read_IIC_Time.IIC_Read_Month;    //ϵͳʵʱʱ��:��         
      	BMS_to_Upmonitor.m_data[3] = Read_IIC_Time.IIC_Read_Day;      //ϵͳʵʱʱ��:��
      	BMS_to_Upmonitor.m_data[4] = Read_IIC_Time.IIC_Read_Hour;     //ϵͳʵʱʱ��:ʱ  
      	BMS_to_Upmonitor.m_data[5] = Read_IIC_Time.IIC_Read_Minute;   //ϵͳʵʱʱ��:��
      	BMS_to_Upmonitor.m_data[6] = 0xFF;
      	BMS_to_Upmonitor.m_data[7] = 0xFF;       
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
      
      case 1:    //ϵͳ����״̬��ʱ��
        BMS_to_Upmonitor.m_data[0] = 1;     //��� 0x01
        BMS_to_Upmonitor.m_data[1] = g_WorkStateJudge.WorkState;    //ϵͳ����״̬��00 �ŵ磬01 ��䣬02����  
        BMS_to_Upmonitor.m_data[2] = (uint8)(g_SysTime.BMS_TotalRun_MiniteTime/60);    //BMSϵͳ����ʱ�䣨Сʱ��       
        BMS_to_Upmonitor.m_data[3] = (uint16)(g_SysTime.BMS_TotalRun_MiniteTime/60) >> 8;    
        BMS_to_Upmonitor.m_data[4] = 0xFF;    //BMSϵͳ��������ʱ��
        BMS_to_Upmonitor.m_data[5] = 0xFF;        
        BMS_to_Upmonitor.m_data[6] = 0xFF;    //BMSϵͳ�ϴ�����ʱ��              
        BMS_to_Upmonitor.m_data[7] = 0xFF;   
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
      
      case 2:      //����Լ�״̬���Լ�ʧ��ԭ��
      
        BMS_to_Upmonitor.m_data[0] = 2;     //��� 0x02
        BMS_to_Upmonitor.m_data[1] = 0x01;   //����Լ�״̬  00 �Լ��У�01 �ɹ���02 ʧ��
        BMS_to_Upmonitor.m_data[2] = 0x00;       
        BMS_to_Upmonitor.m_data[3] = 0x00;    
        BMS_to_Upmonitor.m_data[4] = 0x00;    
        BMS_to_Upmonitor.m_data[5] = 0x00;        
        BMS_to_Upmonitor.m_data[6] = 0x00;                 
        BMS_to_Upmonitor.m_data[7] = 0x00; 
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
    }
  }
  
  //0x1830C0F4    ���ϵͳ��ѹ��Ϣ
  for(i=0;i<2;i++)
  {
    BMS_to_Upmonitor.m_ID = 0x1830C0F4; 
    switch(i)
    {
      case 0:    //�����ѹ��ѹ��
      	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
      	BMS_to_Upmonitor.m_data[1] = (uint8)(g_VoltInfo.SysVolt_Total*0.001);     //�����ѹ�������ۼӣ�   0.1V�ֱ���
      	BMS_to_Upmonitor.m_data[2] = ((uint16)(g_VoltInfo.SysVolt_Total*0.001))>>8;             
      	BMS_to_Upmonitor.m_data[3] = 0xFF; //(uint8)(SOC_DATA.BMSU_Volt*10);     //�����ѹ����ѹģ����ѹ��  0.1V�ֱ���
      	BMS_to_Upmonitor.m_data[4] = 0xFF; //((uint16)(SOC_DATA.BMSU_Volt*10)) >> 8;        
      	BMS_to_Upmonitor.m_data[5] = 0xFF; //(uint8)(abs(g_TempInfo.TotalVolt_V-SOC_DATA.BMSU_Volt)*10);  //ѹ�abs(�ۼӵ�ѹ-���Ե�ѹ)��    0.1V�ֱ���
      	BMS_to_Upmonitor.m_data[6] = 0xFF; //((uint16)(abs(g_TempInfo.TotalVolt_V-SOC_DATA.BMSU_Volt)*10)) >> 8;
      	BMS_to_Upmonitor.m_data[7] = 0xFF;       
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
      
      case 1:    //�������/��͵�ѹ��ѹ��
        BMS_to_Upmonitor.m_data[0] = i;     //��� 0x01
        BMS_to_Upmonitor.m_data[1] = (uint8)(g_VoltInfo.CellVolt_Max);          //������ߵ�ѹ���ֱ��ʣ�0.0001V��
        BMS_to_Upmonitor.m_data[2] = ((uint16)(g_VoltInfo.CellVolt_Max)) >> 8;   
        BMS_to_Upmonitor.m_data[3] = (uint8)(g_VoltInfo.CellVolt_Min);          //������͵�ѹ���ֱ��ʣ�0.0001V)   
        BMS_to_Upmonitor.m_data[4] = ((uint16)(g_VoltInfo.CellVolt_Min)) >> 8;
        BMS_to_Upmonitor.m_data[5] = (uint8)(g_VoltInfo.CellVolt_Diff);                //����ѹ��ֱ��ʣ�0.0001V) 
        BMS_to_Upmonitor.m_data[6] = ((uint16)(g_VoltInfo.CellVolt_Diff)) >> 8;             
        BMS_to_Upmonitor.m_data[7] = 0xFF;   
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
    }
  }
   
  //0x1840C0F4    ���ϵͳ�¶���Ϣ
  for(i=0;i<2;i++)
  {
    BMS_to_Upmonitor.m_ID = 0x1840C0F4;
    switch(i)
    {
      case 0:     //�������/����¶ȼ��²�
      	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
      	BMS_to_Upmonitor.m_data[1] = (uint8)(g_TempInfo.CellTemp_Max);     //����������¶�   1��ֱ���
      	BMS_to_Upmonitor.m_data[2] = (uint8)(g_TempInfo.CellTemp_Min);     //����������¶�   1��ֱ���             
      	BMS_to_Upmonitor.m_data[3] = (uint8)(g_TempInfo.CellTemp_Diff + 40);    //�������²�  1��ֱ���
      	BMS_to_Upmonitor.m_data[4] = 0xFF;        
      	BMS_to_Upmonitor.m_data[5] = 0xFF;  
      	BMS_to_Upmonitor.m_data[6] = 0xFF;
      	BMS_to_Upmonitor.m_data[7] = 0xFF;       
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
    }
  }
  
  //0x1850C0F4    ���ϵͳ������Ϣ
  BMS_to_Upmonitor.m_ID = 0x1850C0F4;     	
	BMS_to_Upmonitor.m_data[0] = (uint8)((g_DataColletInfo.DataCollet_Current_Filter + 750)*10);        //��������ֵ   0.1A�ֱ���  ƫ������-750
	BMS_to_Upmonitor.m_data[1] = ((uint16)((g_DataColletInfo.DataCollet_Current_Filter + 750)*10))>>8;
	BMS_to_Upmonitor.m_data[2] = (uint8)((CurrLimit.Curr_Charge_Cons + 750)*10);     //����������ֵ   0.1A�ֱ���  ƫ������-750  ��δд��          
	BMS_to_Upmonitor.m_data[3] = ((uint16)((CurrLimit.Curr_Charge_Cons + 750)*10))>>8;
	BMS_to_Upmonitor.m_data[4] = 0xFF;  //(uint8)((Current_Limit.ConstantDischargeCurrent + 750)*10);     //�ŵ��������ֵ   0.1A�ֱ���  ƫ������-750   ��δд��
	BMS_to_Upmonitor.m_data[5] = 0xFF;  //((uint16)((Current_Limit.ConstantDischargeCurrent + 750)*10))>>8;  
	BMS_to_Upmonitor.m_data[6] = 0xFF;  //(uint8)((Current_Limit.FeedbackCurrent + 750)*10);          //�������������ֵ   0.1A�ֱ���  ƫ������-750   ��δд��
	BMS_to_Upmonitor.m_data[7] = 0xFF;  //((uint16)((Current_Limit.FeedbackCurrent + 750)*10))>>8;       
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
  UpMonitor_DelayTimeus(20);

  //0x1860C0F4    ���ϵͳ��Ե��Ϣ
  BMS_to_Upmonitor.m_ID = 0x1860C0F4;     	
  BMS_to_Upmonitor.m_data[0] = (uint8)(g_IsoDetect.insulation_Vposit*10);    //���Եص�ѹ   0.1V�ֱ���
  BMS_to_Upmonitor.m_data[1] = ((uint16)(g_IsoDetect.insulation_Vposit*10))>>8;           
  BMS_to_Upmonitor.m_data[2] = (uint8)(g_IsoDetect.insulation_Vnegt*10);           //���Եص�ѹ   0.1V�ֱ���
  BMS_to_Upmonitor.m_data[3] = ((uint16)(g_IsoDetect.insulation_Vnegt*10))>>8;              
  BMS_to_Upmonitor.m_data[4] = (uint8)(g_IsoDetect.insulation_resist_P*10);           //���Եص���ֵ   0.1V�ֱ���
  BMS_to_Upmonitor.m_data[5] = ((uint16)(g_IsoDetect.insulation_resist_P*10))>>8;               
  BMS_to_Upmonitor.m_data[6] = (uint8)(g_IsoDetect.insulation_resist_N*10);            //���Եص���ֵ  0.1V�ֱ��ʣ�δд��
  BMS_to_Upmonitor.m_data[7] = ((uint16)(g_IsoDetect.insulation_resist_N*10))>>8;
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
  UpMonitor_DelayTimeus(20);
     
  //0x1870C0F4    ���SOC��SOH��Ϣ
  for(i=0;i<2;i++)
  {
    BMS_to_Upmonitor.m_ID = 0x1870C0F4;
    switch(i)
    {
      case 0:     //SOC,SOC,���γ�/�ŵ���
      	BMS_to_Upmonitor.m_data[0] = i;     //��� 0x00
      	BMS_to_Upmonitor.m_data[1] = (uint8)((g_SOCInfo.SOC_ValueRead+0.005)*100);      //SOC_Read      1%�ֱ���
      	BMS_to_Upmonitor.m_data[2] = (uint8)(g_SOCInfo.SOC_ValueVoltGet*100);      //SOC_Volt      1��ֱ���             
      	BMS_to_Upmonitor.m_data[3] = (uint8)((g_BMSMonitor_SOH.SOH+0.005)*100);     //SOH           1%�ֱ���
      	BMS_to_Upmonitor.m_data[4] = 0xFF;                                     //���γ����
      	BMS_to_Upmonitor.m_data[5] = 0xFF;  
      	BMS_to_Upmonitor.m_data[6] = 0xFF;//(uint8)(g_EnergyInfo.Energy_Once_DisCharge*10);    //���ηŵ���
      	BMS_to_Upmonitor.m_data[7] = 0xFF;//((uint16)(g_EnergyInfo.Energy_Once_DisCharge*10))>>8;        
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
      
      case 1:    //�����¶ȼ�����/�����ǹ�¶�
        BMS_to_Upmonitor.m_data[0] = i;      //��� 0x01
        BMS_to_Upmonitor.m_data[1] = (uint8)(g_EnergyInfo.Energy_Total_Charge*10);            //�ۼƳ�����   0.1kWh�ֱ���
        BMS_to_Upmonitor.m_data[2] = ((uint16)(g_EnergyInfo.Energy_Total_Charge*10)) >> 8; 
        BMS_to_Upmonitor.m_data[3] = ((uint32)(g_EnergyInfo.Energy_Total_Charge*10)) >> 16;             
        BMS_to_Upmonitor.m_data[4] = (uint8)(g_EnergyInfo.Energy_Total_DisCharge*10);           //�ۼƷŵ����   0.1kWh�ֱ���
        BMS_to_Upmonitor.m_data[5] = ((uint16)(g_EnergyInfo.Energy_Total_DisCharge*10)) >> 8;              
        BMS_to_Upmonitor.m_data[6] = ((uint32)(g_EnergyInfo.Energy_Total_DisCharge*10)) >> 16;         
        BMS_to_Upmonitor.m_data[7] = 0xFF;
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
    }
  }
  
  //0x1880C0F4    ��ع�����Ϣ
  for(i=0;i<2;i++)
  {
    Positive = Port_StateGet(Relay_Positive_PORT,Relay_Positive_pin);
    BMS_to_Upmonitor.m_ID = 0x1880C0F4;
    switch(i)
    {
      case 0:     
      	BMS_to_Upmonitor.m_data[0] = i;     //���  0x00
      	#if(ENABLE_RELAYADHESION_JUDGE == 1)    //�̵����жϹ���ʹ��
      	  BMS_to_Upmonitor.m_data[1] = (Positive & 0x03) + ((1 << 2) & 0x0C) + ((1 << 4) & 0x30) + ((1 << 6) & 0xC0);      //�̵���״̬ 0 �򿪣�1�ر�
      	  BMS_to_Upmonitor.m_data[2] = 0xFF; //Ԥ���ֽ�             
        	BMS_to_Upmonitor.m_data[3] = (g_Flt_Charge.Level_Volt_Cell_High & 0x03) + ((g_Flt_DisChg.Level_Volt_Cell_Low  << 2) & 0x0C) + (((g_Flt_DisChg.Level_Volt_Cell_Diff_High|g_Flt_Charge.Level_Volt_Cell_Diff_High) << 4) & 0x30) + ((g_Flt_Charge.Level_Volt_Sys_High  << 6) & 0xC0);     //�����ع�ѹ/Ƿѹ/ѹ�������ѹ����
        	BMS_to_Upmonitor.m_data[4] = (g_Flt_DisChg.Level_Volt_Sys_Low  & 0x03) + (((g_Flt_DisChg.Level_Insul|g_Flt_Charge.Level_Insul) << 2) & 0x0C) + ((g_Flt_DisChg.Level_Temp_High << 4) & 0x30) + ((g_Flt_DisChg.Level_Temp_Low << 6) & 0xC0);     //��ѹ���ͣ���Ե���ϣ��ŵ��¶ȹ���/����
        	BMS_to_Upmonitor.m_data[5] = (g_Flt_DisChg.Level_Temp_Diff_High & 0x03) + ((g_Flt_Charge.Level_Temp_High<< 2) & 0x0C) + ((g_Flt_Charge.Level_Temp_Low << 4) & 0x30) + ((g_Flt_Charge.Level_Temp_Diff_High << 6) & 0xC0);   //�ŵ��²���󣬳���¶ȹ���/���ͣ�����²����
        	BMS_to_Upmonitor.m_data[6] = (0x00 & 0x03) + ((0x00 << 2) & 0x0C) + ((g_Flt_Charge.Level_Current_Charge_High << 4) & 0x30) + ((g_Flt_DisChg.Level_Current_DisCharge_High << 6) & 0xC0);   //SOC�ߣ�SOC�ͣ����������󣬷ŵ��������                                     
        	BMS_to_Upmonitor.m_data[7] = (0x00 & 0x03) + ((0x00 << 2) & 0x0C) + ((0x00 << 4) & 0x30) + (0b11 << 6) ;  //���ǹ�¶ȹ��ߣ�δд��������Ӵ����¶ȹ��ߣ�δд������ѹ��������       
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
          UpMonitor_DelayTimeus(20);
        #else                         //�̵����жϹ���δʹ��
          BMS_to_Upmonitor.m_data[1] = 0x00;
      	  BMS_to_Upmonitor.m_data[2] = 0xFF;              
        	BMS_to_Upmonitor.m_data[3] = 0x00;  //�����ع�ѹ/Ƿѹ/ѹ�������ѹ����
        	BMS_to_Upmonitor.m_data[4] = 0x00;  //��ѹ���ͣ���Ե���ϣ��ŵ��¶ȹ���/����
        	BMS_to_Upmonitor.m_data[5] = 0x00;  //�ŵ��²���󣬳���¶ȹ���/���ͣ�����²����
        	BMS_to_Upmonitor.m_data[6] = 0x00;  //SOC�ߣ�SOC�ͣ����������󣬷ŵ��������                                     
        	BMS_to_Upmonitor.m_data[7] = 0x00;  //���ǹ�¶ȹ��ߣ�����Ӵ����¶ȹ��ߣ���ѹ��������       
          while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
          UpMonitor_DelayTimeus(20);
        #endif
      	
      break;
      
      case 1:    
        BMS_to_Upmonitor.m_data[0] = i;      //��� 0x01
        BMS_to_Upmonitor.m_data[1] = 0x00;   //BMSоƬ�¶ȹ���
        BMS_to_Upmonitor.m_data[2] = State_Offline.RelayFlt_Positive&0x01;//(DiscFlt.HIVL_ECT0_Fault & 0x01) + ((DiscFlt.HIVL_ECT1_Fault << 1) & 0x02) + ((DiscFlt.HIVL_ECT2_Fault << 2) & 0x04) + ((DiscFlt.HIVL_ECT3_Fault << 3) & 0x08) + (0b11111 << 4) ;      //��������
        BMS_to_Upmonitor.m_data[3] = ((State_Offline.CSSU1) & 0x01) + ((State_Offline.VCU << 1) & 0x02) + ((State_Offline.HVU << 2) & 0x04) + ((State_Offline.Charge<<3)&0x08);     //ͨ�Ź���  0000 ������0001 CSSU���ߣ�0010 VUC���ߣ�0100 HVU���ߣ�1000 TBOX����(��ʱû����Ϊ0x01)          
        BMS_to_Upmonitor.m_data[4] = g_PassiveBalance.BalanceOn;           
        BMS_to_Upmonitor.m_data[5] = g_PassiveBalance.BalanceNode;              
        BMS_to_Upmonitor.m_data[6] = 0x00;         
        BMS_to_Upmonitor.m_data[7] = 0x00;
        while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
        UpMonitor_DelayTimeus(20);
      break;
    }
  }
  g_Roll_Tick.Roll_BMSUp++;
}


/*=======================================================================
 *������:      Task_BMUToUpMonitor(void) 
 *����:        BMS collect information to UpMonitor
 *����:        ��       
 *���أ�       ��
 *˵����       BMS���Ͳɼ��ĵ�ѹ��Ϣ��Ϣ����λ����
========================================================================*/ 
void Task_BMUToUpMonitor(void)
{
  uint8  i,j;
  CANFRAME BMS_to_Upmonitor;
  uint8 batt,batt1; 
  
  batt=(NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true+NUM4_Batper_true+NUM5_Batper_true)/3;
  batt1=(NUM1_Batper_true+NUM2_Batper_true+NUM3_Batper_true+NUM4_Batper_true+NUM5_Batper_true)%3;
  
  BMS_to_Upmonitor.m_IDE = 1;
	BMS_to_Upmonitor.m_RTR = 0;
	BMS_to_Upmonitor.m_dataLen = 8;
	BMS_to_Upmonitor.m_priority = 6; 
    
  BMS_to_Upmonitor.m_ID = BMS_Send_Information1;//0x18FF9700       
  for(i = 0; i <batt ; i++) 
  {
    BMS_to_Upmonitor.m_data[0] = g_Flt_Charge.Level_Charge_BalanceON_Flag;  //BMS�����Ƿ������Ӱ���о���    
    BMS_to_Upmonitor.m_data[1] = (uint8)(i);             
    BMS_to_Upmonitor.m_data[2] = (uint8)g_LTC6811_VoltInfo.CellVolt[i*3];
    BMS_to_Upmonitor.m_data[3] = (g_LTC6811_VoltInfo.CellVolt[i*3]>>8)&0xFF;
    BMS_to_Upmonitor.m_data[4] = (uint8)g_LTC6811_VoltInfo.CellVolt[i*3+1];
    BMS_to_Upmonitor.m_data[5] = (g_LTC6811_VoltInfo.CellVolt[i*3+1]>>8)&0xFF;
    BMS_to_Upmonitor.m_data[6] = (uint8)g_LTC6811_VoltInfo.CellVolt[i*3+2];
    BMS_to_Upmonitor.m_data[7] = (g_LTC6811_VoltInfo.CellVolt[i*3+2]>>8)&0xFF;
    while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
    UpMonitor_DelayTimeus(20);
  }  
  switch(batt1) 
  {
    case 1:
    BMS_to_Upmonitor.m_data[0] = g_Flt_Charge.Level_Charge_BalanceON_Flag;  //BMS�����Ƿ������Ӱ���о���  
    BMS_to_Upmonitor.m_data[1] = (uint8)(i);                                //ÿ��6804�ɼ���ѹ�ı�� 
    BMS_to_Upmonitor.m_data[2] = (uint8)g_LTC6811_VoltInfo.CellVolt[i*3];
    BMS_to_Upmonitor.m_data[3] = (g_LTC6811_VoltInfo.CellVolt[i*3]>>8)&0xFF;
    BMS_to_Upmonitor.m_data[4] = 0xFF;
    BMS_to_Upmonitor.m_data[5] = 0xFF;
    BMS_to_Upmonitor.m_data[6] = 0xFF;
    BMS_to_Upmonitor.m_data[7] = 0xFF;
    while(CAN_ToUpMonitor(&BMS_to_Upmonitor)); 
    UpMonitor_DelayTimeus(20);
    break;
    
    case 2:
    BMS_to_Upmonitor.m_data[0] = g_Flt_Charge.Level_Charge_BalanceON_Flag;  //BMS�����Ƿ������Ӱ���о���  
    BMS_to_Upmonitor.m_data[1] = (uint8)(i);                                //ÿ��6804�ɼ���ѹ�ı�� 
    BMS_to_Upmonitor.m_data[2] = (uint8)g_LTC6811_VoltInfo.CellVolt[i*3];
    BMS_to_Upmonitor.m_data[3] = (g_LTC6811_VoltInfo.CellVolt[i*3]>>8)&0xFF;
    BMS_to_Upmonitor.m_data[4] = (uint8)g_LTC6811_VoltInfo.CellVolt[i*3+1];
    BMS_to_Upmonitor.m_data[5] = (g_LTC6811_VoltInfo.CellVolt[i*3+1]>>8)&0xFF;
    BMS_to_Upmonitor.m_data[6] = 0xFF;
    BMS_to_Upmonitor.m_data[7] = 0xFF;
    while(CAN_ToUpMonitor(&BMS_to_Upmonitor)); 
    UpMonitor_DelayTimeus(20);
    break;
    
  default:
    break; 
  }   
                    
  BMS_to_Upmonitor.m_ID = BMS_Send_Information2;//0x18FF9710       
	BMS_to_Upmonitor.m_data[0] = (uint8)g_LTC6811_VoltInfo.CellVolt_Max;
  BMS_to_Upmonitor.m_data[1] = (g_LTC6811_VoltInfo.CellVolt_Max>>8)&0x00FF;
  BMS_to_Upmonitor.m_data[2] = g_LTC6811_VoltInfo.CellVolt_MaxNode;
  BMS_to_Upmonitor.m_data[3] = (uint8)g_LTC6811_VoltInfo.CellVolt_Min;
  BMS_to_Upmonitor.m_data[4] = (g_LTC6811_VoltInfo.CellVolt_Min>>8)&0x00FF;
  BMS_to_Upmonitor.m_data[5] = g_LTC6811_VoltInfo.CellVolt_MinNode;
  BMS_to_Upmonitor.m_data[6] = g_PassiveBalance.BalanceOn;//����ĵ�ر��  
  BMS_to_Upmonitor.m_data[7] = g_PassiveBalance.BalanceNode;//����ĵ�ر��        
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
  UpMonitor_DelayTimeus(20);
 
  BMS_to_Upmonitor.m_ID = BMS_Send_Information3;//18FF9800      
	for(i=0; i< ((NUM_Tem+6)/7) ;i++)         
  {
    BMS_to_Upmonitor.m_data[0] = i;
    for(j=1; j < ((NUM_Tem+1)%7); j++) 
    {
      BMS_to_Upmonitor.m_data[j] = g_LTC6811_TempInfo.CellTemp[j-1+i*7];
    } 
    for(j=((NUM_Tem+1)%7); j<8; j++)
    {
      BMS_to_Upmonitor.m_data[j] = 0xFF;
    } 
    while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
    UpMonitor_DelayTimeus(20); 
  }
  j=NUM_Tem%7;
  if((j!=0)&&(NUM_Tem>7))
  {
    BMS_to_Upmonitor.m_data[0] = i;
    for(i=1; i<j+1; i++)
    {
       BMS_to_Upmonitor.m_data[i] = g_LTC6811_TempInfo.CellTemp[i-1+BMS_to_Upmonitor.m_data[0]*7];
    }
    for(i=j+1; j<8; j++)
    {
       BMS_to_Upmonitor.m_data[i] = 0xFF;
    }
    while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
    UpMonitor_DelayTimeus(20); 
  }
  
  BMS_to_Upmonitor.m_ID = BMS_Send_Information4;//0x18FF9810
  BMS_to_Upmonitor.m_data[0] = g_LTC6811_TempInfo.CellTemp_Max;
	BMS_to_Upmonitor.m_data[1] = g_LTC6811_TempInfo.CellTemp_MaxNode;   
	BMS_to_Upmonitor.m_data[2] = g_LTC6811_TempInfo.CellTemp_Min;
	BMS_to_Upmonitor.m_data[3] = g_LTC6811_TempInfo.CellTemp_MinNode;   
	BMS_to_Upmonitor.m_data[4] = g_LTC6811_TempInfo.CellTemp_Tatoltemp;
	BMS_to_Upmonitor.m_data[5] = g_LTC6811_TempInfo.CellTemp_Tatoltemp>>8;   
	BMS_to_Upmonitor.m_data[6] = 0xFF;
	BMS_to_Upmonitor.m_data[7] = 0xFF;       	 
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor)); 
  UpMonitor_DelayTimeus(20);
  

  BMS_to_Upmonitor.m_ID = BMS_Send_Information5;//0x18FF9900
  BMS_to_Upmonitor.m_data[0] = g_LTC6811_TempInfo.ICTemp_OverState;
	BMS_to_Upmonitor.m_data[1] = g_LTC6811_OpwireInfo.OpenwireErr;	
	BMS_to_Upmonitor.m_data[2] = g_LTC6811_VoltInfo.CellVolt_Total;   
	BMS_to_Upmonitor.m_data[3] = g_LTC6811_VoltInfo.CellVolt_Total>>8;
	BMS_to_Upmonitor.m_data[4] = g_LTC6811_VoltInfo.CellVolt_Total>>16;
	BMS_to_Upmonitor.m_data[5] = (uint8)g_VoltInfo.SysVolt_Total;
	BMS_to_Upmonitor.m_data[6] = (uint8)(g_VoltInfo.SysVolt_Total>>8);
	BMS_to_Upmonitor.m_data[7] = (uint8)(g_VoltInfo.SysVolt_Total>>16);       
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor));
  UpMonitor_DelayTimeus(20); 
   
  BMS_to_Upmonitor.m_ID = BMS_Send_Information6;//0x19FF9900   
  BMS_to_Upmonitor.m_dataLen = 6;          
	for(i = 0; i < NUM_IC ; i++)
	{
    BMS_to_Upmonitor.m_data[i*2] = g_LTC6811_OpwireInfo.OpenwireLocation[i];                   // ���߿�·
  	BMS_to_Upmonitor.m_data[i*2 + 1] = (uint8)((g_LTC6811_OpwireInfo.OpenwireLocation[i]>>8)&0x00FF);	       
	}
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor)); 
  UpMonitor_DelayTimeus(20);
  
  BMS_to_Upmonitor.m_ID = BMS_Send_Information7;//0x18FF9600
  BMS_to_Upmonitor.m_dataLen = 8;
	BMS_to_Upmonitor.m_data[0] = NUM1_Batper_true;
  BMS_to_Upmonitor.m_data[1] = NUM2_Batper_true;
  BMS_to_Upmonitor.m_data[2] = NUM3_Batper_true;
  BMS_to_Upmonitor.m_data[3] = 0xFF;
  BMS_to_Upmonitor.m_data[4] = 0xFF;
  BMS_to_Upmonitor.m_data[5] = 0xFF;
  BMS_to_Upmonitor.m_data[6] = 0xFF;  
  BMS_to_Upmonitor.m_data[7] = 0xFF;   
  while(CAN_ToUpMonitor(&BMS_to_Upmonitor)); 
  
  g_Roll_Tick.Roll_BMUUp++;
}
/*=======================================================================
 *������:      Task_BMSToUpMonitor(void) 
 *����:        BMS���͸���λ����Ϣ
 *����:        ��       
 *���أ�       ��
 *˵����       ����λ���״η���ID��0x19FFF4C0��data[0]==0xAAʱͬʱ������ֵ��Ϣ�͵����Ϣ
               ����λ������ID��0x19FFF4C0��data[1]==0xAAʱ������ֵ��Ϣ
               ����:data[0]=0xAA����λ�������ȡ��Ϣ��500ms�������·�
               data[1]=0xAA����λ�������ȡ��Ϣ��ֻ����һ��
========================================================================*/
void Task_BMSToUpMonitor(void)
{
  if(g_MonitorBMS_Start.Msg_StarUpload == 1)
  {
    CAN_ToUpMonitorMsg();
    g_MonitorBMS_Start.Msg_StarUpload = 0;
  }
}