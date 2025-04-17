/*=======================================================================
 *Subsystem:   ���
 *File:        UpMonitorToBMS.C
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

MonitorBMS_SOC_T    g_MonitorBMS_SOC;
MonitorBMS_Start_T  g_MonitorBMS_Start;

/*=======================================================================
 *������:      UpMonitor_to_Bms(pCANFRAME data)
 *����:        ��0CA0��ʼ�洢��
 *����:        ��       
 *���أ�       ��
 *˵����       ��λ��������Ϣ��BMS�����б궨����,��һ��switch case�ķ�ʽ
               ѡ��궨�����ݡ�
========================================================================*/
void UpMonitor_to_Bms(pCANFRAME data)
{
  uint8   Val8,state[2];
  CANFRAME Monitor_Biao;
  uint16  Val;
  uint32  Val32;
  uint8  Send_ID;
     
  switch(data -> m_ID) 
  { 
      //0x1910F4C0   ��ص�ѹ������ֵ
      case 0x1910F4C0:
        Send_ID = 0x10;
        switch(data -> m_data[0])
        {
          //�����ѹ
          case 00 :
            Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CA0=(uint16)(Val);  
            }
            Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);  
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CA2=(uint16)(Val);  
            }

            g_BMSMonitor_Volt.Volt_Cell_High1 =(*(uint16*)(0x0CA0));
            g_BMSMonitor_Volt.Volt_Cell_High2 =(*(uint16*)(0x0CA2));
            //Monitor_BMS_Volt.Single_Over3_Volt =(*(uint16*)(0x0CA4));        
          break;
          //����Ƿѹ   
          case 01:
            Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CA8=(uint16)(Val);  
            }
            Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);  
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CAA=(uint16)(Val);  
            }

            g_BMSMonitor_Volt.Volt_Cell_Low1 =(*(uint16*)(0x0CA8));
            g_BMSMonitor_Volt.Volt_Cell_Low2 =(*(uint16*)(0x0CAA));
            //Monitor_BMS_Volt.Single_Under3_Volt =(*(uint16*)(0x0CAC));                               
          break;
          //ϵͳ��ѹ  
          case 02:
            Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CB0=(uint16)(Val);  
            }
            Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);   
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CB2=(uint16)(Val);  
            }

            g_BMSMonitor_Volt.Volt_Sys_High1 =(*(uint16*)(0x0CB0));
            g_BMSMonitor_Volt.Volt_Sys_High2 =(*(uint16*)(0x0CB2));
            //Monitor_BMS_Volt.Pack_Over3_Volt =(*(uint16*)(0x0CB4));
          break;
          //ϵͳǷѹ
          case 03:
            Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CB8=(uint16)(Val);  
            }
            Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);  
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CBA=(uint16)(Val);  
            }

            g_BMSMonitor_Volt.Volt_Sys_Low1 =(*(uint16*)(0x0CB8));
            g_BMSMonitor_Volt.Volt_Sys_Low2 =(*(uint16*)(0x0CBA));
            //Monitor_BMS_Volt.Pack_Under3_Volt =(*(uint16*)(0x0CBC));
          break;
          //����ѹ��
          case 04:
            Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
            if(Val!=0xFFFF) 
            {
               *(uint16*)0x0CC0=(uint16)(Val);  
            }
            g_BMSMonitor_Volt.Volt_Cell_Diff1 =(*(uint16*)(0x0CC0));
          break;
        }
      break;
      
      // 0x1911F4C0    ����¶ȱ�����ֵ
      case 0x1911F4C0:
      Send_ID = 0x11;
      switch(data -> m_data[0])
      {
        //������¶�
        case 00 :
          Val8 = data -> m_data[1];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CD0=(uint8)(Val8);  
          }
          Val8 = data -> m_data[2];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CD1=(uint8)(Val8);  
          }

          Val8 = data -> m_data[4];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CD3=(uint8)(Val8);  
          }
          Val8 = data -> m_data[5];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CD4=(uint8)(Val8);  
          }
          g_BMSMonitor_Temp.Temp_Charge_High1 =(*(uint8*)(0x0CD0));
          g_BMSMonitor_Temp.Temp_Charge_High2 =(*(uint8*)(0x0CD1));
          //Monitor_BMS_Temp.Charge_Over3_Temp =(*(uint8*)(0x0CD2));
          g_BMSMonitor_Temp.Temp_Charge_Low1 =(*(uint8*)(0x0CD3));
          g_BMSMonitor_Temp.Temp_Charge_Low2 =(*(uint8*)(0x0CD4));
          //Monitor_BMS_Temp.Charge_Under3_Temp =(*(uint8*)(0x0CD5));        
        break;
        //�ŵ����¶�   
        case 01:
          Val8 = data -> m_data[1];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CD8=(uint8)(Val8);  
          }
          Val8 = data -> m_data[2];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CD9=(uint8)(Val8);  
          }

          Val8 = data -> m_data[4];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CDB=(uint8)(Val8);  
          }
          Val8 = data -> m_data[5];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CDC=(uint8)(Val8);  
          }
          g_BMSMonitor_Temp.Temp_DisCharge_High1 =(*(uint8*)(0x0CD8));
          g_BMSMonitor_Temp.Temp_DisCharge_High2 =(*(uint8*)(0x0CD9));
          //Monitor_BMS_Temp.DisCharge_Over3_Temp =(*(uint8*)(0x0CDA));
          g_BMSMonitor_Temp.Temp_DisCharge_Low1 =(*(uint8*)(0x0CDB));
          g_BMSMonitor_Temp.Temp_DisCharge_Low2 =(*(uint8*)(0x0CDC));
          //Monitor_BMS_Temp.DisCharge_Under3_Temp =(*(uint8*)(0x0CDD));                               
        break;
        //��/�ŵ����²�  
        case 02:
          Val8 = data -> m_data[1];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CE0=(uint8)(Val8);  
          }
          Val8 = data -> m_data[4];   
          if(Val8!=0xFF) 
          {
             *(uint8*)0x0CE3=(uint8)(Val8);  
          }

          g_BMSMonitor_Temp.Temp_Charge_Diff1 =(*(uint8*)(0x0CE0));
          //Monitor_BMS_Temp.Charge_Diff2_Temp =(*(uint8*)(0x0CE1));
          //Monitor_BMS_Temp.Charge_Diff3_Temp =(*(uint8*)(0x0CE2));
          g_BMSMonitor_Temp.Temp_DisCharge_Diff1 =(*(uint8*)(0x0CE3));
          //Monitor_BMS_Temp.DisCharge_Diff2_Temp =(*(uint8*)(0x0CE4));
          //Monitor_BMS_Temp.DisCharge_Diff3_Temp =(*(uint8*)(0x0CE5));
        break;
      }
      break;
      
      // 0x1912F4C0      ��ص�����ֵ
      case 0x1912F4C0:
      Send_ID = 0x12;
      switch(data -> m_data[0])
      {
        //ϵͳ�ŵ����
        case 00 :
          Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);  
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0CF0=(uint16)(Val);  
          }
          Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);    
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0CF2=(uint16)(Val);  
          }
          g_BMSMonitor_Curr.Current_DisCharge_High1 =(*(uint16*)(0x0CF0));
          g_BMSMonitor_Curr.Current_DisCharge_High2 =(*(uint16*)(0x0CF2));
        break;
        //ϵͳ������  
        case 01:
          Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]); 
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0CF8=(uint16)(Val);  
          }
          Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);   
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0CFA=(uint16)(Val);  
          }
          g_BMSMonitor_Curr.Current_Charge_High1 =(*(uint16*)(0x0CF8));
          g_BMSMonitor_Curr.Current_Charge_High2 =(*(uint16*)(0x0CFA));
        break;
      }
      break;
      
      // 0x1913F4C0      ��Ե������ֵ
      case 0x1913F4C0:
      Send_ID = 0x13;
      Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);  
      if(Val!=0xFFFF) 
      {
         *(uint16*)0x0D0A=(uint16)(Val);  
      }
      g_BMSMonitor_Insul.Insulation_Resis2 =(*(uint16*)(0x0D0A));
      break;
      
      // 0x1915F4C0      ���������Ϣ
      case 0x1915F4C0:
      Send_ID = 0x15;
      switch(data -> m_data[0])
      {
      
        case 00 :
          Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0D10=(uint16)(Val);  
          }
          Val = (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);  
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0D12=(uint16)(Val);  
          }
          Val = (((uint16)(data -> m_data[6]))<<8) + (data -> m_data[5]); 
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0D14=(uint16)(Val);  
          }
          g_MonitorBMS_SOC.SOC_Init =(*(uint16*)(0x0D10));
          g_SOCInfo.SOC_Init =  g_MonitorBMS_SOC.SOC_Init;
          g_MonitorBMS_SOC.Ref_Volt =(*(uint16*)(0x0D12));
          g_MonitorBMS_SOC.Ref_Current =(*(uint16*)(0x0D14));        
        break;
       
        case 01:
          Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);   
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0D18=(uint16)(Val);  
          }
          Val32 = (((uint32)(data -> m_data[6]))<<24) + (((uint32)(data -> m_data[5]))<<16) + (((uint16)(data -> m_data[4]))<<8) + (data -> m_data[3]);  
          if(Val32!=0xFFFF) 
          {
             *(uint32*)0x0D1A=(uint32)(Val32);  
          }
          g_MonitorBMS_SOC.Volt_Resis =(*(uint16*)(0x0D18));
          g_MonitorBMS_SOC.Pack_Total_Capacity =(*(uint16*)(0x0D1A));
        break;
        
        case 02:
          Val = (((uint16)(data -> m_data[2]))<<8) + (data -> m_data[1]);  
          if(Val!=0xFFFF) 
          {
             *(uint16*)0x0D20=(uint16)(Val);  
          }
          g_MonitorBMS_SOC.BMS_Running_Time =(*(uint16*)(0x0D20)) * 60;
          Val8 = data -> m_data[3];                       //�±������̵���
          if(Val8!=0xFF)
          {
            if(Val8 == 1)
            {
              PositiveRelay_Control(Relay_ON); 
            }
            else if(Val8 == 0)
            {
              PositiveRelay_Control(Relay_OFF);
            }
          }
          g_SysTime.BMS_TotalRun_MiniteTime = (*(uint16*)0x0D20);
        break;
      }
      break;
     
      //0x1950F4C0
      case 0x1950F4C0:
        Send_ID = 0x50;
        Val8 = data -> m_data[0]; 
        if(Val8!=0xFF)
        {
          g_FltCodeSave.ReadFlag = Val8;  
        }
        Val8 = data -> m_data[1]; 
        if(Val8!=0xFF)
        {
          g_FltCodeSave.ClearFlag = Val8;  
        }
      break;
      
      // 0x19FFF4C0      ��������֡
      case 0x19FFF4C0:
        Send_ID = 0xFF;
        state[0] =  data -> m_data[0];
        state[1] =  data -> m_data[1];
        if(state[0] == 0xAA)
        {
          g_MonitorBMS_Start.Msg_StarUpload = 1;  
        }
        if(state[1] == 0xAA)
        {
          g_MonitorBMS_Start.Threshold_StarUpload = 1;
        }
      break;
  }
  
  if(Send_ID == 0xFE)  //�汾������֮��ŷ���
  {
      Monitor_Biao.m_ID = 0x18FEC0F4;       // �汾�ţ�
    	Monitor_Biao.m_IDE = 1;
    	Monitor_Biao.m_RTR = 0;
    	Monitor_Biao.m_dataLen =  8;
    	Monitor_Biao.m_priority = 6;
    	Monitor_Biao.m_data[0] = '2';
    	Monitor_Biao.m_data[1] = '0';
    	Monitor_Biao.m_data[2] = '1';
    	Monitor_Biao.m_data[3] = '8';
    	Monitor_Biao.m_data[4] = '0';
    	Monitor_Biao.m_data[5] = '1';
    	Monitor_Biao.m_data[6] = '0';
    	Monitor_Biao.m_data[7] = '1';
      while(CAN_ToUpMonitor(&Monitor_Biao));         
  } 
  //�궨�ظ�֡
  else if(Send_ID != 0)   //�ɹ�
  {
      Monitor_Biao.m_ID = 0x18FFC0F4;      
      Monitor_Biao.m_IDE = 1;
      Monitor_Biao.m_RTR = 0;
      Monitor_Biao.m_dataLen = 8;
      Monitor_Biao.m_priority = 6;
      Monitor_Biao.m_data[0] = Send_ID;
      Monitor_Biao.m_data[1] = 0xFF;
    	Monitor_Biao.m_data[2] = 0xFF;
    	Monitor_Biao.m_data[3] = 0xFF;
    	Monitor_Biao.m_data[4] = 0xFF;
    	Monitor_Biao.m_data[5] = 0xFF;
    	Monitor_Biao.m_data[6] = 0xFF;
    	Monitor_Biao.m_data[7] = 0xFF;
      while(CAN_ToUpMonitor(&Monitor_Biao));
  }
  else    //ʧ��
  {
      Monitor_Biao.m_ID = 0x18FFC0F4;      
      Monitor_Biao.m_IDE = 1;
      Monitor_Biao.m_RTR = 0;
      Monitor_Biao.m_dataLen = 8;
      Monitor_Biao.m_priority = 6;
      Monitor_Biao.m_data[0] = 0xEE;
      Monitor_Biao.m_data[1] = 0xFF;
    	Monitor_Biao.m_data[2] = 0xFF;
    	Monitor_Biao.m_data[3] = 0xFF;
    	Monitor_Biao.m_data[4] = 0xFF;
    	Monitor_Biao.m_data[5] = 0xFF;
    	Monitor_Biao.m_data[6] = 0xFF;
    	Monitor_Biao.m_data[7] = 0xFF;
      while(CAN_ToUpMonitor(&Monitor_Biao));
  } 
     
}         
