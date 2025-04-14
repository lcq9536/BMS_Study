/*=======================================================================
 *Subsystem:   ���
 *File:        DataFromCSSU.c
 *Author:      WenYuhao
 *Description: ͨ�ţ�
               �ӿڣ�CAN2
               �����ʣ�500kb
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
  #include  "includes.h"
  
  FromCSSU_Volt_T g_FromCSSU_Volt;
  FromCSSU_Temp_T g_FromCSSU_Temp;
  FromCSSU_FltData_T g_FromCSSU_FltData;
/*=======================================================================
 *������:      DataFromCSSU(pCANFRAME data)  
 *����:        ������������
 *����:        ID������ID��       
 *���أ�       ��
 *˵����       ���հ����Ӱ壬��Ե��������
========================================================================*/
void DataFromCSSU(pCANFRAME data)
{
  uint8 i=0,j=0;
  uint16 Total_Temp;
  switch(data->m_ID) 
  { 
    //���ģ�鵥���ѹ��Ϣ
    case 0x18FF9701:      
      g_FromCSSU_FltData.CSSU_BalanceState = data->m_data[0];//�����Ӱ�����״̬
      
      j = data->m_data[1];  //6804�ɼ���ѹ���,��0һ����������

      for(i=0; i<3&&(3*j+i<(SYS_SERIES_YiDongLi)); i++)   //&&(3*j+i<(SYS_SERIES_YiDongLi))
      {
         g_FromCSSU_Volt.CellVolt[j*3+i] = data->m_data[2+i*2] + (((uint16)data->m_data[3+i*2])<<8);
      }
    break;
    
    //���ģ�������С��ѹ��Ϣ  
    case 0x18FF9711:
      g_FromCSSU_Volt.CellVolt_Max     = data->m_data[0]+(((uint16)data->m_data[1])<<8);
      g_FromCSSU_Volt.CellVolt_MaxNode = data->m_data[2];
      g_FromCSSU_Volt.CellVolt_Min     = data->m_data[3]+(((uint16)data->m_data[4])<<8);
      g_FromCSSU_Volt.CellVolt_MinNode = data->m_data[5];
    break;
    
    //ÿ��ģ�鵥���¶���Ϣ
    case 0x18FF9801:
      for(i=0; i<SYS_NUMBER_MODULE_TEMP; i++) 
      {
         g_FromCSSU_Temp.CellTemp[i] = data->m_data[i+1];
      }
    break; 
    
    //ÿ��ģ�������С�¶���Ϣ  
    case 0x18FF9811:
      g_FromCSSU_Temp.CellTemp_Max     = data->m_data[0];
      g_FromCSSU_Temp.CellTemp_MaxNode = data->m_data[1];
      g_FromCSSU_Temp.CellTemp_Min     = data->m_data[2];
      g_FromCSSU_Temp.CellTemp_MinNode = data->m_data[3];
      
      Total_Temp   = (data->m_data[4] + (((uint16)data->m_data[5])<<8));
      g_FromCSSU_Temp.CellTemp_Ave     = Total_Temp /SYS_NUMBER_MODULE_TEMP;
    break;  
    
    //ÿ��ģ����ѹ��Ϣ
    case 0x18FF9901:
      g_FromCSSU_FltData.CSSUFlt_ChipTemp = data -> m_data[0];
      g_FromCSSU_FltData.OpenWire_Status  = data -> m_data[1];
      g_FromCSSU_Volt.CSSUVolt_Total      = (data -> m_data[2]+(((uint16)data -> m_data[3])<<8)+(((uint32)data -> m_data[4])<<16));
      g_FromCSSU_Volt.InsulVolt_Total     = (data -> m_data[5]+(((uint16)data -> m_data[6])<<8)+(((uint32)data -> m_data[7])<<16));
    break;
    
    default:
    break;
 
  } 
} 
  
  
  
  
  